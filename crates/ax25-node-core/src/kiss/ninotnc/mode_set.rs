//! Set the NinoTNC's operating mode over KISS SETHW and prove it took.
//!
//! Ports the verification loop of `NinoTncSerialPort.SetModeAsync` +
//! `NinoTncModeVerification` (packet.net #633). SETHW is fire-and-forget: the
//! firmware never acknowledges it, and on firmware 3.44 with DIP 1111 it has been
//! bench-observed to silently leave the TNC in the previous mode. Everything
//! downstream then scores zero, which reads like broken RF rather than an ignored
//! mode change. So after each SETHW we settle, ask for GETALL, compare the running
//! mode, and retry.
//!
//! This is a sans-I/O state machine: the caller owns the UART and the clock, feeds
//! in the current time (milliseconds, any monotonic origin) and every status
//! report heard from the TNC, and performs the [`Step`]s it returns. That keeps it
//! host-testable on virtual time and lets the firmware drive it from its existing
//! read pump.
//!
//! ## Deliberate divergences from packet.net
//!
//! When a readback shows the MODE DIP switches are **not** `1111`, this gives up
//! at once with [`ModeSetOutcome::DipNotSoftware`] instead of spending the
//! remaining retries. SETHW can only take effect with all four switches on, so
//! further attempts cannot succeed, and naming the switch position is the most
//! useful thing a setup screen can tell the operator. (If the DIPs happen to
//! select exactly the requested mode, the running-mode check matches first and the
//! outcome is [`ModeSetOutcome::Applied`].)
//!
//! Likewise, a readback from firmware older than pico-node supports (3/4.44;
//! [`super::firmware::MIN_SUPPORTED_MINOR`]) ends it with
//! [`ModeSetOutcome::FirmwareTooOld`], and [`refuse_before_sending`] lets a
//! caller that already knows the firmware skip the SETHW altogether. First seen
//! on a bench NinoTNC running 3.39, which predates SETHW mode selection (41).

use core::fmt;

use super::catalog::{self, NinoTncMode};
use super::firmware::{FirmwareVersion, MIN_SUPPORTED_MINOR};
use super::sethw;
use super::status::NinoTncStatusFrame;

/// The "Set from KISS" DIP position. SETHW to this mode cannot be read back
/// meaningfully (the running mode is whatever the TNC last stored), so it is sent
/// unverified, as packet.net does.
pub const SET_FROM_KISS_MODE: u8 = 15;

/// How hard to try. Defaults mirror `NinoTncModeVerification.Default`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ModeVerifyPolicy {
    /// Wait after each SETHW before asking for GETALL. packet.net's bench rig
    /// settled on 1.5 s; the frames straight after a mode change are unreliable.
    pub settle_ms: u64,
    /// SETHW + readback attempts before giving up (values below 1 act as 1).
    pub attempts: u8,
    /// How long to wait for a status report after sending GETALL.
    pub readback_timeout_ms: u64,
}

impl Default for ModeVerifyPolicy {
    fn default() -> Self {
        Self {
            settle_ms: 1_500,
            attempts: 3,
            readback_timeout_ms: 5_000,
        }
    }
}

/// An action for the caller to perform on the serial link.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Step {
    /// Send KISS SETHW (`0x06`) with this payload byte (mode, `+16` if RAM-only).
    SendSetHardware {
        /// The SETHW payload byte.
        payload: u8,
    },
    /// Send a GETALL query ([`super::commands::build_get_all_into`]).
    SendGetAll,
}

/// How a mode change ended.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModeSetOutcome {
    /// The TNC reported running the requested mode.
    Applied {
        /// The mode now running.
        mode: NinoTncMode,
    },
    /// Sent without verification (mode 15, which has no readback to compare).
    SentUnverified {
        /// The requested mode.
        mode: u8,
    },
    /// The TNC's MODE DIP switches are not all on, so it ignores SETHW.
    DipNotSoftware {
        /// The requested mode.
        requested: u8,
        /// The DIP position the TNC reported (low four bits).
        dip: u8,
    },
    /// The TNC's firmware is older than pico-node supports (3/4.44).
    FirmwareTooOld {
        /// The requested mode.
        requested: u8,
        /// The firmware the TNC reported.
        version: FirmwareVersion,
    },
    /// Every attempt was made and the TNC never reported the requested mode.
    NotApplied {
        /// The requested mode.
        requested: u8,
        /// Attempts made.
        attempts: u8,
        /// What the TNC last said it was running, if any readback landed.
        last_running: Option<NinoTncMode>,
        /// The raw firmware mode byte from the last readback, when the catalog
        /// did not recognise it (newer firmware than our table).
        last_unknown_byte: Option<u8>,
        /// Whether any readback landed at all.
        heard_any: bool,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    Settling { until_ms: u64 },
    AwaitingReadback { until_ms: u64 },
    Done(ModeSetOutcome),
}

/// One in-flight mode change. Create with [`ModeSetter::start`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ModeSetter {
    mode: u8,
    payload: u8,
    persist_to_flash: bool,
    policy: ModeVerifyPolicy,
    attempt: u8,
    phase: Phase,
    last_running: Option<NinoTncMode>,
    last_unknown_byte: Option<u8>,
    heard_any: bool,
}

impl ModeSetter {
    /// Begin a mode change. Returns the setter and the first step (always a
    /// SETHW), or `None` if `mode > 15`.
    pub fn start(
        mode: u8,
        persist_to_flash: bool,
        now_ms: u64,
        policy: ModeVerifyPolicy,
    ) -> Option<(Self, Step)> {
        let payload = sethw::build_payload_byte(mode, persist_to_flash)?;
        let phase = if mode == SET_FROM_KISS_MODE {
            Phase::Done(ModeSetOutcome::SentUnverified { mode })
        } else {
            Phase::Settling {
                until_ms: now_ms.saturating_add(policy.settle_ms),
            }
        };
        let setter = Self {
            mode,
            payload,
            persist_to_flash,
            policy,
            attempt: 1,
            phase,
            last_running: None,
            last_unknown_byte: None,
            heard_any: false,
        };
        Some((setter, Step::SendSetHardware { payload }))
    }

    /// A mode change that was decided without sending anything (see
    /// [`refuse_before_sending`]), so it can be shown like any other.
    pub fn decided(mode: u8, outcome: ModeSetOutcome) -> Self {
        Self {
            mode,
            payload: 0,
            persist_to_flash: false,
            policy: ModeVerifyPolicy::default(),
            attempt: 0,
            phase: Phase::Done(outcome),
            last_running: None,
            last_unknown_byte: None,
            heard_any: false,
        }
    }

    /// The requested mode.
    pub fn mode(&self) -> u8 {
        self.mode
    }

    /// Whether the change also writes the TNC's own flash.
    pub fn persist_to_flash(&self) -> bool {
        self.persist_to_flash
    }

    /// The attempt in progress (1-based).
    pub fn attempt(&self) -> u8 {
        self.attempt
    }

    /// The configured attempt budget (at least 1).
    pub fn attempts(&self) -> u8 {
        self.policy.attempts.max(1)
    }

    /// When [`Self::on_time`] next needs calling, or `None` once finished.
    pub fn deadline_ms(&self) -> Option<u64> {
        match self.phase {
            Phase::Settling { until_ms } | Phase::AwaitingReadback { until_ms } => Some(until_ms),
            Phase::Done(_) => None,
        }
    }

    /// The outcome, once finished.
    pub fn outcome(&self) -> Option<ModeSetOutcome> {
        match self.phase {
            Phase::Done(o) => Some(o),
            _ => None,
        }
    }

    /// Advance on the clock. Call at (or after) [`Self::deadline_ms`].
    pub fn on_time(&mut self, now_ms: u64) -> Option<Step> {
        match self.phase {
            Phase::Settling { until_ms } if now_ms >= until_ms => {
                self.phase = Phase::AwaitingReadback {
                    until_ms: now_ms.saturating_add(self.policy.readback_timeout_ms),
                };
                Some(Step::SendGetAll)
            }
            // A readback that never lands says nothing about the mode: count the
            // attempt as failed and re-send.
            Phase::AwaitingReadback { until_ms } if now_ms >= until_ms => self.next_attempt(now_ms),
            _ => None,
        }
    }

    /// Feed a status report heard from the TNC (a GETALL reply, the periodic
    /// status beacon, or a TX-Test diagnostic mapped through
    /// [`NinoTncStatusFrame::from_diagnostic`]). Reports heard while settling are
    /// ignored: they may predate the mode change.
    pub fn on_status(&mut self, status: &NinoTncStatusFrame, now_ms: u64) -> Option<Step> {
        if !matches!(self.phase, Phase::AwaitingReadback { .. }) {
            return None;
        }
        self.heard_any = true;
        self.last_running = status.running_mode;
        self.last_unknown_byte = match status.running_mode {
            Some(_) => None,
            None => status.firmware_mode_byte,
        };

        // Checked first: on unsupported firmware the mode-byte table cannot be
        // trusted either, so a "match" would prove nothing.
        if let Some(refused) = refuse_before_sending(self.mode, status.firmware_version) {
            self.phase = Phase::Done(refused);
            return None;
        }
        if status.running_mode.map(|m| m.mode) == Some(self.mode) {
            if let Some(mode) = catalog::try_get_by_mode(self.mode) {
                self.phase = Phase::Done(ModeSetOutcome::Applied { mode });
                return None;
            }
        }
        if let Some(dip) = status.dip_switches {
            if dip != SET_FROM_KISS_MODE {
                self.phase = Phase::Done(ModeSetOutcome::DipNotSoftware {
                    requested: self.mode,
                    dip,
                });
                return None;
            }
        }
        self.next_attempt(now_ms)
    }

    fn next_attempt(&mut self, now_ms: u64) -> Option<Step> {
        if self.attempt >= self.attempts() {
            self.phase = Phase::Done(ModeSetOutcome::NotApplied {
                requested: self.mode,
                attempts: self.attempt,
                last_running: self.last_running,
                last_unknown_byte: self.last_unknown_byte,
                heard_any: self.heard_any,
            });
            return None;
        }
        self.attempt += 1;
        self.phase = Phase::Settling {
            until_ms: now_ms.saturating_add(self.policy.settle_ms),
        };
        Some(Step::SendSetHardware {
            payload: self.payload,
        })
    }

    /// A one-line, operator-facing description of where this change is up to.
    pub fn write_summary<W: fmt::Write + ?Sized>(&self, w: &mut W) -> fmt::Result {
        match self.phase {
            Phase::Done(outcome) => write_outcome(&outcome, w),
            _ => {
                write!(w, "Setting mode ")?;
                write_mode(self.mode, w)?;
                write!(w, ", attempt {} of {}...", self.attempt, self.attempts())
            }
        }
    }
}

/// The outcome to report without sending anything, when the TNC's firmware is
/// known and older than pico-node supports. `None` means go ahead (including
/// when the firmware is not known yet: the readback will catch it).
pub fn refuse_before_sending(mode: u8, firmware: Option<FirmwareVersion>) -> Option<ModeSetOutcome> {
    match firmware {
        Some(version) if !version.is_supported() => Some(ModeSetOutcome::FirmwareTooOld {
            requested: mode,
            version,
        }),
        _ => None,
    }
}

/// Describe a finished mode change in plain words.
pub fn write_outcome<W: fmt::Write + ?Sized>(outcome: &ModeSetOutcome, w: &mut W) -> fmt::Result {
    match *outcome {
        ModeSetOutcome::FirmwareTooOld { requested, version } => {
            write!(w, "Mode ")?;
            write_mode(requested, w)?;
            write!(
                w,
                " not set: the TNC runs firmware {}.{} and pico-node needs {}.{MIN_SUPPORTED_MINOR} \
or later. Update the TNC firmware first.",
                version.major, version.minor, version.major
            )
        }
        ModeSetOutcome::Applied { mode } => {
            write!(w, "Mode ")?;
            write_mode(mode.mode, w)?;
            write!(w, " confirmed by the TNC.")
        }
        ModeSetOutcome::SentUnverified { mode } => {
            write!(w, "Mode {mode} sent (the TNC cannot confirm this one).")
        }
        ModeSetOutcome::DipNotSoftware { requested, dip } => {
            write!(w, "The TNC ignored mode ")?;
            write_mode(requested, w)?;
            write!(w, ": its MODE DIP switches read ")?;
            write_dip(dip, w)?;
            write!(
                w,
                ". Set all four MODE switches to 1 (on) to set the mode from here."
            )
        }
        ModeSetOutcome::NotApplied {
            requested,
            attempts,
            last_running,
            last_unknown_byte,
            heard_any,
        } => {
            write!(w, "The TNC did not apply mode ")?;
            write_mode(requested, w)?;
            write!(w, " after {attempts} tries")?;
            match (last_running, last_unknown_byte, heard_any) {
                (Some(m), _, _) => {
                    write!(w, "; it is still running mode ")?;
                    write_mode(m.mode, w)?;
                    write!(w, ".")
                }
                (None, Some(b), _) => write!(
                    w,
                    "; it reports mode byte 0x{b:02X}, which the node's mode table (firmware \
x.44) does not know."
                ),
                (None, None, true) => write!(w, "; its reply did not say which mode it runs."),
                (None, None, false) => write!(
                    w,
                    ": no reply from the TNC. Check the serial wiring (TX and RX crossed, \
ground) and that the TNC is powered."
                ),
            }
        }
    }
}

/// `6 (1200 AFSK AX.25)`, or just the number for an unknown mode.
pub fn write_mode<W: fmt::Write + ?Sized>(mode: u8, w: &mut W) -> fmt::Result {
    match catalog::try_get_by_mode(mode) {
        Some(m) => write!(w, "{} ({})", m.mode, m.name),
        None => write!(w, "{mode}"),
    }
}

/// A DIP position as the four switch states, e.g. `0110`.
pub fn write_dip<W: fmt::Write + ?Sized>(dip: u8, w: &mut W) -> fmt::Result {
    for bit in (0..4).rev() {
        w.write_char(if dip & (1 << bit) != 0 { '1' } else { '0' })?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::string::String;

    fn status(running: Option<u8>, dip: Option<u8>) -> NinoTncStatusFrame {
        NinoTncStatusFrame {
            running_mode: running.and_then(catalog::try_get_by_mode),
            firmware_mode_byte: None,
            dip_switches: dip,
            ..Default::default()
        }
    }

    fn summary(s: &ModeSetter) -> String {
        let mut out = String::new();
        s.write_summary(&mut out).unwrap();
        out
    }

    #[test]
    fn happy_path_settles_reads_back_and_confirms() {
        let policy = ModeVerifyPolicy::default();
        let (mut s, step) = ModeSetter::start(6, false, 1_000, policy).unwrap();
        assert_eq!(step, Step::SendSetHardware { payload: 22 });
        assert_eq!(s.deadline_ms(), Some(2_500));
        assert_eq!(s.on_time(2_499), None, "not before the settle ends");
        assert_eq!(s.on_time(2_500), Some(Step::SendGetAll));
        assert_eq!(s.on_status(&status(Some(6), Some(15)), 2_600), None);
        assert!(matches!(s.outcome(), Some(ModeSetOutcome::Applied { mode }) if mode.mode == 6));
        assert_eq!(s.deadline_ms(), None);
        assert_eq!(summary(&s), "Mode 6 (1200 AFSK AX.25) confirmed by the TNC.");
    }

    #[test]
    fn persist_to_flash_sends_the_bare_mode_number() {
        let (_, step) = ModeSetter::start(14, true, 0, ModeVerifyPolicy::default()).unwrap();
        assert_eq!(step, Step::SendSetHardware { payload: 14 });
    }

    #[test]
    fn out_of_range_mode_is_refused() {
        assert!(ModeSetter::start(16, false, 0, ModeVerifyPolicy::default()).is_none());
    }

    #[test]
    fn a_silently_ignored_sethw_is_retried_and_can_come_good() {
        // The packet.net #633 bench failure: DIP 1111, SETHW 11 left mode 8 running;
        // a re-send took.
        let (mut s, _) = ModeSetter::start(11, false, 0, ModeVerifyPolicy::default()).unwrap();
        assert_eq!(s.on_time(1_500), Some(Step::SendGetAll));
        assert_eq!(
            s.on_status(&status(Some(8), Some(15)), 1_600),
            Some(Step::SendSetHardware { payload: 27 })
        );
        assert_eq!(s.attempt(), 2);
        assert_eq!(summary(&s), "Setting mode 11 (2400 QPSK IL2P+CRC), attempt 2 of 3...");
        assert_eq!(s.on_time(3_100), Some(Step::SendGetAll));
        assert_eq!(s.on_status(&status(Some(11), Some(15)), 3_200), None);
        assert!(matches!(s.outcome(), Some(ModeSetOutcome::Applied { .. })));
    }

    #[test]
    fn gives_up_after_the_attempt_budget_naming_what_is_running() {
        let (mut s, _) = ModeSetter::start(11, false, 0, ModeVerifyPolicy::default()).unwrap();
        let mut now = 0;
        for attempt in 1..=3u8 {
            now += 1_500;
            assert_eq!(s.on_time(now), Some(Step::SendGetAll), "attempt {attempt}");
            let next = s.on_status(&status(Some(8), Some(15)), now);
            if attempt < 3 {
                assert_eq!(next, Some(Step::SendSetHardware { payload: 27 }));
            } else {
                assert_eq!(next, None);
            }
        }
        assert_eq!(
            summary(&s),
            "The TNC did not apply mode 11 (2400 QPSK IL2P+CRC) after 3 tries; it is still \
running mode 8 (300 BPSK IL2P+CRC)."
        );
    }

    #[test]
    fn dip_switches_not_all_on_fails_fast_and_says_so() {
        let (mut s, _) = ModeSetter::start(8, false, 0, ModeVerifyPolicy::default()).unwrap();
        s.on_time(1_500);
        assert_eq!(s.on_status(&status(Some(6), Some(0b0110)), 1_600), None);
        assert_eq!(
            s.outcome(),
            Some(ModeSetOutcome::DipNotSoftware {
                requested: 8,
                dip: 6
            })
        );
        assert_eq!(
            summary(&s),
            "The TNC ignored mode 8 (300 BPSK IL2P+CRC): its MODE DIP switches read 0110. Set \
all four MODE switches to 1 (on) to set the mode from here."
        );
    }

    #[test]
    fn dips_that_already_select_the_requested_mode_count_as_applied() {
        let (mut s, _) = ModeSetter::start(6, false, 0, ModeVerifyPolicy::default()).unwrap();
        s.on_time(1_500);
        s.on_status(&status(Some(6), Some(6)), 1_600);
        assert!(matches!(s.outcome(), Some(ModeSetOutcome::Applied { .. })));
    }

    #[test]
    fn silence_resends_then_reports_no_reply() {
        let (mut s, _) = ModeSetter::start(6, false, 0, ModeVerifyPolicy::default()).unwrap();
        let mut now = 0;
        for attempt in 1..=3u8 {
            now += 1_500;
            assert_eq!(s.on_time(now), Some(Step::SendGetAll));
            now += 5_000;
            let next = s.on_time(now);
            if attempt < 3 {
                assert_eq!(next, Some(Step::SendSetHardware { payload: 22 }));
            } else {
                assert_eq!(next, None);
            }
        }
        assert!(summary(&s).contains("no reply from the TNC"));
    }

    #[test]
    fn a_status_heard_while_settling_is_ignored() {
        let (mut s, _) = ModeSetter::start(6, false, 0, ModeVerifyPolicy::default()).unwrap();
        // The old mode's periodic beacon, arriving inside the settle window.
        assert_eq!(s.on_status(&status(Some(8), Some(15)), 500), None);
        assert_eq!(s.outcome(), None);
        assert_eq!(s.attempt(), 1);
    }

    #[test]
    fn an_unknown_firmware_mode_byte_is_reported_raw() {
        let (mut s, _) = ModeSetter::start(6, false, 0, ModeVerifyPolicy {
            attempts: 1,
            ..ModeVerifyPolicy::default()
        })
        .unwrap();
        s.on_time(1_500);
        let st = NinoTncStatusFrame {
            firmware_mode_byte: Some(0x77),
            dip_switches: Some(15),
            ..Default::default()
        };
        s.on_status(&st, 1_600);
        assert!(summary(&s).contains("mode byte 0x77"));
    }

    #[test]
    fn mode_15_is_sent_unverified() {
        let (s, step) = ModeSetter::start(15, false, 0, ModeVerifyPolicy::default()).unwrap();
        assert_eq!(step, Step::SendSetHardware { payload: 31 });
        assert_eq!(s.deadline_ms(), None);
        assert_eq!(
            s.outcome(),
            Some(ModeSetOutcome::SentUnverified { mode: 15 })
        );
    }

    #[test]
    fn a_3_39_tnc_is_refused_on_the_first_readback() {
        // The bench NinoTNC on 2026-09-25: firmware 3.39, DIPs 1111, running
        // mode byte 0x80 (not in the 3.44 table). Before this, the node spent
        // three SETHWs and blamed an unknown mode byte.
        let (mut s, _) = ModeSetter::start(5, true, 0, ModeVerifyPolicy::default()).unwrap();
        s.on_time(1_500);
        let st = NinoTncStatusFrame {
            firmware_version: FirmwareVersion::parse("3.39"),
            firmware_mode_byte: Some(0x80),
            dip_switches: Some(15),
            ..Default::default()
        };
        assert_eq!(s.on_status(&st, 1_600), None, "no retry");
        assert_eq!(s.attempt(), 1);
        assert_eq!(
            summary(&s),
            "Mode 5 (3600 QPSK IL2P+CRC) not set: the TNC runs firmware 3.39 and pico-node \
needs 3.44 or later. Update the TNC firmware first."
        );
    }

    #[test]
    fn known_old_firmware_is_refused_before_sending_anything() {
        let v = |s| FirmwareVersion::parse(s);
        assert!(matches!(
            refuse_before_sending(6, v("3.39")),
            Some(ModeSetOutcome::FirmwareTooOld { requested: 6, .. })
        ));
        assert!(refuse_before_sending(6, v("3.43")).is_some());
        assert_eq!(refuse_before_sending(6, v("3.44")), None);
        assert_eq!(refuse_before_sending(6, None), None, "unknown: let the readback decide");
    }

    #[test]
    fn dip_renders_as_four_switches() {
        let mut out = String::new();
        write_dip(0b1111, &mut out).unwrap();
        write_dip(0b0001, &mut out).unwrap();
        assert_eq!(out, "11110001");
    }
}
