//! Gate 7 (HW-BRINGUP.md §4): the on-target test suite — `cargo test` in this
//! crate flashes the real RP2040 over the debug probe and runs each case with a
//! device reset in between (probe-rs autodetects the embedded-test binary).
//!
//! The headline case mirrors the core's host-side two-session wire harness
//! (`ax25_node_core::sdl::harness_tests`): two full SDL sessions complete a
//! SABM/UA connect, exchange an I-frame, and tear down with DISC/UA — with every
//! frame carried between them as *encoded wire octets* through the real codec +
//! `classify_incoming`. Here that whole loop executes on the physical M0+ (no
//! FPU, no atomics CAS, embedded-alloc heap) — the on-target proof the research
//! note's Loop C exists for.

#![no_std]
#![no_main]

extern crate alloc;

use defmt_rtt as _;

use embedded_alloc::LlffHeap;

#[global_allocator]
static HEAP: LlffHeap = LlffHeap::empty();

const HEAP_SIZE: usize = 16 * 1024;

#[embedded_test::tests]
mod tests {
    use super::{HEAP, HEAP_SIZE};

    use alloc::vec;
    use alloc::vec::Vec;
    use core::mem::MaybeUninit;

    use ax25_node_core::ax25::{Callsign, Frame, PID_NO_LAYER3};
    use ax25_node_core::sdl::{
        classify_incoming, DataLinkSignal, Event, MockTimerService, Session, State, WireSink,
    };

    /// One station: a full SDL session + a wire sink addressed peer-ward.
    /// (The on-target twin of the host harness's `Station`.)
    struct Station {
        session: Session,
        timers: MockTimerService,
        sink: WireSink,
    }

    impl Station {
        fn new(local: &str, remote: &str) -> Self {
            Station {
                session: Session::new(),
                timers: MockTimerService::new(),
                sink: WireSink::new(
                    Callsign::parse(local).unwrap(),
                    Callsign::parse(remote).unwrap(),
                    Vec::new(),
                ),
            }
        }

        fn post(&mut self, event: Event) -> Vec<Vec<u8>> {
            self.sink.sent.clear();
            self.session
                .post_event(event, &mut self.timers, &mut self.sink);
            core::mem::take(&mut self.sink.sent)
        }

        fn take_upward(&mut self) -> Vec<DataLinkSignal> {
            core::mem::take(&mut self.sink.upward)
        }
    }

    /// Deliver wire frames into `dst` (decode → classify → post), returning what
    /// `dst` emits in response.
    fn deliver(dst: &mut Station, frames: &[Vec<u8>]) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        for bytes in frames {
            let frame = Frame::decode(bytes).expect("harness frame decodes");
            let event = classify_incoming(&frame).expect("harness frame classifies");
            out.extend(dst.post(event));
        }
        out
    }

    #[init]
    fn init() {
        // Heap arena for ax25-node-core's alloc use. The device resets between
        // test cases, so this runs once per case on a fresh chip.
        {
            static mut ARENA: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
            #[allow(static_mut_refs)]
            unsafe {
                HEAP.init(ARENA.as_ptr() as usize, HEAP_SIZE)
            }
        }
        let _p = embassy_rp::init(Default::default());
    }

    /// The codec fundamentals execute on the M0+: callsign parse/display, frame
    /// encode/decode round trip, CRC-16/X.25 over a known vector.
    #[test]
    fn core_codec_runs_on_target() {
        let call = Callsign::parse("M0LTE-7").unwrap();
        let mut buf = [0u8; 16];
        let n = call.write_display(&mut buf).unwrap();
        assert_eq!(&buf[..n], b"M0LTE-7");

        // CRC-16/X.25 known-answer: "123456789" -> 0x906E.
        assert_eq!(ax25_node_core::crc::compute(b"123456789"), 0x906E);

        let sink = WireSink::new(
            Callsign::parse("M0LTE-1").unwrap(),
            Callsign::parse("IDENT").unwrap(),
            Vec::new(),
        );
        let frame = sink.build_frame(&ax25_node_core::sdl::FrameSpec::Information {
            p: false,
            nr: 2,
            ns: 3,
            pid: PID_NO_LAYER3,
            info: vec![0xAA, 0x55],
        });
        let decoded = Frame::decode(&frame.encode()).expect("round trip decodes");
        assert_eq!(decoded, frame);
    }

    /// The full connected-mode lifecycle on the physical M0+: SABM/UA connect,
    /// I-frame + ack exchange, DISC/UA teardown — every frame as wire octets.
    #[test]
    fn sdl_connect_iframe_disconnect_on_target() {
        let mut a = Station::new("M0LTE-1", "M0LTE-2");
        let mut b = Station::new("M0LTE-2", "M0LTE-1");

        // Connect: A's SABM → B accepts with UA → A confirms.
        let from_a = a.post(Event::DlConnectRequest);
        assert_eq!(a.session.state, State::AwaitingConnection);
        let from_b = deliver(&mut b, &from_a);
        assert_eq!(b.session.state, State::Connected);
        assert!(b.take_upward().contains(&DataLinkSignal::ConnectIndication));
        let leftover = deliver(&mut a, &from_b);
        assert_eq!(a.session.state, State::Connected);
        assert!(a.take_upward().contains(&DataLinkSignal::ConnectConfirm));
        assert!(leftover.is_empty());

        // Data: A's I-frame (N(S)=0) → B delivers upward, V(R) advances.
        let from_a = a.post(Event::DlDataRequest(PID_NO_LAYER3, vec![0x01, 0x02, 0x03]));
        assert_eq!(a.session.context.vs, 1);
        let from_b = deliver(&mut b, &from_a);
        assert_eq!(b.session.context.vr, 1);
        assert!(b.take_upward().iter().any(
            |u| matches!(u, DataLinkSignal::DataIndication(_, info) if info == &vec![0x01, 0x02, 0x03])
        ));
        let _ = deliver(&mut a, &from_b);
        assert_eq!(a.session.state, State::Connected);

        // Teardown: A's DISC → B's UA + DisconnectIndication → A confirms.
        let from_a = a.post(Event::DlDisconnectRequest);
        assert_eq!(a.session.state, State::AwaitingRelease);
        let from_b = deliver(&mut b, &from_a);
        assert_eq!(b.session.state, State::Disconnected);
        assert!(b
            .take_upward()
            .contains(&DataLinkSignal::DisconnectIndication));
        let _ = deliver(&mut a, &from_b);
        assert_eq!(a.session.state, State::Disconnected);
        assert!(a.take_upward().contains(&DataLinkSignal::DisconnectConfirm));
    }

    /// The NET/ROM tap ingests a NODES broadcast on-target (fixed-capacity
    /// routing table, integer quality maths — the no-FPU path on real silicon).
    #[test]
    fn netrom_nodes_ingest_on_target() {
        use ax25_node_core::ax25::{frame::CONTROL_UI, Address, PID_NETROM};
        use ax25_node_core::netrom::wire::nodes_broadcast_builder::{
            write_nodes_frame, NodesAdvertisementEntry, MAX_NODES_FRAME_LEN,
        };
        use ax25_node_core::netrom::wire::Alias;
        use ax25_node_core::netrom::{NetRomService, ObserveOutcome, PortId};

        // One advertised destination, built with the production builder.
        let entry = NodesAdvertisementEntry {
            destination: Callsign::parse("M0LTE-3").unwrap(),
            destination_alias: Alias::from_str_lossy("REMOTE"),
            best_neighbour: Callsign::parse("M0LTE-2").unwrap(),
            quality: 192,
        };
        let mut info_buf = [0u8; MAX_NODES_FRAME_LEN];
        let n = write_nodes_frame(&Alias::from_str_lossy("PEER"), &[entry], &mut info_buf)
            .expect("buffer fits");

        let frame = Frame {
            destination: Address {
                callsign: Callsign::parse("NODES").unwrap(),
                crh: true,
                extension: false,
            },
            source: Address {
                callsign: Callsign::parse("M0LTE-2").unwrap(),
                crh: false,
                extension: false,
            },
            digipeaters: Vec::new(),
            control: CONTROL_UI,
            pid: Some(PID_NETROM),
            info: info_buf[..n].to_vec(),
        };

        let mut svc = NetRomService::new();
        let outcome = svc.observe_frame(
            &frame,
            Callsign::parse("M0LTE-1").unwrap(),
            PortId::from_str_lossy("test"),
            0,
        );
        assert_eq!(outcome, ObserveOutcome::Ingested { entries: 1 });
        // Two destinations learned: the assumed direct route to the broadcaster
        // plus the advertised M0LTE-3.
        assert_eq!(svc.destination_count(), 2);
    }
}
