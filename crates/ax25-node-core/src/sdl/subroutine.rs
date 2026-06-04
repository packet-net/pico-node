//! Subroutine walking — ports `Packet.Ax25.Session.DefaultSubroutineRegistry`.
//!
//! The dispatcher routes every `kind: subroutine` verb here by canonical name.
//! [`invoke`] looks the name up in the generated figc4.7 [`ax25sdl::DATA_LINK_SUBROUTINES`]
//! table, evaluates each path's guard against the current context (first match
//! wins), and executes that path's action chain via the dispatcher — which may
//! recurse into further subroutine calls (`Establish_Data_Link` →
//! `Clear_Exception_Conditions` / `Select_T1` / `Enquiry_Response`). The nesting is
//! shallow + non-cyclic (the figc4.7 call graph is a DAG), so the recursion is
//! bounded on the M0+ stack (flip-link guards an overflow into a clean fault).
//!
//! Unlike the C# registry, there is no Wire()/no-op-stub indirection: the table is
//! a compile-time constant, so the walker is always live. The two legacy-alias
//! names the C# kept (`Select_T1_Value`, capital-F `Check_Need_For_Response`) are
//! resolved at the dispatcher call site (it already passes the canonical names);
//! the `Enquiry_Response_F_0/F_1` context-binding aliases are handled by
//! [`invoke_enquiry_response`].

use ax25sdl::{SubroutinePath, DATA_LINK_SUBROUTINES};

use super::dispatch::execute_actions;
use super::guard::eval_guard;
use super::tx::Tx;

/// Invoke the named figc4.7 subroutine: walk its paths, run the first whose guard
/// holds. Unknown names panic (a transcription/wiring bug, not a wire condition) —
/// the Rust analogue of the C# registry's throw.
pub fn invoke(name: &str, tx: &mut Tx<'_>) {
    let spec = DATA_LINK_SUBROUTINES
        .subroutines
        .iter()
        .find(|s| s.name == name)
        .unwrap_or_else(|| panic_unknown(name));
    walk_first_matching_path(spec.paths, tx);
}

/// Invoke `Enquiry_Response` with the figc4.7b `(F = 0)` / `(F = 1)` annotation:
/// bind the pending P/F bit before walking the canonical body, so the response
/// frame goes out with the right F bit (m0lte/ax25sdl#45). Without this, a poll
/// gets an F=0 response and the poller's `response_and_F_eq_1` guard never matches.
pub fn invoke_enquiry_response(tx: &mut Tx<'_>, f_bit: bool) {
    tx.pending.pf = Some(f_bit);
    invoke("Enquiry_Response", tx);
}

/// Walk a subroutine's paths in order; execute the first whose guard holds, then
/// return. No matching path is a silent no-op (matches the C# walker + the SDL
/// semantics that an unmatched decision falls through).
fn walk_first_matching_path(paths: &[SubroutinePath], tx: &mut Tx<'_>) {
    for path in paths {
        if eval_guard(path.guard, tx.session, tx.timers, &tx.trigger) {
            execute_actions(path.actions, path.loops, tx);
            return;
        }
    }
}

#[cold]
#[inline(never)]
fn panic_unknown(name: &str) -> ! {
    panic!("unknown SDL subroutine: `{name}` — not declared in DATA_LINK_SUBROUTINES (transcription typo or wiring bug)");
}
