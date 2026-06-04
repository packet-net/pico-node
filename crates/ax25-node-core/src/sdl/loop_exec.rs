//! `LoopRange` expander — ports `Packet.Ax25.Session.SdlLoopExecutor`.
//!
//! A generated transition/subroutine path carries a flat `actions` list plus zero
//! or more [`ax25sdl::LoopRange`]s describing `loop_while` constructs as slices
//! over that list. This expander walks the actions, and when it reaches a loop
//! body it repeats the body while the loop's predicate holds — test-at-head
//! (`while`; body may run zero times) or test-at-tail (`do-while`; body runs at
//! least once) — up to a safety cap, then continues past the body.
//!
//! It consumes the **real** generated [`ax25sdl::LoopRange`] directly (the
//! `predicate` is a typed [`ax25sdl::GuardTerm`], evaluated by the caller's
//! `predicate_holds` closure). Loops are assumed non-overlapping and listed in
//! body-start order — the invariant the generator guarantees and the C# executor
//! relies on.

use ax25sdl::LoopRange;

/// Safety cap on loop-body iterations — matches the C# executor's 1024 guard, so a
/// pathological predicate can't hang the node.
pub const MAX_ITERATIONS: usize = 1024;

/// Walk `action_count` actions, expanding any [`LoopRange`] in `loops`, invoking
/// `run_action(index)` for each action to execute and `predicate_holds(range)` to
/// decide whether a loop continues. Returns the number of actions executed (for
/// tests / instrumentation).
pub fn run_loop(
    action_count: usize,
    loops: &[LoopRange],
    mut run_action: impl FnMut(usize),
    mut predicate_holds: impl FnMut(&LoopRange) -> bool,
) -> usize {
    let mut executed = 0usize;
    let mut i = 0usize;
    while i < action_count {
        if let Some(range) = loops.iter().find(|r| r.start == i) {
            executed += run_one_loop(range, &mut run_action, &mut predicate_holds);
            i = range.start + range.length; // continue past the body
        } else {
            run_action(i);
            executed += 1;
            i += 1;
        }
    }
    executed
}

fn run_one_loop(
    range: &LoopRange,
    run_action: &mut impl FnMut(usize),
    predicate_holds: &mut impl FnMut(&LoopRange) -> bool,
) -> usize {
    let mut executed = 0usize;
    let body = range.start..range.start + range.length;

    if range.test_at_end {
        // do-while: run the body, then test; repeat while the predicate holds.
        let mut iters = 0;
        loop {
            for idx in body.clone() {
                run_action(idx);
                executed += 1;
            }
            iters += 1;
            if iters >= MAX_ITERATIONS || !predicate_holds(range) {
                break;
            }
        }
    } else {
        // while: test, then run the body; repeat while the predicate holds.
        let mut iters = 0;
        while predicate_holds(range) {
            for idx in body.clone() {
                run_action(idx);
                executed += 1;
            }
            iters += 1;
            if iters >= MAX_ITERATIONS {
                break;
            }
        }
    }
    executed
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec::Vec;
    use ax25sdl::{Ax25Guard, GuardTerm};

    fn lr(start: usize, length: usize, test_at_end: bool) -> LoopRange {
        LoopRange {
            start,
            length,
            // The predicate atom is irrelevant to the executor (the test drives
            // `predicate_holds` directly); any valid typed atom serves.
            predicate: GuardTerm {
                atom: Ax25Guard::PeerReceiverBusy,
                negate: false,
            },
            test_at_end,
        }
    }

    #[test]
    fn no_loops_runs_actions_in_order() {
        let mut order = Vec::new();
        let n = run_loop(3, &[], |i| order.push(i), |_| false);
        assert_eq!(order, vec![0, 1, 2]);
        assert_eq!(n, 3);
    }

    #[test]
    fn while_loop_runs_body_zero_times_when_predicate_false() {
        let loops = [lr(1, 1, false)];
        let mut order = Vec::new();
        run_loop(3, &loops, |i| order.push(i), |_| false);
        assert_eq!(order, vec![0, 2]);
    }

    #[test]
    fn do_while_runs_body_at_least_once() {
        let loops = [lr(1, 1, true)];
        let mut order = Vec::new();
        run_loop(3, &loops, |i| order.push(i), |_| false);
        assert_eq!(order, vec![0, 1, 2]);
    }

    #[test]
    fn while_loop_repeats_until_predicate_false() {
        let loops = [lr(1, 1, false)];
        let mut order = Vec::new();
        let mut left = 3;
        run_loop(
            3,
            &loops,
            |i| order.push(i),
            |_| {
                if left > 0 {
                    left -= 1;
                    true
                } else {
                    false
                }
            },
        );
        assert_eq!(order, vec![0, 1, 1, 1, 2]);
    }

    #[test]
    fn multi_action_loop_body() {
        let loops = [lr(1, 2, false)];
        let mut order = Vec::new();
        let mut left = 2;
        run_loop(
            4,
            &loops,
            |i| order.push(i),
            |_| {
                if left > 0 {
                    left -= 1;
                    true
                } else {
                    false
                }
            },
        );
        assert_eq!(order, vec![0, 1, 2, 1, 2, 3]);
    }

    #[test]
    fn safety_cap_bounds_a_runaway_predicate() {
        let loops = [lr(0, 1, false)];
        let mut count = 0usize;
        run_loop(1, &loops, |_| count += 1, |_| true);
        assert_eq!(count, MAX_ITERATIONS);
    }

    #[test]
    fn do_while_safety_cap() {
        let loops = [lr(0, 1, true)];
        let mut count = 0usize;
        run_loop(1, &loops, |_| count += 1, |_| true);
        assert_eq!(count, MAX_ITERATIONS);
    }
}
