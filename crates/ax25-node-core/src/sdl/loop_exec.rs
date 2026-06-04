//! `LoopRange` expander — ports `Packet.Ax25.Session.SdlLoopExecutor`.
//!
//! A generated transition/subroutine path carries a flat `actions` list plus zero
//! or more [`LoopRange`]s describing `loop_while` constructs as slices over that
//! list. This expander walks the actions, and when it reaches a loop body it
//! repeats the body while the loop's predicate holds — test-at-head (`while`; body
//! may run zero times) or test-at-tail (`do-while`; body runs at least once) — up
//! to a safety cap, then continues past the body.
//!
//! It depends only on the *shape* of [`LoopRange`] (which mirrors the generated
//! `ax25sdl` Rust `LoopRange` type field-for-field), not on the table data, so it
//! is fully host-testable now — independent of the upstream "publish + no_std +
//! typed" blockers in [`super`]. When the real tables are wired in, the generated
//! `LoopRange` slices feed straight into [`run_loop`].

/// A `loop_while` rendered as a slice over the flat action list. Field-for-field
/// the same as `ax25sdl`'s generated `LoopRange` (`start`, `length`, `predicate`,
/// `test_at_end`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LoopRange {
    /// Index of the first action of the loop body in the flat list.
    pub start: usize,
    /// Number of actions in the loop body.
    pub length: usize,
    /// The continue condition (already negated where the figure's continuing edge
    /// is the decision's No branch). Opaque here — evaluated by the caller's guard
    /// closure. (When SP-010 reaches the Rust backend this becomes a typed guard.)
    pub predicate: &'static str,
    /// `false` = test-at-head (while), `true` = test-at-tail (do-while).
    pub test_at_end: bool,
}

/// Safety cap on loop-body iterations — matches the C# executor's 1024 guard, so
/// a pathological predicate can't hang the node.
pub const MAX_ITERATIONS: usize = 1024;

/// Walk `action_count` actions, expanding any [`LoopRange`] in `loops`, invoking
/// `run_action(index)` for each action to execute and `predicate_holds(range)` to
/// decide whether a loop continues. Returns the number of actions executed (for
/// tests / instrumentation).
///
/// Assumes loops do not overlap and are listed in body-start order — the same
/// invariant the generator guarantees and the C# executor relies on.
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

    fn lr(start: usize, length: usize, test_at_end: bool) -> LoopRange {
        LoopRange {
            start,
            length,
            predicate: "p",
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
        // actions: [0] then while-body [1] then [2]. Predicate false => body skipped.
        let loops = [lr(1, 1, false)];
        let mut order = Vec::new();
        run_loop(3, &loops, |i| order.push(i), |_| false);
        assert_eq!(order, vec![0, 2]);
    }

    #[test]
    fn do_while_runs_body_at_least_once() {
        // do-while body [1], predicate false after first pass => runs exactly once.
        let loops = [lr(1, 1, true)];
        let mut order = Vec::new();
        run_loop(3, &loops, |i| order.push(i), |_| false);
        assert_eq!(order, vec![0, 1, 2]);
    }

    #[test]
    fn while_loop_repeats_until_predicate_false() {
        // Body [1] repeats 3 times then continues to [2].
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
        // Body is actions [1,2], repeats twice, then [3].
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
        // Always-true predicate must terminate at MAX_ITERATIONS, not hang.
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
