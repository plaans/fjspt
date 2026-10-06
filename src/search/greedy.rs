// ============= Forward progression ===========

use crate::problem::{Encoding, OperationId};
use crate::search::Model;

use aries_solver::prelude::*;

use aries_solver::backtrack::{Backtrack, DecLvl, DecisionLevelTracker};
use aries_solver::core::state::OptDomain;
use aries_solver::core::views::{Boundable, Term};
use aries_solver::solver::search::{Decision, SearchControl};
use aries_solver::solver::stats::Stats;
use itertools::Itertools;

#[derive(Clone)]
pub struct EstBrancher {
    pb: Encoding,
    lvl: DecisionLevelTracker,
    #[allow(unused)]
    last: Option<OperationId>,
}

impl EstBrancher {
    pub fn new(pb: &Encoding) -> Self {
        EstBrancher {
            pb: pb.clone(),
            lvl: Default::default(),
            last: None,
        }
    }

    #[allow(unused)]
    fn print(&self, op: OperationId, model: &Model) {
        let alt = self.pb.all_alternatives().find(|alt| alt.id == op).unwrap();
        println!(
            " {:?} -- prez: {}, start: [{}, {}]",
            op,
            model.entails(alt.presence),
            model.lb(alt.start()),
            model.ub(alt.start())
        );
        self.print_in_transports(op, model);
    }
    fn print_in_transports(&self, op_id: OperationId, model: &Model) {
        for t in self.pb.transports_with_robots_to(op_id) {
            if model.entails(!t.presence) {
                continue;
            }
            let earliest_start = model.lb(t.start);
            println!(
                "      -> [{earliest_start}, {}] {:?}",
                model.entails(t.presence),
                t
            );
        }
    }
}

impl SearchControl<String> for EstBrancher {
    fn next_decision(&mut self, _stats: &Stats, model: &Model) -> Option<Decision> {
        // println!("POST - propagation: {:?}", self.last);
        // if let Some(last) = self.last {
        //     self.print(last, model);
        // }
        // println!("\n\n");

        // Among all tasks that are not decided (not present nor absent or not bound start variable),
        // select the one with the smallest "earliest starting time (est)", tie breaking by the least slack
        // (latest start time `lst` is equivalent to slack when comparing variables with the same `est`)
        let best =
            active_tasks(&self.pb, model).min_by_key(|(_var, est, lst, _op_id)| (*est, *lst))?;

        let (task_start, task_est, _task_lst, op_id) = best;

        // println!();
        // println!("Best: {:?}", best);

        // decision is to set the start time to the selected task to the smallest possible value.
        // if no task was selected, it means that they are all instantiated and we have a complete schedule
        let prez = model.presence(task_start);

        let incoming_transports = self
            .pb
            .transports_with_robots_to(op_id)
            .filter_map(|t| {
                if model.entails(!t.presence) {
                    return None;
                }
                let earliest_start = model.lb(t.start);
                Some((earliest_start, t.presence, t))
            })
            .collect_vec();

        // among all transports that may serve this operation, select the one that finishes first
        let earliest_ending_transport = incoming_transports
            .iter()
            .min_by_key(|(est, _, t)| est + t.duration)
            .map(|(_, _, t)| t);

        // sanity check that propagation works well (only applicable before the first conflict (because a clause may be learned that changes propagation))
        if let Some(earliest_ending_transport) = earliest_ending_transport {
            let earliest_transport_end =
                model.lb(earliest_ending_transport.start + earliest_ending_transport.duration);
            debug_assert!(
                earliest_transport_end <= task_est,
                "Propagation is weaker than it should be ({earliest_transport_end} <= {task_est})"
            );
        }

        // DEBUG/ print the selected task current status before the decision
        // println!("\n# PRE DECISION: {op_id:?}");
        // self.print(op_id, model);
        // self.last = Some(op_id);

        // we want to set the selected task to be present executed at its earliest.
        // If propagation is strong enough (which it always should be!) this should be the lower bound.
        // This should be possible with the transport `t` that finishes the earliest (because its bounds should have been propagated to the task)
        // We need several decisions for this:
        //  - enable the transport (if there is one)
        //  - force the transport to start at its earliest (if there is one)
        //  - enable the task
        //  - force the task to start at its earliest
        //
        // We try these decision in order until all have been enforced,
        if let Some(trans) = earliest_ending_transport
            && !model.entails(trans.presence)
        {
            // println!("  -- 1 ------ {:?}", earliest_ending_transport);
            Some(Decision::SetLiteral(trans.presence))
        } else if let Some(trans) = earliest_ending_transport
            && let (lb, ub) = model.bounds(trans.start)
            && lb != ub
        {
            // println!("  -- 2 ------ {:?}", earliest_ending_transport);
            Some(Decision::SetLiteral(trans.start.leq(lb)))
        } else if !model.entails(prez) {
            // println!("  -- 3 ------ ");
            Some(Decision::SetLiteral(prez))
        } else {
            // println!("  -- 4 ------ ");
            Some(Decision::SetLiteral(Lit::leq(task_start, task_est)))
        }
    }

    fn clone_to_box(&self) -> Box<dyn SearchControl<String> + Send> {
        Box::new(self.clone())
    }
}

impl Backtrack for EstBrancher {
    fn save_state(&mut self) -> DecLvl {
        self.lvl.save_state()
    }

    fn num_saved(&self) -> u32 {
        self.lvl.num_saved()
    }

    fn restore_last(&mut self) {
        self.lvl.restore_last()
    }
}

/// Returns an iterator over all timepoints that not bound yet.
/// Each item in the iterator is a tuple `(var, est, lst)` where:
///  - `var` is the temporal variable
///  - `est` is its lower bound (the earliest start time of the task)
///  - `lst` is its upper bound (the latest start time of the task)
///  - `est < lst`: the start time of the task has not been decided yet.
fn active_tasks<'a>(
    pb: &'a Encoding,
    model: &'a Model,
) -> impl Iterator<Item = (Var, IntCst, IntCst, OperationId)> + 'a {
    pb.all_alternatives().filter_map(move |a| {
        let v = a.start().var.variable();
        // keep all variables that not absent and not bound
        match model.opt_domain_of(v) {
            OptDomain::Present(lb, ub) if lb < ub => Some((v, lb, ub, a.id)),
            OptDomain::Unknown(lb, ub) => Some((v, lb, ub, a.id)),
            _ => None,
        }
    })
}
