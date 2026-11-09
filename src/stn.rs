//! Export of a solution as a partial-order schedule, i.e. a Simple Temporal Network (STN).
//!
//! The solver fixes every start time, but most of the CP model is made of difference
//! constraints (`a + c <= b`). Keeping only the difference constraints that are active
//! in the solution (selected alternatives, selected robots, and the order chosen for each
//! machine and each robot) gives an STN of which the solution is one assignment.
//!
//! Timepoints:
//!  - `origin` (always 0)
//!  - start and end of each operation (on its selected machine)
//!  - pickup and drop of each transport (by its selected robot)
//!
//! Constraints (`lb <= X_b - X_a <= ub`, `None` = unbounded):
//!  - operation:        end - start == duration
//!  - job order:        end(op_i) <= start(op_i+1)
//!  - transport:        end(prev op) <= pickup,  drop <= start(next op)
//!  - transport travel: drop - pickup >= transport time  (relaxed from `==` in the solver)
//!  - machine order:    end(op) <= start(next op on the same machine)
//!  - robot order:      drop(t) + empty travel <= pickup(next transport of the same robot)
//!  - origin:           every timepoint >= 0
//!
//! The makespan bound is NOT included as a constraint: its value is exported separately so
//! that the consumer can choose the deadline (e.g. the solution makespan, or a relaxed one).

use crate::problem::{Encoding, OperationId, Problem};
use aries_solver::prelude::*;
use itertools::Itertools;
use std::collections::HashMap;
use std::fmt::Write;

#[derive(Default)]
struct Stn {
    names: Vec<String>,
    /// Value of each timepoint in the solution
    values: Vec<IntCst>,
    /// (a, b, lb, ub) meaning lb <= X_b - X_a <= ub
    constraints: Vec<(usize, usize, Option<IntCst>, Option<IntCst>)>,
}

impl Stn {
    fn timepoint(&mut self, name: String, value: IntCst) -> usize {
        self.names.push(name);
        self.values.push(value);
        self.names.len() - 1
    }

    fn add(&mut self, a: usize, b: usize, lb: Option<IntCst>, ub: Option<IntCst>) {
        // sanity check: the solution must satisfy every exported constraint
        let diff = self.values[b] - self.values[a];
        assert!(
            lb.is_none_or(|lb| lb <= diff) && ub.is_none_or(|ub| diff <= ub),
            "solution violates exported constraint {} -> {}",
            self.names[a],
            self.names[b]
        );
        self.constraints.push((a, b, lb, ub));
    }
}

/// A transport waypoint of a robot: the machine where it must be and the matching timepoint.
struct Waypoint {
    machine: u32,
    timepoint: usize,
}

/// Builds the STN of the solution and returns it as a JSON string.
pub fn export_stn(solution: &Solution, pb: &Problem, e: &Encoding) -> String {
    let mut stn = Stn::default();
    let origin = stn.timepoint("origin".to_string(), 0);

    // ---- operations (selected alternative only)
    let mut op_tps: HashMap<OperationId, (usize, usize)> = HashMap::new();
    let mut ops_on_machine: Vec<Vec<(IntCst, usize, usize)>> = vec![Vec::new(); pb.num_machines as usize];

    for job in pb.jobs() {
        let mut prev_end: Option<usize> = None;
        for op in e.operations_ids(job) {
            let alt = e
                .alternatives(job, op)
                .find(|a| solution.entails(a.presence))
                .unwrap();
            let start = solution.eval(alt.start()).unwrap();
            let s = stn.timepoint(format!("op_j{job}_o{op}_start"), start);
            let t = stn.timepoint(format!("op_j{job}_o{op}_end"), start + alt.duration);
            stn.add(origin, s, Some(0), None);
            stn.add(s, t, Some(alt.duration), Some(alt.duration));
            if let Some(prev) = prev_end {
                stn.add(prev, s, Some(0), None);
            }
            prev_end = Some(t);
            op_tps.insert(alt.id, (s, t));
            ops_on_machine[alt.machine as usize].push((start, s, t));
        }
    }

    // ---- machine order (as chosen in the solution)
    for ops in &mut ops_on_machine {
        ops.sort();
        for ((_, _, end_i), (_, start_j, _)) in ops.iter().copied().tuple_windows() {
            stn.add(end_i, start_j, Some(0), None);
        }
    }

    // ---- transports (selected robot only)
    let mut transports_of_robot: Vec<Vec<(IntCst, IntCst, u32, u32, usize, usize)>> =
        vec![Vec::new(); pb.num_robots];

    for transport in e.transports() {
        let Some(tr) = transport
            .alternatives
            .iter()
            .find(|t| solution.entails(t.presence))
        else {
            continue; // transport between two alternatives that were not selected
        };
        let start = solution.eval(tr.start).unwrap();
        let end = start + tr.duration;
        let name = format!("r{}_j{}_o{}", tr.robot, tr.next.job, tr.next.op);
        let pick = stn.timepoint(format!("{name}_pickup"), start);
        let drop = stn.timepoint(format!("{name}_drop"), end);

        stn.add(origin, pick, Some(0), None);
        stn.add(pick, drop, Some(tr.duration), None);
        stn.add(op_tps[&tr.previous].1, pick, Some(0), None);
        stn.add(drop, op_tps[&tr.next].0, Some(0), None);

        transports_of_robot[tr.robot as usize].push((start, end, tr.from_machine, tr.to_machine, pick, drop));
    }

    // ---- robot order (as chosen in the solution) + waypoint sequences
    let mut waypoints: Vec<Vec<Waypoint>> = Vec::new();
    for transports in &mut transports_of_robot {
        transports.sort();
        for (ti, tj) in transports.iter().copied().tuple_windows() {
            let (_, _, _, to_i, _, drop_i) = ti;
            let (_, _, from_j, _, pick_j, _) = tj;
            let empty_travel = pb.transport_time(to_i, from_j).unwrap() as IntCst;
            stn.add(drop_i, pick_j, Some(empty_travel), None);
        }
        waypoints.push(
            transports
                .iter()
                .flat_map(|&(_, _, from, to, pick, drop)| {
                    [
                        Waypoint { machine: from, timepoint: pick },
                        Waypoint { machine: to, timepoint: drop },
                    ]
                })
                .collect(),
        );
    }

    to_json(&stn, &waypoints, solution.eval(e.makespan).unwrap())
}

fn to_json(stn: &Stn, waypoints: &[Vec<Waypoint>], makespan: IntCst) -> String {
    let bound = |b: Option<IntCst>| b.map_or("null".to_string(), |v| v.to_string());
    let mut out = String::new();
    writeln!(out, "{{").unwrap();
    writeln!(out, "  \"makespan\": {makespan},").unwrap();

    writeln!(out, "  \"timepoints\": [").unwrap();
    let tps = stn
        .names
        .iter()
        .zip(&stn.values)
        .map(|(n, v)| format!("    {{\"name\": \"{n}\", \"value\": {v}}}"))
        .join(",\n");
    writeln!(out, "{tps}\n  ],").unwrap();

    writeln!(out, "  \"constraints\": [").unwrap();
    let cs = stn
        .constraints
        .iter()
        .map(|&(a, b, lb, ub)| format!("    [{a}, {b}, {}, {}]", bound(lb), bound(ub)))
        .join(",\n");
    writeln!(out, "{cs}\n  ],").unwrap();

    writeln!(out, "  \"robots\": [").unwrap();
    let rs = waypoints
        .iter()
        .map(|wps| {
            let items = wps
                .iter()
                .map(|w| format!("{{\"machine\": {}, \"timepoint\": {}}}", w.machine, w.timepoint))
                .join(", ");
            format!("    [{items}]")
        })
        .join(",\n");
    writeln!(out, "{rs}\n  ]").unwrap();
    writeln!(out, "}}").unwrap();
    out
}
