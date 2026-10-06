#![allow(unused, clippy::needless_range_loop)]

use aries_solver::core::u32_to_cst;
use aries_solver::lang::exclusive_choice::exclu_choice;

use aries_solver::prelude::*;

use itertools::Itertools;
use std::fmt::{Debug, Formatter};

#[derive(Clone, Debug)]
pub struct Op {
    pub job: u32,
    pub op_id: u32,
    pub alternatives: Vec<Alt>,
}

impl Op {
    pub fn min_duration(&self) -> IntCst {
        self.alternatives.iter().map(|a| a.duration).min().unwrap()
    }
    /// Returns the maximum duration the operation may take.
    pub fn max_duration(&self) -> IntCst {
        self.alternatives.iter().map(|a| a.duration).max().unwrap()
    }
}

#[derive(Clone, Debug)]
pub struct Alt {
    pub machine: u32,
    pub duration: IntCst,
}

#[derive(Clone, Debug)]
pub struct Problem {
    pub num_jobs: u32,
    pub num_machines: u32,
    pub operations: Vec<Op>,
    /// If set, indicates the transportation between any pair of machines.
    /// For instance `transport_time[3][5]` indicates the time necessary to transport a piece from machine `3` to machine `5`.
    pub transport_times: Option<Vec<Vec<u32>>>,
    pub num_robots: usize,
}

impl Problem {
    pub fn ops(&self) -> impl Iterator<Item = &Op> + '_ {
        self.operations.iter()
    }

    pub fn ops_by_job(&self, job: u32) -> impl Iterator<Item = &Op> + '_ {
        self.ops().filter(move |op| op.job == job)
    }

    pub fn operation(&self, job: u32, op_id: u32) -> &Op {
        self.ops()
            .find(move |op| op.job == job && op.op_id == op_id)
            .unwrap()
    }

    pub fn machines(&self) -> impl Iterator<Item = u32> {
        0..self.num_machines
    }
    pub fn jobs(&self) -> impl Iterator<Item = u32> {
        0..self.num_jobs
    }

    /// Computes a lower bound on the makespan as the maximum of the operation durations in each
    /// job and on each machine.
    pub fn makespan_lower_bound(&self) -> IntCst {
        let max_of_jobs: IntCst = self
            .jobs()
            .map(|j| self.ops_by_job(j).map(|op| op.min_duration()).sum())
            .max()
            .unwrap();

        let mut max_by_machine = vec![0; self.num_machines as usize];
        for op in self.ops() {
            if op.alternatives.len() == 1 {
                let alt = &op.alternatives[0];
                max_by_machine[alt.machine as usize] += alt.duration;
            }
        }
        let max_of_machines: IntCst = max_by_machine.iter().max().copied().unwrap();

        max_of_jobs.max(max_of_machines)
    }

    /// Computes a naïve upper bound on the makespan, assuming all operations are scheduled in sequence.
    ///
    /// Under a makespan-minimization objective, this can be used as an upper bound on the domains of start-time variables.
    pub fn makespan_upper_bound(&self) -> IntCst {
        let max_transition_time = if let Some(transport_times) = self.transport_times.as_ref() {
            transport_times
                .iter()
                .flat_map(|tt| tt.iter())
                .copied()
                .max()
        } else {
            None
        };
        let max_transition_time = max_transition_time.unwrap_or(0) as IntCst;
        // a single that goes to the
        self.jobs()
            .map(|j| {
                // for each jo the robot move from BASE, then for each operation waits for the operation to finish + move to next,
                // returning to BASE when finished
                max_transition_time // move from initial loc
                    + self
                        .ops_by_job(j)
                        .map(|op| op.max_duration() + max_transition_time) // wait for job to finish + move to next machine/final position
                        .sum::<IntCst>()
            })
            .sum::<IntCst>()
    }

    /// Returns the required transportation time (if specified) to move between the two machines
    pub fn transport_time(&self, from_machine: u32, to_machine: u32) -> Option<u32> {
        if let Some(tt) = self.transport_times.as_ref() {
            let line = &tt[from_machine as usize % tt.len()];
            let time = line[to_machine as usize % line.len()];
            Some(time)
        } else {
            None
        }
    }

    pub fn set_transport_times(&mut self, transport_times: Vec<Vec<u32>>) {
        self.transport_times = Some(transport_times);
    }
}

/// Represents an operation that must be executed and is associated to one or more alternative.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub struct Operation {
    pub job: u32,
    pub op: u32,
    start: VarCst,
    end: VarCst,
}

#[derive(Copy, Clone, Eq, PartialEq, Hash)]
pub struct OperationId {
    pub job: u32,
    pub op: u32,
    /// If this represents an alternative, id of the alternative.
    /// A `None` value is used to identify the top-level `Operation`
    pub alt: Option<u32>,
}

impl Debug for OperationId {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "({}, {}, ", self.job, self.op)?;
        if let Some(alt) = self.alt {
            write!(f, "{alt})")
        } else {
            write!(f, ")")
        }
    }
}

/// Represents one alternative to an operation
#[derive(Clone, Debug)]
pub struct OperationAlternative {
    pub id: OperationId,
    pub machine: u32,
    pub duration: IntCst,
    pub start: Var,
    pub presence: Lit,
}

impl OperationAlternative {
    pub fn start(&self) -> VarCst {
        self.start.into()
    }

    pub fn end(&self) -> VarCst {
        self.start + self.duration
    }
}
#[derive(Debug, Clone)]
pub struct Transport {
    pub from_machine: u32,
    pub to_machine: u32,
    pub previous: OperationId,
    pub next: OperationId,
    pub presence: Lit,
    pub start: VarCst,
    pub duration: IntCst,
    pub alternatives: Vec<TransportWithRobot>,
}
#[derive(Clone, Copy, Debug)]
pub struct TransportWithRobot {
    pub from_machine: u32,
    pub to_machine: u32,
    pub previous: OperationId,
    pub next: OperationId,
    pub robot: u32,
    pub presence: Lit,
    pub start: VarCst,
    pub duration: IntCst,
}

/// Encoding of a scheduling problem, where each operation and alternative is associated with its variables in the CSP.
#[derive(Clone)]
pub struct Encoding {
    pub makespan: Var,
    operations: Vec<Operation>,
    alternatives: Vec<OperationAlternative>,
    transports: Vec<Transport>,
}

impl Encoding {
    pub fn new(pb: &Problem, lower_bound: IntCst, upper_bound: IntCst, m: &mut Model) -> Self {
        let makespan = m.new_variable(lower_bound, upper_bound);

        let mut operations = Vec::new();
        let mut alternatives = Vec::new();

        for op in pb.ops() {
            let job_id = op.job;
            let op_id = op.op_id;

            for (alt_id, alt) in op.alternatives.iter().enumerate() {
                let id = OperationId {
                    job: job_id,
                    op: op_id,
                    alt: Some(alt_id as u32),
                };
                let presence = if op.alternatives.len() == 1 {
                    Lit::TRUE
                } else {
                    m.new_presence_variable(Lit::TRUE, "").true_lit()
                };
                let start = m.new_optional_ivar(0, upper_bound, presence, "");
                alternatives.push(OperationAlternative {
                    id,
                    machine: alt.machine,
                    duration: alt.duration,
                    start,
                    presence,
                });
            }

            // build the top-level operation
            let operation = if op.alternatives.len() == 1 {
                // a single alternative, reused its variables of the operation
                let alt = alternatives.last().unwrap();
                Operation {
                    job: job_id,
                    op: op_id,
                    start: alt.start(),
                    end: alt.end(),
                }
            } else {
                // more that one alternative, create new variables for start/end
                let id = OperationId {
                    job: job_id,
                    op: op_id,
                    alt: None,
                };
                Operation {
                    job: job_id,
                    op: op_id,
                    start: m.new_optional_variable(0, upper_bound, Lit::TRUE).into(),
                    end: m.new_optional_variable(0, upper_bound, Lit::TRUE).into(),
                }
            };
            operations.push(operation);
        }
        Encoding {
            makespan,
            operations,
            alternatives,
            transports: Default::default(), // will be filled in later
        }
    }

    pub fn operations_ids(&self, job: u32) -> impl Iterator<Item = u32> + '_ {
        self.alternatives
            .iter()
            .filter_map(move |o| if o.id.job == job { Some(o.id.op) } else { None })
            .sorted()
            .unique()
    }

    pub fn all_operations(&self) -> impl Iterator<Item = &Operation> + '_ {
        self.operations.iter()
    }

    pub fn operation(&self, job: u32, op: u32) -> &Operation {
        self.all_operations()
            .find(move |alt| alt.job == job && alt.op == op)
            .unwrap()
    }

    pub fn alternatives(
        &self,
        job: u32,
        op: u32,
    ) -> impl Iterator<Item = &OperationAlternative> + '_ {
        self.alternatives
            .iter()
            .filter(move |alt| alt.id.job == job && alt.id.op == op)
    }

    pub fn all_alternatives(&self) -> impl Iterator<Item = &OperationAlternative> + '_ {
        self.alternatives.iter()
    }

    pub fn alternatives_on_machine(
        &self,
        machine: u32,
    ) -> impl Iterator<Item = &OperationAlternative> + '_ {
        self.all_alternatives()
            .filter(move |a| a.machine == machine)
    }

    pub fn transports(&self) -> impl Iterator<Item = &Transport> {
        self.transports.iter()
    }

    pub fn transports_from(&self, op: OperationId) -> impl Iterator<Item = &Transport> {
        self.transports().filter(move |t| t.previous == op)
    }

    pub fn transports_with_robots_from(
        &self,
        op: OperationId,
    ) -> impl Iterator<Item = &TransportWithRobot> {
        self.transports_from(op).flat_map(|t| t.alternatives.iter())
    }
    pub fn transports_to(&self, op: OperationId) -> impl Iterator<Item = &Transport> {
        self.transports().filter(move |t| t.next == op)
    }

    pub fn transports_with_robots_to(
        &self,
        op: OperationId,
    ) -> impl Iterator<Item = &TransportWithRobot> {
        self.transports_to(op).flat_map(|t| t.alternatives.iter())
    }
}

pub(crate) fn encode(
    pb: &Problem,
    lower_bound: u32,
    upper_bound: Option<u32>,
) -> (Model, Encoding) {
    let lower_bound = u32_to_cst(lower_bound);
    let upper_bound = upper_bound
        .map(u32_to_cst)
        .unwrap_or(pb.makespan_upper_bound());
    let mut m = Model::new();
    let mut e = Encoding::new(pb, lower_bound, upper_bound, &mut m);

    // enforce makespan after last alternative
    for oa in e.all_alternatives() {
        m.enforce_scoped(leq(oa.end(), e.makespan), [oa.presence]);
    }

    // enforce makespan after last operation and minimal duration of operation (based on alternatives)
    for o in e.all_operations() {
        m.enforce(leq(o.end, e.makespan));
        let min_duration = pb.operation(o.job, o.op).min_duration();
        m.enforce(leq(o.start + min_duration, o.end));
    }

    // make sure we have exactly one alternative per operation
    for j in pb.jobs() {
        for op in e.operations_ids(j) {
            let operation = e.operation(j, op);

            let starts = e.alternatives(j, op).map(|alt| alt.start()).collect_vec();
            m.enforce(alternative(operation.start, starts));
            let ends = e.alternatives(j, op).map(|alt| alt.end()).collect_vec();
            m.enforce(alternative(operation.end, ends));
        }
    }

    // for each machine, impose that any two alternatives do not overlap
    for machine in 0..(pb.num_machines) {
        m.enforce(no_overlap(
            e.alternatives_on_machine(machine)
                .map(|a| Interval::new_fixed_duration(a.start, a.duration)),
        ));
    }

    let mut transports = Vec::new();

    // enforce total order between tasks of the same job
    for j in pb.jobs() {
        let ops = e.operations_ids(j).collect_vec();

        for i in 1..ops.len() {
            let op1 = ops[i - 1];
            let op2 = ops[i];

            let o1 = e.operation(j, op1);
            let o2 = e.operation(j, op2);
            m.enforce(leq(o1.end, o2.start));

            // add transportation time between machines.
            // These are machine-dependent and thus placed between any pair of alternatives
            for a1 in e.alternatives(j, op1) {
                for a2 in e.alternatives(j, op2) {
                    if let Some(transport_time) = pb.transport_time(a1.machine, a2.machine)
                    // && transport_time > 0
                    {
                        let both_present = m.conjunctive_scope(&[a1.presence, a2.presence]);
                        let transport_start =
                            m.new_optional_var(0, pb.makespan_upper_bound(), both_present);
                        let transport_end = transport_start + transport_time;
                        let transport = Transport {
                            from_machine: a1.machine,
                            to_machine: a2.machine,
                            previous: a1.id,
                            next: a2.id,
                            presence: both_present,
                            start: transport_start.into(),
                            duration: transport_time as IntCst,
                            alternatives: Default::default(),
                        };
                        transports.push(transport);
                        m.enforce_scoped(leq(a1.end(), transport_start), [both_present]);
                        m.enforce_scoped(leq(transport_end, a2.start), [both_present]);
                    }
                }
            }
        }
    }
    let num_robots = pb.num_robots;
    let mut transports_of_robots: Vec<Vec<TransportWithRobot>> = vec![Vec::new(); num_robots];

    for mut t in transports {
        for r in 0..num_robots {
            let present = m.new_presence_variable(t.presence, "").true_lit();
            let start = m.new_optional_variable(0, pb.makespan_upper_bound(), present);

            let transport_by_robot = TransportWithRobot {
                from_machine: t.from_machine,
                to_machine: t.to_machine,
                previous: t.previous,
                next: t.next,
                robot: r as u32,
                presence: present,
                start: start.into(),
                duration: t.duration,
            };
            transports_of_robots[r].push(transport_by_robot);
            t.alternatives.push(transport_by_robot);
        }
        // choose exactly one robot for the transport
        m.enforce(alternative(t.start, t.alternatives.iter().map(|a| a.start)));

        e.transports.push(t);
    }
    for r in 0..num_robots {
        // for this robots, ensure that for any two transport tasks, they are not overlapping and
        // there is sufficient delay between to allow the robot to move from the first's final machine to the second's start machine
        for (i, transport_i) in transports_of_robots[r].iter().enumerate() {
            for transport_j in &transports_of_robots[r][i + 1..] {
                m.enforce_scoped(
                    // one of those two must be satisfied
                    exclu_choice(
                        // start(ti) + dur(ti) + tt(mi, mj) <= start(tj)
                        leq(
                            transport_i.start
                                + transport_i.duration
                                + pb.transport_time(
                                    transport_i.to_machine,
                                    transport_j.from_machine,
                                )
                                .unwrap(),
                            transport_j.start,
                        ),
                        // start(tj) + dur(tj) + tt(mj, mi) <= start(ti)
                        leq(
                            transport_j.start
                                + transport_j.duration
                                + pb.transport_time(
                                    transport_j.to_machine,
                                    transport_i.from_machine,
                                )
                                .unwrap(),
                            transport_i.start,
                        ),
                    ),
                    [transport_i.presence, transport_j.presence],
                );
            }
        }
    }

    // redundant constraints that strengthen propagation between the transports and the alternative tasks that come before/after
    for alt in e.all_alternatives() {
        let incoming_transports = e
            .transports_with_robots_to(alt.id)
            .map(|transport| transport.start + transport.duration)
            .collect_vec();

        if !incoming_transports.is_empty() {
            let transport_end = m.new_optional_variable(0, upper_bound, alt.presence);
            m.enforce(alternative(transport_end, incoming_transports));
            m.enforce_scoped(leq(transport_end, alt.start), [alt.presence]);
        }
    }
    for alt in e.all_alternatives() {
        let outgoing_transports = e
            .transports_with_robots_from(alt.id)
            .map(|transport| transport.start)
            .collect_vec();

        if !outgoing_transports.is_empty() {
            let transport_start = m.new_optional_variable(0, upper_bound, alt.presence);
            m.enforce(alternative(transport_start, outgoing_transports));
            m.enforce_scoped(leq(alt.end(), transport_start), [alt.presence]);
        }
    }

    (m, e)
}
