mod parser;
mod problem;
mod search;

use aries_solver::prelude::*;

use crate::problem::{Encoding, Problem};
use crate::search::{SearchStrategy, Solver};
use anyhow::Context;
use aries_bench_data::IntermediateResult;
use aries_solver::solver::{Exit, SearchLimit};
use std::path::Path;
use std::time::{Duration, Instant};
use structopt::StructOpt;

#[derive(Debug, StructOpt)]
#[structopt(name = "aries-scheduler")]
pub struct Opt {
    /// File containing the instance to solve.
    files: Vec<String>,
    /// Indicates a layout file, containing a matrix with the transportation times between all pairs of machines.
    #[structopt(long = "layout")]
    layout_file: Option<String>,
    #[structopt(long = "robots")]
    num_robots: usize,
    /// When set, the solver will fail with an exit code of 1 if the found solution does not have this makespan.
    #[structopt(long = "expected-makespan")]
    expected_makespan: Option<u32>,
    #[structopt(long = "lower-bound")]
    lower_bound: Option<u32>,
    #[structopt(long = "upper-bound")]
    upper_bound: Option<u32>,
    /// Search strategy to use
    #[structopt(long = "search", default_value = "default")]
    search: SearchStrategy,
    /// maximum runtime, in seconds.
    #[structopt(long = "timeout", short = "t")]
    timeout: Option<u32>,
    /// If set, a summary of the run will be saved in the indicated directory.
    /// This option is intended to ease the collection of benchmark results with `aries-bench`
    #[structopt(long = "report", short = "r")]
    report: Option<String>,
}

fn main() -> anyhow::Result<()> {
    // Terminate the process if a thread panics.
    // take_hook() returns the default hook in case when a custom one is not set
    let orig_hook = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |panic_info| {
        // invoke the default handler and exit the process
        orig_hook(panic_info);
        std::process::exit(1);
    }));
    // read command line arguments
    let opt = Opt::from_args();

    for file in &opt.files {
        solve(file, &opt)?;
    }
    Ok(())
}

fn solve(instance: &str, opt: &Opt) -> anyhow::Result<()> {
    let deadline = opt
        .timeout
        .map(|dur| SearchLimit::Deadline(Instant::now() + Duration::from_secs(dur as u64)))
        .unwrap_or(SearchLimit::None);
    let start_time = std::time::Instant::now();
    let filecontent = read_file(instance)?;
    let mut pb = parser::flexshop(&filecontent);
    if let Some(layout) = opt.layout_file.as_ref() {
        let file_content = read_file(layout)?;
        let transport_times = parser::transport_time(&file_content);
        pb.set_transport_times(transport_times);
    }
    pb.num_robots = opt.num_robots;
    // println!("{:?}", pb);

    let lower_bound = (opt.lower_bound.unwrap_or(0)).max(pb.makespan_lower_bound() as u32);
    println!("Initial lower bound: {lower_bound}");

    let (model, encoding) = problem::encode(&pb, lower_bound, opt.upper_bound);
    let makespan: Var = encoding.makespan;

    let solver = Solver::new(model);
    let mut solver = search::get_solver(solver, &opt.search, &encoding);
    let mut solution_history: Vec<IntermediateResult> = Default::default();

    let mut best = Option::None;
    let result = solver.minimize_with_callback(
        makespan,
        |obj, sol| {
            println!("New solution with makespan: {}", obj);
            solution_history.push(IntermediateResult {
                timestamp: start_time.elapsed(),
                objective: obj as i64,
            });
            best = Some(sol.clone());
        },
        deadline,
    );
    solver.print_stats();
    println!();

    let status = match result {
        Ok(Some((obj, solution))) => {
            let optimum = solution.value_of(makespan).unwrap();
            assert_eq!(obj, optimum); // sanity check
            println!("> OPTIMAL (cost: {optimum})");

            if let Some(expected) = opt.expected_makespan {
                assert_eq!(
                    optimum as u32, expected,
                    "The makespan found ({optimum}) is not the expected one ({expected})"
                );
            };
            println!(
                "XX\t{}\t{}\t{}",
                instance,
                optimum,
                start_time.elapsed().as_secs_f64()
            );
            aries_bench_data::SolveStatus::SolvedOpt
        }
        Ok(None) => {
            println!("> UNSATISFIABLE");
            assert!(opt.expected_makespan.is_none(), "Expected a valid solution");
            aries_bench_data::SolveStatus::SolvedUnsat
        }
        Err(Exit::Interrupted) => match best.as_ref() {
            Some(sol) => {
                let best_cost = sol.value_of(makespan).unwrap();
                println!("> TIMEOUT (best solution cost {best_cost})");
                aries_bench_data::SolveStatus::Timeout
            }
            None => {
                println!("> TIMEOUT (no solution found)");
                aries_bench_data::SolveStatus::Timeout
            }
        },
    };
    if let Some(solution) = best.as_ref() {
        // export the solution to file if specified
        print_solution(solution, &pb, &encoding);
    }

    // print solve statistics to file (useful for benchmarking)
    if let Some(report_dir) = opt.report.as_ref() {
        let mut problem = aries_bench_data::Problem {
            name: instance.to_string(),
            timeout: opt
                .timeout
                .map(|t| Duration::from_secs(t as u64))
                .unwrap_or(Duration::MAX),
            flags: Default::default(),
        };
        problem
            .flags
            .insert("num_robots".to_string(), opt.num_robots.to_string());
        if let Some(lb) = opt.lower_bound {
            problem.flags.insert("lb".to_string(), lb.to_string());
        }
        if let Some(ub) = opt.upper_bound {
            problem.flags.insert("ub".to_string(), ub.to_string());
        }
        if let Some(layout) = opt.layout_file.as_ref() {
            problem
                .flags
                .insert("layout".to_string(), layout.to_string());
        }

        let result = aries_bench_data::SolveResult {
            problem,
            status,
            runtime: start_time.elapsed(),
            objective_value: best.map(|sol| sol.value_of(makespan).unwrap() as i64),
            metrics: Default::default(),
            objective_history: solution_history,
        }
        .with_metric(
            aries_bench_data::SolverMetric::NumConflicts,
            solver.stats.num_conflicts as f64,
        )
        .with_metric(
            aries_bench_data::SolverMetric::NumDecisions,
            solver.stats.num_decisions as f64,
        )
        .with_metric(
            aries_bench_data::SolverMetric::NumDomUpdates,
            solver.stats.num_dom_updates as f64,
        );

        result.save_to_dir(report_dir)?;
    }

    println!("TOTAL RUNTIME: {:.6}", start_time.elapsed().as_secs_f64());
    Ok(())
}

/// Write the solution to file if the file is not None
fn print_solution(solution: &Solution, pb: &Problem, encoding: &Encoding) {
    for job in pb.jobs() {
        println!("Job: {job}");
        for op in encoding.operations_ids(job) {
            let alt = encoding
                .alternatives(job, op)
                .find(|alt| solution.entails(alt.presence))
                .unwrap();

            let incoming_transport = encoding
                .transports_with_robots_to(alt.id)
                .find(|t| solution.entails(t.presence));
            if let Some(in_transport) = incoming_transport {
                let start = solution.eval(in_transport.start).unwrap();
                let end = start + in_transport.duration;

                println!(
                    "  [{start}, {end}] transport({}, {} -> {}",
                    in_transport.robot, in_transport.from_machine, in_transport.to_machine
                );
            }
            let start = solution.eval(alt.start()).unwrap();
            let end = solution.eval(alt.end()).unwrap();
            println!(
                "  [{start}, {end}] process(j: {}, op: {}, alt: {}, m: {})",
                alt.id.job,
                alt.id.op,
                alt.id.alt.unwrap(),
                alt.machine
            );
            // println!("    robot ->: {:?}", incoming_transport.map(|t| t.robot));
            // println!("    Alt: {:?} (machine {})", alt.id, alt.machine);
        }
    }
}

fn read_file(file: impl AsRef<Path>) -> anyhow::Result<String> {
    std::fs::read_to_string(file.as_ref())
        .with_context(move || format!("Cannot read file: '{:?}'", file.as_ref()))
}
