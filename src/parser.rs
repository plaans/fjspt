use aries_solver::core::u32_to_cst;
use itertools::Itertools;

use crate::problem::*;

fn is_comment(line: &str) -> bool {
    line.chars().any(|c| c == '#')
}

fn ints(input_line: &str) -> impl Iterator<Item = usize> + '_ {
    input_line.split_whitespace().map(|n| n.parse().unwrap())
}

/// an iterator over non commented lines
fn lines(input: &str) -> impl Iterator<Item = &str> + '_ {
    input.lines().filter(|l| !is_comment(l))
}

pub(crate) fn flexshop(input: &str) -> Problem {
    println!("{input}");
    let mut lines = input.lines();

    let x: Vec<&str> = lines.next().unwrap().split_whitespace().collect();
    let num_jobs = x[0].parse().unwrap();
    let num_machines = x[1].parse().unwrap();

    fn next(it: &mut impl Iterator<Item = usize>) -> u32 {
        it.next().unwrap() as u32
    }

    let mut operations = Vec::with_capacity((num_jobs * num_jobs) as usize);

    for job in 0..num_jobs {
        let line = lines.next().unwrap();
        let ints = &mut ints(line);
        let num_ops = next(ints);
        for op_id in 0..num_ops {
            let num_alts = next(ints);
            let mut alternatives = Vec::with_capacity(num_alts as usize);
            for _ in 0..num_alts {
                let machine = next(ints) - 1;
                let duration = u32_to_cst(next(ints));
                alternatives.push(Alt { machine, duration })
            }
            operations.push(Op {
                job,
                op_id,
                alternatives,
            })
        }
    }
    Problem {
        num_jobs,
        num_machines,
        operations,
        transport_times: None,
    }
}

pub(crate) fn transport_time(input: &str) -> Vec<Vec<u32>> {
    let mut tt = Vec::new();
    for line in lines(input) {
        let line = ints(line).map(|i| i as u32).collect_vec();
        tt.push(line);
    }
    tt
}
