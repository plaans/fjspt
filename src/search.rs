mod greedy;

use crate::problem::Encoding;
use crate::search::SearchStrategy::Custom;
use crate::search::greedy::EstBrancher;
use aries_solver::prelude::*;
use aries_solver::solver::search::Brancher;
use aries_solver::solver::search::combinators::{CombinatorExt, UntilFirstConflict};
use std::str::FromStr;

pub type Solver = aries_solver::prelude::Solver<String>;

/// Variants of the search strategy
#[derive(Eq, PartialEq, Debug, Clone)]
pub enum SearchStrategy {
    /// Default strategy, typically an optimization oriente solution-guidance qith LRB
    Default,
    /// Custom strategy, parsed from the command line arguments.
    Custom(String),
}
impl FromStr for SearchStrategy {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s {
            "default" => Ok(SearchStrategy::Default),
            _ => Ok(Custom(s.to_string())),
        }
    }
}

/// Builds a solver for the given strategy.
#[allow(unused)] // todo: remove once we have search strategies
pub fn get_solver(mut base_solver: Solver, strategy: &SearchStrategy, pb: &Encoding) -> Solver {
    // Bootstraping branching strategy: a greedy EST strategy to bootstrap the search
    let first_est: Brancher<String> =
        Box::new(UntilFirstConflict::new(Box::new(EstBrancher::new(pb))));
    // main brancher (after first conflict and as long binary vars are not set): Conflict baed search (LRB, ...)
    let main_brancher = aries_solver::solver::search::default_brancher();
    // let final_brancher = Lexical::with_min().clone_to_box();
    let brancher = first_est.and_then(main_brancher); //.and_then(final_brancher);
    base_solver.set_brancher_boxed(brancher);
    base_solver
}
