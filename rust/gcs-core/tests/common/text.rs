//! A document's text read (`common::read`) and solved: where the spatial, set, region and rounds
//! tests start from.
use gcs_core::diagnose::{diagnose, DiagnoseOptions};
use gcs_core::program::Elaborated;
use gcs_core::solve::{solve, SolveOpts};

use super::read;

/// `src` read and solved, or the test fails saying why, with the text.
pub fn solved(src: &str) -> Elaborated {
    let mut e = read(src);
    let r = solve(&mut e.sketch, SolveOpts::default());
    assert!(r.success, "{}\n{src}", r.message);
    e
}

/// The freedom `src` leaves, read and solved.
pub fn dof(src: &str) -> i64 {
    let mut sk = super::solved(&read(src));
    diagnose(&mut sk, DiagnoseOptions::default()).dof
}
