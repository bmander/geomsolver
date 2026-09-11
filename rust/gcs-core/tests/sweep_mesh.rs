//! The certified swept boundary: its test foundation. `harness` reads the
//! fixtures, `tools` and `motions` hold the Solvent snippets every case is
//! written from, and each remaining module is a stage of the construction
//! judged against independent truth (closed forms, sampled membership, and
//! the field itself, in that order of trust).
mod harness;
mod tools;
mod motions;
mod labels;
mod certificate;
