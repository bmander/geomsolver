//! The certified swept boundary: its test foundation. `harness` reads the
//! fixtures, `tools` and `motions` hold the Solvent snippets every case is
//! written from, and each remaining module is a stage of the construction
//! judged against independent truth (closed forms, sampled membership, and
//! the field itself, in that order of trust).
mod harness;
mod tools;
mod motions;
mod forms;
mod cases;
mod labels;
mod certificate;
mod closed;
mod overlaps;
mod pieces;
mod creases;
mod grazing;
mod reference;
mod evidence;
mod status;
mod calibration;
mod audit_report;

mod cylinder_oracle;
mod cylinder_replay;
mod provenance;

mod cylinder_domains;
mod shared;
mod shared_intersections;

mod source_charts;
