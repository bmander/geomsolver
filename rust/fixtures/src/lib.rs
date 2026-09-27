//! Test fixtures shared by the core's and the CLI's suites, written once: `read` elaborates a
//! document, `tools` and `motions` hold the Solvent snippets a small swept case is written from,
//! and `gear` rewrites and reads the spiral-bevel project at a design.  A dev-dependency only,
//! so none of it reaches a released artefact.
pub mod read;
pub mod tools;
pub mod motions;
pub mod gear;

pub use read::{elaborate,read,module,beside,read_beside,solid};

