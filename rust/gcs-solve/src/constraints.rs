//! Constraint types: entities → a local parameter tuple, constants and a kernel.
//!
//! A constraint is `(kind, args)` where `args` are the constructor arguments in `spec` order.
//! `spec` drives everything reflective — JSON I/O, the constraint list, value editing, the
//! toolbar applier, duplicate detection and the witness's dimension jitter — so a new type is
//! covered everywhere as soon as it declares one.
//!
//! Residual forms follow the program: distance uses |p−q|² − d² (no sqrt), parallel is a 2×2
//! determinant, angle a wrapped atan2 gap (directed, so it needs no chirality), tangency a
//! signed distance minus the radius with a chirality flag fixed at construction.

#[allow(unused_imports)]
use crate::fmath::Det;
use crate::expr::Free;
use crate::kernels::{self, Kernel, K};
use crate::model::{EntKind, EntRef, Sketch};
use crate::space::across;

mod arg;
mod columns;
pub use columns::curve_contact_consts;
mod compare;
mod constraint;
mod infer;
mod kind;
mod words;

pub use arg::*;
pub use compare::*;
pub use constraint::*;
pub use infer::*;
pub use kind::*;
pub use words::*;
