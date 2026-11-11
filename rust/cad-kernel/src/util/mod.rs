//! What the kernel is computed with, and the solver too (it re-exports each by name): the
//! elementary functions the same bits on every target, vectors and boxes, roots, dense linear
//! algebra, quadrature, exact expansions and the filtered predicates over them, a JSON reader and
//! writer, a seeded generator, a clock and the thread pool.
pub mod bvh;
pub mod bypass;
pub mod clock;
pub mod expansion;
pub mod fmath;
pub mod interval;
pub mod isolate;
pub mod json;
pub mod linalg;
pub mod meter;
pub mod par;
pub mod predicates;
pub mod quadrature;
pub mod rng;
pub mod roots;
pub mod space;
pub mod union_find;
pub mod warm;
