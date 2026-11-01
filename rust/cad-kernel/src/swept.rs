//! **What a sweep is built from, the kernel's** (#721 F2): the tool a solid is swept as, read as
//! its faces (`surface`, `tool`), and what the kernel builds of it along a trajectory. The solver
//! reads the tool, the blank and the motion off a drawing, lowers them to these and puts what the
//! kernel refuses into words.
pub mod surface;
pub mod tool;
pub mod constant_twist;
pub mod sheet;
pub mod helical;
