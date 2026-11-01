//! **What a sweep is built from, the kernel's** (#721 F2): the tool a solid is swept as, read as
//! its faces (`surface`, `tool`), and what the kernel builds of it along a trajectory — a constant
//! twist's characteristic walked (`constant_twist`) and its sheet (`helical`), a generating sweep's
//! contact times (`generating`), its cutter's sections (`cutter`) and their contacts traced
//! (`trace`), a ball's tube (`canal`), each sheet fitted (`sheet`). The solver reads the tool, the blank and the motion off
//! a drawing, lowers them to these and puts what the kernel refuses into words.
pub mod surface;
pub mod tool;
pub mod constant_twist;
pub mod sheet;
pub mod helical;
pub mod generating;
pub mod trace;
pub mod cutter;
pub mod canal;
pub mod planar;
pub mod region;
