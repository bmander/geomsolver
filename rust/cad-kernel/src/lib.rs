//! A boundary-representation kernel of our own (docs/rust-kernel-plan.md): exact analytic faces
//! and edges with their topology, built from the CAD recipe the native host is handed, measured
//! without meshing, and — as the plan's rungs land — combined, meshed and written as STEP.
//!
//! Its own crate, depending on nothing: the solver (`gcs-solve`) re-exports it as `brep`, and its
//! numerical ground (`util`: the deterministic elementary functions, vectors, intervals, roots,
//! the dense linear algebra, JSON) as the modules it always named them by.  Nothing here knows
//! the solver or Solvent; the compiler holds that line.
#![allow(clippy::needless_range_loop, clippy::too_many_arguments, clippy::type_complexity)]
pub mod util;
pub mod geom;
pub mod mesh;
pub mod nurbs;
pub mod topo;
pub mod boolean;
pub mod build;
pub mod props;
pub mod step;
pub mod step_check;
pub mod query;
pub mod ssi;
pub mod recipe;
pub mod json;
pub mod pattern;
pub mod planar;
pub mod corners;
pub mod plane2;
pub mod section;
pub mod fillet;
pub mod spill;
pub mod family;
pub mod trajectory;
pub mod contact;

use crate::util::json::Json;
use std::sync::OnceLock;

/// Three numbers of a recipe or a dump (`[x, y, z]`).
pub(crate) fn vec3(j: &Json) -> geom::V { let a = j.arr(); [a[0].as_f64(),a[1].as_f64(),a[2].as_f64()] }

/// `j`'s field `k`, or an error naming it under `what` (`recipe`, `profile`, `brep json`).
pub(crate) fn field<'a>(j: &'a Json,k: &str,what: &str) -> Result<&'a Json,String> {
    j.get(k).ok_or(format!("{what}: no `{k}`"))
}

/// What a kernel error is, where a caller acts on it rather than only saying it.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Kind {
    /// A boundary that is no manifold: an edge used as often along as against, more than once
    /// (`topo::Brep::check`), as where two pieces of material touch along it.
    NotManifold,
    /// A boundary that does not close: an edge used more often one way than the other, or by no
    /// face (`topo::Brep::check`), where a face is missing — never what two bodies touching leave, so
    /// the kernel's own failure.
    Open,
    /// A recipe of another kind than a route builds (`recipe::Recipe::meridian`,
    /// `section::Sectioned`): its caller builds it another way.
    Refused,
    /// A recipe this kernel cannot read (`recipe::Recipe::read`).
    Recipe,
    /// Anything else this kernel could not do.
    Failed,
}

/// An error of this kernel: what kind it is, and what it says. Said, it is its message alone, and
/// a caller wanting a `String` takes that (`From`).
#[derive(Clone,Debug,PartialEq)]
pub struct KernelError { pub kind: Kind,pub msg: String }

impl KernelError {
    pub fn new(kind: Kind,msg: impl Into<String>) -> KernelError { KernelError {kind,msg:msg.into()} }
    /// The same error saying `f` of what it said, its kind kept.
    pub fn saying(self,f: impl FnOnce(&str) -> String) -> KernelError { KernelError {msg:f(&self.msg),..self} }
}

impl std::fmt::Display for KernelError {
    fn fmt(&self,f: &mut std::fmt::Formatter) -> std::fmt::Result { f.write_str(&self.msg) }
}

impl std::error::Error for KernelError {}

impl From<String> for KernelError { fn from(msg: String) -> KernelError { KernelError {kind:Kind::Failed,msg} } }
impl From<&str> for KernelError { fn from(msg: &str) -> KernelError { KernelError {kind:Kind::Failed,msg:msg.into()} } }
impl From<KernelError> for String { fn from(e: KernelError) -> String { e.msg } }

/// `SOLVENT_BREP_DEBUG` is set: the kernel narrates its Booleans and traces (read once).
pub(crate) fn debug() -> bool {
    static ON: OnceLock<bool> = OnceLock::new();
    *ON.get_or_init(|| std::env::var_os("SOLVENT_BREP_DEBUG").is_some())
}

/// `SOLVENT_BREP_TIME` is set: the kernel times its slow Boolean steps (read once).
pub(crate) fn timing() -> bool {
    static ON: OnceLock<bool> = OnceLock::new();
    *ON.get_or_init(|| std::env::var_os("SOLVENT_BREP_TIME").is_some())
}
