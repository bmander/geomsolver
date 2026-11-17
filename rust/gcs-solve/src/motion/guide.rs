//! **A frame along a path** (#398 C): a jointed frame whose joint is held along a path (`g
//! coincident f.axis`, `std.stations.Along`) stands where the path's parameter puts it, its pose at
//! `u` read off the path after the solve — the frame square to the path's tangent there, turned as
//! the path makes it from where the frame is drawn, its **home**.  The motion is the kernel's frame
//! carried along a curve (`brep::trajectory::Along`), by how much of the path's torsion the frame
//! turns with (the joint's ratio): **0, the rotation-minimising frame** (`std.stations.On`'s and
//! `std.stations.Minimal`'s), its axes square to the tangent turning only as the tangent does; **1,
//! the path's own frame** (`std.stations.Frenet`'s), turning with the torsion besides.
//!
//! The path is read here, the kernel's curve (`brep::trajectory::Guide`): its point and first three
//! derivatives in its own parameter (the third only for the path's own frame's rate) — a line's
//! fraction of its run, an arc's of its turn, a spline's knot parameter (in a plane, or over points
//! in space), a traced curve's formal (`crate::path`, the one reader, #645) — and bounds on its
//! first two derivatives, rigorous where they have one — a line, an arc, a polynomial spline's
//! hodographs — so its enclosures over a stretch (for `MotionBounds` and the speed bound) fail
//! closed for any other path, as a law does for a function it carries no enclosure of.
use crate::brep::trajectory::{Along, Guide as Curve};
use crate::brep::KernelError;
use crate::model::{EntRef, Sketch};
use crate::path::Path;
use crate::space::{norm, sub};

type V = [f64; 3];

impl Curve for Path {
    fn domain(&self) -> [f64; 2] {
        let (lo, hi) = Path::domain(self);
        [lo, hi]
    }
    fn jet(&self, u: f64, third: bool) -> Result<[V; 4], KernelError> {
        Ok(Path::jet(self, u, third)?)
    }
    fn derivative_bounds(&self) -> Option<[f64; 2]> {
        Path::derivative_bounds(self)
    }
}

/// **A frame along a path, read** (`MotionDef::Guided`): the frame `frame` along the path `guide`,
/// the path's own (`frenet`) or the rotation-minimising one, its home where the frame's origin
/// stands on the path.
pub(crate) fn read(sk: &Sketch, name: &str, guide: EntRef, frame: u32, frenet: bool)
    -> Result<Along<Path>, String> {
    let path = Path::read(sk, guide).map_err(|m| format!("`{name}`: {m}"))?;
    let chart = sk.frames.get(frame as usize).and_then(|f| f.chart.as_ref())
        .ok_or_else(|| format!("`{name}` has no origin to stand on its path"))?;
    let o: V = chart.o.map(|q| sk.params[q as usize].value);
    let home = path.nearest(o)?;
    let gap = norm(sub(path.jet(home, false)?[0], o));
    if gap > 1e-6 * (1.0 + norm(o)) {
        return Err(format!("`{name}` is not drawn on its path: its origin stands {} from it \
                            — draw it at its home there, `std.stations.Along(f, g, up: …)`",
                           crate::vocab::num(gap)));
    }
    Ok(Along::from(path, home, frenet, format!("`{name}`"))?)
}
