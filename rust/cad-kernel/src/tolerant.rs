//! **Tolerant topology** (#735 A3; tolerant modelling, Jackson 1995): one model of when two
//! features are the same. Every vertex and edge carries the tolerance it was built to — the
//! farthest any point merged into a vertex stood from it, the largest gap measured between an
//! edge's curve, its pcurves' images and its vertices (`topo::Edge::tol`) — and features within
//! each other's tolerance are merged in one place, by one rule, at one time: the rules below, which
//! an arrangement (`boolean`) applies phase by phase, faces sewn (`sew`) in the same order, and
//! nothing else.
//!
//! **The rules**, each against the arrangement's tolerance `tol` (a length) or a feature's own
//! where that is larger:
//! - **a point onto a vertex** (`Pool::at`): within `tol` of it, or of any point already merged
//!   there — so a vertex is where its first point stood, and its tolerance how far the others were;
//! - **a vertex onto an edge** (`on_stretch`): within `8 tol` of the edge's curve inside its stretch
//!   and clear of its ends by as much: the edge is cut there;
//! - **a point onto a meeting** (`on_meeting`): within `8 tol / sin θ` of the curve two surfaces
//!   meet in at an angle `θ` — a point within `tol` of both stands that far off their meeting;
//! - **a root met flatly onto a crossing** (`FLAT`): a root of an edge's distance from a surface
//!   known only as the stretch within `16 tol` of it is the crossing found exactly inside that
//!   stretch;
//! - **an edge onto an edge** (`same_edge`): two pieces between the same vertices, the middle of one
//!   within `8 tol` of the other's curve inside its stretch, are one edge;
//! - **a sliver into its edge**: a piece of a face with no inside — no point of it read inside its
//!   outline drawn as fine as it goes — bounds nothing, and goes;
//! - **a crossing onto a shallow meeting**: two meetings on a face are crossed where they come
//!   within the reach of the shallower (`on_meeting` at the point), and where two of the three
//!   surfaces touch there, at the third's root along their curve or the vertex already in the
//!   stretch the two tangent meetings stay within reach of each other (`boolean`'s `touch_point`,
//!   `crossing_in_touch`); an edge touching a surface its own face touches, at its crossing with
//!   their touch curve (`touch_root`).
//!
//! Each is applied once, in that order, so the same two features are never merged by one rule in
//! one call and parted by another in the next.
use super::geom::{around,Curve,V};
use crate::util::space::{distance,dot,Grid};

/// A vertex onto a curve, and an edge onto an edge: within this many tolerances.
pub const ON_CURVE: f64 = 8.;

/// A root met flatly is the crossing within this many tolerances of the surface along its edge.
pub const FLAT: f64 = 16.;

/// The least sine of the angle two surfaces meet at that `on_meeting` widens its reach for: below
/// it, a point near both is read no farther than at it.
pub const LEAST_SINE: f64 = 1e-4;

/// How far off the curve two surfaces meet in, at an angle whose sine is `sine`, a point within
/// `tol` of both may stand, and still be a point of it.
pub fn on_meeting(tol: f64,sine: f64) -> f64 { ON_CURVE*tol/sine.max(LEAST_SINE) }

/// **A vertex onto an edge** (the second rule): the parameter at which the point `p` stands on the
/// stretch `t` of `c` (a closed curve's read from the stretch's start), where it stands within
/// `ON_CURVE tol` of the curve inside the stretch and no nearer either end than that; the edge is
/// cut there.
pub fn on_stretch(c: &Curve,t: [f64;2],p: V,tol: f64) -> Option<f64> {
    let mut s = c.inverse(p);
    if let Some(period) = c.period() { s = around(s,t[0],period); }
    // a vertex at the edge's end in all but the pool's labelling is its end
    let near = |q: V| distance(q,p) <= ON_CURVE*tol;
    (s > t[0] && s < t[1] && near(c.point(s)) && !near(c.point(t[0])) && !near(c.point(t[1]))).then_some(s)
}

/// **An edge onto an edge** (the fifth rule): the piece of `a` over its stretch `ta`, between the
/// vertices `va`, is the piece of `b` over `tb` between the same vertices where the middle of `a`
/// stands within `ON_CURVE tol` of `b`'s curve inside its stretch; and then whether they run the
/// same way round — the same first vertex, or (closed) tangents agreeing there.
pub fn same_edge(a: &Curve,ta: [f64;2],va: [u32;2],b: &Curve,tb: [f64;2],vb: [u32;2],tol: f64) -> Option<bool> {
    let ma = 0.5*(ta[0]+ta[1]);
    let mid = a.point(ma);
    let mut t = b.inverse(mid);
    if let Some(period) = b.period() { t = around(t,tb[0],period); }
    if t < tb[0]-1e-9 || t > tb[1]+1e-9 || distance(b.point(t),mid) > ON_CURVE*tol { return None }
    Some(if va[0] != va[1] { va[0] == vb[0] } else { dot(a.tangent(ma),b.tangent(t)) > 0. })
}

/// **Points merged where they come within the tolerance of one another** (the first rule), each
/// vertex keeping how far the farthest point merged into it stood (its tolerance).
#[derive(Clone,Debug)]
pub struct Pool {
    pub pts: Vec<V>,
    /// Each vertex's tolerance: the farthest a point merged into it stood from it (0 alone).
    pub tols: Vec<f64>,
    pub tol: f64,
    grid: Grid,
}

impl Pool {
    pub fn new(tol: f64) -> Pool { Pool {pts:Vec::new(),tols:Vec::new(),tol,grid:Grid::new(4.*tol)} }
    /// The vertex `p` is: the first within the tolerance of it, else a new one there.
    pub fn at(&mut self,p: V) -> u32 {
        assert!(p.iter().all(|x| x.is_finite() && x.abs() < 1e12),"a vertex at {p:?}");
        let mut found = None;
        let pts = &self.pts;
        let tol = self.tol;
        self.grid.around(p,|i| if found.is_none() { let d = distance(pts[i as usize],p); if d <= tol { found = Some((i,d)) } });
        if let Some((i,d)) = found {
            let t = &mut self.tols[i as usize];
            *t = t.max(d);
            return i
        }
        self.pts.push(p);
        self.tols.push(0.);
        let i = (self.pts.len()-1) as u32;
        self.grid.insert(p,i);
        i
    }
    /// The vertices, each with its tolerance.
    pub fn vertices(&self) -> Vec<super::topo::Vertex> {
        self.pts.iter().zip(&self.tols).map(|(&p,&tol)| super::topo::Vertex {p,tol}).collect()
    }
}
