//! Project evaluated solids into vector drawings, independently of the sketch editor.
//!
//! [`Renderer::prepare`] retains access to a solid's cached edges and immutable geometry.
//! Reuse it for multiple [`View`]s. [`Drawing`] carries page coordinates and source paths;
//! document styling and serialization are adapters, shared by canvas and SVG consumers.
//! The current implementation uses polygonal geometry at the evaluated solid's tolerance.
//! Preparation, visibility and projection are separate seams for profiling and replacement.

pub(crate) mod document;
mod dimensions;
mod projection;
mod visibility;

pub use dimensions::{generated, Dim};
pub use document::{inputs, layout, section, view, Drawn};
use crate::{csg::Edge, plane::Basis, solid::{EvaluatedSolid, LocalPoint, PageFrame}};

/// One projection and its placement on the page. `section` is a world-space cutting plane;
/// the section retains material behind its origin along the view normal.
#[derive(Clone, Copy, Debug)]
pub struct View {
    pub frame: PageFrame,
    pub section: Option<Basis>,
}

/// One stroke of a derived picture, in page coordinates.
#[derive(Clone, Debug, PartialEq)]
pub struct Stroke {
    pub pts: Vec<(f64, f64)>,
    /// True where the solid stands between this edge and the eye.
    pub hidden: bool,
    /// A silhouette rather than a corner — a cylinder's side, drawn where the surface turns
    /// away.  Kept apart because it is *not* a fact about the object, only about this view.
    pub silhouette: bool,
    /// `body.bore.wall` — which face of which solid it bounds.
    pub path: String,
}

/// A vector drawing with tight bounds, or no bounds when it contains no strokes.
#[derive(Clone, Debug, PartialEq)]
pub struct Drawing {
    pub strokes: Vec<Stroke>,
    pub bounds: Option<[f64; 4]>,
}
impl Drawing {
    fn new(strokes: Vec<Stroke>) -> Self {
        let bounds = bounds_of(strokes.iter().flat_map(|s| s.pts.iter().copied()));
        Self { strokes, bounds }
    }
}

/// A prepared renderer borrows one immutable evaluated solid. The solid owns the shared
/// geometry cache; changing a document produces a new snapshot, never mutating this one.
/// All view-dependent work lives here, and no editor or document is needed to project it.
pub struct Renderer<'a> {
    solid: &'a EvaluatedSolid,
    edges: &'a [Edge],
}
impl<'a> Renderer<'a> {
    pub fn prepare(solid: &'a EvaluatedSolid) -> Self {
        Self { solid, edges: solid.edges() }
    }

    pub fn project(&self, view: View) -> Drawing {
        projection::project(self, view)
    }

    /// Conservative bounds without visibility or edge-crossing work. A section can only
    /// shrink these bounds; callers needing tight bounds use the completed Drawing instead.
    pub fn bounds(&self, frame: PageFrame) -> Option<[f64; 4]> {
        let b = self.solid.bounds();
        if b.is_empty() { return None }
        bounds_of((0..8).map(|i| {
            let p = LocalPoint(std::array::from_fn(|k| if i & (1 << k) == 0 { b.lo[k] } else { b.hi[k] }));
            self.solid.to_page(p, frame).0
        }))
    }
}

fn bounds_of(points: impl Iterator<Item = (f64, f64)>) -> Option<[f64; 4]> {
    points.fold(None, |bounds, (x, y)| Some(match bounds {
        None => [x, y, x, y],
        Some([a, b, c, d]) => [a.min(x), b.min(y), c.max(x), d.max(y)],
    }))
}
