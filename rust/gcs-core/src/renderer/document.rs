//! Adapt document entities, view placement and styles to the renderer's geometric interface.
use crate::model::Sketch;
use crate::plane::Basis;
use crate::solid::{self, ApproximationPolicy, PageFrame};
use super::{Renderer, RenderStats, Stroke, View};
use std::collections::BTreeMap;

/// The numeric inputs of the document's projected pictures, excluding screen resolution.
/// Unrelated sketch geometry must not invalidate a projection while it is dragged. Solid
/// extents and viewing/cutting planes matter even when they move no sketch parameter.
pub fn inputs(sk: &Sketch) -> Vec<f64> {
    if sk.derived.is_empty() { return Vec::new(); }
    let mut out = vec![sk.extent()]; // the visibility walk's tolerance
    let frame = |out: &mut Vec<f64>, p: Option<u32>| {
        let (basis, (c, s, o)) = view_frame(sk, p.map(|i| i as usize));
        out.extend(basis.u);
        out.extend(basis.v);
        out.extend(basis.o);
        out.extend([c, s, o.0, o.1]);
    };
    for d in &sk.derived {
        out.extend(solid::reads(sk, d.solid as usize, 0.0));
        frame(&mut out, d.plane);
        if let Some(at) = d.at { frame(&mut out, Some(at)); }
    }
    out
}

/// Whether the pictures are the same at every pixel length: every solid they project is swept, and
/// a swept solid is one surface (its field's, or its exact B-rep's mesh) whatever the unit asked.
/// A host keeping a picture through a zoom then has nothing to refine once the camera rests.
pub fn detail_free(sk: &Sketch) -> bool {
    sk.derived.iter().all(|d| sk.is_swept(d.solid as usize))
}

/// Project a document solid in one of its named planes.
pub fn view(sk: &Sketch, si: usize, plane_i: Option<usize>, unit: f64) -> Vec<Stroke> {
    render(sk, si, plane_i, unit, None)
}

/// Cut a document solid at `at` and project it in `plane_i`.
pub fn section(sk: &Sketch, si: usize, at: Option<usize>, plane_i: Option<usize>, unit: f64) -> Vec<Stroke> {
    render(sk, si, plane_i, unit, Some(view_frame(sk, at).0))
}

fn render(sk: &Sketch, si: usize, plane_i: Option<usize>, unit: f64, section: Option<Basis>) -> Vec<Stroke> {
    let Ok(solid) = sk.evaluated_solid(si, ApproximationPolicy::from_unit(unit)) else { return Vec::new() };
    let (basis, pose) = view_frame(sk, plane_i);
    Renderer::prepare(&solid).project(View { frame: PageFrame::new(basis, pose), section }).strokes
}

/// The plane a picture is drawn in, and the pose its coordinates are read in — a plane's own,
/// so the identity.  `None` is the front plane, what a 2D sketch is read on.
pub(crate) fn view_frame(sk: &Sketch, plane_i: Option<usize>) -> (Basis, (f64, f64, (f64, f64))) {
    let basis = plane_i.filter(|&i| i < sk.planes.len()).map_or(Basis::page(), |i| sk.basis(i));
    (basis, crate::plane::IDENTITY_POSE)
}

/// **Every picture the document asked for, laid out.**  The one entry both front ends read, so
/// the SVG export and the canvas are one picture of one drawing and not two — `callout::layout`'s
/// bargain, and the reason `paint.ts` owns no 3D arithmetic.
///
/// The classes are the core's: `.visible`, `.hidden`, `.section` under whatever the statement
/// itself carries, so a sheet says what a hidden line looks like the way it says what a
/// dimension does, and a document that already writes `style .hidden` gets it for free.
pub fn layout(sk: &Sketch, unit: f64) -> Vec<Drawn> {
    layout_with_stats(sk, unit).0
}

/// Render all document views, sharing each solid's prepared visibility index, with work counts.
pub fn layout_with_stats(sk: &Sketch, unit: f64) -> (Vec<Drawn>, RenderStats) {
    let mut solids = BTreeMap::new();
    for d in &sk.derived {
        solids.entry(d.solid).or_insert_with(|| {
            sk.evaluated_solid(d.solid as usize, ApproximationPolicy::from_unit(unit))
        });
    }
    let renderers: BTreeMap<_, _> = solids.iter().filter_map(|(&i, s)| {
        s.as_ref().ok().map(|s| (i, Renderer::prepare(s)))
    }).collect();
    let mut out = Vec::new();
    for (i, d) in sk.derived.iter().enumerate() {
        let Some(renderer) = renderers.get(&d.solid) else { continue };
        let (basis, pose) = view_frame(sk, d.plane.map(|i| i as usize));
        let strokes = renderer.project(View {
            frame: PageFrame::new(basis, pose),
            section: d.at.map(|i| view_frame(sk, Some(i as usize)).0),
        }).strokes;
        for s in strokes {
            let mut class = crate::style::Classes(vec![
                if s.hidden { "hidden".to_string() } else { "visible".to_string() },
            ]);
            if d.at.is_some() && !s.hidden {
                class.0.push("section".to_string());
            }
            class.0.extend(d.class.0.iter().cloned());
            out.push(Drawn {
                of: i,
                solid: sk.solids.get(d.solid as usize).map(|x| x.name.clone()).unwrap_or_default(),
                path: s.path,
                hidden: s.hidden,
                silhouette: s.silhouette,
                style: crate::style::resolve(&sk.sheet, &class),
                pts: s.pts,
            });
        }
    }
    let mut stats = RenderStats::default();
    for renderer in renderers.values() {
        let s = renderer.stats();
        stats.visibility_rays += s.visibility_rays;
        stats.boundary_candidates += s.boundary_candidates;
        stats.boundary_exhaustive += s.boundary_exhaustive;
        stats.crossing_candidates += s.crossing_candidates;
        stats.crossing_exhaustive += s.crossing_exhaustive;
    }
    (out, stats)
}

/// One polyline of a derived picture, resolved: page coordinates and the ink to stroke it in.
#[derive(Clone, Debug)]
pub struct Drawn {
    /// Which `view`/`section` statement asked for it.
    pub of: usize,
    pub solid: String,
    pub path: String,
    pub hidden: bool,
    pub silhouette: bool,
    pub style: crate::style::Style,
    pub pts: Vec<(f64, f64)>,
}
