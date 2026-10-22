//! **A scenario's picture** (#510): how the drawing moves under a scenario's supports and loads,
//! drawn on what was analysed.
//!
//! Each body a scenario reads — a frame of beams, a prism read in its plane, a solid read in
//! three dimensions — is a surface of triangles: its places at rest and each place's move under
//! the scenario, both in the world's axes and the document's units.  A frame is a tube along its
//! elements, each sampled along the cubic it bends as (its ends' moves and turns, Hermite's), so
//! the curve drawn between nodes is the analysis's; a region is its quadratic triangles cut in
//! four, stood through its depth with the same move at each level; a solid is its tetrahedra's
//! boundary, each six-node facet cut in four.  The display mesh is never what moves: what is
//! drawn deformed is the mesh that was solved.
//!
//! The picture says how far it should be drawn exaggerated (`Picture::scale`): the largest move
//! as `SHOWN` of the bodies' extent, rounded down to a 1–2–5 step, and never below true scale.
//! A host draws `rest + s·moved` for any `s` it likes; nothing here depends on it.

use crate::model::{Output, Sketch};
use crate::solid::Box3;
use crate::space::norm;

type V = [f64; 3];

/// The largest move drawn, as a fraction of the bodies' extent, at the default factor.
pub const SHOWN: f64 = 0.1;

/// **One body of the picture**: what it is (`frame`, `region`, `solid`) and its name, its
/// vertices' places at rest and their moves, its triangles over them, and the edges of the mesh
/// that was solved (pairs of vertices), for a host to show the elements.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Body {
    pub kind: &'static str,
    pub name: String,
    /// The solid it is, where it is one (a region's or a solid's): what a host ghosts beneath it.
    pub solid: Option<usize>,
    pub rest: Vec<V>,
    pub moved: Vec<V>,
    pub triangles: Vec<[u32; 3]>,
    pub edges: Vec<[u32; 2]>,
}

impl Body {
    /// A vertex at rest at `p`, moving `u`: its index.
    pub(crate) fn vertex(&mut self, p: V, u: V) -> u32 {
        self.rest.push(p);
        self.moved.push(u);
        (self.rest.len() - 1) as u32
    }

    /// Its largest move, and where that vertex stands at rest.
    pub fn largest(&self) -> (f64, V) {
        self.moved.iter().zip(&self.rest).map(|(u, p)| (norm(*u), *p))
            .fold((0.0, [0.0; 3]), |a, b| if b.0 > a.0 { b } else { a })
    }

    /// Each element `[first, middle, last]` side of a quadratic triangle as the mesh's two edges.
    pub(crate) fn side_edges(&mut self, sides: impl IntoIterator<Item = [u32; 3]>) {
        let mut out: Vec<[u32; 2]> = Vec::new();
        for [a, m, b] in sides {
            for e in [[a, m], [m, b]] {
                out.push(if e[0] < e[1] { e } else { [e[1], e[0]] });
            }
        }
        out.sort();
        out.dedup();
        self.edges.extend(out);
    }

    /// A quadratic triangle `[a, b, c, ab, bc, ca]` (vertices) as four flat ones, turned as it is.
    pub(crate) fn quadratic(&mut self, n: [u32; 6]) {
        let [a, b, c, ab, bc, ca] = n;
        self.triangles.extend([[a, ab, ca], [ab, b, bc], [ca, bc, c], [ab, bc, ca]]);
    }
}

/// **A scenario drawn**: its bodies, why any it falls on is not drawn (each in the core's words),
/// the largest move over them and where it is, the extent of what is drawn, and the factor to
/// draw the moves at.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Picture {
    pub bodies: Vec<Body>,
    pub words: Vec<String>,
    pub largest: f64,
    pub largest_at: V,
    pub extent: f64,
    pub scale: f64,
}

impl Picture {
    /// The picture of these bodies, each drawn or why not.
    pub fn of(read: Vec<Result<Body, String>>) -> Picture {
        let mut p = Picture::default();
        let mut bounds = Box3::empty();
        for b in read {
            match b {
                Ok(b) => {
                    b.rest.iter().for_each(|&q| bounds.add(q));
                    let (m, at) = b.largest();
                    if m > p.largest {
                        (p.largest, p.largest_at) = (m, at);
                    }
                    p.bodies.push(b);
                }
                Err(why) => {
                    if !p.words.contains(&why) {
                        p.words.push(why);
                    }
                }
            }
        }
        p.extent = bounds.side();
        p.scale = factor(p.extent, p.largest);
        p
    }

    /// Each vertex's move as a fraction of the largest, body by body: what a host colours by.
    pub fn sizes(&self, body: usize) -> Vec<f64> {
        let b = &self.bodies[body];
        b.moved.iter().map(|u| if self.largest > 0.0 { norm(*u) / self.largest } else { 0.0 })
            .collect()
    }

    /// The largest move as a reader reads it, `0.412mm`.
    pub fn largest_text(&self, sk: &Sketch) -> String {
        crate::beams::value_text(sk, Output::Deflection, self.largest)
    }
}

/// **The factor a picture is drawn at**: the largest move `largest` as `SHOWN` of `extent`,
/// rounded down to 1, 2 or 5 times a power of ten, and never below true scale (a move already
/// that large, or nothing that moves, is drawn as it is).
pub fn factor(extent: f64, largest: f64) -> f64 {
    if !(largest > 0.0) || !(extent > 0.0) {
        return 1.0;
    }
    let s = SHOWN * extent / largest;
    if s <= 1.0 {
        return 1.0;
    }
    let decade = 10f64.powf(s.log10().floor());
    let step = [5.0, 2.0, 1.0].into_iter().find(|&k| k * decade <= s * (1.0 + 1e-12))
        .unwrap_or(1.0);
    (step * decade).max(1.0)
}

/// **The sides of a tube**: how many faces around a frame's element.
pub const SIDES: usize = 6;
/// A tube's radius, as a fraction of the frame's extent.
pub const TUBE: f64 = 0.006;
/// The places a frame's element is sampled at along its cubic, its ends among them.
pub const SAMPLES: usize = 5;

/// **A tube along a sampled element**: rings of `SIDES` about each of `centre`'s places, square
/// to the element's axes `e2`, `e3`, each ring moving as its centre does.  Its centre line's
/// places are the mesh's edges.
pub(crate) fn tube(body: &mut Body, centre: &[(V, V)], e2: V, e3: V, radius: f64) {
    let first = body.rest.len() as u32;
    let around = |k: usize| {
        let a = std::f64::consts::TAU * k as f64 / SIDES as f64;
        let (c, s) = (a.cos(), a.sin());
        [0, 1, 2].map(|i| radius * (c * e2[i] + s * e3[i]))
    };
    for &(p, u) in centre {
        for k in 0..SIDES {
            let o = around(k);
            body.vertex([p[0] + o[0], p[1] + o[1], p[2] + o[2]], u);
        }
    }
    let at = |ring: usize, k: usize| first + (ring * SIDES + k % SIDES) as u32;
    for ring in 0..centre.len().saturating_sub(1) {
        for k in 0..SIDES {
            let (a, b, c, d) = (at(ring, k), at(ring, k + 1), at(ring + 1, k + 1), at(ring + 1, k));
            body.triangles.extend([[a, b, c], [a, c, d]]);
        }
    }
    // the element's chord, ring to ring through one side: where its nodes are
    if let Some(last) = centre.len().checked_sub(1).filter(|&l| l > 0) {
        body.edges.push([at(0, 0), at(last, 0)]);
    }
}

/// **Hermite's cubic on `[0, 1]`** at `t`: the weights of an end's value and its slope (times the
/// length) at each end.
pub(crate) fn hermite(t: f64) -> [f64; 4] {
    let (t2, t3) = (t * t, t * t * t);
    [1.0 - 3.0 * t2 + 2.0 * t3, t - 2.0 * t2 + t3, 3.0 * t2 - 2.0 * t3, t3 - t2]
}
