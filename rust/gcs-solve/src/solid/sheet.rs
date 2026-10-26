//! **A developable panel's band as a solid** (#645): `solid(skin)` over `skin := { q | q
//! distance(<= 6mm, side: outside) panel }`, the panel a set the core recognises as
//! `std.panels.Developable`'s (`RegionShape::Sheet`).  A distance from a sheet is measured along
//! its normal from a foot on it, and past its edges there is no foot, so the band is square there:
//! each station is the planar quad between a ruling and the ruling moved along the normal, and
//! neighbouring stations are joined — built exactly from the panel's rulings, as the lowering
//! table builds a round, never read point by point.
//!
//! Its faces are its two skins, `inside` and `outside` (the panel's sides), its edges along each
//! rail, `a` and `b`, and — an open panel — its end rulings, `start` and `end`.  A band whose rails
//! both come round closes on itself and has no ends.
use super::*;
use crate::developable::{Developable, Ruling};
use crate::model::{RegionShape, RegionTerm};
use crate::space::{add, norm, scale, sub};
use std::rc::Rc;

type V = [f64; 3];

/// The faces of a band, by the index a facet carries.
pub const FACES: [&str; 6] = ["inside", "outside", "a", "b", "start", "end"];

/// The most stations a band is faceted at.
const MAX_STATIONS: usize = 8192;

/// **A band, built**: the panel, the offsets along its normal it runs between (`lo < hi`), and
/// the rulings it is faceted at.
#[derive(Debug)]
pub struct Band {
    pub panel: Developable,
    pub lo: f64,
    pub hi: f64,
    pub stations: Vec<Ruling>,
}

/// The sheet term of a region solid's terms, where its terms are one: a band and nothing else.
pub fn sheet_term(terms: &[RegionTerm]) -> Option<&RegionTerm> {
    match terms {
        [t] if matches!(t.shape, RegionShape::Sheet { .. }) => Some(t),
        _ => None,
    }
}

/// Whether region solid `si` is a band.
pub fn is_band(sk: &Sketch, si: usize) -> bool {
    matches!(sk.solids.get(si).map(|s| &s.def),
             Some(SolidDef::Region { terms, .. }) if sheet_term(terms).is_some())
}

/// Region solid `si`'s band faceted to `unit`, where it is one — remembered against what it
/// reads (`Caches::sheet`) — or why it cannot be built; `None` where it is no band.
pub fn band(sk: &Sketch, si: usize, unit: f64) -> Option<Result<Rc<Band>, String>> {
    if !is_band(sk, si) {
        return None;
    }
    let unit = unit.min(REPORT_UNIT);
    Some(sk.caches.sheet.get_or(si, MemoKey::at(sk, si, unit), || built(sk, si, unit).map(Rc::new)))
}

fn built(sk: &Sketch, si: usize, unit: f64) -> Result<Band, String> {
    let s = &sk.solids[si];
    let SolidDef::Region { terms, .. } = &s.def else { return Err("no region".into()) };
    let t = sheet_term(terms).ok_or("no band")?;
    let RegionShape::Sheet { rails, side } = &t.shape else { return Err("no band".into()) };
    let d = t.lo.value(sk)?;
    if !(d > 0.0) {
        return Err("a band is a distance more than nothing from its sheet".into());
    }
    // the band from the panel to `d` the way the bound reads it
    let (lo, hi) = if *side >= 0.0 { (0.0, d) } else { (-d, 0.0) };
    let panel = crate::model::developed::built(sk, rails)?;
    let tol = crate::curve::flatness(unit);
    let mut stations = panel.stations(tol);
    if stations.len() < 2 {
        return Err("the panel has too few rulings to make a band".into());
    }
    if stations.len() > MAX_STATIONS {
        let step = stations.len().div_ceil(MAX_STATIONS);
        let last = stations[stations.len() - 1];
        stations = stations.into_iter().step_by(step).collect();
        stations.push(last);
    }
    Ok(Band { panel, lo, hi, stations })
}

impl Band {
    /// The band as facets about `origin`, its faces `FACES`: each step between neighbouring
    /// stations two skins and two edges, and an open panel's end rulings capped.  Oriented so its
    /// volume is positive.
    pub(crate) fn primitive(&self, origin: V, name: &str) -> Prim {
        let near = 1e-12 * self.stations.iter().map(|r| norm(sub(r.n, r.m))).fold(1.0, f64::max);
        let mut facets: Vec<Facet> = Vec::new();
        let mut push = |pts: Vec<V>, face: usize, smooth: bool| {
            let mut pts: Vec<V> = pts.into_iter().map(|p| sub(p, origin)).collect();
            pts.dedup_by(|a, b| norm(sub(*a, *b)) <= near);
            while pts.len() > 1 && norm(sub(pts[0], pts[pts.len() - 1])) <= near {
                pts.pop();
            }
            if pts.len() < 3 {
                return;
            }
            // a skew quad in two triangles: the band's skins are ruled, flat along each ruling
            let pieces: Vec<Vec<V>> = if pts.len() == 4 {
                vec![vec![pts[0], pts[1], pts[2]], vec![pts[0], pts[2], pts[3]]]
            } else {
                vec![pts]
            };
            for p in pieces {
                if let Some(n) = facet_normal(&p) {
                    facets.push(Facet { pts: p, n, face, smooth });
                }
            }
        };
        let at = |r: &Ruling, s: f64| -> [V; 2] {
            [add(r.m, scale(r.normal, s)), add(r.n, scale(r.normal, s))]
        };
        let n = self.stations.len();
        let steps = if self.panel.closed() { n } else { n - 1 };
        for k in 0..steps {
            let (p, q) = (&self.stations[k], &self.stations[(k + 1) % n]);
            let ([p0, p1], [p3, p2]) = (at(p, self.lo), at(p, self.hi));
            let ([q0, q1], [q3, q2]) = (at(q, self.lo), at(q, self.hi));
            push(vec![p0, p1, q1, q0], 0, true);
            push(vec![p3, q3, q2, p2], 1, true);
            push(vec![p0, q0, q3, p3], 2, true);
            push(vec![p1, p2, q2, q1], 3, true);
        }
        if !self.panel.closed() {
            for (r, face, rev) in [(&self.stations[0], 4, false), (&self.stations[n - 1], 5, true)] {
                let ([a0, a1], [a3, a2]) = (at(r, self.lo), at(r, self.hi));
                let mut pts = vec![a0, a3, a2, a1];
                if rev {
                    pts.reverse();
                }
                push(pts, face, false);
            }
        }
        // the orientation that makes the volume positive is outward
        let volume: f64 = facets.iter().map(|f| {
            let p = &f.pts;
            (1..p.len() - 1).map(|i| crate::space::dot(p[0], crate::space::cross(p[i], p[i + 1])))
                .sum::<f64>()
        }).sum();
        if volume < 0.0 {
            for f in &mut facets {
                f.pts.reverse();
                f.n = scale(f.n, -1.0);
            }
        }
        let faces = FACES.iter().map(|s| s.to_string()).collect();
        super::primitive::finish(facets, faces, name)
    }
}
