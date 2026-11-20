//! **The inside of a closed shell** (#953): `hull := solid({ p | p in shell.side })`, `shell :=
//! { q | q in bottom or q in port or … }` — the points a shell of developable panels and faces of
//! the drawing encloses (`RegionShape::Enclosed`).  Built exactly to a stated tolerance, never read
//! point by point: each face of the drawing its planar face, each panel the sheet through its
//! rulings at the stations that draw it to that tolerance (`Developable::stations`, as a band's
//! are), every piece named by the set it is in, and all sewn where their boundaries are one curve
//! into the solid they bound (the kernel's `sewn` node, `sew::sewn`).  A shell that does not close
//! is refused by the kernel, the edge that bounds one face alone said.

use super::*;
use crate::developable::Ruling;
use crate::model::{Member, RegionShape};
use crate::brep::recipe::Writer;
use crate::json::{object, Json};
use crate::space::{norm, sub};
use std::rc::Rc;

/// The most stations a panel of a shell is drawn at.
const MAX_STATIONS: usize = 2048;

/// **A closed shell, read**: its pieces in the order its set names them, and the tolerance its
/// panels are drawn to (model units).
#[derive(Debug)]
pub struct Shell {
    pieces: Vec<Piece>,
    tol: f64,
}

/// One piece of a shell: a face of the drawing, or a panel's rulings — each named.
#[derive(Debug)]
enum Piece {
    Face { face: u32, name: String },
    Panel { stations: Vec<Ruling>, name: String },
}

/// The members of region solid `si`'s shell, where its one term is the inside of one.
pub fn members(sk: &Sketch, si: usize) -> Option<&[Member]> {
    let Some(SolidDef::Region { terms, .. }) = sk.solids.get(si).map(|s| &s.def) else {
        return None;
    };
    match terms.as_slice() {
        [t] => match &t.shape {
            RegionShape::Enclosed { members } => Some(members),
            _ => None,
        },
        _ => None,
    }
}

/// **Region solid `si`'s shell read to `unit`**, where it is the inside of one — remembered
/// against what it reads (`Caches::shell`) — or why it cannot be: a panel that cannot be read; a
/// term beside it (the inside of a shell is a solid alone, kept or cut with others by the body
/// words); `None` where it is none.
pub fn built(sk: &Sketch, si: usize, unit: f64) -> Option<Result<Rc<Shell>, String>> {
    let members = members(sk, si)?;
    let unit = unit.min(REPORT_UNIT);
    let tol = crate::curve::flatness(unit);
    let read = || -> Result<Shell, String> {
        let mut pieces = Vec::with_capacity(members.len());
        for m in members {
            pieces.push(match m {
                Member::Face { face, name } => Piece::Face { face: *face, name: name.clone() },
                Member::Sheet { rails, name } => {
                    let panel = crate::model::developed::built(sk, rails)
                        .map_err(|e| format!("its panel `{name}`: {e}"))?;
                    let stations = super::sheet::stations(&panel, tol, MAX_STATIONS)
                        .ok_or_else(|| format!("its panel `{name}` has too few rulings to draw"))?;
                    Piece::Panel { stations, name: name.clone() }
                }
            });
        }
        Ok(Shell { pieces, tol })
    };
    Some(sk.caches.shell.get_or(si, MemoKey::at(sk, si, unit), || read().map(Rc::new)))
}

impl Shell {
    /// **The recipe's node, written by `w`**: `sewn`, each face its profile, each panel its runs
    /// of stations (`runs`), each the sheet of degree one each way through its rulings' ends —
    /// sewn to the tolerance its panels are drawn to.  A run is laid so that where it shrinks to a
    /// point it is a pole along `u`, as a cone's apex is, the one way the kernel reads a pole: a
    /// fan's stations run along `u` (its corner the row of one point), any other run's along `v`
    /// (an end where the rails meet the column of one point, `Developable::meeting`).
    pub fn node(&self, sk: &Sketch, w: &Writer) -> Result<Json, String> {
        let mm = |p: [f64; 3]| w.at(p.map(|x| x * w.scale));
        let clamped = |n: usize| {
            let mut k = vec![0.0];
            k.extend((0..n).map(|k| k as f64));
            k.push((n - 1) as f64);
            Json::Arr(k.into_iter().map(Json::from).collect())
        };
        let ends = || Json::Arr([0.0, 0.0, 1.0, 1.0].map(Json::from).to_vec());
        let mut pieces = Vec::with_capacity(self.pieces.len());
        for p in &self.pieces {
            match p {
                Piece::Face { face, name } => pieces.push(object([("kind", "face".into()),
                    ("profile", super::cad::profile(sk, *face as usize, w)?),
                    ("face", name.as_str().into())])),
                Piece::Panel { stations, name } => {
                    for (run, fan) in runs(stations) {
                        let n = run.len();
                        let (uknots, vknots, poles) = match fan {
                            // rows its stations, columns a ruling's two ends
                            true => (clamped(n), ends(), run.iter()
                                .map(|r| Json::Arr(vec![mm(r.m), mm(r.n)])).collect()),
                            // rows a ruling's two ends, columns its stations
                            false => (ends(), clamped(n), [|r: &Ruling| r.m, |r: &Ruling| r.n]
                                .map(|end| Json::Arr(run.iter().map(|r| mm(end(r))).collect()))
                                .to_vec()),
                        };
                        pieces.push(object([("kind", "net".into()), ("du", 1.0.into()),
                            ("dv", 1.0.into()), ("uknots", uknots), ("vknots", vknots),
                            ("poles", Json::Arr(poles)), ("face", name.as_str().into())]));
                    }
                }
            }
        }
        // a panel's rail drawn through its stations stands off the rail, and off a face or a
        // panel beside it drawn through other stations, by the chords' sag, `tol` at most
        let tol = (self.tol * w.scale).max(crate::brep::geom::tolerance(1.0));
        Ok(object([("kind", "sewn".into()), ("pieces", Json::Arr(pieces)), ("tol", tol.into())]))
    }

    /// **The triangles panel `name` is drawn with** (#953): each strip between neighbouring
    /// stations two — what a row's nodes on a held panel slide over (`interior::Sheet::Facets`),
    /// the sheet it is built of drawn through the same rulings.
    pub fn triangles(&self, name: &str) -> Vec<[f64; 9]> {
        let mut out = Vec::new();
        for p in &self.pieces {
            let Piece::Panel { stations, name: n } = p else { continue };
            if n != name {
                continue;
            }
            for w in stations.windows(2) {
                let (a, b) = (&w[0], &w[1]);
                for t in [[a.m, a.n, b.n], [a.m, b.n, b.m]] {
                    out.push([t[0][0], t[0][1], t[0][2], t[1][0], t[1][1], t[1][2], t[2][0],
                              t[2][1], t[2][2]]);
                }
            }
        }
        out
    }
}

/// **A panel's stations in runs**, each drawn as one sheet, neighbouring runs sharing their
/// station: a fan about a corner (`true`) — one end of every ruling the same point, the corner a
/// panel turns about where a rail breaks or ends — or the rulings between (`false`).  Drawn whole,
/// a fan's corner would be a stretch of the sheet's edge standing still, which no curve of the
/// neighbouring panel along that rail can be read back onto.
fn runs(stations: &[Ruling]) -> Vec<(&[Ruling], bool)> {
    let size = stations.iter().map(|r| norm(sub(r.n, r.m))).fold(1.0, f64::max);
    let same = |p: [f64; 3], q: [f64; 3]| norm(sub(p, q)) <= 1e-9 * size;
    let fan = |a: &Ruling, b: &Ruling| same(a.m, b.m) || same(a.n, b.n);
    let mut out = Vec::new();
    let mut start = 0;
    for k in 1..stations.len() {
        let step = fan(&stations[k - 1], &stations[k]);
        let next = stations.get(k + 1).map(|s| fan(&stations[k], s));
        if next != Some(step) {
            out.push((&stations[start..=k], step));
            start = k;
        }
    }
    out
}
