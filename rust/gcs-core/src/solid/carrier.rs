//! **Where a face of a solid lies**, read off the solved drawing (#162 F5): the numeric twin of
//! the set `flatten::faces` makes of the face — a cap the profile's plane at its extent, a side
//! over a straight edge the plane through the edge along the profile's normal — for a gesture
//! to seed a plane on it, and to find one lying there already.

use crate::model::{EntKind, SolidDef, Sketch};
use crate::plane::Basis;
use crate::space::{add, scale, sub};

/// The surface a face lies on: a frame in it (`u`, `v` along it, `o` on it) and the face's
/// middle, where a sketch on it starts when nothing says where.
#[derive(Clone, Copy, Debug)]
pub struct Carrier {
    pub basis: Basis,
    pub middle: [f64; 3],
}

/// The carrier of the face `path` of object `solid` — the path the face is named by (`b0.near`,
/// `body.b0.far`, `b0.ab`), the object's own name first — or why it has none.
pub fn carrier(sk: &Sketch, solid: usize, path: &str) -> Result<Carrier, String> {
    let object = sk.solids.get(solid).ok_or("there is no such solid")?;
    let rest = path.strip_prefix(&format!("{}.", object.name))
        .ok_or_else(|| format!("`{path}` is no face of `{}`", object.name))?;
    let segs: Vec<&str> = rest.split('.').collect();
    let (face, through) = segs.split_last().ok_or_else(|| format!("`{path}` names no face"))?;
    // through bodies by their operands' names, and from a body to the stock it was swept as
    let mut at = solid;
    let mut steps = through.iter();
    loop {
        let s = &sk.solids[at];
        let SolidDef::Body { stock, on, through: cut, bound } = &s.def else { break };
        let leaf = |o: &&u32| sk.solids[**o as usize].name.rsplit('.').next();
        let operands = || std::iter::once(stock).chain(on).chain(cut).chain(bound);
        if sk.solids[*stock as usize].name == s.name {
            at = *stock as usize;
            continue;
        }
        let Some(seg) = steps.next() else {
            return Err(format!("`{path}` names no face: `{}` is a body", s.name));
        };
        at = *operands().find(|o| leaf(o) == Some(*seg))
            .ok_or_else(|| format!("`{path}` names no face: no `{seg}` in `{}`", s.name))? as usize;
    }
    if steps.next().is_some() {
        return Err(format!("`{path}` names no face"));
    }
    let SolidDef::Prism { face: fi, from, to } = &sk.solids[at].def else {
        return Err(format!("`{path}`: only a prism's faces are drawn on yet"));
    };
    let f = &sk.faces[*fi as usize];
    let unit = (sk.extent() / 256.0).max(super::REPORT_UNIT);
    let poly = super::face_poly(sk, *fi as usize, unit)
        .ok_or_else(|| format!("`{path}`: its profile is no face to draw on"))?;
    let profile = poly.basis;
    let n = profile.normal();
    let (lo, hi) = (from.value.min(to.value), from.value.max(to.value));
    match *face {
        "near" | "far" => {
            let d = if *face == "near" { hi } else { lo };
            let (cx, cy) = super::ring_centroid(&poly.pts);
            let middle = add(profile.lift(cx, cy), scale(n, d));
            Ok(Carrier { basis: Basis { o: add(profile.o, scale(n, d)), ..profile }, middle })
        }
        edge => {
            let k = f.edge_names.iter().position(|e| e == edge)
                .ok_or_else(|| format!("`{path}` names no face"))?;
            let e = f.edges[k];
            if e.kind != EntKind::Line {
                return Err(format!("`{path}` is a curved side: sketch on a flat one"));
            }
            let l = &sk.lines[e.i()];
            let (a, b) = (sk.world_point(l.p1 as usize), sk.world_point(l.p2 as usize));
            let u = crate::plane::unit(sub(b, a)).ok_or_else(|| format!("`{edge}` has no length"))?;
            let middle = add(scale(add(a, b), 0.5), scale(n, (lo + hi) / 2.0));
            Ok(Carrier { basis: Basis { u, v: n, o: a }, middle })
        }
    }
}
