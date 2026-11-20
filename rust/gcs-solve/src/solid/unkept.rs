//! **A union a body keeps none of** (#961 item 6): a body is its stock, plus what is in `union`
//! with it, less what cuts it, within what it is in `common` with — sets, so the order its
//! statements are written in is no order of operations.  A union written after a cut does not put
//! back what the cut takes, and one lying wholly in what the cuts take is not in the body at all:
//! the document is legal, builds, and has lost the material its statement drew.  That is said.
//!
//! **The condition is a containment, read exactly.**  `u union b` states that `u`'s material is
//! `b`'s; the body keeps none of it where no point of `u` is in `b`.  With `b` = (S ∪ U) ∖ C ∩ K
//! and `u` ⊆ U, `u` ∩ `b` = `u` ∖ C ∩ K: whatever the stock and the other unions hold, the body
//! keeps none of `u` exactly where `every q in u: q in C or q not in K` — every point of it in some
//! cut, or outside some common.  It is read off the arrangement of `u` with the cuts and commons
//! (`brep::boolean::arrangement`, `members`): the cells of `u`, each in which of them.  A cell is a
//! piece of space the faces bound, of positive volume — a piece with no inside (a face flush with
//! a cut's, a surface touching another along an edge) is no cell — so the answer is the set
//! statement's, decided by the kernel's classification and not a volume held to a tolerance a
//! sliver could fall under.  A face of `u` flush with a cut's wall keeps nothing beyond the wall:
//! a hub as wide as the bore it stands in is said, one a hair wider is not.
//!
//! **What is not said.**  A union partly taken (a hub bored through) keeps the rest; one the stock
//! or another union holds already keeps its material in the body as stated — redundant, not lost,
//! and so neither is said.  Nor a cut that takes nothing (one that misses the body): the body rule
//! is not what defeats it, and reading it would classify every cut of every body.
//!
//! **What it costs.**  Nothing for a body with no cut and no common, nor for a union whose box
//! meets no cut's where the body has no common (a boss away from every hole): it keeps all of
//! itself there.  Otherwise the union's vertices and a point of each of its faces placed against
//! the near cuts and the commons, one of them kept answering at once (a witness), and only where
//! none is, the arrangement of that union with the cuts whose boxes meet its own and the commons
//! — its terms built where the body stands (`exact::operands`), the body never.

use super::*;
use crate::brep::boolean::{arrangement, members, overlap};
use crate::brep::query::{Located, Place};
use crate::brep::recipe::Node;
use crate::brep::topo::Brep;
use std::collections::BTreeSet;

/// Union operand `union` of body `body` (solid indices), of which the body keeps nothing: every
/// point of it lies in one of `cuts` or outside one of `commons` — those, of the body's cuts and
/// commons, that take some of it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Unkept {
    pub body: usize,
    pub union: usize,
    pub cuts: Vec<usize>,
    pub commons: Vec<usize>,
}

/// The unions of body `body` it keeps none of (`Unkept`), in the order its unions are listed.
/// Empty for a solid that is no body, a body with no cut and no common, and one with swept
/// material, whose boundary is the field's.  Refused where its terms are not built exactly (the
/// body's own refusal is said where it is built): then nothing is known.
pub fn unkept(sk: &Sketch, body: usize) -> Result<Vec<Unkept>, String> {
    let SolidDef::Body { on, through, bound, .. } = &sk.solids[body].def else {
        return Ok(Vec::new());
    };
    // its written terms: what reads its faces (a round's fill and cut, the stratum) is none
    let readers = faces::readers_of(sk, body);
    let written = |of: &[u32]| of.iter().any(|o| !readers.contains(&(*o as usize)));
    if !written(on) || !written(through) && bound.is_empty() || cad::contains_sweep(sk, body) {
        return Ok(Vec::new());
    }
    let (recipe, built) = exact::operands(sk, body)?;
    let Some(Node::Body(terms)) = recipe.nodes.iter().find(|n| n.id == recipe.root).map(|n| &n.node)
    else { return Ok(Vec::new()) };
    // the terms the body is written with; what reads its faces (a round's fill and cut) acts on
    // what they make, after them (the stratum), and is no term written
    let brep = |id: &i64| built.get(id)
        .ok_or_else(|| "a term of the body is not built".to_string());
    let read = |ids: &[i64]| ids.iter().map(|id| Ok((*id, brep(id)?)))
        .collect::<Result<Vec<_>, String>>();
    let (unions, cuts, commons) = (read(&terms.on)?, read(&terms.cut)?, read(&terms.bound)?);
    let all = || unions.iter().chain(&cuts).chain(&commons);
    let size = all().fold(0.0f64, |m, (_, b)| m.max(b.size()));
    let tol = crate::brep::geom::tolerance_over(size, 0.0);
    let boxes: BTreeMap<i64, crate::brep::geom::Box3> =
        all().map(|(id, b)| (*id, b.bounds())).collect();
    let mut out = Vec::new();
    for &(u, ub) in &unions {
        // the cuts that can take any of it; every common can (one apart from it takes it all)
        let near: Vec<usize> = (0..cuts.len())
            .filter(|&k| overlap(&boxes[&u], &boxes[&cuts[k].0], tol)).collect();
        if near.is_empty() && commons.is_empty() {
            continue;
        }
        // **a witness** first: a point of its boundary that every near cut leaves outside and
        // every common holds inside, each read by exact point location — more than the tolerance
        // off their boundaries, so a ball about it is outside the cuts and inside the commons,
        // and the union's material within that ball is kept — is the answer with no arrangement
        // made (a hub bored through keeps its rim, whose vertices lie beyond the bore)
        let own = Located::new(ub, tol);
        let taking: Vec<Located> = near.iter().map(|&k| Located::new(cuts[k].1, tol)).collect();
        let keeping: Vec<Located> = commons.iter().map(|(_, b)| Located::new(b, tol)).collect();
        let faces = ub.faces.iter().enumerate().filter_map(|(fi, f)| {
            let [[u0, u1], [v0, v1]] = crate::brep::boolean::face_parameters(ub, fi);
            let p = f.surface.point([(u0 + u1) / 2.0, (v0 + v1) / 2.0]);
            (own.face_place(fi, p) == Place::In).then_some(p)
        });
        let witnessed = ub.vertices.iter().map(|v| v.p).chain(faces)
            .any(|p| taking.iter().all(|c| c.solid_place(p) == Place::Out)
                && keeping.iter().all(|k| k.solid_place(p) == Place::In));
        if witnessed {
            continue;
        }
        // it first, then the cuts, then the commons
        let mut operands: Vec<&Brep> = vec![ub];
        operands.extend(near.iter().map(|&k| cuts[k].1));
        operands.extend(commons.iter().map(|(_, b)| *b));
        let c = 1 + near.len();
        let complex = arrangement(&operands, tol)?;
        let inside = members(&complex, &operands, tol)?;
        let (mut by_cut, mut by_common) = (BTreeSet::new(), BTreeSet::new());
        let kept = inside.iter().skip(1).filter(|m| m[0]).any(|m| {
            let taken: Vec<usize> = (1..c).filter(|&k| m[k]).map(|k| near[k - 1]).collect();
            let outside: Vec<usize> = (c..m.len()).filter(|&k| !m[k]).map(|k| k - c).collect();
            let kept = taken.is_empty() && outside.is_empty();
            by_cut.extend(taken);
            by_common.extend(outside);
            kept
        });
        // a union of no cell holds nothing to keep or lose
        if kept || by_cut.is_empty() && by_common.is_empty() {
            continue;
        }
        out.push(Unkept {
            body,
            union: u as usize,
            cuts: by_cut.into_iter().map(|k| cuts[k].0 as usize).collect(),
            commons: by_common.into_iter().map(|k| commons[k].0 as usize).collect(),
        });
    }
    Ok(out)
}
