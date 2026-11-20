//! **A union that adds a body nothing, its cuts taking what it drew** (#961 item 6): a body is
//! its stock, plus what is in `union` with it, less what cuts it, within what it is in `common`
//! with — sets, so the order its statements are written in is no order of operations.  A union
//! written after a cut does not put back what the cut takes: a hub lying in a bore, or a web
//! drawn across one with its ends in the wall it was to join, leaves the body as it would be
//! without the statement, and the document builds.  That is said.
//!
//! **The condition, read off the body rule.**  With `b` = (S ∪ U) ∖ C ∩ K and `u` one of U, the
//! body without `u` is (S ∪ U′) ∖ C ∩ K, so the statement `u union b` changes the body by
//! `u` ∖ (S ∪ U′) ∖ C ∩ K.  That alone does not tell a lost union from a redundant one: the
//! issue's hub lies inside its ring's stock, a disc the bore is cut from, so `u` ∖ S is empty
//! for it as for a boss drawn inside a plate and never cut.  What tells them apart is whether the
//! cuts take any of it.  So a union is said where both hold:
//!
//! - **(A) it adds nothing kept**: every point of `u` the body keeps (outside every cut, inside
//!   every common) is the body's by another term — the stock, or another union that adds the body
//!   material of its own;
//! - **(B) something of it is taken**: some point of `u` lies in a cut or outside a common.
//!
//! A union clear of everything else is (A) where the body keeps none of it: that case is said as
//! "keeps none of it".  A boss inside a plate and never cut fails (B), and a boss bored through
//! whose rim stands proud of the plate fails (A): neither is said.  **Two unions do not vouch for
//! one another**: in (A) a point is the body's by another union only where that union adds
//! material no other term holds (`Reader::adds`, read against the stock and every union, so it
//! does not depend on which unions are said) — two copies of one pad, partly cut, each keep what
//! the other holds, and neither is said, since without both the body would lose it; two hubs in
//! one bore, each keeping nothing, are each said.
//!
//! **Read exactly.**  Each answer is read off the arrangement of `u` with the terms whose boxes
//! meet its own (`brep::boolean::arrangement`, `members`): its cells, each in which terms.  A cell
//! is a piece of space the faces bound, of positive volume — a piece with no inside (a face flush
//! with a cut's or the stock's) is no cell — so it is the set statement decided by the kernel's
//! classification, not a volume held to a tolerance.  A hub as wide as its bore keeps none of
//! itself; one a micrometre wider keeps a skin of the ring the ring has already, and is said too.
//!
//! **What is not said.**  A cut that takes nothing (one that misses the body): the body rule is not
//! what defeats it, and reading it would classify every cut of every body.  A `common` leaving the
//! body nothing is the empty body, said by its export (#617).
//!
//! **What it costs.**  Nothing for a body with no cut and no common, nor for a union whose box
//! meets no cut's where the body has no common: (B) fails.  Otherwise the union's vertices and a
//! point of each of its faces are placed against the terms near it: one strictly outside the stock
//! and the other unions, outside every cut and inside every common, has a ball about it the union's
//! material in which the body keeps and no other term gives (a witness: (A) fails, a boss bored
//! through at its rim).  Only where there is none is the arrangement made.  The terms are built
//! where the body stands (`exact::operands`), the body never.

use super::*;
use crate::brep::boolean::{arrangement, members, overlap};
use crate::brep::query::{Located, Place};
use crate::brep::recipe::Node;
use crate::brep::topo::Brep;
use std::cell::RefCell;
use std::collections::BTreeSet;

/// Union operand `union` of body `body` (solid indices) that adds the body nothing it keeps while
/// `cuts` take some of it, or it lies outside some of `commons` (those of the body's that take
/// some).  `held`: the terms whose material is all the body keeps of it — the stock, and unions
/// adding material of their own — empty where it keeps none.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Unkept {
    pub body: usize,
    pub union: usize,
    pub cuts: Vec<usize>,
    pub commons: Vec<usize>,
    pub held: Vec<usize>,
}

/// A term, by its node id, and its B-rep.
type Term<'a> = (i64, &'a Brep);

/// A body's written terms, read against one another.
struct Reader<'a> {
    stock: Term<'a>,
    unions: Vec<Term<'a>>,
    cuts: Vec<Term<'a>>,
    commons: Vec<Term<'a>>,
    boxes: BTreeMap<i64, crate::brep::geom::Box3>,
    located: BTreeMap<i64, Located<'a>>,
    tol: f64,
    /// Whether each union adds material no other term holds (`adds`), read once.
    adds: RefCell<BTreeMap<i64, bool>>,
}

/// What the arrangement of a union with the terms near it says of it.
struct Read {
    /// Each cell of the union the body keeps, with the material terms (stock, other unions) it
    /// lies in: none, where it is the union's alone.
    kept: Vec<Vec<i64>>,
    /// The cuts holding a cell of it, and the commons leaving one out.
    by_cut: BTreeSet<i64>,
    by_common: BTreeSet<i64>,
}

impl<'a> Reader<'a> {
    fn meets(&self, a: i64, b: i64) -> bool {
        overlap(&self.boxes[&a], &self.boxes[&b], self.tol)
    }

    /// The material terms (the stock, then the unions) but `u`, whose boxes meet its own.
    fn beside(&self, u: i64) -> Vec<Term<'a>> {
        std::iter::once(self.stock).chain(self.unions.iter().copied())
            .filter(|&(id, _)| id != u && self.meets(u, id)).collect()
    }

    /// The cuts whose boxes meet `u`'s.
    fn near(&self, u: i64) -> Vec<Term<'a>> {
        self.cuts.iter().copied().filter(|&(id, _)| self.meets(u, id)).collect()
    }

    /// **A witness that union `u` adds the body material** no other term gives: a vertex of it,
    /// or a point of one of its faces, strictly outside every material term beside it and every
    /// near cut, and strictly inside every common — each by exact point location, more than the
    /// tolerance off their boundaries, so a ball about it holds none of theirs, and `u`'s
    /// material within it is kept and the union's alone.
    fn witness(&self, u: Term<'a>, beside: &[Term<'a>], near: &[Term<'a>]) -> bool {
        let own = &self.located[&u.0];
        let faces = u.1.faces.iter().enumerate().filter_map(|(fi, f)| {
            let [[u0, u1], [v0, v1]] = crate::brep::boolean::face_parameters(u.1, fi);
            let p = f.surface.point([(u0 + u1) / 2.0, (v0 + v1) / 2.0]);
            (own.face_place(fi, p) == Place::In).then_some(p)
        });
        let out = |t: &Term, p| self.located[&t.0].solid_place(p) == Place::Out;
        u.1.vertices.iter().map(|v| v.p).chain(faces).any(|p| {
            beside.iter().chain(near).all(|t| out(t, p))
                && self.commons.iter().all(|k| self.located[&k.0].solid_place(p) == Place::In)
        })
    }

    /// The arrangement of union `u` with the material terms beside it, the near cuts and the
    /// commons, read cell by cell.
    fn read(&self, u: Term<'a>, beside: &[Term<'a>], near: &[Term<'a>]) -> Result<Read, String> {
        let operands: Vec<&Brep> = std::iter::once(u.1)
            .chain(beside.iter().chain(near).chain(&self.commons).map(|t| t.1)).collect();
        let (h, c) = (1 + beside.len(), 1 + beside.len() + near.len());
        let complex = arrangement(&operands, self.tol)?;
        let inside = members(&complex, &operands, self.tol)?;
        let mut read =
            Read { kept: Vec::new(), by_cut: BTreeSet::new(), by_common: BTreeSet::new() };
        for m in inside.iter().skip(1).filter(|m| m[0]) {
            let taken: Vec<i64> = (h..c).filter(|&k| m[k]).map(|k| near[k - h].0).collect();
            let outside: Vec<i64> =
                (c..m.len()).filter(|&k| !m[k]).map(|k| self.commons[k - c].0).collect();
            if taken.is_empty() && outside.is_empty() {
                read.kept.push((1..h).filter(|&k| m[k]).map(|k| beside[k - 1].0).collect());
            }
            read.by_cut.extend(taken);
            read.by_common.extend(outside);
        }
        Ok(read)
    }

    /// Whether union `v` adds the body material no other term holds — some point of it kept,
    /// in the stock and no other union: read against every term, never against what is said of
    /// the others, so two unions cannot vouch for one another.
    fn adds(&self, v: Term<'a>) -> Result<bool, String> {
        let known = self.adds.borrow().get(&v.0).copied();
        if let Some(a) = known {
            return Ok(a);
        }
        let (beside, near) = (self.beside(v.0), self.near(v.0));
        let adds = self.witness(v, &beside, &near)
            || self.read(v, &beside, &near)?.kept.iter().any(|holders| holders.is_empty());
        self.adds.borrow_mut().insert(v.0, adds);
        Ok(adds)
    }
}

/// The unions of body `body` that add it nothing it keeps while its cuts or commons take some of
/// them (`Unkept`), in the order its unions are listed.  Empty for a solid that is no body, a
/// body with no cut and no common, and one with swept material, whose boundary is the field's.
/// Refused where its terms are not built exactly (the body's own refusal is said where it is
/// built): then nothing is known.
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
    let brep = |id: &i64| built.get(id)
        .ok_or_else(|| "a term of the body is not built".to_string());
    let read = |ids: &[i64]| ids.iter().map(|id| Ok((*id, brep(id)?)))
        .collect::<Result<Vec<Term>, String>>();
    let stock = (terms.stock, brep(&terms.stock)?);
    let (unions, cuts, commons) = (read(&terms.on)?, read(&terms.cut)?, read(&terms.bound)?);
    let all: Vec<Term> = std::iter::once(stock)
        .chain(unions.iter().chain(&cuts).chain(&commons).copied()).collect();
    let size = all.iter().fold(0.0f64, |m, (_, b)| m.max(b.size()));
    let tol = crate::brep::geom::tolerance_over(size, 0.0);
    let r = Reader {
        boxes: all.iter().map(|(id, b)| (*id, b.bounds())).collect(),
        located: all.iter().map(|(id, b)| (*id, Located::new(b, tol))).collect(),
        stock, unions, cuts, commons, tol, adds: RefCell::new(BTreeMap::new()),
    };
    let mut out = Vec::new();
    for &u in &r.unions {
        // (B) needs a cut that can take some of it, or a common
        let near = r.near(u.0);
        if near.is_empty() && r.commons.is_empty() {
            continue;
        }
        let beside = r.beside(u.0);
        // (A) fails at a witness, with no arrangement made
        if r.witness(u, &beside, &near) {
            r.adds.borrow_mut().insert(u.0, true);
            continue;
        }
        let read = r.read(u, &beside, &near)?;
        // (B): nothing of it taken, or (a union of no cell) nothing of it at all
        if read.by_cut.is_empty() && read.by_common.is_empty() {
            continue;
        }
        // (A): each cell it keeps the stock's or another union's that adds material of its own
        let mut held = BTreeSet::new();
        let mut adds = false;
        'cells: for holders in &read.kept {
            if holders.contains(&r.stock.0) {
                held.insert(r.stock.0);
                continue;
            }
            for &v in holders {
                let term = r.unions.iter().copied().find(|t| t.0 == v).expect("a union beside it");
                if r.adds(term)? {
                    held.insert(v);
                    continue 'cells;
                }
            }
            adds = true;
            break;
        }
        r.adds.borrow_mut().insert(u.0, read.kept.iter().any(|h| h.is_empty()));
        if adds {
            continue;
        }
        let solid = |ids: BTreeSet<i64>| ids.into_iter().map(|id| id as usize).collect();
        out.push(Unkept {
            body,
            union: u.0 as usize,
            cuts: solid(read.by_cut),
            commons: solid(read.by_common),
            held: solid(held),
        });
    }
    Ok(out)
}
