//! **A sweep distributes over union** (#684 C, #586): under one motion over one roll,
//! `sweep(A ∪ B) = sweep(A) ∪ sweep(B)`, so a body cutting the sweep of a union cuts the sweep of
//! each member, and `blank − sweep(A ∪ B)` is `blank − sweep(A) − sweep(B)`. A body that cuts such
//! a sweep is built from a copy of the drawing with that cut written as its members' cuts
//! (`lowered`): one sweep a member, under the union's motion, over the union's roll
//! (`Sketch::distributed` sends the roll and the cutter's face routes back to the union), each
//! placement of the union's sweep a placement of every member's. Each member's sweep is then
//! whatever it is alone — a ball's tube (`canal`), a sweep of the generating or the constant-twist
//! class, admitted by its own rows and refused by them naming the member — and the body is built
//! and judged as any body cutting several sweeps is. Nothing here reads geometry: the copy's field
//! is the drawing's, member by member, and the export holds what it builds to the drawing's own
//! field of the union.
//!
//! A union is its members where it is the whole tool, a union within a union or a placement of
//! one (each member placed as the union is): one within an intersection or a cut (`(A ∪ B) ∩ C`)
//! is not distributed, and is read by its B-rep as any tool is. Nor is one solid under a constant
//! twist whose B-rep the kernel reads (`whole`, #721 F1d): the walk ends a characteristic at the
//! concave corners where its members meet, so it is swept whole, as its set; a union of separate
//! solids, or one under any other motion, is still its members'. A tool of prisms under a motion keeping their plane is
//! not distributed either: the planar subtraction class reads the union's swept region whole
//! (`planar_subtraction::asks`).
use crate::model::{Sketch, SolidDef};

/// A member of a union: a solid of the drawing that is no union, and the placements of the union
/// it stands under (`twins := lump at(under: m) 20deg`), the outermost first.
struct Member {
    solid: u32,
    placed: Vec<u32>,
}

/// The members of union `id` — a body that only puts solids on its stock, or a placement of one —
/// in the order the body rule writes them (the stock, then what is put on it), a union among them
/// its own members; `None` where `id` is no union: a body that cuts or is bounded by anything is
/// not, nor a solid that is no body.
fn members(sk: &Sketch, id: usize) -> Option<Vec<Member>> {
    match &sk.solids.get(id)?.def {
        SolidDef::Placed { source, .. } => {
            let mut inner = members(sk, *source as usize)?;
            for m in &mut inner {
                m.placed.insert(0, id as u32);
            }
            Some(inner)
        }
        SolidDef::Body { stock, on, through, bound }
            if !on.is_empty() && through.is_empty() && bound.is_empty() =>
        {
            let leaf = |o: u32| vec![Member { solid: o, placed: Vec::new() }];
            Some(std::iter::once(stock).chain(on)
                .flat_map(|&o| members(sk, o as usize).unwrap_or_else(|| leaf(o))).collect())
        }
        _ => None,
    }
}

/// The cuts of `body` that are sweeps of unions this distributes (`members`), each by its place in
/// the body's cuts and the chain of placements it stands the sweep at: the cut operand first, the
/// sweep last.
fn union_cuts(sk: &Sketch, body: usize) -> Vec<(usize, Vec<u32>)> {
    let Some(SolidDef::Body { through, .. }) = sk.solids.get(body).map(|s| &s.def) else {
        return Vec::new();
    };
    through.iter().enumerate().filter_map(|(k, &c)| {
        let mut chain = vec![c];
        while let SolidDef::Placed { source, .. } = sk.solids[*chain.last()? as usize].def {
            chain.push(source);
        }
        let p = *chain.last()? as usize;
        let SolidDef::Swept { source, .. } = sk.solids[p].def else { return None };
        let distributed = members(sk, source as usize).is_some()
            && !super::planar_subtraction::asks(sk, p) && !whole(sk, p);
        distributed.then_some((k, chain))
    }).collect()
}

/// Whether union sweep `p` is read whole, as its set (#721 F1d): one solid, its B-rep's faces all
/// joined by the edges they share, swept by a constant twist — a screw, a turn or a slide — whose
/// walk ends a characteristic at the concave corners where its members meet and trims the sheets
/// that meet there, wherever the kernel reads that B-rep. A union of separate solids is its
/// members' sweeps (its characteristic would part into a stretch a member, S2), and so is one under
/// any other motion (the generating class refuses a concave corner carried through the blank, T2).
fn whole(sk: &Sketch, p: usize) -> bool {
    let SolidDef::Swept { motion, .. } = sk.solids[p].def else { return false };
    let twisted = crate::motion::Family::read(sk, motion as usize).ok().and_then(|f| f.trajectory().ok())
        .and_then(|t| crate::brep::swept::constant_twist::twist_of(&t)).is_some();
    twisted && super::cad::tool_brep(sk, p).is_ok_and(|b| connected(&b)
        && crate::brep::swept::tool::Tool::read(&b, super::cad::millimetres(sk)).is_ok())
}

/// Whether every face of `b` is joined to every other by a chain of faces sharing edges: one solid.
fn connected(b: &crate::brep::topo::Brep) -> bool {
    let n = b.faces.len();
    if n == 0 { return false }
    let uses = b.uses();
    let mut seen = vec![false; n];
    let mut stack = vec![0usize];
    seen[0] = true;
    while let Some(f) = stack.pop() {
        for c in b.faces[f].loops.iter().flatten() {
            for &(g, _, _) in &uses[c.edge as usize] {
                if !seen[g] { seen[g] = true; stack.push(g) }
            }
        }
    }
    seen.into_iter().all(|x| x)
}

/// Whether `body` cuts the sweep of a union this distributes, so that it is built from `lowered`.
pub fn lowers(sk: &Sketch, body: usize) -> bool {
    !union_cuts(sk, body).is_empty()
}

/// `sk` with each cut of `body` that is a union's sweep, or a placement of one, written as one cut
/// a member, in its place in the body's cuts: each member's sweep minted once however many
/// placements cut the union's, under the union's motion and over its roll, and each placement
/// minted again over it; `None` where `body` cuts no such sweep. Every solid minted keeps the name
/// of the one it copies, so what the body reads, reports and names is the document's; each
/// member's sweep is recorded in `Sketch::distributed` against the union's.
pub fn lowered(sk: &Sketch, body: usize) -> Option<Sketch> {
    let cuts = union_cuts(sk, body);
    if cuts.is_empty() {
        return None;
    }
    // each union's roll found on the drawing, so the copy reads it and does not march it again
    for (_, chain) in &cuts {
        let _ = super::roll::interval(sk, *chain.last().expect("a chain ends in its sweep") as usize);
    }
    let mut out = sk.clone();
    let mint = |out: &mut Sketch, like: u32, def: SolidDef| -> u32 {
        let mut s = sk.solids[like as usize].clone();
        s.def = def;
        out.solids.push(s);
        (out.solids.len() - 1) as u32
    };
    // `top` under the placements `chain` lists, the outermost first: each minted again over it
    let place = |out: &mut Sketch, mut top: u32, chain: &[u32]| -> u32 {
        for &q in chain.iter().rev() {
            let SolidDef::Placed { motion, at, .. } = &sk.solids[q as usize].def else {
                unreachable!("a chain of placements")
            };
            top = mint(out, q, SolidDef::Placed { source: top, motion: *motion, at: at.clone() });
        }
        top
    };
    // each union's sweep's members' sweeps, minted once however many placements cut it
    let mut sweeps: std::collections::BTreeMap<u32, Vec<u32>> = Default::default();
    let mut written: Vec<(usize, Vec<u32>)> = Vec::new();
    for (k, chain) in cuts {
        let p = *chain.last().expect("a chain ends in its sweep");
        let swept = sweeps.entry(p).or_insert_with(|| {
            let SolidDef::Swept { source, motion, roll } = &sk.solids[p as usize].def else {
                unreachable!("a union's cut is a sweep")
            };
            let members = members(sk, *source as usize).expect("a union's sweep has members");
            members.iter().map(|m| {
                let tool = place(&mut out, m.solid, &m.placed);
                let def = SolidDef::Swept { source: tool, motion: *motion, roll: roll.clone() };
                let i = mint(&mut out, p, def);
                out.distributed.insert(i, p);
                i
            }).collect()
        }).clone();
        // the placements between the body and the sweep, over each member's
        let between = &chain[..chain.len() - 1];
        written.push((k, swept.into_iter().map(|top| place(&mut out, top, between)).collect()));
    }
    let SolidDef::Body { through, .. } = &mut out.solids[body].def else {
        unreachable!("a body's cuts")
    };
    // from the last, so each cut's place is where the document wrote it
    for (k, tops) in written.into_iter().rev() {
        through.splice(k..=k, tops);
    }
    Some(out)
}

/// The sweep `swept` stands for: the union's whose member it sweeps (`lowered`), else itself.
pub fn union_of(sk: &Sketch, swept: usize) -> usize {
    sk.distributed.get(&(swept as u32)).map_or(swept, |&p| p as usize)
}
