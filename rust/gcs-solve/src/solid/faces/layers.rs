//! **Strata in layers** (#183 F6, [0.149]): a round that names another round's faces by the name
//! that round's statement was given (`rim := plate.c0 round(r: 3mm) plate.far`, then `blend :=
//! part.boss round(r: 1mm) part.rim`) reads the body as that round leaves it, and acts after it.
//! A body's readers fall into layers: a word's use stands one layer above the deepest use whose
//! faces it names by its label (`route_step`), the rest in the first.  Each layer reads the body
//! as its other terms and every layer beneath it make it, and acts on it after them, fills then
//! cuts — so two rounds of one layer still never see each other, and nothing is ordered by where a
//! statement is written: only by what it names.
//!
//! The layers are bodies of their own, named as the body is, each the stock of the one above:
//! what `shell := solid(stock)` then `blend … shell` says by hand, written for the document.  A
//! stock of its body's name adds no step to a face's route (`document::operand_primitives`), so
//! every face is called what it was; the field, the recipe and the native host apply each layer's
//! readers as late terms of its own body, the one rule they already have.
use super::path::{names_use, use_key};
use crate::model::{EntKind, EntRef, FaceRef, RegionShape, RegionTerm, Sketch, SolidDef, SolidE, Target};
use std::collections::{BTreeMap, BTreeSet};

/// Every body whose readers name one another's faces laid out in layers, in place.  What it
/// refuses — uses that each name the other, so neither can act on the body the other leaves — is
/// left as written (one layer, each blind to the other), and said: each solid and why.
pub fn layer(sk: &mut Sketch) -> Vec<(usize, String)> {
    let mut refused = Vec::new();
    for b in 0..sk.solids.len() {
        if !matches!(sk.solids[b].def, SolidDef::Body { .. }) {
            continue;
        }
        let readers = super::readers_of(sk, b);
        if readers.len() < 2 {
            continue;
        }
        match layers_of(sk, b, &readers) {
            Ok(of) if of.values().any(|&k| k > 0) => stack(sk, b, &of),
            Ok(_) => {}
            Err((at, why)) => refused.push((at, why)),
        }
    }
    refused
}

/// The labels a solid's faces are named under (`route_step` without its leaf): none for a solid no
/// named use made.
fn labels(name: &str) -> Vec<String> {
    let step = crate::solid::route_step(name);
    let mut segs: Vec<String> = step.split('.').map(str::to_string).collect();
    segs.pop();
    segs
}

/// Every face reference among `terms`, sets measured from included.
fn face_refs<'t>(terms: &'t [RegionTerm], out: &mut Vec<&'t FaceRef>) {
    fn target<'t>(t: &'t Target, out: &mut Vec<&'t FaceRef>) {
        match t {
            Target::Ent(_) => {}
            Target::Face(f) => out.push(f),
            Target::Set { terms, .. } => face_refs(terms, out),
        }
    }
    for t in terms {
        match &t.shape {
            RegionShape::FaceSide { face } => out.push(face),
            RegionShape::Angle { a, b } => out.extend([a, b]),
            RegionShape::Measured { of } => of.iter().for_each(|(t, _)| target(t, out)),
            RegionShape::Ends { edge } => target(edge, out),
            RegionShape::Ball { .. } | RegionShape::Cylinder { .. } | RegionShape::Cone { .. }
            | RegionShape::Plane { .. } => {}
        }
    }
}

/// Each of body `b`'s readers' layer, or the reader a cycle is said at and why.
fn layers_of(sk: &Sketch, b: usize, readers: &BTreeSet<usize>)
    -> Result<BTreeMap<usize, usize>, (usize, String)> {
    let mut uses: BTreeMap<&str, Vec<usize>> = BTreeMap::new();
    for &r in readers {
        uses.entry(use_key(&sk.solids[r].name)).or_default().push(r);
    }
    let keys: Vec<&str> = uses.keys().copied().collect();
    let label: Vec<Vec<String>> = keys.iter().map(|k| labels(&sk.solids[uses[k][0]].name)).collect();
    // what each use names: every use another of its readers' face references reaches by its label
    let names: Vec<BTreeSet<usize>> = keys.iter().enumerate().map(|(u, k)| {
        let mut refs = Vec::new();
        for &r in &uses[k] {
            if let SolidDef::Region { terms, .. } = &sk.solids[r].def {
                face_refs(terms, &mut refs);
            }
        }
        (0..keys.len()).filter(|&v| v != u && refs.iter()
            .any(|f| f.solid as usize == b && names_use(&f.path, &label[v]))).collect()
    }).collect();
    // each use's layer, one above the deepest it names; a use reached again on its own walk names
    // itself through the others
    fn depth(u: usize, names: &[BTreeSet<usize>], memo: &mut Vec<Option<usize>>, walk: &mut Vec<usize>)
        -> Result<usize, Vec<usize>> {
        if let Some(k) = memo[u] {
            return Ok(k);
        }
        if let Some(at) = walk.iter().position(|&w| w == u) {
            return Err(walk[at..].to_vec());
        }
        walk.push(u);
        let mut k = 0;
        for &v in &names[u] {
            k = k.max(depth(v, names, memo, walk)? + 1);
        }
        walk.pop();
        memo[u] = Some(k);
        Ok(k)
    }
    let mut memo = vec![None; keys.len()];
    let mut out = BTreeMap::new();
    for u in 0..keys.len() {
        let k = depth(u, &names, &mut memo, &mut Vec::new()).map_err(|cycle| {
            let said: Vec<String> = cycle.iter().map(|&c| format!("`{}`", label[c].join("."))).collect();
            (uses[keys[cycle[0]]][0], format!(
                "{} each name the faces the other makes, so neither can act on the body the other \
                 leaves: a round reads the body as the rounds it names leave it", said.join(" and ")))
        })?;
        for &r in &uses[keys[u]] {
            out.insert(r, k);
        }
    }
    Ok(out)
}

/// Body `b` laid out in the layers `of` gives its readers: each layer below the top a body of its
/// own, of `b`'s name, standing on the one beneath, its readers reading it; the top `b` itself.
fn stack(sk: &mut Sketch, b: usize, of: &BTreeMap<usize, usize>) {
    let SolidDef::Body { stock, on, through, bound } = sk.solids[b].def.clone() else { return };
    let top = of.values().copied().max().unwrap_or(0);
    let s = &sk.solids[b];
    let (name, class, copy) = (s.name.clone(), s.class.clone(), s.copy);
    let at = |o: &u32| of.get(&(*o as usize)).copied();
    let mut below: Option<u32> = None;
    for k in 0..top {
        let pick = |v: &[u32], plain: bool| -> Vec<u32> {
            v.iter().copied().filter(|o| at(o) == Some(k) || (plain && at(o).is_none())).collect()
        };
        let def = match below {
            None => SolidDef::Body { stock, on: pick(&on, true), through: pick(&through, true), bound: bound.clone() },
            Some(s) => SolidDef::Body { stock: s, on: pick(&on, false), through: pick(&through, false), bound: Vec::new() },
        };
        let layer = sk.solids.len();
        // what it is made of is the top's, the body the document named (#510)
        sk.solids.push(SolidE { def, name: name.clone(), class: class.clone(), copy,
                                material: None, binding: None });
        // the layer's readers read the body it is
        let to = |e: EntRef| Some(if e.kind == EntKind::Solid && e.i() == b { EntRef::solid(layer) } else { e });
        for (&r, _) in of.iter().filter(|(_, &l)| l == k) {
            if let SolidDef::Region { terms, .. } = &mut sk.solids[r].def {
                *terms = terms.iter().map(|t| t.remapped(&to).expect("every entity kept")).collect();
            }
        }
        below = Some(layer as u32);
    }
    let last = |v: &[u32]| v.iter().copied().filter(|o| at(o) == Some(top)).collect();
    let body = SolidDef::Body {
        stock: below.expect("a layer below the top"), on: last(&on), through: last(&through), bound: Vec::new(),
    };
    sk.redefine(b, |d| { *d = body; Some(()) });
}
