//! **Which faces of a body meet** (#625): what `when a meets b` reads, so a block over a body's
//! faces in pairs and threes (`A round(r) B`'s edges and corners) makes copies only for faces
//! that share a point, and not for every pair and three of the names a body lists.
use super::{cached, FacePath};
use crate::model::Sketch;
use std::collections::{BTreeMap, BTreeSet};

/// The faces a solid lists (`face_paths`), each by its name as written from the solid
/// (`FacePath::text`), and the listed faces it shares a point with, read off the stratum
/// (`cached`: the body as made by the terms that do not read its faces) where the drawing
/// stands: a corner of its boundary, so every edge's ends too.  A face a Boolean ate shares none.
/// `None` where that boundary cannot say — a face a round of the body made, which the stratum
/// may not have.
#[derive(Clone, Debug)]
pub struct Meets {
    faces: BTreeMap<String, Option<BTreeSet<String>>>,
}

impl Meets {
    /// The faces `listed` of solid `body` (its `face_paths`), or `None` where its boundary is
    /// not read here: a body with a swept cut, whose boundary is the field's, or one that does
    /// not build.
    pub fn read(sk: &Sketch, body: usize, listed: &[FacePath]) -> Option<Meets> {
        if body >= sk.solids.len() || crate::solid::cad::contains_sweep(sk, body) {
            return None;
        }
        let read = cached(sk, body).ok()?;
        // each patch's corners: two patches sharing an edge share its ends
        let corners: Vec<BTreeSet<u32>> = (0..read.brep().0.faces.len())
            .map(|fi| read.edges_of(fi).iter().flat_map(|&e| read.ends(e)).collect())
            .collect();
        let patches = |f: &FacePath| -> Vec<usize> {
            read.paths().iter().enumerate()
                .filter(|(_, p)| p.route == f.route && p.face == f.face)
                .map(|(k, _)| k).collect()
        };
        let mine: Vec<(String, Vec<usize>)> =
            listed.iter().map(|f| (f.text(), patches(f))).collect();
        let touch = |a: &[usize], b: &[usize]| a.iter()
            .any(|&i| b.iter().any(|&j| i == j || !corners[i].is_disjoint(&corners[j])));
        let faces = listed.iter().zip(&mine).map(|(f, (text, own))| {
            // a round's face the stratum has not: it cannot say what that one meets
            let unread = f.of.is_some() && own.is_empty();
            let meets = (!unread).then(|| mine.iter().filter(|(_, other)| touch(own, other))
                .map(|(t, _)| t.clone()).collect());
            (text.clone(), meets)
        }).collect();
        Some(Meets { faces })
    }

    /// Whether faces `a` and `b` (each by its name from the solid) share a point: where either
    /// is a face the boundary cannot say of, or neither is listed, they are taken to.
    pub fn meets(&self, a: &str, b: &str) -> bool {
        match (self.faces.get(a), self.faces.get(b)) {
            (Some(Some(x)), Some(Some(_))) => x.contains(b),
            _ => true,
        }
    }
}
