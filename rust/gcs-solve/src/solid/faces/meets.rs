//! **Which faces of a body meet** (#625): what `when a meets b` reads, so a block over a body's
//! faces in pairs and threes (`A round(r) B`'s edges and corners) makes copies only for faces
//! that share a point, and not for every pair and three of the names a body lists.
use super::{FacePath, ListedFaces};
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
    /// not read here (`ListedFaces::read`).
    pub fn read(sk: &Sketch, body: usize, listed: &[FacePath]) -> Option<Meets> {
        ListedFaces::read(sk, body, listed).map(|l| Meets::of(&l))
    }

    /// The faces of `listed` that meet.
    pub fn of(listed: &ListedFaces) -> Meets {
        let read = &listed.read;
        // each patch's corners: two patches sharing an edge share its ends
        let corners: Vec<BTreeSet<u32>> = (0..read.brep().0.faces.len())
            .map(|fi| read.edges_of(fi).iter().flat_map(|&e| read.ends(e)).collect())
            .collect();
        let touch = |a: &[usize], b: &[usize]| a.iter()
            .any(|&i| b.iter().any(|&j| i == j || !corners[i].is_disjoint(&corners[j])));
        let faces = listed.faces.iter().map(|(text, own)| {
            let meets = own.as_ref().map(|own| listed.faces.iter()
                .filter(|(_, other)| other.as_ref().is_some_and(|other| touch(own, other)))
                .map(|(t, _)| t.clone()).collect());
            (text.clone(), meets)
        }).collect();
        Meets { faces }
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
