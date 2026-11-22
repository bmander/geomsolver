//! **Which regions the table recognises** (#194): a region's terms read as the faces it rounds
//! or cuts at and the section beside their edges — by what each term bounds, of which faces, by
//! which number, never by the word that wrote them.
use super::Section;
use crate::model::{FaceRef, RegionShape, RegionTerm, Sketch, Target};
use crate::vocab::Cmp;

/// The faces a region rounds or cuts at, and whether it lies within all of them (a convex edge's
/// material) or outside all of them (a concave edge's void).
pub(super) struct Faces<'a> {
    pub(super) faces: Vec<&'a FaceRef>,
    pub(super) within: bool,
}

impl Faces<'_> {
    /// The pairs of faces meeting in an edge: one for two faces, three for three.
    pub(super) fn pairs(&self) -> &'static [[usize; 2]] {
        if self.faces.len() == 2 { &[[0, 1]] } else { &[[0, 1], [1, 2], [2, 0]] }
    }

    /// The pair `p` is, as its faces' surfaces or patches (`surface`), and whether it names them
    /// the other way round.
    fn pair_of(&self, p: [&FaceRef; 2], surface: bool) -> Option<(usize, bool)> {
        self.pairs().iter().enumerate().find_map(|(k, &[i, j])| {
            let is = |x: usize, y: usize| same(self.faces[x], p[0], surface) && same(self.faces[y], p[1], surface);
            if is(i, j) { Some((k, false)) } else if is(j, i) { Some((k, true)) } else { None }
        })
    }

    /// Whether `set` is every face once, each its surface or its patch.
    fn all(&self, set: &[&FaceRef], surface: bool) -> bool {
        set.len() == self.faces.len()
            && self.faces.iter().all(|f| set.iter().filter(|g| same(f, g, surface)).count() == 1)
    }
}


/// A term's number, where it states one and bounds it on one side, with no offset.
fn plain(sk: &Sketch, t: &RegionTerm) -> Option<f64> {
    (t.hi.is_none() && t.grow.is_none()).then(|| t.lo.read(sk)).flatten()
}

/// Whether two face references name the same faces, the second of them its surface or not.  A
/// smooth one (`F.smooth`, [0.149]) is read as its face where a side is (`x`) and as its surface
/// where a distance is: the table builds the round of the face's own edge, and the gate holds it
/// to the field, which finds where the smooth surface runs on across a seam into the section.
fn same(x: &FaceRef, y: &FaceRef, surface: bool) -> bool {
    x.solid == y.solid && x.path == y.path && (!x.surface || x.smooth)
        && (y.surface == surface || y.smooth)
}

/// A term bounding the distance from one face (or its surface): the face, which way, its number.
fn one_face<'a>(sk: &Sketch, t: &'a RegionTerm) -> Option<(&'a FaceRef, Cmp, f64)> {
    let (f, c) = crate::solid::faces::measured_face(t)?;
    (c == 1.0).then_some((f, t.cmp, plain(sk, t)?))
}

/// The faces a set is the points on (`{ q | q coincident a; q coincident b }`): each term a
/// distance from one face (or its surface) at most nothing.
fn on_all<'a>(sk: &Sketch, terms: &'a [RegionTerm]) -> Option<Vec<&'a FaceRef>> {
    terms.iter().map(|t| match one_face(sk, t)? {
        (f, Cmp::Le, d) if d == 0.0 => Some(f),
        _ => None,
    }).collect()
}

/// The faces a meeting is on the surfaces of (`{ q | q coincident a.surface; q coincident
/// b.surface }`), cut or not where their edge ends (`q inside crease.ends`, std's: a run carried
/// on past its end and cut off there, the same points as one ending square where a face cuts it
/// square — and `gate` holds the build to the field where one does not).
fn meeting_of<'a>(sk: &Sketch, terms: &'a [RegionTerm]) -> Option<Vec<&'a FaceRef>> {
    terms.iter().filter_map(|t| match &t.shape {
        RegionShape::Ends { edge } => match edge.as_ref() {
            Target::Set { terms: inner, .. } if on_all(sk, inner).is_some_and(|f| f.len() == 2)
                && t.cmp == Cmp::Le && plain(sk, t) == Some(0.0) => None,
            _ => Some(None),
        },
        _ => Some(match one_face(sk, t)? {
            (f, Cmp::Le, d) if d == 0.0 => Some(f),
            _ => None,
        }),
    }).collect()
}

/// The face a term bounds the side of at nothing, and which way: `p inside f` or `p outside f`.
fn side_of<'a>(sk: &Sketch, t: &'a RegionTerm) -> Option<(&'a FaceRef, Cmp)> {
    let RegionShape::FaceSide { face } = &t.shape else { return None };
    ((!face.surface || face.smooth) && t.cmp != Cmp::In && plain(sk, t)? == 0.0).then_some((face, t.cmp))
}

/// The faces and the section a region's terms describe, where they are one of this table's: the
/// faces' sides, the material's angle at each edge between two of them against a half turn, the
/// nearest point of the faces' meeting on their common part, each edge's ends, and one bound
/// saying the section. A term no entry reads is no entry's region.
pub(super) fn recognised<'a>(sk: &Sketch, terms: &'a [RegionTerm]) -> Option<(Faces<'a>, Section)> {
    let sides: Vec<(&FaceRef, Cmp)> = terms.iter().filter(|t| matches!(t.shape, RegionShape::FaceSide { .. }))
        .map(|t| side_of(sk, t)).collect::<Option<_>>()?;
    let cmp = sides.first()?.1;
    let faces: Vec<&FaceRef> = sides.iter().map(|s| s.0).collect();
    let distinct = faces.iter().enumerate()
        .all(|(i, f)| faces[..i].iter().all(|g| g.path != f.path) && f.solid == faces[0].solid);
    if !(2..=3).contains(&faces.len()) || sides.iter().any(|s| s.1 != cmp) || !distinct {
        return None;
    }
    let faces = Faces { faces, within: cmp == Cmp::Le };
    let n = faces.pairs().len();
    let (mut angles, mut ends, mut slab, mut section) = (vec![0; n], vec![0; n], 0, None);
    let mut rolled: Option<f64> = None;
    for t in terms {
        match &t.shape {
            RegionShape::FaceSide { .. } => {}
            RegionShape::Angle { a, b } => {
                let half = plain(sk, t).is_some_and(|lo| (lo - std::f64::consts::PI).abs() <= 1e-12);
                if !half || t.cmp != cmp {
                    return None;
                }
                angles[faces.pair_of([a, b], false)?.0] += 1;
            }
            RegionShape::Ends { edge } => {
                let Target::Set { terms: inner, .. } = edge.as_ref() else { return None };
                let [x, y] = on_all(sk, inner)?[..] else { return None };
                if t.cmp != Cmp::Le || plain(sk, t)? != 0.0 {
                    return None;
                }
                ends[faces.pair_of([x, y], false)?.0] += 1;
            }
            RegionShape::Measured { of } => match &of[..] {
                // a ball touching every face no more than its radius further off than the nearest
                // ball within them (#994): what the ball rolled along the edge leaves, the same
                // section — its radius the bound's, held to the section's below
                [(Target::Set { terms: x, .. }, cx), (Target::Set { terms: y, .. }, cy)]
                    if (cx + cy).abs() <= 1e-12 * cx.abs()
                        && rolled_by(sk, t, x, y, *cx, &faces).is_some() => {
                    let r = rolled_by(sk, t, x, y, *cx, &faces).expect("read");
                    if rolled.is_some_and(|q: f64| q != r) {
                        return None;
                    }
                    rolled = Some(r);
                }
                // nearer the faces' common part than wherever else their surfaces meet
                [(Target::Set { terms: x, .. }, cx), (Target::Set { terms: y, .. }, cy)] => {
                    let (common, meeting) = if *cx > 0.0 { (x, y) } else { (y, x) };
                    if (cx + cy).abs() > 1e-12 * cx.abs() || t.cmp != Cmp::Le || plain(sk, t)? != 0.0
                        || !faces.all(&on_all(sk, common)?, false) || !faces.all(&meeting_of(sk, meeting)?, true) {
                        return None;
                    }
                    slab += 1;
                }
                // beyond reach of the ball touching every face: where its centre may stand
                [(Target::Set { terms: centres, .. }, c)] => {
                    let r = plain(sk, t)?;
                    if *c != 1.0 || t.cmp != Cmp::Ge || !(r > 0.0) || section.is_some()
                        || !ball_centres(sk, centres, &faces, r) {
                        return None;
                    }
                    section = Some(Section::Round(r));
                }
                // short of a flat: the faces' distances, each over the other's leg, at most one
                [(Target::Face(x), kx), (Target::Face(y), ky)] if faces.faces.len() == 2 => {
                    let l = plain(sk, t)?;
                    if t.cmp != Cmp::Le || !(l > 0.0 && *kx > 0.0 && *ky > 0.0) || section.is_some() {
                        return None;
                    }
                    let flat = Section::Flat { la: l / ky, lb: l / kx };
                    section = Some(if faces.pair_of([x, y], true)?.1 { flat.swapped() } else { flat });
                }
                _ => return None,
            },
            _ => return None,
        }
    }
    (angles.iter().chain(&ends).all(|&k| k == 1) && slab == 1).then_some(())?;
    // the rolled ball's, where stated, the section's own
    let same = |q: f64| matches!(section, Some(Section::Round(r)) if (q - r).abs() <= 1e-12 * r);
    if rolled.is_some_and(|q| !same(q)) {
        return None;
    }
    Some((faces, section?))
}

/// The radius of the ball a term keeps the point near where it rolls (`distance(p, rolled) <=
/// distance(p, centres) + r`): `x` (by `cx`) a ball's centres touching every face — on the
/// region's side of each, at its surface's distance `r` — and the other where the centre of one
/// within every face may stand (`ball_centres`); `None` where the term is not that.
fn rolled_by(sk: &Sketch, t: &RegionTerm, x: &[RegionTerm], y: &[RegionTerm], cx: f64,
    faces: &Faces) -> Option<f64> {
    let (touch, centres) = if cx > 0.0 { (x, y) } else { (y, x) };
    let r = plain(sk, t)?;
    if t.cmp != Cmp::Le || !(r > 0.0) || !ball_centres(sk, centres, faces, r) {
        return None;
    }
    let (mut sides, mut at) = (Vec::new(), Vec::new());
    for u in touch {
        if let Some((f, c)) = side_of(sk, u) {
            if (c == Cmp::Le) != faces.within {
                return None;
            }
            sides.push(f);
            continue;
        }
        let (f, c) = crate::solid::faces::measured_face(u)?;
        let (lo, hi) = (u.lo.read(sk)?, u.hi?);
        let at_r = lo == hi && (lo - r).abs() <= 1e-12 * r;
        if c != 1.0 || u.cmp != Cmp::In || u.grow.is_some() || !at_r {
            return None;
        }
        at.push(f);
    }
    (faces.all(&sides, false) && faces.all(&at, true)).then_some(r)
}

/// Whether a set is where the centre of a ball of radius `r` touching every face may stand: on
/// the region's side of each face, at least `r` from its surface.
fn ball_centres(sk: &Sketch, terms: &[RegionTerm], faces: &Faces, r: f64) -> bool {
    let (mut sides, mut reach) = (Vec::new(), Vec::new());
    for t in terms {
        match (side_of(sk, t), one_face(sk, t)) {
            (Some((f, c)), _) if (c == Cmp::Le) == faces.within => sides.push(f),
            (_, Some((f, Cmp::Ge, d))) if (d - r).abs() <= 1e-12 * r => reach.push(f),
            _ => return false,
        }
    }
    faces.all(&sides, false) && faces.all(&reach, true)
}
