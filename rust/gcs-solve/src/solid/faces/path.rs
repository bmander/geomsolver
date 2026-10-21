//! **A face's name, and what it is analytically** (#188, #212): the path a body names a face by
//! (`FacePath`), the faces of a solid by their paths read off the definitions (`face_paths`), and
//! the analytic surface a path reads back to (`face_source`).
use crate::model::{Sketch, SolidDef};

/// **A face's name, as a body names it** (#188): the route through the body's operands to the
/// primitive the face is a piece of (`plate`, a placed copy's `turned`, none for the body's own
/// sweep, `document::operand_primitives`) and the face's own name in that primitive (`near`, an
/// edge's `ab`).  Produced by the one walk the exact boundary's faces are named by, and matched
/// by the one rule, `matches`.
#[derive(Clone, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct FacePath {
    pub route: Vec<String>,
    pub face: String,
    /// Where a round of the body made it, the use that did (`use_key`: `rim#17`), as the faces a
    /// block runs over list it (`face_paths`, [0.149]); `None` for any other face, and for every
    /// face read off a boundary.
    pub of: Option<String>,
}

impl FacePath {
    /// The face `face` of the primitive at `route` (dotted, empty for the solid itself).
    pub fn new(route: &str, face: &str) -> FacePath {
        let route = route.split('.').filter(|s| !s.is_empty()).map(str::to_string).collect();
        FacePath { route, face: face.to_string(), of: None }
    }

    /// Whether a block over the faces of `path`, standing at `at` (an absolute prefix), runs over
    /// this one (`matches`): a face a round of the body made only where the path names that round
    /// by the label its use was given (`rim := …`, then `plate.rim`, [0.149]), and not inside that
    /// use — so `round(r) block` never rounds the faces it makes, nor another round's it does not
    /// name.
    pub fn listed_by<S: AsRef<str>>(&self, path: &[S], at: &str) -> bool {
        if !self.matches(path) {
            return false;
        }
        let Some(of) = &self.of else { return true };
        let labels = &self.route[..self.route.len().saturating_sub(1)];
        names_use(path, labels) && !at.starts_with(&format!("{of}."))
    }

    /// Its segments: the route's, then the face's own name.
    pub fn segments(&self) -> impl Iterator<Item = &str> {
        self.route.iter().map(String::as_str).chain(self.face.split('.').filter(|s| !s.is_empty()))
    }

    /// Whether a path written past the solid names this face: its segments, from the first, are
    /// this face's — `plate` every face `plate` gave the body, `plate.near` its near cap, the
    /// empty path every face.  Never by a suffix: `near` alone is the body's own sweep's cap,
    /// and an instance's operand is reached from the instance's solid by its route
    /// (`pad.plate.near` is `pad`'s `plate.near`, whatever `plate`'s absolute name).  A segment
    /// a copy's edge named (`hole[1]`) is named by its index or, written without one, with every
    /// other copy's (`flange.drill.hole` is each hole the drills cut).
    pub fn matches<S: AsRef<str>>(&self, path: &[S]) -> bool {
        let mine: Vec<&str> = self.segments().collect();
        path.len() <= mine.len() && path.iter().zip(&mine).all(|(a, b)| same_step(a.as_ref(), b))
    }

    /// As written in a path (`plate.near`).
    pub fn text(&self) -> String { self.segments().collect::<Vec<_>>().join(".") }
}

/// **The faces of a solid by the paths they are named by from it** (#183 F4): each primitive's
/// own (the edges of its section, then `SolidE::caps`) under its route
/// (`document::operand_primitives`, the walk `solid::exact` names faces by, placed copies
/// included) — read off the definitions, so a face a boolean ate is still listed, and reads as
/// nothing.
pub fn face_paths(sk: &Sketch, i: usize) -> Vec<FacePath> {
    let own = |p: usize| -> Vec<String> {
        let s = &sk.solids[p];
        // a sweep's faces are its cutter's, each the face it made, and a region's its terms', each
        // by the set it bounds (#212)
        match &s.def {
            SolidDef::Swept { source, .. } => {
                return face_paths(sk, *source as usize).iter().map(FacePath::text).collect()
            }
            SolidDef::Region { terms, .. } => {
                // a round's own bounds that no set names (`p inside a`) lie on the faces it
                // rounds: no face of its own a path could write ([0.149])
                let reads = super::terms_read_faces(terms);
                let mut v: Vec<String> = terms.iter().map(|t| t.name.clone())
                    .filter(|n| !(reads && n.is_empty())).collect();
                v.dedup();
                return v;
            }
            _ => {}
        }
        let mut v: Vec<String> = s.face().and_then(|f| sk.faces.get(f as usize)).map_or(Vec::new(), |f| {
            f.boundaries().flat_map(|(_, n)| n.iter().cloned()).collect()
        });
        v.extend(s.caps().iter().map(|c| c.to_string()));
        v.dedup();
        v
    };
    if i >= sk.solids.len() {
        return Vec::new();
    }
    let readers = own_readers(sk, i);
    crate::solid::document::operand_primitives(sk, i).into_values().flat_map(|(route, p)| {
        let of = readers.contains(&(p as usize)).then(|| use_key(&sk.solids[p as usize].name).to_string());
        // a round of the body no name was given makes faces no block lists: none, so the faces
        // are named as before it was made, and settle a pass sooner
        let unnamed = of.is_some() && !route.contains('.');
        own(p as usize).into_iter().filter(move |_| !unnamed)
            .map(move |f| FacePath { of: of.clone(), ..FacePath::new(&route, &f) })
    }).collect()
}

/// Whether segment `written` of a face path names segment `mine` of a face's name: the same, or
/// `mine` a copy of it (`hole` names `hole[1]`; `hole[1]` names only itself).
fn same_step(written: &str, mine: &str) -> bool {
    let copy = |k: &str| !k.is_empty() && k.bytes().all(|b| b.is_ascii_digit());
    written == mine || mine.strip_prefix(written).and_then(|r| r.strip_prefix('['))
        .and_then(|r| r.strip_suffix(']')).is_some_and(copy)
}

/// The solids that read body `i`'s faces, and those of each layer it stands on (a stock of its
/// own name, `layers`).
fn own_readers(sk: &Sketch, i: usize) -> std::collections::BTreeSet<usize> {
    let mut out = super::readers_of(sk, i);
    let mut at = i;
    while let Some(SolidDef::Body { stock, .. }) = sk.solids.get(at).map(|s| &s.def) {
        let stock = *stock as usize;
        if stock == at || sk.solids[stock].name != sk.solids[at].name {
            break;
        }
        out.extend(super::readers_of(sk, stock));
        at = stock;
    }
    out
}

/// The statement a solid was made by, as a key: its name up to the first segment a statement's
/// use put there (`#17`, or a named use's `rim#17`), or its whole name where none did.
pub(crate) fn use_key(name: &str) -> &str {
    let mut end = 0;
    for seg in name.split('.') {
        end += seg.len();
        if seg.contains('#') {
            return &name[..end];
        }
        end += 1;
    }
    name
}

/// Whether a face path written past a body names faces a use labelled `labels` made: it reaches
/// into the label, segment by segment, as far as either goes (`plate.rim`, `plate.rim.taken.round`).
/// A path that names the whole body, or reaches the round through another operand, does not.
pub(crate) fn names_use<S: AsRef<str>>(path: &[S], labels: &[String]) -> bool {
    !path.is_empty() && !labels.is_empty() && path.iter().zip(labels).all(|(a, b)| a.as_ref() == b)
}

/// **What a face of a body is, analytically** (#212): read off the definitions along the face's
/// route, never off a B-rep, so a check holding a built body's faces to it is independent of the
/// kernel that built them.
#[derive(Clone, Debug)]
pub enum FaceSource {
    /// A face of revolution: of a revolution's profile, or of a region's meridian (a term's).
    Revolved { surface: crate::solid::RevolvedSurface, solid: usize },
    /// The envelope a cutter's face sweeps: the face (carried to where the cutter's own
    /// placements put it, `within`; `solid` the revolution it is a face of) under the sweep's
    /// motion over its roll, and where the cut that leaves it is placed (`pose`).
    Swept { surface: crate::solid::RevolvedSurface, solid: usize, within: crate::envelope::Motion,
        motion: crate::motion::Family, roll: [f64; 2], pose: crate::envelope::Motion },
}

/// The analytic face `path` names in solid `body` (`FaceSource`): the route walked through the
/// body's operands by their names as `document::operand_primitives` routes them (a placement's
/// pose carried, the first operand of a name taken where several share it, as the copies of one
/// cut do), then the face named in the primitive it reaches — a revolution's profile edge, a
/// region's term, or a sweep's cutter's face by its own path in the cutter.
pub fn face_source(sk: &Sketch, body: usize, path: &FacePath) -> Result<FaceSource, String> {
    let (p, pose) = face_reach(sk, body, &path.route)?;
    let s = &sk.solids[p];
    let name = format!("{}.{}", s.name, path.face);
    match &s.def {
        SolidDef::Swept { source, motion, .. } => {
            // the cutter's face, by its own path in the cutter (`outer_crown.inner`), where the
            // cutter's placements put it
            let (route, face) = path.face.rsplit_once('.').unwrap_or(("", path.face.as_str()));
            let (solid, within) = face_reach(sk, *source as usize, &FacePath::new(route, face).route)?;
            let surface = revolved(sk, solid, face, within, name).map_err(|why| match why {
                Some(why) => why,
                None => format!("`{}` sweeps a face that is no face of revolution", s.name),
            })?;
            let motion = crate::motion::Family::read(sk, *motion as usize)?;
            let roll = crate::solid::roll::interval(sk, p)?;
            Ok(FaceSource::Swept { surface, solid, within, motion, roll, pose })
        }
        _ => {
            let surface = revolved(sk, p, &path.face, pose, name)
                .map_err(|why| why.unwrap_or_else(|| format!("`{}`'s faces are not read analytically", s.name)))?;
            Ok(FaceSource::Revolved { surface, solid: p })
        }
    }
}

/// Face `face` of primitive `p` (a revolution's profile edge, a region's term) as the surface it
/// lies on, at `pose` and called `name`; `Err(None)` where `p` has no faces of revolution.
fn revolved(sk: &Sketch, p: usize, face: &str, pose: crate::envelope::Motion, name: String)
    -> Result<crate::solid::RevolvedSurface, Option<String>> {
    let s = &sk.solids[p];
    let none = || Some(format!("`{}` has no face `{face}`", s.name));
    let mut surface = match &s.def {
        SolidDef::Revolve { face: profile, .. } => {
            let f = sk.faces.get(*profile as usize).ok_or(Some("no profile".to_string()))?;
            let edge = f.boundaries().find_map(|(es, ns)| es.iter().zip(ns).find(|(_, n)| **n == face).map(|(e, _)| *e))
                .ok_or_else(none)?;
            crate::solid::RevolvedSurface::read(sk, p, edge).map_err(Some)?
        }
        SolidDef::Region { .. } => {
            // the term's face by its name in this region, whatever the face is called where it
            // is read (a sweep's face is named under the sweep)
            let m = crate::solid::region::meridian(sk, p).map_err(Some)?;
            let own = format!("{}.{face}", s.name);
            crate::solid::RevolvedSurface::of_meridian(&m, &s.name).into_iter().find(|f| f.name == own).ok_or_else(none)?
        }
        _ => return Err(None),
    }.placed(pose);
    surface.name = name;
    Ok(surface)
}

/// The primitive a route reaches from solid `i`, and the pose the placements along it put it at.
fn face_reach(sk: &Sketch, i: usize, route: &[String]) -> Result<(usize, crate::envelope::Motion), String> {
    let s = sk.solids.get(i).ok_or("no such solid")?;
    match &s.def {
        SolidDef::Body { .. } => {
            for o in s.operands().into_iter().map(|o| o as usize) {
                let operand = &sk.solids[o];
                // a swept solid's own sweep, split off as its stock, adds no step
                if operand.name == s.name {
                    if let Ok(found) = face_reach(sk, o, route) { return Ok(found) }
                    continue;
                }
                if let Some(rest) = crate::solid::past_step(&operand.name, route) {
                    return face_reach(sk, o, rest);
                }
            }
            Err(format!("`{}` has no operand `{}`", s.name, route.join(".")))
        }
        SolidDef::Placed { source, motion, at } => {
            let (p, inner) = face_reach(sk, *source as usize, route)?;
            let outer = crate::motion::Family::read(sk, *motion as usize)?.at(at.value)?;
            Ok((p, inner.then(outer)))
        }
        _ if route.is_empty() => Ok((i, crate::envelope::Motion::identity())),
        _ => Err(format!("`{}` is a primitive, with no operand `{}`", s.name, route.join("."))),
    }
}
