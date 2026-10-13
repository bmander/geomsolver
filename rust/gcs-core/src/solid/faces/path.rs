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
}

impl FacePath {
    /// The face `face` of the primitive at `route` (dotted, empty for the solid itself).
    pub fn new(route: &str, face: &str) -> FacePath {
        let route = route.split('.').filter(|s| !s.is_empty()).map(str::to_string).collect();
        FacePath { route, face: face.to_string() }
    }

    /// Its segments: the route's, then the face's own name.
    pub fn segments(&self) -> impl Iterator<Item = &str> {
        self.route.iter().map(String::as_str).chain(self.face.split('.').filter(|s| !s.is_empty()))
    }

    /// Whether a path written past the solid names this face: its segments, from the first, are
    /// this face's — `plate` every face `plate` gave the body, `plate.near` its near cap, the
    /// empty path every face.  Never by a suffix: `near` alone is the body's own sweep's cap,
    /// and an instance's operand is reached from the instance's solid by its route
    /// (`pad.plate.near` is `pad`'s `plate.near`, whatever `plate`'s absolute name).
    pub fn matches<S: AsRef<str>>(&self, path: &[S]) -> bool {
        let mine: Vec<&str> = self.segments().collect();
        path.len() <= mine.len() && path.iter().zip(&mine).all(|(a, b)| a.as_ref() == *b)
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
                let mut v: Vec<String> = terms.iter().map(|t| t.name.clone()).collect();
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
    crate::solid::document::operand_primitives(sk, i).into_values().flat_map(|(route, p)| {
        own(p as usize).into_iter().map(move |f| FacePath::new(&route, &f))
    }).collect()
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
                let short = operand.name.rsplit('.').next().unwrap_or(&operand.name);
                if route.first().is_some_and(|r| r == short) {
                    return face_reach(sk, o, &route[1..]);
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
