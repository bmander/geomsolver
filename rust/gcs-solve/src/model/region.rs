//! A region solid's terms (§6.21): what each bound on its probe measures (`RegionShape`),
//! a face or a set it measures from (`FaceRef`, `Target`), and its numbers (`TermNumber`).

use super::*;

/// What a region solid's term bounds (§6.21): the measure a bound on its probe reads, recognised
/// by the constraint's kind and never by the set it came from.
#[derive(Clone, Debug, PartialEq)]
pub enum RegionShape {
    /// A distance from a point: a ball about it.
    Ball { center: u32 },
    /// A distance from a line: a cylinder about it.
    Cylinder { line: u32 },
    /// An angle from a line's direction, seen from its start: a cone, apex the line's `p1`.
    Cone { axis: u32 },
    /// An ordinate from a point along a plane's normal or a line's direction (`along`, a plane,
    /// a line or an axis: `RegionShape::normal`): a half-space.  `sign` is -1 where the probe is
    /// the ordinate's first operand, read back from it.
    Plane { along: EntRef, from: u32, sign: f64 },
    /// A sum of measurements of the probe, each times its number (§6.21, #183 F3): `distance(p,
    /// f1) + distance(p, f2) <= 10mm` is `[(f1, 1), (f2, 1)]` against 10.  Each is the distance
    /// from what it names (`Target`).
    Measured { of: Vec<(Target, f64)> },
    /// The signed distance from a face's surface (#183 F1), positive on the side away from the
    /// body's material: `p inside F` at most nothing, `p outside F` at least, `p distance(<= d,
    /// side: outside) F` at most `d`.
    FaceSide { face: FaceRef },
    /// The material's angle at the edge where faces `a` and `b` meet, at the point of it nearest
    /// the probe (#183 F9), in radians: under a half turn where the edge is convex.
    Angle { a: FaceRef, b: FaceRef },
    /// Where an edge ends (#183 F7): on the side the edge runs on of each face it ends on.  The
    /// edge is a set, the points on two faces (`Target::Set`), resolved when the solid is built.
    Ends { edge: Box<Target> },
    /// **A developable panel's band** (#645): the signed distance along the panel's normal from a
    /// foot on it, `side` the way it is read (1 its outside, −1 its inside) — `p distance(<= d,
    /// side: outside) panel` the band from the panel to `d` out, square at its edges, for there is
    /// no foot past them.  The panel is the one a plane touches along each ruling between `rails`
    /// (`developable`), recognised by what its set says.
    Sheet { rails: Box<[super::Rail; 2]>, side: f64 },
}

impl RegionShape {
    /// A half-space's unit normal as the drawing stands: its plane's `n`, its line's or its
    /// axis's direction — `None` for another shape, or a line of no length.
    pub fn normal(&self, sk: &Sketch) -> Option<[f64; 3]> {
        let RegionShape::Plane { along, .. } = *self else { return None };
        match along.kind {
            EntKind::Frame => Some(sk.basis(along.i()).normal()),
            EntKind::Line => {
                let l = sk.lines.get(along.i())?;
                let (a, b) = (sk.world_point(l.p1 as usize), sk.world_point(l.p2 as usize));
                crate::space::normalised(crate::space::sub(b, a))
            }
            EntKind::Axis => {
                let d = sk.axes.get(along.i())?.d.map(|q| sk.params[q as usize].value);
                crate::space::normalised(d)
            }
            _ => None,
        }
    }

    /// The entities its measure reads.
    pub fn entities(&self) -> Vec<EntRef> {
        match *self {
            RegionShape::Ball { center } => vec![EntRef::point(center as usize)],
            RegionShape::Cylinder { line } | RegionShape::Cone { axis: line } => {
                vec![EntRef::line(line as usize)]
            }
            RegionShape::Plane { along, from, .. } => vec![along, EntRef::point(from as usize)],
            RegionShape::Measured { ref of } => of.iter().flat_map(|(t, _)| t.entities()).collect(),
            RegionShape::FaceSide { ref face } => vec![EntRef::solid(face.solid as usize)],
            RegionShape::Angle { ref a, ref b } => {
                vec![EntRef::solid(a.solid as usize), EntRef::solid(b.solid as usize)]
            }
            RegionShape::Ends { ref edge } => edge.entities(),
            RegionShape::Sheet { ref rails, .. } => {
                rails.iter().flat_map(|r| r.links.iter().copied()).collect()
            }
        }
    }

    /// The same measure over the entities `map` renumbers them to, or `None` where one went.
    pub fn remapped(&self, map: &Remap) -> Option<RegionShape> {
        let i = |kind: EntKind, at: u32| renumbered(map, kind, at);
        Some(match *self {
            RegionShape::Ball { center } => {
                RegionShape::Ball { center: i(EntKind::Point, center)? }
            }
            RegionShape::Cylinder { line } => {
                RegionShape::Cylinder { line: i(EntKind::Line, line)? }
            }
            RegionShape::Cone { axis } => RegionShape::Cone { axis: i(EntKind::Line, axis)? },
            RegionShape::Plane { along, from, sign } => RegionShape::Plane {
                along: map(along)?,
                from: i(EntKind::Point, from)?,
                sign,
            },
            RegionShape::Measured { ref of } => RegionShape::Measured {
                of: of.iter().map(|(t, c)| t.remapped(map).map(|t| (t, *c))).collect::<Option<_>>()?,
            },
            RegionShape::FaceSide { ref face } => RegionShape::FaceSide { face: face.remapped(map)? },
            RegionShape::Angle { ref a, ref b } => {
                RegionShape::Angle { a: a.remapped(map)?, b: b.remapped(map)? }
            }
            RegionShape::Ends { ref edge } => {
                RegionShape::Ends { edge: Box::new(edge.remapped(map)?) }
            }
            RegionShape::Sheet { ref rails, side } => RegionShape::Sheet {
                rails: Box::new([rails[0].remapped(map)?, rails[1].remapped(map)?]),
                side,
            },
        })
    }
}

/// A face of a solid, or the faces, as a set's body names it (#183 F1): `pad.plate.near` is
/// solid `pad`'s faces that its operand `plate` gave it as its near cap; `pad.boss` every face
/// `boss` gave it; `block` every face of `block`.  Read off the solid's exact boundary, as the
/// faces stand in it (a boolean never renames).  `surface` reads the face's whole surface
/// (`F.surface`), not its patch; `smooth` that surface carried on across its tangent seams
/// (`F.smooth`).  `path` is the segments written past the solid, matched
/// against each face's `solid::faces::FacePath` (#188).
#[derive(Clone, Debug, PartialEq)]
pub struct FaceRef {
    pub solid: u32,
    pub path: Vec<String>,
    pub surface: bool,
    /// `F.smooth` ([0.149]): the faces named and every face of the body going on from them
    /// tangent across a seam, read as one smooth surface — a distance from it, or a side of it,
    /// the one of the piece whose patch is nearest (`solid::faces::Reading`).  Read with
    /// `surface`, the pieces' surfaces.
    pub smooth: bool,
}

/// What a measurement of a region's probe measures the distance from (#183 F1–F3).
#[derive(Clone, Debug, PartialEq)]
pub enum Target {
    /// A point, a line produced, an axis or a plane.
    Ent(EntRef),
    /// A face's patch, or its surface.
    Face(FaceRef),
    /// A set (#183 F2): the points its body holds, applied to a hidden point of its own whose
    /// bounds are `terms` (filled when the solid is built).
    Set { probe: u32, terms: Vec<RegionTerm> },
}

impl Target {
    pub fn entities(&self) -> Vec<EntRef> {
        match self {
            Target::Ent(e) => vec![*e],
            Target::Face(f) => vec![EntRef::solid(f.solid as usize)],
            Target::Set { terms, .. } => terms.iter().flat_map(|t| t.shape.entities()).collect(),
        }
    }

    pub fn remapped(&self, map: &Remap) -> Option<Target> {
        Some(match self {
            Target::Ent(e) => Target::Ent(map(*e)?),
            Target::Face(f) => Target::Face(f.remapped(map)?),
            Target::Set { probe, terms } => Target::Set {
                probe: renumbered(map, EntKind::Point, *probe)?,
                terms: terms.iter().map(|t| t.remapped(map)).collect::<Option<_>>()?,
            },
        })
    }
}

impl FaceRef {
    pub fn remapped(&self, map: &Remap) -> Option<FaceRef> {
        let solid = renumbered(map, EntKind::Solid, self.solid)?;
        Some(FaceRef { solid, ..self.clone() })
    }
}

impl RegionTerm {
    /// The one-sided bound `cmp` on `shape` at `lo`, of no set and no offset.
    pub fn new(shape: RegionShape, cmp: crate::vocab::Cmp, lo: TermNumber) -> RegionTerm {
        RegionTerm { shape, cmp, lo, hi: None, grow: None, name: String::new() }
    }

    /// The same term over the entities `map` renumbers them to, or `None` where one went.
    pub fn remapped(&self, map: &Remap) -> Option<RegionTerm> {
        Some(RegionTerm { shape: self.shape.remapped(map)?, ..self.clone() })
    }

    /// Its low end and its offset (nothing where it has none) as the drawing stands
    /// (`TermNumber::value`): refused where either reads an unknown nothing determines.
    pub fn value(&self, sk: &Sketch) -> Result<(f64, f64), String> {
        Ok((self.lo.value(sk)?, self.grow_value(sk)?))
    }

    /// `value`, `None` where nothing determines either.
    pub fn read(&self, sk: &Sketch) -> Option<(f64, f64)> {
        Some((self.lo.read(sk)?, self.grow_read(sk)?))
    }

    /// Its offset as the drawing stands, nothing where it has none (`TermNumber::value`).
    pub fn grow_value(&self, sk: &Sketch) -> Result<f64, String> {
        self.grow.as_ref().map_or(Ok(0.0), |g| g.value(sk))
    }

    /// `grow_value`, `None` where nothing determines it.
    pub fn grow_read(&self, sk: &Sketch) -> Option<f64> {
        self.grow.as_ref().map_or(Some(0.0), |g| g.read(sk))
    }
}

/// One bound of a region solid: its measure, which way, and its numbers (a length or an angle in
/// radians) — the low end and an interval's high.  `name` is the set the bound came from, where
/// it came from one: what the faces it makes are called.  `grow` is an offset (§6.21, F4): the
/// bound is on the signed distance from the surface the measure's number makes, positive where
/// the measure exceeds it, compared with `grow` — the set grown by it, or shrunk where it is
/// negative.  Only a cone's is kept as one; a distance's is its number moved by it.
#[derive(Clone, Debug, PartialEq)]
pub struct RegionTerm {
    pub shape: RegionShape,
    pub cmp: crate::vocab::Cmp,
    pub lo: TermNumber,
    pub hi: Option<f64>,
    pub grow: Option<TermNumber>,
    pub name: String,
}

/// A region term's number (§6.21): stated, or affine in one of the drawing's unknowns — `c + m ·
/// x` in the kernels' units, `x` the unknown by its name — read once the drawing is solved, as a
/// sphere's radius is where a point stated on it sets it.
#[derive(Clone, Debug, PartialEq)]
pub struct TermNumber {
    pub c: f64,
    pub free: Option<(String, f64)>,
}

impl TermNumber {
    /// A stated number.
    pub fn stated(c: f64) -> TermNumber {
        TermNumber { c, free: None }
    }

    /// What it reads as the drawing stands: refused where it reads an unknown no row of the
    /// drawing holds, which nothing then determines.
    pub fn value(&self, sk: &Sketch) -> Result<f64, String> {
        self.read(sk).ok_or_else(|| {
            let name = self.free.as_ref().map_or("", |(n, _)| n.as_str());
            format!("`{name}` is an unknown nothing determines: state a point on the set it sizes")
        })
    }

    /// `value`, `None` where nothing determines it.
    pub fn read(&self, sk: &Sketch) -> Option<f64> {
        let Some((name, m)) = &self.free else { return Some(self.c) };
        let x = sk.free_vars.get(name).map(|&p| &sk.params[p as usize]).filter(|p| !p.fixed)?;
        Some(self.c + m * x.value)
    }
}
