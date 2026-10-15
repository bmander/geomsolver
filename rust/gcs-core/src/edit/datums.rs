//! A datum dropped by a button, constrained to what is selected, and a plane on a face.

use super::*;

/// `a word b`, written as the source says it: the elaborator reads which relation the word means
/// off the operands' kinds (`infix_op`), as it does for anything typed.
fn written(s: Said) -> StmtKind {
    let form = syntax::Written {
        word: syntax::Name::new(s.word),
        fixity: crate::constraints::Fixity::Infix,
        ops: vec![syntax::Ref::new(s.a), syntax::Ref::new(s.b)],
        args: s.args,
        span: Span::default(),
        rates: Vec::new(),
    };
    StmtKind::Relation(syntax::Relation::of(syntax::RelationForm::Written(form)))
}

/// A line or an axis, where it is in space: a point on it and its direction.
struct Ray {
    name: String,
    axis: bool,
    at: [f64; 3],
    dir: [f64; 3],
}

/// What is selected when a datum is dropped, sorted by kind, each where the drawing has it.
struct Selection {
    points: Vec<(String, [f64; 3])>,
    rays: Vec<Ray>,
    planes: Vec<(String, crate::plane::Basis)>,
}

impl Selection {
    fn read(e: &Elaborated, sk: &Sketch, names: &[String]) -> Result<Selection, String> {
        let mut out = Selection { points: Vec::new(), rays: Vec::new(), planes: Vec::new() };
        let param = |q: u32| sk.params[q as usize].value;
        for n in names {
            let r = e.map.ent_named(n).ok_or_else(|| format!("`{n}` is nothing in this drawing"))?;
            match r.kind {
                EntKind::Point => out.points.push((n.clone(), sk.world_point(r.i()))),
                EntKind::Line => {
                    let l = &sk.lines[r.i()];
                    let (a, b) = (sk.world_point(l.p1 as usize), sk.world_point(l.p2 as usize));
                    out.rays.push(Ray { name: n.clone(), axis: false, at: a, dir: crate::space::sub(b, a) });
                }
                EntKind::Axis => {
                    let ax = &sk.axes[r.i()];
                    out.rays.push(Ray { name: n.clone(), axis: true, at: ax.a.map(param), dir: ax.d.map(param) });
                }
                EntKind::Plane => out.planes.push((n.clone(), sk.basis(r.i()))),
                other => return Err(format!("{} does not place a plane or an axis: select points, \
                                             lines, axes or planes", other.a())),
            }
        }
        Ok(out)
    }
}

/// A relation a datum is dropped with: `v0 coincident p`, `v0 distance(20) v1`.
struct Said {
    a: String,
    word: &'static str,
    args: Vec<syntax::OpArg>,
    b: String,
}

/// `a word b`, with nothing in the word's parentheses.
fn said(a: impl Into<String>, word: &'static str, b: impl Into<String>) -> Said {
    Said { a: a.into(), word, args: Vec::new(), b: b.into() }
}

/// A datum dropped by a button (#162 F1): a free plane or axis, constrained to what is selected.
///
/// | selected          | a plane `v0`                         | an axis `x0`                     |
/// |-------------------|--------------------------------------|----------------------------------|
/// | nothing           | free, parallel to `current`, set off | free, square to `current`        |
/// | a point           | through it                           | through it, square to `current`  |
/// | two points        | —                                    | through both                     |
/// | three points      | through all three                    | —                                |
/// | a line or an axis | —                                    | along it                         |
/// | a point and a ray | through the point, square to the ray | —                                |
/// | two rays          | holding both                         | —                                |
/// | a plane           | parallel to it, set off              | its normal through its origin    |
/// | a point, a plane  | —                                    | its normal through the point     |
/// | two planes        | —                                    | where they meet                  |
///
/// A ray is a drawn line or an axis.  Every relation is an ordinary statement (`v0 coincident p`,
/// `t perpendicular v0`), its kind the elaborator's to read off the operands, so the datum is
/// constrained further like anything else; the seeds are read off the drawing `sk`, so the first
/// solve starts at the answer meant.  `current` is the plane being drawn on (`None` the page).
/// One splice, held to the elaborator.
pub fn add_datum(
    e: &Elaborated,
    sk: &Sketch,
    kind: EntKind,
    from: &[String],
    current: Option<&str>,
    name: Option<&str>,
) -> Edit {
    Edit::refused_or(&e.program, datum_edit(e, sk, kind, from, current, name))
}

fn datum_edit(
    e: &Elaborated,
    sk: &Sketch,
    kind: EntKind,
    from: &[String],
    current: Option<&str>,
    name: Option<&str>,
) -> Result<Edit, String> {
    let sel = Selection::read(e, sk, from)?;
    let current = match current {
        Some(n) => match e.map.ent_named(n) {
            Some(r) if r.kind == EntKind::Plane => sk.basis(r.i()),
            _ => return Err(format!("`{n}` is not a plane in this drawing")),
        },
        None => crate::plane::Basis::page(),
    };
    let name = chosen_name(&mut taken_names(&e.program), name, kind)?;
    let (decl, said) = if kind == EntKind::Plane {
        plane_datum(sk, &sel, &current, &name)?
    } else {
        axis_datum(sk, &sel, &current, &name)?
    };
    let mut stmts = vec![StmtKind::Decl(decl)];
    stmts.extend(said.into_iter().map(written));
    append_checked(&e.program, &stmts, vec![name])
}

/// How far a datum with nothing to stand on is set off what it is parallel to: a round fifth of
/// the drawing's reach, so it is not lying on it (W113).
pub(super) fn set_off(sk: &Sketch) -> f64 {
    round_sig(crate::overview::axis_reach(sk) / 5.0, 1)
}

/// `p coincident x`, for each of the points: an axis through each.
fn through(points: &[(String, [f64; 3])], x: &str) -> Vec<Said> {
    points.iter().map(|(p, _)| said(p.clone(), "coincident", x)).collect()
}

/// The plane's row of `add_datum`'s table: its declaration, seeded with the frame the rule reads
/// off the drawing, and what it is said to hold.
fn plane_datum(sk: &Sketch, sel: &Selection, current: &crate::plane::Basis, name: &str)
    -> Result<(Decl, Vec<Said>), String> {
    use crate::plane::Basis;
    use crate::space::{across, add, scale, sub};
    let Selection { points, rays, planes } = sel;
    let off = |b: &Basis| Basis { o: add(b.o, scale(b.normal(), set_off(sk))), ..*b };
    let (frame, mut rest): (Basis, Vec<Said>) = match (points.len(), rays.len(), planes.len()) {
        (0, 0, 0) => (off(current), Vec::new()),
        (1, 0, 0) => (Basis { o: points[0].1, ..*current }, Vec::new()),
        (3, 0, 0) => {
            let (p, q, r) = (points[0].1, points[1].1, points[2].1);
            crate::plane::unit(sub(q, p))
                .ok_or("two of the points are in one place: they make no plane")?;
            let b = Basis::explicit(sub(q, p), sub(r, p))
                .ok_or("the three points lie on a line: they make no single plane")?;
            (Basis { o: p, ..b }, Vec::new())
        }
        (1, 1, 0) => {
            let r = &rays[0];
            let n = crate::plane::unit(r.dir)
                .ok_or("the line has no length: it says no direction")?;
            let (u, v) = across(n);
            // square to an axis outright; to a drawn line by standing square to both the plane's
            // own axes, there being no line-square-to-a-plane relation
            let square = if r.axis {
                vec![said(r.name.clone(), "perpendicular", name)]
            } else {
                let to = |a: &str| said(r.name.clone(), "perpendicular", format!("{name}.{a}"));
                vec![to("u"), to("v")]
            };
            (Basis { u, v, o: points[0].1 }, square)
        }
        (0, 2, 0) => {
            // along the second, or — where the two run alike — across the gap between them
            let (a, b) = (&rays[0], &rays[1]);
            let f = Basis::explicit(a.dir, b.dir)
                .or_else(|| Basis::explicit(a.dir, sub(b.at, a.at)))
                .ok_or("the two lie along one line: they make no single plane")?;
            let held = rays.iter().map(|r| said(r.name.clone(), "coincident", name));
            (Basis { o: a.at, ..f }, held.collect())
        }
        // parallel, and as far off as it is set: the offset stated, so it is sized at once
        (0, 0, 1) => {
            let (p, b) = &planes[0];
            let d = syntax::OpArg::Dim(syntax::num(set_off(sk)), Span::default());
            let apart = Said { args: vec![d], ..said(p.clone(), "distance", name) };
            (off(b), vec![said(name, "parallel", p.clone()), apart])
        }
        _ => return Err("a plane is dropped with nothing selected, or through a point, three \
                         points, a point and a line or axis, two lines or axes, or a plane"
            .into()),
    };
    // the points the plane passes through come first, as they are selected: the plane through
    // each, a row, so it may lie on a point's own plane — `a coincident v0` would draw the point
    // in it too (§6.7)
    let mut all: Vec<Said> =
        points.iter().map(|(p, _)| said(name, "coincident", p.clone())).collect();
    all.append(&mut rest);
    let seed = |d: [f64; 3]| vec![syntax::Kid::Hint(syntax::KidSeed { v: seed_round(d), axis: true,
        ..Default::default() })];
    let decl = gesture_decl(EntKind::Plane, name, vec![seed(frame.u), seed(frame.v), Vec::new()],
        &seed_round(frame.o));
    Ok((decl, all))
}

/// The axis's row of `add_datum`'s table: its declaration, seeded with the direction and a place
/// the rule reads off the drawing, and what it is said to pass through or stand square to.
fn axis_datum(sk: &Sketch, sel: &Selection, current: &crate::plane::Basis, name: &str)
    -> Result<(Decl, Vec<Said>), String> {
    use crate::space::{add, cross, dot, scale, sub};
    let Selection { points, rays, planes } = sel;
    let unit = |v: [f64; 3], why: &'static str| crate::plane::unit(v).ok_or(why);
    let n = name.to_string();
    type Row = ([f64; 3], [f64; 3], Vec<Said>);
    let (dir, at, said): Row = match (points.len(), rays.len(), planes.len()) {
        (0, 0, 0) => (current.normal(), add(current.o, scale(current.u, set_off(sk))), Vec::new()),
        (1, 0, 0) => (current.normal(), points[0].1, Vec::new()),
        (2, 0, 0) => {
            let d = unit(sub(points[1].1, points[0].1),
                "the two points are in one place: they say no direction")?;
            (d, points[0].1, Vec::new())
        }
        (0, 1, 0) => {
            let r = &rays[0];
            let along = vec![said(r.name.clone(), "coincident", n.clone())];
            (unit(r.dir, "the line has no length")?, r.at, along)
        }
        (1, 0, 1) => (planes[0].1.normal(), points[0].1,
            vec![said(n.clone(), "perpendicular", planes[0].0.clone())]),
        (0, 0, 1) => {
            let (p, b) = &planes[0];
            (b.normal(), b.o, vec![said(n.clone(), "perpendicular", p.clone()),
                                   said(format!("{p}.origin"), "coincident", n.clone())])
        }
        (0, 0, 2) => {
            let (a, b) = (&planes[0].1, &planes[1].1);
            let d = unit(cross(a.normal(), b.normal()),
                "the two planes are parallel: they do not meet")?;
            let at = crate::csg::meet(a.normal(), a.along_normal(), b.normal(), b.along_normal(), d)
                .ok_or("the two planes are parallel: they do not meet")?;
            (d, at, planes.iter().map(|(p, _)| said(n.clone(), "coincident", p.clone())).collect())
        }
        _ => return Err("an axis is dropped with nothing selected, or through a point, two points, \
                         along a line or an axis, square to a plane, a plane through a point, or \
                         where two planes meet"
            .into()),
    };
    let mut all = through(points, name);
    all.extend(said);
    // an axis's place is the point on it nearest the world's origin
    let foot = sub(at, scale(dir, dot(at, dir)));
    let mut seed = seed_round(dir).to_vec();
    seed.extend(seed_round(foot));
    Ok((gesture_decl(EntKind::Axis, name, Vec::new(), &seed), all))
}

/// **Sketch on a face** (#162 F5): a plane lying on the face `face` of object `solid` (the path
/// the face is named by, `b0.near`), its origin on the surface the face lies on and the plane
/// touching it there —
///
/// ```text
/// v1 := plane(u: hint(dir: …), v: hint(dir: …)) hint(origin: …)
/// v1.origin coincident b0.near.surface
/// v1 tangent(at: v1.origin) b0.near.surface
/// ```
///
/// — so it follows the face wherever the solid's extents and profile take it, and keeps the
/// freedom to slide and turn on it (constrained further like anything else).  Seeded on the
/// face's carrier (`solid::carrier`) where the eye's ray met the face (`at`), else at its middle.
/// One splice, held to the elaborator.
pub fn sketch_on_face(
    e: &Elaborated,
    sk: &Sketch,
    solid: usize,
    face: &str,
    at: Option<[f64; 3]>,
    name: Option<&str>,
) -> Edit {
    Edit::refused_or(&e.program, on_face_edit(e, sk, solid, face, at, name))
}

fn on_face_edit(
    e: &Elaborated,
    sk: &Sketch,
    solid: usize,
    face: &str,
    at: Option<[f64; 3]>,
    name: Option<&str>,
) -> Result<Edit, String> {
    let c = crate::solid::carrier::carrier(sk, solid, face)?;
    let origin = at.map_or(c.middle, |p| {
        let (a, b) = c.basis.view_coords(p);
        c.basis.lift(a, b)
    });
    let name = chosen_name(&mut taken_names(&e.program), name, EntKind::Plane)?;
    let seed = |d: [f64; 3]| vec![syntax::Kid::Hint(syntax::KidSeed { v: seed_round(d), axis: true,
        ..Default::default() })];
    let decl = gesture_decl(EntKind::Plane, &name, vec![seed(c.basis.u), seed(c.basis.v), Vec::new()],
        &seed_round(origin));
    let point = format!("{name}.origin");
    let touch = syntax::OpArg::Named(syntax::Name::new("at"),
                                     syntax::Arg::Ref(syntax::Ref::new(point.clone())));
    // what is drawn on a face is drawn on the surface it lies on (§6.9 [0.68])
    let surface = format!("{face}.surface");
    let stmts = [
        StmtKind::Decl(decl),
        written(said(point, "coincident", &surface)),
        written(Said { args: vec![touch], ..said(name.clone(), "tangent", &surface) }),
    ];
    append_checked(&e.program, &stmts, vec![name])
}

/// The plane already lying on the face `face` of object `solid`, by name — the profile's own for
/// a cap on it, `std.front` included — so a sketch on the face is drawn there rather than on a
/// second plane lying on the first (W113).  `None` where none does, or the face has no carrier.
pub fn face_plane(e: &Elaborated, sk: &Sketch, solid: usize, face: &str) -> Option<String> {
    let c = crate::solid::carrier::carrier(sk, solid, face).ok()?;
    let tol = 1e-9 * sk.extent().max(1.0);
    (0..sk.planes.len()).filter(|&i| sk.basis(i).coplanar(&c.basis, tol))
        .find_map(|i| e.map.name_of(EntRef::new(EntKind::Plane, i)).cloned())
}

/// `x` to `digits` significant digits.
pub(super) fn round_sig(x: f64, digits: i32) -> f64 {
    if x == 0.0 || !x.is_finite() {
        return x;
    }
    let k = 10f64.powi(digits - 1 - x.abs().log10().floor() as i32);
    (x * k).round() / k
}

/// **A place the pointer gave, to the digits the zoom justifies.**  `px` is the page length one
/// screen pixel stands for; `x` goes to the decade grid no coarser than it — at 6 pixels a
/// millimetre, tenths — so it moves less than half a pixel, and the seed a gesture writes has no
/// digit the eye could not have meant.  Shortest round-trip (`syntax::num`) then prints it as
/// that decimal: below a unit the grid is divided out rather than multiplied in, since a tenth is
/// no exact double and `123 × 0.1` is not the double nearest 12.3.  A pixel of no size, or a
/// place that is no number, is left as it is.
pub fn placed(x: f64, px: f64) -> f64 {
    if !x.is_finite() || !px.is_finite() || px <= 0.0 {
        return x;
    }
    let k = px.log10().floor() as i32;
    // `+ 0.0`: a place just left of zero rounds to a negative zero, which nobody wrote
    if k >= 0 {
        let q = 10f64.powi(k);
        (x / q).round() * q + 0.0
    } else {
        let s = 10f64.powi(-k);
        (x * s).round() / s + 0.0
    }
}

/// A seed as the source should read it: a part a millionth of the whole is zero, and the rest
/// keeps nine significant digits — a direction off the drawing is no more exact than that.
fn seed_round(v: [f64; 3]) -> [f64; 3] {
    let big = v.iter().fold(0f64, |m, x| m.max(x.abs()));
    v.map(|x| if x.abs() <= 1e-6 * big { 0.0 } else { round_sig(x, 9) })
}
