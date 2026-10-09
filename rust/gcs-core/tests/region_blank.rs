//! **The hypoid's blank, as a region** (#145, F2/F3): the gate the region solid is held to
//! before the blank's modules are written in it.  For the bevel pair and the configured hypoid,
//! each member's blank — the heel ball, within the tip cone, less the toe ball and the back cone
//! — is stated as one region, `{ p | p inside tip; p inside heel; p outside toe; p outside back }`, its
//! spheres about the layout's own apex and its cones measured off the layout's own meridians, and
//! `solid(R)` must be the solid today's blank modules make of their carriers: the same volume
//! exactly, the same material at points either side.
use gcs_core::model::{SolidDef, SolidE};
use gcs_core::program::Elaborated;
use gcs_core::solid::MaterialField;
use gcs_core::space::{add, cross, dot, norm, scale, sub};

type V = [f64; 3];

fn point(e: &Elaborated, name: &str) -> V {
    let p = e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`"));
    e.sketch.world_point(p.i())
}

fn line(e: &Elaborated, name: &str) -> (V, V) {
    let l = e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`"));
    let l = &e.sketch.lines[l.i()];
    (e.sketch.world_point(l.p1 as usize), e.sketch.world_point(l.p2 as usize))
}

fn unit(v: V) -> V {
    scale(v, 1.0 / norm(v))
}

/// Where the line through `p` along `u` comes nearest the line through `a` along `d`, on the
/// latter: a cone's apex, where its meridian meets the member's axis.
fn meet(p: V, u: V, a: V, d: V) -> V {
    let w = sub(p, a);
    let (b, c) = (dot(u, d), dot(u, u));
    let t = (dot(w, d) * c - dot(w, u) * b) / (c * dot(d, d) - b * b);
    add(a, scale(d, t))
}

/// Text a number is written as, to the last bit.
fn num(x: f64) -> String {
    format!("{x:.17e}")
}

/// A point fixed where `x` is.
fn fixed(name: &str, x: V) -> String {
    let v = format!("({}, {}, {})", num(x[0]), num(x[1]), num(x[2]));
    format!("{name} := point hint({v})\nfix({v}) {name}\n")
}

/// The statements making member `m`'s blank as a region, and today's beside it, appended to the
/// solved pair `e`'s document.
fn blank_text(e: &Elaborated, m: &str, apex: &str) -> String {
    let at = |n: &str| format!("pair.reference.{m}_blank.{n}");
    let (a0, a1) = line(e, &format!("pair.reference.{m}.ax"));
    let d = unit(sub(a1, a0));
    let o = point(e, &format!("pair.reference.{apex}"));
    let radius = |end: &str| norm(sub(point(e, &at(&format!("span.{end}"))), o));
    let mut text = String::new();
    // each cone: its apex where its meridian meets the axis, its axis the member's direction from
    // there, its half-angle the meridian's from the axis
    for cone in ["tip", "back"] {
        let (p, q) = (point(e, &at(&format!("{cone}.p"))), point(e, &at(&format!("{cone}.q"))));
        let u = unit(sub(q, p));
        let top = meet(p, u, a0, d);
        let half = dot(u, d).clamp(-1.0, 1.0).acos().to_degrees();
        text += &fixed(&format!("{m}_{cone}_apex"), top);
        text += &fixed(&format!("{m}_{cone}_far"), add(top, d));
        text += &format!("{m}_{cone}_about := line({m}_{cone}_apex, {m}_{cone}_far)\n\
                          {m}_{cone} := std.Cone({m}_{cone}_about, half: {}deg)\n", num(half));
    }
    // the spheres about the apex, where the layout put it
    text += &fixed(&format!("{m}_centre"), o);
    for ball in ["heel", "toe"] {
        text += &format!("{m}_{ball} := std.Sphere({m}_centre, r: {})\n", num(radius(ball)));
    }
    text += &format!("{m}_r := {{ p | p inside {m}_tip; p inside {m}_heel; p outside {m}_toe; \
                      p outside {m}_back }}\n{m}_region := solid({m}_r)\n");
    text
}

/// Today's blank of member `m`, as its modules make it (`blank.member.MemberBlank`): the heel's
/// carrier within the tip's, less the toe's and the back's — added to the drawing as a body over
/// them, the layout being private to the pair.
fn today(e: &mut Elaborated, m: &str) -> usize {
    let carrier = |n: &str| {
        let name = format!("pair.reference.{m}_blank.{n}.carrier");
        e.map.ent_named(&name).unwrap_or_else(|| panic!("no `{name}`")).i() as u32
    };
    let def = SolidDef::Body {
        stock: carrier("heel"),
        on: Vec::new(),
        through: vec![carrier("toe"), carrier("back")],
        bound: vec![carrier("tip")],
    };
    e.sketch.solids.push(SolidE { def, name: format!("{m}_today"), class: Default::default() });
    e.sketch.solids.len() - 1
}

#[test]
fn the_blank_is_its_region() {
    for (label, configuration) in fixtures::gear::designs().into_iter().take(2) {
        let read = |extra: &str| fixtures::read_beside(&(fixtures::gear::source() + extra),
            &fixtures::gear::project(),
            &mut |name, text| if name == "configuration" { configuration.clone() } else { text });
        let e = read("");
        let extra = blank_text(&e, "gear", "gear.O") + &blank_text(&e, "pinion", "pinion.A");
        let mut e = read(&extra);
        for (m, apex) in [("gear", "gear.O"), ("pinion", "pinion.A")] {
            let made = today(&mut e, m);
            let region = e.map.ent_named(&format!("{m}_region")).unwrap().i();
            let volume = |s: usize| {
                let x = e.sketch.exact_solid(s).unwrap_or_else(|why| panic!("{label} {m}: {why}"));
                gcs_core::brep::props::volume(&x.brep)
            };
            let (v, today) = (volume(region), volume(made));
            assert!((v - today).abs() <= 1e-9 * today, "{label} {m}: {v} against {today}");
            // the same material at points through the blank and round it
            let field = |s| MaterialField::read(&e.sketch, s, 1e-10).unwrap();
            let (r, t) = (field(region), field(made));
            let o = point(&e, &format!("pair.reference.{apex}"));
            let (a0, a1) = line(&e, &format!("pair.reference.{m}.ax"));
            let d = unit(sub(a1, a0));
            let up = if d[2].abs() < 0.9 { [0.0, 0.0, 1.0] } else { [1.0, 0.0, 0.0] };
            let across = unit(cross(d, up));
            let heel = point(&e, &format!("pair.reference.{m}_blank.span.heel"));
            let reach = norm(sub(heel, o)) * 1.2;
            let (mut decided, mut agreed) = (0, 0);
            for i in 0..24 {
                for j in 0..24 {
                    let x = add(o, add(scale(d, reach * (i as f64 / 23.0 * 2.0 - 1.0)),
                        scale(across, reach * j as f64 / 23.0)));
                    let (a, b) = (r.side(x), t.side(x));
                    if a.abs() < 1e-6 || b.abs() < 1e-6 {
                        continue;
                    }
                    decided += 1;
                    agreed += usize::from((a < 0.0) == (b < 0.0));
                }
            }
            assert!(decided > 400 && agreed == decided, "{label} {m}: {agreed} of {decided}");
        }
    }
}

