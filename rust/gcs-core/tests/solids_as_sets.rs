//! **Solids as sets** (#737): `solid(S)` is the one constructor, the points inside a set, and a
//! prism, a drafted prism, a revolution, a prism through everything and a Boolean of solids are
//! sets — words of `std.solids` over the language's own words, or the same set written by hand.
//! The evaluator recognises a set by what its terms say (`solid::defined`), never by a word's
//! name, so each word and its set written by hand build the same solid: the same definition, so
//! the same exactness, face names, volume and export bytes.  A set no entry recognises is still
//! read point by point.  The retired spellings are refused naming the set they say.
use gcs_core::brep::{export, props, sweep::Say};
use gcs_core::model::{RegionShape, Sense, SolidDef};
use gcs_core::program::Elaborated;
use gcs_core::solid::faces::{face_paths, FacePath};
use gcs_core::solid::MaterialField;
use std::f64::consts::{PI, TAU};

use crate::common::{ent, text::solved};

/// A 60 × 40 rectangle `sec` in the front plane, its corner `a` at the origin.
const RECT: &str = "unit mm\nuse std\nuse std.sets\nuse std.solids\nin std.front {\n\
    a := point\n  b := point hint((60, 0))\n  c := point hint((60, 40))\n  d := point hint((0, 40))\n\
    horizontal (ab := line(a, b)) ->\n  vertical (bc := line(b, c)) ->\n\
    horizontal (cd := line(c, d)) ->\n  vertical (da := line(d, a)) -> close\n\
    a distance(60) b\n  b distance(40) c\n  fix((0, 0)) a\n  sec := face(ab, bc, cd, da)\n\
    o := point hint((30, 20))\n  a distance(30, along: u) o\n  a distance(20, along: v) o\n\
    hole := circle(center: o) hint(r: 5)\n  radius(5) hole\n  hole_f := face(hole)\n}\n";

/// A 4 × 6 rectangle `sec` in the front plane, 10 from the vertical line `ax` it turns about.
const RING: &str = "unit mm\nuse std\nuse std.place (horizontal)\nuse std.motion (turns)\n\
    use std.solids\nin std.front {\n  p0 := point hint((10, 0))\n  p1 := point hint((14, 0))\n\
    p2 := point hint((14, 6))\n  p3 := point hint((10, 6))\n  horizontal (e0 := line(p0, p1)) ->\n\
    vertical (e1 := line(p1, p2)) ->\n  horizontal (e2 := line(p2, p3)) ->\n\
    vertical (e3 := line(p3, p0)) -> close\n  q0 := point\n  q1 := point hint((0, 10))\n\
    ax := vertical line(q0, q1)\n  fix((0, 0)) q0\n  q0 distance(10) q1\n\
    q0 distance(10, along: u) p0\n  q0 horizontal p0\n  p0 distance(4) p1\n  p1 distance(6) p2\n\
    sec := face(e0, e1, e2, e3)\n}\nturn := frame\nturn turns(1) ax\n";

/// What the solid `name` is defined as.
fn def<'a>(e: &'a Elaborated, name: &str) -> &'a SolidDef {
    &e.sketch.solids[ent(e, name).i()].def
}

/// Its faces' names, sorted.
fn faces(e: &Elaborated, name: &str) -> Vec<String> {
    let mut v: Vec<String> = face_paths(&e.sketch, ent(e, name).i()).iter().map(FacePath::text)
        .collect();
    v.sort();
    v
}

/// Its exact volume.
fn volume(e: &Elaborated, name: &str) -> f64 {
    let x = e.sketch.exact_solid(ent(e, name).i()).unwrap_or_else(|m| panic!("`{name}`: {m}"));
    props::volume(&x.brep)
}

/// Its exact export as this kernel writes it, STEP and STL, written under one name.
fn exported(e: &Elaborated, name: &str) -> (String, Vec<u8>) {
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let i = ent(e, name).i();
    let x = export::exact(&e.sketch, i, None, None, &say).unwrap_or_else(|m| panic!("{m:?}"));
    let step = export::step(&x, "part", None, &say).unwrap();
    let stl = export::stl(&e.sketch, i, &x, None, &say).unwrap();
    (step, stl)
}

/// `a` and `b` built alike: one definition's kind and numbers, one set of face names, one
/// volume, one STEP and one STL to the byte.
fn alike(e: &Elaborated, a: &str, b: &str) {
    // the numbers, not the text they were written as
    let numbers = |d: &SolidDef| {
        let s = format!("{d:?}");
        let mut out = String::new();
        for (k, part) in s.split("text: \"").enumerate() {
            out.push_str(if k == 0 { part } else { part.split_once('"').map_or(part, |(_, r)| r) });
        }
        out
    };
    assert_eq!(numbers(def(e, a)), numbers(def(e, b)), "{a} and {b}");
    assert_eq!(faces(e, a), faces(e, b), "{a} and {b}'s faces");
    assert_eq!(volume(e, a).to_bits(), volume(e, b).to_bits(), "{a} and {b}'s volumes");
    // a binary STL's 80-byte header is free text; what follows it is the solid
    let ((sa, la), (sb, lb)) = (exported(e, a), exported(e, b));
    assert!(sa == sb, "{a} and {b}'s STEP alike");
    assert!(la.get(80..) == lb.get(80..), "{a} and {b}'s STL alike");
}

/// **A prism** is the points whose foot on its section's plane is within it, between two
/// ordinates along its normal: `std.solids.Prism` and the same set by hand are one prism, its
/// caps `near` (the greater ordinate) and `far`, each side named by its edge.
#[test]
fn a_prism_by_hand_is_the_words() {
    let e = solved(&format!("{RECT}word := solid(std.solids.Prism(sec, from: -30mm, to: 0))\n\
        hand := solid({{ p | p distance(<= 0mm, along: n) sec; p inside sec\n\
                          p distance(>= -30mm, along: n) sec }})\n"));
    let SolidDef::Prism { from, to, draft: None, .. } = def(&e, "word") else {
        panic!("a prism: {:?}", def(&e, "word"))
    };
    assert_eq!((from.value, to.value), (-30.0, 0.0));
    alike(&e, "word", "hand");
    assert_eq!(faces(&e, "hand"), ["ab", "bc", "cd", "da", "far", "near"]);
    assert!((volume(&e, "hand") - 72_000.0).abs() < 1e-9 * 72_000.0);
}

/// **A drafted prism**: its section at each ordinate its own moved into the material by the
/// ordinate's size times the draft's tangent — the word and the set by hand one drafted prism,
/// its angle read as the draughtsman states it.
#[test]
fn a_drafted_prism_by_hand_is_the_words() {
    let e = solved(&format!("{RECT}word := solid(std.solids.Drafted(sec, from: -5mm, to: 5mm, \
        draft: 3deg))\n\
        hand := solid({{ p | p distance(>= -5mm, along: n) sec; p distance(<= 5mm, along: n) sec\n\
            out := {{ q | q outside sec }}\n  into := {{ q | q inside sec }}\n\
            distance(p, out) - distance(p, into) >= tan(3deg) * distance(p, sec) }})\n"));
    let SolidDef::Prism { draft: Some(d), .. } = def(&e, "word") else {
        panic!("a drafted prism: {:?}", def(&e, "word"))
    };
    assert_eq!(d.value.to_bits(), 3f64.to_radians().to_bits());
    alike(&e, "word", "hand");
}

/// **A revolution** is a face's patch swept by a turn about a line in its plane: a whole turn,
/// a quarter, and a quarter the other way (an interval below nothing, `Sense::Cw`), each the
/// word's and the hand's alike, Pappus's volume, and a turn short of a whole one capped `start`
/// and `end`.
#[test]
fn a_revolution_by_hand_is_the_words() {
    let e = solved(&format!("{RING}whole := solid(std.solids.Revolution(sec, about: ax))\n\
        quarter := solid(std.solids.Revolution(sec, about: ax, to: 90deg))\n\
        back := solid(std.solids.Revolution(sec, about: ax, from: -90deg, to: 0deg))\n\
        whole_hand := solid({{ p | p coincident(under: turn, over t in (0deg, 360deg)) sec }})\n\
        quarter_hand := solid({{ p | p coincident(under: turn, over t in (0deg, 90deg)) sec }})\n\
        back_hand := solid({{ p | p coincident(under: turn, over t in (-90deg, 0deg)) sec }})\n"));
    for (name, sweep, sense) in [("whole", TAU, Sense::Ccw), ("quarter", TAU / 4.0, Sense::Ccw),
                                 ("back", TAU / 4.0, Sense::Cw)] {
        let SolidDef::Revolve { sweep: s, sense: k, .. } = def(&e, name) else {
            panic!("{name}: a revolution: {:?}", def(&e, name))
        };
        assert!((s.value - sweep).abs() < 1e-15 && *k == sense, "{name}: {s:?} {k:?}");
        alike(&e, name, &format!("{name}_hand"));
    }
    let pappus = 2.0 * PI * 12.0 * 24.0;
    assert!((volume(&e, "whole") - pappus).abs() < 1e-9 * pappus);
    assert!(faces(&e, "quarter").iter().any(|f| f == "start"));
    assert!(faces(&e, "quarter").iter().any(|f| f == "end"));
}

/// **A prism through everything**: a section unbounded along its normal, cut from a body, is the
/// prism through that body's material — `std.solids.Through` and the set by hand alike, each
/// naming no body (#746) — and the plate less the bore its closed form.
#[test]
fn a_prism_through_everything_is_bounded_by_what_it_cuts() {
    let e = solved(&format!("{RECT}plate := solid(std.solids.Prism(sec, from: -30mm, to: 0))\n\
        drill := solid(std.solids.Through(face(hole)))\n\
        bore := solid({{ p | p inside hole_f }})\n\
        body := solid(plate)\ndrill cut body\nother := solid(plate)\nbore cut other\n"));
    for (tool, body) in [("drill", "body"), ("bore", "other")] {
        assert!(matches!(def(&e, tool), SolidDef::Through { .. }), "{tool}: {:?}", def(&e, tool));
        assert_eq!(gcs_core::solid::applied_to(&e.sketch, ent(&e, tool).i()),
                   [ent(&e, body).i() as u32], "{tool}");
    }
    let want = 72_000.0 - PI * 25.0 * 30.0;
    for b in ["body", "other"] {
        assert!((volume(&e, b) - want).abs() < 1e-9 * want, "{b}: {}", volume(&e, b));
    }
}

/// Two plates of radius 20 on the top plane's axis, `low` from -5 to 0 and `high` from 40 to 50,
/// a third `far` from 100 to 110, a pin of radius 2 through everything on the same axis, and
/// `feed` sliding 100 a turn up it.
const STACK: &str = "unit mm\nuse std\nuse std.solids\nuse std.motion (slides)\n\
    in std.front {\n  construction centerline shaft := line(std.origin, hint((0, 1)))\n\
    fix((0, 1)) shaft.p2\n}\nfeed := frame\nfeed slides(100mm) shaft\nin std.top {\n\
    rim := radius(20mm) circle(center: std.top.origin)\n\
    hole := radius(2mm) circle(center: std.top.origin)\n\
    low := solid(std.solids.Prism(face(rim), from: -5mm, to: 0))\n\
    high := solid(std.solids.Prism(face(rim), from: 40mm, to: 50mm))\n\
    far := solid(std.solids.Prism(face(rim), from: 100mm, to: 110mm))\n\
    construction pin := solid(std.solids.Through(face(hole)))\n}\n";

/// A plate of radius 20 and `depth` less the pin's bore of radius 2.
fn bored(depth: f64) -> f64 { PI * (400.0 - 4.0) * depth }

/// Whether the material field of body `name` holds no material at `p`.
fn hollow_at(e: &Elaborated, name: &str, p: [f64; 3]) -> bool {
    use gcs_core::interval::{minimum::Options, Interval as I};
    let field = MaterialField::read(&e.sketch, ent(e, name).i(), 1e-10).unwrap();
    let answer = field.evaluator(256).bounds(p.map(|x| I::point(x).unwrap()),
        Options { value_tolerance: 1e-6, max_evaluations: 50000 }).unwrap();
    answer.value.bounds()[0] > 0.0
}

/// **One tool through any number of bodies** (#746): a prism through everything names no body,
/// so cut from two plates at different heights it reaches through each — exactly, faceted and as
/// a field — where once the first body it was cut from bounded it alone and the second kept its
/// material above the first.
#[test]
fn a_prism_through_everything_cut_from_two_bodies_goes_through_each() {
    let e = solved(&format!("{STACK}a := solid(low)\nb := solid(high)\npin cut a\npin cut b\n"));
    let pin = ent(&e, "pin").i();
    assert_eq!(gcs_core::solid::applied_to(&e.sketch, pin),
               [ent(&e, "a").i() as u32, ent(&e, "b").i() as u32]);
    for (body, depth, z) in [("a", 5.0, -2.5), ("b", 10.0, 45.0)] {
        let exact = volume(&e, body);
        assert!((exact - bored(depth)).abs() < 1e-9 * bored(depth), "{body}: {exact}");
        // the faceted body is the plate's facets less the bore's: the plate cut by a prism
        // bounded by hand past both is the same solid
        let faceted = |e: &Elaborated, name: &str| e.sketch.evaluated_solid(ent(e, name).i(),
            gcs_core::solid::ApproximationPolicy::Report).unwrap().volume();
        let unpinned = STACK.replace("construction pin := solid(std.solids.Through(face(hole)))\n", "");
        let by_hand = solved(&format!("{unpinned}x := solid(std.solids.Prism(face(hole), from: -10mm, \
            to: 60mm))\n{body} := solid({})\nx cut {body}\n", if body == "a" { "low" } else { "high" }));
        let (ours, theirs) = (faceted(&e, body), faceted(&by_hand, body));
        assert!((ours - theirs).abs() < 1e-9 * theirs, "{body}: {ours} against {theirs}");
        assert!(hollow_at(&e, body, [0.0, 0.0, z]), "{body}: the bore at {z}");
        assert!(!hollow_at(&e, body, [10.0, 0.0, z]), "{body}: the plate at {z}");
    }
}

/// **A copy is bounded by its own body** (#746): a pin cut from one plate, and a copy of it slid a
/// hundred up the axis cut from another there, each go through their own — the pin's span is read
/// from where each copy puts its section, so it reaches no further than the two need.
#[test]
fn a_placed_copy_of_a_prism_through_everything_goes_through_its_own_body() {
    let e = solved(&format!("{STACK}a := solid(low)\nc := solid(far)\n\
        lifted := pin at(under: feed) 360deg\npin cut a\nlifted cut c\n"));
    for (body, depth, z) in [("a", 5.0, -2.5), ("c", 10.0, 105.0)] {
        let exact = volume(&e, body);
        assert!((exact - bored(depth)).abs() < 1e-9 * bored(depth), "{body}: {exact}");
        assert!(hollow_at(&e, body, [0.0, 0.0, z]), "{body}: the bore at {z}");
    }
    // the pin spans the low plate and, read from the copy's section, the far one: -5 to 10
    let csg = gcs_core::solid::resolve(&e.sketch, ent(&e, "pin").i(), gcs_core::solid::REPORT_UNIT);
    let b = csg.bbox();
    assert!((b.lo[2] + 5.0).abs() < 1e-2 && (b.hi[2] - 10.0).abs() < 1e-2, "{b:?}");
}

/// **Booleans as sets**: the points inside a solid and outside another are the body term over
/// them — the same solid `bore cut body` makes — and the outside of what is outside two solids is
/// their union, exactly.
#[test]
fn a_boolean_of_solids_is_the_body_term() {
    let e = solved(&format!("{RECT}stock := solid(std.solids.Prism(sec, from: -30mm, to: 0))\n\
        bore := solid(std.solids.Prism(face(hole), from: -40mm, to: 10mm))\n\
        cut_by_word := solid(stock)\nbore cut cut_by_word\n\
        cut_as_set := solid({{ p | p inside stock; p outside bore }})\n\
        joined := solid({{ p | p outside {{ q | q outside stock; q outside bore }} }})\n"));
    let SolidDef::Body { stock, through, .. } = def(&e, "cut_as_set") else {
        panic!("a body: {:?}", def(&e, "cut_as_set"))
    };
    assert_eq!((*stock as usize, through.len()), (ent(&e, "stock").i(), 1));
    assert_eq!(volume(&e, "cut_by_word").to_bits(), volume(&e, "cut_as_set").to_bits());
    let SolidDef::Body { on, .. } = def(&e, "joined") else { panic!("a union") };
    assert_eq!(on.len(), 1);
    let want = 72_000.0 + PI * 25.0 * 20.0;
    assert!((volume(&e, "joined") - want).abs() < 1e-9 * want, "{}", volume(&e, "joined"));
}

/// **The outside of a set of several statements** is where any of them fails: the union of two
/// balls as the outside of what is outside both, read point by point — in either ball and in
/// neither.
#[test]
fn outside_a_conjunction_is_a_union() {
    let e = solved("unit mm\nuse std\nuse std.sets\nc2 := point\nfix((4, 0, 0)) c2\n\
        one := std.sets.Sphere(std.origin, r: 5)\nother := std.sets.Sphere(c2, r: 5)\n\
        both := solid({ p | p outside { q | q outside one; q outside other } })\n");
    let s = ent(&e, "both").i();
    let SolidDef::Region { terms, .. } = &e.sketch.solids[s].def else { panic!("a region") };
    assert!(matches!(terms[0].shape, RegionShape::Outside { .. }), "{terms:?}");
    let field = MaterialField::read(&e.sketch, s, 1e-10).unwrap();
    for (p, inside) in [([-4.0, 0.0, 0.0], true), ([8.0, 0.0, 0.0], true), ([2.0, 4.0, 0.0], true),
                        ([2.0, 0.0, 6.0], false), ([10.0, 0.0, 0.0], false)] {
        assert_eq!(field.side(p) < 0.0, inside, "{p:?}");
    }
}

/// **A drawn face read as a set** where no sweep says it: within the face's loops at the
/// point's foot and within a ball — a quarter ball, read point by point.
#[test]
fn a_face_is_a_set_where_no_sweep_says_it() {
    let e = solved(&format!("{RECT}r := solid({{ p | p inside sec; p inside std.sets.Sphere(a, \
        r: 20mm) }})\n"));
    let s = ent(&e, "r").i();
    assert!(matches!(def(&e, "r"), SolidDef::Region { .. }));
    let field = MaterialField::read(&e.sketch, s, 1e-10).unwrap();
    // the front plane's `u` is the world's x and its `v` the world's z
    for (p, inside) in [([5.0, -5.0, 5.0], true), ([5.0, 5.0, 5.0], true), ([-5.0, 0.0, 5.0], false),
                        ([5.0, 0.0, -5.0], false), ([15.0, 0.0, 15.0], false)] {
        assert_eq!(field.side(p) < 0.0, inside, "{p:?}");
    }
}

/// **The retired spellings** are refused where they are read, naming the set they say from the
/// document's own arguments.
#[test]
fn the_sweep_labels_are_retired_naming_the_set() {
    let says = |src: String, needle: &str| {
        let (_, errs, _) = gcs_core::library::parse_linked(&src);
        assert!(errs.iter().any(|e| e.message.contains(needle)), "`{needle}` among {errs:?}");
    };
    says(format!("{RECT}block := solid(sec, depth: 30mm)\n"),
         "write `solid(std.solids.Prism(sec, from: -30mm, to: 0))`");
    says(format!("{RING}r := solid(sec, about: ax, sweep: 90deg, sense: cw)\n"),
         "`solid(std.solids.Revolution(sec, about: ax, from: -90deg, to: 0deg))`");
    says(format!("{RECT}d := solid(hole_f, through: sec)\n"),
         "`solid(std.solids.Through(hole_f))`, cut from the body it goes through");
    says(format!("{RING}s := sweep(sec over t in (0deg, 90deg), under: turn)\n"),
         "`solid({ p | p inside(under: turn, over t in (0deg, 90deg)) sec })`");
}

/// **A set written by hand is read before it is built as the word is** (#744): the flattener
/// reads its statements, not a word's name, so a prism by hand adds nothing to the solve — no
/// point its set is applied to — and is the word's to every reader of a solid before it is built.
#[test]
fn a_set_by_hand_adds_nothing_to_the_solve() {
    let word = solved(&format!("{RECT}s := solid(std.solids.Prism(sec, from: -30mm, to: 0))\n"));
    let hand = solved(&format!("{RECT}s := solid({{ p | p inside sec\n\
        p distance(>= -30mm, along: n) sec; p distance(<= 0mm, along: n) sec }})\n"));
    assert_eq!(word.sketch.points.len(), hand.sketch.points.len());
    assert_eq!(word.sketch.params.len(), hand.sketch.params.len());
    alike(&hand, "s", "s");
    assert!(matches!(def(&hand, "s"), SolidDef::Prism { draft: None, .. }));
}

/// **Sketching on the faces of a prism by hand**, as on the word's: a point on a cap stands at
/// its ordinate, and a plane touching a straight side at its origin lies on it.
#[test]
fn a_prism_by_hand_is_sketched_on() {
    let e = solved(&format!("{RECT}hand := solid({{ p | p inside sec\n\
        p distance(>= -30mm, along: n) sec; p distance(<= 0mm, along: n) sec }})\n\
        m := point hint((5, 10, 5))\nm coincident hand.far.surface\n\
        v1 := plane(u: hint(dir: (1, 0, 0)), v: hint(dir: (0, 1, 0))) hint(origin: (10, 8, 1))\n\
        v1.origin coincident hand.ab.surface\nv1 tangent(at: v1.origin) hand.ab.surface\n"));
    // the front plane's normal is the world's −y: the far cap 30 behind it
    let m = e.sketch.world_point(ent(&e, "m").i());
    assert!((m[1] - 30.0).abs() < 1e-9, "{m:?}");
    let n = e.sketch.basis(ent(&e, "v1").i()).normal();
    assert!(n[0].abs() < 1e-9 && n[1].abs() < 1e-9, "{n:?}");
}

/// **A revolution by hand has named sides** the solve reads before it is built: `surface(S, e)`
/// over the set turned about a line is the edge turned about it, as over the word.
#[test]
fn a_revolution_by_hand_has_named_sides() {
    use gcs_core::model::Held;
    let held = |solid: &str| {
        let e = solved(&format!("{RING}rev := solid({solid})\nside := surface(rev, e1)\n\
            gear := frame\nf := envelope(side over t in (-60deg, 60deg), under: turn, \
            seen_from: gear)\n"));
        let f = e.sketch.envelopes.iter().find(|x| x.name == "f").expect("an envelope");
        let Held::Revolution { axis, edge } = f.held.clone() else {
            panic!("`{solid}`: {:?}", f.held)
        };
        assert_eq!((axis, edge), (ent(&e, "ax"), ent(&e, "e1")));
    };
    held("std.solids.Revolution(sec, about: ax, to: 90deg)");
    held("{ p | p coincident(under: ax, over t in (0deg, 90deg)) sec }");
}

/// **A draft is read off the bound's tree**, not its printed text: the lean written with its
/// product the other way round and its spacing changed is the word's drafted prism, and the
/// same bound over another face's plane is no draft of this one's.
#[test]
fn a_draft_is_read_off_its_tree() {
    let word = "word := solid(std.solids.Drafted(sec, from: -5mm, to: 5mm, draft: 3deg))\n";
    let hand = "hand := solid({ p | p distance(<= 5mm, along: n) sec\n\
        p distance(>= -5mm, along: n) sec\n  into := { q | q inside sec }\n\
        out := { q | q outside sec }\n\
        distance(p,out)-distance( p, into ) >= distance(p, sec) * tan( 3deg ) })\n";
    // read before it is built, as the word is: nothing added to the solve
    let points = |s: &str| solved(&format!("{RECT}{s}")).sketch.points.len();
    assert_eq!(points(word), points(hand));
    let e = solved(&format!("{RECT}{word}{hand}"));
    let SolidDef::Prism { draft: Some(d), .. } = def(&e, "hand") else {
        panic!("a drafted prism: {:?}", def(&e, "hand"))
    };
    assert_eq!(d.value.to_bits(), 3f64.to_radians().to_bits());
    alike(&e, "word", "hand");
}
