//! **A sweep rolls through what it cuts** (#271, §6.9, `solid::roll`): `solid(tool, under: m,
//! through: body)` rolls over the roll about 0 during which the tool meets the body's material,
//! found after the solve. Held to a closed form (a ball turned past a ball: they part at
//! `2 asin(r / R)`), never short of it and a small margin past; read alike by every reader; and the
//! spiral bevel, which states no roll, admitted where its stated roll was refused (10:48).
use gcs_core::model::{Roll, SolidDef};
use gcs_core::{program, solid, solve, syntax};

/// A ball of radius 1 three from the z axis, turned about it, and a ball of radius 1 where the
/// tool starts (`at` its centre): the tool's roll through `part`, the blank's body.
fn balls(at: &str, sweep: &str) -> String {
    format!("\
unit mm
use std
tool_c := point
fix((3, 0, 0)) tool_c
blank_c := point
fix({at}) blank_c
lump := std.Sphere(tool_c, r: 1mm)
stock := std.Sphere(blank_c, r: 1mm)
construction tool := solid(lump)
blank := solid(stock)
o := point
top := point
fix((0, 0, 0)) o
fix((0, 0, 1)) top
spindle := line(o, top)
turn := motion(about: spindle)
construction removal := solid(tool, {sweep})
part := solid(blank)
removal cut part
")
}

fn read(src: &str) -> program::Elaborated {
    let mut e = crate::common::read(src);
    assert!(solve::solve(&mut e.sketch, Default::default()).success);
    e
}

/// Where the balls part, radians: their centres three from the axis, two apart.
fn parting() -> f64 { 2.0 * (1.0f64 / 3.0).asin() }

#[test]
fn a_roll_through_a_body_ends_just_past_where_the_tool_clears_it() {
    let e = read(&balls("(3, 0, 0)", "under: turn, through: part"));
    let removal = crate::common::ent(&e, "removal").i();
    let SolidDef::Swept { roll: Roll::Through { body }, .. } = &e.sketch.solids[removal].def else {
        panic!("a sweep through a body")
    };
    assert_eq!(e.sketch.solids[*body as usize].name, "part");
    let [from, to] = solid::roll::interval(&e.sketch, removal).unwrap();
    // never short of where the balls part, at most a few steps past it, each way
    for end in [-from, to] {
        assert!(end >= parting(), "{} short of {}", end.to_degrees(), parting().to_degrees());
        assert!(end <= parting() + 4.0 * solid::roll::STEP,
            "{} more than four steps past {}", end.to_degrees(), parting().to_degrees());
    }
    // the tool at each end clear of the blank: its centre more than two from the blank's
    for end in [from, to] {
        let apart = (3.0 * end.cos() - 3.0).hypot(3.0 * end.sin());
        assert!(apart > 2.0, "{}: {apart}", end.to_degrees());
    }
    // the march's own proof over the step past each end
    let tool = solid::SpatialField::read(&e.sketch, crate::common::ent(&e, "tool").i(), 1e-10).unwrap();
    let blank = solid::SpatialField::read(&e.sketch, crate::common::ent(&e, "blank").i(), 1e-10).unwrap();
    let family = gcs_core::motion::Family::read(&e.sketch, crate::common::ent(&e, "turn").i()).unwrap();
    let support = tool.support_bounds().unwrap().unwrap();
    assert!(solid::roll::clear(&tool, support, &blank, &family, [to, to + solid::roll::STEP]).unwrap());
    assert!(solid::roll::clear(&tool, support, &blank, &family, [from - solid::roll::STEP, from]).unwrap());
    // and none where the tool sits in the blank
    assert!(!solid::roll::clear(&tool, support, &blank, &family, [0.0, solid::roll::STEP]).unwrap());
}

/// The interval is one for every reader: the contacts' domain is it, and the document's swept
/// field holds the tool's centre turned to either end and nothing a ball's width past it.
#[test]
fn every_reader_reads_one_roll() {
    use gcs_core::interval::{minimum::Options, Interval as I};
    let e = read(&balls("(3, 0, 0)", "under: turn, through: part"));
    let removal = crate::common::ent(&e, "removal").i();
    let roll = solid::roll::interval(&e.sketch, removal).unwrap();
    let contacts = solid::SweepContacts::read(&e.sketch, removal, 1e-10).unwrap();
    assert_eq!(contacts.domain(), roll);
    let field = solid::MaterialField::read(&e.sketch, removal, 1e-10).unwrap();
    let material = |t: f64| {
        let p = [3.0 * t.cos(), 3.0 * t.sin(), 0.0];
        let answer = field.evaluator(256).bounds(p.map(|x| I::point(x).unwrap()),
            Options { value_tolerance: 1e-6, max_evaluations: 50000 }).unwrap();
        answer.value.bounds()
    };
    for end in roll {
        // the tool's centre at the end, swept; half a radian on along the turn, 1.48 from the
        // last ball's centre, not
        assert!(material(end)[1] < 0.0, "{}: {:?}", end.to_degrees(), material(end));
        let past = end + end.signum() * 0.5;
        assert!(material(past)[0] > 0.0, "{}: {:?}", past.to_degrees(), material(past));
    }
}

/// A tool that does not meet the body at 0, or one never clear of it, is refused naming why.
#[test]
fn a_roll_with_no_engagement_about_0_is_refused() {
    let e = read(&balls("(-3, 0, 0)", "under: turn, through: part"));
    let removal = crate::common::ent(&e, "removal").i();
    let why = solid::roll::interval(&e.sketch, removal).unwrap_err();
    assert!(why.contains("`removal` rolls through `part`") && why.contains("does not meet it at 0"), "{why}");
    // a ball on the axis is never clear of a body it starts in
    let e = read(&balls("(3, 0, 0)", "under: turn, through: part").replace("fix((3, 0, 0)) tool_c",
        "fix((0, 0, 0)) tool_c").replace("stock := std.Sphere(blank_c, r: 1mm)", "stock := std.Sphere(blank_c, r: 4mm)"));
    let why = solid::roll::interval(&e.sketch, crate::common::ent(&e, "removal").i()).unwrap_err();
    assert!(why.contains("not clear of it within two turns"), "{why}");
}

/// `through:` beside a stated roll is no sweep, it names a solid, and a body made of the sweep
/// that rolls through it is E041.
#[test]
fn through_is_a_roll_of_its_own() {
    let mixed = balls("(3, 0, 0)", "under: turn, through: part, from: 0deg, to: 10deg");
    let (_, errors) = syntax::parse(&mixed);
    assert!(errors.iter().any(|d| d.message.contains("`through:` a body, without other sweep labels")),
        "{errors:?}");
    // the roll names a solid
    crate::common::refused(&balls("(3, 0, 0)", "under: turn, through: spindle"), "E080",
        "`through:` requires a solid", "spindle");
    // a body made of the sweep that rolls through it
    let cycle = balls("(3, 0, 0)", "under: turn, through: part").replace("part := solid(blank)\nremoval cut part\n",
        "part := solid(removal)\n");
    let (prog, errs, _) = gcs_core::library::parse_linked(&cycle);
    assert!(errs.is_empty(), "{errs:?}");
    let e = program::elaborate(&prog);
    assert!(e.diags.iter().any(|d| d.code.as_str() == "E041"),
        "{:?}", e.diags.iter().map(|d| &d.message).collect::<Vec<_>>());
}

/// Printed as written, and read back to the same roll.
#[test]
fn a_roll_through_prints_as_written() {
    let src = balls("(3, 0, 0)", "under: turn, through: part");
    let (p, errors) = syntax::parse(&src);
    assert!(errors.is_empty(), "{errors:?}");
    let stmt = p.root().body.iter()
        .find(|st| src[st.span.lo as usize..st.span.hi as usize].starts_with("construction removal")).unwrap();
    let mut printed = String::new();
    syntax::write_stmt_to(&mut printed, &stmt.kind).unwrap();
    assert!(printed.contains("solid(tool, under: turn, through: part)"), "{printed}");
    let e = read(&src.replace(&src[stmt.span.lo as usize..stmt.span.hi as usize], &printed));
    let removal = crate::common::ent(&e, "removal").i();
    assert!(matches!(e.sketch.solids[removal].def, SolidDef::Swept { roll: Roll::Through { .. }, .. }));
}

/// **The issue's case** (#271): the spiral bevel at 10:48, the gear close to a crown gear, whose
/// stated 45-degree roll left the space cutter in the blank (E1). Rolled through its blank, each
/// member's roll ends where the tool's surface, sampled as E1 samples it, is clear of the blank, a
/// small margin past where it first leaves it, and the gear is admitted.  Cut by the mill the
/// issue's pair had (`fixtures::gear::recorded_machine`): at a ten-tooth pinion the balance that
/// finds the mill (#283) does not solve from its seeds.
#[test]
fn the_pair_at_ten_teeth_is_admitted_with_no_roll_stated() {
    let text = fixtures::gear::configure(&fixtures::gear::source(),
        &[("pinion_teeth", 10.), ("pressure_shift", 12.5)]);
    assert!(!text.contains("roll_limit"), "the pair states no roll");
    let e = fixtures::gear::read(&text);
    let sk = &e.sketch;
    for member in ["pinion", "gear"] {
        let removal = fixtures::solid(&e, &format!("pair.{member}.cut.removal"));
        let SolidDef::Swept { motion, roll: Roll::Through { body }, .. } = &sk.solids[removal].def else {
            panic!("{member}: the cut rolls through its body")
        };
        let roll = solid::roll::interval(sk, removal).unwrap();
        // E1's reading: the tool's surface sampled, each point at a roll read against the blank
        let blank = match &sk.solids[*body as usize].def {
            SolidDef::Body { stock, .. } => *stock as usize,
            _ => *body as usize,
        };
        let blank = solid::SpatialField::read(sk, blank, 1e-10).unwrap();
        let family = gcs_core::motion::Family::read(sk, *motion as usize).unwrap();
        let contacts = solid::SweepContacts::read(sk, removal, 1e-10).unwrap();
        let tool = contacts.source_material();
        let mut surface = Vec::new();
        for s in contacts.patches() {
            for i in 0..=24 {
                for j in 0..240 {
                    let Ok(p) = s.at(i as f64 / 24.0, j as f64 / 240.0) else { continue };
                    if tool.value(p.position).abs() < 1e-6 { surface.push(p.position); }
                }
            }
        }
        let meets = |t: f64| {
            let pose = family.at(t).unwrap();
            surface.iter().any(|&p| blank.value(pose.point(p)) < 0.0)
        };
        let step = 0.25f64.to_radians();
        for (way, end) in [(-1.0, roll[0]), (1.0, roll[1])] {
            let mut first = 0.0;
            while meets(way * first) { first += step; }
            assert!(!meets(end), "{member}: the tool at {} meets the blank", end.to_degrees());
            assert!(end.abs() >= first - step && end.abs() <= first + 8.0f64.to_radians(),
                "{member}: rolls to {} against first clear at {}", end.to_degrees(), first.to_degrees());
        }
        eprintln!("{member}: rolls {:?} degrees", roll.map(f64::to_degrees));
    }
    let gear = fixtures::solid(&e, "pair.gear.body");
    solid::admission::admit_body(sk, gear, &solid::admission::Options::default())
        .unwrap_or_else(|r| panic!("the gear refused: {r}"));
}

/// **The gate** (#271): the pair stating no roll builds each member exactly at 24:48 as its stated
/// rolls built it (±35 degrees for the pinion, ±45 for the gear: `fixtures::gear::roll`), the same
/// volume and faces, and at 10:48, where the stated gear's was refused, builds both.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about a minute: six members built exactly")]
fn members_rolled_through_their_blanks_build_as_their_stated_rolls_did() {
    use gcs_core::brep::{export, props, sweep::Say};
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let built = |text: &str, member: &str| {
        let e = fixtures::gear::read(text);
        let body = fixtures::solid(&e, &format!("pair.{member}.body"));
        let exact = export::exact(&e.sketch, body, None, None, &say)
            .unwrap_or_else(|r| panic!("{member} refused at {:?}: {}", r.stage, r.message));
        (props::volume(&exact.solid), exact.solid.faces.len())
    };
    let text = fixtures::gear::source();
    for (member, degrees) in [("pinion", 35.0), ("gear", 45.0)] {
        let (through, faces) = built(&text, member);
        let (stated, stated_faces) = built(&fixtures::gear::roll(&text, member, degrees), member);
        assert!((through - stated).abs() <= 1e-9 * stated, "{member}: {through} against {stated}");
        assert_eq!(faces, stated_faces, "{member}");
    }
    let ten = fixtures::gear::configure(&text, &[("pinion_teeth", 10.)]);
    for member in ["pinion", "gear"] {
        let (volume, faces) = built(&ten, member);
        assert!(volume > 0.0 && faces > 0, "{member}: {volume} mm³, {faces} faces");
    }
}
