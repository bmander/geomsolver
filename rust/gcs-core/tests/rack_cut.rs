//! **A rack cuts a spur gear, natively** (issue #61): the generating-sweep class widened to a
//! prism tool under a translation seen from a rotation — the rack rolling on the blank's pitch
//! circle.  The fixture is one tooth space of a 20-tooth, module-2 gear at a 20° pressure angle:
//! no undercut, so the class's fold and crossing rows hold.
//!
//! Every number is in the page: the blank a disc extruded 6 mm along the page's normal (`depth:`
//! runs from −6 to 0), the rack tooth a trapezoid extruded past both of its faces, −8 to 2 (its
//! caps clear of the blank), the
//! blank turning about the page's normal through `o` (`motion(about: o)`).

use gcs_core::solid::admission::{admit_body, Condition, Error, Options};

use crate::common::build;

/// Pitch radius 20, module 2: addendum circle 22, the rack's tip `m` below the pitch line (the
/// root circle 18), its tooth `π m / 2` thick at the pitch line, flanks 20° from the radius.
/// The tip stays inside the line of action, which ends `r sin² α` = 2.34 below the pitch line:
/// a flank reaching deeper generates the involute's folded branch below the base circle, which a
/// real cutter's tip trims away and the class's fold row (E3) refuses.  The rack rolls on the
/// pitch circle: `2π · 20` mm of slide a turn of the blank.
pub const SPACE: &str = "\
unit mm
o := point
fix(x == 0, y == 0) o
c := circle(center: o) hint(r: 22)
radius(22mm) c
disc := face(c)
blank := solid(disc, depth: 6mm)

s0 := point hint(x: 20, y: 0)
s1 := point hint(x: 20, y: 10)
fix(x == 20, y == 0) s0
fix(x == 20, y == 10) s1
slide := line(s0, s1)

t0 := point hint(x: 18, y: -0.84)
t1 := point hint(x: 18, y: 0.84)
t2 := point hint(x: 24, y: 3.03)
t3 := point hint(x: 24, y: -3.03)
fix(x == 18, y == -0.842856) t0
fix(x == 18, y == 0.842856) t1
fix(x == 24, y == 3.026666) t2
fix(x == 24, y == -3.026666) t3
tooth := face(t0, t1, t2, t3, -> close)
construction rack_tooth := solid(tooth, from: -8mm, to: 2mm)

turn := motion(about: o, ratio: 1)
rack := motion(along: slide, advance: 2 * pi * 20mm)
cutting := motion(rack, relative_to: turn)
construction space := solid(rack_tooth, under: cutting, from: -60deg, to: 60deg)
gear := solid(blank)
space cut gear
";

fn admit(src: &str) -> Result<gcs_core::solid::admission::Admission, Error> {
    let mut e = build(src);
    let r = gcs_core::solve::solve(&mut e.sketch, gcs_core::solve::SolveOpts::default());
    assert!(r.success, "{}", r.message);
    let gear = e.map.ent_named("gear").unwrap().i();
    admit_body(&e.sketch, gear, &Options::default())
}

/// **The rack's sweep is in the class.**  A prism tool (its sides extruded lines), a translation
/// seen from a rotation, clear of the blank at both ends of the roll, each tool point touching
/// once, no fold, no crossing: admitted, with contacts in the blank.
#[test]
fn a_racks_sweep_is_admitted() {
    let admission = match admit(SPACE) {
        Ok(a) => a,
        Err(e) => panic!("refused: {e}"),
    };
    let sweep = &admission.sweeps()[0];
    assert!(sweep.contacts > 100, "only {} contacts in the blank", sweep.contacts);
    assert!(sweep.least_area_factor > 1e-3);
}

/// A rack tooth no longer than the blank is thick has caps in it, and the class cuts with a
/// prism's sides only: refused at T1, naming the cap.
#[test]
fn a_rack_with_a_cap_in_the_blank_is_refused() {
    let src = SPACE.replace("solid(tooth, from: -8mm, to: 2mm)", "solid(tooth, from: -5mm, to: 2mm)");
    match admit(&src) {
        Err(Error::Refused(r)) => {
            assert_eq!(r.condition, Condition::Tool);
            assert!(r.message.contains("a cap of the prism"), "{}", r.message);
        }
        other => panic!("not refused at T1: {:?}", other.map(|_| ())),
    }
}

