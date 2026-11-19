//! **Where a round's edge ends on a face all but along it** (#952), from a cast bracket's side
//! plate: a plate drawn on the side plane, 13.2 thick, a lug's disc unioned onto it, and a rim
//! rounded 2.  A face a run ends on is no end of it where its surface goes back across the run
//! itself — the plate's side running into the lug's rim at a slant cuts the arc a second time —
//! and the ring ends square there, as at a seam.  Two runs turning a degree from one another at a
//! corner of the plate meet in the plane halving the turn, each cut there and glued to the other;
//! a run carried past its end along the next side goes no further than that side does, where the
//! plate turns back in; and at a corner turning back in by less than a right angle each run ends
//! square, the next side's plane no face it ends on.  Each volume is its closed form's to 1e-9
//! (Pappus for a ring, the stack's mitre at a convex corner, nothing more at a reflex one), and
//! each solid is exported with no sliver.
use gcs_core::brep::{export, props, sweep::Say};
use std::f64::consts::PI;

use crate::common::rounds::square_run;
use crate::common::{close, ent, text::solved};

/// The plate's thickness: drawn on the side plane from 40.75 to 53.95.
const THICK: f64 = 13.2;
/// The round's radius.
const R: f64 = 2.0;
/// The lug's radius, about (50.75, 1.55) on the side plane.
const LUG: f64 = 7.5;

/// A document drawing the plate through `outline` (its corners on the side plane, in order), then
/// `rest`.
fn plate(outline: &[[f64; 2]], rest: &str) -> String {
    let n = outline.len();
    let steps: Vec<String> = (1..=n).map(|k| {
        let [u, v] = outline[k % n];
        format!("step(u: {u}mm, v: {v}mm) line")
    }).collect();
    format!("unit mm\nuse std\nuse std.solids\nuse std.solids (union)\nuse std.rounds (round)\n\
        in std.side {{\n  outline := {} -> close\n}}\n\
        plate := solid(std.solids.Prism(outline, from: 40.75mm, to: 53.95mm))\n{rest}", steps.join(" -> "))
}

/// The lug unioned onto the plate, and its rim on the plate's near side rounded.
const LUGGED: &str = "in std.side {\n  c := point\n  fix((50.75mm, 1.55mm)) c\n\
    lug := circle(center: c) hint(r: 7.5)\n  radius(7.5mm) lug\n}\n\
    lugt := solid(std.solids.Prism(face(lug), from: 40.75mm, to: 53.95mm))\n\
    body := solid(plate)\nlugt union body\npart := solid(body)\n\
    part.body.lugt.near round(r: 2mm) part.body.lugt\n";

/// The solid `name` built exactly, and exported: its STEP written, and no face of it thinner than
/// the export's tolerance.
fn exported(src: &str, name: &str) -> f64 {
    let e = solved(src);
    assert!(e.diags.is_empty(), "{:?}", e.diags);
    let say = Say { stage: &|_: &str| {}, mark: &|_| {} };
    let body = ent(&e, name).i();
    let exact = export::exact(&e.sketch, body, None, None, &say).unwrap_or_else(|r| panic!("{name}: {r:?}"));
    if let Some(s) = export::slivers(&e.sketch, body, &exact, None) {
        panic!("{}", s.message);
    }
    export::step(&exact, name, None, &say).unwrap();
    props::volume(&exact.solid)
}

/// A polygon's area (shoelace), signed: positive counter-clockwise.
fn area(p: &[[f64; 2]]) -> f64 {
    0.5 * (0..p.len()).map(|i| { let (a, b) = (p[i], p[(i + 1) % p.len()]); a[0] * b[1] - b[0] * a[1] }).sum::<f64>()
}

/// How far in from each face the centroid of a round's section stands (`square_run`'s).
fn inset(r: f64) -> f64 { (10.0 - 3.0 * PI) / (3.0 * (4.0 - PI)) * r }

/// What the lug's rim rounded takes, the ring over `turn` of its arc ending square at both ends
/// (Pappus: the section's centroid turned about the lug's axis).
fn ring(turn: f64) -> f64 { square_run(R) * (LUG - inset(R)) * turn }

/// **A plate's side running into the lug's rim all but along it**: the plate's top runs into the
/// lug tangent, its bottom at 1.77° to the rim, whose plane goes on under the arc for 3.5° and
/// comes back across it — no side of it holds the ring, which was kept under it to a lens and could
/// not be glued to its other quarter ("two pieces joined at a section share no face there").  The
/// ring ends square at both ends: the half disc's rim.  And the plate's top crossing the rim (the
/// plate's corner inside the disc), whose plane cuts the arc a second time across the disc: the
/// ring from that crossing round to the bottom.
#[test]
fn a_ring_ends_square_where_the_face_it_ends_on_goes_back_across_it() {
    let slanted = [[50.75, 9.05], [50.75, -5.95], [41.05, -6.25], [35.75, 9.05]];
    let whole = (area(&slanted).abs() + 0.5 * PI * LUG * LUG) * THICK;
    close(exported(&plate(&slanted, LUGGED), "part"), whole - ring(PI));
    // the plate's top at 8.05, crossing the rim at 6.5 above the lug's centre: the disc over it
    // is half of its segment there, and the ring turns from that crossing round to the bottom
    let crossed = [[33.05, 14.75], [44.05, 8.05], [50.75, 8.05], [50.75, -5.95], [41.05, -6.25], [35.05, -11.25]];
    let (h, w) = (6.5, (LUG * LUG - 6.5 * 6.5).sqrt());
    let segment = LUG * LUG * (h / LUG).acos() - h * w;
    let whole = (area(&crossed).abs() + 0.5 * PI * LUG * LUG + 0.5 * segment) * THICK;
    close(exported(&plate(&crossed, LUGGED), "part"), whole - ring(h.atan2(-w) + 0.5 * PI));
    // and its bottom crossing the rim too, 6.5 below the lug's centre: the plate's near face and
    // the lug's are pieces of one plane, the seam between them no seam a round spills over (bug
    // 3's), and the ring turns from one crossing round to the other
    let both = [[33.05, 14.75], [44.05, 8.05], [50.75, 8.05], [50.75, -4.95], [41.05, -4.95], [35.05, -11.25]];
    let whole = (area(&both).abs() + 0.5 * PI * LUG * LUG + segment) * THICK;
    close(exported(&plate(&both, LUGGED), "part"), whole - ring(2.0 * h.atan2(-w)));
}

/// The rim of a plate drawn through `outline` rounded all round: a run along each side less the
/// stack's mitre at each convex corner, and at each reflex corner turning back less than a right
/// angle each run ending square — the next side's plane, which reached back over its section
/// `s / tan(turn)` from the corner at depth `s` in from its own side, no face it ends on.
fn rim(outline: &[[f64; 2]]) {
    let turning = area(outline).signum();
    let (mut perimeter, mut cots) = (0.0, 0.0);
    for i in 0..outline.len() {
        let [a, b, c] = [outline[(i + outline.len() - 1) % outline.len()], outline[i], outline[(i + 1) % outline.len()]];
        let (din, dout) = ([b[0] - a[0], b[1] - a[1]], [c[0] - b[0], c[1] - b[1]]);
        perimeter += din[0].hypot(din[1]);
        let (cross, dot) = (din[0] * dout[1] - din[1] * dout[0], din[0] * dout[0] + din[1] * dout[1]);
        let turn = cross.abs().atan2(dot);
        assert!(cross * turning > 0.0 || turn < 0.5 * PI, "a reflex corner of a right angle or more");
        if cross * turning > 0.0 {
            // convex: the interior angle's half cotangent, `tan(turn / 2)`
            cots += (0.5 * turn).tan();
        }
    }
    let taken = (1.0 - PI / 4.0) * R * R * perimeter - (5.0 / 3.0 - PI / 2.0) * R.powi(3) * cots;
    let src = plate(outline, "body := solid(plate)\nbody.plate.near round(r: 2mm) body.plate\n");
    close(exported(&src, "body"), area(outline).abs() * THICK - taken);
}

/// **Two runs turning a degree from one another, beside a corner turning back in**: two sides of
/// the plate 1.13° apart, the first of them turning back in from the side before at 39.8°.
/// Carried past their corner until its whole section had crossed the other's face, each run went
/// a hundred millimetres on along the other, its round all but on the other's: past the reflex
/// corner, a groove along the side's plane through the plate's top, and where the other's run was
/// cut there, a tongue of its cap 5 µm wide between the two rounds.  Now the two are cut in the
/// plane halving their turn and glued there, and a run carried along a side ends where that side
/// does — square, where it ends at the reflex corner.
#[test]
fn runs_a_degree_apart_meet_halfway_and_carry_no_further_than_the_side() {
    rim(&[[50.75, 9.05], [50.75, -6.25], [41.05, -6.25], [35.05, -11.25], [25.05, -19.25], [25.05, 9.05]]);
}

/// **A side turning back in by a few degrees**: the plate's corner turning 77.8° and the side after
/// it 7.6° back in.  The next side's plane, kept off as a face the run ended on, reached back over
/// the round's section along the whole of the side (`2 / tan 7.6°`, 15 of its 15.5) and cut the
/// round away there, its cap meeting the top's round at its touch in a sliver 6 µm wide; each run
/// now ends square at the reflex corner.
#[test]
fn a_side_turning_back_in_by_a_few_degrees_cuts_no_round_back() {
    rim(&[[16.15, 55.95], [19.4481, 40.75], [24.9996, 25.35], [24.9996, 0.0], [-24.95, 0.0], [-24.95, 55.95]]);
}
