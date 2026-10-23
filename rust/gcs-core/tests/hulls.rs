//! **A filled body's mesh moved with the drawing** (#567): a solid that is no prism, its exact
//! boundary refined and filled, its nodes read as functions of the drawing's numbers — each node
//! of the boundary on the sheets it was meshed on (`interior::Hull`, `hulls::hull_of`), each
//! inside following them harmonically.  The rules are `docs/contract/analysis.md`'s.
use gcs_core::delaunay::tetra::Tet10;
use gcs_core::diagnose::analysis::scenario_read;
use gcs_core::model::Sketch;
use gcs_core::solid::faces::BodyFaces;

/// `src` read and solved, and its solid made of a material.
fn solid(src: &str) -> (Sketch, usize) {
    let sk = crate::common::solved(&crate::common::read(src));
    let s = (0..sk.solids.len()).find(|&s| sk.solids[s].material.is_some()).expect("a solid");
    (sk, s)
}

/// The library case `key`'s source, `from` written `to`.
fn edited(key: &str, edits: &[(&str, &str)]) -> (String, String) {
    let src = gcs_core::examples::source(key).unwrap().to_string();
    let mut to = src.clone();
    for (a, b) in edits {
        assert!(to.contains(a), "{key}: no `{a}`");
        to = to.replace(a, b);
    }
    (src, to)
}

/// **A filled body's mesh moves with the drawing** (#567, M1): on the pin lug (its hole's
/// diameter and its width changed: a cylinder cut through a block) and the turned shaft (its
/// barrel's radius and its shoulder's round: planes, cylinders and a torus turned about an axis),
/// meshed at four times a reading's size, every node stands where it was meshed at the compile's
/// pose to the bit; moved to the drawing solved at the changed dimensions, every node of the
/// boundary stands on the boundary rebuilt there as near as it stood on the one it was meshed on
/// (to 1e-9 of the extent), some as far as the dimensions changed, and no element is turned over
/// (its Jacobian's Bernstein coefficients, `Tet10::jacobian_ratio`).
#[test]
fn a_filled_bodys_mesh_moves_with_the_drawing() {
    let cases = [
        (edited("pin_lug", &[("d := 16mm", "d := 17mm"), ("w := 40mm", "w := 41mm")]), 1.0),
        (edited("turned_shaft", &[("R := 20mm", "R := 21mm"), ("rho := 4mm", "rho := 4.5mm")]),
         1.0),
    ];
    for ((from, to), most) in &cases {
        let (sk0, s) = solid(from);
        let (sk1, _) = solid(to);
        let mesh = gcs_core::bodies::mesh(&sk0, s, &[], 4.0).expect("meshed");
        let (hull, cols) = gcs_core::hulls::hull_of(&sk0, s, &mesh).expect("its hull");
        let at = |sk: &Sketch| -> Vec<f64> {
            cols.iter().map(|&c| sk.params[c as usize].value).collect()
        };
        let tets: Vec<[usize; 10]> = mesh.tets.clone();
        let here = hull.places(&at(&sk0), &tets, true).expect("placed where it was meshed");
        let x0: Vec<f64> = mesh.nodes.iter().flatten().copied().collect();
        assert!(here.x.iter().zip(&x0).all(|(a, b)| a.to_bits() == b.to_bits()),
                "{}: moved where it was meshed", sk0.solids[s].name);
        let there = hull.places(&at(&sk1), &tets, true).expect("placed where the drawing moved");
        let (b0, b1) = (BodyFaces::whole(&sk0, s).unwrap(), BodyFaces::whole(&sk1, s).unwrap());
        let extent = hull.extent;
        let (mut tagged, mut moved) = (0, 0.0f64);
        for i in 0..mesh.nodes.len() {
            if hull.nodes[i].is_empty() {
                continue;
            }
            tagged += 1;
            let p = [there.x[3 * i], there.x[3 * i + 1], there.x[3 * i + 2]];
            let (d0, d1) = (b0.signed_distance(mesh.nodes[i]).0.abs(), b1.signed_distance(p).0.abs());
            assert!(d1 <= d0 + 1e-9 * extent, "node {i} on {:?}: {d1} off where it stood {d0} off",
                    hull.nodes[i]);
            moved = moved.max(gcs_core::space::distance(p, mesh.nodes[i]));
        }
        assert!(moved >= 0.99 * most && tagged > 0, "{tagged} tagged, moved {moved}");
        let ratio = |x: &[f64], e: &[usize; 10]| {
            Tet10 { nodes: e.map(|n| [x[3 * n], x[3 * n + 1], x[3 * n + 2]]) }.jacobian_ratio()
        };
        let mut bad = 0;
        for e in &mesh.tets {
            let (r0, r1) = (ratio(&x0, e), ratio(&there.x, e));
            if !(r1 > 0.0) {
                bad += 1;
                eprintln!("turned over: {r0} -> {r1}: {:?}", e.map(|n| (n, hull.nodes[n].clone())));
                for n in e {
                    let p = [there.x[3 * n], there.x[3 * n + 1], there.x[3 * n + 2]];
                    eprintln!("  {n}: {:?} -> {:?}", mesh.nodes[*n], p);
                }
            }
        }
        assert!(bad == 0, "{bad} elements turned over");
        eprintln!("{}: {} nodes, {tagged} on its sheets, {} sheets, moved {moved:.4}",
                  sk0.solids[s].name, mesh.nodes.len(), hull.sheets.len());
    }
}

/// A pin lug whose hole's diameter `d` is an unknown, its corner's give under the pull on the
/// hole driven.
pub const LUG: &str = "unit mm
use std
l := 80mm
w := 40mm
t := 10mm
P := 20kN
param d: Length hint(16mm)
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)
in std.front {
  a := point
  b := point
  c := point
  e := point
  centre := point
  fix((0mm, 0mm)) a
  fix((l, 0mm)) b
  fix((l, w)) c
  fix((0mm, w)) e
  fix((l - w / 2, w / 2)) centre
  outline := (low := line(a, b)) -> (end := line(b, c)) -> (high := line(c, e)) ->
             (root := line(e, a)) -> close
  bore := radius(d / 2) circle(center: centre) hint(r: 8)
  hole := face(bore)
}
lug := solid(outline, from: -t / 2, to: t / 2) of steel
pin := solid(hole, from: -t, to: t)
pin cut lug
scenario pull {
  fixed lug.root
  load lug.pin.bore P along: std.x
  deflection(b, along: std.x) == SAG
}
";

/// A stepped shaft turned about its axis, its barrel's radius `R` an unknown, its head's give
/// under a pull on it driven.
pub const SHAFT: &str = "unit mm
use std (horizontal, vertical)
r := 12mm
rho := 4mm
shoulder := 60mm
L := 160mm
sigma := 60MPa
param R: Length hint(20mm)
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3, yield: 250MPa)
in std.front {
  o := point
  a := point hint((20, 0))
  b := point hint((20, 60))
  f := point
  c := point
  g := point
  h := point
  k := point
  fix((0mm, 0mm)) o
  o horizontal a
  o distance(R) a
  a vertical b
  b horizontal f
  fix((r + rho, shoulder)) f
  fix((r + rho, shoulder + rho)) c
  fix((r, shoulder + rho)) g
  fix((r, L)) h
  fix((0mm, L)) k
  base := line(o, a)
  barrel := line(a, b)
  step := line(b, f)
  round := arc(center: c, start: g, end: f) hint(r: rho)
  neck := line(g, h)
  head := line(h, k)
  spine := line(k, o)
  section := face(base, barrel, step, round, neck, head, spine)
}
shaft := solid(section, about: std.z) of steel
scenario pull {
  pinned shaft.base along: std.z
  pinned o along: std.x
  pinned o along: std.y
  pinned a along: std.y
  pressure shaft.head -sigma
  deflection(k, along: std.z) == SAG
}
";

/// A round bar turned about its axis, its radius `r` an unknown, held along its axis at its base
/// and pulled along it at its head — of no Poisson's ratio, so the points holding it across hold
/// nothing of a narrowing.
pub const BAR: &str = "unit mm
use std (horizontal)
L := 100mm
P := 10kN
param r: Length hint(10mm)
steel := material(E: 200GPa, nu: 0, rho: 7850kg / 1m^3, yield: 250MPa)
in std.front {
  o := point
  a := point hint((10, 0))
  b := point hint((10, 100))
  c := point
  fix((0mm, 0mm)) o
  fix((0mm, L)) c
  o horizontal a
  c horizontal b
  o distance(r) a
  c distance(r) b
  profile := (base := line(o, a)) -> (side := line(a, b)) -> (head := line(b, c)) ->
             (spine := line(c, o)) -> close
}
bar := solid(profile, about: std.z) of steel
scenario pull {
  pinned bar.base along: std.z
  pinned o along: std.x
  pinned o along: std.y
  pinned a along: std.y
  load bar.head P along: std.z
  deflection(c, along: std.z) == SAG
}
";

/// **A turned bar's radius is sized by its stretch** (#567, M4): its radius an unknown, its head
/// driven to move `δ` under a pull `P`, the solve finds `r = √(P L / (π E δ))` within 5e-4 — a
/// uniform bar pulled is read exactly by quadratic elements, so to the mesh's approximation of its
/// section's circle, the bar meshed at 10mm and moved to the 7.98mm found — leaving no freedom.
#[test]
fn a_turned_bars_radius_is_sized_by_its_stretch() {
    let sag = 0.025;
    let mut e = crate::common::text::solved(&BAR.replace("SAG", &format!("{sag}mm")));
    let r = e.sketch.params[*e.sketch.free_vars.get("r").expect("r") as usize].value;
    let want = (10e3 * 100.0 / (std::f64::consts::PI * 200e3 * sag)).sqrt();
    assert!((r - want).abs() <= 5e-4 * want, "{r} against {want}");
    assert_eq!(crate::common::dof(&mut e.sketch), 0);
    let s = &e.scenarios[0];
    let (words, read) = scenario_read(&e.sketch, &s.scenario);
    assert!(words.is_empty(), "{words:?}");
    let said = gcs_core::report::scenario_answers(&e.sketch, s, &read);
    assert!(said[0].text.contains(&format!("driven == {sag}mm, held")), "{}", said[0].text);
    // read afresh at the radius found, the claims' reading agrees with the row
    let fresh = read.answers[0].clone().unwrap().unwrap();
    let error = read.errors[0].unwrap_or(0.0);
    assert!((fresh - sag).abs() <= error.max(1e-4 * sag), "{fresh} ± {error} against {sag}");
}

/// **What a driving row does not move is said** (#567): a filled body whose faces are a drafted
/// prism's — tapered sides, which no sheet reads yet — has no row, and its driving deflection is
/// **E040** where it is written, naming the face.
#[test]
fn a_face_no_sheet_moves_is_said() {
    let src = LUG.replace("SAG", "0.015mm")
        .replace("lug := solid(outline, from: -t / 2, to: t / 2) of steel",
                 "lug := solid(outline, from: -t / 2, to: t / 2, draft: 2deg) of steel");
    let (prog, errs) = crate::common::parse(&src);
    assert!(errs.is_empty(), "{errs:?}");
    let e = gcs_core::program::elaborate(&prog);
    let said: Vec<_> = e.errors().map(|d| (d.code.as_str().to_string(), d.message.clone())).collect();
    assert!(said.iter().any(|(c, m)| c == "E040" && m.contains("is not moved")
                                     && m.contains("is drafted")), "{said:?}");
}

/// **The library's rod is sized by its sag** (#567, M5; the slow tier): `sized_conrod.sv`, the
/// engine's connecting rod imported, its web's thickness an unknown and its small end's corner
/// driven to sink 0.29mm under 30kN of firing load.  The rod is no prism; its row is made on a
/// coarse mesh, solved there, made again twice as fine and solved on, to a reading's own size.  It
/// solves to no freedom, the web found between the rod's 5mm and its flanges' 18mm, its row held
/// at a reading's own size; the report says it driven and held, and the claims' reading of the
/// solved drawing, meshed afresh, agrees with the row within its error.  Its work is pinned by
/// counts, not times: the last solve's evaluations, and its structure solved by CG over its
/// corners within 45 iterations at the size it is held at.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, minutes: the rod's web driven at three sizes")]
fn the_librarys_rod_is_sized_by_its_sag() {
    use gcs_core::constraints::{CKind, InteriorRow};
    let src = gcs_core::examples::source("sized_conrod").unwrap();
    let mut e = crate::common::read(src);
    let case = gcs_core::examples::case("sized_conrod").expect("a case");
    assert_eq!(gcs_core::io::dumps(&e.sketch, None), gcs_core::io::dumps(&case, None));
    let res = gcs_core::solve::solve(&mut e.sketch, gcs_core::solve::SolveOpts::default());
    assert!(res.success, "{}", res.message);
    eprintln!("solved: {} evaluations, {} rates in the last solve", res.nfev, res.njev);
    assert!(res.nfev <= 8 && res.njev <= 6, "{} evaluations, {} rates", res.nfev, res.njev);
    let sk = &mut e.sketch;
    let wt = sk.params[*sk.free_vars.get("wt").expect("wt") as usize].value;
    assert!(wt > 5.0 && wt < 18.0, "the web {wt}");
    assert!(sk.meshed.values().all(|r| r.coarser == 1.0), "held at a reading's own size");
    assert_eq!(crate::common::dof(sk), 0);
    let c = sk.constraints.iter().find(|c| c.kind == CKind::Deflection).expect("its row");
    let Ok(InteriorRow::Meshed(row)) = c.interior_row(sk) else { panic!("a mesh held") };
    let st = row.structure.solve(&row.x).expect("solved");
    let its = st.iterations().expect("by CG");
    assert!(its <= 45, "CG took {its} iterations");
    let s = &e.scenarios[0];
    let (words, read) = scenario_read(&e.sketch, &s.scenario);
    assert!(words.is_empty(), "{words:?}");
    let said = gcs_core::report::scenario_answers(&e.sketch, s, &read);
    assert!(said[0].text.contains("driven == 0.29mm, held"), "{}", said[0].text);
    let fresh = read.answers[0].clone().unwrap().unwrap();
    let error = read.errors[0].expect("an estimate");
    assert!((fresh - 0.29).abs() <= error.max(1e-3), "{fresh} ± {error} against 0.29mm");
    eprintln!("wt = {wt}; CG {its}; afresh {fresh} ± {error}");
}

/// **A filled body's mesh is made again where it folds** (#567, M5; the slow tier): the pin lug
/// driven to give 0.02mm, its hole's diameter grown from 16mm to 29.2mm in a lug 40mm wide.  The
/// mesh its row was made on at the drawing's start cannot be read where the drive ends — its
/// elements between the hole and the lug's sides turn over (the row reads NaN there) — so the
/// solve folds on it, is meshed again where it stopped (`Stop::Folded`, `Sketch::remesh`) and
/// solved on, and holds: the hole found, no freedom left, its row held at a reading's own size.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, half a minute: a lug's hole driven past its mesh")]
fn a_filled_bodys_mesh_is_made_again_where_it_folds() {
    use gcs_core::constraints::CKind;
    let src = LUG.replace("SAG", "0.02mm");
    let first = crate::common::read(&src);
    let mut e = crate::common::text::solved(&src);
    let sk = &mut e.sketch;
    let d = sk.params[*sk.free_vars.get("d").expect("d") as usize].value;
    assert!(d > 28.0 && d < 31.0, "{d}");
    assert!(sk.meshed.values().all(|r| r.coarser == 1.0));
    assert_eq!(crate::common::dof(sk), 0);
    // the mesh the drawing started on folds before the hole the drive ends at
    let c = first.sketch.constraints.iter().find(|c| c.kind == CKind::Deflection).unwrap();
    let mut there = first.sketch.clone();
    there.set_x(&e.sketch.get_x());
    let r = c.residual(&first.sketch, &c.local_values(&there))[0];
    assert!(r.is_nan(), "the first mesh reads {r} where the drive ends");
}
