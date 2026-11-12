//! The hypoid drive (`hypoid_drive/drive.sv`): the pair of `spiral_bevel/hypoid.sv` imported
//! whole, each member on a shaft in a plain sleeve in one frame inside the pair, each made a
//! printed part with a spoked hub, and a counterweighted crank on each shaft. Nothing of the pair
//! is restated, so what is held here is the drive: it solves on the pair to DOF 0, every part of
//! its own is built exactly, each crank turns balanced, and every claim the document makes holds.
use gcs_core::diagnose::{diagnose, judge_solids, DiagnoseOptions};
use gcs_core::program::Elaborated;
use std::path::PathBuf;

fn examples() -> PathBuf { PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../examples") }

fn source() -> String {
    std::fs::read_to_string(examples().join("hypoid_drive/drive.sv")).unwrap()
}

/// `text` elaborated with its modules read as solventc reads them from the drive's directory:
/// `spiral_bevel.hypoid` from the examples, `std` from the library — and solved.
fn read(text: &str) -> Elaborated { fixtures::read_beside(text, &examples(), &mut |_, t| t) }

/// The two ends of the named line, in space.
fn ends(e: &Elaborated, line: &str) -> [[f64; 3]; 2] {
    let l = e.map.ent_named(line).unwrap().i();
    e.sketch.line_ends(l).map(|p| e.sketch.world_point(p))
}

#[test]
fn the_drive_solves_on_the_imported_pair() {
    let e = read(&source());
    assert!(e.diags.is_empty(), "{:#?}", e.diags);
    let mut sk = e.sketch.clone();
    let d = diagnose(&mut sk, DiagnoseOptions { solid_claims: false, ..Default::default() });
    assert_eq!(d.dof, 0, "the pair, its face mill and the drive, every root chosen by a word");
    // the parts the drive makes of its own are built exactly: plates of half-spaces, the rest
    // revolutions, the spokes and splines placed copies of slabs
    for side in ["gear_side", "pinion_side"] {
        for part in ["shaft", "hub", "crank.body", "crank.handle", "crank.pin"] {
            let name = format!("{side}.{part}");
            let i = fixtures::solid(&e, &name);
            assert!(e.sketch.exact_solid(i).is_ok(), "{name}: {:?}", e.sketch.exact_solid(i).err());
        }
    }
    assert!(e.sketch.exact_solid(fixtures::solid(&e, "frame.body")).is_ok());
}

/// Volume, first moment and second moment (`∫ x xᵀ`) of a closed mesh about `o`.
fn moments(positions: &[f64], o: [f64; 3]) -> (f64, [f64; 3], [[f64; 3]; 3]) {
    let (mut v, mut m1, mut m2) = (0.0, [0.0; 3], [[0.0; 3]; 3]);
    for t in positions.chunks(9) {
        let p: Vec<[f64; 3]> =
            (0..3).map(|k| std::array::from_fn(|i| t[3 * k + i] - o[i])).collect();
        let (a, b, c) = (p[0], p[1], p[2]);
        let det = a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
            + a[2] * (b[0] * c[1] - b[1] * c[0]);
        let dv = det / 6.0;
        let s = [a[0] + b[0] + c[0], a[1] + b[1] + c[1], a[2] + b[2] + c[2]];
        v += dv;
        for i in 0..3 {
            m1[i] += dv * s[i] / 4.0;
            for j in 0..3 {
                m2[i][j] += dv * (a[i] * a[j] + b[i] * b[j] + c[i] * c[j] + s[i] * s[j]) / 20.0;
            }
        }
    }
    (v, m1, m2)
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 { a[0] * b[0] + a[1] * b[1] + a[2] * b[2] }

/// **Each crank turns balanced.**  Everything that goes round with a crank — its plate, the pin
/// pressed into it and the handle on the pin — has its centre of mass on the shaft (no shake) and
/// no product of inertia between the shaft and a direction square to it (no wobble), read off the
/// built meshes, not off the arithmetic `Crank` sizes its fan and pad by.  The plate alone,
/// without the pin and handle the fan was sized for, is neither.
#[test]
fn each_crank_spins_balanced() {
    let e = read(&source());
    let sk = &e.sketch;
    for (side, axis) in [("gear_side", "pair.gear_cone.ax"),
                         ("pinion_side", "pair.pinion_cone.ax")] {
        let [o, end] = ends(&e, axis);
        let d = { let r = [end[0] - o[0], end[1] - o[1], end[2] - o[2]]; let n = dot(r, r).sqrt();
                  [r[0] / n, r[1] / n, r[2] / n] };
        let read = |parts: &[&str]| {
            let (mut v, mut m1, mut m2) = (0.0, [0.0; 3], [[0.0; 3]; 3]);
            for part in parts {
                let mesh = sk.solid_mesh(fixtures::solid(&e, &format!("{side}.crank.{part}")), 0.0);
                let (a, b, c) = moments(&mesh.positions, o);
                v += a;
                for i in 0..3 {
                    m1[i] += b[i];
                    for j in 0..3 { m2[i][j] += c[i][j]; }
                }
            }
            // off the axis: the first moment's part square to it; the couple: `∫ r⊥ z`, z from
            // the centre of mass's station on the axis
            let along = dot(m1, d);
            let off: [f64; 3] = std::array::from_fn(|i| m1[i] - along * d[i]);
            let z = along / v;
            let m2d: [f64; 3] = std::array::from_fn(|i| dot(m2[i], d));
            let couple: [f64; 3] = std::array::from_fn(|i| {
                let c = m2d[i] - z * m1[i];
                c - dot(m2d, d) * d[i] + z * along * d[i]
            });
            (dot(off, off).sqrt() / v, dot(couple, couple).sqrt())
        };
        let (shake, wobble) = read(&["body", "pin", "handle"]);
        let (bare_shake, bare_wobble) = read(&["body"]);
        eprintln!("{side}: {shake:.4} mm off the axis, couple {wobble:.0} mm⁵; the body alone \
                   {bare_shake:.3} mm, {bare_wobble:.0} mm⁵");
        // the pin's shank and the eye's hole overlap by the press fit, a ring counted in both
        // meshes once each though a pressed part holds it once: 0.014 mm of this, at the throw
        assert!(shake < 0.02, "{side}: its centre of mass {shake} mm off the shaft");
        assert!(wobble < 1e-2 * bare_wobble,
                "{side}: a couple of {wobble} mm⁵ against {bare_wobble}");
        assert!(bare_shake > 1.0, "{side}: the fan was sized for the pin and handle too");
    }
}

/// **Every claim the drive makes holds** — the frame clear of both members, the shafts and hubs
/// running in it with their clearance and play, the handles loose on their pins, and each crank
/// clear of the frame and the other member through a whole turn — and each member's hub lies on
/// its blank's back face, nowhere apart from it.
#[test]
#[cfg_attr(not(feature = "slow"),
           ignore = "slow tier, about 10 min: the drive's claims, four swept")]
fn every_claim_the_drive_makes_holds() {
    let seated = "\nclaim gear_side.hub clear(1mm) pair.gear_blank.material\n\
                  claim pinion_side.hub clear(1mm) pair.pinion_blank.material\n";
    let e = read(&format!("{}{seated}", source()));
    let verdicts = judge_solids(&e.sketch);
    let (made, seats) = verdicts.split_at(verdicts.len() - 2);
    assert_eq!(made.len(), 17, "the document's claims");
    for v in made {
        assert_eq!(v.holds(), Some(true), "{}", gcs_core::report::solid_claim_text(v));
    }
    // the hub's rim and the blank's back are one cone, so their facets cross by the faceting:
    // what the measurement can say is that the two are nowhere apart
    for v in seats {
        let apart = v.measured().unwrap() + v.tolerance().unwrap();
        assert!(apart < 0.01, "a hub seated on its blank: {}",
                gcs_core::report::solid_claim_text(v));
    }
}
