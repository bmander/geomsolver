//! The crate on its own: what its README promises, built and measured through the public API.
//! (The kernel's full suite is still geomsolver's, `rust/gcs-core/tests`.)
use cad_kernel::boolean::{combined, Op};
use cad_kernel::build::{disc, prism, square};
use cad_kernel::mesh::mesh;
use cad_kernel::props::{area, volume};
use std::f64::consts::PI;

type Result = std::result::Result<(), Box<dyn std::error::Error>>;

#[test]
fn a_block_drilled_through() -> Result {
    let block = prism(&square([0., 0., 0.], [0., 0., 1.], 5.), 0., 10.)?;
    let pin = prism(&disc([0., 0., -1.], [0., 0., 1.], 3.), 0., 12.)?;
    let drilled = combined(&block, &pin, Op::Cut, 0.)?;
    assert!((volume(&drilled) - (1000. - 90. * PI)).abs() < 1e-6);
    let a: f64 = drilled.faces.iter().map(|f| area(&drilled, f)).sum();
    assert!((a - (600. + 42. * PI)).abs() < 1e-6);
    Ok(())
}

#[test]
fn two_cylinders_crossing_share_a_steinmetz_solid() -> Result {
    let r = 5.;
    let along_x = prism(&disc([-15., 0., 0.], [1., 0., 0.], r), 0., 30.)?;
    let along_y = prism(&disc([0., -15., 0.], [0., 1., 0.], r), 0., 30.)?;
    let both = combined(&along_x, &along_y, Op::Common, 0.)?;
    assert!((volume(&both) - 16. * r * r * r / 3.).abs() < 1e-6);
    Ok(())
}

#[test]
fn a_mesh_is_closed_and_bounds_the_solid() -> Result {
    let block = prism(&square([0., 0., 0.], [0., 0., 1.], 5.), 0., 10.)?;
    let pin = prism(&disc([0., 0., -1.], [0., 0., 1.], 3.), 0., 12.)?;
    let drilled = combined(&block, &pin, Op::Cut, 0.)?;
    let m = mesh(&drilled, 1e-3, 0.1)?;
    // every edge used once each way
    let mut uses = std::collections::BTreeMap::new();
    for t in &m.tris {
        for k in 0..3 { *uses.entry((t[k], t[(k + 1) % 3])).or_insert(0) += 1; }
    }
    assert!(uses.iter().all(|(&(a, b), &n)| n == 1 && uses.get(&(b, a)) == Some(&1)));
    let v: f64 = m.tris.iter().map(|t| {
        let [a, b, c] = t.map(|i| m.pts[i as usize]);
        (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
            + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.
    }).sum();
    assert!((v / volume(&drilled) - 1.).abs() < 1e-3);
    Ok(())
}

#[test]
fn a_cylinder_turned_about_another_axis_touches_its_envelope_where_closed_forms_say() -> Result {
    use cad_kernel::sweep::envelope::contact;
    use cad_kernel::sweep::motion::{Family, Step};
    use cad_kernel::sweep::surface::{Meridian, RevolvedSurface};
    use std::f64::consts::TAU;
    // a cylinder of radius 2 about the line x = 10 along z, turned about the z axis
    let turn = Family::new("turn".into(), vec![Step::Rotation {
        origin: [0.; 3], axis: [0., 0., 1.], ratio: 1., advance: 0., law: None }])?;
    let screw = turn.twist().ok_or("a turn is a constant twist")?;
    assert!(screw.is_turn() && screw.ratio == 1.);
    let wall = RevolvedSurface::new("wall".into(), Meridian::Line { start: [12., 0., 0.], delta: [0., 0., 1.] },
        [10., 0., 0.], [0., 0., 1.], TAU);
    for (i, t) in [0., 0.3, 1.7, -2.5].into_iter().enumerate() {
        let pose = turn.at(t)?;
        // the pose turns a point by the roll
        let p = pose.point([1., 0., 0.]);
        assert!((p[0] - t.cos()).abs() < 1e-14 && (p[1] - t.sin()).abs() < 1e-14);
        // the wall's normal is inward, so n · v = −10 sin φ at the wall's angle φ, whatever the roll
        let v = i as f64 / 7.;
        let c = contact(wall.at(0.5, v).map_err(|e| format!("{e:?}"))?, pose).map_err(|e| format!("{e:?}"))?;
        assert!((c.normal_velocity + 10. * (TAU * v).sin()).abs() < 1e-12, "{}", c.normal_velocity);
    }
    Ok(())
}
