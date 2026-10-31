//! The crate on its own: what its README promises, built and measured through the public API.
//! (The kernel's full suite is still geomsolver's, `rust/gcs-core/tests`.)
use cad_kernel::boolean::{combined, Op};
use cad_kernel::build::{cone, disc, prism, sphere, square, torus};
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
fn the_sphere_the_cone_and_the_torus_are_their_closed_forms() -> Result {
    let close = |a: f64, b: f64| (a - b).abs() <= 1e-9 * b;
    let up = [0., 0., 1.];
    assert!(close(volume(&sphere([1., 2., 3.], up, 5.)?), 500. / 3. * PI));
    assert!(close(volume(&cone([0., 0., 0.], up, 5., 2., 6.)?), 78. * PI));
    assert!(close(volume(&torus([0., 0., 0.], up, 10., 2.)?), 80. * PI * PI));
    // a ball and a block share a hemisphere, the ball's poles leaning off the block's floor
    let ball = sphere([0., 0., 0.], [0.3, 0.2, 1.], 5.)?;
    let above = prism(&square([0., 0., 0.], up, 10.), 0., 10.)?;
    assert!(close(volume(&combined(&ball, &above, Op::Common, 0.)?), 250. / 3. * PI));
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
