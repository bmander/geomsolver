//! The solid gallery must contain material, preserve its holes, and render its authored sheets.
use gcs_core::{drawing, examples, library, program, solid, solve};
use std::{collections::BTreeMap, f64::consts::PI};

#[test]
fn gallery_solids_have_the_designed_volume_and_render() {
    let cases = [
        ("solid_loft", include_str!("../../examples/solid_loft.svd"), 5120.0),
        (
            "solid_flange",
            include_str!("../../examples/solid_flange.svd"),
            PI * ((32.0_f64.powi(2) - 8.0_f64.powi(2)) * 6.0
                + (15.0_f64.powi(2) - 8.0_f64.powi(2)) * 10.0
                - 6.0 * 3.0_f64.powi(2) * 6.0),
        ),
        (
            "solid_pulley",
            include_str!("../../examples/solid_pulley.svd"),
            PI * ((28.0_f64.powi(2) - 6.0_f64.powi(2)) * 18.0
                + (12.0_f64.powi(2) - 6.0_f64.powi(2)) * 10.0)
                - 2.0 * PI * 26.0 * (180.0 / 7.0),
        ),
        (
            "solid_elbow",
            include_str!("../../examples/solid_elbow.svd"),
            0.25 * 2.0 * PI * 30.0 * (18.0_f64.powi(2) - 14.0_f64.powi(2)),
        ),
        (
            "solid_tray",
            include_str!("../../examples/solid_tray.svd"),
            72.0 * 48.0 * 3.0 + (72.0 * 48.0 - 66.0 * 42.0) * 13.0 + 4.0 * PI * (25.0 - 4.0) * 8.0,
        ),
    ];
    for (key, sheet, volume) in cases {
        assert!(examples::CASES.iter().any(|(_, k, _)| *k == key));
        let (p, parse_errors, link_errors) = library::parse_linked(examples::source(key).unwrap());
        assert!(
            parse_errors.is_empty() && link_errors.is_empty(),
            "{key}: {parse_errors:?} {link_errors:?}"
        );
        let mut e = program::elaborate(&p);
        assert!(e.ok(), "{key}: {:?}", e.diags);
        // The contour must be determined by its relationships, not by the supplied coordinates.
        // Disturb every free point before solving; the same dimensions and volumes must return.
        for i in 0..e.sketch.points.len() {
            for (axis, param) in e.sketch.point_params(i).into_iter().enumerate() {
                let p = &mut e.sketch.params[param as usize];
                if !p.fixed {
                    p.value += 0.3 * ((i * 7 + axis * 3) as f64).sin();
                }
            }
        }
        assert!(
            solve::solve(&mut e.sketch, Default::default()).success,
            "{key}"
        );
        // the object a sheet draws: the flange takes its features on its own sweep
        let object = if key == "solid_flange" { "flange" } else { "body" };
        let body = e.map.ent_named(object).unwrap().i();
        assert_eq!(
            gcs_core::diagnose::diagnose(&mut e.sketch, Default::default()).dof,
            0,
            "{key}: the design must determine all of its geometry"
        );
        let evaluated = e
            .sketch
            .evaluated_solid(body, solid::ApproximationPolicy::Mesh)
            .unwrap();
        assert!(
            (evaluated.volume() - volume).abs() / volume < 0.002,
            "{key}: expected {volume}, got {}",
            evaluated.volume()
        );
        assert!(evaluated.stl().unwrap().len() > 84, "{key}: empty STL");

        if key == "solid_tray" {
            for (point, material) in [
                ([0.0, 14.0, 0.0], true),   // floor
                ([0.0, 5.0, 0.0], false),   // open pocket
                ([29.0, 9.0, 14.0], true),  // boss survives the pocket cut
                ([26.0, 9.0, 14.0], false), // blind screw bore
                ([26.0, 14.0, 14.0], true), // floor underneath the bore
            ] {
                assert_eq!(
                    evaluated.contains_world(solid::WorldPoint(point)),
                    material,
                    "{point:?}"
                );
            }
        }
        let doc = drawing::parse(sheet).unwrap();
        let models = BTreeMap::from([(
            "m".into(),
            drawing::Model {
                sketch: &e.sketch,
                names: &e.map,
            },
        )]);
        let svg = drawing::render(&doc, &models, None).unwrap();
        assert!(
            svg.contains("data-path="),
            "{key}: no solid edges in drawing"
        );
    }
}
