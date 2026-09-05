//! Run with `cargo run --release -p gcs-core --example solid_cost`.
use gcs_core::{gltf, program, solid, solve, syntax};
use std::time::Instant;
fn main() {
    for (name, source) in [
        (
            "box",
            include_str!("../tests/fixtures/solid_issue51/glb_control.sv").to_string(),
        ),
        (
            "boolean",
            include_str!("../tests/fixtures/solid_issue51/boolean_extent_0.sv").to_string(),
        ),
        (
            "bore",
            include_str!("../tests/fixtures/solid_issue51/ghost_dimension_through.sv")
                .replace("x: 20, y: 20", "x: 5, y: 5"),
        ),
    ] {
        let (p, _) = syntax::parse(&source);
        let mut e = program::elaborate(&p);
        assert!(e.ok());
        assert!(solve::solve(&mut e.sketch, Default::default()).success);
        let i = e.map.ent_named("result").unwrap().i();
        for (policy, unit) in [("mesh", 0.0), ("report", solid::REPORT_UNIT)] {
            let mut cold = Vec::new();
            let mut warm = Vec::new();
            let mut triangles = 0;
            for _ in 0..5 {
                e.sketch.solid_cache.borrow_mut().clear();
                let start = Instant::now();
                let _ = e.sketch.solid_boundary(i, unit);
                let _ = e.sketch.solid_edges(i, unit);
                triangles = e.sketch.solid_mesh(i, unit).positions.len() / 9;
                std::hint::black_box(gltf::glb(&e.sketch, &[i], unit));
                cold.push(start.elapsed().as_secs_f64() * 1000.0);
                let start = Instant::now();
                std::hint::black_box(gltf::glb(&e.sketch, &[i], unit));
                warm.push(start.elapsed().as_secs_f64() * 1000.0);
            }
            cold.sort_by(f64::total_cmp);
            warm.sort_by(f64::total_cmp);
            println!(
                "{name}/{policy}: cold {:.3} ms, cached export {:.3} ms, {triangles} triangles",
                cold[2], warm[2]
            );
        }
    }
}
