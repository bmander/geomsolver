//! **The hypoid's blank, as a region** (#145, F2/F3): the gate between the members' blank and
//! the checks'.  Each member is cut from `solid(R)`, `R` its blank's region — within the tip
//! cone and the heel sphere, outside the toe sphere and the back cone, sized by the solved
//! layout (`blank/member.sv`) — while the checks draw the same limits as revolved sections for
//! their named walls (`blank/limits.sv`).  In `pair.sv`, which has both, the region must be the
//! solid the drawn limits make by the body rule (the heel's carrier within the tip's, less the
//! toe's and the back's): the same volume exactly, the same material at points either side, for
//! the configured hypoid and the bevel pair.
use gcs_core::model::{SolidDef, SolidE};
use gcs_core::program::Elaborated;
use gcs_core::solid::MaterialField;
use gcs_core::space::{add, cross, norm, scale, sub};

type V = [f64; 3];

fn named(e: &Elaborated, name: &str) -> usize {
    e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i()
}

fn point(e: &Elaborated, name: &str) -> V {
    e.sketch.world_point(named(e, name))
}

fn line(e: &Elaborated, name: &str) -> (V, V) {
    let l = &e.sketch.lines[named(e, name)];
    (e.sketch.world_point(l.p1 as usize), e.sketch.world_point(l.p2 as usize))
}

fn unit(v: V) -> V {
    scale(v, 1.0 / norm(v))
}

/// The drawn limits' blank of member `m`, by the body rule over their carriers — added to the
/// drawing, since the checks draw the limits and make no blank of them.
fn drawn(e: &mut Elaborated, m: &str) -> usize {
    let carrier = |n: &str| named(e, &format!("{m}_limits.{n}.carrier")) as u32;
    let def = SolidDef::Body {
        stock: carrier("heel"),
        on: Vec::new(),
        through: vec![carrier("toe"), carrier("back")],
        bound: vec![carrier("tip")],
    };
    e.sketch.solids.push(SolidE { def, name: format!("{m}_drawn"), class: Default::default() });
    e.sketch.solids.len() - 1
}

#[test]
fn the_blank_is_its_region() {
    let project = fixtures::gear::project();
    let pair = std::fs::read_to_string(project.join("pair.sv")).unwrap();
    for (label, configuration) in fixtures::gear::designs().into_iter().take(2) {
        let mut e = fixtures::read_beside(&pair, &project,
            &mut |name, text| if name == "configuration" { configuration.clone() } else { text });
        for (m, apex) in [("gear", "gear.O"), ("pinion", "pinion.A")] {
            let made = drawn(&mut e, m);
            let region = named(&e, &format!("pair.{m}_blank.material"));
            assert!(matches!(e.sketch.solids[region].def, SolidDef::Region { .. }));
            let volume = |s: usize| {
                let x = e.sketch.exact_solid(s).unwrap_or_else(|why| panic!("{label} {m}: {why}"));
                gcs_core::brep::props::volume(&x.brep)
            };
            let (v, want) = (volume(region), volume(made));
            assert!((v - want).abs() <= 1e-9 * want, "{label} {m}: {v} against {want}");
            // the same material at points through the blank and round it
            let field = |s| MaterialField::read(&e.sketch, s, 1e-10).unwrap();
            let (r, t) = (field(region), field(made));
            let o = point(&e, &format!("pair.{apex}"));
            let (a0, a1) = line(&e, &format!("pair.{m}.ax"));
            let d = unit(sub(a1, a0));
            let up = if d[2].abs() < 0.9 { [0.0, 0.0, 1.0] } else { [1.0, 0.0, 0.0] };
            let across = unit(cross(d, up));
            let heel = point(&e, &format!("{m}_limits.span.heel"));
            let reach = norm(sub(heel, o)) * 1.2;
            let (mut decided, mut agreed) = (0, 0);
            for i in 0..24 {
                for j in 0..24 {
                    let x = add(o, add(scale(d, reach * (i as f64 / 23.0 * 2.0 - 1.0)),
                        scale(across, reach * j as f64 / 23.0)));
                    let (a, b) = (r.side(x), t.side(x));
                    if a.abs() < 1e-6 || b.abs() < 1e-6 {
                        continue;
                    }
                    decided += 1;
                    agreed += usize::from((a < 0.0) == (b < 0.0));
                }
            }
            assert!(decided > 400 && agreed == decided, "{label} {m}: {agreed} of {decided}");
        }
    }
}
