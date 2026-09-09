//! Source components checked against the separate generating-field construction.
use super::*;

#[test]
fn declarative_gear_space_matches_active_flanks_within_reachable_blank() {
    let pair = Pair::read([24,48],2.);
    let e = &pair.model;
    let source = SpatialField::read(&e.sketch,
        e.map.ent_named("pair.gear_space.body").unwrap().i(),pair.module*1e-10).unwrap();
    let reference = closure::GearSpace::read(&pair).field;
    let mut counts = [0;2];
    // All generating and indexing motions fix the apex. This ball covers every
    // point queried by a cut, including the neighboring cutter's rotated frame.
    for x in -24..=24 { for y in -24..=24 { for z in -2..=2 {
        let p = [x as f64*pair.rm/20.,y as f64*pair.rm/20.,z as f64*pair.module/2.];
        if p.iter().map(|v| v*v).sum::<f64>().sqrt() > 1.1*pair.rm { continue; }
        let b = p.map(|v| I::point(v).unwrap());
        let expected = reference.bounds(b).unwrap().bounds();
        if expected[0] <= 1e-7 && expected[1] >= -1e-7 { continue; }
        let actual = source.bounds(b).unwrap().bounds();
        let inside = expected[1] < 0.;
        assert!(if inside { actual[1] < 0. } else { actual[0] > 0. },
            "{p:?}: expected {expected:?}, source {actual:?}");
        counts[usize::from(inside)] += 1;
    } } }
    assert!(counts[0] > 1000 && counts[1] > 10,"{counts:?}");
}
