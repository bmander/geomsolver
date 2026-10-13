//! One sector of an indexed body (`solid::sector`, docs/native-speed-plan.md): the indexing read
//! off the placements' poses, and the sector's side through the gaps the cuts leave, on synthetic
//! cuts whose gaps are known — a spiral band, as a spiral bevel's tooth space turns across its face.
use gcs_core::envelope::Motion;
use gcs_core::solid::sector::{self,Boundary,Frame,Indexing,Slices};
use gcs_core::space::{add,sub,norm,cross};
use std::f64::consts::TAU;

type V = [f64;3];

/// The turn by `angle` about the line through `o` along `a`.
fn turn(o: V,a: V,angle: f64) -> Motion {
    Motion::translation(o.map(|x| -x),[0.;3]).unwrap().then(Motion::rotation(a,angle,0.).unwrap())
        .then(Motion::translation(o,[0.;3]).unwrap())
}

/// `count` placements of a sweep first placed at `first`, turned about the line (o, a), in a
/// shuffled order.
fn placements(first: Motion,o: V,a: V,count: usize) -> Vec<Motion> {
    let mut order: Vec<usize> = (0..count).collect();
    order.rotate_left(count/3);
    order.swap(0,count/3);
    let mut poses: Vec<Motion> = order.iter().map(|&k| first.then(turn(o,a,k as f64*TAU/count as f64))).collect();
    // the first placement listed is the sweep's own first
    let at = order.iter().position(|&k| k == 0).unwrap();
    poses.swap(0,at);
    poses
}

#[test]
fn the_indexing_is_read_off_the_placements() {
    let (o,a) = ([1.,2.,-3.],[0.3,-0.2,0.9]);
    let first = Motion::rotation([1.,1.,0.],0.4,0.).unwrap().then(Motion::translation([5.,0.,1.],[0.;3]).unwrap());
    let second = Motion::translation([0.,7.,2.],[0.;3]).unwrap();
    let sweeps = [placements(first,o,a,12),placements(second,o,a,12)];
    let indexing = sector::indexing(&sweeps,10.).unwrap();
    assert_eq!(indexing.count,12);
    let unit = a.map(|x| x/norm(a));
    assert!(norm(cross(indexing.axis,unit)) < 1e-12,"{:?}",indexing.axis);
    // the origin is on the line
    assert!(norm(cross(sub(indexing.origin,o),unit)) < 1e-9);
    // each placement is its index's turn of the sweep's first, and every index is taken once
    for (s,poses) in sweeps.iter().enumerate() {
        let mut seen = indexing.indices[s].clone();
        seen.sort();
        assert_eq!(seen,(0..12).collect::<Vec<_>>());
        for (j,&k) in indexing.indices[s].iter().enumerate() {
            let expected = poses[0].then(indexing.turn(k));
            for p in [[0.,0.,0.],[3.,-1.,2.],[10.,4.,-6.]] {
                assert!(norm(sub(poses[j].point(p),expected.point(p))) < 1e-9);
            }
        }
    }
}

#[test]
fn what_is_not_one_indexing_is_refused_with_its_reason() {
    let (o,a) = ([0.,0.,0.],[0.,0.,1.]);
    let first = Motion::translation([4.,0.,0.],[0.;3]).unwrap();
    let refused = |sweeps: &[Vec<Motion>],why: &str| {
        let e = sector::indexing(sweeps,10.).unwrap_err();
        assert!(e.contains(why),"{e}");
    };
    refused(&[placements(first,o,a,2)],"three or more");
    refused(&[placements(first,o,a,6),placements(first,o,a,5)],"no one indexing");
    // turned about a line elsewhere
    refused(&[placements(first,o,a,6),placements(first,[1.,0.,0.],a,6)],"not a turn about the indexing ax");
    // a turn that also slides along its axis, as a helix's placements do
    let mut slid = placements(first,o,a,6);
    slid[1] = slid[1].then(Motion::translation([0.,0.,0.5],[0.;3]).unwrap());
    refused(&[slid],"slides");
    // a placement off the pitch
    let mut off = placements(first,o,a,6);
    off[2] = first.then(turn(o,a,0.3));
    refused(&[off],"not a whole number");
}

/// A spiral band of contacts about the z axis: in each slice of height z in [0, 10], a cut
/// spanning `width` radians about the angle 0.08 z, at radii 4 to 5, turned to `count` places.
fn spiral(count: usize,width: f64) -> (Indexing,Vec<V>) {
    let sweeps = [placements(Motion::identity(),[0.;3],[0.,0.,1.],count)];
    let indexing = sector::indexing(&sweeps,10.).unwrap();
    let mut cuts = Vec::new();
    for i in 0..=40 { for j in 0..=12 { for r in [4.,4.5,5.] {
        let z = 10.*i as f64/40.;
        let phi = 0.08*z+width*(j as f64/12.-0.5);
        cuts.push([r*phi.cos(),r*phi.sin(),z]);
    } } }
    (indexing,cuts)
}

#[test]
fn the_side_runs_midway_across_a_spiral_cuts_gaps() {
    let (indexing,cuts) = spiral(12,0.2);
    let pitch = TAU/12.;
    let b = Boundary::choose(&indexing,&cuts,&[Slices::Spheres([0.,0.,-50.]),Slices::Planes]).unwrap();
    // the planes square to the axis are the slices the cut crosses squarely
    assert_eq!(b.slices,Slices::Planes);
    // the gap is the pitch less the cut, half each side, times the least radius
    let half = (pitch-0.2)/2.;
    assert!((b.clearance-4.*half).abs() < 0.02,"{}",b.clearance);
    // the side follows the spiral, midway: half a pitch from the cut's middle
    for z in [1.,5.,9.] {
        let off = (b.angle_at(z)-0.08*z).rem_euclid(pitch)-pitch/2.;
        assert!(off.abs() < 0.01,"at {z}: {off}");
    }
    // the side as a grid is clear of every cut by nearly the gap, and each cut is in one sector
    let (grid,_) = b.grid([-1.,11.],[3.5,5.5],9);
    let clear = b.clearance_of(&grid).unwrap();
    assert!(clear > 0.95*3.5*half,"{clear}");
    let sectors: std::collections::BTreeSet<usize> = cuts.iter().map(|&p| b.sector(p)).collect();
    assert_eq!(sectors.len(),1);
    // and a point in the middle of a cut is refused
    assert!(b.clearance_of(&[cuts[(20*13+6)*3]]).is_err());
}

#[test]
fn cuts_wider_than_the_pitch_leave_no_gap() {
    let (indexing,cuts) = spiral(12,0.6);
    let b = Boundary::choose(&indexing,&cuts,&[Slices::Planes]).unwrap();
    assert!(b.clearance < 0.,"{}",b.clearance);
}

#[test]
fn a_frame_reads_back_the_points_it_makes() {
    let frame = Frame::new([1.,-2.,3.],[0.,1.,1.]);
    for slices in [Slices::Planes,Slices::Spheres(add([1.,-2.,3.],[0.,2.,2.]))] {
        for (s,w,phi) in [(3_f64,0.7_f64,0.4_f64),(-2.,1.3,-2.9)] {
            let w = if matches!(slices,Slices::Planes) { w*4. } else { w };
            let s = if matches!(slices,Slices::Planes) { s } else { s.abs()+1. };
            let p = frame.point(slices,s,w,phi);
            let [a,b,c] = frame.coordinates(slices,p);
            assert!((a-s).abs() < 1e-12 && (b-w).abs() < 1e-12 && (c-phi).abs() < 1e-12,"{slices:?}: {:?}",[a,b,c]);
        }
    }
}

#[test]
fn a_blanks_section_span_is_read_and_one_reaching_its_axis_refused() {
    let frame = Frame::new([0.;3],[0.,0.,1.]);
    let annulus = |p: V| { let r = p[0].hypot(p[1]); (3. ..=5.).contains(&r) && (0. ..=2.).contains(&p[2]) };
    let ([s0,s1],[w0,w1]) = sector::section_span(frame,Slices::Planes,[[-6.,-6.,-1.],[6.,6.,3.]],&annulus).unwrap();
    assert!(s0 < 0. && s0 > -0.4 && s1 > 2. && s1 < 2.4,"{s0} {s1}");
    assert!(w0 < 3. && w0 > 2.6 && w1 > 5. && w1 < 5.4,"{w0} {w1}");
    let disc = |p: V| p[0].hypot(p[1]) <= 5. && (0. ..=2.).contains(&p[2]);
    assert!(sector::section_span(frame,Slices::Planes,[[-6.,-6.,-1.],[6.,6.,3.]],&disc).is_err());
}
