//! **The tube road's membership and its patches** (#880 T1f, `swept::tube::membership`;
//! `docs/tube-trims-by-membership.md` §2–3): each chart cut into fixed patches whose edges are shared
//! lines, and what crosses each edge read once by membership — the same crossings the present
//! road's trims make of those lines.

use gcs_core::brep::swept::rings::Ring;
use gcs_core::brep::swept::tube::membership::{Line,Patches,edge_crossings,patches};
use gcs_core::brep::swept::tube::{Chart,Depth,Tube,edges,seams,swallowtails};
use crate::common::bench::tumbling;

/// The present road's trims of a sheet (its folds, their swallowtail points, the depth's seams,
/// from no seeds) and the discs about its swallowtail points.
fn traced(tube: &Tube,chart: Chart,tol: f64,feature: f64) -> (Vec<Vec<[f64;2]>>,Vec<gcs_core::brep::swept::tube::Disc>) {
    let (pieces,discs,_) = traced_covered(tube,chart,tol,feature);
    (pieces.into_iter().map(|p| p.uv).collect(),discs)
}

/// Where the polylines `pieces` cross the patch edge `line`: each chord's crossing of the line's
/// parameter, within its span.
fn crossed(pieces: &[Vec<[f64;2]>],line: &Line) -> Vec<[f64;2]> {
    let (k,o) = (1-line.along,line.along);
    let mut out = Vec::new();
    for p in pieces {
        for w in p.windows(2) {
            let (a,b) = (w[0],w[1]);
            if (a[k]-line.at)*(b[k]-line.at) > 0. || a[k] == b[k] { continue }
            let f = (line.at-a[k])/(b[k]-a[k]);
            let s = a[o]+f*(b[o]-a[o]);
            if s >= line.span[0] && s <= line.span[1] { out.push(line.place(s)) }
        }
    }
    out
}

/// **The patches' edges are crossed where the trims cross them**, on the chart `chart`: every
/// crossing read by membership on a patch edge has one of the present road's traced pieces within
/// `near` in space, and every crossing of a patch edge by a traced piece has one read by membership
/// within `near` — or, where the zero set grazes the edge (a piece running along it, its chords
/// crossing it again and again where membership reads the one touch; a double curve beside a fold,
/// its depth changing by 4e-3 a unit across it), the other within the feature and on the zero set,
/// its depth within `near` of nought: along a grazing line a crossing's place is ill-conditioned, its
/// standing on the boundary is not. The pieces' chords keep within the tolerance of the zero set, so
/// cross a line within a few of it; membership's crossings are settled to it.
fn agree(tube: &Tube,chart: Chart,tol: f64,feature: f64,near: f64) -> (usize,f64) {
    use gcs_core::brep::swept::tube::membership::membership;
    let (pieces,discs) = traced(tube,chart,tol,feature);
    let field = Depth {tube,chart,within:tol,holes:&discs};
    let p: Patches = patches(tube,chart);
    let clock = std::time::Instant::now();
    let read = edge_crossings(&field,&p,feature);
    let x = |uv: [f64;2]| tube.at(chart,uv).unwrap().x;
    let gap = |q: [f64;2],set: &[[f64;2]]| set.iter().map(|&r| gcs_core::brep::util::space::distance(x(q),x(r))).fold(f64::INFINITY,f64::min);
    let (mut count,mut worst) = (0,0f64);
    for (line,found) in &read {
        let present = crossed(&pieces,line);
        let ours: Vec<[f64;2]> = found.iter().map(|c| c.uv).collect();
        for &q in &present {
            let d = gap(q,&ours);
            if d <= near { worst = worst.max(d); continue }
            let depth = membership(&field,q).depth().map_or(f64::INFINITY,f64::abs);
            assert!(depth <= near && d <= feature,"{chart:?}: the trim's crossing of {line:?} at {q:?} read by membership {d:e} off, its depth {depth:e}");
        }
        for c in found {
            let d = gap(c.uv,&present);
            if d <= near { continue }
            // (a grazing crossing: the nearest trim's on the zero set within the feature)
            let r = present.iter().copied().min_by(|a,b| gap(c.uv,&[*a]).total_cmp(&gap(c.uv,&[*b])));
            let depth = r.and_then(|r| membership(&field,r).depth()).map_or(f64::INFINITY,f64::abs);
            assert!(depth <= near && d <= feature,"{chart:?}: membership's crossing of {line:?} at {:?} {d:e} off any trim's, the nearest's depth {depth:e}",c.uv);
        }
        count += found.len();
    }
    eprintln!("{chart:?}: {} patches, {} edges, {count} crossings read in {:?}, the worst {worst:e} off",p.count()[0]*p.count()[1],read.len(),clock.elapsed());
    (count,worst)
}

/// **A chart's patches share their edges**: on the half turn, each sheet is cut into four of `θ` by
/// stretches of the roll each turning at most a quarter turn, each patch's four sides are edges of
/// the chart's one list, every side of a patch the side of the patch beside it — a whole turn of `θ`
/// joining its last to its first — and no edge is any patch's twice.
#[test]
fn a_charts_patches_share_their_edges() {
    let (tool,path,roll,_) = tumbling(90.);
    let ring = Ring::read(&tool).unwrap();
    let tube = Tube::new(&ring,&path,roll).unwrap();
    for chart in [Chart::Sheet(1.),Chart::Sheet(-1.),Chart::End(0)] {
        let p = patches(&tube,chart);
        let [nu,nv] = p.count();
        let lines = p.edges();
        let mut used = vec![0usize;lines.len()];
        for i in 0..nu { for j in 0..nv {
            let s = p.sides(i,j);
            let r = p.rect(i,j);
            assert_eq!(lines[s[0]],Line {along:0,at:r[1][0],span:r[0]},"{chart:?} ({i}, {j}): its side at v {}",r[1][0]);
            assert_eq!(lines[s[2]].span,r[0]);
            assert_eq!(lines[s[1]].span,r[1]);
            assert_eq!(lines[s[3]],Line {along:1,at:r[0][0],span:r[1]},"{chart:?} ({i}, {j}): its side at u {}",r[0][0]);
            for k in s { used[k] += 1 }
            let mut once = s.to_vec();
            once.dedup();
            assert_eq!(once.len(),4,"{chart:?} ({i}, {j}): a side twice");
        } }
        // (a line inside, or one a whole turn closes, two patches'; one on an open rim, one patch's)
        for (k,l) in lines.iter().enumerate() {
            let rim = (l.along == 0 && !p.periodic[1] && (l.at == p.vs[0] || l.at == p.vs[nv]))
                || (l.along == 1 && !p.periodic[0] && (l.at == p.us[0] || l.at == p.us[nu]));
            assert_eq!(used[k],if rim { 1 } else { 2 },"{chart:?}: the edge {l:?} used {} times",used[k]);
        }
        if let Chart::Sheet(_) = chart { assert_eq!(nu,4,"a sheet's quarters of θ") }
    }
}

/// **The half turn's patch edges are crossed where its trims cross them** (#880 T1f): on both sheets
/// every crossing of a patch edge by the present road's traced trims is one membership reads on the
/// edge, to four tolerances in space (the trims' chords stand within one of the zero set), and every
/// one membership reads is a trim's.
#[test]
fn the_half_turns_patch_edges_are_crossed_where_its_trims_cross_them() {
    let (tool,path,roll,tol) = tumbling(90.);
    let ring = Ring::read(&tool).unwrap();
    let tube = Tube::new(&ring,&path,roll).unwrap();
    let feature = 0.25*ring.thickness;
    let mut total = 0;
    for chart in [Chart::Sheet(1.),Chart::Sheet(-1.)] { total += agree(&tube,chart,tol,feature,4.*tol).0 }
    assert!(total > 0,"the half turn's trims cross its patch edges");
}

/// **The crossed members' patch edges are crossed where their trims cross them** (#880 T1f): on each
/// sheet of both members, the pinch's hole left to the collar, every crossing of a patch edge by the
/// present road's traced trims is one membership reads on the edge, to four tolerances in space (as
/// `agree` reads them), and every one membership reads is a trim's.
#[test]
#[cfg_attr(not(feature = "slow"),ignore = "slow tier, both crossed members' sheets traced twice over, ~20 s")]
fn the_crossed_members_patch_edges_are_crossed_where_their_trims_cross_them() {
    for observer in [1.,-1.] {
        let (tool,path,roll,tol) = crate::common::bench::crossed(observer);
        let ring = Ring::read(&tool).unwrap();
        let feature = 0.25*ring.thickness;
        let tube = Tube::pinched(&ring,&path,roll,tol,feature).unwrap();
        let mut total = 0;
        for chart in [Chart::Sheet(1.),Chart::Sheet(-1.)] { total += agree(&tube,chart,tol,feature,4.*tol).0 }
        assert!(total > 0,"the crossed member's trims cross its patch edges");
    }
}

/// **Polylines in space, their segments in a tree**: the least distance from a point to them, best
/// first (`util::bvh`).
struct Lines { segs: Vec<([f64;3],[f64;3])>,tree: gcs_core::brep::util::bvh::Bvh<3> }

impl Lines {
    fn new(lines: &[Vec<[f64;3]>]) -> Lines {
        use gcs_core::brep::util::bvh::{Bounds,Bvh};
        let segs: Vec<([f64;3],[f64;3])> = lines.iter().flat_map(|l| l.windows(2).map(|w| (w[0],w[1])).chain((l.len() == 1).then(|| (l[0],l[0])))).collect();
        let tree = Bvh::new(segs.iter().map(|&(a,b)| Bounds {lo:std::array::from_fn(|k| a[k].min(b[k])),hi:std::array::from_fn(|k| a[k].max(b[k]))}));
        Lines {segs,tree}
    }
    fn off(&self,x: [f64;3]) -> f64 {
        use gcs_core::brep::util::bvh::Bounds;
        let gap = |b: Bounds<3>| b.gap(Bounds {lo:x,hi:x});
        let best = std::cell::Cell::new(f64::INFINITY);
        self.tree.query_nearest(gap,|| best.get(),|i| { let (a,b) = self.segs[i]; best.set(best.get().min(gcs_core::brep::util::space::segment_distance(x,a,b))) });
        best.get()
    }
}

/// The present road's traced pieces of a sheet with their far covers, and its discs and folds.
fn traced_covered(tube: &Tube,chart: Chart,tol: f64,feature: f64)
    -> (Vec<gcs_core::brep::zero_set::Piece<gcs_core::brep::swept::depth::Cover>>,Vec<gcs_core::brep::swept::tube::Disc>,Vec<Vec<[f64;2]>>) {
    let Chart::Sheet(side) = chart else { panic!("a sheet") };
    let (folds,_) = edges(tube,chart,tol,feature).unwrap();
    let lines: Vec<Vec<[f64;2]>> = folds.into_iter().map(|p| p.uv).collect();
    let tails: Vec<[f64;2]> = lines.iter().flat_map(|f| swallowtails(tube,side,f)).collect();
    let (pieces,_,discs) = seams(tube,chart,&lines,&tails,&[],tol,feature).unwrap();
    (pieces,discs,lines)
}

/// **A chart's stretches are its present trims** (step 3): every place of a stretch within `near`
/// in space of the present road's traced pieces, and every place of theirs within `near` of a
/// stretch — or, where the zero set is ill-conditioned (beside the corner where an end pose touches
/// the sheet, along a graze), within the feature of one with the depth within `near` of nought all
/// the way between; and every place a stretch ends because the face across changes within the
/// feature of a place where the present pieces' face across changes, so read — away from the
/// corners, which both reach. Returns the stretches and the places compared.
fn stretched(tube: &Tube,chart: Chart,tol: f64,feature: f64,near: f64) -> (usize,usize) {
    use gcs_core::brep::swept::tube::membership::{Ending,membership,stretches};
    let begun = std::time::Instant::now();
    let (pieces,discs,folds) = traced_covered(tube,chart,tol,feature);
    let present_took = begun.elapsed();
    let field = Depth {tube,chart,within:tol,holes:&discs};
    let p: Patches = patches(tube,chart);
    let clock = std::time::Instant::now();
    let read = edge_crossings(&field,&p,feature);
    let st = stretches(&field,&folds,&p,&read,feature).unwrap();
    let took = clock.elapsed();
    let compare = std::time::Instant::now();
    let x = |uv: [f64;2]| tube.at(chart,uv).unwrap().x;
    // (each chord's image in space: the chord straight in the chart's parameters, mapped in eight —
    // a chord straight in space cuts through the tube, a chord 8.8e-3 long 3e-5 off the sheet)
    let flat = |lines: &[Vec<[f64;2]>]| -> Vec<Vec<[f64;3]>> { lines.iter().map(|l| {
        let mut out: Vec<[f64;3]> = l.first().map(|&q| x(q)).into_iter().collect();
        for w in l.windows(2) { for k in 1..=8 { let f = k as f64/8.; out.push(x([w[0][0]+(w[1][0]-w[0][0])*f,w[0][1]+(w[1][1]-w[0][1])*f])) } }
        out
    }).collect() };
    let present_uv: Vec<Vec<[f64;2]>> = pieces.iter().map(|p| p.uv.clone()).collect();
    let ours_uv: Vec<Vec<[f64;2]>> = st.iter().map(|s| s.uv.clone()).collect();
    let (present,ours) = (Lines::new(&flat(&present_uv)),Lines::new(&flat(&ours_uv)));
    // (the depth within `near` of nought all the way from `q` to the nearest place of `lines`)
    let graze = |q: [f64;2],lines: &[Vec<[f64;2]>]| -> bool {
        let Some(r) = lines.iter().flatten().copied().min_by(|a,b| gcs_core::brep::util::space::distance(x(q),x(*a)).total_cmp(&gcs_core::brep::util::space::distance(x(q),x(*b)))) else { return false };
        (0..=16).all(|k| { let f = k as f64/16.; let m = [q[0]+(r[0]-q[0])*f,q[1]+(r[1]-q[1])*f];
            membership(&field,m).depth().is_none_or(|d| d.abs() <= near) })
    };
    let held = |q: [f64;2],lines: &Lines,uv: &[Vec<[f64;2]>]| -> (bool,f64) {
        let d = lines.off(x(q));
        (d <= near || (d <= feature && graze(q,uv)),d)
    };
    // (the corners — a fold's end on the roll's rim, where the sheet and the end pose touch and a
    // trace stops at the fold short of the rim or runs on to it as a rounding's: settled in step 4 by
    // the rim's own reading — left out, within the feature of each)
    let [a,b] = tube.roll;
    let corners: Vec<[f64;3]> = folds.iter().flat_map(|f| [f[0],f[f.len()-1]]).filter(|q| (q[1]-a).abs() < 1e-6 || (q[1]-b).abs() < 1e-6).map(|q| x(q)).collect();
    let by_corner = |q: [f64;2]| corners.iter().any(|&c| gcs_core::brep::util::space::distance(x(q),c) <= feature);
    for &c in &corners {
        let reach = |lines: &Lines| lines.off(c) <= feature;
        assert_eq!(reach(&ours),reach(&present),"{chart:?}: the corner {c:?} reached by the stretches and the present trims alike");
    }
    let mut compared = 0;
    for s in &st { for &q in &s.uv {
        if by_corner(q) { continue }
        let (ok,d) = held(q,&present,&present_uv);
        assert!(ok,"{chart:?}: the stretch's place {q:?} {d:e} off the present trims");
        compared += 1;
    } }
    for l in &present_uv { for &q in l {
        if by_corner(q) { continue }
        let (ok,d) = held(q,&ours,&ours_uv);
        assert!(ok,"{chart:?}: the present trim's place {q:?} {d:e} off the stretches");
        compared += 1;
    } }
    // the face across: where it changes along the present pieces, against the stretches' ends so
    let across = |q: [f64;2],c: &gcs_core::brep::swept::depth::Cover| tube.partner(x(q),c).map(|(ch,_)| ch);
    let switches: Vec<[f64;2]> = pieces.iter().flat_map(|p| (1..p.uv.len()).filter(|&k| across(p.uv[k-1],&p.at[k-1]) != across(p.uv[k],&p.at[k]))
        .map(|k| p.uv[k]).collect::<Vec<_>>()).collect();
    let ends: Vec<[f64;2]> = st.iter().flat_map(|s| s.ends.iter().zip([s.uv[0],s.uv[s.uv.len()-1]]).filter(|(e,_)| **e == Ending::Across).map(|(_,q)| q).collect::<Vec<_>>()).collect();
    let apart = |q: [f64;2],set: &[[f64;2]]| set.iter().map(|&r| gcs_core::brep::util::space::distance(x(q),x(r))).fold(f64::INFINITY,f64::min);
    let ends: Vec<[f64;2]> = ends.into_iter().filter(|&q| !by_corner(q)).collect();
    let switches: Vec<[f64;2]> = switches.into_iter().filter(|&q| !by_corner(q)).collect();
    for &q in &ends { let d = apart(q,&switches); assert!(d <= feature,"{chart:?}: a stretch's change of the face across at {q:?} {d:e} off the present's"); }
    for &q in &switches { let d = apart(q,&ends); assert!(d <= feature,"{chart:?}: the present's change of the face across at {q:?} {d:e} off the stretches'"); }
    eprintln!("{chart:?}: {} stretches in {took:?} (the present traced in {present_took:?}, compared in {:?}), {compared} places within each other's, {} changes of the face across on each",st.len(),compare.elapsed(),switches.len());
    (st.len(),compared)
}

/// **The half turn's stretches are its present trims** (step 3, `membership::stretches`): its sheet
/// `−`'s trims lie mostly beside its corner, which both reach.
#[test]
fn the_half_turns_stretches_are_its_trims() {
    let (tool,path,roll,tol) = tumbling(90.);
    let ring = Ring::read(&tool).unwrap();
    let tube = Tube::new(&ring,&path,roll).unwrap();
    let feature = 0.25*ring.thickness;
    for chart in [Chart::Sheet(1.),Chart::Sheet(-1.)] { stretched(&tube,chart,tol,feature,4.*tol); }
}

/// The crossed member `observer` (1 forward, −1 reverse): its two sheets' stretches against the
/// present road's traces, side by side; the places compared.
fn crossed_stretched(observer: f64) -> usize {
    let (tool,path,roll,tol) = crate::common::bench::crossed(observer);
    let ring = Ring::read(&tool).unwrap();
    let feature = 0.25*ring.thickness;
    let tube = Tube::pinched(&ring,&path,roll,tol,feature).unwrap();
    std::thread::scope(|s| {
        let other = s.spawn(|| stretched(&tube,Chart::Sheet(-1.),tol,feature,4.*tol).1);
        stretched(&tube,Chart::Sheet(1.),tol,feature,4.*tol).1+other.join().unwrap()
    })
}

/// **The crossed forward member's stretches are its present trims** (step 3), as the full turn's,
/// the pinch's hole the collar's.
#[test]
fn the_crossed_forward_members_stretches_are_its_trims() {
    assert!(crossed_stretched(1.) > 0,"the forward member's trims compared");
}

/// **The crossed reverse member's stretches are its present trims** (step 3), as the forward's.
#[test]
#[cfg_attr(not(feature = "slow"),ignore = "slow tier, its sheets traced twice over, beside its near pinch")]
fn the_crossed_reverse_members_stretches_are_its_trims() {
    assert!(crossed_stretched(-1.) > 0,"the reverse member's trims compared");
}


/// **The full turn's stretches are its present trims** (step 3, `membership::stretches`): its two
/// sheets' stretches — 33, traced patch by patch in 4 s where the present road traces the sheets in 8
/// — and the present road's traced pieces within four tolerances of each other in space at some
/// 8 500 places (or within the feature along a graze, the depth within four tolerances of nought
/// between), and their 12 changes of the face across the same, the corners left out.
#[test]
fn the_full_turns_stretches_are_its_trims() {
    let (tool,path,roll,tol) = tumbling(180.);
    let ring = Ring::read(&tool).unwrap();
    let tube = Tube::new(&ring,&path,roll).unwrap();
    let feature = 0.25*ring.thickness;
    // (the two sheets side by side: each traced twice over, by the present road and by patches)
    let compared: usize = std::thread::scope(|s| {
        let other = s.spawn(|| stretched(&tube,Chart::Sheet(-1.),tol,feature,4.*tol).1);
        stretched(&tube,Chart::Sheet(1.),tol,feature,4.*tol).1+other.join().unwrap()
    });
    assert!(compared > 0,"the full turn's trims compared");
}
