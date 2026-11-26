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
    let Chart::Sheet(side) = chart else { panic!("a sheet") };
    let (folds,_) = edges(tube,chart,tol,feature).unwrap();
    let lines: Vec<Vec<[f64;2]>> = folds.into_iter().map(|p| p.uv).collect();
    let tails: Vec<[f64;2]> = lines.iter().flat_map(|f| swallowtails(tube,side,f)).collect();
    let (pieces,_,discs) = seams(tube,chart,&lines,&tails,&[],tol,feature).unwrap();
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
