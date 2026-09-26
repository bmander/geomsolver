//! Repairing the extracted boundary into a manifold, and finding the facets a protecting ball
//! holds off the surface.
use super::*;
use crate::space::{cross,norm};
use std::collections::BTreeMap;

/// The facets with a protecting ball for a vertex whose surface does not cross the facet's normal
/// line within ten times `facet_distance` of its centroid, each as the balls at its vertices.
/// Judged by the side at points along that stretch of the normal, all the same.
pub(super) fn standing_off(r: &mut Refiner,facets: &[([u32;3],u32,usize)]) -> Vec<Vec<usize>> {
    let d = 10.*r.criteria.facet_distance;
    let mut out = Vec::new();
    for &(f,_,_) in facets {
        let held: Vec<usize> = f.iter().filter_map(|v| r.ball_of.get(v).copied()).collect();
        if held.is_empty() { continue; }
        let [a,b,c] = f.map(|v| r.tri.points()[v as usize].p);
        let n = cross(sub(b,a),sub(c,a));
        let l = norm(n);
        if !(l > 0.) { continue; }
        let g: P = std::array::from_fn(|k| (a[k]+b[k]+c[k])/3.);
        // Stepping out from the facet both ways: a thin wall's surface crosses the line twice,
        // and its two ends alone would read alike on a facet lying on it.
        let at = |t: f64| [0,1,2].map(|k| g[k]+t*d*n[k]/l);
        let first = r.side(at(-1.));
        if [-0.3,-0.1,0.1,0.3,1.].iter().all(|&t| r.side(at(t)) == first) { out.push(held); }
    }
    out
}

/// Refine at the boundary's manifold faults until none is left (true), or until refining them
/// inserts nothing, every point needed having fallen in a ball (false: `blocking` says which).
pub(super) fn repair(r: &mut Refiner) -> Result<bool,String> {
    for round in 0..32 {
        let facets = r.extract();
        let faults = non_manifold(&facets);
        if faults.is_empty() { return Ok(true); }
        r.report.manifold_rounds = round+1;
        r.blocking.clear();
        let mut any = false;
        for (t,i,key) in faults {
            // Each insertion may remake the facets after it in the list.
            if !r.tri.alive(t) { continue; }
            let mut face = facet_vertices(r.tri.tet(t).v,i);
            face.sort_unstable();
            if face != key { continue; }
            if let Some((key,centre)) = r.restricted(t,i) {
                // A crossing is placed only to `bisection`: a surface centre no farther than that
                // from its facet's vertices says nothing about where the boundary is, and inserted
                // it duplicates a vertex.
                let near = key.iter().map(|&v| dist2(centre,r.tri.points()[v as usize].p)).fold(f64::INFINITY,f64::min).sqrt();
                if near <= 2.*r.criteria.bisection { r.report.too_small += 1; continue; }
                any |= r.insert(centre)?;
                r.refine()?;
            }
        }
        if !any { return Ok(false); }
    }
    Ok(non_manifold(&r.extract()).is_empty())
}

/// The facets at a manifold fault: each edge used other than twice and each vertex whose
/// triangles form more than one fan, as the largest facet of each.
pub(super) fn non_manifold(facets: &[([u32;3],u32,usize)]) -> Vec<(u32,usize,[u32;3])> {
    let mut edges: BTreeMap<(u32,u32),Vec<usize>> = BTreeMap::new();
    let mut around: BTreeMap<u32,Vec<usize>> = BTreeMap::new();
    for (k,(f,_,_)) in facets.iter().enumerate() {
        for j in 0..3 {
            let (a,b) = (f[j],f[(j+1)%3]);
            edges.entry((a.min(b),a.max(b))).or_default().push(k);
            around.entry(f[j]).or_default().push(k);
        }
    }
    let mut faults: Vec<usize> = edges.values().filter(|l| l.len() != 2).flatten().copied().collect();
    for (&v,list) in &around {
        // The triangles about v in one fan: connected through edges at v.
        let mut seen = vec![false;list.len()];
        let mut stack = vec![0];
        seen[0] = true;
        while let Some(x) = stack.pop() {
            let fx = facets[list[x]].0;
            for (y,&ky) in list.iter().enumerate() {
                if seen[y] { continue; }
                let fy = facets[ky].0;
                let shared = fx.iter().filter(|&&a| a != v && fy.contains(&a)).count();
                if shared > 0 { seen[y] = true; stack.push(y); }
            }
        }
        if seen.iter().any(|s| !s) { faults.extend(list.iter().copied()); }
    }
    faults.sort_unstable();
    faults.dedup();
    faults.into_iter().map(|k| { let mut key = facets[k].0; key.sort_unstable(); (facets[k].1,facets[k].2,key) }).collect()
}
