//! Where refinement starts: boundary points along rays from the centre, and from a lattice where
//! the rays find none.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::*;
use crate::space::lerp;

/// Seed the boundary along rays from the centre.
pub(super) fn seed(r: &mut Refiner) -> Result<(),String> {
    let (rays,steps) = (128,96);
    let golden = std::f64::consts::PI*(3.-5f64.sqrt());
    // The rays leave from beside the centre, in no particular direction: a centre on the surface
    // (a symmetric part's, on its mid-plane or a face through it) would have every ray crossing
    // there, and seed a cluster of points a bisection apart that no triangulation meshes.
    let radius = r.radius;
    let centre = [0.5773,0.3162,0.7071].map(|x| x*1e-3*radius);
    let centre: P = std::array::from_fn(|k| r.centre[k]+centre[k]);
    // Two crossings closer than a quarter facet are one seed.
    let spacing = 0.25*r.criteria.facet_size;
    let mut planted: Vec<P> = Vec::new();
    for k in 0..rays {
        let z = 1.-2.*(k as f64+0.5)/rays as f64;
        let s = (1.-z*z).sqrt();
        let d = [s*(golden*k as f64).dcos(),s*(golden*k as f64).dsin(),z];
        let point = |j: usize| lerp(centre,[centre[0]+radius*d[0],centre[1]+radius*d[1],centre[2]+radius*d[2]],j as f64/steps as f64);
        let mut last = (point(0),r.side(point(0)));
        for j in 1..=steps {
            let p = point(j);
            let sp = r.side(p);
            if sp != last.1 {
                let c = r.crossing(last.0,last.1,p);
                if !planted.iter().any(|&q| dist2(q,c) < spacing*spacing) {
                    planted.push(c);
                    if r.insert_judging(c,false)? { r.report.seeds += 1; }
                }
            }
            last = (p,sp);
        }
    }
    Ok(())
}

/// Seed from a lattice over the bounding ball, every pair of neighbours on opposite sides giving
/// a seed: `LATTICE` a side, so a body wider than the spacing is found wherever it is. Asked only
/// when the rays left refinement nothing to start from — rays from one centre miss a thin body
/// passing between them, a wire wound round an empty middle. Not always: seeds scattered over a
/// surface land by sharp edges no ball protects yet, and refinement cannot always repair what
/// they start there, so a body the rays miss while they find another stays missed until
/// features seed it.
pub(super) fn lattice(r: &mut Refiner) -> Result<(),String> {
    let (centre,radius) = (r.centre,r.radius);
    // One lattice seed a cell twice the facet size: a seed finds the surface there, and
    // refinement, not the seeding, decides how finely it is meshed — every seed stays a vertex.
    let cell = 2.*r.criteria.facet_size;
    // inserted into and asked, never iterated
    let mut taken: std::collections::HashSet<[i64;3]> = std::collections::HashSet::new();
    // The protecting balls seed the surface round every feature already, and a seed just outside
    // one leaves slivers against it: seeds keep a facet's size clear of the balls.
    let clear = r.criteria.facet_size;
    let mut plant = |r: &mut Refiner,p: P| -> Result<(),String> {
        if r.ball_within(p,clear).is_some() { return Ok(()); }
        if taken.insert(p.map(|x| (x/cell).floor() as i64)) && r.insert_judging(p,false)? { r.report.seeds += 1; }
        Ok(())
    };
    let n = LATTICE;
    let h = 2.*radius/n as f64;
    let at = |i: usize,j: usize,k: usize| [centre[0]-radius+h*i as f64,centre[1]-radius+h*j as f64,centre[2]-radius+h*k as f64];
    let mut sides = vec![0i8;(n+1)*(n+1)*(n+1)];
    let index = |i: usize,j: usize,k: usize| (i*(n+1)+j)*(n+1)+k;
    for i in 0..=n { for j in 0..=n { for k in 0..=n { sides[index(i,j,k)] = r.side(at(i,j,k)); } } }
    for i in 0..=n { for j in 0..=n { for k in 0..=n {
        let s = sides[index(i,j,k)];
        for (di,dj,dk) in [(1,0,0),(0,1,0),(0,0,1)] {
            let (a,b,c) = (i+di,j+dj,k+dk);
            if a > n || b > n || c > n || sides[index(a,b,c)] == s { continue; }
            let x = r.crossing(at(i,j,k),s,at(a,b,c));
            plant(r,x)?;
        }
    } } }
    Ok(())
}

/// Seed lattice points a side of the bounding ball's cube.
const LATTICE: usize = 28;
