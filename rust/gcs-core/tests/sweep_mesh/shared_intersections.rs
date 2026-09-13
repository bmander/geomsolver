//! Independent binary64 facet-intersection diagnostic. Shared topological
//! simplices are permitted; contact elsewhere and coplanar area overlap refuse.
use gcs_core::solid::swept_boundary::KeptMesh;
type V3 = [f64;3];
fn sub(a: V3,b: V3) -> V3 { std::array::from_fn(|k| a[k]-b[k]) }
fn dot(a: V3,b: V3) -> f64 { (0..3).map(|k| a[k]*b[k]).sum() }
fn cross(a: V3,b: V3) -> V3 { [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]] }
fn distance(a: V3,b: V3) -> f64 { dot(sub(a,b),sub(a,b)).sqrt() }
fn plane(t: [V3;3]) -> V3 { let n = cross(sub(t[1],t[0]),sub(t[2],t[0])); let l = dot(n,n).sqrt(); n.map(|x| x/l) }
fn inside(p: V3,t: [V3;3],n: V3,tol: f64) -> bool {
    (0..3).all(|i| dot(cross(sub(t[(i+1)%3],t[i]),sub(p,t[i])),n) >= -tol*distance(t[i],t[(i+1)%3]))
}
fn intersects(m: &KeptMesh,a: usize,b: usize,tol: f64) -> bool {
    let (ta,tb) = (m.triangles[a],m.triangles[b]);
    let (p,q) = (ta.map(|v| m.vertices[v as usize]),tb.map(|v| m.vertices[v as usize]));
    let (np,nq) = (plane(p),plane(q));
    let shared: Vec<_> = ta.iter().filter(|v| tb.contains(v)).map(|&v| m.vertices[v as usize]).collect();
    let allowed = |x: V3| {
        if shared.iter().any(|&p| distance(x,p) <= tol) { return true; }
        if shared.len() == 2 {
            let d = sub(shared[1],shared[0]); let t = dot(sub(x,shared[0]),d)/dot(d,d);
            let foot = std::array::from_fn(|k| shared[0][k]+t.clamp(0.,1.)*d[k]);
            return distance(x,foot) <= tol;
        }
        false
    };
    if dot(np,nq).abs() > 1.-1e-12 {
        if q.iter().any(|&q| dot(sub(q,p[0]),np).abs() > tol) { return false; }
        // Clip by three inward half-planes; area distinguishes overlapping
        // interiors from coincident edge/vertex simplices.
        let mut poly = p.to_vec();
        for k in 0..3 {
            let n = cross(nq,sub(q[(k+1)%3],q[k])); let mut next = Vec::new();
            for i in 0..poly.len() {
                let (a,b) = (poly[i],poly[(i+1)%poly.len()]);
                let (da,db) = (dot(sub(a,q[k]),n),dot(sub(b,q[k]),n));
                if da >= 0. { next.push(a); }
                if (da > 0. && db < 0.) || (da < 0. && db > 0.) {
                    let t = da/(da-db); next.push(std::array::from_fn(|k| a[k]+t*(b[k]-a[k])));
                }
            }
            poly = next;
        }
        if poly.len() >= 3 {
            let area: f64 = (1..poly.len()-1).map(|i| dot(cross(sub(poly[i],poly[0]),sub(poly[i+1],poly[0])),np).abs()/2.).sum();
            // A length tolerance must scale with the clipped perimeter,
            // not its square; roundoff at a long shared edge encloses a tiny
            // strip whose area can exceed tol^2 without resolved penetration.
            let perimeter: f64 = (0..poly.len()).map(|i| distance(poly[i],poly[(i+1)%poly.len()])).sum();
            if area > tol*perimeter { return true; }
        }
        return poly.iter().any(|&p| !allowed(p));
    }
    for (p,q,n) in [(p,q,nq),(q,p,np)] { for k in 0..3 {
        let (a,b) = (p[k],p[(k+1)%3]); let (da,db) = (dot(sub(a,q[0]),n),dot(sub(b,q[0]),n));
        if da.abs() <= tol && inside(a,q,n,tol) && !allowed(a) { return true; }
        if (da > 0. && db < 0.) || (da < 0. && db > 0.) {
            let t = da/(da-db); let x = std::array::from_fn(|k| a[k]+t*(b[k]-a[k]));
            if inside(x,q,n,tol) && !allowed(x) { return true; }
        }
    } }
    false
}
pub(super) fn unexpected(m: &KeptMesh,tol: f64) -> Vec<(usize,usize)> {
    let mut boxes: Vec<_> = m.triangles.iter().enumerate().map(|(i,t)| {
        let points = t.map(|v| m.vertices[v as usize]);
        let lo: V3 = std::array::from_fn(|k| points.iter().map(|p| p[k]).fold(f64::INFINITY,f64::min));
        let hi: V3 = std::array::from_fn(|k| points.iter().map(|p| p[k]).fold(f64::NEG_INFINITY,f64::max)); (i,lo,hi)
    }).collect();
    let extent = |k: usize| boxes.iter().map(|b| b.2[k]).fold(f64::NEG_INFINITY,f64::max)
        - boxes.iter().map(|b| b.1[k]).fold(f64::INFINITY,f64::min);
    let axis = (0..3).max_by(|&a,&b| extent(a).total_cmp(&extent(b))).unwrap();
    boxes.sort_by(|a,b| a.1[axis].total_cmp(&b.1[axis])); let mut found = Vec::new();
    for i in 0..boxes.len() {
        let (a,lo,hi) = boxes[i];
        for &(b,blo,bhi) in &boxes[i+1..] {
            if blo[axis] > hi[axis]+tol { break; }
            if (0..3).filter(|&k| k != axis).any(|k| blo[k] > hi[k]+tol || lo[k] > bhi[k]+tol) { continue; }
            if intersects(m,a,b,tol) { found.push((a,b)); }
        }
    }
    found
}
#[test]
fn intersection_diagnostic_distinguishes_shared_edges_overlap_and_crossing() {
    let mesh = |vertices,triangles| KeptMesh {vertices,triangles,sheet:vec![0,1]};
    let tilted = vec![[2.343260135303477,-0.15308465193173437,-1.2430840033808386],
        [2.343260135303477,-0.14351686118600104,-1.2375600368182393],
        [2.4377757932427357,-0.20267802021511683,-1.2717167479092697],
        [2.4377757932427357,-0.21618988822945795,-1.2795178285446043]];
    assert!(unexpected(&mesh(tilted,vec![[0,1,2],[0,2,3]]),1e-9).is_empty());
    let unshared_edge = vec![[0.,0.,0.],[1.,0.,0.],[1.,1.,0.],
        [0.,0.,0.],[1.,1.,0.],[0.,1.,0.]];
    assert_eq!(unexpected(&mesh(unshared_edge,vec![[0,1,2],[3,4,5]]),1e-10).len(),1);
    let square = vec![[0.,0.,0.],[1.,0.,0.],[1.,1.,0.],[0.,1.,0.]];
    assert!(unexpected(&mesh(square.clone(),vec![[0,1,2],[0,2,3]]),1e-10).is_empty());
    assert_eq!(unexpected(&mesh(square,vec![[0,1,2],[0,1,3]]),1e-10).len(),1);
    assert_eq!(unexpected(&mesh(vec![[0.,0.,0.],[2.,0.,0.],[0.,2.,0.],[1.,1.,-1.],[1.,1.,1.]],vec![[0,1,2],[0,3,4]]),1e-10).len(),1);
}
