//! A B-rep meshed within a stated sag: every edge sampled once, in space, so the two faces that
//! share it share its points to the bit; every face triangulated in its own parameters (scaled
//! to lengths, mirrored where the face is reversed, so its triangles turn with its material
//! normal) by clipping ears from its loops with the holes bridged in, flipped toward the
//! constrained Delaunay triangulation, and refined where a triangle's measured sag — the surface's
//! distance from it at its middle and at the middles of its inner sides — exceeds the bar.
use super::geom::{Uv,V};
use super::topo::{Brep,EdgeCurve,Face};
use crate::delaunay::expansion::Expansion;
use crate::space::{add,cross,distance,dot,norm,scale,sub};

/// Triangles over shared points, each turning with the solid's outward normal.
#[derive(Clone,Debug,Default)]
pub struct Mesh { pub pts: Vec<V>,pub tris: Vec<[u32;3]>,
    /// The greatest sag measured on any face's triangles.
    pub sag: f64 }

impl Mesh {
    /// Nine coordinates a triangle, for STL.
    pub fn triangles(&self) -> Vec<f64> {
        self.tris.iter().flat_map(|t| t.iter().flat_map(|&i| self.pts[i as usize])).collect()
    }
}

/// `(b − a) × (c − a)`'s sign, exactly.
fn orient(a: [f64;2],b: [f64;2],c: [f64;2]) -> i8 {
    let det = (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]);
    let bound = 1e-14*((b[0]-a[0]).abs()*(c[1]-a[1]).abs()+(b[1]-a[1]).abs()*(c[0]-a[0]).abs());
    if det.abs() > bound { return det.signum() as i8 }
    let l = Expansion::diff(b[0],a[0]).mul(&Expansion::diff(c[1],a[1]));
    let r = Expansion::diff(b[1],a[1]).mul(&Expansion::diff(c[0],a[0]));
    l.sub(&r).sign()
}

/// Whether `d` is strictly inside the circle through the counter-clockwise `a`, `b`, `c`, exactly.
fn in_circle_exact(a: [f64;2],b: [f64;2],c: [f64;2],d: [f64;2]) -> bool {
    let e = |x: f64,y: f64| Expansion::diff(x,y);
    let (ax,ay,bx,by,cx,cy) = (e(a[0],d[0]),e(a[1],d[1]),e(b[0],d[0]),e(b[1],d[1]),e(c[0],d[0]),e(c[1],d[1]));
    let sq = |x: &Expansion,y: &Expansion| x.mul(x).add(&y.mul(y));
    let cr = |x1: &Expansion,y1: &Expansion,x2: &Expansion,y2: &Expansion| x1.mul(y2).sub(&x2.mul(y1));
    let det = sq(&ax,&ay).mul(&cr(&bx,&by,&cx,&cy)).sub(&sq(&bx,&by).mul(&cr(&ax,&ay,&cx,&cy))).add(&sq(&cx,&cy).mul(&cr(&ax,&ay,&bx,&by)));
    det.sign() > 0
}

/// Whether `d` is strictly inside the circle through the counter-clockwise `a`, `b`, `c`: a filtered
/// evaluation, exact where it is too near zero to decide.
fn in_circle(a: [f64;2],b: [f64;2],c: [f64;2],d: [f64;2]) -> bool {
    let (ax,ay,bx,by,cx,cy) = (a[0]-d[0],a[1]-d[1],b[0]-d[0],b[1]-d[1],c[0]-d[0],c[1]-d[1]);
    let det = (ax*ax+ay*ay)*(bx*cy-cx*by)-(bx*bx+by*by)*(ax*cy-cx*ay)+(cx*cx+cy*cy)*(ax*by-bx*ay);
    let scale = (ax*ax+ay*ay)*(bx.abs()*cy.abs()+cx.abs()*by.abs())+(bx*bx+by*by)*(ax.abs()*cy.abs()+cx.abs()*ay.abs())
        +(cx*cx+cy*cy)*(ax.abs()*by.abs()+bx.abs()*ay.abs());
    if det.abs() > 1e-12*scale { det > 0. } else { in_circle_exact(a,b,c,d) }
}

/// An edge's parameters, refined until each chord's middle is within `bar` of the curve and no
/// chord turns more than `angular` (a closed edge in at least three pieces).
fn edge_params(b: &Brep,e: usize,bar: f64,angular: f64) -> Vec<f64> {
    let edge = &b.edges[e];
    let EdgeCurve::Curve(c) = &edge.curve else { return vec![edge.t[0],edge.t[1]] };
    let mut out = vec![edge.t[0]];
    // the first pieces: a closed edge in three; a B-spline's every knot span in one more piece than
    // its degree, since a midpoint test is blind to an S-bend whose chord's middle is on the curve
    let mut cuts = vec![edge.t[0]];
    let near = 1e-6*(edge.t[1]-edge.t[0]);
    if let super::geom::Curve::BSpline(s) = c {
        cuts.extend(s.breaks(edge.t).into_iter().filter(|&k| k-edge.t[0] > near && edge.t[1]-k > near));
    }
    cuts.push(edge.t[1]);
    let each = match c { super::geom::Curve::BSpline(s) => s.degree+1,_ if edge.closed() => 3,_ => 1 };
    let mut stack: Vec<(f64,f64,u32)> = cuts.windows(2).flat_map(|w| (0..each).map(move |k| {
        let h = (w[1]-w[0])/each as f64;
        (w[0]+h*k as f64,w[0]+h*(k+1) as f64,0)
    })).collect();
    stack.reverse();
    while let Some((a,z,depth)) = stack.pop() {
        let (pa,pz) = (c.point(a),c.point(z));
        let m = (a+z)/2.;
        let pm = c.point(m);
        let chord = sub(pz,pa);
        let l2 = dot(chord,chord);
        let s = if l2 > 0. { (dot(sub(pm,pa),chord)/l2).clamp(0.,1.) } else { 0. };
        let off = distance(pm,add(pa,scale(chord,s)));
        let (ta,tz) = (c.tangent(a),c.tangent(z));
        let turn = (dot(ta,tz)/(norm(ta)*norm(tz)).max(1e-300)).clamp(-1.,1.).acos();
        // and the quarter points, which a stretch centred on an inflection cannot hide
        let strays = || [0.25,0.75].iter().any(|&f| {
            let q = c.point(a+(z-a)*f);
            let s = if l2 > 0. { (dot(sub(q,pa),chord)/l2).clamp(0.,1.) } else { 0. };
            distance(q,add(pa,scale(chord,s))) > bar
        });
        if depth < 40 && (off > bar || turn > angular || strays()) { stack.push((m,z,depth+1)); stack.push((a,m,depth+1)); }
        else { out.push(z); }
    }
    out
}

/// A triangulation of points in the plane with some sides held (a face's boundary).
struct Tri2 { pts: Vec<[f64;2]>,tris: Vec<[u32;3]>,held: std::collections::BTreeSet<(u32,u32)> }

impl Tri2 {
    fn held(&self,a: u32,b: u32) -> bool { self.held.contains(&(a.min(b),a.max(b))) }
    /// Every triangle opposite each side: `(a, b)` → the triangle holding the side `a → b`.
    fn sides(&self) -> std::collections::BTreeMap<(u32,u32),usize> {
        let mut m = std::collections::BTreeMap::new();
        for (i,t) in self.tris.iter().enumerate() { for k in 0..3 { m.insert((t[k],t[(k+1)%3]),i); } }
        m
    }
    /// Lawson's flips until no side but a held one fails the empty-circle test: a queue of sides
    /// to look at, each flip queueing the four around it.
    fn delaunay(&mut self) {
        let mut side = self.sides();
        let mut queue: Vec<(u32,u32)> = side.keys().copied().filter(|&(a,b)| a < b).collect();
        let mut flips = 0usize;
        while let Some((a,b)) = queue.pop() {
            if self.held(a,b) { continue }
            let (Some(&i),Some(&j)) = (side.get(&(a,b)),side.get(&(b,a))) else { continue };
            if i == j { continue }
            let (ti,tj) = (self.tris[i],self.tris[j]);
            let c = *ti.iter().find(|&&v| v != a && v != b).unwrap();
            let d = *tj.iter().find(|&&v| v != a && v != b).unwrap();
            let p = |v: u32| self.pts[v as usize];
            // (a, b, c) counter-clockwise: flip to (c, d) where d is in its circle and the
            // quadrilateral is convex
            if !in_circle(p(a),p(b),p(c),p(d)) { continue }
            if orient(p(c),p(a),p(d)) <= 0 || orient(p(d),p(b),p(c)) <= 0 { continue }
            self.tris[i] = [c,a,d];
            self.tris[j] = [d,b,c];
            side.remove(&(a,b)); side.remove(&(b,a));
            for t in [i,j] { let tr = self.tris[t]; for q in 0..3 { side.insert((tr[q],tr[(q+1)%3]),t); } }
            for (x,y) in [(a,c),(c,b),(b,d),(d,a)] { queue.push((x.min(y),x.max(y))); }
            flips += 1;
            if flips > 50_000_000 { break }
        }
    }
    /// A new point inside triangle `t`, the triangle split three ways.
    fn split(&mut self,t: usize,p: [f64;2]) -> u32 {
        self.pts.push(p);
        let n = (self.pts.len()-1) as u32;
        let [a,b,c] = self.tris[t];
        self.tris[t] = [a,b,n];
        self.tris.push([b,c,n]);
        self.tris.push([c,a,n]);
        n
    }
}

/// The constrained Delaunay triangulation of a face's loops: every corner inserted (Bowyer–Watson,
/// in a triangle about them all), every side of every loop recovered by flipping the sides that
/// cross it, and only the triangles inside the loops kept (the outer loop counter-clockwise, its
/// holes clockwise: a winding number of one).
fn cdt(pts: &[[f64;2]],loops: &[Vec<usize>]) -> Result<Vec<[u32;3]>,String> {
    let n = pts.len();
    let (mut lo,mut hi) = ([f64::INFINITY;2],[f64::NEG_INFINITY;2]);
    for p in pts { for k in 0..2 { lo[k] = lo[k].min(p[k]); hi[k] = hi[k].max(p[k]); } }
    let size = (hi[0]-lo[0]).max(hi[1]-lo[1]).max(1e-300);
    let mid = [(lo[0]+hi[0])/2.,(lo[1]+hi[1])/2.];
    let mut all = pts.to_vec();
    all.push([mid[0]-20.*size,mid[1]-20.*size]);
    all.push([mid[0]+20.*size,mid[1]-20.*size]);
    all.push([mid[0],mid[1]+20.*size]);
    let mut tris: Vec<[u32;3]> = vec![[n as u32,n as u32+1,n as u32+2]];
    for i in 0..n {
        let p = all[i];
        let bad: Vec<usize> = (0..tris.len()).filter(|&t| { let [a,b,c] = tris[t]; in_circle(all[a as usize],all[b as usize],all[c as usize],p) }).collect();
        if bad.is_empty() {
            // on a side or a corner of the triangle about it: split the triangle it lies in
            let Some(t) = (0..tris.len()).find(|&t| { let [a,b,c] = tris[t].map(|v| all[v as usize]);
                orient(a,b,p) >= 0 && orient(b,c,p) >= 0 && orient(c,a,p) >= 0 }) else { return Err("a corner lies outside its face's triangulation".into()) };
            let [a,b,c] = tris[t];
            tris[t] = [a,b,i as u32];
            tris.push([b,c,i as u32]);
            tris.push([c,a,i as u32]);
            continue
        }
        // the cavity's boundary: sides of one bad triangle only
        let mut count: std::collections::BTreeMap<(u32,u32),(u32,u32,usize)> = std::collections::BTreeMap::new();
        for &t in &bad { for k in 0..3 {
            let (a,b) = (tris[t][k],tris[t][(k+1)%3]);
            let e = count.entry((a.min(b),a.max(b))).or_insert((a,b,0));
            e.2 += 1;
        } }
        let mut keep: Vec<[u32;3]> = tris.iter().enumerate().filter(|(t,_)| !bad.contains(t)).map(|(_,t)| *t).collect();
        for &(a,b,k) in count.values() {
            if k == 1 && orient(all[a as usize],all[b as usize],p) > 0 { keep.push([a,b,i as u32]); }
        }
        tris = keep;
    }
    // every loop side recovered: flip what crosses it until it is a side of the triangulation
    let side_of = |tris: &[[u32;3]],a: u32,b: u32| tris.iter().any(|t| (0..3).any(|k| (t[k] == a && t[(k+1)%3] == b) || (t[k] == b && t[(k+1)%3] == a)));
    for l in loops {
        for k in 0..l.len() {
            let (a,b) = (l[k] as u32,l[(k+1)%l.len()] as u32);
            let mut guard = 0;
            while !side_of(&tris,a,b) {
                // a side crossing (a, b), whose quadrilateral is convex: flip it
                let (pa,pb) = (all[a as usize],all[b as usize]);
                let mut flipped = false;
                'find: for i in 0..tris.len() {
                    for q in 0..3 {
                        let (u,v) = (tris[i][q],tris[i][(q+1)%3]);
                        if u == a || u == b || v == a || v == b { continue }
                        let (pu,pv) = (all[u as usize],all[v as usize]);
                        if !(orient(pa,pb,pu)*orient(pa,pb,pv) < 0 && orient(pu,pv,pa)*orient(pu,pv,pb) < 0) { continue }
                        let Some(j) = (0..tris.len()).find(|&j| j != i && (0..3).any(|r| tris[j][r] == v && tris[j][(r+1)%3] == u)) else { continue };
                        let w = *tris[i].iter().find(|&&x| x != u && x != v).unwrap();
                        let x = *tris[j].iter().find(|&&y| y != u && y != v).unwrap();
                        // (u, v, w) and (v, u, x): flip to (w, x) where the quadrilateral is convex
                        if orient(all[w as usize],all[u as usize],all[x as usize]) <= 0 || orient(all[x as usize],all[v as usize],all[w as usize]) <= 0 { continue }
                        tris[i] = [w,u,x];
                        tris[j] = [x,v,w];
                        flipped = true;
                        break 'find
                    }
                }
                guard += 1;
                if !flipped || guard > 100_000 { return Err("a face's boundary could not be recovered in its triangulation".into()) }
            }
        }
    }
    // the triangles inside: winding number one about their centroids
    let inside = |c: [f64;2]| -> bool {
        let mut w = 0;
        for l in loops { for k in 0..l.len() {
            let (a,b) = (pts[l[k]],pts[l[(k+1)%l.len()]]);
            if a[1] <= c[1] { if b[1] > c[1] && orient(a,b,c) > 0 { w += 1; } }
            else if b[1] <= c[1] && orient(a,b,c) < 0 { w -= 1; }
        } }
        w == 1
    };
    Ok(tris.into_iter().filter(|t| t.iter().all(|&v| (v as usize) < n) && {
        let [a,b,c] = t.map(|v| all[v as usize]);
        inside([(a[0]+b[0]+c[0])/3.,(a[1]+b[1]+c[1])/3.])
    }).collect())
}

/// The distance from the surface at `uv` to the plane of the triangle `a`, `b`, `c` (space).
fn off(f: &Face,uv: Uv,a: V,b: V,c: V) -> f64 {
    let n = cross(sub(b,a),sub(c,a));
    let l = norm(n);
    if !(l > 0.) { return 0. }
    dot(sub(f.surface.point(uv),a),n).abs()/l
}

/// The mesh of the whole boundary within `bar` (a length) and `angular` (radians an edge's chords
/// may turn), and the greatest sag measured.
pub fn mesh(b: &Brep,bar: f64,angular: f64) -> Result<Mesh,String> {
    let mut m = Mesh::default();
    let vid: Vec<u32> = b.vertices.iter().map(|v| { m.pts.push(v.p); (m.pts.len()-1) as u32 }).collect();
    // every edge's samples, once: parameters and point ids
    let samples: Vec<(Vec<f64>,Vec<u32>)> = (0..b.edges.len()).map(|e| {
        let ts = edge_params(b,e,bar,angular);
        let edge = &b.edges[e];
        let ids = ts.iter().enumerate().map(|(k,&t)| {
            if k == 0 { vid[edge.v[0] as usize] }
            else if k == ts.len()-1 { vid[edge.v[1] as usize] }
            else if matches!(edge.curve,EdgeCurve::Degenerate) { vid[edge.v[0] as usize] }
            else { m.pts.push(edge.point(t,&b.vertices)); (m.pts.len()-1) as u32 }
        }).collect();
        (ts,ids)
    }).collect();
    for (fi,f) in b.faces.iter().enumerate() {
        // the loops' corners: their places in the face's scaled, oriented parameters and their ids
        let mut pts: Vec<[f64;2]> = Vec::new();
        let mut uvs: Vec<Uv> = Vec::new();
        let mut ids: Vec<u32> = Vec::new();
        let mut loops: Vec<Vec<usize>> = Vec::new();
        let mut raw: Vec<Vec<(Uv,u32)>> = Vec::new();
        for l in &f.loops {
            let mut run = Vec::new();
            for c in l {
                let e = &b.edges[c.edge as usize];
                let (ts,is) = &samples[c.edge as usize];
                let mut these: Vec<(Uv,u32)> = ts.iter().zip(is).map(|(&t,&i)| (c.pcurve.at(t,e,&f.surface,&b.vertices),i)).collect();
                if c.reversed { these.reverse(); }
                run.extend_from_slice(&these[..these.len()-1]);
            }
            raw.push(run);
        }
        let (mut lo,mut hi) = ([f64::INFINITY;2],[f64::NEG_INFINITY;2]);
        for p in raw.iter().flatten() { for k in 0..2 { lo[k] = lo[k].min(p.0[k]); hi[k] = hi[k].max(p.0[k]); } }
        let mid = [(lo[0]+hi[0])/2.,(lo[1]+hi[1])/2.];
        let (_,su,sv) = f.surface.d1(mid);
        let (ku,kv) = (norm(su).max(1e-9*(1.+b.size())),norm(sv).max(1e-9*(1.+b.size())));
        let place = |uv: Uv| [if f.reversed { -uv[0]*ku } else { uv[0]*ku },uv[1]*kv];
        for run in &raw {
            let mut l = Vec::new();
            for &(uv,i) in run {
                pts.push(place(uv));
                uvs.push(uv);
                ids.push(i);
                l.push(pts.len()-1);
            }
            loops.push(l);
        }
        let tris = cdt(&pts,&loops).map_err(|e| format!("face {fi} ({}): {e}",f.surface.kind()))?;
        let mut held = std::collections::BTreeSet::new();
        for l in &loops { for k in 0..l.len() { let (a,b) = (l[k] as u32,l[(k+1)%l.len()] as u32); held.insert((a.min(b),a.max(b))); } }
        let mut t2 = Tri2 {pts,tris,held};
        t2.delaunay();
        // refine where the surface stands off a triangle by more than the bar
        let unplace = |p: [f64;2]| [if f.reversed { -p[0]/ku } else { p[0]/ku },p[1]/kv];
        let curved = !matches!(f.surface,super::geom::Surface::Plane(_));
        let mut worst: f64 = 0.;
        if curved {
            for _round in 0..40 {
                let mut added = 0;
                let n = t2.tris.len();
                for ti in 0..n {
                    let t = t2.tris[ti];
                    let corner = |k: usize| -> (Uv,V) {
                        let v = t[k] as usize;
                        let uv = if v < uvs.len() { uvs[v] } else { unplace(t2.pts[v]) };
                        (uv,f.surface.point(uv))
                    };
                    let (a,b3,c) = (corner(0),corner(1),corner(2));
                    let centre = [(a.0[0]+b3.0[0]+c.0[0])/3.,(a.0[1]+b3.0[1]+c.0[1])/3.];
                    let mut s = off(f,centre,a.1,b3.1,c.1);
                    for (p,q,k) in [(&a,&b3,(0,1)),(&b3,&c,(1,2)),(&c,&a,(2,0))] {
                        if t2.held(t[k.0],t[k.1]) { continue }
                        s = s.max(off(f,[(p.0[0]+q.0[0])/2.,(p.0[1]+q.0[1])/2.],a.1,b3.1,c.1));
                    }
                    worst = worst.max(s);
                    if s > bar && t2.pts.len() < 400_000 {
                        let p = [(t2.pts[t[0] as usize][0]+t2.pts[t[1] as usize][0]+t2.pts[t[2] as usize][0])/3.,
                            (t2.pts[t[0] as usize][1]+t2.pts[t[1] as usize][1]+t2.pts[t[2] as usize][1])/3.];
                        t2.split(ti,p);
                        added += 1;
                    }
                }
                if added == 0 { break }
                worst = 0.;
                t2.delaunay();
            }
        }
        m.sag = m.sag.max(worst);
        // the new points' ids, and the triangles
        let first_new = uvs.len();
        let new_ids: Vec<u32> = (first_new..t2.pts.len()).map(|v| { m.pts.push(f.surface.point(unplace(t2.pts[v]))); (m.pts.len()-1) as u32 }).collect();
        let id = |v: u32| if (v as usize) < first_new { ids[v as usize] } else { new_ids[v as usize-first_new] };
        for t in &t2.tris {
            let tri = [id(t[0]),id(t[1]),id(t[2])];
            // a triangle with two corners at one point (a pole) has no area
            if tri[0] == tri[1] || tri[1] == tri[2] || tri[2] == tri[0] { continue }
            m.tris.push(tri);
        }
    }
    Ok(m)
}
