//! Coplanar coverage made one: every plane's triangles, from whichever
//! sheets and caps put them there, unioned in that plane and retriangulated
//! without overlap. A face grazing the sweep is generated many times over (a
//! turned cylinder's top by each half of its rim, by the whole face, and by
//! both caps), each source triangulating its own part of one planar region
//! differently, so no rule dropping whole triangles can leave a region
//! covered once with a boundary the neighbouring pieces can join. The union
//! is exact on the polygons: each triangle is clipped by the fragments
//! already placed and only what they do not cover is kept, every fragment a
//! convex polygon fanned into triangles. Vertices of the input keep their
//! identities; the T-junctions the clipping leaves (a fragment's corner on
//! another's edge) are the stitch's tolerant split to resolve.
use super::trim::KeptMesh;

type V3 = [f64;3];
pub(super) type P2 = (Option<u32>,f64,f64);

fn sub(a: V3,b: V3) -> V3 { [a[0]-b[0],a[1]-b[1],a[2]-b[2]] }
fn dot(a: V3,b: V3) -> f64 { a[0]*b[0]+a[1]*b[1]+a[2]*b[2] }
fn cross(a: V3,b: V3) -> V3 { [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]] }
fn norm(a: V3) -> f64 { dot(a,a).sqrt() }
fn add(a: V3,b: V3) -> V3 { [a[0]+b[0],a[1]+b[1],a[2]+b[2]] }

pub(super) fn area2(p: &[P2]) -> f64 {
    let n = p.len();
    (0..n).map(|i| { let (a,b) = (p[i],p[(i+1)%n]); a.1*b.2-b.1*a.2 }).sum::<f64>()
}

/// A convex polygon split by a directed line: the parts to its left and to
/// its right, points within `eps` of the line belonging to both.
pub(super) fn cut(p: &[P2],d: &[f64],eps: f64) -> (Vec<P2>,Vec<P2>) {
    let n = p.len();
    let (mut left,mut right) = (Vec::new(),Vec::new());
    for i in 0..n {
        let (a,b) = (p[i],p[(i+1)%n]); let (da,db) = (d[i],d[(i+1)%n]);
        if da >= -eps { left.push(a); }
        if da <= eps { right.push(a); }
        if (da > eps && db < -eps) || (da < -eps && db > eps) {
            let t = da/(da-db);
            let x: P2 = (None,a.1+t*(b.1-a.1),a.2+t*(b.2-a.2));
            left.push(x); right.push(x);
        }
    }
    (left,right)
}

/// Whether two convex polygons overlap in area (touching is not overlapping).
pub(super) fn overlap(p: &[P2],q: &[P2],eps: f64) -> bool {
    let axes = |poly: &[P2]| (0..poly.len()).map(|i| { let (a,b) = (poly[i],poly[(i+1)%poly.len()]); (-(b.2-a.2),b.1-a.1) }).collect::<Vec<_>>();
    for (ax,ay) in axes(p).into_iter().chain(axes(q)) {
        let len = (ax*ax+ay*ay).sqrt();
        if !(len > 0.) { continue; }
        let project = |poly: &[P2]| poly.iter().map(|v| (v.1*ax+v.2*ay)/len).fold((f64::INFINITY,f64::NEG_INFINITY),|(lo,hi),x| (lo.min(x),hi.max(x)));
        let (plo,phi) = project(p); let (qlo,qhi) = project(q);
        if phi <= qlo+eps || qhi <= plo+eps { return false; }
    }
    true
}

/// The parts of `p` not covered by the convex polygon `f`, grown outward by
/// `slack` (so tiles of one surface with cracks between them cover it).
pub(super) fn clip(p: Vec<P2>,f: &[P2],eps: f64,slack: f64,area_eps: f64,out: &mut Vec<Vec<P2>>) {
    let mut rest = p;
    for i in 0..f.len() {
        let (a,b) = (f[i],f[(i+1)%f.len()]);
        let (ex,ey) = (b.1-a.1,b.2-a.2);
        let len = (ex*ex+ey*ey).sqrt();
        if !(len > 0.) { continue; }
        // the inward normal of a counter-clockwise polygon's edge
        let (nx,ny) = (-ey/len,ex/len);
        let d: Vec<f64> = rest.iter().map(|v| (v.1-a.1)*nx+(v.2-a.2)*ny+slack).collect();
        let (inside,outside) = cut(&rest,&d,eps);
        if outside.len() >= 3 && area2(&outside) > area_eps { out.push(outside); }
        if inside.len() < 3 || area2(&inside) <= area_eps { return; }
        rest = inside;
    }
    // what is left lies inside every edge of `f`: covered
}

/// Every plane's triangles unioned and retriangulated; the rest untouched.
/// Two triangles are coplanar when their normals agree to a millionth and
/// each one's vertices lie within `tolerance` of the other's plane. Returns
/// the mesh and how many triangles the union replaced.
pub fn planar_union(mesh: &KeptMesh,tolerance: f64) -> (KeptMesh,usize) {
    // union first, trim second: a foot is read off the triangles standing
    // on the plane, and a folded ribbon's own triangles say nothing
    // reliable about which way they face until the union has flattened them
    let (unioned,replaced) = pass(mesh,tolerance,false);
    (pass(&unioned,tolerance,true).0,replaced)
}

/// One pass over the planes: the union of each group's triangles, and with
/// `trim` their trimming at the feet of the sheets standing on the plane.
fn pass(mesh: &KeptMesh,tolerance: f64,trim: bool) -> (KeptMesh,usize) {
    let normal = |t: &[u32;3]| -> Option<(V3,f64)> {
        let [a,b,c] = t.map(|v| mesh.vertices[v as usize]);
        let n = cross(sub(b,a),sub(c,a)); let l = norm(n);
        (l > 0.).then(|| { let n = n.map(|x| x/l); (n,dot(n,a)) })
    };
    // groups by unsigned plane; the side a group faces is its members'
    // area-weighted majority, so one sliver wound by a garbage normal
    // cannot turn a plane over
    let mut groups: Vec<(V3,f64,Vec<usize>,V3)> = Vec::new();
    let mut loose: Vec<usize> = Vec::new();
    for (i,t) in mesh.triangles.iter().enumerate() {
        let Some((n,d)) = normal(t) else { loose.push(i); continue };
        let corners = t.map(|v| mesh.vertices[v as usize]);
        let weighted = { let [a,b,c] = corners; cross(sub(b,a),sub(c,a)) };
        let g = groups.iter().position(|(gn,gd,_,_)| dot(n,*gn).abs() >= 1.-1e-6 && corners.iter().all(|p| (dot(*gn,*p)-gd).abs() <= tolerance) && (d*dot(n,*gn).signum()-gd).abs() <= tolerance);
        match g { Some(g) => { groups[g].2.push(i); groups[g].3 = add(groups[g].3,weighted); },None => groups.push((n,d,vec![i],weighted)) }
    }
    // the side each triangle faces: its group's majority where it has one
    // (a folded ribbon's own winding is not to be trusted), else its own
    let mut facing_of: Vec<Option<V3>> = vec![None;mesh.triangles.len()];
    for (_,_,members,facing) in &groups {
        let l = norm(*facing);
        if l > 0. { for &i in members { facing_of[i] = Some(facing.map(|x| x/l)); } }
    }
    let mut out = KeptMesh {vertices:mesh.vertices.clone(),triangles:Vec::new(),sheet:Vec::new()};
    let mut replaced = 0;
    for &i in &loose { out.triangles.push(mesh.triangles[i]); out.sheet.push(mesh.sheet[i]); }
    for (_,_,members,facing) in &groups {
        if members.len() < 2 { for &i in members { out.triangles.push(mesh.triangles[i]); out.sheet.push(mesh.sheet[i]); } continue; }
        let n = &facing.map(|x| x/norm(*facing));
        // a frame in the plane
        let least = (0..3).min_by(|&a,&b| n[a].abs().total_cmp(&n[b].abs())).unwrap();
        let mut axis = [0.;3]; axis[least] = 1.;
        let u = { let c = cross(*n,axis); let l = norm(c); c.map(|x| x/l) };
        let v = cross(*n,u);
        let origin = mesh.vertices[mesh.triangles[members[0]][0] as usize];
        let to2 = |idx: u32| -> P2 { let p = sub(mesh.vertices[idx as usize],origin); (Some(idx),dot(p,u),dot(p,v)) };
        let mut scale = 0_f64;
        let polys: Vec<(Vec<P2>,u32)> = members.iter().filter_map(|&i| {
            let mut pts: Vec<P2> = mesh.triangles[i].iter().map(|&x| to2(x)).collect();
            for p in &pts { scale = scale.max(p.1.abs()).max(p.2.abs()); }
            if area2(&pts) < 0. { pts.reverse(); }
            (area2(&pts) > 0.).then_some((pts,mesh.sheet[i]))
        }).collect();
        let eps = 1e-9*scale.max(f64::MIN_POSITIVE);
        let area_eps = 1e-14*scale*scale;
        let bbox = |p: &[P2]| p.iter().fold((f64::INFINITY,f64::INFINITY,f64::NEG_INFINITY,f64::NEG_INFINITY),|(x0,y0,x1,y1),q| (x0.min(q.1),y0.min(q.2),x1.max(q.1),y1.max(q.2)));
        let mut fragments: Vec<(Vec<P2>,u32,(f64,f64,f64,f64))> = Vec::new();
        for (q,sheet) in polys {
            let mut pieces = vec![q];
            for (f,_,fb) in &fragments {
                let mut next = Vec::new();
                for p in pieces {
                    let pb = bbox(&p);
                    if pb.2 <= fb.0+eps || fb.2 <= pb.0+eps || pb.3 <= fb.1+eps || fb.3 <= pb.1+eps || !overlap(&p,f,eps) { next.push(p); continue; }
                    clip(p,f,eps,0.,area_eps,&mut next);
                }
                pieces = next;
                if pieces.is_empty() { break; }
            }
            for p in pieces { let b = bbox(&p); fragments.push((p,sheet,b)); }
        }
        replaced += members.len();
        // The plane's region ends where another sheet stands on it: every
        // edge of a triangle outside the group with both ends in the plane
        // is that sheet's foot, and the region lies to one side of it within
        // the edge's own span. Which side is read off the region itself: the
        // side holding the greater part of its area there is the region,
        // and what lies on the other side is an overhang (a ribbon's chord
        // dipping past a wall's row) and goes. No crease of a solid has a
        // sheet standing in the middle of a plane's region, so a foot with
        // comparable area on both sides is left alone.
        let in_group: std::collections::BTreeSet<usize> = members.iter().copied().collect();
        let mut feet: Vec<(P2,P2)> = Vec::new();
        for (i,t) in mesh.triangles.iter().enumerate() {
            if !trim || in_group.contains(&i) { continue; }
            let Some(m) = facing_of[i].or_else(|| normal(t).map(|(m,_)| m)) else { continue };
            if dot(m,*n).abs() > 0.995 { continue; }
            let corners = t.map(|v| mesh.vertices[v as usize]);
            for k in 0..3 {
                let (a,b) = (corners[k],corners[(k+1)%3]);
                if (dot(*n,sub(a,origin))).abs() > tolerance || (dot(*n,sub(b,origin))).abs() > tolerance { continue; }
                feet.push((to2(t[k]),to2(t[(k+1)%3])));
            }
        }
        // the part of a polygon beyond the line through a, b (on the side
        // of o) and alongside the edge, as pieces
        let beside = |poly: &Vec<P2>,a: P2,b: P2,o: (f64,f64),eps: f64| -> Vec<Vec<P2>> {
            let (ex,ey) = (b.1-a.1,b.2-a.2);
            let len = (ex*ex+ey*ey).sqrt();
            let (tx,ty) = (ex/len,ey/len);
            let d: Vec<f64> = poly.iter().map(|p| (p.1-a.1)*o.0+(p.2-a.2)*o.1).collect();
            let (beyond,_) = cut(poly,&d,eps);
            if beyond.len() < 3 { return Vec::new(); }
            let d: Vec<f64> = beyond.iter().map(|p| (p.1-a.1)*tx+(p.2-a.2)*ty).collect();
            let (past_a,_) = cut(&beyond,&d,eps);
            if past_a.len() < 3 { return Vec::new(); }
            let d: Vec<f64> = past_a.iter().map(|p| (p.1-b.1)*tx+(p.2-b.2)*ty).collect();
            let (_,alongside) = cut(&past_a,&d,eps);
            if alongside.len() < 3 { Vec::new() } else { vec![alongside] }
        };
        for (a,b) in &feet {
            let (ex,ey) = (b.1-a.1,b.2-a.2);
            let len = (ex*ex+ey*ey).sqrt();
            if !(len > 0.) { continue; }
            let o = (-ey/len,ex/len);
            let side = |o: (f64,f64)| -> f64 { fragments.iter().map(|(p,_,_)| beside(p,*a,*b,o,eps).iter().map(|q| area2(q)).sum::<f64>()).sum::<f64>() };
            let (left,right) = (side(o),side((-o.0,-o.1)));
            let (small,large) = (left.min(right),left.max(right));
            if !(small > area_eps) || small > 0.25*large { continue; }
            let away = if left < right { o } else { (-o.0,-o.1) };
            let (ex,ey) = (b.1-a.1,b.2-a.2);
            let (tx,ty) = (ex/len,ey/len);
            let mut trimmed = Vec::new();
            for (poly,sheet,_) in fragments {
                // beyond the foot's line, on the side away from the region
                let d: Vec<f64> = poly.iter().map(|p| (p.1-a.1)*away.0+(p.2-a.2)*away.1).collect();
                let (beyond,within) = cut(&poly,&d,eps);
                if within.len() >= 3 && area2(&within) > area_eps { let bb = bbox(&within); trimmed.push((within,sheet,bb)); }
                if beyond.len() < 3 || area2(&beyond) <= area_eps { continue; }
                // of that, only the part alongside the edge itself goes
                let d: Vec<f64> = beyond.iter().map(|p| (p.1-a.1)*tx+(p.2-a.2)*ty).collect();
                let (past_a,before_a) = cut(&beyond,&d,eps);
                if before_a.len() >= 3 && area2(&before_a) > area_eps { let bb = bbox(&before_a); trimmed.push((before_a,sheet,bb)); }
                if past_a.len() < 3 || area2(&past_a) <= area_eps { continue; }
                let d: Vec<f64> = past_a.iter().map(|p| (p.1-b.1)*tx+(p.2-b.2)*ty).collect();
                let (past_b,_alongside) = cut(&past_a,&d,eps);
                if past_b.len() >= 3 && area2(&past_b) > area_eps { let bb = bbox(&past_b); trimmed.push((past_b,sheet,bb)); }
            }
            fragments = trimmed;
        }
        // fragments to triangles, new corners shared where they coincide
        let mut fresh: std::collections::BTreeMap<(i64,i64),u32> = Default::default();
        let quantum = eps;
        for (poly,sheet,_) in fragments {
            let idx: Vec<u32> = poly.iter().map(|p| match p.0 {
                Some(i) => i,
                None => {
                    let key = ((p.1/quantum).round() as i64,(p.2/quantum).round() as i64);
                    *fresh.entry(key).or_insert_with(|| {
                        out.vertices.push(std::array::from_fn(|k| origin[k]+p.1*u[k]+p.2*v[k]));
                        (out.vertices.len()-1) as u32
                    })
                }
            }).collect();
            for k in 1..poly.len()-1 {
                let (a,b,c) = (poly[0],poly[k],poly[k+1]);
                let twice = (b.1-a.1)*(c.2-a.2)-(c.1-a.1)*(b.2-a.2);
                if twice <= area_eps || idx[0] == idx[k] || idx[k] == idx[k+1] || idx[0] == idx[k+1] { continue; }
                out.triangles.push([idx[0],idx[k],idx[k+1]]); out.sheet.push(sheet);
            }
        }
    }
    (out,replaced)
}
