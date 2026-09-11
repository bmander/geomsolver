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

use crate::space::{add,cross,dot,norm,sub};

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
        crate::space::stable_normal(a,b,c).map(|n| (n,dot(n,a)))
    };
    // groups by unsigned plane; the side a group faces is its members'
    // area-weighted majority, so one sliver wound by a garbage normal
    // cannot turn a plane over
    let mut groups: Vec<(V3,f64,Vec<usize>,V3)> = Vec::new();
    let mut loose: Vec<usize> = Vec::new();
    // Groups indexed by their normal on a grid coarser than the match: two
    // normals within the 1e-6 of a match differ by under 1.5e-3 in every
    // component, so a matching group is in a cell next to the triangle's
    // normal or to its reverse. Of the groups found there, the first made is
    // taken, as a scan of every group would: a curved sheet is a group per
    // triangle, and that scan was quadratic.
    let mut index = crate::space::Grid::new(2e-3);
    for (i,t) in mesh.triangles.iter().enumerate() {
        let Some((n,d)) = normal(t) else { loose.push(i); continue };
        let corners = t.map(|v| mesh.vertices[v as usize]);
        let weighted = { let [a,b,c] = corners; cross(sub(b,a),sub(c,a)) };
        let matches = |(gn,gd,_,_): &(V3,f64,Vec<usize>,V3)| dot(n,*gn).abs() >= 1.-1e-6 && corners.iter().all(|p| (dot(*gn,*p)-gd).abs() <= tolerance) && (d*dot(n,*gn).signum()-gd).abs() <= tolerance;
        let mut g: Option<usize> = None;
        for sign in [1.,-1.] {
            index.around(n.map(|x| sign*x),|c| { let c = c as usize; if g.is_none_or(|g| c < g) && matches(&groups[c]) { g = Some(c); } });
        }
        match g {
            Some(g) => { groups[g].2.push(i); groups[g].3 = add(groups[g].3,weighted); },
            None => { index.insert(n,groups.len() as u32); groups.push((n,d,vec![i],weighted)); }
        }
    }
    // the side each triangle faces: its group's majority where it has one
    // (a folded ribbon's own winding is not to be trusted), else its own
    let mut facing_of: Vec<Option<V3>> = vec![None;mesh.triangles.len()];
    for (_,_,members,facing) in &groups {
        let l = norm(*facing);
        if l > 0. { for &i in members { facing_of[i] = Some(facing.map(|x| x/l)); } }
    }
    // Triangles by the grid cells their boxes cover, for the trim's feet: a
    // foot matters only where its line crosses the region (area on both
    // sides of it within its span), and the region only shrinks as it is
    // trimmed, so every foot that matters has a point within the region's
    // own diagonal of the region's box. The rest are passed over as the
    // scan of every triangle passed over them.
    let boxes: Vec<(V3,V3)> = mesh.triangles.iter().map(|t| {
        let c = t.map(|v| mesh.vertices[v as usize]);
        (std::array::from_fn(|k| c[0][k].min(c[1][k]).min(c[2][k])),std::array::from_fn(|k| c[0][k].max(c[1][k]).max(c[2][k])))
    }).collect();
    let (lo,hi) = boxes.iter().fold(([f64::INFINITY;3],[f64::NEG_INFINITY;3]),|(lo,hi),(a,b)| (std::array::from_fn(|k| lo[k].min(a[k])),std::array::from_fn(|k| hi[k].max(b[k]))));
    // Cells about a triangle's size, widened while the mesh's box would hold more cells than a
    // few per triangle; a query reaching more of those cells than there are triangles reads them all.
    let count = mesh.triangles.len();
    let size = boxes.iter().map(|(a,b)| (0..3).map(|k| b[k]-a[k]).fold(0.,f64::max)).sum::<f64>()/count.max(1) as f64;
    let mut cell = if size.is_finite() && size > 0. { size } else { 1. };
    let dims = |cell: f64| -> [usize;3] { std::array::from_fn(|k| (((hi[k]-lo[k])/cell).floor().max(0.) as usize).saturating_add(1)) };
    while { let d = dims(cell); d[0].saturating_mul(d[1]).saturating_mul(d[2]) > 8*count+64 } { cell *= 1.5; }
    let d = dims(cell);
    let at = |p: V3| -> [usize;3] { std::array::from_fn(|k| (((p[k]-lo[k])/cell).floor().max(0.) as usize).min(d[k]-1)) };
    let feet_grid = {
        let mut grid = crate::space::Grid::new(cell);
        if trim { for (i,(a,b)) in boxes.iter().enumerate() { grid.insert_box(*a,*b,i as u32); } }
        grid.pack()
    };
    // the last group each triangle was gathered for, so one reached through several cells is
    // taken once
    let mut stamp = vec![u32::MAX;count];
    let mut query = 0u32;
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
        let mut near: Vec<usize> = Vec::new();
        if trim {
            let (glo,ghi) = members.iter().fold(([f64::INFINITY;3],[f64::NEG_INFINITY;3]),|(lo,hi),&i| (std::array::from_fn(|k| lo[k].min(boxes[i].0[k])),std::array::from_fn(|k| hi[k].max(boxes[i].1[k]))));
            let reach = norm(sub(ghi,glo))+tolerance;
            let (qa,qb) = (glo.map(|x| x-reach),ghi.map(|x| x+reach));
            let (ka,kb) = (at(qa),at(qb));
            let cells = (0..3).map(|k| kb[k]-ka[k]+1).product::<usize>();
            if cells > count {
                // a region as wide as the mesh (a planar face) reaches every
                // triangle, and the plain scan is the cheaper walk
                near.extend(0..count);
            } else {
                query += 1;
                feet_grid.in_box(qa,qb,|i| {
                    let i = i as usize;
                    if stamp[i] == query { return; }
                    stamp[i] = query;
                    let (a,b) = boxes[i];
                    if (0..3).all(|k| a[k] <= qb[k] && b[k] >= qa[k]) { near.push(i); }
                });
                near.sort_unstable();
            }
        }
        for &i in &near {
            let t = &mesh.triangles[i];
            if in_group.contains(&i) { continue; }
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
        // fragments to triangles, a new corner shared with the nearest one made before it within
        // `eps` (rounding to a grid instead split two coincident corners either side of a cell's
        // edge, so a move far below `eps` changed which corners were one)
        let mut fresh = crate::space::Grid::new(eps.max(f64::MIN_POSITIVE));
        let mut made: Vec<(f64,f64,u32)> = Vec::new();
        for (poly,sheet,_) in fragments {
            let idx: Vec<u32> = poly.iter().map(|p| match p.0 {
                Some(i) => i,
                None => {
                    let at = [p.1,p.2,0.];
                    let mut nearest: Option<(f64,u32)> = None;
                    fresh.around(at,|m| {
                        let (x,y,i) = made[m as usize];
                        let d = ((x-p.1).powi(2)+(y-p.2).powi(2)).sqrt();
                        if d <= eps && nearest.is_none_or(|(e,_)| d < e) { nearest = Some((d,i)); }
                    });
                    nearest.map(|(_,i)| i).unwrap_or_else(|| {
                        out.vertices.push(std::array::from_fn(|k| origin[k]+p.1*u[k]+p.2*v[k]));
                        let i = (out.vertices.len()-1) as u32;
                        fresh.insert(at,made.len() as u32); made.push((p.1,p.2,i));
                        i
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
