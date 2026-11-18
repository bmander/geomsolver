//! **A leg under a slide `D`, its candidates** (the module's list, `super`): each plane face at the
//! end pose it leaves by, each silhouette edge swept, each round face at both end poses with its
//! contact swept, and each circle edge swept.
use crate::geom::{Curve,Frame,Surface};
use crate::query::{Located,Place};
use crate::turned::Exact;
use crate::util::space::{add,cross,dot,norm,scale,sub,unit};
#[allow(unused_imports)]
use crate::util::fmath::Det;
use super::{BandContact,Cutter,PAST,SITES,Touch,V,outward,within};
use super::gather::{Candidates,Leg,Round};

/// Every candidate `leg`, a slide, gathers.
pub(super) fn gather(leg: &Leg,cand: &mut Candidates) {
    let d = sub(leg.end.point([0.;3]),leg.start.point([0.;3]));
    let located = Located::new(leg.tool,leg.tol);
    faces(leg,cand,d);
    edges(leg,cand,d);
    // a face no plane: at each end pose its own surface over the stretch it spans (a cylinder's
    // solid, a cone's frustum), and its contact swept: each ruling where its normal is square to the
    // slide swept into the plane tangent there, or a cylinder along the slide swept along itself
    for &k in &leg.read.round {
        let round = Round::new(leg,&located,k);
        match round.face.surface {
            Surface::Cylinder(f,r) => cylinder(leg,cand,&located,&round,(f,r.abs()),d),
            Surface::Cone(f,_,half) => cone(leg,cand,&located,&round,(f,half),d),
            Surface::Sphere(f,r) => ball(leg,cand,&located,&round,(f,r.abs()),d),
            Surface::Torus(f,big,small) => torus(leg,cand,&located,&round,(f,big.abs(),small.abs()),d),
            _ => unreachable!("`read` takes only cylinders, cones, balls and tori round"),
        }
    }
    rims(leg,cand,d);
}

/// Each plane face at the end pose it leaves the slide by (at the start where it enters it, either
/// where it runs along it: one plane), read at its own points there.
fn faces(leg: &Leg,cand: &mut Candidates,d: V) {
    for (k,&(o,n)) in leg.read.faces.iter().enumerate() {
        if leg.read.round.contains(&k) { continue }
        let pose = if dot(n,d) > leg.eps { &leg.end } else { &leg.start };
        cand.add(Cutter::Plane((pose.point(o),n)),leg.face_sites(k).into_iter().map(|x| (pose.point(x),n)).collect());
    }
}

/// Each silhouette edge (its faces' `n·D` of both signs) swept, in the plane through it holding
/// `D`.
fn edges(leg: &Leg,cand: &mut Candidates,d: V) {
    let eps = leg.eps;
    for (&(p,q,_),&[n1,n2]) in leg.read.edges.iter().zip(&leg.read.normals) {
        let (s1,s2) = (dot(n1,d),dot(n2,d));
        if s1*s2 > 0. || (s1.abs() <= eps && s2.abs() <= eps) { continue }
        let n = cross(sub(q,p),d);
        if norm(n) <= eps*norm(d) { continue }
        let n = unit(n);
        let sites = (0..=SITES).flat_map(|i| leg.poses.iter().map(move |m| (m.point(add(p,scale(sub(q,p),within(i)))),n))).collect();
        cand.add(Cutter::Plane((leg.start.point(p),n)),sites);
    }
}

/// A cylinder's face of radius `r`: at both end poses its own solid, and its two rulings square to
/// the slide each swept into its tangent plane — or, along the slide, the face swept along itself.
fn cylinder(leg: &Leg,cand: &mut Candidates,located: &Located,round: &Round,(f,r): (Frame,f64),d: V) {
    let [lo,hi] = round.span;
    let a = unit(f.z);
    let across = sub(d,scale(a,dot(d,a)));
    for pose in [&leg.start,&leg.end] {
        cand.add(Cutter::Cylinder {o:pose.point(f.o),axis:a,r,span:[lo,hi]},round.sites_at(pose));
    }
    if norm(across) <= 1e-9*norm(d) {
        let s = dot(d,a);
        cand.add(Cutter::Cylinder {o:leg.start.point(f.o),axis:a,r,span:[lo+s.min(0.),hi+s.max(0.)]},
            leg.poses.iter().flat_map(|m| round.sites_at(m)).collect());
        return
    }
    let w = unit(cross(a,across));
    for w in [w,scale(w,-1.)] {
        let u = dot(w,f.y).datan2(dot(w,f.x));
        tangent_plane(leg,cand,located,round,u,w);
    }
}

/// A cone's face of half angle `half`: at both end poses its frustum, and each ruling whose normal
/// is square to the slide (two, one or none) swept into its tangent plane.
fn cone(leg: &Leg,cand: &mut Candidates,located: &Located,round: &Round,(f,half): (Frame,f64),d: V) {
    let [lo,hi] = round.span;
    let (sa,ca) = half.dsin_cos();
    // the generator through the cone's point at `u = 0`, turned about its axis
    let p0 = round.face.surface.point([0.,0.]);
    let e = unit(add(scale(f.x,sa),scale(f.z,ca)));
    for pose in [&leg.start,&leg.end] {
        cand.add(Cutter::Revolved {o:pose.point(f.o),axis:f.z,p:pose.point(p0),e,span:[lo*ca,hi*ca]},round.sites_at(pose));
    }
    // the normal `cos α ρ̂(u) − sin α z` square to the slide:
    // `A cos u + B sin u = C`
    let (aa,bb,cc) = (ca*dot(d,f.x),ca*dot(d,f.y),sa*dot(d,f.z));
    let rr = aa.dhypot(bb);
    if !(rr > 1e-12*norm(d)) || cc.abs() > rr { return }
    let (phi,delta) = (bb.datan2(aa),(cc/rr).clamp(-1.,1.).dacos());
    for u in [phi+delta,phi-delta] {
        let w = unit(round.face.surface.normal_raw([u,0.5*(lo+hi)]));
        tangent_plane(leg,cand,located,round,u,w);
    }
}

/// A ball's face of radius `r`: at both end poses its ball, and its great circle square to the
/// slide carried from the start to the end — the cylinder of its radius about the centre's path,
/// over that path.
fn ball(leg: &Leg,cand: &mut Candidates,located: &Located,round: &Round,(f,r): (Frame,f64),d: V) {
    for pose in [&leg.start,&leg.end] {
        cand.add(Cutter::Ball {c:pose.point(f.o),axis:pose.vector(f.z),r},round.sites_at(pose));
    }
    let (len,dd) = (norm(d),unit(d));
    cand.add(Cutter::Cylinder {o:leg.start.point(f.o),axis:dd,r,span:[0.,len]},circle(leg,located,round.k,f.o,dd,r));
}

/// A torus's face, its tube's middle circle of radius `big` and its tube of radius `small`: at both
/// end poses its torus, and its contact swept — along its axis, across it, or leaning to it
/// (`torus_leaning`).
fn torus(leg: &Leg,cand: &mut Candidates,located: &Located,round: &Round,(f,big,small): (Frame,f64,f64),d: V) {
    let (k,z) = (round.k,unit(f.z));
    let tori = [&leg.start,&leg.end].map(|pose| {
        (cand.add(Cutter::Torus {c:pose.point(f.o),axis:pose.vector(z),major:big,minor:small},round.sites_at(pose)),pose)
    });
    let (len,dd) = (norm(d),unit(d));
    if norm(cross(z,dd)) <= 1e-9 {
        // along the axis: its equators, inner and outer, swept into coaxial cylinders
        let s = dot(d,z);
        for r in [big+small,big-small] {
            cand.add(Cutter::Cylinder {o:leg.start.point(f.o),axis:z,r,span:[s.min(0.),s.max(0.)]},circle(leg,located,k,f.o,z,r));
        }
    } else if dot(z,dd).abs() > 1e-9 {
        torus_leaning(leg,cand,located,round,(f,big,small),d,tori)
    } else {
        // across the axis: its top and bottom rings swept within their level planes
        for side in [1.,-1.] {
            let o = add(f.o,scale(z,side*small));
            let sites: Vec<(V,V)> = circle(leg,located,k,o,z,big).into_iter().map(|(x,_)| (x,scale(z,side))).collect();
            cand.add(Cutter::Plane((leg.start.point(o),z)),sites);
        }
        // and its meridians square to the slide swept into cylinders along it
        let w = unit(cross(z,dd));
        for side in [1.,-1.] {
            let centre = add(f.o,scale(w,side*big));
            cand.add(Cutter::Cylinder {o:leg.start.point(centre),axis:dd,r:small,span:[0.,len]},circle(leg,located,k,centre,dd,small));
        }
    }
}

/// A torus's face leaning to the slide: a whole ring's two contacts (#805), each a closed curve
/// smooth across the branch locus, swept along the slide into the cylinder over its apparent
/// contour (the contour's region extruded from short of the torus at the start pose to past it at
/// the end) — or a band's (#810), its outer contact's cylinder where the band holds all of it, else
/// its arc on the band swept into the sheet over that arc of the contour, cut at the seams it
/// crosses (`Cutter::Sheet`), so no part of it stands inside the sweep, and at each seam that
/// point's path along the slide given as the sheet's touch with the plane the neighbour's ruling
/// there sweeps (`boolean::Given`), the sheet's rim on it. Each read at the contact's points on the
/// face carried over the roll with the torus's normal there. `tori` are the torus's candidates at
/// the two end poses.
fn torus_leaning(leg: &Leg,cand: &mut Candidates,located: &Located,round: &Round,(f,big,small): (Frame,f64,f64),d: V,
    tori: [(usize,&crate::trajectory::Pose);2]) {
    let (k,z) = (round.k,unit(f.z));
    let (len,dd) = (norm(d),unit(d));
    // the torus at each end pose touches a contour's cylinder or sheet along the contact there, and
    // only there short of the fold (#805, `Slid::meets_again`): given whole, each half or arc of it
    // on its own face
    let contacts = |sheet: usize,on: &[crate::slid::Slid]| -> Vec<Touch> {
        tori.iter().flat_map(|&(torus,pose)| on.iter().map(move |h| Touch {cutters:[torus,sheet],
            curves:vec![Curve::Slid(std::sync::Arc::new(h.at(pose.point(f.o),false)))]})).collect()
    };
    let band = leg.read.bands.iter().find(|b| b.0 == k).map(|&(_,band,contact)| (band,contact));
    let carried = |h: &crate::slid::Slid| -> Vec<(V,V)> {
        (0..=SITES).map(|i| h.d2(h.span[0]+(h.span[1]-h.span[0])*within(i)).0)
            .filter(|&x| located.face_place(k,x) != Place::Out)
            .flat_map(|x| { let n = outward(round.face,x); leg.poses.iter().map(move |m| (m.point(x),m.vector(n))) }).collect()
    };
    let sides: &[f64] = match band { None => &[1.,-1.],Some((_,BandContact::Whole)) => &[1.],Some(_) => &[] };
    let reach = big*norm(sub(dd,scale(z,dot(dd,z))))+small;
    let span = [-(1.+PAST)*reach,len+(1.+PAST)*reach];
    for &side in sides {
        let on = crate::slid::halves(f,big,small,dd,side,false);
        let sites = on.iter().flat_map(|h| carried(h)).collect();
        let contour = cand.add(Cutter::Contour {halves:on.map(|h| h.at(leg.start.point(f.o),true)),span},sites);
        cand.touches.extend(contacts(contour,&on));
    }
    if let Some((band,BandContact::Arc {centre,half,seam})) = band {
        let arc = band.arc(dd,centre,half);
        let sheet = cand.add(Cutter::Sheet {arc:arc.at(leg.start.point(f.o),true)},carried(&arc));
        cand.touches.extend(contacts(sheet,&[arc]));
        // where the contact crosses the seam: its path along the slide, on the plane the
        // neighbour's ruling there sweeps (entered by that neighbour, `tangent_plane`; here, with no
        // sites, it is named)
        for u in [centre-half,centre+half] {
            let (x,n) = band.at(u,band.v[seam]);
            let plane = cand.add(Cutter::Plane((leg.start.point(x),n)),Vec::new());
            cand.touches.push(Touch {cutters:[plane,sheet],curves:vec![Curve::Line {p:leg.start.point(x),d:dd}]});
            cand.seams.extend([leg.start.point(x),leg.end.point(x)]);
        }
    }
}

/// A circle edge swept: an oblique cylinder (the circle carried along the slide), or within its own
/// plane where the slide lies in it.
fn rims(leg: &Leg,cand: &mut Candidates,d: V) {
    for rim in &leg.read.rims {
        let (f,r) = (rim.frame,rim.r);
        let a = unit(f.z);
        let n_of = |phi: f64| -> V {
            let (sp,cp) = phi.dsin_cos();
            let t = add(scale(f.x,-sp),scale(f.y,cp));
            let n = cross(t,d);
            if norm(n) <= 1e-12*norm(d) { a } else { unit(n) }
        };
        let sites: Vec<(V,V)> = (0..=SITES).flat_map(|i| {
            let phi = rim.span[0]+(rim.span[1]-rim.span[0])*within(i);
            let (sp,cp) = phi.dsin_cos();
            let x = f.at([r*cp,r*sp,0.]);
            let n = n_of(phi);
            leg.poses.iter().map(move |m| (m.point(x),n))
        }).collect();
        let c = leg.start.point(f.o);
        if dot(d,a).abs() <= 1e-9*norm(d) { cand.add(Cutter::Plane((c,a)),sites); }
        else { cand.add(Cutter::Oblique {c,a,r:r.abs(),d},sites); }
    }
}

/// The circle about `centre` of radius `r` square to `n`, read where it is on the tool's face `k`,
/// each point carried over the roll with its way out from the centre.
fn circle(leg: &Leg,located: &Located,k: usize,centre: V,n: V,r: f64) -> Vec<(V,V)> {
    let g = Frame::about(centre,n);
    (0..4*SITES).flat_map(|i| {
        let (sp,cp) = ((i as f64+0.5)*std::f64::consts::TAU/(4*SITES) as f64).dsin_cos();
        let w = add(scale(g.x,cp),scale(g.y,sp));
        let x = add(centre,scale(w,r));
        let on = located.face_place(k,x) != Place::Out;
        leg.poses.iter().filter(move |_| on).map(move |m| (m.point(x),w))
    }).collect()
}

/// A round face's ruling at `u` over its span, where its normal `w` is square to the slide, swept:
/// the plane tangent to the face there, holding the slide, read at the ruling's points carried over
/// the roll (none where the ruling is not on the face).
fn tangent_plane(leg: &Leg,cand: &mut Candidates,located: &Located,round: &Round,u: f64,w: V) {
    let ([lo,hi],k) = (round.span,round.k);
    let surface = &round.face.surface;
    let at = |s: f64| surface.point([u,lo+(hi-lo)*s]);
    if located.face_place(k,at(0.5)) == Place::Out { return }
    let sites = (0..=SITES).flat_map(|i| { let x = at((i as f64+0.5)/(SITES+1) as f64); leg.poses.iter().map(move |m| (m.point(x),w)) }).collect();
    cand.add(Cutter::Plane((leg.start.point(at(0.5)),w)),sites);
}
