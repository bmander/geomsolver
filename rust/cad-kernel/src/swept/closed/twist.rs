//! **A leg under a turn or a screw, its candidates** (the module's list, `super`): each plane face
//! at the end poses and its contact line turned or screwed, each round face at the end poses and its
//! contact turned, each sharp circle turned or screwed, each level curve turned within its plane,
//! and each edge turned or screwed.
use std::collections::BTreeMap;
use crate::geom::{Curve,Frame,Surface};
use crate::query::{Located,Place};
use crate::trajectory::{Pose,Screw};
use crate::util::space::{add,cross,dot,norm,scale,sub,unit};
#[allow(unused_imports)]
use crate::util::fmath::Det;
use super::{Cutter,PAST,SITES,Touch,Unbuilt,V,foot_on,outward,whole_turn,within};
use super::gather::{Candidates,Leg,Round};

/// Each face's candidate at each end pose, by the face and the pose's place in `Twist::ends`, where
/// it holds a touch given by construction (#810).
type Ends = BTreeMap<(usize,usize),usize>;

/// Every candidate `leg`, a turn or a screw, gathers.
pub(super) fn gather(leg: &Leg,cand: &mut Candidates) -> Result<(),Unbuilt> {
    let tw = Twist::new(leg);
    let mut at_ends = Ends::new();
    faces(&tw,cand,&mut at_ends);
    // a round face (`reads` takes one only under a turn): at both end poses its own surface (the
    // cylinder, the frustum, the ball), and its contact turned. Its normal passes through the axis
    // where it lies in the plane through the axis and the face's own axis, or its centre: a
    // cylinder's or a cone's two generators there, each turned (a cylinder or a cone about the axis,
    // `turned`), or a ball's great circle there, turned into a torus
    let located = Located::new(leg.tool,leg.tol);
    for &k in &leg.read.round {
        let round = Round::new(leg,&located,k);
        match round.face.surface {
            Surface::Cylinder(f,r) => cylinder(&tw,cand,&located,&round,&mut at_ends,(f,r.abs()))?,
            Surface::Cone(f,r,half) => cone(&tw,cand,&located,&round,&mut at_ends,(f,r,half))?,
            Surface::Sphere(f,r) if !tw.screw.is_turn() => ball_screwed(&tw,cand,&located,&round,(f,r.abs()))?,
            Surface::Sphere(f,r) => ball_turned(&tw,cand,&located,&round,(f,r.abs()))?,
            Surface::Torus(f,big,small) => torus(&tw,cand,&located,&round,(f,big.abs(),small.abs()))?,
            _ => unreachable!("`read` takes only cylinders, cones, balls and tori round"),
        }
    }
    if tw.screw.is_turn() { turned_rims(&tw,cand,&at_ends)? } else { screwed_rims(&tw,cand) }
    // under a turn a curve in a level plane (a circle about a line along the axis, an ellipse or a
    // conic a level plane cuts from a leaning round face, #780 C3) turns within it
    for points in &leg.read.levels {
        let sites: Vec<(V,V)> = points.iter().flat_map(|&x| leg.poses.iter().map(move |m| (m.point(x),tw.axis))).collect();
        cand.add(Cutter::Plane((points[0],tw.axis)),sites);
    }
    for &(p,q,_) in &leg.read.edges {
        if let Some(cutter) = tw.line_cutter(p,q) { cand.add(cutter,tw.line_sites(p,q,cutter)); }
    }
    Ok(())
}

/// **A leg under a turn or a screw**, as its sources are turned: the leg, its twist's axis through
/// `c`, the turn's frame a turned meridian is read in (#780 C3, L2), whether the roll is a whole
/// turn, the poses each face is taken at, and the heights a source's run on by over the roll.
struct Twist<'a> {
    leg: &'a Leg<'a>,
    screw: Screw,
    c: V,
    axis: V,
    g: Frame,
    whole: bool,
    /// Both ends of the roll — but a whole turn's sweep is its tool's solid of revolution, bounded
    /// by its contacts, edges and rims turned alone, which its faces at either end only touch (#780
    /// C3: a leaning stem at an end pose touching its contact's surface of revolution along the
    /// contact, crossed there by its ball's torus, was a triple touch no arrangement holds by
    /// rounding), so none.
    ends: Vec<&'a Pose>,
    /// Under a screw the heights a source spans run on by the step it takes over the roll, the rest
    /// as under a turn (a cylinder about the axis keeps itself under both).
    lift: [f64;2],
}

impl<'a> Twist<'a> {
    fn new(leg: &'a Leg<'a>) -> Twist<'a> {
        let screw = leg.screw;
        let (c,axis) = (screw.origin,screw.axis);
        let whole = whole_turn(&screw,leg.roll);
        let ends = if screw.is_turn() && whole { vec![] } else { vec![&leg.start,&leg.end] };
        let [a,b] = leg.roll;
        let rate = screw.advance/std::f64::consts::TAU;
        Twist {leg,screw,c,axis,g:Frame::about(c,axis),whole,ends,lift:[(rate*a).min(rate*b),(rate*a).max(rate*b)]}
    }

    /// A line through `p` along `e` to `q` turned whole: a cylinder about the axis where it runs
    /// along it, a plane square to the axis where it runs square to it, else a hyperboloid (a double
    /// cone where it meets the axis).
    fn turned(&self,p: V,e: V,q: V) -> Option<Cutter> {
        let (c,axis) = (self.c,self.axis);
        let e = unit(e);
        let (h0,h1) = (dot(sub(p,c),axis),dot(sub(q,c),axis));
        let span = [h0.min(h1)+self.lift[0],h0.max(h1)+self.lift[1]];
        if norm(cross(e,axis)) <= 1e-9 {
            let r = { let r = sub(p,c); norm(sub(r,scale(axis,dot(r,axis)))) };
            return (r > self.leg.eps).then_some(Cutter::Cylinder {o:c,axis,r,span})
        }
        if dot(e,axis).abs() <= 1e-9 { return Some(Cutter::Plane((p,axis))) }
        Some(Cutter::Revolved {o:c,axis,p,e,span})
    }

    /// A segment from `p` to `q` under the twist: turned (`turned`), or under a screw, leaning to its
    /// axis, a patch of the ruled helicoid it sweeps (#777 M5), square to it the helicoidal sheet it
    /// sweeps across the box.
    fn line_cutter(&self,p: V,q: V) -> Option<Cutter> {
        let e = unit(sub(q,p));
        if !self.screw.is_turn() && norm(cross(e,self.axis)) > 1e-9 {
            return Some(if dot(e,self.axis).abs() > 1e-9 { Cutter::Patch {p,q} } else { Cutter::Screwed {p,e} })
        }
        self.turned(p,sub(q,p),q)
    }

    /// The sites of a line from `p` to `q` turned: its points at each pose, with the normal of the
    /// surface it turns into there.
    fn line_sites(&self,p: V,q: V,cutter: Cutter) -> Vec<(V,V)> {
        let (c,axis,screw) = (self.c,self.axis,&self.screw);
        (0..=SITES).flat_map(|i| {
            let x = add(p,scale(sub(q,p),within(i)));
            let r = sub(x,c);
            let radial = sub(r,scale(axis,dot(r,axis)));
            let normal = match cutter {
                Cutter::Plane((_,m)) => m,
                Cutter::Cylinder {..} => unit(radial),
                Cutter::Revolved {e,..} => {
                    let (rho,ez) = (norm(radial),dot(e,axis));
                    let slope = dot(radial,sub(e,scale(axis,ez)))/(ez*rho.max(1e-300));
                    unit(sub(scale(radial,1./rho.max(1e-300)),scale(axis,slope)))
                }
                // the sheet's normal: square to the line and to the screw's velocity there (which the
                // screw carries as it carries the normal)
                Cutter::Screwed {e,..} => unit(cross(e,screw.velocity(x))),
                Cutter::Patch {p,q} => unit(cross(unit(sub(q,p)),screw.velocity(x))),
                Cutter::Oblique {..} | Cutter::Ball {..} | Cutter::Torus {..} | Cutter::ScrewedDisc {..} | Cutter::Pipe {..}
                    | Cutter::Turned(_) | Cutter::Contour {..} | Cutter::Sheet {..} => unreachable!("a line turns into none of these"),
            };
            self.leg.poses.iter().map(move |m| (m.point(x),m.vector(normal)))
        }).collect()
    }

    /// A round face's ruling at `u` round its axis, where it is on the face, under the twist: its
    /// candidate and sites.
    fn ruling(&self,located: &Located,round: &Round,u: f64) -> Option<(Cutter,Vec<(V,V)>)> {
        let ([lo,hi],surface) = (round.span,&round.face.surface);
        let (p,q) = (surface.point([u,lo]),surface.point([u,hi]));
        if located.face_place(round.k,surface.point([u,0.5*(lo+hi)])) == Place::Out { return None }
        let cutter = self.line_cutter(p,q)?;
        Some((cutter,self.line_sites(p,q,cutter)))
    }

    /// The way from the axis to `o` (a round face's own axis or centre), square to the axis.
    fn away(&self,o: V) -> Option<V> {
        let r = sub(o,self.c);
        let w = sub(r,scale(self.axis,dot(r,self.axis)));
        (norm(w) > self.leg.eps).then(|| unit(w))
    }

    /// Whether a round face about the frame `f`'s axis leans to the turn's (#780 C3).
    fn leaning(&self,f: &Frame) -> bool { self.screw.is_turn() && norm(cross(f.z,self.axis)) > 1e-9 }

    /// The span a cylinder's or a cone's own surface is entered over at the end poses: about an axis
    /// leaning to the turn's (#780 C3), half `PAST` past its own each way (no end of it on its
    /// contact's meridian's, `PAST` past, nor on a face it meets at a concave edge — a stem's rim on
    /// the ball it enters); else its own.
    fn own(&self,f: &Frame,[lo,hi]: [f64;2]) -> [f64;2] {
        if self.leaning(f) { let past = 0.5*PAST*(hi-lo); [lo-past,hi+past] } else { [lo,hi] }
    }
}

/// Each plane face at each end pose, and its contact turned or screwed: under a turn a face square
/// to the axis its own plane, any other its contact line turned; under a screw a face leaning to the
/// axis its contact a patch of a ruled helicoid, one along it its line nearest the axis.
fn faces(tw: &Twist,cand: &mut Candidates,at_ends: &mut Ends) {
    let leg = tw.leg;
    let (c,axis,eps,screw) = (tw.c,tw.axis,leg.eps,&tw.screw);
    for (k,&(o,n)) in leg.read.faces.iter().enumerate() {
        if leg.read.round.contains(&k) { continue }
        let own = leg.face_sites(k);
        for (j,pose) in tw.ends.iter().enumerate() {
            at_ends.insert((k,j),cand.add(Cutter::Plane((pose.point(o),pose.vector(n))),own.iter().map(|&x| (pose.point(x),pose.vector(n))).collect()));
        }
        let m = cross(n,axis);
        // a face square to the axis: under a turn its own plane (kept in itself), under a screw no
        // contact at all (its normal meets the screw's velocity everywhere, by the step)
        if norm(m) <= 1e-9 {
            if screw.is_turn() { cand.add(Cutter::Plane((o,n)),leg.poses.iter().flat_map(|pose| own.iter().map(|&x| (pose.point(x),n))).collect()); }
            continue
        }
        // under a screw a face leaning to the axis touches where its normal is square to the screw's
        // velocity, `n · v(x) = 0`, affine in `x`: a line of its plane, leaning, screwed into a patch
        // of a ruled helicoid (#777 M5)
        if !screw.is_turn() && dot(n,axis).abs() > 1e-9 {
            if let Some((from,to)) = leaning_contact(screw,o,n,&leg.corners[k],eps) {
                let cutter = Cutter::Patch {p:from,q:to};
                cand.add(cutter,tw.line_sites(from,to,cutter));
            }
            continue
        }
        // under a screw a face along the axis touches along its line nearest the axis, as under a
        // turn
        // the face's contact line: its plane's points whose normal passes through the axis, where its
        // plane meets the plane through the axis along `n × axis`; it meets the axis, or runs along
        // it
        if dot(n,sub(o,c)).abs() <= eps { continue }
        let (x0,e) = (add(c,scale(n,dot(n,sub(o,c)))),unit(cross(n,m)));
        let Some((t0,t1)) = contact_stretch(leg,k,n,x0,e) else { continue };
        let (from,to) = (add(x0,scale(e,t0)),add(x0,scale(e,t1)));
        if let Some(cutter) = tw.turned(from,e,to) { cand.add(cutter,tw.line_sites(from,to,cutter)); }
    }
}

/// The stretch of a plane face's contact line through `x0` along `e` the face `k` (its normal `n`)
/// spans: over its corners, or where a face with an edge no line has one, the chord its outline
/// cuts from the line, a twentieth more each way (none where the line misses the face, #780 L2).
fn contact_stretch(leg: &Leg,k: usize,n: V,x0: V,e: V) -> Option<(f64,f64)> {
    let Some(outline) = &leg.outlines[k] else {
        return Some(leg.corners[k].iter().map(|v| dot(sub(*v,x0),e)).fold((f64::INFINITY,f64::NEG_INFINITY),|(l,h),t| (l.min(t),h.max(t))))
    };
    let across = unit(cross(n,e));
    let side = |p: V| dot(sub(p,x0),across);
    let cuts: Vec<f64> = (0..outline.len()).filter_map(|i| {
        let (p,q) = (outline[i],outline[(i+1)%outline.len()]);
        let (sp,sq) = (side(p),side(q));
        (sp*sq <= 0. && sp != sq).then(|| dot(sub(add(p,scale(sub(q,p),sp/(sp-sq))),x0),e))
    }).collect();
    if cuts.is_empty() { return None }
    let (lo,hi) = cuts.iter().fold((f64::INFINITY,f64::NEG_INFINITY),|(l,h),&t| (l.min(t),h.max(t)));
    let past = PAST*(hi-lo);
    Some((lo-past,hi+past))
}

/// A cylinder's face of radius `r`: its own surface at the end poses, and its contact — about an
/// axis leaning to the turn's turned (`turned_contacts`), coaxial the face itself, else its two
/// generators in the plane through both axes turned.
fn cylinder(tw: &Twist,cand: &mut Candidates,located: &Located,round: &Round,at_ends: &mut Ends,(f,r): (Frame,f64))
    -> Result<(),Unbuilt> {
    let (leg,k,[lo,hi]) = (tw.leg,round.k,round.span);
    // past each sharp circle it holds that turns into a ring (#780 L2), beyond the loop along which
    // its surface at an end pose crosses that ring (`turned::Crossing`, its heights from the circle
    // the turn keeps): its ends there stand clear of that loop, which near the circle runs where the
    // two all but touch
    let mut span = tw.own(&f,round.span);
    if tw.leaning(&f) {
        let a = unit(f.z);
        for rim in &leg.read.rims {
            let Some(crossing) = crate::turned::Crossing::held(&rim.ring(&tw.g),tw.g.dir_local(a),tw.g.local(f.o),r,leg.eps) else { continue };
            let ([h0,h1],at) = (crossing.heights(),dot(sub(rim.frame.o,f.o),a));
            let reach = 1.25*h0.abs().max(h1.abs());
            span = [span[0].min(at-reach),span[1].max(at+reach)];
        }
    }
    for (j,pose) in tw.ends.iter().enumerate() {
        at_ends.insert((k,j),cand.add(Cutter::Cylinder {o:pose.point(f.o),axis:pose.vector(unit(f.z)),r,span},round.sites_at(pose)));
    }
    if tw.leaning(&f) {
        let turned = turned_contacts(tw,cand,located,round,(f,r,0.,1.))?;
        cand.touches.extend(own_contacts(&round.face.surface,k,&turned,&tw.ends,at_ends,leg.tol));
        return Ok(())
    }
    let ways = match tw.away(f.o) { Some(w) => vec![w,scale(w,-1.)],None => vec![] };
    // coaxial, the face turns into itself
    if ways.is_empty() { cand.add(Cutter::Cylinder {o:f.o,axis:unit(f.z),r,span:[lo,hi]},leg.poses.iter().flat_map(|m| round.sites_at(m)).collect()); }
    for w in ways {
        if let Some((cutter,sites)) = tw.ruling(located,round,dot(w,f.y).datan2(dot(w,f.x))) { cand.add(cutter,sites); }
    }
    Ok(())
}

/// A cone's face of half angle `half`: its frustum at the end poses, and its contact — about an
/// axis leaning to the turn's turned (`turned_contacts`), under a screw its two rulings square to
/// the velocity screwed, coaxial the face itself, else its two generators in the plane through both
/// axes turned.
fn cone(tw: &Twist,cand: &mut Candidates,located: &Located,round: &Round,at_ends: &mut Ends,(f,r,half): (Frame,f64,f64))
    -> Result<(),Unbuilt> {
    let (leg,k,[lo,hi],screw) = (tw.leg,round.k,round.span,&tw.screw);
    let (sa,ca) = half.dsin_cos();
    let p0 = round.face.surface.point([0.,0.]);
    let e = unit(add(scale(f.x,sa),scale(f.z,ca)));
    let [l0,h0] = tw.own(&f,round.span);
    for (j,pose) in tw.ends.iter().enumerate() {
        at_ends.insert((k,j),cand.add(Cutter::Revolved {o:pose.point(f.o),axis:pose.vector(f.z),p:pose.point(p0),
            e:pose.vector(e),span:[l0*ca,h0*ca]},round.sites_at(pose)));
    }
    if tw.leaning(&f) {
        let turned = turned_contacts(tw,cand,located,round,(f,r,sa,ca))?;
        cand.touches.extend(own_contacts(&round.face.surface,k,&turned,&tw.ends,at_ends,leg.tol));
        return Ok(())
    }
    if !screw.is_turn() {
        // under a screw its normal `cos α ρ̂(u) − sin α z` is square to the velocity
        // `ω axis × (x − c) + κ axis` where `A cos u + B sin u = C`, whatever the height along the
        // ruling (the turn's part along `ρ̂` is `ω ρ̂ · (axis × (o − c))`, the climb's along `z` is
        // `κ`): two rulings turned off the plane through both axes by `asin(κ tan α / (ω |d|))`, or
        // none (a cone about the screw's own axis, or a climb too steep for its lean). Each is a line
        // leaning to the axis, screwed into a patch of a ruled helicoid (#777 M5), which the cone at
        // each end pose touches along that ruling
        let w = cross(tw.axis,sub(f.o,tw.c));
        let kappa = screw.advance/std::f64::consts::TAU*dot(f.z,tw.axis).signum();
        let (aa,bb,cc) = (screw.ratio*ca*dot(w,f.x),screw.ratio*ca*dot(w,f.y),sa*kappa);
        let rr = aa.dhypot(bb);
        if !(rr > 1e-12*(screw.ratio.abs()*leg.size+kappa.abs())) || cc.abs() >= rr { return Ok(()) }
        let (phi,delta) = (bb.datan2(aa),(cc/rr).clamp(-1.,1.).dacos());
        for u in [phi+delta,phi-delta] {
            if let Some((cutter,sites)) = tw.ruling(located,round,u) { cand.add(cutter,sites); }
        }
        return Ok(())
    }
    let ways = match tw.away(f.o) { Some(w) => vec![w,scale(w,-1.)],None => vec![] };
    if ways.is_empty() {
        cand.add(Cutter::Revolved {o:f.o,axis:f.z,p:p0,e,span:[lo*ca,hi*ca]},leg.poses.iter().flat_map(|m| round.sites_at(m)).collect());
    }
    for w in ways {
        if let Some((cutter,sites)) = tw.ruling(located,round,dot(w,f.y).datan2(dot(w,f.x))) { cand.add(cutter,sites); }
    }
    Ok(())
}

/// A ball's face of radius `r` under a screw: its ball at the end poses, and its great circle square
/// to its centre's velocity, screwed: the pipe — entered in stretches of the roll each meeting itself
/// nowhere where it comes back onto itself (#780 L1, X5).
fn ball_screwed(tw: &Twist,cand: &mut Candidates,located: &Located,round: &Round,(f,r): (Frame,f64)) -> Result<(),Unbuilt> {
    let (leg,k) = (tw.leg,round.k);
    for pose in &tw.ends {
        cand.add(Cutter::Ball {c:pose.point(f.o),axis:pose.vector(f.z),r},round.sites_at(pose));
    }
    let n = unit(tw.screw.velocity(f.o));
    let g = Frame::about(f.o,n);
    // the circle's points on the face and their normals, carried over each stretch's own roll
    let ring: Vec<(V,V)> = (0..4*SITES).filter_map(|i| {
        let (sp,cp) = ((i as f64+0.5)*std::f64::consts::TAU/(4*SITES) as f64).dsin_cos();
        let w = add(scale(g.x,cp),scale(g.y,sp));
        let x = add(f.o,scale(w,r));
        (located.face_place(k,x) != Place::Out).then_some((x,w))
    }).collect();
    for span in pipe_stretches(&tw.screw,f.o,n,r,leg.roll,leg.eps)? {
        let poses: Vec<Pose> = (0..=SITES).map(|i| leg.pose_at(span[0]+(span[1]-span[0])*within(i))).collect::<Result<_,_>>()?;
        let sites = ring.iter().flat_map(|&(x,w)| poses.iter().map(move |m| (m.point(x),m.vector(w)))).collect();
        cand.add(Cutter::Pipe {c:f.o,n,r,span},sites);
    }
    Ok(())
}

/// A ball's face of radius `r` under a turn: its ball at the end poses, and its great circle in the
/// plane through the axis turned into a torus (a ball about a point of the axis turns into itself).
fn ball_turned(tw: &Twist,cand: &mut Candidates,located: &Located,round: &Round,(f,r): (Frame,f64)) -> Result<(),Unbuilt> {
    let (leg,k,c,axis) = (tw.leg,round.k,tw.c,tw.axis);
    for pose in &tw.ends {
        cand.add(Cutter::Ball {c:pose.point(f.o),axis:pose.vector(f.z),r},round.sites_at(pose));
    }
    let height = dot(sub(f.o,c),axis);
    let Some(w) = tw.away(f.o) else {
        // a ball about a point of the axis turns into itself
        cand.add(Cutter::Ball {c:f.o,axis:f.z,r},leg.poses.iter().flat_map(|m| round.sites_at(m)).collect());
        return Ok(())
    };
    let major = norm(sub(sub(f.o,c),scale(axis,height)));
    // a ball reaching across the axis turns into no ring torus: not held yet
    if !(major > r+leg.eps) { return Err(Unbuilt::Curved {at:f.o}) }
    let sites: Vec<(V,V)> = (0..4*SITES).flat_map(|i| {
        let (sp,cp) = ((i as f64+0.5)*std::f64::consts::TAU/(4*SITES) as f64).dsin_cos();
        let n = add(scale(w,cp),scale(axis,sp));
        let x = add(f.o,scale(n,r));
        let on = located.face_place(k,x) != Place::Out;
        leg.poses.iter().filter(move |_| on).map(move |m| (m.point(x),m.vector(n)))
    }).collect();
    cand.add(Cutter::Torus {c:add(c,scale(axis,height)),axis,major,minor:r},sites);
    Ok(())
}

/// A torus's face, its tube's middle circle of radius `big` and its tube of radius `r`: its torus at
/// the end poses, its top and bottom rings each in its own plane, and its meridians in the plane
/// through both axes each turned into a torus about the axis (a torus about the axis turns into
/// itself).
fn torus(tw: &Twist,cand: &mut Candidates,located: &Located,round: &Round,(f,big,r): (Frame,f64,f64)) -> Result<(),Unbuilt> {
    let (leg,k,c,axis) = (tw.leg,round.k,tw.c,tw.axis);
    let z = unit(f.z);
    for pose in &tw.ends {
        cand.add(Cutter::Torus {c:pose.point(f.o),axis:pose.vector(z),major:big,minor:r},round.sites_at(pose));
    }
    // the face's points of a circle on the torus about `o` in the plane of `p` and `q`, each with
    // its normal, carried over the roll
    let circle_sites = |o: V,p: V,q: V,rad: f64| -> Vec<(V,V)> {
        (0..4*SITES).flat_map(|i| {
            let (sp,cp) = ((i as f64+0.5)*std::f64::consts::TAU/(4*SITES) as f64).dsin_cos();
            let x = add(o,add(scale(p,rad*cp),scale(q,rad*sp)));
            let n = outward(round.face,x);
            let on = located.face_place(k,x) != Place::Out;
            leg.poses.iter().filter(move |_| on).map(move |m| (m.point(x),m.vector(n)))
        }).collect()
    };
    // its top and bottom rings, their normals along the axis: each in its own plane
    for side in [1.,-1.] {
        let o = add(f.o,scale(z,side*r));
        cand.add(Cutter::Plane((o,scale(z,side))),circle_sites(o,f.x,cross(z,f.x),big));
    }
    let height = dot(sub(f.o,c),axis);
    let Some(w) = tw.away(f.o) else {
        // a torus about the axis turns into itself
        cand.add(Cutter::Torus {c:f.o,axis:z,major:big,minor:r},leg.poses.iter().flat_map(|m| round.sites_at(m)).collect());
        return Ok(())
    };
    // its meridians in the plane through both axes, each turned into a torus about the axis (one
    // reaching across the axis turns into no ring torus: not held yet)
    for side in [1.,-1.] {
        let m = add(f.o,scale(w,side*big));
        let major = norm(sub(sub(m,c),scale(axis,dot(sub(m,c),axis))));
        if !(major > r+leg.eps) { return Err(Unbuilt::Curved {at:m}) }
        cand.add(Cutter::Torus {c:add(c,scale(axis,height)),axis,major,minor:r},circle_sites(m,w,axis,r));
    }
    Ok(())
}

/// Under a turn a sharp circle leaning to the axis (#780 L2): its fan's contact — where the fan
/// between its faces' normals holds one square to the turn's velocity — lies on the surface its
/// points sweep, a ring whose meridian is read exactly (`turned::Rim`, `Cutter::Turned`), its sites
/// the circle's points carried over the roll, each with the ring's normal there (square to the
/// circle and to the velocity). An arc of one sweeps a part of its circle's ring: the whole ring is
/// its candidate, its sites the arc's own points, and membership keeps what bounds the sweep; two
/// arcs of one circle (a rim a Boolean split) enter one ring. Part of a turn, the faces it bounds at
/// each end pose hold it, met in closed form where they are planes and cylinders (`ssi`'s
/// `plane_holding_rim`, `side_holding_rim`); beside any other round face (a cone, a ball) it is
/// refused by name, that face's meeting with the ring not yet in closed form.
fn turned_rims(tw: &Twist,cand: &mut Candidates,at_ends: &Ends) -> Result<(),Unbuilt> {
    let leg = tw.leg;
    let (tool,eps,tol) = (leg.tool,leg.eps,leg.tol);
    for rim in &leg.read.rims {
        let (f,r) = (rim.frame,rim.r.abs());
        let source = rim.ring(&tw.g);
        let held = |k: &usize| matches!(tool.faces[*k].surface,Surface::Plane(_) | Surface::Cylinder(..));
        if !source.simple(eps) || !tw.whole && !rim.faces.iter().all(held) { return Err(Unbuilt::Curved {at:f.o}) }
        let [w0,w1] = rim.span;
        let sites: Vec<(V,V)> = (0..=4*SITES).flat_map(|i| {
            let (sp,cp) = (w0+(w1-w0)*(i as f64+0.5)/(4*SITES+1) as f64).dsin_cos();
            let x = f.at([r*cp,r*sp,0.]);
            let n = unit(cross(add(scale(f.x,-sp),scale(f.y,cp)),tw.screw.velocity(x)));
            leg.poses.iter().map(move |m| (m.point(x),m.vector(n)))
        }).collect();
        let ring = crate::turned::Turned::rim(tw.g,source,[rim.span[0],rim.span[0]+std::f64::consts::TAU]);
        let ring_at = cand.add(Cutter::Turned(ring),sites);
        // each face holding it at each end pose meets its ring in closed form, given (#810: a plane
        // in two circles, `ssi::plane_holding_rim`; a cylinder in the circle and the loop crossing
        // it, `ssi::side_holding_rim`)
        let curve = Curve::Turned(std::sync::Arc::new(ring));
        for (j,pose) in tw.ends.iter().enumerate() {
            for &fk in &rim.faces {
                let Some(&at) = at_ends.get(&(fk,j)) else { continue };
                let met = match tool.faces[fk].surface.moved(&crate::geom::Rigid {r:pose.r,t:pose.p}) {
                    Surface::Plane(pl) => crate::ssi::plane_holding_rim(&pl,&curve,tol),
                    Surface::Cylinder(cy,r) => crate::ssi::side_holding_rim(&cy,r.abs(),&curve,tol),
                    _ => None,
                };
                if let Some(curves) = met { cand.touches.push(Touch {cutters:[at,ring_at],curves}) }
            }
        }
    }
    Ok(())
}

/// Under a screw a circle square to the axis is screwed, the solid its disc sweeps
/// (`Cutter::ScrewedDisc`; under a turn it is a level curve).
fn screwed_rims(tw: &Twist,cand: &mut Candidates) {
    for rim in &tw.leg.read.rims {
        let (f,r) = (rim.frame,rim.r);
        // the screwed sheet's normal: square to the circle and to the screw's velocity
        let normal_at = |x: V| unit(cross(cross(tw.axis,unit(sub(x,f.o))),tw.screw.velocity(x)));
        let sites: Vec<(V,V)> = (0..=SITES).flat_map(|i| {
            let (sp,cp) = (rim.span[0]+(rim.span[1]-rim.span[0])*within(i)).dsin_cos();
            let x = f.at([r*cp,r*sp,0.]);
            let n = normal_at(x);
            tw.leg.poses.iter().map(move |m| (m.point(x),m.vector(n)))
        }).collect();
        cand.add(Cutter::ScrewedDisc {c:f.o,r:r.abs()},sites);
    }
}

/// **A plane face's contact under a screw, leaning to its axis** (#777 M5): its points where the
/// screw's velocity lies in the plane, `n · v(x) = 0` (affine in `x`, so a plane), met with the
/// face's own plane in a line, clipped to the face's polygon `corners` (convex): the chord's ends,
/// or `None` where the line misses the face.
fn leaning_contact(screw: &Screw,o: V,n: V,corners: &[V],eps: f64) -> Option<(V,V)> {
    let a = |x: V| dot(n,screw.velocity(x));
    let a0 = a(o);
    let g: V = std::array::from_fn(|i| { let mut e = [0.;3]; e[i] = 1.; a(add(o,e))-a0 });
    let e = cross(n,g);
    if norm(e) <= 1e-12*norm(g).max(1e-300) { return None }
    let e = unit(e);
    // the line's point: on the plane, on `a = 0`, and level with `o` along it
    let x0 = crate::ssi::solve3([n,g,e],[dot(n,o),dot(g,o)-a0,dot(e,o)]);
    // where it crosses each side of the polygon, its parameter along `e`
    let side = unit(cross(n,e));
    let mut ts: Vec<f64> = Vec::new();
    for k in 0..corners.len() {
        let (u,v) = (corners[k],corners[(k+1)%corners.len()]);
        let (du,dv) = (dot(sub(u,x0),side),dot(sub(v,x0),side));
        if (du > eps && dv > eps) || (du < -eps && dv < -eps) || (du-dv).abs() <= 1e-300 { continue }
        let s = (du/(du-dv)).clamp(0.,1.);
        ts.push(dot(sub(add(u,scale(sub(v,u),s)),x0),e));
    }
    let (t0,t1) = ts.iter().fold((f64::INFINITY,f64::NEG_INFINITY),|(l,h),&t| (l.min(t),h.max(t)));
    (t1-t0 > eps).then(|| (add(x0,scale(e,t0)),add(x0,scale(e,t1))))
}

/// **A cylinder's or a cone's face about an axis leaning to the turn's, its contact turned** (#780
/// C3): on each ring at slant `v` over the face's span it touches where `A cos u + B sin u = C`
/// (`turned::Contact`), two curves along the face, each turned into a surface of revolution about
/// the turn's axis whose meridian is read exactly (`Cutter::Turned`), its sites the curve's points
/// on the face carried over the roll with the face's normal there (the turned surface's: it touches
/// the face there). An axis in one plane with the turn's touches along its two generators in that
/// plane, lines, each turned as any ruling is (`Twist::ruling`, a hyperboloid) — refused where it
/// crosses the turn's axis within the face, whose ring there touches all round. Refused by what it
/// is (`Unbuilt::Curved`) where a contact is no curve over the span (the two roots on a ring
/// meeting, or the curve reaching the axis) or its meridian folds back along the axis, which its
/// solid, closed down to the axis at both ends, cannot hold.
fn turned_contacts(tw: &Twist,cand: &mut Candidates,located: &Located,round: &Round,(f,r,sa,ca): (Frame,f64,f64,f64))
    -> Result<Vec<(usize,crate::turned::Turned)>,Unbuilt> {
    use crate::turned::{Contact,Jet,Turned};
    let (face,k,[lo,hi]) = (round.face,round.k,round.span);
    let (c,axis,eps) = (tw.c,tw.axis,tw.leg.eps);
    let a = unit(f.z);
    let carried = |x: V| -> Vec<(V,V)> {
        if located.face_place(k,x) == Place::Out { return vec![] }
        let n = outward(face,x);
        tw.leg.poses.iter().map(|m| (m.point(x),m.vector(n))).collect()
    };
    // an axis in one plane with the turn's: its generators in that plane, turned (`ruling`)
    let np = cross(a,axis);
    if dot(unit(np),sub(f.o,c)).abs() <= eps {
        // where it crosses the turn's axis, along its own, within the face: a ring there touches
        // all round
        if foot_on(f.o,a,c,axis).is_some_and(|v| v >= lo*ca-eps && v <= hi*ca+eps) {
            return Err(Unbuilt::Curved {at:f.o})
        }
        let w = unit(cross(np,a));
        for w in [w,scale(w,-1.)] {
            if let Some((cutter,sites)) = tw.ruling(located,round,dot(w,f.y).datan2(dot(w,f.x))) { cand.add(cutter,sites); }
        }
        return Ok(Vec::new())
    }
    let g = Frame::about(c,axis);
    // the meridian `PAST` past the face's span each way, so its ends stand clear of the face's own
    // at the end poses (where the face's rim, the meridian's end and the touch along the contact
    // would meet at one point), as a patch's under a screw do (#777 M5)
    let past = PAST*(hi-lo);
    let span = [lo-past,hi+past];
    let mut entered = Vec::new();
    for side in [1.,-1.] {
        let contact = Contact {o:g.local(f.o),a:g.dir_local(a),r,sa,ca,side};
        let turned = Turned::meridian(g,contact,span);
        let at = g.at(contact.point(Jet::at(0.5*(lo+hi))).0.map(|j| j.v));
        if !turned.defined(span) { return Err(Unbuilt::Curved {at}) }
        let sites: Vec<(V,V)> = (0..=SITES).flat_map(|i| {
            let x = g.at(contact.point(Jet::at(lo+(hi-lo)*within(i))).0.map(|j| j.v));
            carried(x)
        }).collect();
        if !sites.is_empty() { entered.push((cand.add(Cutter::Turned(turned),sites),turned)) }
    }
    Ok(entered)
}

/// **A leaning round face's touches with its own contact turned** (#780 C3, given by construction
/// since #810): the face (`surface`, the tool's face `k`) at each end pose `ends[j]`
/// (its candidate `at_ends[(k, j)]`) touches each surface of revolution its contact turns into
/// (`turned`: the candidates and their meridians) along that contact turned to the pose — the
/// curve `ssi`'s `own_contact` writes, given where it holds it whole, the face standing on one side
/// of the surface wherever both reach; where it does not, nothing is given and the two are traced.
fn own_contacts(surface: &Surface,k: usize,turned: &[(usize,crate::turned::Turned)],ends: &[&crate::trajectory::Pose],
    at_ends: &std::collections::BTreeMap<(usize,usize),usize>,tol: f64) -> Vec<Touch> {
    let mut out = Vec::new();
    for (j,pose) in ends.iter().enumerate() {
        let Some(&at) = at_ends.get(&(k,j)) else { continue };
        let posed = surface.moved(&crate::geom::Rigid {r:pose.r,t:pose.p});
        for &(i,t) in turned {
            if let Some(curves) = crate::ssi::own_contact(&posed,&Curve::Turned(std::sync::Arc::new(t)),tol) {
                out.push(Touch {cutters:[at,i],curves})
            }
        }
    }
    out
}

/// **The stretches of the roll `[a, b]` a ball's pipe is entered in** (#780 L1, X5), the ball of
/// radius `r` about `c` and its great circle square to the unit `n`, its centre's velocity under
/// `screw`. In the screw's helical coordinates `(ρ, ψ, ζ = z − κψ)` the screw is a shift in `ψ`, so
/// the pipe is the circle's points' helices, and two of its stretches `t` of a turn apart meet
/// where the circle holds two points at one `ρ` whose `ζ` differ by a whole number of turns' rise
/// (`geom::Pipe::returns`): `t` is then their `ψ` apart less those turns, at least `2π − 2w` (`w`
/// the circle's reach round the axis from its centre, `Pipe::spread`). A pipe whose roll is short of
/// that, or whose circle stands at no two such points, meets itself nowhere, and is entered whole,
/// as before; one that comes back onto itself, in stretches of at most half
/// a turn each, each meeting itself nowhere, which the arrangement meets pairwise like any two
/// candidates (`ssi`'s `pipes_meeting`: helices, and the circle where two abut). Refused, by what it
/// is, where the circle reaches the axis (`w` none: then two points less than a turn apart along
/// one helix may meet) or where two of its points stand on one helix less than a turn apart; and by
/// name (`Unbuilt::Folded`) where the ball's radius reaches the helix's radius of curvature and the
/// pipe folds (its fold locus, `(ρ² + κ²)/ρ`, the design's step after).
fn pipe_stretches(screw: &crate::trajectory::Screw,c: V,n: V,r: f64,[a,b]: [f64;2],eps: f64) -> Result<Vec<[f64;2]>,Unbuilt> {
    use std::f64::consts::{PI,TAU};
    let pipe = crate::geom::Pipe {carry:crate::geom::Carry::Screw {center:screw.origin,axis:unit(screw.axis),angle:screw.ratio,
        advance:screw.advance/TAU},frame:Frame::about(c,n),r};
    let refused = || Unbuilt::Curved {at:c};
    let (Some(rho),Some(crate::geom::Carry::Screw {advance:kappa,..})) = (pipe.offset(),pipe.radian()) else { return Err(refused()) };
    // the helix's radius of curvature, `(ρ² + κ²)/ρ`: a ball reaching it folds its pipe
    if !(r < (rho*rho+kappa*kappa)/rho.max(1e-300)) { return Err(Unbuilt::Folded {at:c}) }
    let Some(w) = pipe.spread() else { return Err(refused()) };
    let turned = (screw.ratio*(b-a)).abs();
    // nor may it meet itself within less than a turn, two of its circle's points at one `ρ` and
    // `ζ` (the circle's orbits crossing; past a thousandth of a radian, where only the circle's own
    // points stand, the helices crossing its plane all one way), within the roll
    let near = (2.*w).min(turned)+1e-3;
    if near > 2e-3 && pipe.returns([1e-3,near],eps).is_none_or(|r| !r.is_empty()) { return Err(refused()) }
    let back = TAU-2.*w;
    if turned < back-1e-9 { return Ok(vec![[a,b]]) }
    let returns: Vec<f64> = pipe.returns([back-1e-9,turned+1e-9],eps).ok_or_else(refused)?.into_iter().map(|[..,t]| t).collect();
    if returns.is_empty() { return Ok(vec![[a,b]]) }
    // the joints are the build's choice, not the sweep's: placed so that no joint stands within a
    // twentieth of a radian of a return's turn from an end or another joint, where the two
    // circles would cross one return's helix at one point (two stretches' joints a degenerate
    // vertex of three curves), which left a cell too thin to settle. Evenly spaced, those apart
    // are the spacing's multiples (the two ends' own distance, the whole roll, is no joint's)
    let clear = |n: usize| (1..n).all(|i| { let x = turned*i as f64/n as f64; returns.iter().all(|&t| (x-t.abs()).abs() > 0.05) });
    let fewest = (turned/PI).ceil().max(2.) as usize;
    let pieces = (fewest..fewest+16).find(|&n| clear(n)).ok_or_else(refused)?;
    Ok((0..pieces).map(|i| [a+(b-a)*i as f64/pieces as f64,a+(b-a)*(i+1) as f64/pieces as f64]).collect())
}
