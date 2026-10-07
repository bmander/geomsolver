//! **A ball rolled along a closed spine, as a field** (issue #66): the material between the ball
//! and the corner it rounds — outside the tube of radius `r` about the spine, inside the cone
//! between the two directions from the spine to where the ball touches the faces, within the
//! corner's reach — read from the spine and the contact curves, never from the fitted canal face,
//! so agreement holds that face to what it should be. A fillet's piece is this less (or within)
//! the faces' material (`document.rs`).
//!
//! Its enclosure over a box is by distances, which are one-Lipschitz, and the cone's Lipschitz
//! bound near the spine: every constant is read from the curves' poles (convex hulls of spans and
//! of their hodographs), none sampled, and a corner reaching past the spine's unique-foot
//! neighbourhood (its curvature, its distance from itself) is refused.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::{Error,I,V};
use crate::brep::nurbs::BSpline;
use crate::bvh::{Bounds,Bvh};
use crate::space::{dot,norm,scale,sub};
use std::ops::Range;

type P = [f64;3];

/// The hodograph of a cubic's span: the poles of its derivative over the span ending at knot
/// index `j` (`knots[j] < knots[j + 1]`), and those of its second derivative.
fn hodographs(c: &BSpline,j: usize) -> (Vec<P>,Vec<P>) {
    let (p,t,q) = (c.degree,&c.knots,&c.poles);
    let d1: Vec<P> = (j-p..j).map(|i| scale(sub(q[i+1],q[i]),p as f64/(t[i+p+1]-t[i+1]))).collect();
    let d2: Vec<P> = (0..d1.len()-1).map(|i| {
        let k = j-p+i;
        scale(sub(d1[i+1],d1[i]),(p-1) as f64/(t[k+p+1]-t[k+2]))
    }).collect();
    (d1,d2)
}

/// The spans of a curve: the knot index ending each.
fn spans(c: &BSpline) -> Vec<usize> {
    (c.degree..c.poles.len()).filter(|&j| c.knots[j+1] > c.knots[j]).collect()
}

/// The box of a set of points.
fn hull(pts: &[P]) -> Bounds<3> {
    let mut b = Bounds {lo:[f64::INFINITY;3],hi:[f64::NEG_INFINITY;3]};
    for p in pts { for k in 0..3 { b.lo[k] = b.lo[k].min(p[k]); b.hi[k] = b.hi[k].max(p[k]); } }
    b
}

/// The largest of the points' lengths.
fn longest(pts: &[P]) -> f64 { pts.iter().map(|&q| norm(q)).fold(0.,f64::max) }

/// The canal leaf. Lengths in model units.
#[derive(Clone,Debug)]
pub struct CanalField {
    spine: BSpline,
    contacts: [BSpline;2],
    r: f64,
    reach: f64,
    /// The spine's spans' pole boxes, for a lower bound on the distance to it and to prune the
    /// samples a foot starts from.
    boxes: Bvh<3>,
    span_boxes: Vec<Bounds<3>>,
    /// Points of the spine to start a foot from, in order along it, and those within each span.
    samples: Vec<(f64,P)>,
    in_span: Vec<Range<usize>>,
    /// Bounds: the spine's curvature, and how fast the cone's sides turn per unit length of it.
    kappa: f64,
    turn: f64,
    /// Within this distance of the spine a point's nearest point on it is unique.
    near: f64,
    support: [f64;6],
    /// Whether the spine is open: its foot then held to its ends, the leaf left to be bounded
    /// past them (a run's trims, which its ends lie beyond by more than the reach).
    open: bool,
}

impl CanalField {
    /// The leaf over a spine and the two contact curves (each over the same parameter as the
    /// spine), the ball's radius and the corner's reach from the spine — the spine closed, or open
    /// and the leaf to be bounded short of its ends; `Err` where the corner reaches past where the
    /// spine's foot is unique.
    pub fn new(spine: BSpline,contacts: [BSpline;2],r: f64,reach: f64,closed: bool) -> Result<Self,String> {
        if spine.degree < 2 { return Err("a canal's spine is at least quadratic".into()) }
        let ks = spans(&spine);
        let span_boxes: Vec<Bounds<3>> = ks.iter().map(|&j| hull(&spine.poles[j-spine.degree..=j])).collect();
        // speed from below and above and acceleration from above, span by span, by the
        // hodographs' hulls
        let origin = Bounds {lo:[0.;3],hi:[0.;3]};
        let (mut slow,mut fast,mut kappa) = (f64::INFINITY,0_f64,0_f64);
        for &j in &ks {
            let (d1,d2) = hodographs(&spine,j);
            let speed = hull(&d1).gap(origin);
            if !(speed > 0.) { return Err("a canal's spine turns too fast for its spans to bound".into()) }
            slow = slow.min(speed);
            fast = fast.max(longest(&d1));
            kappa = kappa.max(longest(&d2)/(speed*speed));
        }
        // the contacts' rates, and the least angle between the directions to them
        let rate = |c: &BSpline| spans(c).iter().map(|&j| longest(&hodographs(c,j).0)).fold(0.,f64::max);
        let e_rate = (rate(&contacts[0]).max(rate(&contacts[1]))+fast)/r;
        let [u0,u1] = spine.domain();
        let period = u1-u0;
        let n = 1024;
        let mut least = f64::INFINITY;
        let mut samples = Vec::with_capacity(n+1);
        for k in 0..n+usize::from(!closed) {
            let u = u0+period*k as f64/n as f64;
            let c = spine.point(u);
            samples.push((u,c));
            let e = [0,1].map(|i| scale(sub(contacts[i].point(u),c),1./r));
            least = least.min(dot(e[0],e[1]).clamp(-1.,1.).dacos());
        }
        // the angle between them changes no faster than both turn
        let least = least-2.*e_rate*period/n as f64;
        if !(least > 0.) { return Err("a canal's contacts come together".into()) }
        let s = least.dsin();
        let turn = (4.*e_rate/s+2.*e_rate/(s*s))/slow;
        let reach_in = |i: usize| [spine.knots[ks[i]],spine.knots[ks[i]+1]];
        // the samples within each span (both in order along the spine)
        let in_span: Vec<Range<usize>> = (0..ks.len()).map(|i| {
            let [a,z] = reach_in(i);
            let from = samples.partition_point(|&(u,_)| u < a);
            from..from.max(samples.partition_point(|&(u,_)| u < z))
        }).collect();
        // where the foot is unique: within the radius of curvature, and half the distance between
        // stretches of the spine more than half a turn apart along it — nearer than that, its
        // curvature alone keeps it from coming back to itself (a chord over arc `s` is at least
        // `2 sin(κ s / 2) / κ`); farther, their spans' hulls are measured. Two spans are nearer
        // than half a turn when their far ends are, the long way round or the short
        let half_turn = std::f64::consts::PI/kappa;
        let mut gap = f64::INFINITY;
        for i in 0..ks.len() { for j in i+1..ks.len() {
            let ([a0,a1],[b0,b1]) = (reach_in(i),reach_in(j));
            let apart = if closed { (b1-a0).min(a1+period-b0) } else { b1-a0 };
            if fast*apart < half_turn { continue }
            gap = gap.min(span_boxes[i].gap(span_boxes[j]));
        } }
        let near = (1./kappa).min(0.5*gap);
        if !(reach < 0.9*near) {
            return Err(format!("the ball reaches {reach:.4} from its centre's path, past the {near:.4} where that path's \
                nearest point is unique: too large for the meeting it rolls along"));
        }
        let all = span_boxes.iter().fold(span_boxes[0],|m,&b| Bounds {lo:std::array::from_fn(|k| m.lo[k].min(b.lo[k])),
            hi:std::array::from_fn(|k| m.hi[k].max(b.hi[k]))});
        let support = [all.lo[0]-reach,all.hi[0]+reach,all.lo[1]-reach,all.hi[1]+reach,all.lo[2]-reach,all.hi[2]+reach];
        let boxes = Bvh::new(span_boxes.iter().copied());
        Ok(CanalField {spine,contacts,r,reach,boxes,span_boxes,samples,in_span,kappa,turn,near,support,open:!closed})
    }

    /// The parameter and point of the spine nearest `x`: from the nearest sample (among those of
    /// spans whose boxes could hold a nearer one), Newton on the foot condition, the parameter kept
    /// round the loop.
    fn foot(&self,x: P) -> (f64,P) {
        let best = std::cell::Cell::new((f64::INFINITY,0.));
        let at = Bounds {lo:x,hi:x};
        self.boxes.query_nearest(|b| b.gap(at),|| best.get().0.sqrt(),|i| {
            for &(u,c) in &self.samples[self.in_span[i].clone()] {
                let d = dot(sub(c,x),sub(c,x));
                if d < best.get().0 { best.set((d,u)) }
            }
        });
        let mut u = best.get().1;
        let [u0,u1] = self.spine.domain();
        let period = u1-u0;
        for _ in 0..12 {
            let (c,d1,d2) = self.spine.d2(u);
            let w = sub(c,x);
            let (g,h) = (dot(w,d1),dot(d1,d1)+dot(w,d2));
            if h <= 0. { return (u,c) }
            let step = g/h;
            if step.abs() < 1e-15*period { return (u,c) }
            u = if self.open { (u-step).clamp(u0,u1) } else { u0+(u-step-u0).rem_euclid(period) };
        }
        (u,self.spine.point(u))
    }

    /// The cone's term at `x` with its foot: the signed distance into the cone between the two
    /// directions to the contacts, in the normal plane.
    fn cone(&self,x: P,u: f64,c: P) -> f64 {
        let e = [0,1].map(|i| { let d = sub(self.contacts[i].point(u),c); scale(d,1./norm(d)) });
        let cos = dot(e[0],e[1]);
        let side = |a: P,b: P| { let n = sub(b,scale(a,cos)); scale(n,1./norm(n)) };
        let d = sub(x,c);
        (-dot(d,side(e[0],e[1]))).max(-dot(d,side(e[1],e[0])))
    }

    /// The field at a point.
    pub fn value(&self,x: [f64;3]) -> f64 {
        let (u,c) = self.foot(x);
        let rho = norm(sub(x,c));
        (self.r-rho).max(self.cone(x,u,c)).max(rho-self.reach)
    }

    /// An enclosure of the field over the box `p`.
    pub fn bounds(&self,p: V) -> Result<I,Error> {
        let (centre,diagonal) = crate::space::box_centre_diagonal(&p);
        let half = 0.5*diagonal;
        let pbox = Bounds {lo:p.map(|i| i.bounds()[0]),hi:p.map(|i| i.bounds()[1])};
        // the distance to the spine: below by its spans' hulls, above by its foot from the centre
        let (u,c) = self.foot(centre);
        let rho_c = norm(sub(centre,c));
        let rho_hi = rho_c+half;
        let least = std::cell::Cell::new(f64::INFINITY);
        self.boxes.query_nearest(|b| b.gap(pbox),|| least.get(),|i| least.set(least.get().min(self.span_boxes[i].gap(pbox))));
        let hulls = least.get();
        // within the neighbourhood where the foot is unique, the foot is the nearest point and the
        // distance one-Lipschitz about the centre's
        let rho_lo = if rho_hi < self.near { hulls.max(rho_c-half) } else { hulls };
        let rho = I::new(rho_lo,rho_hi.max(rho_lo))?;
        let r = I::point(self.r)?;
        let tube = r.sub(rho)?;
        let reach = rho.sub(I::point(self.reach)?)?;
        // the cone's term is no farther than the spine either way; near it, Lipschitz
        let far = I::new(-rho_hi.max(rho_lo),rho_hi.max(rho_lo))?;
        let cone = if rho_hi < self.near {
            let l = (1.+rho_hi*self.turn)/(1.-rho_hi*self.kappa);
            let v = self.cone(centre,u,c);
            let lo = (v-l*half).max(far.bounds()[0]);
            let hi = (v+l*half).min(far.bounds()[1]);
            I::new(lo.min(hi),hi.max(lo))?
        } else { far };
        Ok(super::max(super::max(tube,cone),reach))
    }

    /// A box enclosing its material: the spine's hull grown by the reach.
    pub fn support_bounds(&self) -> Result<Option<V>,Error> {
        let s = self.support;
        Ok(Some([I::new(s[0],s[1])?,I::new(s[2],s[3])?,I::new(s[4],s[5])?]))
    }
}
