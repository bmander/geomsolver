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

type P = [f64;3];

fn sub(a: P,b: P) -> P { [a[0]-b[0],a[1]-b[1],a[2]-b[2]] }
fn dot(a: P,b: P) -> f64 { a[0]*b[0]+a[1]*b[1]+a[2]*b[2] }
fn norm(a: P) -> f64 { dot(a,a).sqrt() }
fn scale(a: P,k: f64) -> P { [a[0]*k,a[1]*k,a[2]*k] }

/// The distance between two axis-aligned boxes (zero where they meet).
fn box_gap(a: &(P,P),b: &(P,P)) -> f64 {
    let gap = |k: usize| (a.0[k]-b.1[k]).max(b.0[k]-a.1[k]).max(0.);
    (gap(0).powi(2)+gap(1).powi(2)+gap(2).powi(2)).sqrt()
}

fn hull(pts: &[P]) -> (P,P) {
    let mut lo = [f64::INFINITY;3]; let mut hi = [f64::NEG_INFINITY;3];
    for p in pts { for k in 0..3 { lo[k] = lo[k].min(p[k]); hi[k] = hi[k].max(p[k]); } }
    (lo,hi)
}

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

/// The spans of a curve: each the knot index ending it and its poles' box.
fn spans(c: &BSpline) -> Vec<usize> {
    (c.degree..c.poles.len()).filter(|&j| c.knots[j+1] > c.knots[j]).collect()
}

/// The canal leaf. Lengths in model units.
#[derive(Clone,Debug)]
pub struct CanalField {
    spine: BSpline,
    contacts: [BSpline;2],
    r: f64,
    reach: f64,
    /// The spine's spans' pole boxes, for a lower bound on the distance to it.
    boxes: Vec<(P,P)>,
    /// Points of the spine to start a foot from.
    samples: Vec<(f64,P)>,
    /// Bounds: the spine's curvature, and how fast the cone's sides turn per unit length of it.
    kappa: f64,
    turn: f64,
    /// Within this distance of the spine a point's nearest point on it is unique.
    near: f64,
}

impl CanalField {
    /// The leaf over a closed spine and the two contact curves (each over the same parameter as
    /// the spine), the ball's radius and the corner's reach from the spine; `Err` where the
    /// corner reaches past where the spine's foot is unique.
    pub fn new(spine: BSpline,contacts: [BSpline;2],r: f64,reach: f64) -> Result<Self,String> {
        if spine.degree < 2 { return Err("a canal's spine is at least quadratic".into()) }
        let ks = spans(&spine);
        let boxes: Vec<(P,P)> = ks.iter().map(|&j| hull(&spine.poles[j-spine.degree..=j])).collect();
        // speed from below and acceleration from above, span by span, by the hodographs' hulls
        let origin = ([0.;3],[0.;3]);
        let (mut slow,mut kappa) = (f64::INFINITY,0_f64);
        for &j in &ks {
            let (d1,d2) = hodographs(&spine,j);
            let speed = box_gap(&hull(&d1),&origin);
            if !(speed > 0.) { return Err("a canal's spine turns too fast for its spans to bound".into()) }
            slow = slow.min(speed);
            let accel = d2.iter().map(|&q| norm(q)).fold(0.,f64::max);
            kappa = kappa.max(accel/(speed*speed));
        }
        // the contacts' rates, and the least angle between the directions to them
        let rate = |c: &BSpline| spans(c).iter().flat_map(|&j| hodographs(c,j).0).map(norm).fold(0.,f64::max);
        let spine_rate = spans(&spine).iter().flat_map(|&j| hodographs(&spine,j).0).map(norm).fold(0.,f64::max);
        let e_rate = (rate(&contacts[0]).max(rate(&contacts[1]))+spine_rate)/r;
        let [u0,u1] = spine.domain();
        let n = 1024;
        let mut least = f64::INFINITY;
        let mut samples = Vec::with_capacity(n);
        for k in 0..n {
            let u = u0+(u1-u0)*k as f64/n as f64;
            let c = spine.point(u);
            samples.push((u,c));
            let e = [0,1].map(|i| scale(sub(contacts[i].point(u),c),1./r));
            least = least.min(dot(e[0],e[1]).clamp(-1.,1.).dacos());
        }
        // the angle between them changes no faster than both turn
        let least = least-2.*e_rate*(u1-u0)/n as f64;
        if !(least > 0.) { return Err("a canal's contacts come together".into()) }
        let s = least.dsin();
        let turn = (4.*e_rate/s+2.*e_rate/(s*s))/slow;
        // where the foot is unique: within the radius of curvature, and half the distance between
        // stretches of the spine more than half a turn apart along it — nearer than that, its
        // curvature alone keeps it from coming back to itself (a chord over arc `s` is at least
        // `2 sin(κ s / 2) / κ`); farther, their spans' hulls are measured
        let m = boxes.len();
        let reach_in = |i: usize| [spine.knots[ks[i]],spine.knots[ks[i]+1]];
        let [u0,u1] = spine.domain();
        let period = u1-u0;
        let half_turn = std::f64::consts::PI/kappa;
        let mut gap = f64::INFINITY;
        for i in 0..m { for j in i+1..m {
            let ([a0,a1],[b0,b1]) = (reach_in(i),reach_in(j));
            let along = (b0-a1).max(0.).min((a0+period-b1).max(0.));
            if slow*along < half_turn { continue }
            gap = gap.min(box_gap(&boxes[i],&boxes[j]));
        } }
        let near = (1./kappa).min(0.5*gap);
        if !(reach < 0.9*near) {
            return Err(format!("the ball reaches {reach:.4} from its centre's path, past the {near:.4} where that path's \
                nearest point is unique: too large for the meeting it rolls along"));
        }
        Ok(CanalField {spine,contacts,r,reach,boxes,samples,kappa,turn,near})
    }

    /// The parameter and point of the spine nearest `x`: from the nearest sample, Newton on the
    /// foot condition, the parameter kept round the loop.
    fn foot(&self,x: P) -> (f64,P) {
        let (mut u,_) = self.samples.iter().map(|&(u,c)| (u,dot(sub(c,x),sub(c,x))))
            .min_by(|a,b| a.1.total_cmp(&b.1)).unwrap();
        let [u0,u1] = self.spine.domain();
        let period = u1-u0;
        for _ in 0..12 {
            let (c,d1,d2) = self.spine.d2(u);
            let w = sub(c,x);
            let (g,h) = (dot(w,d1),dot(d1,d1)+dot(w,d2));
            if h <= 0. { break }
            let step = g/h;
            u = u0+(u-step-u0).rem_euclid(period);
            if step.abs() < 1e-15*period { break }
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
        let b = p.map(|i| i.bounds());
        let centre: P = std::array::from_fn(|k| 0.5*(b[k][0]+b[k][1]));
        let half = (0..3).map(|k| (0.5*(b[k][1]-b[k][0])).powi(2)).sum::<f64>().sqrt();
        let pbox = ([b[0][0],b[1][0],b[2][0]],[b[0][1],b[1][1],b[2][1]]);
        // the distance to the spine: below by its spans' hulls, above by its foot from the centre
        let hulls = self.boxes.iter().map(|h| box_gap(&pbox,h)).fold(f64::INFINITY,f64::min);
        let (u,c) = self.foot(centre);
        let rho_c = norm(sub(centre,c));
        let rho_hi = rho_c+half;
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
        let mut lo = [f64::INFINITY;3]; let mut hi = [f64::NEG_INFINITY;3];
        for (a,b) in &self.boxes { for k in 0..3 { lo[k] = lo[k].min(a[k]); hi[k] = hi[k].max(b[k]); } }
        let at = |k: usize| I::new(lo[k]-self.reach,hi[k]+self.reach);
        Ok(Some([at(0)?,at(1)?,at(2)?]))
    }
}
