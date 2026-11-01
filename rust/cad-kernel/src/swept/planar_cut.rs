//! **Where a point, line or circle cuts under a motion of its plane** (#721 F2c): the envelope of
//! a tool carried by a planar motion, read at one roll in closed form. The motion is a postfix
//! program of turns about centres and slides along directions, the roll `t` in degrees
//! (`PlanarStep`); it carries the tool to `X(s, t) = R(Φ(t))·T(s) + d(t)`, `s` the tool's own
//! parameter, and the tool cuts where its tangent runs along the velocity of the material point
//! under it, `F(s, t) = X_s × X_t = 0` — the normal through the instant centre. A point cuts at
//! itself; a line once, `F` being linear in `s`; a circle twice, on the line from its centre
//! through the instant centre, `side` saying which (`+1` the nearer). The cut's point `C(t)` and
//! its derivative along the roll (per degree), `C′ = X_s s′ + X_t` with `s′ = −F_t / F_s` (the
//! implicit function theorem), are exact for these tools: what an extruded cutter's sheet is
//! built from (`swept::extruded`).
#[allow(unused_imports)]
use crate::util::fmath::Det;

type P = [f64;2];

const DEG: f64 = std::f64::consts::PI/180.;
const NEWTON_MAX: usize = 40;

/// One step of a planar motion's postfix program (the roll `t` in degrees): a turn about `centre`
/// by `ratio` degrees a degree; a slide along the unit `direction` by `advance` a whole turn of the
/// roll; the relative of the two steps before it, the first seen from the second.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum PlanarStep {
    Turn { centre: P,ratio: f64 },
    Slide { direction: P,advance: f64 },
    Relative,
}

/// What cuts: a point, a line from `a` to `b` (its parameter the fraction along it), or a circle
/// about `centre` of `radius` (its parameter the angle from the plane's first axis), on the side
/// `side` says.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Cutting { Point(P),Line { a: P,b: P },Circle { centre: P,radius: f64,side: f64 } }

/// The tool's cut at one roll: its point, its derivative along the roll (per degree), and the
/// tool's parameter cutting there.
#[derive(Clone,Copy,Debug)]
pub struct Cut { pub c: P,pub dc: P,pub s: f64 }

/// A number and its first two derivatives in the roll.
#[derive(Clone,Copy,Debug,Default)]
struct J { v: f64,d: f64,dd: f64 }

impl J {
    fn constant(v: f64) -> J { J {v,d:0.,dd:0.} }
    fn add(self,o: J) -> J { J {v:self.v+o.v,d:self.d+o.d,dd:self.dd+o.dd} }
    fn sub(self,o: J) -> J { J {v:self.v-o.v,d:self.d-o.d,dd:self.dd-o.dd} }
    fn mul(self,o: J) -> J { J {v:self.v*o.v,d:self.d*o.v+self.v*o.d,dd:self.dd*o.v+2.*self.d*o.d+self.v*o.dd} }
    fn scale(self,k: f64) -> J { J {v:self.v*k,d:self.d*k,dd:self.dd*k} }
    fn neg(self) -> J { self.scale(-1.) }
    fn sin_cos(self) -> (J,J) {
        let (s,c) = self.v.dsin_cos();
        (J {v:s,d:c*self.d,dd:-s*self.d*self.d+c*self.dd},J {v:c,d:-s*self.d,dd:-c*self.d*self.d-s*self.dd})
    }
}

/// The motion at the roll `t`: its turn `Φ` and its offset `d`, `X = R(Φ)·T + d`, each with its
/// first two derivatives in the roll.
fn motion(program: &[PlanarStep],t: f64) -> Option<(J,[J;2])> {
    let roll = J {v:t,d:1.,dd:0.};
    let mut stack: Vec<(J,[J;2])> = Vec::with_capacity(program.len());
    for step in program {
        let m = match *step {
            PlanarStep::Turn {centre:[cx,cy],ratio} => {
                let phi = roll.scale(ratio*DEG);
                let (sn,cs) = phi.sin_cos();
                // about the centre: R(c' − c) + c, so d = c − R c
                (phi,[J::constant(cx).sub(cs.scale(cx).sub(sn.scale(cy))),J::constant(cy).sub(sn.scale(cx).add(cs.scale(cy)))])
            }
            PlanarStep::Slide {direction:[dx,dy],advance} => {
                let len = dx.dhypot(dy);
                if !(len > 0.) { return None }
                // `advance` per whole turn of the roll
                let k = advance/360./len;
                (J::constant(0.),[roll.scale(dx*k),roll.scale(dy*k)])
            }
            PlanarStep::Relative => {
                let (obs,src) = (stack.pop()?,stack.pop()?);
                // observer⁻¹ ∘ source: the source as the observer sees it
                let (sn,cs) = obs.0.neg().sin_cos();
                let (x,y) = (src.1[0].sub(obs.1[0]),src.1[1].sub(obs.1[1]));
                (src.0.sub(obs.0),[cs.mul(x).sub(sn.mul(y)),sn.mul(x).add(cs.mul(y))])
            }
        };
        stack.push(m);
    }
    (stack.len() == 1).then(|| stack[0])
}

fn cross(a: P,b: P) -> f64 { a[0]*b[1]-a[1]*b[0] }
/// A quarter turn counter-clockwise.
fn perp(a: P) -> P { [-a[1],a[0]] }

/// **The tool's cut at roll `t`** (degrees) under `program`: `None` where the program is no
/// motion, a line's or circle's root is not found (a line carried along itself, a slide with no
/// instant centre to aim a circle by and no velocity), or the root is singular.
pub fn cut_at(tool: Cutting,program: &[PlanarStep],t: f64) -> Option<Cut> {
    let (phi,d) = motion(program,t)?;
    let omega = phi.d;
    let (sn,cs) = (phi.v.dsin(),phi.v.dcos());
    let turn = |p: P| [cs*p[0]-sn*p[1],sn*p[0]+cs*p[1]];
    let (dv,d1,d2) = ([d[0].v,d[1].v],[d[0].d,d[1].d],[d[0].dd,d[1].dd]);
    // the tool's point, tangent and second derivative in `s`
    let tool_at = |s: f64| -> (P,P,P) {
        match tool {
            Cutting::Point(p) => (p,[0.;2],[0.;2]),
            Cutting::Line {a,b} => ([a[0]+s*(b[0]-a[0]),a[1]+s*(b[1]-a[1])],[b[0]-a[0],b[1]-a[1]],[0.;2]),
            Cutting::Circle {centre,radius,..} => {
                let (sn,cs) = s.dsin_cos();
                ([centre[0]+radius*cs,centre[1]+radius*sn],[-radius*sn,radius*cs],[-radius*cs,-radius*sn])
            }
        }
    };
    // X, X_s, X_t (the material point's velocity), and F with its derivatives in `s` and `t`
    let read = |s: f64| {
        let (p,tp,tpp) = tool_at(s);
        let (rp,rt,rtt) = (turn(p),turn(tp),turn(tpp));
        let x = [rp[0]+dv[0],rp[1]+dv[1]];
        let w = perp(rp);
        let xt = [omega*w[0]+d1[0],omega*w[1]+d1[1]];
        let f = cross(rt,xt);
        let f_s = cross(rtt,xt)+omega*(rt[0]*rt[0]+rt[1]*rt[1]);
        // X_s turns at ω J X_s; X_t changes at ω J (ω J R T) + d″ = −ω² R T + d″
        let vt = [-omega*omega*rp[0]+d2[0],-omega*omega*rp[1]+d2[1]];
        let f_t = omega*cross(perp(rt),xt)+cross(rt,vt);
        (x,rt,xt,f,f_s,f_t)
    };
    let s = match tool {
        Cutting::Point(_) => 0.,
        Cutting::Line {..} => newton(&read,0.)?,
        Cutting::Circle {centre,side,..} => {
            let p = turn(centre);
            let p = [p[0]+dv[0],p[1]+dv[1]];
            // toward the instant centre; for a slide, square to the velocity
            let w = if omega.abs() < 1e-15 { [-d1[1],d1[0]] } else { [dv[0]-d1[1]/omega-p[0],dv[1]+d1[0]/omega-p[1]] };
            let w = [w[0]*side,w[1]*side];
            // back into the tool's own placement
            newton(&read,(cs*w[1]-sn*w[0]).datan2(cs*w[0]+sn*w[1]))?
        }
    };
    let (x,xs,xt,_,f_s,f_t) = read(s);
    let ds = if matches!(tool,Cutting::Point(_)) { 0. } else {
        if !(f_s != 0.) { return None }
        -f_t/f_s
    };
    Some(Cut {c:x,dc:[xs[0]*ds+xt[0],xs[1]*ds+xt[1]],s})
}

/// Newton on `F(s) = 0` from `s`, as the cut's `read` gives `F` and `F_s`.
fn newton(read: &dyn Fn(f64) -> (P,P,P,f64,f64,f64),mut s: f64) -> Option<f64> {
    for _ in 0..NEWTON_MAX {
        let (_,_,_,r,rs,_) = read(s);
        if !(rs != 0.) || !r.is_finite() { return None }
        let step = r/rs;
        s -= step;
        if step.abs() <= 1e-14*(1.+s.abs()) { return s.is_finite().then_some(s) }
    }
    let (_,_,_,r,..) = read(s);
    (r.abs() <= 1e-10*(1.+s.abs())).then_some(s)
}
