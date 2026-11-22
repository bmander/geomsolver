//! **A frame carried along a curve** (`Along`, K2): the rigid motion that takes what is drawn at a
//! point of a curve, its **home**, along it, its frame square to the curve's tangent.  Two frames,
//! by how much of the curve's torsion the frame turns with:
//! - **none, the rotation-minimising frame**: its axes square to the tangent turn only as the
//!   tangent does, `ω = T × T′ = C′ × C″ / |C′|²`, so along a curve in a plane its axis square to
//!   the plane stays put, and a tool drawn upright is carried upright.  Integrated by double
//!   reflection (Wang, Jüttler, Zheng and Liu, 2008, `reflected`) over a grid laid from the home,
//!   each node one step from the last and any other parameter one step from its nearest node:
//!   fourth order, its error at the grid's step a part in 10¹¹ of a turn; along a curve in a plane
//!   each reflection keeps the plane's normal, so the frame is the tangent, the normal and their
//!   cross product exactly but for rounding.
//! - **all of it, the curve's own (Frenet) frame**: the tangent, the principal normal toward the
//!   centre of curvature and the binormal, read off the derivatives directly; it turns with the
//!   torsion besides, `ω = T × T′ + τ |C′| T`.
//!
//! The motion is `M(u) x = R(u) (x − c(h)) + c(u)`, `R(u) = F(u) F(h)ᵀ` the turn from the frame at
//! the home `h` to the one at `u`, so the choice of the frame's axes at the home (square to the
//! tangent) moves nothing.  The curve is anything that reads its point and first three derivatives
//! (`Guide`): the kernel's B-spline, or a caller's own path.  Enclosed over any stretch of the
//! parameter (`Path::stretch`, what a sweep's speed bound reads) where the curve bounds its first
//! two derivatives (`Guide::derivative_bounds`: a polynomial B-spline's hodographs) and the frame
//! is the rotation-minimising one — and failing closed otherwise.
use crate::nurbs::BSpline;
use crate::util::space::{across,add,cross,dot,norm,normalised,scale,sub};
use crate::{Kind,KernelError};
use super::{mm,mv,reflected,transpose,Extremes,Motion,Path,Pose,Stretch,M3,V3};

/// How many steps of the grid the rotation-minimising frame is integrated over, end to end.
const GRID: usize = 4096;

/// How many samples a stretch is enclosed from (`Along::stretch`).
const SAMPLES: usize = 64;

/// A frame as three unit vectors: the tangent and two directions square to it.
type Frame = [V3;3];

/// **A curve a frame is carried along**: where its parameter runs, its point and first three
/// derivatives at a parameter (the third only where `third` asks for it, zero otherwise), and
/// bounds on `|C′|` and `|C″|` over the whole of it where they are known for certain.
pub trait Guide: Send+Sync+std::fmt::Debug {
    fn domain(&self) -> [f64;2];
    fn jet(&self,u: f64,third: bool) -> Result<[V3;4],KernelError>;
    fn derivative_bounds(&self) -> Option<[f64;2]>;
    /// The curve as the kernel's B-spline, where it is one: what a dump writes of it (#942
    /// T1d-ring); none for a caller's own path, read only by evaluating it.
    fn spline(&self) -> Option<&BSpline> { None }
}

impl Guide for BSpline {
    fn domain(&self) -> [f64;2] { BSpline::domain(self) }
    /// The third derivative off the hodograph's second.
    fn jet(&self,u: f64,third: bool) -> Result<[V3;4],KernelError> {
        let (x,d,dd) = self.d2(u);
        let ddd = if third {
            self.hodograph().ok_or("a rational or constant curve's third derivative is not read")?.d2(u).2
        } else { [0.;3] };
        Ok([x,d,dd,ddd])
    }
    fn derivative_bounds(&self) -> Option<[f64;2]> { BSpline::derivative_bounds(self) }
    fn spline(&self) -> Option<&BSpline> { Some(self) }
}

/// **A frame carried along a curve** (the module's): the curve, the frame's name in what it says,
/// whether it is the curve's own, its home, the curve's point and the frame there, the
/// rotation-minimising frame at the grid's nodes out from the home each way, bounds on the curve's
/// first two derivatives, and the whole curve's stretch.
#[derive(Debug)]
pub struct Along<G: Guide = BSpline> {
    guide: G,
    name: String,
    frenet: bool,
    home: f64,
    origin: V3,
    home_frame: Frame,
    /// The curve's point and the frame at the nodes from the home down to the domain's start, and
    /// up to its end, each evenly spaced and beginning at the home.
    down: Vec<(V3,Frame)>,
    up: Vec<(V3,Frame)>,
    bounds: Option<[f64;2]>,
    whole: Option<Stretch>,
}

impl Along<BSpline> {
    /// **The rotation-minimising frame carried along `curve` from its start.**
    pub fn new(curve: BSpline) -> Result<Along,KernelError> {
        let home = curve.domain()[0];
        Along::from(curve,home,false,"the frame")
    }
}

impl<G: Guide> Along<G> {
    /// **The frame carried along `guide` from `home`**, the curve's own (`frenet`) or the
    /// rotation-minimising one, called `name` in what it says: refused where the curve runs over
    /// no interval or `home` is off it, or (rotation-minimising) where it has no tangent somewhere
    /// on the grid, or (its own) no curvature at the home.
    pub fn from(guide: G,home: f64,frenet: bool,name: impl Into<String>) -> Result<Along<G>,KernelError> {
        let name = name.into();
        let [lo,hi] = guide.domain();
        if !(lo < hi) { return Err(KernelError::new(Kind::Failed,format!("{name}'s path runs over no interval"))) }
        if !(home >= lo && home <= hi) { return Err(KernelError::new(Kind::Failed,format!("{name}'s home {home} is off its path's ({lo}, {hi})"))) }
        let at = guide.jet(home,false)?;
        let mut along = Along {bounds:guide.derivative_bounds(),guide,name,frenet,home,origin:at[0],home_frame:[[0.;3];3],
            down:Vec::new(),up:Vec::new(),whole:None};
        along.home_frame = if frenet { along.frenet(home,at)? } else {
            let t = along.tangent(home,at[1])?;
            let (u,v) = across(t);
            [t,u,v]
        };
        if !frenet {
            // each way from the home, steps of at most a `GRID`th of the domain
            let step = (hi-lo)/GRID as f64;
            let run = |to: f64| -> Result<Vec<(V3,Frame)>,KernelError> {
                let n = ((to-home).abs()/step).ceil() as usize;
                let mut out = vec![(along.origin,along.home_frame)];
                for k in 1..=n {
                    let u = if k == n { to } else { home+(to-home)*k as f64/n as f64 };
                    let x = along.guide.jet(u,false)?;
                    let (x0,f0) = out[k-1];
                    out.push((x[0],reflected(&f0,x0,x[0],along.tangent(u,x[1])?)));
                }
                Ok(out)
            };
            (along.down,along.up) = (run(lo)?,run(hi)?);
            along.whole = along.enclosed(lo,hi);
        }
        Ok(along)
    }

    /// The curve the frame runs along.
    pub fn guide(&self) -> &G { &self.guide }

    /// The unit tangent where the curve's derivative is `d`, or why there is none.
    fn tangent(&self,u: f64,d: V3) -> Result<V3,KernelError> {
        normalised(d).ok_or_else(|| KernelError::new(Kind::Failed,format!("{}'s path has no tangent at {u}",self.name)))
    }

    /// The curve's own frame at `u`, off its jet `x`, or why it has none.
    fn frenet(&self,u: f64,x: [V3;4]) -> Result<Frame,KernelError> {
        let t = self.tangent(u,x[1])?;
        let b = normalised(cross(x[1],x[2])).ok_or_else(|| KernelError::new(Kind::Failed,format!("{}'s path has no curvature at {u}, \
            so no frame of its own there",self.name)))?;
        Ok([t,cross(b,t),b])
    }

    /// The turn at `u` from the home, `F(u) F(h)ᵀ`, off the curve's jet `x` there: the curve's own
    /// frame, or the rotation-minimising one a step from its nearest node.
    fn turn(&self,u: f64,x: [V3;4]) -> Result<M3,KernelError> {
        let [lo,hi] = self.guide.domain();
        if !(u >= lo-1e-12*(hi-lo) && u <= hi+1e-12*(hi-lo)) {
            return Err(KernelError::new(Kind::Failed,format!("{} runs along its path over ({lo}, {hi}), and {u} is past it",self.name)))
        }
        let f = if self.frenet { self.frenet(u,x)? } else {
            let (nodes,end) = if u >= self.home { (&self.up,hi) } else { (&self.down,lo) };
            let n = nodes.len()-1;
            let k = if n == 0 { 0 } else { ((u-self.home)/(end-self.home)*n as f64).round().clamp(0.,n as f64) as usize };
            let (x0,f0) = nodes[k];
            reflected(&f0,x0,x[0],self.tangent(u,x[1])?)
        };
        Ok(mm(transpose(f),self.home_frame))
    }

    /// `stretch` over `[a, b]` within the domain, read from samples a step `w` apart: every point
    /// within `|C′| w / 2` of one, every entry of the turn within `|ω| w / 2` of one's, and
    /// `|ω| ≤ |C″| / |C′|` with `|C′|` no less than the slowest sample less `|C″| w / 2`.
    fn enclosed(&self,a: f64,b: f64) -> Option<Stretch> {
        let [d1,d2] = self.bounds?;
        let w = (b-a)/SAMPLES as f64;
        let mut seen = Extremes::new();
        let mut slow = f64::INFINITY;
        for j in 0..=SAMPLES {
            let u = if j == SAMPLES { b } else { a+w*j as f64 };
            let x = self.guide.jet(u,false).ok()?;
            seen.turn(&self.turn(u,x).ok()?);
            seen.point(x[0]);
            slow = slow.min(norm(x[1]));
        }
        let slowest = slow-d2*w/2.;
        if !(slowest > 0.) { return None }
        let spin = d2/slowest;
        seen.enclosed(d1*w/2.,spin*w/2.,d1*(1.+1e-12),spin*(1.+1e-12),self.origin)
    }
}

impl<G: Guide> Path for Along<G> {
    /// The pose at `u` and its rate: `x ↦ R (x − c(h)) + c(u)`, turning at `ω = C′ × C″ / |C′|²`,
    /// and the curve's own frame with the torsion besides.
    fn at(&self,u: f64) -> Result<Motion,KernelError> {
        let x = self.guide.jet(u,self.frenet)?;
        let r = self.turn(u,x)?;
        let (d,dd) = (x[1],x[2]);
        let b = cross(d,dd);
        let mut w = scale(b,1./dot(d,d));
        if self.frenet { w = add(w,scale(d,dot(b,x[3])/dot(b,b))) }
        let dr = mm([[0.,-w[2],w[1]],[w[2],0.,-w[0]],[-w[1],w[0],0.]],r);
        Ok(Motion::of_parts(r,dr,sub(x[0],mv(r,self.origin)),sub(d,mv(dr,self.origin))))
    }

    fn pose(&self,u: f64) -> Result<Pose,KernelError> {
        let x = self.guide.jet(u,false)?;
        let r = self.turn(u,x)?;
        Ok(Pose {r,p:sub(x[0],mv(r,self.origin))})
    }

    /// **What the motion does over `[a, b]`, enclosed** (`enclosed`; the whole curve's read once).
    /// None for the curve's own frame, a curve with no bound on its derivatives, or where the curve
    /// may stop.
    fn stretch(&self,a: f64,b: f64) -> Option<Stretch> {
        if self.frenet || !(a <= b) { return None }
        let [lo,hi] = self.guide.domain();
        if a <= lo && b >= hi { return self.whole }
        self.enclosed(a.max(lo),b.min(hi))
    }

    fn along(&self) -> Option<(BSpline,f64,bool,String)> {
        Some((self.guide.spline()?.clone(),self.home,self.frenet,self.name.clone()))
    }
}
