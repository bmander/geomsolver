//! **The tube road: a torus's sweep as the offset of the surface its core circle sweeps** (#880
//! T1f). A torus is the set of points within its tube `r` of its core circle, so the swept solid
//! is the set of points within `r` of the surface `S(θ, t)` the core circle sweeps — the circle's
//! point at `θ` posed at the roll `t` — and its boundary lies on the offset of `S` at `r`:
//! * **two sheets**, `X±(θ, t) = S ± r N`, `N` the unit normal `S_θ × S_t` normalised: each one
//!   smooth map over the whole `(θ, t)` rectangle (θ periodic) wherever `S` is regular. They are
//!   the two contact branches of every instant, with no pieces: where the ring road (`rings`) reads
//!   a fold the sheet has a cuspidal edge in space — its Jacobian `J` (`Tube::jacobian`) nought,
//!   the map's rank dropping — but stays a smooth map of `(θ, t)`; on the far side of that edge
//!   (`J < 0`) the instants beside a point's own hold it (`depth::Reading::Near`), so only `J > 0`
//!   can bound the sweep, and the edge is no boundary but at isolated points (the swallowtail
//!   points, where the double curve meets it: `swallowtails`);
//! * where `S` is singular (a meridian touched whole for an instant: a pinch, `Ring::touched_once`)
//!   a collar chart replaces both sheets about it (`rings::Collar`; not yet entered here);
//! * the two end poses' tori.
//!
//! **A face is the part of a chart no other instant covers**: a point's exact depth is the least
//! over the roll of its distance to the posed core circle, less `r`; on a sheet the point's own
//! instant reads nought, so what is read is the least past its own contact (`depth::cover`), and the
//! trim curves are that depth's zero set on each sheet (`Depth`, a `zero_set::Field`) — double
//! curves, each point with its covering instant and the covering sheet's point, so both faces'
//! parameters come by construction. Nothing here is cut into pieces, stretches or bridges.
//!
//! (The peer kernel cadrs, MIT OR Apache-2.0, `crates/cadrs_kernel/src/native/sweep_tube.rs`, builds
//! these sweeps from the same charts: offset sheets, pipe and pinch charts trimmed by membership.
//! Its idea is taken, none of its code: its trims are marching squares on fixed grids; here every
//! curve is traced to the tolerance from events found exactly.)

pub mod cover;
pub mod faces;

use super::closed::Unbuilt;
use super::depth::{Cover,Reading};
use super::patches::Element;
use super::rings::{Collar,Ring};
use crate::trajectory::Trajectory;
use crate::util::fmath::{Det,HALF_READ,THIRD_READ};
use crate::util::space::{add,cross,dot,norm,scale,sub};
use crate::zero_set::{Field,Value};
use std::f64::consts::TAU;
use std::sync::atomic::{AtomicUsize,Ordering};

type V = [f64;3];

/// The certified covers the road has read (`Tube::cover`), counted for its profile.
static COVERS: AtomicUsize = AtomicUsize::new(0);

/// How many certified covers the road has read in this process.
pub fn covers_read() -> usize { COVERS.load(Ordering::Relaxed) }

/// **A torus under a motion, read as its core circle's sweep and its tube**: the ring (its frame,
/// for depth), the motion and its roll, the core circle's radius and height in the ring's frame,
/// and the tube.
pub struct Tube<'a> { pub ring: &'a Ring,pub path: &'a Trajectory,pub roll: [f64;2],pub radius: f64,pub height: f64,pub r: f64,
    pub rates: Option<Rates>,
    /// The pinch the collar charts, where the core circle's sweep is singular (`Tube::pinched`).
    pub pinch: Option<Pinch<'a>> }

/// **How fast the motion carries the core circle over the roll**, bounded: its rotation's spin
/// (`Trajectory::spin_bound`) and the first four rates of the circle's points
/// (`Trajectory::point_jet_bound_over`, forward over the circle's box). None where the motion
/// gives no such bound.
#[derive(Clone,Copy,Debug)]
pub struct Rates { pub spin: f64,pub jet: [f64;4] }

/// **The spine's point and its rates at `(θ, t)`**: `S`, `S_θ`, `S_t`, `S_θθ`, `S_θt`, `S_tt`.
#[derive(Clone,Copy,Debug)]
pub struct Spine { pub s: V,pub s_th: V,pub s_t: V,pub s_thth: V,pub s_tht: V,pub s_tt: V }

/// **A sheet's point and its two rates at `(θ, t)`**, its spine's unit normal, and the spine's
/// area factor `|S_θ × S_t|`.
#[derive(Clone,Copy,Debug)]
pub struct Point { pub x: V,pub x_th: V,pub x_t: V,pub n: V,pub area: f64 }

impl<'a> Tube<'a> {
    /// **The tool read as a tube**: a ring whose meridian is one whole circle (a torus), its core
    /// circle about the ring's axis. Refused by name otherwise.
    pub fn new(ring: &'a Ring,path: &'a Trajectory,roll: [f64;2]) -> Result<Tube<'a>,Unbuilt> {
        let [Element::Arc {c,r,sweep,..}] = ring.elements[..] else {
            return Err(Unbuilt::Failed("the tube road: a tool whose meridian is not one circle".into()))
        };
        // (a whole turn of the meridian: its sweep a full turn to the rounding of an angle)
        if (sweep.abs()-TAU).abs() > 16.*f64::EPSILON*TAU || !(c[0] > r) {
            return Err(Unbuilt::Failed("the tube road: a tool whose meridian is not one circle clear of its axis".into()))
        }
        let [a,b] = roll;
        let roll = [a.min(b),a.max(b)];
        // (the circle's box: its centre and, along each axis, its radius times its plane's reach)
        let f = &ring.frame;
        let centre = f.at([0.,0.,c[1]]);
        let rates = (|| {
            let domain = crate::util::interval::Interval::new(roll[0],roll[1]).ok()?;
            let boxed: [crate::util::interval::Interval;3] = std::array::from_fn(|k| {
                let reach = c[0]*f.x[k].dhypot(f.y[k]);
                crate::util::interval::Interval::new(centre[k]-reach,centre[k]+reach)
            }).map(|i| i.ok()).map(|i| i.unwrap_or(crate::util::interval::Interval::ZERO));
            Some(Rates {spin:path.spin_bound().ok()?,jet:path.point_jet_bound_over(boxed,domain,true).ok()?})
        })();
        Ok(Tube {ring,path,roll,radius:c[0],height:c[1],r,rates,pinch:None})
    }

    /// The core circle's point at `θ` in the tool's coordinates, and its first and second rates.
    fn circle(&self,theta: f64) -> [V;3] {
        let (s,c) = theta.dsin_cos();
        let (f,rr,h) = (&self.ring.frame,self.radius,self.height);
        [f.at([rr*c,rr*s,h]),f.dir([-rr*s,rr*c,0.]),f.dir([-rr*c,-rr*s,0.])]
    }

    /// **The spine at `(θ, t)`**: the motion's pose and rate read in closed form for `S`, `S_θ`,
    /// `S_t`, `S_θθ` and `S_θt`; `S_tt`, the rate's rate, by a central difference of `S_t` a cube
    /// root of the rounding of the roll apart (where its rounding and truncation balance).
    pub fn spine(&self,theta: f64,t: f64) -> Option<Spine> {
        let [c,c1,c2] = self.circle(theta);
        let m = self.path.at(t).ok()?;
        let o = m.velocity([0.;3]);
        let h = THIRD_READ*(self.roll[1]-self.roll[0]);
        let (later,earlier) = (self.path.at(t+h).ok()?.velocity(c),self.path.at(t-h).ok()?.velocity(c));
        Some(Spine {s:m.point(c),s_th:m.vector(c1),s_t:m.velocity(c),s_thth:m.vector(c2),s_tht:sub(m.velocity(c1),o),
            s_tt:scale(sub(later,earlier),0.5/h)})
    }

    /// **The sheet on `side` (±1) at `(θ, t)`**, `S + side · r N`, with its rates in closed form
    /// from the spine's: `N_x = (n_x − N (N · n_x)) / |n|`, `n = S_θ × S_t`. None where the spine is
    /// singular — its area factor within the half digits of its rates' product (a pinch, which the
    /// collar charts).
    pub fn point(&self,side: f64,theta: f64,t: f64) -> Option<Point> {
        let sp = self.spine(theta,t)?;
        let n = cross(sp.s_th,sp.s_t);
        let area = norm(n);
        if !(area > HALF_READ*norm(sp.s_th)*norm(sp.s_t)) { return None }
        let u = scale(n,1./area);
        let n_th = add(cross(sp.s_thth,sp.s_t),cross(sp.s_th,sp.s_tht));
        let n_t = add(cross(sp.s_tht,sp.s_t),cross(sp.s_th,sp.s_tt));
        let rate = |nx: V| scale(sub(nx,scale(u,dot(u,nx))),1./area);
        let k = side*self.r;
        Some(Point {x:add(sp.s,scale(u,k)),x_th:add(sp.s_th,scale(rate(n_th),k)),x_t:add(sp.s_t,scale(rate(n_t),k)),n:u,area})
    }

    /// **The sheet's Jacobian at `(θ, t)`** against its spine's: `(X_θ × X_t) · N / |S_θ × S_t|`,
    /// `1 − 2 r H + r² K` with the spine's curvatures read on the side — one where the tube is
    /// thin, nought on the cuspidal edge, below nought beyond it (where the sheet runs backwards
    /// and the neighbouring instants hold it).
    pub fn jacobian(&self,side: f64,theta: f64,t: f64) -> Option<f64> {
        let p = self.point(side,theta,t)?;
        Some(dot(cross(p.x_th,p.x_t),p.n)/p.area)
    }

    /// The direction in `(θ, t)` along which the sheet's rates cancel (its kernel, where they are
    /// parallel), unit in the parameters: `(|X_t|², −X_θ · X_t)` or `(−X_θ · X_t, |X_θ|²)`,
    /// whichever is longer.
    pub fn kernel(&self,side: f64,theta: f64,t: f64) -> Option<[f64;2]> {
        let p = self.point(side,theta,t)?;
        let (a,b,c) = (dot(p.x_th,p.x_th),dot(p.x_th,p.x_t),dot(p.x_t,p.x_t));
        let k = if c >= a { [c,-b] } else { [-b,a] };
        let l = k[0].dhypot(k[1]);
        (l > 0.).then(|| [k[0]/l,k[1]/l])
    }

    /// The Jacobian's gradient in `(θ, t)`, by central differences a cube root of the rounding of
    /// an angle and of the roll apart.
    pub fn jacobian_gradient(&self,side: f64,theta: f64,t: f64) -> Option<[f64;2]> {
        let (h,ht) = (THIRD_READ,THIRD_READ*(self.roll[1]-self.roll[0]));
        let j = |a: f64,b: f64| self.jacobian(side,a,b);
        Some([(j(theta+h,t)?-j(theta-h,t)?)/(2.*h),(j(theta,t+ht)?-j(theta,t-ht)?)/(2.*ht)])
    }

    /// **The tool's depth at the point `x` at the roll `τ`** in closed form — its distance from the
    /// posed core circle less the tube — and that distance's gradient in space (the unit direction
    /// from the circle's nearest point). None on the core circle's axis, where no point is nearest.
    pub fn depth_at(&self,x: V,tau: f64) -> Option<(f64,V)> {
        let pose = self.path.pose(tau).ok()?;
        let q = pose.inverse().point(x);
        let l = self.ring.frame.local(q);
        let rho = l[0].dhypot(l[1]);
        if !(rho > 0.) { return None }
        let near = [self.radius*l[0]/rho,self.radius*l[1]/rho,self.height];
        let d = sub(l,near);
        let dist = norm(d);
        if !(dist > 0.) { return None }
        Some((dist-self.r,pose.vector(self.ring.frame.dir(scale(d,1./dist)))))
    }

    /// **How the sheet's point at `(θ, t)` stands under the sweep** (`depth::cover`, the certified
    /// march, to `within`).
    pub fn reading(&self,side: f64,theta: f64,t: f64,within: f64) -> Option<Reading> {
        let p = self.point(side,theta,t)?;
        self.cover(p.x,t,within)
    }

    /// **How the point `x`, its own instant `own`, stands under the sweep**, to `within`.
    pub fn cover(&self,x: V,own: f64,within: f64) -> Option<Reading> {
        COVERS.fetch_add(1,Ordering::Relaxed);
        if let Some(r) = self.certified(x,own,within) { return Some(r) }
        super::depth::cover(self.ring,self.path,self.roll,x,own,0.,within).ok()
    }
}

/// **A chart of the tube road**: a sheet `X±` over `(θ, t)` (`Sheet(±1)`), the tool posed at an
/// end of the roll over `(θ, ψ)` (`End(0)` the start, `End(1)` the end), `ψ` round its tube from
/// the outward way about the ring's axis toward its own, or the collar about the pinch over
/// `(ψ, ρ)` (`Collar`, `Pinch`).
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Chart { Sheet(f64),End(usize),Collar }

impl Tube<'_> {
    /// The chart's rectangle of parameters.
    pub fn rect(&self,c: Chart) -> crate::zero_set::Rect {
        match c {
            Chart::Sheet(_) => [[0.,TAU],self.roll],Chart::End(_) => [[0.,TAU],[0.,TAU]],
            Chart::Collar => { let r = self.pinch.as_ref().map_or(0.,|p| p.rho); [[0.,TAU],[-r,r]] }
        }
    }

    /// The instant a chart's point is its own at.
    pub fn own(&self,c: Chart,uv: [f64;2]) -> f64 {
        match c {
            Chart::Sheet(_) => uv[1],Chart::End(k) => self.roll[k],
            Chart::Collar => self.pinch.as_ref().and_then(|p| p.collar.place(uv[0],uv[1])).map_or(f64::NAN,|p| p[1]),
        }
    }

    /// **The chart's point at `uv`**, its two rates and a unit normal: a sheet's (`point`, its
    /// spine's normal), the posed tool's in closed form (its outward normal), or the collar's
    /// (`collar_at`).
    pub fn at(&self,c: Chart,uv: [f64;2]) -> Option<Point> {
        match c {
            Chart::Sheet(side) => self.point(side,uv[0],uv[1]),
            Chart::End(k) => {
                let m = self.path.at(self.roll[k]).ok()?;
                let ((s,co),(sp,cp)) = (uv[0].dsin_cos(),uv[1].dsin_cos());
                let f = &self.ring.frame;
                let (er,et,ez) = (f.dir([co,s,0.]),f.dir([-s,co,0.]),f.z);
                let n = add(scale(er,cp),scale(ez,sp));
                let x = add(f.at([self.radius*co,self.radius*s,self.height]),scale(n,self.r));
                Some(Point {x:m.point(x),x_th:m.vector(scale(et,self.radius+self.r*cp)),
                    x_t:m.vector(scale(add(scale(er,-sp),scale(ez,cp)),self.r)),n:m.vector(n),area:1.})
            }
            Chart::Collar => self.collar_at(uv[0],uv[1]),
        }
    }

    /// **Which side of a chart can bound the sweep**, positive there: on a sheet its Jacobian `J`
    /// (beyond its cuspidal edge the instants beside hold it); on an end pose the part moving out
    /// of the sweep — at the start the part the motion leaves behind (`n · v < 0`), at the end the
    /// part leading (`n · v > 0`), read against the point's speed — the rest held by the instants
    /// inside the roll. Nought on its contact circles, the sheets' rims. A sheet's points deep in
    /// the pinch's hole (`Pinch::deep`) are the collar's, not the sheet's (none): it is read on
    /// across the hole's rim, which as an inner edge stops the tracer, so its exit there is settled
    /// on the field; on the collar its own side (`collar_side`).
    pub fn side(&self,c: Chart,uv: [f64;2]) -> Option<f64> {
        match c {
            Chart::Sheet(side) => {
                if self.pinch.as_ref().is_some_and(|p| p.deep(uv)) { return None }
                self.jacobian(side,uv[0],uv[1])
            }
            Chart::Collar => self.collar_side(uv[0],uv[1]),
            Chart::End(k) => {
                let m = self.path.at(self.roll[k]).ok()?;
                let p = self.at(c,uv)?;
                let q = m.inverse().point(p.x);
                let v = m.velocity(q);
                let speed = norm(v);
                (speed > 0.).then(|| if k == 0 { -1. } else { 1. }*dot(p.n,v)/speed)
            }
        }
    }
}

/// **The far cover's depth over a chart, as a field** (`zero_set::Field`): the chart's points by
/// their parameters, each read by `depth::cover` to `within` against its own instant — the value the
/// far cover's depth, what it was read at its `Cover` (the covering instant and the tool's point
/// there). Not defined where the chart cannot bound the sweep (`Tube::side` at or below nought),
/// where the neighbours hold it (`Near`), or where nothing else covers it (`Alone`). Its gradient
/// is the envelope theorem's: the covering instant's distance gradient against the chart's rates,
/// each in closed form.
pub struct Depth<'a> { pub tube: &'a Tube<'a>,pub chart: Chart,pub within: f64,pub holes: &'a [Disc] }

impl Depth<'_> {
    /// The chart's point where it can bound the sweep, with its rates.
    fn kept(&self,uv: [f64;2]) -> Option<Point> {
        if self.holes.iter().any(|d| d.side(uv) <= 0.) { return None }
        let p = self.tube.at(self.chart,uv)?;
        (self.tube.side(self.chart,uv)? > 0.).then_some(p)
    }

    fn value(&self,p: &Point,at: f64) -> Option<Value<Cover>> {
        let (depth,g) = self.tube.depth_at(p.x,at)?;
        let pose = self.tube.path.pose(at).ok()?;
        let point = pose.inverse().point(p.x);
        Some(Value {f:depth,grad:Some([dot(g,p.x_th),dot(g,p.x_t)]),at:Cover {depth,at,point,piece:0}})
    }

    /// How the chart's point at `uv` stands under the sweep (`depth::cover`).
    pub fn reading(&self,uv: [f64;2]) -> Option<(Point,Reading)> {
        let p = self.kept(uv)?;
        let r = self.tube.cover(p.x,self.tube.own(self.chart,uv),self.within)?;
        Some((p,r))
    }
}

impl Field for Depth<'_> {
    type At = Cover;
    /// A sheet passes through itself along its double curves with itself, each traced on both its
    /// preimages: a seed of the second near the first in space is not the first.
    fn folded(&self) -> bool { true }
    fn least(&self,uv: [f64;2]) -> Option<Value<Cover>> {
        match self.reading(uv)? {
            (p,Reading::Far(c)) => self.value(&p,c.at),
            _ => None,
        }
    }

    /// The least of the tool's depth at the point about the covering instant `from` names: over the
    /// stretch of the roll a quarter of the ring's thickness long in the point's motion either side
    /// of it (the feature `cover` steps by), within the roll and short of the point's own instant
    /// by half the way to it (its own contact, nought there, is no cover) — `depth::Covered`'s.
    fn along(&self,uv: [f64;2],from: &Cover) -> Option<Value<Cover>> {
        let p = self.kept(uv)?;
        let [a,b] = self.tube.roll;
        let domain = crate::util::interval::Interval::new(a,b).ok()?;
        let speed = self.tube.path.inverse_point_speed_bound(p.x,domain).ok()?;
        let reach = if speed > 0. { 0.25*self.tube.ring.thickness/speed } else { b-a };
        let (mut lo,mut hi) = ((from.at-reach).max(a),(from.at+reach).min(b));
        let own = self.tube.own(self.chart,uv);
        if own >= lo && own <= hi { if from.at > own { lo = 0.5*(own+from.at) } else { hi = 0.5*(own+from.at) } }
        let d = |t: f64| self.tube.depth_at(p.x,t).map_or(f64::INFINITY,|r| r.0);
        let mut t = if hi > lo { golden(&d,lo,hi) } else { from.at };
        // (an end of the roll the stretch reaches read there too — the pose there may hold the
        // least, on its boundary: past the point's own contact, climbing toward that end, `d` is
        // no single basin over the stretch, and the golden section may settle on the climb)
        for e in [a,b] { if (e == lo || e == hi) && d(e) <= d(t) { t = e } }
        self.value(&p,t)
    }

    /// The chart's point and rates; on a sheet inside the pinch's hole the net's (`Tube::net_point`,
    /// its rates by differences): the sheet's own map is a cone point at the pinch, where a cell
    /// holding it keeps a reach of the tube's size however it is halved, and the search asked four
    /// million cells of the forward member.
    fn surface(&self,uv: [f64;2]) -> (V,V,V) {
        if let (Chart::Sheet(side),Some(p)) = (self.chart,&self.tube.pinch) {
            if p.side(uv) < 0. {
                let [h,k] = self.tube.steps(self.chart);
                let x = |a: f64,b: f64| self.tube.net_point(side,a,b).unwrap_or([f64::NAN;3]);
                let d = |a: V,b: V,e: f64| scale(sub(a,b),0.5/e);
                return (x(uv[0],uv[1]),d(x(uv[0]+h,uv[1]),x(uv[0]-h,uv[1]),h),d(x(uv[0],uv[1]+k),x(uv[0],uv[1]-k),k))
            }
        }
        match self.tube.at(self.chart,uv) { Some(p) => (p.x,p.x_th,p.x_t),None => ([f64::NAN;3],[f64::NAN;3],[f64::NAN;3]) }
    }

    /// **The depth's bound over a cell**: the tool's depth at each instant is 1-Lipschitz in space,
    /// so where the same basins of the roll cover the cell the far cover's depth lies within the
    /// cell's reach of its value at its middle: the middle's far cover less the reach (the greatest
    /// distance from the middle's point to the corners' and sides' middles', as `zero_set` reads a
    /// reach), one certified cover a cell where nine were read — the half turn's sheet `−` traced in
    /// 13.6 s, not 32.7. Where nothing else covers the middle (`Alone`) there is no far cover to
    /// bound from: the nine are read, their least far cover less their spread, all nine `Alone` read
    /// clear — sampled evidence, not proof: a cover born and dying between the nine is missed, as the
    /// search's sampling would miss it. Above, where the middle is covered deeper than the reach,
    /// the middle's far cover plus the reach (its covering instant holds the whole cell, so it is
    /// past each point's own contact): so a run of a rim or a cell covered through is cleared, where
    /// the full turn's rim at its start, covered for a stretch, was halved to the tolerance and every
    /// piece of it touched. None where a point is not kept or the middle's neighbours hold it (the
    /// cell meets the chart's edge).
    fn bound(&self,c: crate::zero_set::Rect) -> Option<crate::zero_set::Bound> {
        // (the middle's cover read, the others' points only for the reach)
        let mid = [0.5*(c[0][0]+c[0][1]),0.5*(c[1][0]+c[1][1])];
        let (pm,rm) = self.reading(mid)?;
        let mut xs = Vec::with_capacity(9);
        for i in 0..3 { for j in 0..3 {
            let uv = [c[0][0]+(c[0][1]-c[0][0])*i as f64/2.,c[1][0]+(c[1][1]-c[1][0])*j as f64/2.];
            xs.push(self.kept(uv)?.x);
        } }
        let reach = xs.iter().map(|x| crate::util::space::distance(*x,pm.x)).fold(0.,f64::max);
        // (where the middle is held below nought deeper than the reach, the covering instant holds
        // every point of the cell: its depth there, 1-Lipschitz, is below nought, so that instant
        // lies past each point's own contact and the far cover is at most it)
        let hi = match rm { Reading::Far(cv) if cv.depth+reach < 0. => cv.depth+reach,_ => f64::MAX };
        let lo = match rm {
            Reading::Far(cv) => cv.depth-reach,
            Reading::Near => return None,
            // (nothing else covers the middle: the nine read, as evidence)
            Reading::Alone => {
                let mut least = f64::INFINITY;
                for i in 0..3 { for j in 0..3 {
                    let uv = [c[0][0]+(c[0][1]-c[0][0])*i as f64/2.,c[1][0]+(c[1][1]-c[1][0])*j as f64/2.];
                    match self.reading(uv)?.1 { Reading::Far(cv) => least = least.min(cv.depth),Reading::Alone => {},Reading::Near => return None }
                } }
                let all = xs.iter().flat_map(|x| xs.iter().map(move |y| crate::util::space::distance(*x,*y))).fold(0.,f64::max);
                if least.is_finite() { least-all } else { all }
            }
        };
        Some(crate::zero_set::Bound {value:crate::util::interval::Iv::new(lo,hi),grad:None})
    }
}

/// **Where a chart stops bounding the sweep, as a field**: `Tube::side` over its parameters — on a
/// sheet its cuspidal edge, on an end pose its contact circles. Its metric on a sheet is the
/// spine's (the sheet's own is singular on the edge), its gradient by differences.
pub struct Edge<'a> { pub tube: &'a Tube<'a>,pub chart: Chart }

impl Field for Edge<'_> {
    type At = ();
    fn least(&self,uv: [f64;2]) -> Option<Value<()>> {
        let grad = match self.chart {
            Chart::Sheet(side) => self.tube.jacobian_gradient(side,uv[0],uv[1]),Chart::End(_) => None,
            Chart::Collar => self.tube.side_gradient(Chart::Collar,uv),
        };
        Some(Value {f:self.tube.side(self.chart,uv)?,grad,at:()})
    }
    fn along(&self,uv: [f64;2],_: &()) -> Option<Value<()>> { self.least(uv) }
    fn bound(&self,c: crate::zero_set::Rect) -> Option<crate::zero_set::Bound> {
        match self.chart {
            Chart::Sheet(side) => self.tube.jacobian_bound(side,c),Chart::End(k) => self.tube.contact_bound(k,c),
            Chart::Collar => None,
        }
    }
    fn surface(&self,uv: [f64;2]) -> (V,V,V) {
        let s = match self.chart {
            Chart::Sheet(_) => self.tube.spine(uv[0],uv[1]).map(|s| (s.s,s.s_th,s.s_t)),
            Chart::End(_) => self.tube.at(self.chart,uv).map(|p| (p.x,p.x_th,p.x_t)),
            Chart::Collar => self.tube.collar_metric(uv[0],uv[1]),
        };
        s.unwrap_or(([f64::NAN;3],[f64::NAN;3],[f64::NAN;3]))
    }
}

impl Tube<'_> {
    /// **The sign of a sheet's Jacobian over a cell of its parameters, proved** (the cuspidal edges'
    /// bound, `Edge::bound`): `J = det(I − s r II)/det I` (`I`, `II` the spine's fundamental forms,
    /// `s` the side), so with `n = S_θ × S_t`, `w = |n|`, and `II'` the second form read against `n`
    /// (`S_θθ · n`, `S_θt · n`, `S_tt · n`), `J = det Q / w⁴` for the symmetric
    /// `Q = w I − s r II'`. At the cell's middle `Q` is read, and each entry's rates over the cell are
    /// bounded by the core circle's (`R` its radius; `Ω`, `V`, `A`, `J₃` the motion's spin and the
    /// circle's speed, acceleration and jerk, `Rates`): `|S_θ| = |S_θθ| = |S_θθθ| = R`,
    /// `|S_θt|, |S_θθt| ≤ ΩR`, `|S_θtt| ≤ Ω²R`, `|S_t| ≤ V`, `|S_tt| ≤ A`, `|S_ttt| ≤ J₃`, and so
    /// `|n| ≤ RV`, `|n_θ| ≤ RV + ΩR²`, `|n_t| ≤ ΩRV + RA`, each product's rate by the product rule.
    /// Each entry then moves over the cell by at most its rates times the cell's half widths, and
    /// `Q`'s eigenvalues by at most that change's Frobenius norm `δ` (Weyl): where the middle's
    /// least eigenvalue stands above `δ`, `Q` is definite over the cell and `J > 0`; where its least
    /// stands below `−δ` and its greatest above `δ`, `Q` is indefinite and `J < 0` — each with `J`'s
    /// enclosure from `w`'s over the cell. (The middle's `S_tt`, a central difference, is off by at
    /// most its truncation `h² J₄/6` and its rounding: both added to `δ`.) None where neither is
    /// proved, the spine may be singular over the cell, or the motion gives no bound.
    pub fn jacobian_bound(&self,side: f64,c: crate::zero_set::Rect) -> Option<crate::zero_set::Bound> {
        let Rates {spin:o,jet:[v,a,j3,j4]} = self.rates?;
        let (rr,r) = (self.radius,self.r);
        let mid = [0.5*(c[0][0]+c[0][1]),0.5*(c[1][0]+c[1][1])];
        let (dth,dt) = (0.5*(c[0][1]-c[0][0]),0.5*(c[1][1]-c[1][0]));
        let sp = self.spine(mid[0],mid[1])?;
        let n = cross(sp.s_th,sp.s_t);
        let w = norm(n);
        let (e,f,g) = (dot(sp.s_th,sp.s_th),dot(sp.s_th,sp.s_t),dot(sp.s_t,sp.s_t));
        let k = side*r;
        let q = [w*e-k*dot(sp.s_thth,n),w*f-k*dot(sp.s_tht,n),w*g-k*dot(sp.s_tt,n)];
        // (the middle's `S_tt`'s difference: `spine`'s step, its truncation and its rounding)
        let h = THIRD_READ*(self.roll[1]-self.roll[0]);
        let off = h*h*j4/6.+16.*f64::EPSILON*v/h;
        // the rates over the cell, each bounded from the middle's value by its own rates
        let st = norm(sp.s_t);
        let vs = (st+o*rr*dth+a*dt).min(v);
        let acc = (norm(sp.s_tt)+off+o*o*rr*dth+j3*dt).min(a);
        let (nth,nt) = (rr*vs+o*rr*rr,o*rr*vs+rr*acc);
        let (wlo,whi) = (w-nth*dth-nt*dt,(w+nth*dth+nt*dt).min(rr*vs));
        if !(wlo > 0. && st > 0.) { return None }
        let nb = whi;
        let (fth,ft) = (rr*vs+o*rr*rr,o*rr*vs+rr*acc);
        let (gth,gt) = (2.*o*rr*vs,2.*acc*vs);
        let d11 = (nth*rr*rr+r*(rr*nb+rr*nth))*dth+(nt*rr*rr+r*(o*rr*nb+rr*nt))*dt;
        let d12 = (nth*rr*vs+nb*fth+r*(o*rr*nb+o*rr*nth))*dth+(nt*rr*vs+nb*ft+r*(o*o*rr*nb+o*rr*nt))*dt;
        let d22 = (nth*vs*vs+nb*gth+r*(o*o*rr*nb+acc*nth))*dth+(nt*vs*vs+nb*gt+r*(j3*nb+acc*nt))*dt+r*off*nb;
        // where the tube is thin beside the spine's bending, the plainer test: `I ⪰ λ Id` with
        // `λ ≥ W/(E + G)` (`λ λ' = W`, `λ' ≤ E + G`), so `P = I − (s r/w) II'` is definite where
        // `λ > r |II'|/w` over the cell, `|II'|` its Frobenius norm, each entry bounded from the
        // middle's by its rates (`Q`'s own less the first form's); and `J = det P / W ≥ m²/w²`
        if r > 0. {
            let (l,m,nn) = (dot(sp.s_thth,n).abs(),dot(sp.s_tht,n).abs(),dot(sp.s_tt,n).abs());
            let dl = (rr*nb+rr*nth)*dth+(o*rr*nb+rr*nt)*dt;
            let dm = (o*rr*nb+o*rr*nth)*dth+(o*o*rr*nb+o*rr*nt)*dt;
            let dn = (o*o*rr*nb+acc*nth)*dth+(j3*nb+acc*nt)*dt+off*nb;
            let second = ((l+dl).powi(2)+2.*(m+dm).powi(2)+(nn+dn).powi(2)).sqrt();
            let margin = wlo*wlo/(rr*rr+vs*vs)-r*second/wlo;
            if margin > 0. {
                return Some(crate::zero_set::Bound {value:crate::util::interval::Iv::new(margin*margin/(whi*whi),f64::MAX),grad:None})
            }
        }
        // (read in the parameters scaled by the middle's rates, `diag(1/R, 1/|S_t|)`: a congruence,
        // so definiteness is kept, and `Q`'s two ways weighed alike)
        let (u1,u2) = (1./rr,1./st);
        let q = [q[0]*u1*u1,q[1]*u1*u2,q[2]*u2*u2];
        let (d11,d12,d22) = (d11*u1*u1,d12*u1*u2,d22*u2*u2);
        let delta = (d11*d11+2.*d12*d12+d22*d22).sqrt();
        let half = 0.5*(q[0]+q[2]);
        let spread = (0.5*(q[0]-q[2])).dhypot(q[1]);
        let (least,most) = (half-spread,half+spread);
        // (`J = det Q / w⁴`, `det Q` the scaled one's times `R² |S_t|²`)
        let scale = rr*rr*st*st/whi.powi(4);
        let value = if least > delta {
            let l = least-delta;
            crate::util::interval::Iv::new(l*l*scale,f64::MAX)
        } else if least < -delta && most > delta {
            crate::util::interval::Iv::new(-f64::MAX,(least+delta)*(most-delta)*scale)
        } else { return None };
        Some(crate::zero_set::Bound {value,grad:None})
    }
}

impl Tube<'_> {
    /// **The sign of an end pose's `Tube::side` over a cell of its `(θ, ψ)`, proved** (its contact
    /// circles' bound): the side is `± n · v / |v|`, `n` the posed tool's unit normal and `v` the
    /// motion's velocity at its point, so its sign is `g = ± n · v`'s. Over the cell `g` moves from
    /// the middle's by at most its rates times the half widths: `|n_θ| ≤ 1`, `|n_ψ| = 1`,
    /// `|x_θ| ≤ R + r`, `|x_ψ| = r`, and `v`, affine in the point, turns with it by the motion's spin
    /// `Ω` at most (`|v_θ| ≤ Ω (R + r)`, `|v_ψ| ≤ Ω r`), `|v|` over the cell at most the middle's
    /// plus those. Where `g` keeps its sign the side is enclosed (`g` over the greatest `|v|`). None
    /// where it is not proved or the motion gives no bound.
    pub fn contact_bound(&self,k: usize,c: crate::zero_set::Rect) -> Option<crate::zero_set::Bound> {
        let o = self.rates?.spin;
        let mid = [0.5*(c[0][0]+c[0][1]),0.5*(c[1][0]+c[1][1])];
        let (dth,dps) = (0.5*(c[0][1]-c[0][0]),0.5*(c[1][1]-c[1][0]));
        let m = self.path.at(self.roll[k]).ok()?;
        let p = self.at(Chart::End(k),mid)?;
        let v = m.velocity(m.inverse().point(p.x));
        let g = if k == 0 { -1. } else { 1. }*dot(p.n,v);
        let (big,r) = (self.radius+self.r,self.r);
        let vmax = norm(v)+o*big*dth+o*r*dps;
        let spread = (vmax+o*big)*dth+(vmax+o*r)*dps;
        let value = if g > spread { crate::util::interval::Iv::new((g-spread)/vmax,f64::MAX) }
            else if g < -spread { crate::util::interval::Iv::new(-f64::MAX,(g+spread)/vmax) } else { return None };
        Some(crate::zero_set::Bound {value,grad:None})
    }
}

/// The golden-section least of `f` over `[lo, hi]`, to the rounding of the roll.
fn golden(f: &dyn Fn(f64) -> f64,mut lo: f64,mut hi: f64) -> f64 {
    let g = 0.5*(5f64.sqrt()-1.);
    loop {
        let (x1,x2) = (hi-g*(hi-lo),lo+g*(hi-lo));
        if !(x1 > lo && x2 < hi && x1 < x2) { break }
        if f(x1) < f(x2) { hi = x2 } else { lo = x1 }
    }
    0.5*(lo+hi)
}

/// **A field's zero set over a chart's whole rectangle**: traced over the rectangle cut in two
/// across each parameter that runs a whole turn (`periodic`), each part searched for closed curves
/// too (`zero_set::trace`; its search once read a cell's longer way between its sides' ends, which
/// over a whole turn are one point, and is mended, the halves kept for their seeds across the cuts)
/// — and the pieces of every part
/// returned (a curve across a cut in two, each ending on the rim there), with each part's search.
/// Inner edges (`edges`, polylines in the parameters) are given to each part as their runs inside
/// it, all with the one side function `side`; seeds to the part holding them, each brought into the
/// turn.
#[allow(clippy::too_many_arguments,clippy::type_complexity)]
pub fn round<F: Field>(field: &F,rect: crate::zero_set::Rect,periodic: [bool;2],seeds: &[[f64;2]],edges: &[Vec<[f64;2]>],
    side: &dyn Fn([f64;2]) -> f64,tol: f64,feature: f64) -> Result<(Vec<crate::zero_set::Piece<F::At>>,Vec<crate::zero_set::Search>),String> {
    let cut = |k: usize| -> Vec<[f64;2]> {
        let [a,b] = rect[k];
        if periodic[k] { vec![[a,0.5*(a+b)],[0.5*(a+b),b]] } else { vec![[a,b]] }
    };
    let wrap = |s: [f64;2]| std::array::from_fn::<f64,2,_>(|k| if periodic[k] { rect[k][0]+(s[k]-rect[k][0]).rem_euclid(rect[k][1]-rect[k][0]) } else { s[k] });
    let parts: Vec<crate::zero_set::Rect> = cut(0).into_iter().flat_map(|wu| cut(1).into_iter().map(move |wv| [wu,wv])).collect();
    let part = |[wu,wv]: crate::zero_set::Rect,seeds: &[[f64;2]]| {
        let inside = |s: &[f64;2]| s[0] >= wu[0] && s[0] <= wu[1] && s[1] >= wv[0] && s[1] <= wv[1];
        let mine: Vec<[f64;2]> = seeds.iter().map(|&s| wrap(s)).filter(inside).collect();
        // (each inner edge's runs inside the part, its points brought into the turn, each run a
        // polyline read at a fraction of its points' count)
        let mut lines: Vec<Vec<[f64;2]>> = Vec::new();
        for e in edges {
            let mut run: Vec<[f64;2]> = Vec::new();
            for &q in e {
                let q = wrap(q);
                if inside(&q) { run.push(q) } else { if run.len() >= 2 { lines.push(std::mem::take(&mut run)) } run.clear() }
            }
            if run.len() >= 2 { lines.push(run) }
        }
        let ats: Vec<Box<dyn Fn(f64) -> [f64;2]>> = lines.iter().map(|e| {
            let e: &Vec<[f64;2]> = e;
            Box::new(move |f: f64| {
                let x = f.clamp(0.,1.)*(e.len()-1) as f64;
                let k = (x.floor() as usize).min(e.len()-2);
                let g = x-k as f64;
                [e[k][0]+(e[k+1][0]-e[k][0])*g,e[k][1]+(e[k+1][1]-e[k][1])*g]
            }) as Box<dyn Fn(f64) -> [f64;2]>
        }).collect();
        let inner: Vec<crate::zero_set::Edge> = ats.iter().map(|at| crate::zero_set::Edge {at:&**at,side}).collect();
        crate::zero_set::trace(field,&crate::zero_set::Question {rect:[wu,wv],edges:&inner,seeds:&mine,tol,feature})
    };
    // (each part traced; a piece leaving a part across a cut seeds the parts beside it — beside a
    // seam the far cover may be read on one side only, so the cut's own sign changes miss it — and
    // a part given new seeds traced again, until none is)
    let mut given: Vec<Vec<[f64;2]>> = vec![seeds.to_vec();parts.len()];
    let mut traced: Vec<Option<crate::zero_set::Traced<F::At>>> = (0..parts.len()).map(|_| None).collect();
    let mut asked: Vec<bool> = vec![true;parts.len()];
    while let Some(k) = asked.iter().position(|&a| a) {
        asked[k] = false;
        let t = part(parts[k],&given[k])?;
        for p in &t.pieces {
            for (uv,end) in [(p.uv[0],p.ends[0]),(p.uv[p.uv.len()-1],p.ends[1])] {
                if end != crate::zero_set::End::Rim { continue }
                for (j,r) in parts.iter().enumerate() {
                    if j == k { continue }
                    let w = wrap(uv);
                    // (on a cut, the side of the part beside it: within the rounding of the cut)
                    let on = |x: f64,[a,b]: [f64;2]| x >= a-HALF_READ*(b-a) && x <= b+HALF_READ*(b-a);
                    let cut_k = |i: usize| periodic[i] && (uv[i] == parts[k][i][0] || uv[i] == parts[k][i][1]);
                    if !(cut_k(0) || cut_k(1)) { continue }
                    if !(on(w[0],r[0]) && on(w[1],r[1])) { continue }
                    let w = [w[0].clamp(r[0][0],r[0][1]),w[1].clamp(r[1][0],r[1][1])];
                    let w = std::array::from_fn::<f64,2,_>(|i| if cut_k(i) && (w[i]-r[i][0]).abs() > (w[i]-r[i][1]).abs() { r[i][1] }
                        else if cut_k(i) { r[i][0] } else { w[i] });
                    if given[j].iter().any(|g| *g == w) { continue }
                    given[j].push(w);
                    if traced[j].is_some() { asked[j] = true }
                }
            }
        }
        traced[k] = Some(t);
    }
    let (mut pieces,mut searches) = (Vec::new(),Vec::new());
    for t in traced.into_iter().flatten() { pieces.extend(t.pieces); searches.push(t.search); }
    Ok((pieces,searches))
}

/// **Where a chart stops bounding the sweep**: `Tube::side`'s zero set over the chart's whole
/// rectangle (`round`) to `tol` in space (a sheet's spine's), `feature` its longest step — on a
/// sheet its cuspidal edges, on an end pose its contact circles.
pub fn edges(tube: &Tube,chart: Chart,tol: f64,feature: f64) -> Result<(Vec<crate::zero_set::Piece<()>>,Vec<crate::zero_set::Search>),Unbuilt> {
    let periodic = [true,matches!(chart,Chart::End(_))];
    let hole = tube.hole(chart)?;
    let side = |uv: [f64;2]| tube.pinch.as_ref().filter(|_| !hole.is_empty()).map_or(1.,|p| p.side(uv));
    round(&Edge {tube,chart},tube.rect(chart),periodic,&[],&hole,&side,tol,feature)
        .map_err(|why| Unbuilt::Failed(format!("the tube road's chart {chart:?}: where it stops bounding the sweep: {why}")))
}

/// **A disc of a chart's parameters about a swallowtail point**, cut from the depth's domain: an
/// ellipse in the parameters, the points within `radius` of the centre in the metric `metric` (the
/// spine's there, so `radius` is a length in space).
#[derive(Clone,Copy,Debug)]
pub struct Disc { pub at: [f64;2],pub metric: [[f64;2];2],pub radius: f64 }

impl Disc {
    /// Positive outside the disc, nought on its rim.
    pub fn side(&self,uv: [f64;2]) -> f64 {
        let d = [uv[0]-self.at[0],uv[1]-self.at[1]];
        let m = self.metric;
        (d[0]*(m[0][0]*d[0]+m[0][1]*d[1])+d[1]*(m[1][0]*d[0]+m[1][1]*d[1])).max(0.).sqrt()-self.radius
    }

    /// The rim's point at the fraction `f` of a turn: `radius L⁻ᵀ (cos, sin)`, `M = L Lᵀ`.
    pub fn rim(&self,f: f64) -> [f64;2] {
        let m = self.metric;
        let l11 = m[0][0].sqrt();
        let l21 = m[1][0]/l11;
        let l22 = (m[1][1]-l21*l21).max(0.).sqrt();
        let (s,c) = (TAU*f).dsin_cos();
        let (x,y) = (self.radius*c,self.radius*s);
        // (Lᵀ d = (x, y), Lᵀ upper triangular)
        let d1 = y/l22;
        let d0 = (x-l21*d1)/l11;
        [self.at[0]+d0,self.at[1]+d1]
    }

    /// The rim as a closed polyline of `n` chords.
    pub fn polygon(&self,n: usize) -> Vec<[f64;2]> { (0..=n).map(|k| self.rim(k as f64/n as f64)).collect() }
}

/// Chords a disc's rim is read in: a turn in 64, each a 64th of the disc's girth (structural: the
/// rim is an edge of the depth's domain, sampled as `zero_set` samples an inner edge).
const RIM: usize = 64;

/// **The disc about a swallowtail point `at` of a chart** inside which the depth is read as
/// nought: within it the far cover's depth is within the tolerance everywhere the rim shows — the
/// double curve's two leaves and the tail between them within the tolerance of the sheet beyond (a
/// cover clear above nought from an instant outside the disc's span of the roll is another
/// basin's, not counted), so the trim may run to the point along any path in it — its radius halved from the feature size
/// until every far cover read on its rim (`RIM` points) is within `tol`; refused where that asks a
/// disc under the tolerance.
pub fn disc(tube: &Tube,chart: Chart,at: [f64;2],tol: f64,feature: f64) -> Result<Disc,Unbuilt> {
    let Some(sp) = tube.spine(at[0],at[1]) else { return Err(Unbuilt::Failed(format!("the tube road: no spine at the swallowtail point {at:?}"))) };
    let metric = [[dot(sp.s_th,sp.s_th),dot(sp.s_th,sp.s_t)],[dot(sp.s_th,sp.s_t),dot(sp.s_t,sp.s_t)]];
    let field = Depth {tube,chart,within:tol,holes:&[]};
    let mut radius = feature;
    while radius >= tol {
        let d = Disc {at,metric,radius};
        // (a cover standing clear above nought from an instant outside the disc's own span of the
        // roll is another basin's — an end pose's, far off — not the swallowtail's lens, whose two
        // leaves pair points of the disc: it does not count)
        let span = (0..RIM).map(|k| d.rim(k as f64/RIM as f64)[1]).fold([f64::INFINITY,f64::NEG_INFINITY],|s,t| [s[0].min(t),s[1].max(t)]);
        let deep = (0..RIM).any(|k| matches!(field.reading(d.rim(k as f64/RIM as f64)),Some((_,Reading::Far(c)))
            if c.depth < -tol || c.depth > tol && c.at >= span[0] && c.at <= span[1]));
        if !deep { return Ok(d) }
        radius *= 0.5;
    }
    Err(Unbuilt::Failed(format!("the tube road: the depth about the swallowtail point {at:?} is not within the tolerance on any disc")))
}

/// **The depth's zero set over a chart** (`Depth`, `round`) to `tol`, `feature` its longest step,
/// the chart's own edges (`edges`) given as inner edges, a disc about each swallowtail point
/// (`tails`, `disc`) cut from its domain with its rim an inner edge too, `seeds` traced as well: the
/// curves where another instant's cover of the chart begins, each point with its covering instant,
/// and the discs.
#[allow(clippy::type_complexity)]
pub fn seams(tube: &Tube,chart: Chart,edges: &[Vec<[f64;2]>],tails: &[[f64;2]],seeds: &[[f64;2]],tol: f64,feature: f64)
    -> Result<(Vec<crate::zero_set::Piece<Cover>>,Vec<crate::zero_set::Search>,Vec<Disc>),Unbuilt> {
    let periodic = [true,matches!(chart,Chart::End(_))];
    // (a swallowtail point held inside by another instant — every far cover on the rim of the
    // feature's disc about it below nought by more than the tolerance — bounds nothing: the double
    // curve it ends is covered through, and no trim comes near it)
    let held = |at: [f64;2]| -> bool {
        let Some(sp) = tube.spine(at[0],at[1]) else { return false };
        let metric = [[dot(sp.s_th,sp.s_th),dot(sp.s_th,sp.s_t)],[dot(sp.s_th,sp.s_t),dot(sp.s_t,sp.s_t)]];
        let d = Disc {at,metric,radius:feature};
        let field = Depth {tube,chart,within:tol,holes:&[]};
        (0..RIM).all(|k| matches!(field.reading(d.rim(k as f64/RIM as f64)),Some((_,Reading::Far(c))) if c.depth < -tol))
    };
    let holes: Vec<Disc> = tails.iter().filter(|&&at| !held(at)).map(|&at| disc(tube,chart,at,tol,feature)).collect::<Result<_,_>>()?;
    let mut inner: Vec<Vec<[f64;2]>> = edges.to_vec();
    inner.extend(holes.iter().map(|d| d.polygon(RIM)));
    inner.extend(tube.hole(chart)?);
    let side = |uv: [f64;2]| holes.iter().map(|d| d.side(uv)).fold(tube.side(chart,uv).unwrap_or(-1.),f64::min)
        .min(match (chart,&tube.pinch) { (Chart::Sheet(_),Some(p)) => p.side(uv),_ => f64::INFINITY });
    let mut seeds = seeds.to_vec();
    seeds.extend(beside_edges(tube,chart,edges,tol,feature));
    let clock = std::time::Instant::now();
    let (pieces,search) = round(&Depth {tube,chart,within:tol,holes:&holes},tube.rect(chart),periodic,&seeds,&inner,&side,tol,feature)
        .map_err(|why| Unbuilt::Failed(format!("the tube road's chart {chart:?}: its depth's zero set: {why}")))?;
    if std::env::var_os("SOLVENT_TUBE_DEBUG").is_some() { eprintln!("seams of {chart:?}: {} pieces in {:?}",pieces.len(),clock.elapsed()) }
    Ok((pieces,search,holes))
}

/// **The swallowtail points on a cuspidal edge** (`edges`' piece `uv`): where the sheet's kernel
/// runs along the edge — the gauge `∇J · k` (`k` the kernel, its sense carried continuously along
/// the edge) changing sign between two of its points — each settled by Newton's method on
/// `(J, ∇J · k) = 0`, its Jacobian by differences, from the chord's root of the gauge, while the
/// steps shrink (to the rounding).
pub fn swallowtails(tube: &Tube,side: f64,uv: &[[f64;2]]) -> Vec<[f64;2]> {
    let gauge = |p: [f64;2],sense: [f64;2]| -> Option<(f64,f64,[f64;2])> {
        let k = tube.kernel(side,p[0],p[1])?;
        let k = if k[0]*sense[0]+k[1]*sense[1] < 0. { [-k[0],-k[1]] } else { k };
        let g = tube.jacobian_gradient(side,p[0],p[1])?;
        Some((tube.jacobian(side,p[0],p[1])?,g[0]*k[0]+g[1]*k[1],k))
    };
    let mut out = Vec::new();
    let mut sense = [1.,0.];
    let mut last: Option<([f64;2],f64)> = None;
    for &p in uv {
        let Some((_,g,k)) = gauge(p,sense) else { last = None; continue };
        sense = k;
        if let Some((q,gq)) = last {
            if (g < 0.) != (gq < 0.) {
                let f = gq/(gq-g);
                let mut x = [q[0]+(p[0]-q[0])*f,q[1]+(p[1]-q[1])*f];
                let (h,ht) = (THIRD_READ,THIRD_READ*(tube.roll[1]-tube.roll[0]));
                let mut prev = f64::INFINITY;
                for _ in 0..32 {
                    let Some((j0,g0,_)) = gauge(x,sense) else { break };
                    let (Some(a),Some(b),Some(c),Some(d)) = (gauge([x[0]+h,x[1]],sense),gauge([x[0]-h,x[1]],sense),
                        gauge([x[0],x[1]+ht],sense),gauge([x[0],x[1]-ht],sense)) else { break };
                    let m = [[(a.0-b.0)/(2.*h),(c.0-d.0)/(2.*ht)],[(a.1-b.1)/(2.*h),(c.1-d.1)/(2.*ht)]];
                    let det = m[0][0]*m[1][1]-m[0][1]*m[1][0];
                    if !(det.abs() > 0.) { break }
                    let dx = [(j0*m[1][1]-g0*m[0][1])/det,(m[0][0]*g0-m[1][0]*j0)/det];
                    let step = dx[0].dhypot(dx[1]);
                    if !(step < prev) { break }
                    prev = step;
                    x = [x[0]-dx[0],x[1]-dx[1]];
                }
                out.push(x);
            }
        }
        last = Some((p,g));
    }
    out
}

/// Chords the pinch's hole's rim is read in as an inner edge of a sheet: a turn of the collar in
/// four times a disc's (structural: the rim is one curve read in three charts, and the tracer's
/// runs end on it where its chord stands; each crossing is then settled on the rim itself).
const HOLE_RIM: usize = 4*RIM;

/// **The pinch** (#880 T1f): where the instant `t*` touches the meridian at `θ*` whole
/// (`Ring::touched_once`), the core circle's sweep `S` is singular — `S_t` along `S_θ`, the sheets'
/// normal undefined there — and the collar (`rings::Collar`) charts both sheets about it as one
/// annulus over `(ψ, ρ)`, `(A, B)(θ, t) = ρ (−sin ψ, cos ψ)`: `ρ > 0` the sheet `+` (`S + r N` with
/// `N = (B e_r − A e_z)/|(A, B)|`, the point `c + r (cos ψ, sin ψ)` round the meridian), `ρ < 0`
/// the sheet `−`, `ρ = 0` the meridian at `t*`. Its hole in the sheets' `(θ, t)` is
/// `|(A, B)| < ρ₀` about the instant (within `reach`, the rim's box doubled: a near miss elsewhere
/// in the roll is no hole), its rim `|(A, B)| = ρ₀` one curve read in three charts — the collar's
/// `ρ = ±ρ₀` and each sheet's `place(ψ, ±ρ₀)` — so one edge by construction. `sense` is the sign
/// of `det ∂(A, B)/∂(θ, t)`, which orients the collar against the sheets.
pub struct Pinch<'a> { pub collar: Collar<'a>,pub rho: f64,pub reach: [f64;2],pub sense: f64 }

impl<'a> Pinch<'a> {
    /// The pinch of radius `rho` about the collar's instant; none where its rim is not read whole.
    pub fn new(collar: Collar<'a>,rho: f64) -> Option<Pinch<'a>> {
        let [[a,b],[c,d]] = collar.jacobian;
        let mut p = Pinch {collar,rho,reach:[0.;2],sense:(a*d-b*c).signum()};
        p.reach = p.box_of(rho)?;
        Some(p)
    }

    /// The rim's box at `rho` about the instant, read at `HOLE_RIM` points, doubled.
    fn box_of(&self,rho: f64) -> Option<[f64;2]> {
        let (t0,th0) = self.collar.at;
        let mut reach = [0f64;2];
        for i in 0..HOLE_RIM {
            let [th,t] = self.collar.place(TAU*i as f64/HOLE_RIM as f64,rho)?;
            let way = (th-th0+std::f64::consts::PI).rem_euclid(TAU)-std::f64::consts::PI;
            reach = [reach[0].max(2.*way.abs()),reach[1].max(2.*(t-t0).abs())];
        }
        Some(reach)
    }

    /// **Which side of the hole's rim a sheet's `(θ, t)` lies**: `|(A, B)| − ρ₀`, nought on the rim,
    /// below inside; outside the hole's reach the radius itself (no hole there).
    pub fn side(&self,uv: [f64;2]) -> f64 {
        let (t0,th0) = self.collar.at;
        let way = (uv[0]-th0+std::f64::consts::PI).rem_euclid(TAU)-std::f64::consts::PI;
        if !(way.abs() <= self.reach[0] && (uv[1]-t0).abs() <= self.reach[1]) { return self.rho }
        self.collar.ab(uv[0],uv[1]).map_or(self.rho,|ab| ab[0].dhypot(ab[1])-self.rho)
    }

    /// **Whether a sheet's `(θ, t)` lies deep in the hole**: `|(A, B)|` under `ρ₀/2`, where the
    /// sheet's net reads its stand-in (`Tube::net_point`) — structural: between it and the rim the
    /// sheet is its own, and read, beyond the rim, as the tracer's exit asks.
    pub fn deep(&self,uv: [f64;2]) -> bool { !(self.side(uv) > -0.5*self.rho) }

    /// The rim's place `(θ, t)` on the sheet `side` at the collar's angle `ψ`.
    pub fn rim(&self,side: f64,psi: f64) -> Option<[f64;2]> { self.collar.place(psi,side*self.rho) }

    /// **The collar's angle at a sheet's `(θ, t)`** about the pinch: `(A, B) = ρ (−sin ψ, cos ψ)`
    /// with `ρ`'s sign the sheet's.
    pub fn psi(&self,side: f64,uv: [f64;2]) -> Option<f64> {
        let [a,b] = self.collar.ab(uv[0],uv[1])?;
        Some((-side*a).datan2(side*b).rem_euclid(TAU))
    }

    /// The Jacobian of `(A, B)` by `(θ, t)` at a place, by central differences at `ε^(1/3)` of the
    /// angle and of the roll's extent (`THIRD_READ`), as the collar reads its own.
    fn rates(&self,th: f64,t: f64) -> Option<[[f64;2];2]> {
        let c = &self.collar;
        let (ht,hs) = (THIRD_READ,THIRD_READ*(c.roll[1]-c.roll[0]).abs());
        let (p,m) = (c.ab(th+ht,t)?,c.ab(th-ht,t)?);
        let (q,n) = (c.ab(th,t+hs)?,c.ab(th,t-hs)?);
        Some([[(p[0]-m[0])/(2.*ht),(q[0]-n[0])/(2.*hs)],[(p[1]-m[1])/(2.*ht),(q[1]-n[1])/(2.*hs)]])
    }
}

impl<'a> Tube<'a> {
    /// **The tool read as a tube, its pinch charted**: `Tube::new`, and where an instant inside
    /// the roll touches a meridian whole (`Ring::touched_once`) the collar about it, its radius
    /// `ρ₀` by the rule: the collar's reach — the greatest of the terms' scale halved at which its
    /// rim is read whole (beyond it `(A, B)` folds over the roll and the chart ends) — and within
    /// it the collar's own cuspidal edges traced (`edges` on `Chart::Collar`, to `tol`, `feature`
    /// its step) and their swallowtail points (`swallowtails_on`); `ρ₀` half the least `|ρ|` of
    /// those and of the reach, so the collar holds no swallowtail point and its rim stands at most
    /// halfway out to each (the sheets' own reading of them, discs and joins, stays theirs).
    /// Refused by name where two instants touch a meridian whole or the collar is not regular.
    pub fn pinched(ring: &'a Ring,path: &'a Trajectory,roll: [f64;2],tol: f64,feature: f64) -> Result<Tube<'a>,Unbuilt> {
        let mut tube = Tube::new(ring,path,roll)?;
        let events = ring.touched_once(path,tube.roll)?;
        let event = match events[..] {
            [] => return Ok(tube),
            [e] => e,
            _ => return Err(Unbuilt::Failed(format!("the tube road: {} instants touch a meridian whole, not one",events.len()))),
        };
        let fresh = || Collar::new(ring,path,tube.roll,event)
            .ok_or_else(|| Unbuilt::Failed(format!("the tube road: the collar about the meridian touched whole at {event:?} is not regular")));
        let collar = fresh()?;
        // (the reach: the terms' scale halved until the rim is read whole)
        let whole = |rho: f64| (0..HOLE_RIM).all(|i| { let psi = TAU*i as f64/HOLE_RIM as f64;
            collar.place(psi,rho).is_some() && collar.place(psi,-rho).is_some() });
        let mut reach = collar.size;
        while !whole(reach) {
            reach *= 0.5;
            if !(reach > tol) { return Err(Unbuilt::Failed(format!("the tube road: the collar at {event:?} is read whole on no rim"))) }
        }
        tube.pinch = Pinch::new(collar,reach);
        let (folds,_) = edges(&tube,Chart::Collar,tol,feature)?;
        let tails: Vec<[f64;2]> = folds.iter().flat_map(|f| swallowtails_on(&tube,Chart::Collar,&f.uv)).collect();
        let least = tails.iter().map(|p| p[1].abs()).fold(reach,f64::min);
        tube.pinch = Pinch::new(fresh()?,0.5*least);
        if tube.pinch.is_none() { return Err(Unbuilt::Failed(format!("the tube road: the collar at {event:?} not read on its rim"))) }
        if std::env::var_os("SOLVENT_TUBE_DEBUG").is_some() {
            eprintln!("pinch at {event:?}: reach {reach}, the collar's swallowtail points {tails:?}, ρ₀ {}",0.5*least)
        }
        Ok(tube)
    }

    /// **The pinch's hole on a chart**, as its inner edges: on a sheet the rim `place(ψ, ±ρ₀)` read
    /// in `HOLE_RIM` chords; none on any other chart or where there is no pinch.
    pub fn hole(&self,chart: Chart) -> Result<Vec<Vec<[f64;2]>>,Unbuilt> {
        let (Chart::Sheet(side),Some(p)) = (chart,&self.pinch) else { return Ok(Vec::new()) };
        let line: Option<Vec<[f64;2]>> = (0..=HOLE_RIM).map(|i| p.rim(side,TAU*i as f64/HOLE_RIM as f64)).collect();
        let mut line = line.ok_or_else(|| Unbuilt::Failed("the tube road: the pinch's rim not read".into()))?;
        for j in 1..line.len() { line[j][0] = line[j-1][0]+(line[j][0]-line[j-1][0]+std::f64::consts::PI).rem_euclid(TAU)-std::f64::consts::PI }
        Ok(vec![line])
    }

    /// **The collar's point at `(ψ, ρ)`** with its rates in closed form: its place `(θ, t)`
    /// (`Collar::place`), the tool's point `c + r (cos ψ, sin ψ)` round the meridian at `θ` posed at
    /// `t`, and its rates by the chain rule through the place's — `D ∂(θ, t)/∂ψ = ρ (−cos ψ, −sin ψ)`
    /// and `D ∂(θ, t)/∂ρ = (−sin ψ, cos ψ)`, `D` the Jacobian of `(A, B)` by `(θ, t)`; its normal the
    /// tool's outward one there.
    pub fn collar_at(&self,psi: f64,rho: f64) -> Option<Point> { self.collar_parts(psi,rho).map(|p| p.0) }

    /// **A regular metric for the collar's cuspidal edges** (`Edge`'s on the collar, as a sheet's
    /// reads its spine's): the collar's own rates are parallel on its edges, so its point and, for
    /// rates, the tool's own way round its tube there posed and the collar's rate in `ρ`.
    pub fn collar_metric(&self,psi: f64,rho: f64) -> Option<(V,V,V)> {
        self.collar_parts(psi,rho).map(|(p,round)| (p.x,round,p.x_t))
    }

    /// The collar's point and rates (`collar_at`), and the tool's own rate round its tube posed.
    fn collar_parts(&self,psi: f64,rho: f64) -> Option<(Point,V)> {
        let p = self.pinch.as_ref()?;
        let [th,t] = p.collar.place(psi,rho)?;
        let [[d00,d01],[d10,d11]] = p.rates(th,t)?;
        let det = d00*d11-d01*d10;
        if !(det != 0.) { return None }
        let solve = |b0: f64,b1: f64| [(d11*b0-d01*b1)/det,(d00*b1-d10*b0)/det];
        let ((sp,cp),(s,c)) = (psi.dsin_cos(),th.dsin_cos());
        let f = &self.ring.frame;
        let (er,et,ez) = (f.dir([c,s,0.]),f.dir([-s,c,0.]),f.z);
        let nl = add(scale(er,cp),scale(ez,sp));
        let q = add(f.at([self.radius*c,self.radius*s,self.height]),scale(nl,self.r));
        let m = self.path.at(t).ok()?;
        let (xth,xt) = (m.vector(scale(et,self.radius+self.r*cp)),m.velocity(q));
        let xpsi = m.vector(scale(add(scale(er,-sp),scale(ez,cp)),self.r));
        let ([thp,tp],[thr,tr]) = (solve(-rho*cp,-rho*sp),solve(-sp,cp));
        Some((Point {x:m.point(q),x_th:add(xpsi,add(scale(xth,thp),scale(xt,tp))),x_t:add(scale(xth,thr),scale(xt,tr)),
            n:m.vector(nl),area:1.},xpsi))
    }

    /// **Which side of the collar can bound the sweep**: `−sgn(D) (X_ψ × X_ρ) · n` over the rates'
    /// lengths — the sign of the sheets' `J` at the place (the collar is one regular chart of both
    /// sheets, oriented against them by `D`), smooth through the meridian, where `J` is not read;
    /// nought on the collar's cuspidal edges.
    pub fn collar_side(&self,psi: f64,rho: f64) -> Option<f64> {
        let p = self.collar_at(psi,rho)?;
        let l = norm(p.x_th)*norm(p.x_t);
        (l > 0.).then(|| -self.pinch.as_ref().map_or(1.,|q| q.sense)*dot(cross(p.x_th,p.x_t),p.n)/l)
    }

    /// **The sheet's point for its net, the hole filled smoothly** (`rings::Hole::stand_in`'s rule
    /// on the tube): inside the hole, where `ρ = |(A, B)|` is below `ρ₀/2`, `S + s r (B e_r − A e_z)/h`
    /// with `h² = ρ² + (ρ₀/2)² β(2ρ/ρ₀)`, `β(x) = e^(1 − 1/(1 − x²))` — smooth through the pinch, the
    /// sheet itself from `ρ₀/2` out — so a net fitted over a window holding the hole fits the very
    /// sheet across its rim, which the faces cut it along; the sheet's point elsewhere.
    pub fn net_point(&self,side: f64,theta: f64,t: f64) -> Option<V> {
        if let Some(p) = &self.pinch {
            if p.side([theta,t]) < 0. {
                let [a,b] = p.collar.ab(theta,t)?;
                let rho = a.dhypot(b);
                let half = 0.5*p.rho;
                if rho < half {
                    let x = rho/half;
                    let h = (rho*rho+half*half*(1.-1./(1.-x*x)).exp()).sqrt();
                    let sp = self.spine(theta,t)?;
                    let m = self.path.at(t).ok()?;
                    let (s,c) = theta.dsin_cos();
                    let f = &self.ring.frame;
                    let nn = sub(scale(m.vector(f.dir([c,s,0.])),b),scale(m.vector(f.z),a));
                    return Some(add(sp.s,scale(nn,side*self.r/h)))
                }
            }
        }
        self.point(side,theta,t).map(|p| p.x)
    }

    /// The step of a chart's differences in each parameter: `ε^(1/3)` of an angle, of the roll, or
    /// of the collar's width (`THIRD_READ`).
    fn steps(&self,c: Chart) -> [f64;2] {
        let r = self.rect(c);
        [THIRD_READ,match c { Chart::End(_) => THIRD_READ,_ => THIRD_READ*(r[1][1]-r[1][0]) }]
    }

    /// The direction in a chart's parameters along which its rates cancel (`kernel`, on any chart).
    pub fn kernel_on(&self,c: Chart,uv: [f64;2]) -> Option<[f64;2]> {
        let p = self.at(c,uv)?;
        let (a,b,cc) = (dot(p.x_th,p.x_th),dot(p.x_th,p.x_t),dot(p.x_t,p.x_t));
        let k = if cc >= a { [cc,-b] } else { [-b,a] };
        let l = k[0].dhypot(k[1]);
        (l > 0.).then(|| [k[0]/l,k[1]/l])
    }

    /// The gradient of a chart's side (`Tube::side`) in its parameters, by central differences.
    pub fn side_gradient(&self,c: Chart,uv: [f64;2]) -> Option<[f64;2]> {
        let [h,k] = self.steps(c);
        let f = |a: f64,b: f64| self.side(c,[a,b]);
        Some([(f(uv[0]+h,uv[1])?-f(uv[0]-h,uv[1])?)/(2.*h),(f(uv[0],uv[1]+k)?-f(uv[0],uv[1]-k)?)/(2.*k)])
    }
}

/// **The swallowtail points on a chart's cuspidal edge** (`swallowtails`, read on any chart): where
/// the chart's kernel runs along the edge — `∇side · k` changing sign between two of its points —
/// each settled by Newton's method on `(side, ∇side · k) = 0` while its steps shrink.
pub fn swallowtails_on(tube: &Tube,chart: Chart,uv: &[[f64;2]]) -> Vec<[f64;2]> {
    let gauge = |p: [f64;2],sense: [f64;2]| -> Option<(f64,f64,[f64;2])> {
        let k = tube.kernel_on(chart,p)?;
        let k = if k[0]*sense[0]+k[1]*sense[1] < 0. { [-k[0],-k[1]] } else { k };
        let g = tube.side_gradient(chart,p)?;
        Some((tube.side(chart,p)?,g[0]*k[0]+g[1]*k[1],k))
    };
    let [h,ht] = tube.steps(chart);
    let mut out = Vec::new();
    let mut sense = [1.,0.];
    let mut last: Option<([f64;2],f64)> = None;
    for &p in uv {
        let Some((_,g,k)) = gauge(p,sense) else { last = None; continue };
        sense = k;
        if let Some((q,gq)) = last {
            if (g < 0.) != (gq < 0.) {
                let f = gq/(gq-g);
                let mut x = [q[0]+(p[0]-q[0])*f,q[1]+(p[1]-q[1])*f];
                let mut prev = f64::INFINITY;
                for _ in 0..32 {
                    let Some((j0,g0,_)) = gauge(x,sense) else { break };
                    let (Some(a),Some(b),Some(c),Some(d)) = (gauge([x[0]+h,x[1]],sense),gauge([x[0]-h,x[1]],sense),
                        gauge([x[0],x[1]+ht],sense),gauge([x[0],x[1]-ht],sense)) else { break };
                    let m = [[(a.0-b.0)/(2.*h),(c.0-d.0)/(2.*ht)],[(a.1-b.1)/(2.*h),(c.1-d.1)/(2.*ht)]];
                    let det = m[0][0]*m[1][1]-m[0][1]*m[1][0];
                    if !(det.abs() > 0.) { break }
                    let dx = [(j0*m[1][1]-g0*m[0][1])/det,(m[0][0]*g0-m[1][0]*j0)/det];
                    let step = dx[0].dhypot(dx[1]);
                    if !(step < prev) { break }
                    prev = step;
                    x = [x[0]-dx[0],x[1]-dx[1]];
                }
                out.push(x);
            }
        }
        last = Some((p,g));
    }
    out
}

/// **Where a trim crosses the pinch's rim beside a cuspidal edge's end**, as seeds: the rim is one
/// curve in `ψ`, read on the collar at `(ψ, ±ρ₀)` and on a sheet at `place(ψ, ±ρ₀)`, its rate in
/// space the collar's in `ψ`. From each edge's end on it, along it each way by steps doubling from
/// the tolerance in space out to the feature, the far cover's depth is read where the chart bounds
/// the sweep; where it stands below nought beside the edge and passes nought further on, its root
/// is halved to the rounding. About the pinch a double curve runs within a degree of the cuspidal
/// edge it folds beside (the forward's 0.6°, a sliver 2.3e-3 across, covered at most 6.5e-5 deep),
/// between the rim's samples, and is crossed only there.
pub fn beside_edges(tube: &Tube,chart: Chart,edges: &[Vec<[f64;2]>],tol: f64,feature: f64) -> Vec<[f64;2]> {
    let Some(pinch) = &tube.pinch else { return Vec::new() };
    let field = Depth {tube,chart,within:tol,holes:&[]};
    let rho = pinch.rho;
    let value = |uv: [f64;2]| -> Option<f64> { match field.reading(uv)?.1 { Reading::Far(c) => Some(c.depth),Reading::Alone => Some(f64::INFINITY),_ => None } };
    // (an end on the rim: its rim's sign and angle)
    let on = |q: [f64;2]| -> Option<(f64,f64)> { match chart {
        Chart::Collar => (q[1].abs() >= rho*(1.-HALF_READ)).then(|| (q[1].signum(),q[0])),
        Chart::Sheet(side) => (pinch.side(q).abs() <= HALF_READ*rho).then(|| pinch.psi(side,q).map(|psi| (side,psi)))?,
        Chart::End(_) => None,
    } };
    let rim = |sign: f64,psi: f64| -> Option<[f64;2]> { match chart { Chart::Collar => Some([psi,sign*rho]),_ => pinch.rim(sign,psi) } };
    let mut out = Vec::new();
    for e in edges {
        for q in [e[0],e[e.len()-1]] {
            let Some((sign,psi)) = on(q) else { continue };
            let Some(p) = tube.collar_at(psi,sign*rho) else { continue };
            let rate = norm(p.x_th);
            if !(rate > 0.) { continue }
            for way in [1.,-1.] {
                let at = |s: f64| rim(sign,psi+way*s/rate);
                let mut s = tol;
                let mut below: Option<f64> = None;
                while s <= feature {
                    let Some(x) = at(s) else { break };
                    if tube.side(chart,x).is_some_and(|v| v > 0.) {
                        match value(x) {
                            Some(v) if v < 0. => below = Some(s),
                            Some(_) => {
                                if let Some(a) = below {
                                    let (mut a,mut b) = (a,s);
                                    loop {
                                        let m = 0.5*(a+b);
                                        if !(m > a && m < b) { break }
                                        if at(m).and_then(value).is_some_and(|v| v < 0.) { a = m } else { b = m }
                                    }
                                    if let Some(x) = at(b) { out.push(x) }
                                }
                                break
                            }
                            None => break,
                        }
                    } else if below.is_some() { break }
                    s *= 2.;
                }
            }
        }
    }
    out
}
