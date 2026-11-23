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

use super::closed::Unbuilt;
use super::depth::{Cover,Reading};
use super::patches::Element;
use super::rings::Ring;
use crate::trajectory::Trajectory;
use crate::util::fmath::{Det,HALF_READ,THIRD_READ};
use crate::util::space::{add,cross,dot,norm,scale,sub};
use crate::zero_set::{Field,Value};
use std::f64::consts::TAU;

type V = [f64;3];

/// **A torus under a motion, read as its core circle's sweep and its tube**: the ring (its frame,
/// for depth), the motion and its roll, the core circle's radius and height in the ring's frame,
/// and the tube.
pub struct Tube<'a> { pub ring: &'a Ring,pub path: &'a Trajectory,pub roll: [f64;2],pub radius: f64,pub height: f64,pub r: f64 }

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
        Ok(Tube {ring,path,roll:[a.min(b),a.max(b)],radius:c[0],height:c[1],r})
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
        super::depth::cover(self.ring,self.path,self.roll,p.x,t,0.,within).ok()
    }
}

/// **A chart of the tube road**: a sheet `X±` over `(θ, t)` (`Sheet(±1)`), or the tool posed at
/// an end of the roll over `(θ, ψ)` (`End(0)` the start, `End(1)` the end), `ψ` round its tube from
/// the outward way about the ring's axis toward its own.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Chart { Sheet(f64),End(usize) }

impl Tube<'_> {
    /// The chart's rectangle of parameters.
    pub fn rect(&self,c: Chart) -> crate::zero_set::Rect {
        match c { Chart::Sheet(_) => [[0.,TAU],self.roll],Chart::End(_) => [[0.,TAU],[0.,TAU]] }
    }

    /// The instant a chart's point is its own at.
    pub fn own(&self,c: Chart,uv: [f64;2]) -> f64 {
        match c { Chart::Sheet(_) => uv[1],Chart::End(k) => self.roll[k] }
    }

    /// **The chart's point at `uv`**, its two rates and a unit normal: a sheet's (`point`, its
    /// spine's normal), or the posed tool's in closed form (its outward normal).
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
        }
    }

    /// **Which side of a chart can bound the sweep**, positive there: on a sheet its Jacobian `J`
    /// (beyond its cuspidal edge the instants beside hold it); on an end pose the part moving out
    /// of the sweep — at the start the part the motion leaves behind (`n · v < 0`), at the end the
    /// part leading (`n · v > 0`), read against the point's speed — the rest held by the instants
    /// inside the roll. Nought on its contact circles, the sheets' rims.
    pub fn side(&self,c: Chart,uv: [f64;2]) -> Option<f64> {
        match c {
            Chart::Sheet(side) => self.jacobian(side,uv[0],uv[1]),
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
pub struct Depth<'a> { pub tube: &'a Tube<'a>,pub chart: Chart,pub within: f64 }

impl Depth<'_> {
    /// The chart's point where it can bound the sweep, with its rates.
    fn kept(&self,uv: [f64;2]) -> Option<Point> {
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
        let r = super::depth::cover(self.tube.ring,self.tube.path,self.tube.roll,p.x,self.tube.own(self.chart,uv),0.,self.within).ok()?;
        Some((p,r))
    }
}

impl Field for Depth<'_> {
    type At = Cover;
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

    fn surface(&self,uv: [f64;2]) -> (V,V,V) {
        match self.tube.at(self.chart,uv) { Some(p) => (p.x,p.x_th,p.x_t),None => ([f64::NAN;3],[f64::NAN;3],[f64::NAN;3]) }
    }

    /// **The depth's bound over a cell**: a depth is 1-Lipschitz in space, so over the cell it lies
    /// within the cell's reach of its value at any point of it. Read at the cell's middle, corners
    /// and sides' middles (its reach from each the greatest distance to another, as `zero_set` reads
    /// a reach): the least of those covered by another instant (`Far`) less the reach. A point
    /// nothing else covers (`Alone`) has no far cover to bound, and where all nine are so the cell is
    /// read clear — sampled evidence, not proof: a cover born and dying between the nine is missed,
    /// as the search's sampling would miss it. None where a point is not kept or its neighbours hold
    /// it (the cell meets the chart's edge).
    fn bound(&self,c: crate::zero_set::Rect) -> Option<crate::zero_set::Bound> {
        let mut xs = Vec::with_capacity(9);
        let mut least = f64::INFINITY;
        for i in 0..3 { for j in 0..3 {
            let uv = [c[0][0]+(c[0][1]-c[0][0])*i as f64/2.,c[1][0]+(c[1][1]-c[1][0])*j as f64/2.];
            let (p,r) = self.reading(uv)?;
            match r {
                Reading::Far(c) => least = least.min(c.depth),
                Reading::Alone => {}
                Reading::Near => return None,
            }
            xs.push(p.x);
        } }
        let reach = xs.iter().flat_map(|x| xs.iter().map(move |y| crate::util::space::distance(*x,*y))).fold(0.,f64::max);
        let lo = if least.is_finite() { least-reach } else { reach };
        Some(crate::zero_set::Bound {value:crate::util::interval::Iv::new(lo,f64::MAX),grad:None})
    }
}

/// **Where a chart stops bounding the sweep, as a field**: `Tube::side` over its parameters — on a
/// sheet its cuspidal edge, on an end pose its contact circles. Its metric on a sheet is the
/// spine's (the sheet's own is singular on the edge), its gradient by differences.
pub struct Edge<'a> { pub tube: &'a Tube<'a>,pub chart: Chart }

impl Field for Edge<'_> {
    type At = ();
    fn least(&self,uv: [f64;2]) -> Option<Value<()>> {
        let grad = match self.chart { Chart::Sheet(side) => self.tube.jacobian_gradient(side,uv[0],uv[1]),Chart::End(_) => None };
        Some(Value {f:self.tube.side(self.chart,uv)?,grad,at:()})
    }
    fn along(&self,uv: [f64;2],_: &()) -> Option<Value<()>> { self.least(uv) }
    fn surface(&self,uv: [f64;2]) -> (V,V,V) {
        let s = match self.chart {
            Chart::Sheet(_) => self.tube.spine(uv[0],uv[1]).map(|s| (s.s,s.s_th,s.s_t)),
            Chart::End(_) => self.tube.at(self.chart,uv).map(|p| (p.x,p.x_th,p.x_t)),
        };
        s.unwrap_or(([f64::NAN;3],[f64::NAN;3],[f64::NAN;3]))
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
/// too (`zero_set::trace`) — the search halves a cell across its longer way in space, read between
/// its sides' ends, and over a whole turn those ends are one point — and the pieces of every part
/// returned (a curve across a cut in two, each ending on the rim there), with each part's search.
/// Inner edges (`edges`, polylines in the parameters) are given to the part holding them whole, all
/// with the one side function `side`; seeds to the part holding them, brought into the turn.
#[allow(clippy::too_many_arguments,clippy::type_complexity)]
pub fn round<F: Field>(field: &F,rect: crate::zero_set::Rect,periodic: [bool;2],seeds: &[[f64;2]],edges: &[Vec<[f64;2]>],
    side: &dyn Fn([f64;2]) -> f64,tol: f64,feature: f64) -> Result<(Vec<crate::zero_set::Piece<F::At>>,Vec<crate::zero_set::Search>),String> {
    let cut = |k: usize| -> Vec<[f64;2]> {
        let [a,b] = rect[k];
        if periodic[k] { vec![[a,0.5*(a+b)],[0.5*(a+b),b]] } else { vec![[a,b]] }
    };
    let wrap = |s: [f64;2]| std::array::from_fn::<f64,2,_>(|k| if periodic[k] { rect[k][0]+(s[k]-rect[k][0]).rem_euclid(rect[k][1]-rect[k][0]) } else { s[k] });
    let (mut pieces,mut searches) = (Vec::new(),Vec::new());
    for wu in cut(0) { for wv in cut(1) {
        let inside = |s: &[f64;2]| s[0] >= wu[0] && s[0] <= wu[1] && s[1] >= wv[0] && s[1] <= wv[1];
        let mine: Vec<[f64;2]> = seeds.iter().map(|&s| wrap(s)).filter(inside).collect();
        // (each inner edge a polyline of the part's, read at a fraction of its points' count)
        let lines: Vec<&Vec<[f64;2]>> = edges.iter().filter(|e| e.len() >= 2 && e.iter().all(inside)).collect();
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
        let traced = crate::zero_set::trace(field,&crate::zero_set::Question {rect:[wu,wv],edges:&inner,seeds:&mine,tol,feature})?;
        pieces.extend(traced.pieces);
        searches.push(traced.search);
    } }
    Ok((pieces,searches))
}

/// **Where a chart stops bounding the sweep**: `Tube::side`'s zero set over the chart's whole
/// rectangle (`round`) to `tol` in space (a sheet's spine's), `feature` its longest step — on a
/// sheet its cuspidal edges, on an end pose its contact circles.
pub fn edges(tube: &Tube,chart: Chart,tol: f64,feature: f64) -> Result<(Vec<crate::zero_set::Piece<()>>,Vec<crate::zero_set::Search>),Unbuilt> {
    let periodic = [true,matches!(chart,Chart::End(_))];
    round(&Edge {tube,chart},tube.rect(chart),periodic,&[],&[],&|_| 1.,tol,feature)
        .map_err(|why| Unbuilt::Failed(format!("the tube road's chart {chart:?}: where it stops bounding the sweep: {why}")))
}

/// **The depth's zero set over a chart** (`Depth`, `round`) to `tol`, `feature` its longest step,
/// the chart's own edges (`edges`) given as inner edges, `seeds` traced too: the curves where
/// another instant's cover of the chart begins, each point with its covering instant.
pub fn seams(tube: &Tube,chart: Chart,edges: &[Vec<[f64;2]>],seeds: &[[f64;2]],tol: f64,feature: f64)
    -> Result<(Vec<crate::zero_set::Piece<Cover>>,Vec<crate::zero_set::Search>),Unbuilt> {
    let periodic = [true,matches!(chart,Chart::End(_))];
    let side = |uv: [f64;2]| tube.side(chart,uv).unwrap_or(-1.);
    round(&Depth {tube,chart,within:tol},tube.rect(chart),periodic,seeds,edges,&side,tol,feature)
        .map_err(|why| Unbuilt::Failed(format!("the tube road's chart {chart:?}: its depth's zero set: {why}")))
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
