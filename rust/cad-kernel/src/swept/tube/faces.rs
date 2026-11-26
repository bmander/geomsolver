//! **The tube road's faces sewn** (#880 T1f): each chart's kept region as faces, their loops made
//! of the traced curves with both faces' parameters, sewn by construction (`sew::sewn`).
//!
//! **The trims.** Each sheet's depth zero set (`seams`) is read as runs by what covers it — another
//! instant inside the roll (a double curve of the sheets, each traced twice, once on each of its
//! preimages: the run whose own instant is the later kept, its partner read off its cover), or an
//! end pose (a double curve of the sheet and the posed tool, traced on the sheet alone: on the end
//! pose it is the image). Every end of a run is an event found exactly, and each is one vertex of
//! everything meeting there:
//! * a **rim crossing** — a run leaving the sheet's rim, where it meets its end pose's contact
//!   circle: solved on the rim, one curve read in two charts, as the root along it of the far
//!   cover's depth (`crossing`), and both charts given that point; its partner on the covering
//!   instant is where the partner preimage's cover passes to the end pose (a **switch**);
//! * a **swallowtail point** — the double curve's two preimages joined to it inside its disc along
//!   the swallowtail's normal form (`rings::joined_by_normal_form`, the ring road's rule);
//! * a **corner** — where a cuspidal edge reaches the roll's end (`corner`, the root of `J` along
//!   the rim): there the sheet's double curve with the end pose meets the pose's contact circle,
//!   every far cover within the rounding, and the run is joined to it straight in the sheet's
//!   parameters, each point of the join held within the tolerance of the end pose.
//!
//! **The faces.** Every chart is cut across its angle at two angles clear of every trim (the
//! windows), and a sheet from each swallowtail point straight to the rim the swallowtail opens away
//! from (so no face meets itself along its double curve); the rims are split at every vertex on
//! them. Each window's edges are walked as a planar graph, and each part kept where its inner point
//! bounds the sweep (`Tube::side` above nought, and no other instant holding it). A sheet's face
//! lies on a net fitted over its window as a map (`nurbs::fit_net`), an end pose's on its torus.
use super::{Chart,Depth,Disc,Tube,edges,seams,swallowtails,swallowtails_on};
use crate::geom::{Curve,Frame,Surface};
use crate::nurbs::BSpline;
use crate::swept::closed::Unbuilt;
use crate::swept::depth::{Cover,Reading};
use crate::topo::{Brep,Coedge,Edge,EdgeCurve,Face,Pcurve};
use crate::util::fmath::{Det,HALF_READ};
use crate::util::space::{distance,dot,sub};
use crate::zero_set::Field;
use std::f64::consts::{PI,TAU};
use std::sync::Arc;

type V = [f64;3];
type P = [f64;2];

/// How finely the road fits its curves and nets, as a part of the tolerance (structural: what
/// `upright` and the ring road fit to, so the sewing's gaps are the tolerance's).
const FIT: f64 = 0.1;
/// How finely a sheet's net laid along the pinch's rows is graded (`Lay`), as a part of the
/// tolerance (measured; a net laid square is graded to `FIT`): graded where it bends, a leaning net
/// meets its bar about the pinch over wider stretches than a square one, and its miss there leans
/// one way. The crossed tori's forward member, its leaning nets graded to `FIT`, read a volume
/// 2.3e-7 below cadrs's (1.4e-8 of it, past brep-bench's agreement of exact kernels; Open CASCADE
/// read its file 1.9e-7 below the file before), its nets laid square 3.5e-8 off; graded to a third
/// of `FIT`, 6e-9 off, its STEP file 6.1 MB where `FIT` wrote 5.2 MB (and the nets doubled whole
/// 24 MB, 5e-8 off).
const SHEET_FIT: f64 = 0.03;
/// The degree of every curve and net fitted.
const DEGREE: usize = 5;
/// The most points a fitted curve is read at (structural, as `upright`'s).
const MOST_FIT: usize = 4096;
/// Newton's steps a settling takes before it is read as not converging (structural, as
/// `zero_set`'s corrector's).
const CORRECTIONS: usize = 32;

fn debug() -> bool { std::env::var_os("SOLVENT_TUBE_DEBUG").is_some() }

fn failed(why: String) -> Unbuilt { Unbuilt::Failed(format!("the tube road: {why}")) }

impl Tube<'_> {
    /// **The end pose `k`'s parameters `(θ, ψ)` of a point on its torus**, in closed form (`ψ`
    /// round the tube from the outward way about the axis, as `Chart::End` reads it).
    pub fn on_end(&self,k: usize,x: V) -> Option<P> {
        let pose = self.path.pose(self.roll[k]).ok()?;
        let l = self.ring.frame.local(pose.inverse().point(x));
        let rho = l[0].dhypot(l[1]);
        Some([l[1].datan2(l[0]).rem_euclid(TAU),(l[2]-self.height).datan2(rho-self.radius)])
    }

    /// The sheet's parameters of its point `x`, from `uv` by Gauss–Newton on `X(θ, t) = x` (the
    /// sheet regular there), while the steps shrink.
    fn settled(&self,side: f64,x: V,mut uv: P) -> Option<P> {
        let mut prev = f64::INFINITY;
        for _ in 0..CORRECTIONS {
            let p = self.point(side,uv[0],uv[1])?;
            let r = sub(x,p.x);
            let (a,b,c) = (dot(p.x_th,p.x_th),dot(p.x_th,p.x_t),dot(p.x_t,p.x_t));
            let (g0,g1) = (dot(p.x_th,r),dot(p.x_t,r));
            let det = a*c-b*b;
            if !(det > 0.) { return None }
            let d = [(c*g0-b*g1)/det,(a*g1-b*g0)/det];
            let step = d[0].dhypot(d[1]);
            if !(step < prev) { break }
            prev = step;
            uv = [uv[0]+d[0],uv[1]+d[1]];
        }
        Some(uv)
    }

    /// **What covers the point `x` read at `c`, as a chart's parameters**: an end pose's, where the
    /// covering instant is an end of the roll (to its rounding); else the sheet whose point at the
    /// covering instant and the core circle's angle of the tool's point is the nearer, settled.
    pub fn partner(&self,x: V,c: &Cover) -> Option<(Chart,P)> {
        let [a,b] = self.roll;
        for (k,e) in [a,b].into_iter().enumerate() {
            if (c.at-e).abs() <= HALF_READ*(b-a) { return Some((Chart::End(k),self.on_end(k,x)?)) }
        }
        let l = self.ring.frame.local(c.point);
        let th = l[1].datan2(l[0]).rem_euclid(TAU);
        if let Some(uv) = self.in_collar(x,c) { return Some((Chart::Collar,uv)) }
        let side = [1.,-1.].into_iter().min_by(|&s,&r| {
            let d = |s: f64| self.point(s,th,c.at).map_or(f64::INFINITY,|p| distance(p.x,x));
            d(s).total_cmp(&d(r))
        })?;
        // (a place inside a band's seam: the band's chart of that sheet, settled there)
        if let Some((k,q)) = self.bands.iter().enumerate().find_map(|(k,b)| Some((k,b.holding([th,c.at])?))) {
            let chart = Chart::Band(k,side);
            return Some((chart,self.settled_on(chart,x,q)?))
        }
        Some((Chart::Sheet(side),self.settled(side,x,[th,c.at])?))
    }

    /// **The collar's parameters of a point `x` its cover `c` names inside the pinch's hole**: `ψ`
    /// round the tube at the tool's point in closed form, `ρ` from `(A, B)` at the place read
    /// square to it, settled by Gauss–Newton on the collar's point (`settled_on`); none where the
    /// cover's place lies outside the hole.
    fn in_collar(&self,x: V,c: &Cover) -> Option<P> {
        let p = self.pinch.as_ref()?;
        let l = self.ring.frame.local(c.point);
        let th = l[1].datan2(l[0]).rem_euclid(TAU);
        if !(p.side([th,c.at]) < 0.) { return None }
        let psi = (l[2]-self.height).datan2(l[0].dhypot(l[1])-self.radius).rem_euclid(TAU);
        let [a,b] = p.collar.ab(th,c.at)?;
        let (sp,cp) = psi.dsin_cos();
        self.settled_on(Chart::Collar,x,[psi,-a*sp+b*cp])
    }

    /// A chart's parameters of its point `x`, from `uv` by Gauss–Newton on its point (`settled`'s,
    /// on any chart), while the steps shrink.
    fn settled_on(&self,chart: Chart,x: V,mut uv: P) -> Option<P> {
        let mut prev = f64::INFINITY;
        for _ in 0..CORRECTIONS {
            let p = self.at(chart,uv)?;
            let r = sub(x,p.x);
            let (a,b,c) = (dot(p.x_th,p.x_th),dot(p.x_th,p.x_t),dot(p.x_t,p.x_t));
            let (g0,g1) = (dot(p.x_th,r),dot(p.x_t,r));
            let det = a*c-b*b;
            if !(det > 0.) { return None }
            let d = [(c*g0-b*g1)/det,(a*g1-b*g0)/det];
            let step = d[0].dhypot(d[1]);
            if !(step < prev) { break }
            prev = step;
            uv = [uv[0]+d[0],uv[1]+d[1]];
        }
        Some(uv)
    }

    /// **Where a chart's point `q` stands on the pinch's rim**: the rim's sign and the collar's
    /// angle there — on the collar at `ρ = ±ρ₀`, on a sheet where `|(A, B)| = ρ₀` to the rounding of
    /// `ρ₀` (where the tracer's exit settled it on the hole's side).
    fn on_hole(&self,chart: Chart,q: P) -> Option<(f64,f64)> {
        let p = self.pinch.as_ref()?;
        match chart {
            Chart::Collar => (q[1].abs() >= p.rho*(1.-HALF_READ)).then(|| (q[1].signum(),q[0])),
            Chart::Sheet(side) => if p.side(q).abs() <= HALF_READ*p.rho { p.psi(side,q).map(|psi| (side,psi)) } else { None },
            Chart::End(_) | Chart::Band(..) => None,
        }
    }

    /// **Where a cuspidal edge reaches the roll's end `k`**: the root of `J` along the rim from its
    /// traced end `th`, by Newton's method while the steps shrink.
    pub fn corner(&self,side: f64,k: usize,mut th: f64) -> Option<P> {
        let t = self.roll[k];
        let mut prev = f64::INFINITY;
        for _ in 0..CORRECTIONS {
            let (j,g) = (self.jacobian(side,th,t)?,self.jacobian_gradient(side,th,t)?);
            if !(g[0].abs() > 0.) { return None }
            let step = j/g[0];
            if !(step.abs() < prev) { break }
            prev = step.abs();
            th -= step;
        }
        Some([th.rem_euclid(TAU),t])
    }
}

/// **A run's crossing of its sheet's rim at the roll's end `k`**, solved on the rim — one curve read
/// in two charts, well-conditioned along it where neither chart's field is across it (the end pose
/// touches the sheet along its rim) — as the root in `θ` of the far cover's depth at the rim's point
/// continued from `c` (`Depth::along`), its rate the envelope theorem's; by Newton's method while
/// the steps shrink. The crossing's angle and its cover.
fn crossing(field: &Depth,k: usize,mut th: f64,mut c: Cover) -> Option<(f64,Cover)> {
    let t = field.tube.roll[k];
    let mut prev = f64::INFINITY;
    for _ in 0..CORRECTIONS {
        let v = field.along([th,t],&c)?;
        let g = v.grad?[0];
        if !(g.abs() > 0.) { return None }
        let step = v.f/g;
        c = v.at;
        if !(step.abs() < prev) { break }
        prev = step.abs();
        th -= step;
    }
    Some((th,c))
}

/// **A run's crossing of the pinch's rim**, solved on the rim — one curve read in three charts (the
/// collar's `ρ = ±ρ₀`, the sheet's `place(ψ, ±ρ₀)`) — as the root in `ψ` of the far cover's depth
/// on the collar at `(ψ, sign ρ₀)` continued from `c` (`Depth::along`), by Newton's method while the
/// steps shrink. The crossing's angle and its cover.
fn hole_crossing(tube: &Tube,sign: f64,mut psi: f64,mut c: Cover,tol: f64) -> Option<(f64,Cover)> {
    let rho = sign*tube.pinch.as_ref()?.rho;
    let field = Depth {tube,chart:Chart::Collar,within:tol,holes:&[]};
    let mut prev = f64::INFINITY;
    for _ in 0..CORRECTIONS {
        let v = field.along([psi,rho],&c)?;
        let g = v.grad?[0];
        if !(g.abs() > 0.) { return None }
        let step = v.f/g;
        c = v.at;
        if !(step.abs() < prev) { break }
        prev = step.abs();
        psi -= step;
    }
    Some((psi,c))
}

/// A vertex: its point, and its parameters on each chart it lies on (a chart twice where it is
/// a double curve's point of one sheet).
#[derive(Clone,Debug)]
struct Vertex { x: V,on: Vec<(Chart,P)> }

/// **A trim**: a run of a sheet's depth zero set, its points on its own chart and on its partner's
/// (each angle unwrapped along it), the vertices at its ends, and how it is read between them.
#[derive(Clone,Debug)]
struct Trim { own: Chart,other: Chart,uv: Vec<P>,partner: Vec<P>,ends: [usize;2],shape: Shape }

/// How a trim is read between its points: traced (each point settled on the double curve), the
/// swallowtail's normal form from a pair on a disc's rim to its point (`rings::normal_form`, its
/// offsets `d` and kernel's way, from the point's parameters on each chart `base`, a use on a band's
/// chart read in its sheet's and carried through the band, `through`), or straight in its sheet's
/// parameters to a corner on the end pose `k`.
#[derive(Clone,Debug)]
enum Shape { Traced,Normal { base: [P;2],d: [P;2],way: f64,through: [Option<usize>;2] },Straight { k: usize } }

impl Trim {
    /// The trim's point at `s` of `[0, 1]`: its two charts' parameters, settled where it is traced.
    fn at(&self,tube: &Tube,s: f64) -> Option<[P;2]> { self.at_held(tube,s,false) }

    /// `at`, the line a traced point is held on turning smoothly between the polyline's tangents on
    /// any chart where `smooth` (as on a band's always).
    fn at_held(&self,tube: &Tube,s: f64,smooth: bool) -> Option<[P;2]> {
        let n = self.uv.len();
        if s <= 0. { return Some([self.uv[0],self.partner[0]]) }
        if s >= 1. { return Some([self.uv[n-1],self.partner[n-1]]) }
        match self.shape {
            Shape::Traced => {
                // (its place along its own polyline by length in the chart's parameters; on a band's
                // chart by length in space — its `τ` stretched as a sinh, a run near the line moves
                // across the tube in a sliver of it, and a curve fitted evenly in its parameters'
                // length missed by 1.9e-5)
                let mut ls = vec![0.];
                if let Chart::Band(..) = self.own {
                    let xs: Vec<V> = self.uv.iter().map(|&q| tube.at(self.own,q).map_or([f64::NAN;3],|p| p.x)).collect();
                    for w in xs.windows(2) { ls.push(ls[ls.len()-1]+distance(w[0],w[1])) }
                } else {
                    for w in self.uv.windows(2) { ls.push(ls[ls.len()-1]+crate::plane2::dist(w[0],w[1])) }
                }
                let l = s*ls[n-1];
                let j = (0..n-1).find(|&j| ls[j+1] >= l).unwrap_or(n-2);
                let f = if ls[j+1] > ls[j] { (l-ls[j])/(ls[j+1]-ls[j]) } else { 0. };
                let lerp = |a: P,b: P| [a[0]+(b[0]-a[0])*f,a[1]+(b[1]-a[1])*f];
                let (u,p) = (lerp(self.uv[j],self.uv[j+1]),lerp(self.partner[j],self.partner[j+1]));
                let mut dir = [self.uv[j+1][0]-self.uv[j][0],self.uv[j+1][1]-self.uv[j][1]];
                // (on a band's chart the line the point is held on turns smoothly, between the
                // polyline's tangents at its points: its segments' own ways jump at each point, the
                // point settled there jumping along the curve by its chord's sag, which beside the
                // band's line is over the bar — a curve fitted through them missed by 3.1e-6)
                if smooth || matches!(self.own,Chart::Band(..)) {
                    let tangent = |m: usize| {
                        let (a,b) = (self.uv[m.saturating_sub(1)],self.uv[(m+1).min(n-1)]);
                        let d = [b[0]-a[0],b[1]-a[1]];
                        let l = d[0].hypot(d[1]);
                        if l > 0. { [d[0]/l,d[1]/l] } else { d }
                    };
                    let (t0,t1) = (tangent(j),tangent(j+1));
                    dir = [t0[0]+(t1[0]-t0[0])*f,t0[1]+(t1[1]-t0[1])*f];
                }
                settle(tube,(self.own,self.other),[u,p],dir)
            }
            Shape::Normal {base,d,way,through} => {
                let o = crate::swept::rings::normal_form(d,way)(1.-s);
                let at = |i: usize| { let p = [base[i][0]+o[i][0],base[i][1]+o[i][1]]; through[i].map_or(p,|k| tube.bands[k].chart_of(p)) };
                Some([at(0),at(1)])
            }
            Shape::Straight {k} => {
                let (q,c) = (self.uv[0],self.uv[n-1]);
                let u = [q[0]+(c[0]-q[0])*s,q[1]+(c[1]-q[1])*s];
                let e = tube.on_end(k,tube.at(self.own,u)?.x)?;
                let near = self.partner[0];
                Some([u,[unwrap(e[0],near[0]),unwrap(e[1],near[1])]])
            }
        }
    }
}

/// What an edge of the faces is, for its curve in space: a run traced, a sheet's rim at the roll's
/// end `k` between two angles, a cut at an angle across a sheet (between two instants), across an
/// end pose's band (between two angles round its tube, `ψ` unwrapped) or across the collar
/// (between its two rims), the pinch's rim on the sheet `sign` between two of the collar's
/// angles, or band `k`'s seam on the sheet `side` between two fractions round it.
#[derive(Clone,Debug)]
enum Kind { Trim(usize),Rim { side: f64,k: usize,th: [f64;2] },Across { chart: Chart,th: f64,v: [f64;2] },Hole { sign: f64,psi: [f64;2] },
    Seam { k: usize,side: f64,f: [f64;2] } }

/// An edge of the faces: what it is, its vertices, and its uses, each a chart and its polyline
/// there (in the chart's own parameters, an end pose's `ψ` unwrapped along it).
#[derive(Clone,Debug)]
struct Arc2 { kind: Kind,v: [usize;2],uses: Vec<(Chart,Vec<P>)> }

/// **A chart's windows' cuts**: two angles splitting the largest gap in angle the trims' uses on
/// it (and the swallowtail points on it, `points`) leave free, at its thirds — each chart its own,
/// so a trim on one chart round every angle with its partner round the rest on another is cut on
/// each (the bench's tumbling torus: its sheet `−`'s double curves with the end pose run near its
/// rim over 2.5 of a turn's angle, their partners on the end pose over the rest). Refused where no
/// gap is wider than a hundredth of a turn (structural: a trim round every angle is a road not
/// built).
fn windows(trims: &[Trim],points: &[(Chart,P)],chart: Chart) -> Result<[f64;2],Unbuilt> {
    let mut ths: Vec<f64> = trims.iter().flat_map(|t| [(t.own,&t.uv),(t.other,&t.partner)])
        .filter(|(c,_)| *c == chart).flat_map(|(_,l)| l.iter().map(|p| p[0].rem_euclid(TAU))).collect();
    ths.extend(points.iter().filter(|(c,_)| *c == chart).map(|(_,p)| p[0].rem_euclid(TAU)));
    widest(ths).ok_or_else(|| failed(format!("the trims leave no angle of {chart:?} free for the windows' cuts")))
}

/// **Two angles at the thirds of the widest gap `ths` (angles within a turn) leave**, in order within
/// the turn; a quarter and three quarters where there are none; none where no gap is wider than a
/// hundredth of a turn (structural).
fn widest(mut ths: Vec<f64>) -> Option<[f64;2]> {
    if ths.is_empty() { return Some([0.5*PI,1.5*PI]) }
    ths.sort_by(f64::total_cmp);
    let n = ths.len();
    let (gap,at) = (0..n).map(|i| { let next = if i+1 < n { ths[i+1] } else { ths[0]+TAU }; (next-ths[i],ths[i]) })
        .max_by(|p,q| p.0.total_cmp(&q.0)).expect("some angles");
    if gap < 0.01*TAU { return None }
    // (in order within the turn: the windows run from the first to the second and on round)
    let (x,y) = ((at+gap/3.).rem_euclid(TAU),(at+2.*gap/3.).rem_euclid(TAU));
    Some([x.min(y),x.max(y)])
}

/// **A chart's windows' cuts where its trims leave no angle free** (`docs/tube-trims-by-membership.md`
/// §3, the windows' refusal gone): two angles clear of its points alone — its swallowtail points, the
/// pinch's hole and every vertex on it — the cuts then crossing trims, each crossing a vertex
/// (`cut_across`).
fn crossing_windows(points: &[(Chart,P)],vertices: &[Vertex],chart: Chart) -> Result<[f64;2],Unbuilt> {
    let mut ths: Vec<f64> = points.iter().filter(|(c,_)| *c == chart).map(|(_,p)| p[0].rem_euclid(TAU)).collect();
    ths.extend(vertices.iter().flat_map(|v| v.on.iter()).filter(|(c,_)| *c == chart).map(|(_,p)| p[0].rem_euclid(TAU)));
    widest(ths).ok_or_else(|| failed(format!("the points leave no angle of {chart:?} free for the windows' cuts")))
}

/// **Every trim crossing the cut at `θ = c` on `chart` between `v0` and `v1`, split there** (as
/// `met_by_cut` splits one): each crossing of a trim's polyline on the chart (its own, or its
/// partner's) settled on the double curve with the angle held (`settle`), a vertex of both its
/// charts, the trim split into two. The crossings' places along the cut and their vertices, in order.
/// A join crossed is refused by name.
fn cut_across(tube: &Tube,chart: Chart,c: f64,[v0,v1]: [f64;2],trims: &mut Vec<Trim>,vertices: &mut Vec<Vertex>) -> Result<Vec<(f64,usize)>,Unbuilt> {
    let mut out: Vec<(f64,usize)> = Vec::new();
    let (lo,hi) = (v0.min(v1),v0.max(v1));
    loop {
        // (the first crossing found strictly inside a segment: its trim, polyline, segment, fraction)
        let mut found: Option<(usize,bool,usize,f64)> = None;
        'search: for (i,t) in trims.iter().enumerate() {
            for (mine,line,ch) in [(false,&t.uv,t.own),(true,&t.partner,t.other)] {
                if ch != chart { continue }
                for j in 0..line.len()-1 {
                    let (u,w) = (line[j],line[j+1]);
                    let cc = wrap(chart,c,u[0]);
                    if (u[0]-cc).abs() <= 1e-12 || (w[0]-cc).abs() <= 1e-12 || (u[0]-cc)*(w[0]-cc) > 0. { continue }
                    let f = (cc-u[0])/(w[0]-u[0]);
                    let v = u[1]+(w[1]-u[1])*f;
                    if !(v > lo && v < hi) { continue }
                    found = Some((i,mine,j,f));
                    break 'search
                }
            }
        }
        let Some((i,mine,j,f)) = found else { break };
        if !matches!(trims[i].shape,Shape::Traced) { return Err(failed(format!("a window's cut at {c} on {chart:?} meets a join"))) }
        let (k,u,w) = split_trim(tube,trims,vertices,(i,mine,j,f),&format!("a window's cut at {c}"))?;
        out.push((if mine { w[1] } else { u[1] },k));
    }
    Ok(out)
}

/// **Trim `i` split where a cut crosses it**: at the fraction `f` of segment `j` of its polyline on
/// the cut's chart (its own, or its partner's where `mine`), the crossing settled on the double curve
/// with the angle held on that chart (`settle`), a vertex of both its charts, and the trim split there
/// into two (the second pushed). The vertex and the crossing's places on the trim's own chart and its
/// partner's; `what` names the cut in a refusal.
fn split_trim(tube: &Tube,trims: &mut Vec<Trim>,vertices: &mut Vec<Vertex>,(i,mine,j,f): (usize,bool,usize,f64),what: &str)
    -> Result<(usize,P,P),Unbuilt> {
    let t = &trims[i];
    let lerp = |l: &Vec<P>| [l[j][0]+(l[j+1][0]-l[j][0])*f,l[j][1]+(l[j+1][1]-l[j][1])*f];
    let (u,w) = (lerp(&t.uv),lerp(&t.partner));
    let [u,w] = if mine {
        let [w,u] = settle(tube,(t.other,t.own),[w,u],[1.,0.]).ok_or_else(|| failed(format!("{what} not settled on a trim")))?; [u,w]
    } else { settle(tube,(t.own,t.other),[u,w],[1.,0.]).ok_or_else(|| failed(format!("{what} not settled on a trim")))? };
    let x = tube.at(t.own,u).ok_or_else(|| failed(format!("no chart where {what} meets a trim")))?.x;
    vertices.push(Vertex {x,on:vec![(t.own,u),(t.other,w)]});
    let k = vertices.len()-1;
    let mut first = t.clone();
    let mut second = t.clone();
    first.uv.truncate(j+1); first.uv.push(u);
    first.partner.truncate(j+1); first.partner.push(w);
    first.ends[1] = k;
    second.uv = std::iter::once(u).chain(t.uv[j+1..].iter().copied()).collect();
    second.partner = std::iter::once(w).chain(t.partner[j+1..].iter().copied()).collect();
    second.ends[0] = k;
    trims[i] = first;
    trims.push(second);
    Ok((k,u,w))
}

/// The charts, in the order their faces are read.
const CHARTS: [Chart;4] = [Chart::Sheet(1.),Chart::Sheet(-1.),Chart::End(0),Chart::End(1)];

/// A chart's windows' cuts, of each chart's.
fn cuts_of(cuts: &[(Chart,[f64;2])],chart: Chart) -> [f64;2] { cuts.iter().find(|(c,_)| *c == chart).expect("every chart's cuts").1 }

/// The chart's end-pose band: where the sheet `+`'s rim stands on the end pose `k` at `θ`, and
/// which way round the tube the band runs from it (`side` above nought there).
fn band(tube: &Tube,k: usize,th: f64) -> Option<(f64,f64,f64)> {
    let plus = tube.on_end(k,tube.point(1.,th,tube.roll[k])?.x)?[1];
    let minus = tube.on_end(k,tube.point(-1.,th,tube.roll[k])?.x)?[1];
    // (the way whose middle, between the two rims, is kept)
    let up = (minus-plus).rem_euclid(TAU);
    let s = tube.side(Chart::End(k),[th,plus+0.5*up])?;
    Some(if s > 0. { (plus,1.,up) } else { (plus,-1.,TAU-up) })
}

/// A chart traced (`solid`'s): its cuspidal edges, the discs about their swallowtail points,
/// and its depth's seams' runs, each point with what covers it.
struct Traced { folds: Vec<Vec<P>>,discs: Vec<super::Disc>,runs: Vec<(Vec<P>,Vec<Cover>)> }

/// **The closed solid**, the tube road's (the module's method), sewn to `tol`.
pub fn solid(tube: &Tube,tol: f64) -> Result<Brep,Unbuilt> {
    let feature = 0.25*tube.ring.thickness;
    let [a,b] = tube.roll;
    let on_rim = |t: f64| [a,b].iter().position(|&e| (t-e).abs() <= HALF_READ*(b-a));
    let mut vertices: Vec<Vertex> = Vec::new();
    let mut trims: Vec<Trim> = Vec::new();
    // (each swallowtail point, and the cuts from it: its chart, its point, its vertex)
    let mut tails: Vec<(Chart,P,usize)> = Vec::new();
    // (where there is a pinch the collar first: its runs' ends on the hole's rim seed the sheets';
    // each band after the sheets, both its charts: the sheets' runs' ends on its seam seed them)
    let mut charts = if tube.pinch.is_some() { vec![Chart::Collar,Chart::Sheet(1.),Chart::Sheet(-1.)] } else { vec![Chart::Sheet(1.),Chart::Sheet(-1.)] };
    for k in 0..tube.bands.len() { charts.extend([Chart::Band(k,1.),Chart::Band(k,-1.)]) }
    let mut on_hole: Vec<(f64,f64)> = Vec::new();
    // (each crossing of a band's seam: the band, the sheet's side, the fraction round the seam)
    let mut on_seam: Vec<(usize,f64,f64)> = Vec::new();
    // **each chart traced** — its cuspidal edges, their swallowtail points, its depth's seams and
    // their runs — from the seeds the charts read before it left on the pinch's rim
    let traced = |chart: Chart,seeds: Vec<P>| -> Result<Traced,Unbuilt> {
        let side = if let Chart::Sheet(s) = chart { s } else { 0. };
        let (folds,_) = edges(tube,chart,tol,feature)?;
        let folds: Vec<Vec<P>> = folds.into_iter().map(|p| p.uv).collect();
        let points: Vec<P> = folds.iter().flat_map(|f| if let Chart::Sheet(_) = chart { swallowtails(tube,side,f) } else { swallowtails_on(tube,chart,f) }).collect();
        if chart == Chart::Collar && !points.is_empty() { return Err(failed(format!("the collar holds swallowtail points {points:?}"))) }
        if debug() { eprintln!("tube: {chart:?} seeded on the pinch's rim at {seeds:?}") }
        let (pieces,_,discs) = seams(tube,chart,&folds,&points,&seeds,tol,feature)?;
        let field = Depth {tube,chart,within:tol,holes:&discs};
        let runs = merged(tube,chart,&field,pieces,tol);
        Ok(Traced {folds,discs,runs})
    };
    // (the seeds of a chart: what the charts before it left on the pinch's rim, on its side)
    let seeds_of = |chart: Chart,on_hole: &[(f64,f64)],on_seam: &[(usize,f64,f64)]| -> Vec<P> {
        // (a band's chart: the sheet's crossings of its seam, on its side)
        if let Chart::Band(k,side) = chart {
            return on_seam.iter().filter(|c| c.0 == k && c.1 == side).map(|c| tube.bands[k].seam_at(c.2)).collect()
        }
        let side = if let Chart::Sheet(s) = chart { s } else { 0. };
        on_hole.iter().filter(|h| side == 0. || h.0 == side).filter_map(|&(sign,psi)| match chart {
            Chart::Collar => tube.pinch.as_ref().map(|p| [psi,sign*p.rho]),_ => tube.pinch.as_ref().and_then(|p| p.rim(sign,psi)) }).collect()
    };
    // (the two sheets traced side by side once the collar is: a sheet's runs leave on the rim seeds
    // of its own side alone, so neither seeds the other, and the tracing is most of the road's time
    // — the crossed reverse member's two 3.7 and 3.4 s; what each leaves is read after in order, as
    // before, so the vertices and trims are numbered as they were)
    let mut sheets: Vec<Result<Traced,Unbuilt>> = Vec::new();
    for chart in charts {
        let side = match chart { Chart::Sheet(s) | Chart::Band(_,s) => s,_ => 0. };
        let Traced {folds,discs,runs} = if !matches!(chart,Chart::Sheet(_)) { traced(chart,seeds_of(chart,&on_hole,&on_seam))? } else {
            if sheets.is_empty() {
                let both = [Chart::Sheet(1.),Chart::Sheet(-1.)];
                let seeds: Vec<Vec<P>> = both.iter().map(|&c| seeds_of(c,&on_hole,&on_seam)).collect();
                sheets = crate::util::par::map(&[0,1],|&k| traced(both[k],seeds[k].clone()));
                sheets.reverse();
            }
            sheets.pop().expect("a sheet traced")?
        };
        let field = Depth {tube,chart,within:tol,holes:&discs};
        // the corners: each fold's end on a rim, settled there
        let mut corners: Vec<(usize,P)> = Vec::new();
        for f in &folds {
            for q in [f[0],f[f.len()-1]] {
                if !matches!(chart,Chart::Sheet(_)) { continue }
                let Some(k) = on_rim(q[1]) else { continue };
                let c = tube.corner(side,k,q[0]).ok_or_else(|| failed(format!("a corner of the sheet {side} at {q:?} not settled")))?;
                if corners.iter().any(|(_,d)| (d[0]-c[0]).abs() <= HALF_READ) { continue }
                let x = tube.point(side,c[0],c[1]).ok_or_else(|| failed("no sheet at a corner".into()))?.x;
                let e = tube.on_end(k,x).ok_or_else(|| failed("no end pose at a corner".into()))?;
                vertices.push(Vertex {x,on:vec![(chart,c),(Chart::End(k),e)]});
                corners.push((vertices.len()-1,c));
            }
        }
        // each piece cut into runs by what covers it, and the runs kept; each switch of what covers
        // it resolved once for both runs beside it (`switched`: its vertex, or none where it is the
        // partner run's rim crossing, read once every run's ends are — a switch between a sheet's
        // cover and the collar's is the partner's crossing of the pinch's rim, `rim_vertex`)
        // (an end pose's, a sheet's, or — its place inside the pinch's hole — the collar's)
        let in_hole = |c: &Cover| tube.pinch.as_ref().is_some_and(|p| {
            let l = tube.ring.frame.local(c.point);
            p.side([l[1].datan2(l[0]).rem_euclid(TAU),c.at]) < 0.
        });
        // (a cover whose place lies inside a band's seam: the band's)
        let in_band = |c: &Cover| {
            let l = tube.ring.frame.local(c.point);
            tube.band_holding([l[1].datan2(l[0]).rem_euclid(TAU),c.at]).is_some()
        };
        let class = |c: &Cover| on_rim(c.at).map_or(if in_hole(c) { 3 } else if in_band(c) { 4 } else { 2 },|k| k);
        let mut kept: Vec<(Vec<P>,Vec<Cover>,[Option<Option<usize>>;2])> = Vec::new();
        for (uv,at) in runs {
            let n = uv.len();
            let mut from = 0;
            let mut before = None;
            for i in 1..=n {
                if i < n && class(&at[i]) == class(&at[from]) { continue }
                let after = if i < n { Some(if class(&at[i-1]) >= 3 || class(&at[i]) >= 3 { None }
                    else { switched(tube,chart,(uv[i-1],&at[i-1]),(uv[i],&at[i]),&mut vertices,tol)? }) } else { None };
                let (u,c) = (uv[from..i].to_vec(),at[from..i].to_vec());
                let m = c.len()/2;
                // (a double curve of the sheets traced on both preimages: the later one's kept)
                let keep = class(&c[m]) < 2 || tube.own(chart,u[m]) > c[m].at;
                // (each end on the pinch's rim a vertex, kept or not: a kept trim's partner
                // switches chart there; and a seed of the charts read after)
                for (q,cv) in [(u[0],&c[0]),(u[u.len()-1],&c[c.len()-1])] {
                    let Some((sign,psi)) = tube.on_hole(chart,q) else { continue };
                    let j = rim_vertex(tube,&mut vertices,sign,psi,cv.clone(),tol)?;
                    if let Some((_,[psi,_])) = vertices[j].on.iter().find(|(ch,_)| *ch == Chart::Collar) { on_hole.push((sign,*psi)) }
                }
                // (each end on a band's seam a vertex too, and a seed of the band's chart)
                for (q,cv) in [(u[0],&c[0]),(u[u.len()-1],&c[c.len()-1])] {
                    let Some((k,f)) = tube.on_seam(chart,q) else { continue };
                    let j = seam_vertex(tube,&mut vertices,k,side,f,cv.clone(),tol)?;
                    if let Some((_,b)) = vertices[j].on.iter().find(|(ch,_)| *ch == Chart::Band(k,side)) { on_seam.push((k,side,tube.bands[k].fraction(*b))) }
                }
                if keep && !u.is_empty() { kept.push((u,c,[before,after])) }
                (before,from) = (after,i);
            }
        }
        // each run's partner, and its ends resolved
        for (uv,at,switch) in kept {
            let mut partner = Vec::with_capacity(uv.len());
            let mut other = None;
            for (q,c) in uv.iter().zip(&at) {
                let x = tube.at(chart,*q).ok_or_else(|| failed("no sheet at a trim's point".into()))?.x;
                let (ch,p) = tube.partner(x,c).ok_or_else(|| failed(format!("no partner of the trim's point {q:?}")))?;
                if other.is_some_and(|o| o != ch) { return Err(failed(format!("a trim's partner changes chart at {q:?}"))) }
                other = Some(ch);
                partner.push(p);
            }
            let other = other.ok_or_else(|| failed("an empty trim".into()))?;
            // (a run's end on the pinch's rim read twice — the tracer's seed there and its exit settled
            // there, one crossing: a trim does not run along the rim — the end's alone kept, so the way
            // the run leaves it is its own, not a rounding's)
            let (mut uv,mut partner) = (uv,partner);
            for e in 0..2 {
                while uv.len() > 2 {
                    let (i,j) = if e == 0 { (0,1) } else { (uv.len()-1,uv.len()-2) };
                    if tube.on_hole(chart,uv[i]).is_none() || tube.on_hole(chart,uv[j]).is_none() { break }
                    uv.remove(j);
                    partner.remove(j);
                }
            }
            // (each angle unwrapped along the run, and an end pose's ψ)
            for j in 1..uv.len() {
                uv[j][0] = wrap(chart,uv[j][0],uv[j-1][0]);
                partner[j][0] = wrap(other,partner[j][0],partner[j-1][0]);
                if let Chart::End(_) = other { partner[j][1] = unwrap(partner[j][1],partner[j-1][1]) }
            }
            trims.push(Trim {own:chart,other,uv,partner,ends:[usize::MAX;2],shape:Shape::Traced});
            let ti = trims.len()-1;
            for e in 0..2 {
                let t = &trims[ti];
                let i = if e == 0 { 0 } else { t.uv.len()-1 };
                // (the run's last cover, a switch at its start having put a point before it)
                let (q,c) = (t.uv[i],at[if e == 0 { 0 } else { at.len()-1 }].clone());
                match switch[e] {
                    Some(Some(v)) => { ended(&mut trims[ti],e,v,&vertices)?; continue }
                    Some(None) => continue,
                    None => {}
                }
                // a crossing of the pinch's rim
                if let Some((sign,psi)) = tube.on_hole(chart,q) {
                    let j = rim_vertex(tube,&mut vertices,sign,psi,c,tol)?;
                    let v = &vertices[j];
                    let own = v.on.iter().find(|(ch,_)| *ch == chart).map(|(_,p)| *p).expect("on its chart");
                    let other = trims[ti].other;
                    let p = v.on[2..].iter().find(|(ch,_)| *ch == other).map(|(_,p)| *p)
                        .ok_or_else(|| failed(format!("a crossing of the pinch's rim at {q:?} whose partner is not on {other:?}")))?;
                    let t = &mut trims[ti];
                    (t.uv[i],t.partner[i],t.ends[e]) = (own,p,j);
                    continue
                }
                // a crossing of a band's seam
                if let Some((k,f)) = tube.on_seam(chart,q) {
                    let j = seam_vertex(tube,&mut vertices,k,side,f,c,tol)?;
                    let v = &vertices[j];
                    let own = v.on.iter().find(|(ch,_)| *ch == chart).map(|(_,p)| *p).expect("on its chart");
                    let other = trims[ti].other;
                    let p = v.on[2..].iter().find(|(ch,_)| *ch == other).map(|(_,p)| *p)
                        .ok_or_else(|| failed(format!("a crossing of a band's seam at {q:?} whose partner is not on {other:?}")))?;
                    let t = &mut trims[ti];
                    (t.uv[i],t.partner[i],t.ends[e]) = (own,p,j);
                    continue
                }
                // a rim crossing
                if chart == Chart::Collar { return Err(failed(format!("a trim of the collar ends at {q:?}, at no event"))) }
                // (a run against the end pose `k` reaching its sheet's rim at that end: the corner. Beside
                // the rim the end pose stands from the sheet's point by half its depth's second rate
                // in the instant times the square of the instant's way to the end, nought only where
                // that rate is, on the cuspidal edge: the run reaches the rim at the corner alone, and
                // about it the sheet and the end pose touch, so each far cover there stands within the
                // rounding and the run's points stand on their double curve to it alone — whether the
                // trace stops at the cuspidal edge short of the rim or runs on to the rim beside the
                // corner is a rounding's. Its tail is the straight join to the corner (`cornered`)
                // wherever that join holds the end pose, taken back point by point from the rim.)
                let sheet = matches!(chart,Chart::Sheet(_));
                if let (true,Some(k),Chart::End(o)) = (sheet,on_rim(q[1]),trims[ti].other) {
                    if o == k {
                        let apart = |c: P| crate::plane2::dist([unwrap(c[0],q[0]),c[1]],q);
                        let Some(&(v,cq)) = corners.iter().min_by(|x,y| apart(x.1).total_cmp(&apart(y.1))) else {
                            return Err(failed(format!("a trim of the sheet {side} reaches its end pose's rim at {q:?}, at no corner")))
                        };
                        let c = [unwrap(cq[0],q[0]),cq[1]];
                        let n = trims[ti].uv.len();
                        // (the `m`-th of its points from the rim's; the join from the farthest it holds
                        // from, the point before the rim's at least)
                        let point = |m: usize| if e == 0 { m } else { n-1-m };
                        let mut cut = 1;
                        while cut+2 < n && join_holds(tube,side,k,trims[ti].uv[point(cut+1)],c,tol).is_ok() { cut += 1 }
                        let t = &mut trims[ti];
                        if e == 0 { t.uv.drain(..cut); t.partner.drain(..cut); } else { t.uv.truncate(n-cut); t.partner.truncate(n-cut) }
                        let (j,join) = cornered(tube,side,k,cq,v,&trims[ti],e,&mut vertices,tol)?;
                        trims[ti].ends[e] = j;
                        trims.push(join);
                        continue
                    }
                }
                if let (true,Some(k)) = (sheet,on_rim(q[1])) {
                    let (th,c) = crossing(&field,k,q[0],c).ok_or_else(|| failed(format!("a rim crossing from {q:?} not settled")))?;
                    let own = [th.rem_euclid(TAU),tube.roll[k]];
                    let x = tube.at(chart,own).ok_or_else(|| failed("no sheet at a rim crossing".into()))?.x;
                    let (ch,p) = tube.partner(x,&c).ok_or_else(|| failed("no partner of a rim crossing".into()))?;
                    let end = tube.on_end(k,x).ok_or_else(|| failed("no end pose at a rim crossing".into()))?;
                    vertices.push(Vertex {x,on:vec![(chart,own),(Chart::End(k),end),(ch,p)]});
                    let t = &mut trims[ti];
                    t.uv[i] = own;
                    t.partner[i] = p;
                    t.ends[e] = vertices.len()-1;
                    continue
                }
                // a swallowtail point's disc
                // (ending on the rim as `seams` reads it, the march's last step short of or past
                // it: within the disc's radius of it)
                if let Some(d) = discs.iter().find(|d| d.side([wrap(chart,q[0],d.at[0]),q[1]]).abs() <= d.radius) {
                    if !tails.iter().any(|(s,p,_)| *s == chart && *p == d.at) {
                        let x = tube.at(chart,d.at).ok_or_else(|| failed("no sheet at a swallowtail point".into()))?.x;
                        vertices.push(Vertex {x,on:vec![(chart,d.at)]});
                        tails.push((chart,d.at,vertices.len()-1));
                    }
                    let tail = tails.iter().find(|(s,p,_)| *s == chart && *p == d.at).expect("the tail").2;
                    let (j,join) = joined(tube,chart,d,&trims[ti],e,tail,&mut vertices,tol)?;
                    trims[ti].ends[e] = j;
                    trims.push(join);
                    continue
                }
                // a corner, joined straight
                if !sheet { return Err(failed(format!("a trim of {chart:?} ends at {q:?}, at no event"))) }
                let apart = |c: P| crate::plane2::dist([unwrap(c[0],q[0]),c[1]],q);
                let Some(&(v,cq)) = corners.iter().min_by(|x,y| apart(x.1).total_cmp(&apart(y.1))) else {
                    return Err(failed(format!("a trim of the sheet {side} ends at {q:?}, at no event")))
                };
                let Chart::End(k) = trims[ti].other else { return Err(failed(format!("a trim ends at {q:?} beside a corner, its partner no end pose"))) };
                let (j,join) = cornered(tube,side,k,cq,v,&trims[ti],e,&mut vertices,tol)?;
                trims[ti].ends[e] = j;
                trims.push(join);
            }
        }
    }
    met(tube,&mut trims,&vertices,tol,feature)?;
    // the switches: each the partner of a rim crossing on the switching trim's chart
    if debug() {
        for (i,t) in trims.iter().enumerate() { let n = t.uv.len(); eprintln!("tube: run {i} on {:?} against {:?}, ends {:?}, {:?} .. {:?}, partner {:?} .. {:?}; next {:?}, before last {:?}",t.own,t.other,t.ends,t.uv[0],t.uv[n-1],t.partner[0],t.partner[n-1],t.uv[1.min(n-1)],t.uv[n.saturating_sub(2)]) }
        for (i,v) in vertices.iter().enumerate() { eprintln!("tube: vertex {i} {:?}",v.on) }
    }
    for ti in 0..trims.len() {
        for e in 0..2 {
            if trims[ti].ends[e] != usize::MAX { continue }
            let t = &trims[ti];
            let q = t.uv[if e == 0 { 0 } else { t.uv.len()-1 }];
            let apart = |p: P| crate::plane2::dist([wrap(t.own,p[0],q[0]),p[1]],q);
            let near = |v: &Vertex| v.on.iter().filter(|(c,_)| *c == t.own).map(|(_,p)| apart(*p)).fold(f64::INFINITY,f64::min);
            let (v,_) = vertices.iter().enumerate().filter(|(_,v)| v.on.len() >= 3)
                .map(|(j,v)| (j,near(v))).min_by(|x,y| x.1.total_cmp(&y.1))
                .ok_or_else(|| failed(format!("a trim's cover switches at {q:?}, at no rim crossing")))?;
            ended(&mut trims[ti],e,v,&vertices)?;
        }
    }
    // (each trim's angles unwrapped along it once its ends are in place, and an end pose's ψ)
    for t in &mut trims {
        for j in 1..t.uv.len() {
            t.uv[j][0] = wrap(t.own,t.uv[j][0],t.uv[j-1][0]);
            t.partner[j][0] = wrap(t.other,t.partner[j][0],t.partner[j-1][0]);
            if let Chart::End(_) = t.own { t.uv[j][1] = unwrap(t.uv[j][1],t.uv[j-1][1]) }
            if let Chart::End(_) = t.other { t.partner[j][1] = unwrap(t.partner[j][1],t.partner[j-1][1]) }
        }
    }
    build(tube,&mut vertices,&mut trims,&tails,tol)
}

/// **The pieces of a chart's seams joined across the cuts `round` traces in**: two ends on one
/// cut are one point where they stand at one place to the rounding or — each the root along the cut
/// its own part's trace read — within the tolerance of one another in space (an end pose's `ψ`, a
/// whole turn too, read within its turn) — or, on one cut, where the cut between them stands within
/// the tolerance of the zero set, the far cover continued from one end (`Depth::along`) within it
/// at their middle and quarters: a crossing shallow to the cut is read on each side to the
/// tolerance in space, and its two roots along the cut stand farther apart than that (the forward
/// member's sheet `+` crossing `θ = 0` at `t` 1.50984 and 1.50986, 1.2e-5 apart, its two runs left
/// apart and the one's loose end sent to a corner).
fn merged(tube: &Tube,chart: Chart,field: &Depth,pieces: Vec<crate::zero_set::Piece<Cover>>,tol: f64) -> Vec<(Vec<P>,Vec<Cover>)> {
    let [a,b] = tube.roll;
    let runs: Vec<(Vec<P>,Vec<Cover>)> = pieces.into_iter().map(|p| (p.uv,p.at)).collect();
    let near = |p: P,q: P| match (tube.at(chart,p),tube.at(chart,q)) { (Some(x),Some(y)) => distance(x.x,y.x) <= tol,_ => false };
    let turn = |x: f64| (x+PI).rem_euclid(TAU)-PI;
    let (dv,rv) = match chart { Chart::End(_) => (turn as fn(f64) -> f64,HALF_READ),Chart::Sheet(_) => ((|x| x) as fn(f64) -> f64,HALF_READ*(b-a)),
        Chart::Collar => ((|x| x) as fn(f64) -> f64,HALF_READ*tube.pinch.as_ref().map_or(0.,|p| 2.*p.rho)),
        Chart::Band(..) => { let r = tube.rect(chart); ((|x| x) as fn(f64) -> f64,HALF_READ*(r[1][1]-r[1][0])) } };
    // (the segment between two points within the tolerance of the zero set, the far cover read on
    // from the one's at its middle and quarters: an angle, and an end pose's `ψ`, met within its turn)
    let band = |p: P,c: &Cover,q: P| [0.25,0.5,0.75].iter().all(|&f| {
        let r = [p[0]+turn(q[0]-p[0])*f,p[1]+dv(q[1]-p[1])*f];
        crate::zero_set::Field::along(field,r,c).is_some_and(|v| v.f.abs() <= tol)
    });
    // a run in the band of one kept before it — its two ends and its middle each joined to that
    // run's nearest point (in the parameters, within the feature in space) through the band — is
    // that run read again, and dropped: where the far cover meets the chart all but tangentially
    // the depth stands within the tolerance across a strip, and each sign change of it along an
    // inner edge there seeds a run of its own (the forward member's sheet `−`: one double curve
    // traced seven times, 2e-5 apart along `t`)
    let feature = 0.25*tube.ring.thickness;
    let apart = |p: P,q: P| turn(p[0]-q[0]).dhypot(dv(p[1]-q[1]));
    let again = |a: &(Vec<P>,Vec<Cover>),b: &(Vec<P>,Vec<Cover>)| [0,a.0.len()/2,a.0.len()-1].iter().all(|&i| {
        let p = a.0[i];
        let q = b.0.iter().copied().min_by(|x,y| apart(p,*x).total_cmp(&apart(p,*y))).expect("a run of points");
        let close = match (tube.at(chart,p),tube.at(chart,q)) { (Some(x),Some(y)) => distance(x.x,y.x) <= feature,_ => false };
        close && band(p,&a.1[i],q)
    });
    let mut kept: Vec<(Vec<P>,Vec<Cover>)> = Vec::new();
    for r in runs { if !kept.iter().any(|k| again(&r,k)) { kept.push(r) } }
    let mut runs = kept;
    let same = |p: P,c: &Cover,q: P| turn(p[0]-q[0]) == 0. && dv(p[1]-q[1]) == 0.
        || turn(p[0]-q[0]).abs() <= HALF_READ && (dv(p[1]-q[1]).abs() <= rv || near(p,q) || band(p,c,q));
    'merge: loop {
        for i in 0..runs.len() { for j in 0..runs.len() {
            if i == j { continue }
            let (pi,pj) = (runs[i].0[runs[i].0.len()-1],runs[j].0[0]);
            let ci = runs[i].1[runs[i].1.len()-1].clone();
            if same(pi,&ci,pj) {
                let (mut u,mut c) = runs.remove(j);
                let i = if j < i { i-1 } else { i };
                u.remove(0); c.remove(0);
                runs[i].0.extend(u); runs[i].1.extend(c);
                continue 'merge
            }
            let pj = runs[j].0[runs[j].0.len()-1];
            if i < j && same(pi,&ci,pj) {
                let (mut u,mut c) = runs.remove(j);
                u.reverse(); c.reverse();
                u.remove(0); c.remove(0);
                runs[i].0.extend(u); runs[i].1.extend(c);
                continue 'merge
            }
        } }
        break
    }
    runs
}

/// **The two end poses' meeting** (the old road's `depth::ends_met`): the start pose's seams
/// (`seams` on `Chart::End(0)`, its contact circles its inner edges) where the end pose covers it,
/// each run a trim of the start pose against the end pose, its partner the end pose's place of each
/// point (`Tube::on_end`, closed form). Each end of a run is an event found already, one vertex
/// standing on both poses: where it meets the start pose's contact circle — the sheet's rim at the
/// start — the sheet's rim crossing covered by the end pose; where the start pose's cover passes
/// to the end pose from the instants before it, the sheet's rim crossing at the end covered by the
/// start pose. Each is the vertex on both poses nearest the run's end, held within the trace's
/// longest step (`feature`) of it in space, its parameters put at that end (in place of the
/// trace's own reading of it, on the contact circle; before it, at a switch).
fn met(tube: &Tube,trims: &mut Vec<Trim>,vertices: &[Vertex],tol: f64,feature: f64) -> Result<(),Unbuilt> {
    let chart = Chart::End(0);
    let [a,b] = tube.roll;
    let (circles,_) = edges(tube,chart,tol,feature)?;
    let circles: Vec<Vec<P>> = circles.into_iter().map(|p| p.uv).collect();
    let (pieces,_,discs) = seams(tube,chart,&circles,&[],&[],tol,feature)?;
    let ended = |c: &Cover| (c.at-b).abs() <= HALF_READ*(b-a);
    let turn = |x: f64| (x+PI).rem_euclid(TAU)-PI;
    for (uv,at) in merged(tube,chart,&Depth {tube,chart,within:tol,holes:&discs},pieces,tol) {
        let n = uv.len();
        let mut from = 0;
        for i in 1..=n {
            if i < n && ended(&at[i]) == ended(&at[from]) { continue }
            if ended(&at[from]) {
                let mut u = uv[from..i].to_vec();
                let mut partner = Vec::with_capacity(u.len());
                for q in &u {
                    let x = tube.at(chart,*q).ok_or_else(|| failed("no start pose at its meeting with the end pose".into()))?.x;
                    partner.push(tube.on_end(1,x).ok_or_else(|| failed("no end pose at its meeting with the start pose".into()))?);
                }
                let mut ends = [usize::MAX;2];
                for e in 0..2 {
                    let j = if e == 0 { 0 } else { u.len()-1 };
                    let q = u[j];
                    let x = tube.at(chart,q).ok_or_else(|| failed("no start pose at a meeting's end".into()))?.x;
                    let apart = |p: P| turn(p[0]-q[0]).dhypot(turn(p[1]-q[1]));
                    let (v,on) = vertices.iter().enumerate()
                        .filter_map(|(v,w)| {
                            let p0 = w.on.iter().find(|(c,_)| *c == Chart::End(0))?.1;
                            let p1 = w.on.iter().find(|(c,_)| *c == Chart::End(1))?.1;
                            Some((v,[p0,p1]))
                        })
                        .min_by(|x,y| apart(x.1[0]).total_cmp(&apart(y.1[0])))
                        .ok_or_else(|| failed(format!("the end poses' meeting ends at {q:?}, at no vertex on both")))?;
                    if distance(vertices[v].x,x) > feature {
                        return Err(failed(format!("the end poses' meeting ends at {q:?}, {:e} from the nearest vertex on both",distance(vertices[v].x,x))))
                    }
                    // (on the contact circle the trace's end is the event read again: replaced; at a
                    // switch the event lies past the run's last point: put after it)
                    let switch = if e == 0 { from > 0 } else { i < n };
                    let (p0,p1) = (on[0],on[1]);
                    match (switch,e) {
                        (false,0) => { u[0] = p0; partner[0] = p1 }
                        (false,_) => { let m = u.len()-1; u[m] = p0; partner[m] = p1 }
                        (true,0) => { u.insert(0,p0); partner.insert(0,p1) }
                        (true,_) => { u.push(p0); partner.push(p1) }
                    }
                    ends[e] = v;
                }
                trims.push(Trim {own:chart,other:Chart::End(1),uv:u,partner,ends,shape:Shape::Traced});
            }
            from = i;
        }
    }
    Ok(())
}

/// **A switch of what covers a run**, between its points `a` and `b` on the sheet `chart`, one
/// covered by an instant inside the roll (by the sheet `B`'s point there), the other by the end
/// pose `k`: the triple point where `B`'s preimage of the double curve reaches its rim at `k` and
/// the cover passes to the end pose there — a rim crossing on `B`. Where the run covered by `B` is
/// the one kept (its own instant the later), `B`'s run is not, and the crossing is solved here, on
/// `B`'s rim (`crossing`, from `B`'s point covered at the run's own instant), the vertex returned;
/// else `B`'s run is kept and its own rim crossing is this switch (none returned: read once every
/// run's ends are). A cover passing from one end pose to the other is the end poses' meeting, not
/// yet read: refused by name.
fn switched(tube: &Tube,chart: Chart,a: (P,&Cover),b: (P,&Cover),vertices: &mut Vec<Vertex>,tol: f64) -> Result<Option<usize>,Unbuilt> {
    let [lo,hi] = tube.roll;
    let on_rim = |t: f64| [lo,hi].iter().position(|&e| (t-e).abs() <= HALF_READ*(hi-lo));
    let ((q,c),k) = match (on_rim(a.1.at),on_rim(b.1.at)) {
        (None,Some(k)) => (a,k),
        (Some(k),None) => (b,k),
        _ => return Err(failed(format!("a run's cover passes from one end pose to the other at {:?}: the end poses' meeting, not yet read",a.0))),
    };
    if !(q[1] > c.at) { return Ok(None) }
    let x = tube.at(chart,q).ok_or_else(|| failed("no sheet at a switch".into()))?.x;
    let (other,p) = tube.partner(x,c).ok_or_else(|| failed(format!("no partner of the switch's point {q:?}")))?;
    if let Chart::End(_) = other { return Err(failed(format!("a switch at {q:?} whose cover is no sheet"))) }
    // (`B`'s point covered at the run's own instant, the cover `crossing` continues)
    let y = tube.at(other,p).ok_or_else(|| failed("no sheet at a switch's partner".into()))?.x;
    let pose = tube.path.pose(q[1]).map_err(|_| failed("no pose at a switch".into()))?;
    let mine = Cover {depth:0.,at:q[1],point:pose.inverse().point(y),piece:0};
    // (no disc of `B`'s cut about the rim: a swallowtail point is inside its chart)
    let field = Depth {tube,chart:other,within:tol,holes:&[]};
    let (th,cover) = crossing(&field,k,p[0],mine).ok_or_else(|| failed(format!("the switch at {q:?}: its rim crossing not settled")))?;
    let own = [th.rem_euclid(TAU),tube.roll[k]];
    let y = tube.at(other,own).ok_or_else(|| failed("no sheet at a switch's rim crossing".into()))?.x;
    let (ch,pa) = tube.partner(y,&cover).ok_or_else(|| failed("no partner of a switch's rim crossing".into()))?;
    if ch != chart { return Err(failed(format!("the switch at {q:?}: its rim crossing's partner on {ch:?}"))) }
    let end = tube.on_end(k,y).ok_or_else(|| failed("no end pose at a switch's rim crossing".into()))?;
    vertices.push(Vertex {x:y,on:vec![(other,own),(Chart::End(k),end),(ch,pa)]});
    Ok(Some(vertices.len()-1))
}

/// **A trim's end `e` at its switch's vertex `v`**: the switch's own parameters on the trim's chart
/// (of the vertex's places there, the nearest the run's end) and its partner the vertex's place on
/// the trim's partner (another of them where that is the same chart), put at that end.
fn ended(t: &mut Trim,e: usize,v: usize,vertices: &[Vertex]) -> Result<(),Unbuilt> {
    let q = t.uv[if e == 0 { 0 } else { t.uv.len()-1 }];
    let apart = |p: P| crate::plane2::dist([wrap(t.own,p[0],q[0]),p[1]],q);
    let on = &vertices[v].on;
    let (j,own) = on.iter().enumerate().filter(|(_,(c,_))| *c == t.own).map(|(j,(_,p))| (j,*p))
        .min_by(|x,y| apart(x.1).total_cmp(&apart(y.1))).ok_or_else(|| failed(format!("a switch at {q:?} not on the trim's chart")))?;
    let other = on.iter().enumerate().find(|(i,(c,_))| *i != j && *c == t.other).map(|(_,(_,p))| *p)
        .ok_or_else(|| failed("a switch whose crossing is not on the trim's partner".into()))?;
    if e == 0 { t.uv.insert(0,own); t.partner.insert(0,other) } else { t.uv.push(own); t.partner.push(other) }
    t.ends[e] = v;
    Ok(())
}

/// **The vertex where a run crosses the pinch's rim** (`hole_crossing`, from the collar's angle
/// `psi` and the run's cover `c` there): on the collar, on the sheet `sign` and on its partner —
/// one crossing read once, a vertex already at its point (within the tolerance) taken.
fn rim_vertex(tube: &Tube,vertices: &mut Vec<Vertex>,sign: f64,psi: f64,c: Cover,tol: f64) -> Result<usize,Unbuilt> {
    let p = tube.pinch.as_ref().ok_or_else(|| failed("a rim crossing with no pinch".into()))?;
    let (psi,c) = hole_crossing(tube,sign,psi,c,tol).ok_or_else(|| failed(format!("a crossing of the pinch's rim from ψ {psi} not settled")))?;
    let (psi,rho) = (psi.rem_euclid(TAU),sign*p.rho);
    let x = tube.collar_at(psi,rho).ok_or_else(|| failed("no collar at a crossing of its rim".into()))?.x;
    if let Some(j) = vertices.iter().position(|v| v.on.iter().any(|(ch,q)| *ch == Chart::Collar && q[1] == rho) && distance(v.x,x) <= tol) { return Ok(j) }
    let place = p.rim(sign,psi).ok_or_else(|| failed("the pinch's rim not read at a crossing".into()))?;
    let (ch,pp) = tube.partner(x,&c).ok_or_else(|| failed(format!("no partner of a crossing of the pinch's rim at ψ {psi}")))?;
    vertices.push(Vertex {x,on:vec![(Chart::Collar,[psi,rho]),(Chart::Sheet(sign),place),(ch,pp)]});
    Ok(vertices.len()-1)
}

/// **A run's crossing of a band's seam**, solved on the seam — one curve read in two charts (the band's
/// `seam_at(f)`, the sheet's at its place) — as the root in the fraction `f` round it of the far
/// cover's depth on the band's chart continued from `c` (`Depth::along`), its rate by differences a
/// cube root of the rounding apart, by Newton's method while the steps shrink (`hole_crossing`'s rule
/// on the pinch's rim). The crossing's fraction and its cover.
fn seam_crossing(tube: &Tube,k: usize,side: f64,mut f: f64,mut c: Cover,tol: f64) -> Option<(f64,Cover)> {
    let b = &tube.bands[k];
    let field = Depth {tube,chart:Chart::Band(k,side),within:tol,holes:&[]};
    let h = crate::util::fmath::THIRD_READ;
    let mut prev = f64::INFINITY;
    for _ in 0..CORRECTIONS {
        let v = field.along(b.seam_at(f),&c)?;
        let g = (field.along(b.seam_at(f+h),&v.at)?.f-field.along(b.seam_at(f-h),&v.at)?.f)/(2.*h);
        if !(g.abs() > 0.) { return None }
        let step = v.f/g;
        c = v.at;
        if !(step.abs() < prev) { break }
        prev = step.abs();
        f -= step;
    }
    Some((f.rem_euclid(1.),c))
}

/// **The vertex where a run crosses a band's seam** (`seam_crossing`, from the fraction `f` round it
/// and the run's cover `c` there): on the band's chart, on the sheet `side` and on its partner — one
/// crossing read once, a vertex already at its point (within the tolerance) taken.
fn seam_vertex(tube: &Tube,vertices: &mut Vec<Vertex>,k: usize,side: f64,f: f64,c: Cover,tol: f64) -> Result<usize,Unbuilt> {
    let (f,c) = seam_crossing(tube,k,side,f,c,tol).ok_or_else(|| failed(format!("a crossing of band {k}'s seam from {f} not settled")))?;
    let b = &tube.bands[k];
    let (chart,q) = (Chart::Band(k,side),b.seam_at(f));
    let x = tube.at(chart,q).ok_or_else(|| failed("no band at a crossing of its seam".into()))?.x;
    if let Some(j) = vertices.iter().position(|v| v.on.iter().any(|(ch,r)| *ch == chart && b.seam(*r).abs() <= HALF_READ) && distance(v.x,x) <= tol) { return Ok(j) }
    let place = b.place(q);
    let (ch,pp) = tube.partner(x,&c).ok_or_else(|| failed(format!("no partner of a crossing of band {k}'s seam at {f}")))?;
    vertices.push(Vertex {x,on:vec![(chart,q),(Chart::Sheet(side),[place[0].rem_euclid(TAU),place[1]]),(ch,pp)]});
    Ok(vertices.len()-1)
}

/// **A trim joined to its swallowtail point inside its disc** (`rings::joined_by_normal_form`, the
/// ring road's rule): from its end `e` on the disc's rim, with its partner there, to the point (the
/// vertex `tail`) — a trim of its own along the normal form, from a vertex at that end, returned.
fn joined(tube: &Tube,chart: Chart,d: &Disc,t: &Trim,e: usize,tail: usize,vertices: &mut Vec<Vertex>,tol: f64) -> Result<(usize,Trim),Unbuilt> {
    let i = if e == 0 { 0 } else { t.uv.len()-1 };
    let (q0,r0) = (t.uv[i],t.partner[i]);
    let side = match chart { Chart::Sheet(s) | Chart::Band(_,s) => s,_ => 0. };
    let band_of = |c: Chart| if let Chart::Band(k,_) = c { Some(k) } else { None };
    // **the join read in the sheet's parameters** (a use on a band's chart carried to its place and
    // back: `Shape::Normal`'s `through`), and on a band's chart, where that does not hold, **in the
    // band's own** — beside the band's line neither is the sheet's own map's: the sheet's turns
    // fast across the line, the band's `τ` is stretched as a sinh, and the normal form's second order
    // with each (the reverse member's band's point at (6.61, 0.027) held by its own chart alone, its
    // leaves 4.6e-6 apart in the sheet's)
    let attempt = |sheet: bool| -> Option<([P;2],[P;2],f64,[Option<usize>;2],V)> {
        let through = if sheet { [band_of(t.own),band_of(t.other)] } else { [None,None] };
        let to = |k: Option<usize>,p: P| k.map_or(p,|k| tube.bands[k].place(p));
        let (q,r) = (to(through[0],q0),to(through[1],r0));
        let at = if sheet { to(band_of(chart),d.at) } else { d.at };
        let off = |q: P| [if sheet { (q[0]-at[0]+PI).rem_euclid(TAU)-PI } else { q[0]-at[0] },q[1]-at[1]];
        let pair = [off(q),off(r)];
        let k = if sheet { tube.kernel(side,at[0],at[1]) } else { tube.kernel_on(chart,at) }?;
        let x = |q: P| if sheet { tube.point(side,q[0],q[1]).map(|p| p.x) } else { tube.at(chart,q).map(|p| p.x) };
        let xq = x(q)?;
        let span = distance(xq,x(at)?);
        let outer = |_: usize,o: P,_: f64| -> Option<V> {
            let q = [at[0]+o[0],at[1]+o[1]];
            (if sheet { tube.jacobian(side,q[0],q[1]) } else { tube.side(chart,q) }? > 0.).then(|| x(q))?
        };
        crate::swept::rings::joined_by_normal_form(pair,k[1]/k[0],span,(8.*tol*tube.r).sqrt(),&outer,tol)?;
        // (the point's angle unwrapped as the run's, on each chart)
        let base = [[q[0]-pair[0][0],at[1]],[r[0]-pair[1][0],at[1]]];
        Some((base,pair,k[1]/k[0],through,xq))
    };
    let (base,pair,way,through,xq) = attempt(true).or_else(|| band_of(chart).and_then(|_| attempt(false)))
        .ok_or_else(|| failed(format!("a double curve not joined to its swallowtail point {:?}",d.at)))?;
    vertices.push(Vertex {x:xq,on:vec![(t.own,q0),(t.other,r0)]});
    let junction = vertices.len()-1;
    let mut join = Trim {own:t.own,other:t.other,uv:vec![],partner:vec![],ends:[junction,tail],shape:Shape::Normal {base,d:pair,way,through}};
    let ends = [0,1].map(|i| through[i].map_or(base[i],|k| tube.bands[k].chart_of(base[i])));
    join.uv = vec![q0,ends[0]];
    join.partner = vec![r0,ends[1]];
    let line: Vec<[P;2]> = (0..=16).map(|j| join.at(tube,j as f64/16.)).collect::<Option<_>>().ok_or_else(|| failed("a join not read".into()))?;
    join.uv = line.iter().map(|p| p[0]).collect();
    join.partner = line.iter().map(|p| p[1]).collect();
    Ok((junction,join))
}

/// **A trim joined to a corner** of its sheet at the roll's end `k` (its vertex `corner`, at `c`),
/// from its end `e` straight in the sheet's parameters — about the corner every far cover stands
/// within the rounding, the run all but tangent to the rim, so the trace cannot reach it — a trim of
/// its own from a vertex at that end, returned: as many chords as the double curve's longest step
/// (`√(8 tol r)`) takes, each point and chord's middle held within `tol` of the end pose and on the
/// side of the cuspidal edge the sheet bounds; its partner each point's on the end pose.
#[allow(clippy::too_many_arguments)]
fn cornered(tube: &Tube,side: f64,k: usize,c: P,corner: usize,t: &Trim,e: usize,vertices: &mut Vec<Vertex>,tol: f64) -> Result<(usize,Trim),Unbuilt> {
    let i = if e == 0 { 0 } else { t.uv.len()-1 };
    let (q,r) = (t.uv[i],t.partner[i]);
    let c = [q[0]+(c[0]-q[0]+PI).rem_euclid(TAU)-PI,c[1]];
    let x = |p: P| tube.point(side,p[0],p[1]).map(|p| p.x);
    let xq = x(q).ok_or_else(|| failed("no sheet at a trim's end".into()))?;
    let n = join_holds(tube,side,k,q,c,tol)?;
    vertices.push(Vertex {x:xq,on:vec![(t.own,q),(t.other,r)]});
    let junction = vertices.len()-1;
    let mut join = Trim {own:t.own,other:t.other,uv:vec![q,c],partner:vec![r,r],ends:[junction,corner],shape:Shape::Straight {k}};
    let line: Vec<[P;2]> = (0..=2*n).map(|j| join.at(tube,j as f64/(2*n) as f64)).collect::<Option<_>>().ok_or_else(|| failed("a join not read".into()))?;
    join.uv = line.iter().map(|p| p[0]).collect();
    join.partner = line.iter().map(|p| p[1]).collect();
    // (its last point the corner's own on the end pose, the vertex's)
    let n = join.partner.len();
    let e = vertices[corner].on.iter().find(|(ch,_)| *ch == Chart::End(k)).map(|(_,p)| *p).expect("a corner on its end pose");
    join.partner[n-1] = [unwrap(e[0],join.partner[n-2][0]),unwrap(e[1],join.partner[n-2][1])];
    Ok((junction,join))
}

/// **Whether the straight join from `q` to the corner `c`** of the sheet `side` at the roll's end
/// `k` holds (`cornered`'s): its chords' count, each point and chord's middle within `tol` of the
/// end pose and, short of the corner, on the side of the cuspidal edge the sheet bounds.
fn join_holds(tube: &Tube,side: f64,k: usize,q: P,c: P,tol: f64) -> Result<usize,Unbuilt> {
    let x = |p: P| tube.point(side,p[0],p[1]).map(|p| p.x);
    let span = distance(x(q).ok_or_else(|| failed("no sheet at a trim's end".into()))?,x(c).ok_or_else(|| failed("no sheet at a corner".into()))?);
    let n = (span/(8.*tol*tube.r).sqrt()).ceil().max(1.) as usize;
    for j in 1..=2*n {
        let f = j as f64/(2*n) as f64;
        let p = [q[0]+(c[0]-q[0])*f,q[1]+(c[1]-q[1])*f];
        let y = x(p).ok_or_else(|| failed("no sheet on a corner's join".into()))?;
        let (depth,_) = tube.depth_at(y,tube.roll[k]).ok_or_else(|| failed("no depth on a corner's join".into()))?;
        if depth.abs() > tol || (j < 2*n && !(tube.jacobian(side,p[0],p[1]).is_some_and(|j| j > 0.))) {
            return Err(failed(format!("a trim not joined to its corner {c:?}: {depth:e} off the end pose at {p:?}")))
        }
    }
    Ok(n)
}

/// **A curve through `f` over `[0, 1]`** with its uses' parameters on the same parameter:
/// interpolated at evenly spaced parameters doubled until it holds `f` within `fit` at their
/// quarters, and each use's parameters read through its chart within `fit` there too.
fn fitted(f: &dyn Fn(f64) -> (V,Vec<P>),at: &dyn Fn(usize,P) -> Option<V>,bars: &[f64]) -> Result<(BSpline,Vec<BSpline>),Unbuilt> {
    let fit = bars[0];
    let mut n = 8;
    loop {
        let ts: Vec<f64> = (0..=n).map(|i| i as f64/n as f64).collect();
        let reads: Vec<(V,Vec<P>)> = ts.iter().map(|&t| f(t)).collect();
        let pts: Vec<V> = reads.iter().map(|r| r.0).collect();
        if pts.iter().flatten().any(|x| !x.is_finite()) { return Err(failed("a curve could not be read".into())) }
        let curve = crate::nurbs::interpolate(&pts,&ts,DEGREE).ok_or_else(|| failed("a curve could not be interpolated".into()))?;
        let uses = reads[0].1.len();
        let pcs: Vec<BSpline> = (0..uses).map(|u| crate::nurbs::interpolate(&reads.iter().map(|r| [r.1[u][0],r.1[u][1],0.]).collect::<Vec<_>>(),&ts,DEGREE))
            .collect::<Option<_>>().ok_or_else(|| failed("a pcurve could not be interpolated".into()))?;
        let mut worst = 0f64;
        for i in 0..n { for q in [0.25,0.5,0.75] {
            let t = (i as f64+q)/n as f64;
            let y = curve.point(t);
            worst = worst.max(distance(y,f(t).0));
            for (u,pc) in pcs.iter().enumerate() {
                let p = pc.point(t);
                // (each use against its own bar, read as the fit's share of it)
                worst = worst.max(at(u,[p[0],p[1]]).map_or(f64::INFINITY,|z| distance(z,y))*fit/bars[u]);
            }
        } }
        if worst <= fit { return Ok((curve,pcs)) }
        if n >= MOST_FIT {
            if debug() {
                for i in 0..n { for q in [0.25,0.5,0.75] {
                    let t = (i as f64+q)/n as f64;
                    let y = curve.point(t);
                    let e0 = distance(y,f(t).0);
                    let es: Vec<f64> = pcs.iter().enumerate().map(|(u,pc)| { let p = pc.point(t); at(u,[p[0],p[1]]).map_or(f64::INFINITY,|z| distance(z,y))*fit/bars[u] }).collect();
                    if e0 > fit || es.iter().any(|&e| e > fit) { eprintln!("tube: misfit at {t}: {e0:e} {es:?} {:?}",f(t).1) }
                } }
            }
            return Err(failed(format!("a curve fits within {fit:e} only to {worst:e}")))
        }
        n *= 2;
    }
}

/// **A band's trim's polyline refined** until its curve stands within `bar` of each chord's middle in
/// space: each segment's middle settled on the double curve (`settle`) and put in where it stands
/// off the chord by more, at most `REFINED` times over. A trim is read by its length along its
/// polyline (`Trim::at`), and where the polyline is a coarse chord of a curve bending fast — beside
/// a band's line — its speed jumps at each of its points: a curve fitted through it missed by
/// 3.2e-6, its second differences ten times over at each point. None on any other chart, or where
/// nothing is put in.
fn refined(tube: &Tube,t: &Trim,bar: f64) -> Option<Trim> {
    if !matches!((t.own,&t.shape),(Chart::Band(..),Shape::Traced)) { return None }
    let x = |q: P| tube.at(t.own,q).map(|p| p.x);
    let on = |u: P,w: P| x(u).zip(tube.at(t.other,w).map(|p| p.x)).is_some_and(|(a,b)| distance(a,b) <= 1e-3*bar);
    let mut out = t.clone();
    let mut more = false;
    // (each point between its ends settled on the double curve first, held square to its neighbours'
    // chord, where its two charts' points stand apart by more than a thousandth of the bar: a point
    // whose partner was read beside a fold stood 2e-5 across the curve on the band's chart, and the
    // trim read by its length along the polyline jumped along the curve there)
    let n = out.uv.len();
    for j in 1..n.saturating_sub(1) {
        if on(out.uv[j],out.partner[j]) { continue }
        let (p,q) = (out.uv[j-1],out.uv[j+1]);
        let Some([u,w]) = settle(tube,(t.own,t.other),[out.uv[j],out.partner[j]],[q[0]-p[0],q[1]-p[1]]) else { continue };
        if on(u,w) && crate::plane2::dist(u,out.uv[j]) <= 0.25*crate::plane2::dist(p,q) { (out.uv[j],out.partner[j]) = (u,w); more = true }
    }
    for _ in 0..REFINED {
        let (mut uv,mut partner) = (vec![out.uv[0]],vec![out.partner[0]]);
        let mut put = false;
        for j in 0..out.uv.len()-1 {
            let (a,b) = (out.uv[j],out.uv[j+1]);
            let mid = |p: P,q: P| [0.5*(p[0]+q[0]),0.5*(p[1]+q[1])];
            let (u,w) = (mid(a,b),mid(out.partner[j],out.partner[j+1]));
            if let (Some(xa),Some(xb),Some([u,w])) = (x(a),x(b),settle(tube,(t.own,t.other),[u,w],[b[0]-a[0],b[1]-a[1]])) {
                if let Some(xm) = x(u) {
                    // (a middle settled on the curve beside its chord: within a quarter of the chord
                    // of its middle, in the parameters and in space — none settled on another branch —
                    // and its two charts' points one to a thousandth of the bar)
                    let off = distance(xm,crate::util::space::scale(crate::util::space::add(xa,xb),0.5));
                    let (m0,c) = (mid(a,b),crate::plane2::dist(a,b));
                    if off > bar && off <= 0.25*distance(xa,xb) && crate::plane2::dist(u,m0) <= 0.25*c && on(u,w) { uv.push(u); partner.push(w); put = true }
                }
            }
            uv.push(b);
            partner.push(out.partner[j+1]);
        }
        (out.uv,out.partner) = (uv,partner);
        if !put { break }
        more = true;
    }
    more.then_some(out)
}

/// How many times over a band's trim's polyline is refined at most (`refined`): each a halving of
/// the segments that need it, their sag a quarter each time (structural).
const REFINED: usize = 6;

/// **A double curve's point**: its two charts' parameters, from `guess`, settled by Newton's method
/// on `X₁(u₁) = X₂(u₂)` with `u₁` held on the line square to `dir` through the guess, while the
/// steps or the gap between the two points shrink. None where it does not settle. (Beside a sheet's
/// fold the first step mostly carries a partner guessed off the curve onto it and the next is the
/// longer before they shrink: stopped at the first step that did not, a band's trim's point beside
/// a swallowtail point was left 4.8e-8 off the curve, where the next three steps settle it.)
fn settle(tube: &Tube,(c1,c2): (Chart,Chart),guess: [P;2],dir: P) -> Option<[P;2]> {
    let [mut u1,mut u2] = guess;
    let value = dir[0]*u1[0]+dir[1]*u1[1];
    let (mut prev,mut gap) = (f64::INFINITY,f64::INFINITY);
    for _ in 0..CORRECTIONS {
        let (x1,x2) = (tube.at(c1,u1)?,tube.at(c2,u2)?);
        let r = sub(x1.x,x2.x);
        let off = crate::util::space::norm(r);
        let f = [r[0],r[1],r[2],dir[0]*u1[0]+dir[1]*u1[1]-value];
        let m = [[x1.x_th[0],x1.x_t[0],-x2.x_th[0],-x2.x_t[0]],[x1.x_th[1],x1.x_t[1],-x2.x_th[1],-x2.x_t[1]],
            [x1.x_th[2],x1.x_t[2],-x2.x_th[2],-x2.x_t[2]],[dir[0],dir[1],0.,0.]];
        let d = solve4(m,f)?;
        let step = d.iter().map(|x| x*x).sum::<f64>().sqrt();
        if !(step < prev) && !(off < gap) { break }
        (prev,gap) = (step.min(prev),off.min(gap));
        u1 = [u1[0]-d[0],u1[1]-d[1]];
        u2 = [u2[0]-d[2],u2[1]-d[3]];
    }
    Some([u1,u2])
}

/// `m x = f` by Gaussian elimination with partial pivoting; none where `m` is singular.
fn solve4(mut m: [[f64;4];4],mut f: [f64;4]) -> Option<[f64;4]> {
    for c in 0..4 {
        let p = (c..4).max_by(|&i,&j| m[i][c].abs().total_cmp(&m[j][c].abs()))?;
        if !(m[p][c].abs() > 0.) { return None }
        m.swap(c,p); f.swap(c,p);
        for r in c+1..4 {
            let g = m[r][c]/m[c][c];
            for k in c..4 { m[r][k] -= g*m[c][k] }
            f[r] -= g*f[c];
        }
    }
    let mut x = [0.;4];
    for c in (0..4).rev() { x[c] = (f[c]-(c+1..4).map(|k| m[c][k]*x[k]).sum::<f64>())/m[c][c] }
    Some(x)
}

/// `x` moved by whole periods to stand within half a period of `near`.
fn unwrap(x: f64,near: f64) -> f64 { x-TAU*((x-near)/TAU).round() }

/// `unwrap` on a chart whose first parameter runs round (a sheet's angle, the collar's, an end
/// pose's); a band's `τ` as it is: it runs ±7 and is no angle.
fn wrap(chart: Chart,x: f64,near: f64) -> f64 { if let Chart::Band(..) = chart { x } else { unwrap(x,near) } }

/// The faces, their edges and vertices as the trims and the windows give them, sewn.
fn build(tube: &Tube,vertices: &mut Vec<Vertex>,trims: &mut Vec<Trim>,tails: &[(Chart,P,usize)],tol: f64) -> Result<Brep,Unbuilt> {
    let [a,b] = tube.roll;
    let mut points: Vec<(Chart,P)> = tails.iter().map(|t| (t.0,t.1)).collect();
    // (the sheets' windows clear of the pinch's hole, which lies whole in one of them)
    for side in [1.,-1.] { points.extend(tube.hole(Chart::Sheet(side))?.into_iter().flatten().map(|p| (Chart::Sheet(side),p))) }
    // (each chart's cuts clear of its trims where they leave an angle free; else clear of its points
    // alone, each cut then crossing trims, split there: `crossed` holds each such cut's crossings —
    // the chart, the angle, and the crossings' places along it with their vertices)
    let mut charts_cut = CHARTS.to_vec();
    if tube.pinch.is_some() { charts_cut.push(Chart::Collar) }
    let mut cuts: Vec<(Chart,[f64;2])> = Vec::new();
    let mut crossed: Vec<(Chart,f64,Vec<(f64,usize)>)> = Vec::new();
    for &c in &charts_cut {
        match windows(trims,&points,c) {
            Ok(w) => cuts.push((c,w)),
            Err(why) => {
                let w = crossing_windows(&points,vertices,c).map_err(|_| why)?;
                for th in w {
                    let span = match c {
                        Chart::Sheet(_) => [a,b],
                        Chart::Collar => { let r = tube.pinch.as_ref().map_or(0.,|p| p.rho); [-r,r] }
                        Chart::End(k) => { let (plus,way,width) = band(tube,k,th).ok_or_else(|| failed("no band on an end pose".into()))?; [plus,plus+way*width] }
                        Chart::Band(..) => unreachable!("a band's chart is not cut"),
                    };
                    let at = cut_across(tube,c,th,span,trims,vertices)?;
                    if debug() { eprintln!("tube: {c:?}'s cut at {th} crosses {} trims",at.len()) }
                    crossed.push((c,th,at));
                }
                cuts.push((c,w));
            }
        }
    }
    let crossings_of = |chart: Chart,th: f64| -> Vec<(f64,usize)> { crossed.iter().find(|(c,t,_)| *c == chart && *t == th).map_or(Vec::new(),|x| x.2.clone()) };
    let collar_cuts = if tube.pinch.is_some() { cuts_of(&cuts,Chart::Collar) } else { [0.;2] };
    // the cuts from each swallowtail point toward the rim it opens away from (the way whose point a
    // disc's span beside it is kept), each to the first trim it meets or else that rim
    let mut across: Vec<Arc2> = Vec::new();
    for &(chart,p,v) in tails {
        // (on a band's chart, along the line from the point to its seam, the way it opens away from)
        if let Chart::Band(kb,side) = chart {
            let band = &tube.bands[kb];
            // (each way's probe a fiftieth of the band's length, or half the way to its seam that way
            // where the point stands nearer it: a probe past the seam reads no band — a swallowtail
            // point 0.016 inside the seam was refused both ways)
            let step = |dir: f64| (0.02*(band.eta[1]-band.eta[0])).min(0.5*(band.seam_eta(p[0],dir)-p[1]).abs());
            let field = Depth {tube,chart,within:tol,holes:&[]};
            let kept = |eta: f64| match field.reading([p[0],eta]) { Some((_,Reading::Alone)) => true,Some((_,Reading::Far(c))) => c.depth >= 0.,_ => false };
            let dir = if kept(p[1]-step(-1.)) { -1. } else if kept(p[1]+step(1.)) { 1. } else {
                return Err(failed(format!("neither way from the swallowtail point {p:?} on {chart:?} is kept")))
            };
            let target = band.seam_eta(p[0],dir);
            let (foot,e) = match met_by_cut(tube,chart,p,target,v,trims,vertices)? {
                Some(f) => f,
                None => {
                    let q = [p[0],target];
                    let x = tube.at(chart,q).ok_or_else(|| failed("no band at a cut's foot".into()))?.x;
                    let place = band.place(q);
                    vertices.push(Vertex {x,on:vec![(chart,q),(Chart::Sheet(side),[place[0].rem_euclid(TAU),place[1]])]});
                    (vertices.len()-1,target)
                }
            };
            let line: Vec<P> = (0..=32).map(|j| [p[0],p[1]+(e-p[1])*j as f64/32.]).collect();
            across.push(Arc2 {kind:Kind::Across {chart,th:p[0],v:[p[1],e]},v:[v,foot],uses:vec![(chart,line.clone()),(chart,line)]});
            continue
        }
        let Chart::Sheet(side) = chart else { return Err(failed(format!("a swallowtail point on {chart:?}"))) };
        let step = 0.02*(b-a);
        let kept = |t: f64| match tube.reading(side,p[0],t,tol) { Some(Reading::Alone) => true,Some(Reading::Far(c)) => c.depth >= 0.,_ => false };
        let k = if kept(p[1]-step) { 0 } else if kept(p[1]+step) { 1 } else {
            return Err(failed(format!("neither way from the swallowtail point {p:?} is kept")))
        };
        let (foot,e) = match met_by_cut(tube,chart,p,tube.roll[k],v,trims,vertices)? {
            Some(f) => f,
            None => {
                let e = tube.roll[k];
                let x = tube.point(side,p[0],e).ok_or_else(|| failed("no sheet at a cut's foot".into()))?.x;
                let end = tube.on_end(k,x).ok_or_else(|| failed("no end pose at a cut's foot".into()))?;
                vertices.push(Vertex {x,on:vec![(chart,[p[0],e]),(Chart::End(k),end)]});
                (vertices.len()-1,e)
            }
        };
        let line: Vec<P> = (0..=32).map(|j| [p[0],p[1]+(e-p[1])*j as f64/32.]).collect();
        across.push(Arc2 {kind:Kind::Across {chart,th:p[0],v:[p[1],e]},v:[v,foot],uses:vec![(chart,line.clone()),(chart,line)]});
    }
    let mut arcs: Vec<Arc2> = Vec::new();
    for (i,t) in trims.iter().enumerate() {
        arcs.push(Arc2 {kind:Kind::Trim(i),v:t.ends,uses:vec![(t.own,t.uv.clone()),(t.other,t.partner.clone())]});
    }
    arcs.extend(across);
    // the rims, split at every vertex on them and at the cuts of the sheet's and the end pose's
    for side in [1.,-1.] {
        for k in 0..2 {
            let e = tube.roll[k];
            let chart = Chart::Sheet(side);
            let mut splits: Vec<(f64,usize)> = vertices.iter().enumerate()
                .flat_map(|(j,v)| v.on.iter().filter(|(c,p)| *c == chart && p[1] == e).map(move |(_,p)| (p[0].rem_euclid(TAU),j))).collect();
            // (an angle both charts cut at, one vertex)
            let mut at: Vec<f64> = cuts_of(&cuts,chart).into_iter().chain(cuts_of(&cuts,Chart::End(k))).collect();
            at.sort_by(f64::total_cmp);
            at.dedup();
            for c in at {
                let x = tube.point(side,c,e).ok_or_else(|| failed("no sheet at a cut".into()))?.x;
                let end = tube.on_end(k,x).ok_or_else(|| failed("no end pose at a cut".into()))?;
                vertices.push(Vertex {x,on:vec![(chart,[c,e]),(Chart::End(k),end)]});
                splits.push((c,vertices.len()-1));
            }
            splits.sort_by(|p,q| p.0.total_cmp(&q.0));
            let n = splits.len();
            for i in 0..n {
                let (t0,v0) = splits[i];
                let (t1,v1) = if i+1 < n { splits[i+1] } else { (splits[0].0+TAU,splits[0].1) };
                let m = (((t1-t0)/(TAU/256.)).ceil() as usize).max(4);
                let ths: Vec<f64> = (0..=m).map(|j| t0+(t1-t0)*j as f64/m as f64).collect();
                let own: Vec<P> = ths.iter().map(|&t| [t,e]).collect();
                let end: Vec<P> = ths.iter().map(|&t| tube.point(side,t,e).and_then(|p| tube.on_end(k,p.x)).map(|q| [t,q[1]]))
                    .collect::<Option<_>>().ok_or_else(|| failed("a rim not read on its end pose".into()))?;
                arcs.push(Arc2 {kind:Kind::Rim {side,k,th:[t0,t1]},v:[v0,v1],uses:vec![(chart,own),(Chart::End(k),end)]});
            }
        }
    }
    // the pinch's rim, each sheet's, split at every vertex on it and at the collar's cuts
    if let Some(p) = &tube.pinch {
        for sign in [1.,-1.] {
            let rho = sign*p.rho;
            let mut splits: Vec<(f64,usize)> = vertices.iter().enumerate()
                .flat_map(|(j,v)| v.on.iter().filter(|(c,q)| *c == Chart::Collar && q[1] == rho).map(move |(_,q)| (q[0].rem_euclid(TAU),j))).collect();
            for &c in &collar_cuts {
                let x = tube.collar_at(c,rho).ok_or_else(|| failed("no collar at a cut".into()))?.x;
                let place = p.rim(sign,c).ok_or_else(|| failed("the pinch's rim not read at a cut".into()))?;
                vertices.push(Vertex {x,on:vec![(Chart::Collar,[c,rho]),(Chart::Sheet(sign),place)]});
                splits.push((c,vertices.len()-1));
            }
            splits.sort_by(|p,q| p.0.total_cmp(&q.0));
            let n = splits.len();
            for i in 0..n {
                let (t0,v0) = splits[i];
                let (t1,v1) = if i+1 < n { splits[i+1] } else { (splits[0].0+TAU,splits[0].1) };
                let m = (((t1-t0)/(TAU/256.)).ceil() as usize).max(4);
                let psis: Vec<f64> = (0..=m).map(|j| t0+(t1-t0)*j as f64/m as f64).collect();
                let own: Vec<P> = psis.iter().map(|&a| [a,rho]).collect();
                let mut sheet: Vec<P> = psis.iter().map(|&a| p.rim(sign,a)).collect::<Option<_>>().ok_or_else(|| failed("the pinch's rim not read".into()))?;
                for j in 1..sheet.len() { sheet[j][0] = unwrap(sheet[j][0],sheet[j-1][0]) }
                arcs.push(Arc2 {kind:Kind::Hole {sign,psi:[t0,t1]},v:[v0,v1],uses:vec![(Chart::Collar,own),(Chart::Sheet(sign),sheet)]});
            }
        }
        for &c in &collar_cuts {
            let at = |rho: f64| vertices.iter().position(|v| v.on.iter().any(|(ch,q)| *ch == Chart::Collar && q[0] == c && q[1] == rho));
            let (Some(v0),Some(v1)) = (at(-p.rho),at(p.rho)) else { return Err(failed("a collar's cut with no vertex on its rim".into())) };
            chained(&mut arcs,Chart::Collar,c,(-p.rho,v0),&crossings_of(Chart::Collar,c),(p.rho,v1));
        }
    }
    // each band's seam, on both sheets, split at every vertex on it (at its half turns where fewer
    // than two are: a closed edge is two)
    for (kb,band) in tube.bands.iter().enumerate() {
        for side in [1.,-1.] {
            let chart = Chart::Band(kb,side);
            let mut splits: Vec<(f64,usize)> = vertices.iter().enumerate()
                .flat_map(|(j,v)| v.on.iter().filter(|(c,q)| *c == chart && band.seam(*q).abs() <= HALF_READ).map(move |(_,q)| (band.fraction(*q),j))).collect();
            if splits.len() < 2 {
                for f in [0.,0.5] {
                    if splits.iter().any(|s| (s.0-f).abs() < 0.01) { continue }
                    let q = band.seam_at(f);
                    let x = tube.at(chart,q).ok_or_else(|| failed("no band on its seam".into()))?.x;
                    let place = band.place(q);
                    vertices.push(Vertex {x,on:vec![(chart,q),(Chart::Sheet(side),[place[0].rem_euclid(TAU),place[1]])]});
                    splits.push((f,vertices.len()-1));
                }
            }
            splits.sort_by(|p,q| p.0.total_cmp(&q.0));
            let n = splits.len();
            for i in 0..n {
                let (f0,v0) = splits[i];
                let (f1,v1) = if i+1 < n { splits[i+1] } else { (splits[0].0+1.,splits[0].1) };
                let m = (((f1-f0)*256.).ceil() as usize).max(4);
                let fs: Vec<f64> = (0..=m).map(|j| f0+(f1-f0)*j as f64/m as f64).collect();
                let own: Vec<P> = fs.iter().map(|&f| band.seam_at(f)).collect();
                let mut sheet: Vec<P> = own.iter().map(|&q| band.place(q)).collect();
                for j in 1..sheet.len() { sheet[j][0] = unwrap(sheet[j][0],sheet[j-1][0]) }
                arcs.push(Arc2 {kind:Kind::Seam {k:kb,side,f:[f0,f1]},v:[v0,v1],uses:vec![(chart,own),(Chart::Sheet(side),sheet)]});
            }
        }
    }
    // the cuts across each chart at the windows' angles
    let at_rim = |vs: &Vec<Vertex>,chart: Chart,q: P| vs.iter().position(|v| v.on.iter().any(|(c,p)| *c == chart && p[0] == q[0] && p[1] == q[1]));
    for side in [1.,-1.] {
        let chart = Chart::Sheet(side);
        for c in cuts_of(&cuts,chart) {
            let (v0,v1) = (at_rim(vertices,chart,[c,a]).expect("a rim's cut"),at_rim(vertices,chart,[c,b]).expect("a rim's cut"));
            chained(&mut arcs,chart,c,(a,v0),&crossings_of(chart,c),(b,v1));
        }
    }
    for k in 0..2 {
        let chart = Chart::End(k);
        for c in cuts_of(&cuts,chart) {
            let (plus,way,width) = band(tube,k,c).ok_or_else(|| failed("no band on an end pose".into()))?;
            let (v0,v1) = (at_rim(vertices,Chart::Sheet(1.),[c,tube.roll[k]]).expect("a rim's cut"),at_rim(vertices,Chart::Sheet(-1.),[c,tube.roll[k]]).expect("a rim's cut"));
            // (the crossings along the band's way, each read within a turn of its start)
            let along: Vec<(f64,usize)> = crossings_of(chart,c).into_iter().map(|(psi,v)| (plus+way*(way*(psi-plus)).rem_euclid(TAU),v)).collect();
            chained(&mut arcs,chart,c,(plus,v0),&along,(plus+way*width,v1));
        }
    }
    faces(tube,vertices,&arcs,trims,&cuts,tol)
}

/// **A window's cut across `chart` at `θ = c`** from `(v₀, its vertex)` to `(v₁, its vertex)`, as arcs:
/// one, or — where it crosses trims (`crossed`, their places along it and vertices) — one between
/// each two of its vertices in order along it.
fn chained(arcs: &mut Vec<Arc2>,chart: Chart,c: f64,start: (f64,usize),crossed: &[(f64,usize)],end: (f64,usize)) {
    let dir = (end.0-start.0).signum();
    let mut stops: Vec<(f64,usize)> = crossed.to_vec();
    stops.sort_by(|p,q| (dir*p.0).total_cmp(&(dir*q.0)));
    let chain: Vec<(f64,usize)> = std::iter::once(start).chain(stops).chain(std::iter::once(end)).collect();
    for w in chain.windows(2) {
        let ((s0,k0),(s1,k1)) = (w[0],w[1]);
        let line: Vec<P> = (0..=32).map(|j| [c,s0+(s1-s0)*j as f64/32.]).collect();
        arcs.push(Arc2 {kind:Kind::Across {chart,th:c,v:[s0,s1]},v:[k0,k1],uses:vec![(chart,line.clone()),(chart,line)]});
    }
}

/// **Where the cut from the swallowtail point `p` (its vertex `tail`) toward the roll's end `e`
/// first meets a trim on its chart**, if it does before the rim: the trims' polylines on the chart
/// (a run's own, or its partner's where that is the chart) crossed by the cut's angle strictly
/// between `p` and `e`, the nearest `p`; the point settled on the double curve with its angle held
/// (`settle`), a vertex of both its charts, and the trim split there into two (a traced one alone:
/// a join is refused by name). The vertex and the instant the cut ends at. A cut crossing a trim
/// with no vertex there is no planar graph: on the full turn's sheet `−` the cut from its second
/// swallowtail point ran down to the start and crossed a double curve with the end pose there, and
/// one face used a trim on both its sides (#880 T1f).
fn met_by_cut(tube: &Tube,chart: Chart,p: P,e: f64,tail: usize,trims: &mut Vec<Trim>,vertices: &mut Vec<Vertex>)
    -> Result<Option<(usize,f64)>,Unbuilt> {
    // (the nearest crossing: its trim, whether on the partner's polyline, its segment, the point)
    let mut best: Option<(usize,bool,usize,f64,f64)> = None;
    for (i,t) in trims.iter().enumerate() {
        if t.ends.contains(&tail) { continue }
        for (mine,line,ch) in [(false,&t.uv,t.own),(true,&t.partner,t.other)] {
            if ch != chart { continue }
            for j in 0..line.len()-1 {
                let (u,w) = (line[j],line[j+1]);
                let c = wrap(chart,p[0],u[0]);
                if u[0] == w[0] || (u[0]-c)*(w[0]-c) > 0. { continue }
                let f = (c-u[0])/(w[0]-u[0]);
                let t = u[1]+(w[1]-u[1])*f;
                if !((t-p[1])*(e-t) > 0.) { continue }
                if best.is_none_or(|q| (t-p[1]).abs() < (q.4-p[1]).abs()) { best = Some((i,mine,j,f,t)) }
            }
        }
    }
    let Some((i,mine,j,f,_)) = best else { return Ok(None) };
    if !matches!(trims[i].shape,Shape::Traced) { return Err(failed(format!("the cut from the swallowtail point {p:?} meets a join"))) }
    // (the angle held on the cut's own chart: its place on the polyline crossed)
    let (v,u,w) = split_trim(tube,trims,vertices,(i,mine,j,f),&format!("the cut from {p:?}"))?;
    let foot = if mine { w } else { u };
    Ok(Some((v,foot[1])))
}

/// A use's polyline placed in a window from `w0` (its angle within a turn above `w0`, its middle's
/// so, or at the angle `target`, a cut's at the window's side; an end pose's `ψ` read as its offset round the tube from the band's start, the way the band
/// runs).
fn placed(tube: &Tube,chart: Chart,line: &[P],w0: f64,target: Option<f64>) -> Option<Vec<P>> {
    let mid = line[line.len()/2][0];
    let shift = match target { Some(th) => th-line[0][0],None => w0+(mid-w0).rem_euclid(TAU)-mid };
    line.iter().map(|q| {
        let th = q[0]+shift;
        match chart {
            Chart::Sheet(_) | Chart::Collar => Some([th,q[1]]),
            // (a band's chart runs round neither way: placed as it is)
            Chart::Band(..) => Some(*q),
            Chart::End(k) => {
                let (plus,way,_) = band(tube,k,q[0])?;
                let o = ((q[1]-plus)*way).rem_euclid(TAU);
                // (on the band's start itself, nought, not a turn)
                Some([th,if o > TAU-1e-9 { 0. } else { o }])
            }
        }
    }).collect()
}

/// **The faces of every chart**, each window's planar graph walked, each part kept where its inner
/// point bounds the sweep, sewn.
fn faces(tube: &Tube,vertices: &[Vertex],arcs: &[Arc2],trims: &[Trim],cuts: &[(Chart,[f64;2])],tol: f64) -> Result<Brep,Unbuilt> {
    let fit = FIT*tol;
    // **the stretches each sheet's nets are read through** (`Tube::stretches`): those whose outline
    // lies whole inside one of the sheet's windows, so its cuts and its rims are read unstretched and
    // its windows are rectangles in the nets' parameters as in the sheet's
    let held = |side: f64,s: &super::band::Band| {
        let [c0,c1] = cuts_of(cuts,Chart::Sheet(side));
        let outline = s.stretch_outline(64);
        let [a,b] = tube.roll;
        outline.iter().all(|q| q[1] > a && q[1] < b) && [(c0,c1),(c1,c0+TAU)].iter().any(|&(w0,w1)| outline.iter().all(|q| {
            let th = w0+(q[0]-w0).rem_euclid(TAU);
            th > w0 && th < w1
        }))
    };
    let plus: Vec<&super::band::Band> = tube.stretches.iter().filter(|s| held(1.,s)).collect();
    let minus: Vec<&super::band::Band> = tube.stretches.iter().filter(|s| held(-1.,s)).collect();
    let stretched_of = |side: f64| -> &[&super::band::Band] { if side > 0. { &plus } else { &minus } };
    if debug() { eprintln!("tube: stretched sheets: + {}, − {}",plus.len(),minus.len()) }
    // (a use on a sheet read in its nets' parameters, and back)
    let to_net = |c: Chart,q: P| match c { Chart::Sheet(s) => stretched_of(s).iter().rev().fold(q,|q,b| b.unstretch(q)),_ => q };
    let from_net = |c: Chart,q: P| match c { Chart::Sheet(s) => stretched_of(s).iter().fold(q,|q,b| b.stretch(q)),_ => q };
    // each arc's curve in space and its uses' pcurves, in the uses' own parameters
    let mut curves: Vec<(Arc<Curve>,Vec<BSpline>)> = Vec::new();
    for arc in arcs {
        let at = |u: usize,p: P| tube.at(arc.uses[u].0,from_net(arc.uses[u].0,p)).map(|q| q.x);
        // (each use's pcurve fitted in its surface's parameters: a stretched sheet's nets')
        let fitted = |f: &dyn Fn(f64) -> (V,Vec<P>),at: &dyn Fn(usize,P) -> Option<V>,bars: &[f64]| fitted(&|s: f64| {
            let (x,ps) = f(s);
            (x,ps.iter().enumerate().map(|(u,&p)| to_net(arc.uses[u].0,p)).collect())
        },at,bars);
        let (c,pcs) = match arc.kind {
            Kind::Trim(i) => {
                let refined = refined(tube,&trims[i],fit);
                let t = refined.as_ref().unwrap_or(&trims[i]);
                let f = |s: f64| match t.at(tube,s) {
                    Some([u,p]) => (tube.at(t.own,u).map_or([f64::NAN;3],|q| q.x),vec![u,p]),
                    None => ([f64::NAN;3],vec![[f64::NAN;2];2]),
                };
                // (a join's partner held to the tolerance alone: the normal form's leaves stand
                // within it of one another, and a corner's join within it of the end pose)
                let bars = [fit,if let Shape::Traced = t.shape { fit } else { tol }];
                if debug() { eprintln!("tube: trim {i} on {:?} against {:?}, {:?}, from {:?} to {:?}",t.own,t.other,t.shape,t.uv[0],t.uv[t.uv.len()-1]) }
                match fitted(&f,&at,&bars) {
                    Ok(r) => r,
                    // (a sheet's traced trim fitted short: its points held on lines square to its
                    // polyline's segments jump along the curve by a chord's sag where it turns — a
                    // double curve turning 0.05 a chord across a sheet's half turn fitted to 2e-6
                    // against 4.4e-7 — read again with the line turning smoothly, as on a band)
                    Err(e) if matches!(t.shape,Shape::Traced) => {
                        let g = |s: f64| match t.at_held(tube,s,true) {
                            Some([u,p]) => (tube.at(t.own,u).map_or([f64::NAN;3],|q| q.x),vec![u,p]),
                            None => ([f64::NAN;3],vec![[f64::NAN;2];2]),
                        };
                        fitted(&g,&at,&bars).map_err(|_| e)?
                    }
                    Err(e) => return Err(e),
                }
            }
            Kind::Rim {side,k,th} => {
                let f = |s: f64| { let t = th[0]+(th[1]-th[0])*s; let x = tube.point(side,t,tube.roll[k]).map_or([f64::NAN;3],|p| p.x);
                    let e = tube.on_end(k,x).map_or(f64::NAN,|q| q[1]); (x,vec![[t,tube.roll[k]],[t,e]]) };
                let g = |s: f64| { let (x,mut u) = f(s); let e0 = f(0.).1[1][1]; u[1][1] = unwrap(u[1][1],e0); (x,u) };
                fitted(&g,&at,&[fit,fit])?
            }
            Kind::Across {chart,th,v} => {
                let f = |s: f64| { let p = [th,v[0]+(v[1]-v[0])*s]; (tube.at(chart,p).map_or([f64::NAN;3],|q| q.x),vec![p,p]) };
                fitted(&f,&at,&[fit,fit])?
            }
            Kind::Hole {sign,psi} => {
                let pinch = tube.pinch.as_ref().ok_or_else(|| failed("a pinch's rim with no pinch".into()))?;
                let (rho,near) = (sign*pinch.rho,arc.uses[1].1[0][0]);
                let f = |s: f64| {
                    let a = psi[0]+(psi[1]-psi[0])*s;
                    let x = tube.collar_at(a,rho).map_or([f64::NAN;3],|q| q.x);
                    let q = pinch.rim(sign,a).unwrap_or([f64::NAN;2]);
                    (x,vec![[a,rho],[unwrap(q[0],near),q[1]]])
                };
                fitted(&f,&at,&[fit,fit])?
            }
            Kind::Seam {k,side,f} => {
                let (b,chart,near) = (&tube.bands[k],Chart::Band(k,side),arc.uses[1].1[0][0]);
                // (read paced, slowly where it crosses the line: there `τ` swings through nought in a
                // sliver of the seam's turn, `asinh` of the way across over the turn's width, and an
                // arc read evenly in its fraction met it in two of a fit's points — its pcurve 4.8e-7
                // off on a band nine turns wide)
                let paced = paced(f);
                let g = |s: f64| {
                    let q = b.seam_at(paced(s));
                    let p = b.place(q);
                    (tube.at(chart,q).map_or([f64::NAN;3],|x| x.x),vec![q,[unwrap(p[0],near),p[1]]])
                };
                fitted(&g,&at,&[fit,fit])?
            }
        };
        if debug() { eprintln!("tube: arc {} {:?} {:?} mid {:?} ends {:?} {:?}",curves.len(),arc.kind,arc.v,c.point(0.5),c.point(0.),c.point(1.)) }
        curves.push((Arc::new(Curve::BSpline(Arc::new(c))),pcs));
    }
    let mut pieces: Vec<Brep> = Vec::new();
    // (each sheet's net over a window fitted once, its faces there sharing it)
    let mut nets: Vec<(f64,(f64,f64),Arc<crate::nurbs::Net>)> = Vec::new();
    let mut charts = CHARTS.to_vec();
    if tube.pinch.is_some() { charts.push(Chart::Collar) }
    for k in 0..tube.bands.len() { charts.extend([Chart::Band(k,1.),Chart::Band(k,-1.)]) }
    for chart in charts {
        // (a band's chart runs round neither way: one window, the whole of it)
        let banded = matches!(chart,Chart::Band(..));
        let cuts = if banded { [f64::NAN;2] } else { cuts_of(cuts,chart) };
        for w in 0..(if banded { 1 } else { 2 }) {
            let (w0,w1) = if banded { (f64::NEG_INFINITY,f64::INFINITY) } else if w == 0 { (cuts[0],cuts[1]) } else { (cuts[1],cuts[0]+TAU) };
            // the window's edges: (arc, use, its polyline placed)
            let mut lines: Vec<(usize,usize,Vec<P>)> = Vec::new();
            for (i,arc) in arcs.iter().enumerate() {
                for (u,(c,line)) in arc.uses.iter().enumerate() {
                    if *c != chart { continue }
                    // (a cut across a chart is one edge of the graph, both its sides used: its first use
                    // in the window it starts, at the window's side, its second in the one it ends;
                    // a cut from a swallowtail point its first use alone)
                    let target = match arc.kind {
                        Kind::Across {th,..} if cuts.contains(&th) => {
                            if u == 0 && th == cuts[w] { Some(w0) } else if u == 1 && th == cuts[1-w] { Some(w1) } else { continue }
                        }
                        Kind::Across {..} if u == 1 && arc.uses[0].0 == chart => continue,
                        _ => None,
                    };
                    let p = placed(tube,chart,line,w0,target).ok_or_else(|| failed("a line not placed".into()))?;
                    let mid = p[p.len()/2][0];
                    if target.is_none() && !(mid > w0 && mid < w1) { continue }
                    lines.push((i,u,p));
                }
            }
            let loops = walk(&lines,arcs,vertices)?;
            // each loop's polygon in the window and its area (counter-clockwise a part on its left)
            let polys: Vec<(Vec<P>,f64)> = loops.iter().map(|(l,_)| {
                let poly: Vec<P> = l.iter().flat_map(|&(e,fwd)| {
                    let mut p = lines[e].2.clone();
                    if !fwd { p.reverse() }
                    p.pop();
                    p
                }).collect();
                let area: f64 = (0..poly.len()).map(|j| { let (p,q) = (poly[j],poly[(j+1)%poly.len()]); p[0]*q[1]-p[1]*q[0] }).sum();
                (poly,area)
            }).collect();
            // (a loop turning clockwise is the outside of its part of the graph: the window's own,
            // or — where it meets no node of a part turning counter-clockwise and stands inside it —
            // an island in that part, a hole of the least such)
            let mut holes: Vec<Vec<usize>> = vec![Vec::new();loops.len()];
            for (j,(pj,aj)) in polys.iter().enumerate() {
                if *aj > 0. { continue }
                let host = (0..loops.len()).filter(|&i| polys[i].1 > 0. && !loops[i].1.iter().any(|n| loops[j].1.contains(n))
                    && inside(pj[0],&polys[i].0)).min_by(|&x,&y| polys[x].1.total_cmp(&polys[y].1));
                if let Some(i) = host { holes[i].push(j) }
            }
            for (i,(l,_)) in loops.iter().enumerate() {
                let (poly,area) = &polys[i];
                if debug() {
                    let named = |e: usize| match arcs[lines[e].0].kind { Kind::Trim(i) => format!("trim {i}"),Kind::Rim {..} => "rim".into(),
                        Kind::Across {th,..} => format!("across {th}"),Kind::Hole {sign,..} => format!("the pinch's rim {sign}"),
                        Kind::Seam {k,side,..} => format!("band {k}'s seam {side}") };
                    eprintln!("tube: {chart:?} window {w}: a loop {:?}, area {area:e}, holes {:?}",l.iter().map(|&(e,f)| (arcs[lines[e].0].v,f,named(e))).collect::<Vec<_>>(),holes[i])
                }
                if *area <= 0. { continue }
                let rings: Vec<&[P]> = std::iter::once(&poly[..]).chain(holes[i].iter().map(|&j| &polys[j].0[..])).collect();
                let inner = interior(&rings).ok_or_else(|| failed(format!("a part of {chart:?} with no inner point")))?;
                let q = match chart { Chart::Sheet(_) | Chart::Collar | Chart::Band(..) => inner,Chart::End(k) => {
                    let (plus,way,_) = band(tube,k,inner[0]).ok_or_else(|| failed("no band".into()))?; [inner[0],plus+way*inner[1]] } };
                let field = Depth {tube,chart,within:tol,holes:&[]};
                let keep = match field.reading(q) { Some((_,Reading::Alone)) => true,Some((_,Reading::Far(c))) => c.depth >= 0.,_ => false };
                if !keep { continue }
                if debug() { eprintln!("tube: kept {chart:?} window {w}: {:?}",l.iter().map(|&(e,_)| lines[e].0).collect::<Vec<_>>()) }
                let ls: Vec<&[(usize,bool)]> = std::iter::once(&l[..]).chain(holes[i].iter().map(|&j| &loops[j].0[..])).collect();
                let st: &[&super::band::Band] = match chart { Chart::Sheet(s) => stretched_of(s),_ => &[] };
                pieces.push(face(tube,chart,(w0,w1),&lines,&ls,arcs,&curves,vertices,&mut nets,st,tol)?);
            }
        }
    }
    crate::sew::sewn(&pieces,tol).map_err(|e| failed(format!("the faces sewn: {e}")))
}

/// **A seam's arc paced** over its fractions `[a, b]`: `[0, 1]` mapped to the fraction whose polar
/// angle is `φ(σ) = σ + ½ sin 2σ`, `σ` even along the arc — `φ` smooth and rising, its rate `1 +
/// cos 2σ` nought at the seam's two ends on the line (`φ` a quarter and three quarters of a turn),
/// where `τ` swings through nought, so the swing is spread over a stretch of `σ` and every reading
/// of the arc stays smooth in it.
fn paced([a,b]: [f64;2]) -> impl Fn(f64) -> f64 {
    // (`σ` of the fraction `f`, within its turn: `φ` rises with `σ`, so by bisection to the rounding)
    let sigma = |f: f64| {
        let k = f.floor();
        let phi = TAU*(f-k);
        let (mut lo,mut hi) = (0.,TAU);
        for _ in 0..80 { let m = 0.5*(lo+hi); if m+0.5*(2.*m).sin() < phi { lo = m } else { hi = m } }
        TAU*k+0.5*(lo+hi)
    };
    let (s0,s1) = (sigma(a),sigma(b));
    move |s: f64| { let x = s0+(s1-s0)*s; (x+0.5*(2.*x).sin())/TAU }
}

/// A point inside a polygon with holes (`rings`, the outer first; even and odd crossings): the
/// middle of the widest run across it at the middle of its height.
fn interior(rings: &[&[P]]) -> Option<P> {
    let (lo,hi) = rings[0].iter().fold((f64::INFINITY,f64::NEG_INFINITY),|(l,h),p| (l.min(p[1]),h.max(p[1])));
    for f in [0.5,0.3,0.7,0.2,0.8] {
        let y = lo+(hi-lo)*f;
        let mut xs: Vec<f64> = rings.iter().flat_map(|poly| (0..poly.len()).filter_map(move |j| {
            let (a,b) = (poly[j],poly[(j+1)%poly.len()]);
            ((a[1] <= y) != (b[1] <= y)).then(|| a[0]+(y-a[1])/(b[1]-a[1])*(b[0]-a[0]))
        })).collect();
        xs.sort_by(f64::total_cmp);
        if let Some(w) = xs.chunks(2).filter(|w| w.len() == 2).max_by(|a,b| (a[1]-a[0]).total_cmp(&(b[1]-b[0]))) {
            if w[1] > w[0] { return Some([0.5*(w[0]+w[1]),y]) }
        }
    }
    None
}

/// Whether `p` lies inside the polygon `poly` (even and odd crossings of the way out along `u`).
fn inside(p: P,poly: &[P]) -> bool {
    (0..poly.len()).filter(|&j| {
        let (a,b) = (poly[j],poly[(j+1)%poly.len()]);
        (a[1] <= p[1]) != (b[1] <= p[1]) && a[0]+(p[1]-a[1])/(b[1]-a[1])*(b[0]-a[0]) > p[0]
    }).count()%2 == 1
}

/// **A window's planar graph walked into loops**, each part on the left of its edges: at each
/// vertex the edges leaving it in order of their way out, and from an edge arriving the next one
/// clockwise from its own way back. Each loop its edges (a line of `lines`, and whether forward)
/// and the nodes it passes.
#[allow(clippy::type_complexity)]
fn walk(lines: &[(usize,usize,Vec<P>)],arcs: &[Arc2],vertices: &[Vertex]) -> Result<Vec<(Vec<(usize,bool)>,Vec<(usize,usize)>)>,Unbuilt> {
    use std::collections::BTreeMap;
    // (a node: a vertex at one of its places on the chart — a double curve's point has two on one
    // sheet — the place nearest the line's end, its angle and ψ read within a turn)
    let node = |e: usize,end: usize| -> (usize,usize) {
        let (i,u,_) = lines[e];
        let (chart,raw) = &arcs[i].uses[u];
        let v = arcs[i].v[end];
        let q = raw[if end == 0 { 0 } else { raw.len()-1 }];
        let apart = |p: P| {
            let d0 = (p[0]-q[0]+PI).rem_euclid(TAU)-PI;
            let d1 = if let Chart::End(_) = chart { (p[1]-q[1]+PI).rem_euclid(TAU)-PI } else { p[1]-q[1] };
            d0.dhypot(d1)
        };
        let k = vertices[v].on.iter().enumerate().filter(|(_,(c,_))| c == chart)
            .min_by(|x,y| apart(x.1.1).total_cmp(&apart(y.1.1))).map_or(0,|(k,_)| k);
        (v,k)
    };
    // the half-edges leaving each node, with their angle
    let mut out: BTreeMap<(usize,usize),Vec<(f64,usize,bool)>> = BTreeMap::new();
    for (e,(_,_,p)) in lines.iter().enumerate() {
        let (v0,v1) = (node(e,0),node(e,1));
        let ang = |q: P,r: P| (r[1]-q[1]).datan2(r[0]-q[0]);
        out.entry(v0).or_default().push((ang(p[0],p[1]),e,true));
        let n = p.len();
        out.entry(v1).or_default().push((ang(p[n-1],p[n-2]),e,false));
    }
    for l in out.values_mut() { l.sort_by(|x,y| x.0.total_cmp(&y.0)) }
    let mut used = vec![[false;2];lines.len()];
    let mut loops = Vec::new();
    for e0 in 0..lines.len() {
        for fwd0 in [true,false] {
            if used[e0][fwd0 as usize] { continue }
            let mut l = Vec::new();
            let mut nodes = Vec::new();
            let (mut e,mut fwd) = (e0,fwd0);
            loop {
                if used[e][fwd as usize] { break }
                used[e][fwd as usize] = true;
                l.push((e,fwd));
                let v = if fwd { node(e,1) } else { node(e,0) };
                nodes.push(v);
                let ring = out.get(&v).ok_or_else(|| failed("a vertex left by nothing".into()))?;
                // (the half-edge back along this one, and the next clockwise from it)
                let back = ring.iter().position(|&(_,f,w)| f == e && w == !fwd).ok_or_else(|| failed("an edge not left".into()))?;
                let next = ring[(back+ring.len()-1)%ring.len()];
                (e,fwd) = (next.1,next.2);
            }
            if l.len() < 2 { return Err(failed("a window's edge closing on itself".into())) }
            loops.push((l,nodes));
        }
    }
    Ok(loops)
}

/// **One kept part as a face**: on a sheet a net fitted over the window as a map, on an end pose its
/// torus; its loop's edges each the arc's curve and its use's pcurve, in the surface's parameters.
#[allow(clippy::too_many_arguments)]
fn face(tube: &Tube,chart: Chart,(w0,w1): (f64,f64),lines: &[(usize,usize,Vec<P>)],loops: &[&[(usize,bool)]],arcs: &[Arc2],
    curves: &[(Arc<Curve>,Vec<BSpline>)],vertices: &[Vertex],nets: &mut Vec<(f64,(f64,f64),Arc<crate::nurbs::Net>)>,
    stretches: &[&super::band::Band],tol: f64) -> Result<Brep,Unbuilt> {
    // the surface, and the map from the chart's parameters (placed) to the surface's
    let (surface,map): (Surface,Box<dyn Fn(P) -> P>) = match chart {
        // (a band's chart: the sheet over all its rectangle, one net each band and side, its faces
        // there sharing it — kept with the sheets' windows' nets, at no angle a window has)
        Chart::Band(k,side) => {
            let [[t0,t1],[e0,e1]] = tube.rect(chart);
            let key = (f64::NEG_INFINITY,k as f64);
            let net = match nets.iter().find(|(s,w,_)| *s == side && *w == key) {
                Some((_,_,net)) => net.clone(),
                None => {
                    let f = |u: f64,v: f64| tube.at(chart,[t0+(t1-t0)*u,e0+(e1-e0)*v]).map_or([f64::NAN;3],|p| p.x);
                    let (net,err) = crate::nurbs::fit_net(&f,DEGREE,DEGREE,16,16,FIT*tol).ok_or_else(|| failed(format!("the band {k}'s sheet {side} not fitted")))?;
                    if debug() { eprintln!("tube: the band {k}'s sheet {side}: {}×{} poles, fitted to {err:e}",net.poles.len(),net.poles[0].len()) }
                    let net = Arc::new(net);
                    nets.push((side,key,net.clone()));
                    net
                }
            };
            (Surface::BSpline(Frame::WORLD,net),Box::new(move |p: P| [(p[0]-t0)/(t1-t0),(p[1]-e0)/(e1-e0)]))
        }
        Chart::Sheet(side) => {
            let lay = Lay::of(tube,(w0,w1));
            let net = match nets.iter().find(|(s,w,_)| *s == side && *w == (w0,w1)) {
                Some((_,_,net)) => net.clone(),
                None => {
                    let bar = if lay.lean == 0. { FIT } else { SHEET_FIT }*tol;
                    let net = Arc::new(sheet_net(tube,side,&lay,bar,stretches)?);
                    nets.push((side,(w0,w1),net.clone()));
                    net
                }
            };
            (Surface::BSpline(Frame::WORLD,net),Box::new(move |p: P| lay.net(p)))
        }
        Chart::Collar => {
            let [_,[lo,hi]] = tube.rect(chart);
            let f = |u: f64,v: f64| tube.at(chart,[w0+(w1-w0)*u,lo+(hi-lo)*v]).map_or([f64::NAN;3],|p| p.x);
            let (net,_) = crate::nurbs::fit_net(&f,DEGREE,DEGREE,32,16,FIT*tol).ok_or_else(|| failed("the collar not fitted over a window".into()))?;
            (Surface::BSpline(Frame::WORLD,Arc::new(net)),Box::new(move |p: P| [(p[0]-w0)/(w1-w0),(p[1]-lo)/(hi-lo)]))
        }
        Chart::End(k) => {
            let m = tube.path.at(tube.roll[k]).map_err(|_| failed("no pose at an end".into()))?;
            let f = &tube.ring.frame;
            let frame = Frame {o:m.point(f.at([0.,0.,tube.height])),x:m.vector(f.x),y:m.vector(f.y),z:m.vector(f.z)};
            (Surface::Torus(frame,tube.radius,tube.r),Box::new(|p: P| p))
        }
    };
    let mut out = Brep::default();
    let mut rings: Vec<Vec<Coedge>> = Vec::new();
    // (the outer loop's first point: on an end pose a hole begins within half a turn of `ψ` of it)
    let mut start: Option<P> = None;
    for (n,l) in loops.iter().enumerate() {
    let mut uses = Vec::new();
    // (an end pose's pcurves each moved by whole turns of `ψ` to start where the last ended)
    let mut last: Option<P> = start;
    let mut first: Option<u32> = None;
    for (j,&(e,fwd)) in l.iter().enumerate() {
        let (i,u,ref placed) = lines[e];
        let arc = &arcs[i];
        let (curve,pcs) = &curves[i];
        let pc = &pcs[u];
        // (the pcurve's own angle moved by the turns the line's placing moved it, and on an end
        // pose its ψ by the turns that meet the last)
        let shift_th = placed[0][0]-arc.uses[u].1[0][0];
        let begin = if fwd { pc.point(0.) } else { pc.point(1.) };
        let shift_v = match (chart,last) {
            (Chart::End(_),Some(q)) => { let s = [begin[0]+shift_th,begin[1]]; unwrap(s[1],q[1])-s[1] }
            _ => 0.,
        };
        let moved = |p: V| map([p[0]+shift_th,p[1]+shift_v]);
        let poles: Vec<V> = pc.poles.iter().map(|&p| { let q = moved(p); [q[0],q[1],0.] }).collect();
        let pcurve = BSpline {poles,..pc.clone()};
        if start.is_none() { let b = if fwd { pcurve.point(0.) } else { pcurve.point(1.) }; start = Some([b[0],b[1]]) }
        let end = if fwd { pcurve.point(1.) } else { pcurve.point(0.) };
        last = Some([end[0],end[1]]);
        let [v0,v1] = arc.v;
        let (vs,ve) = if fwd { (v0,v1) } else { (v1,v0) };
        let start_v = if j == 0 { let id = out.vertex(vertices[vs].x); first = Some(id); id } else { (out.vertices.len()-1) as u32 };
        let end_v = if j+1 == l.len() { first.expect("a first vertex") } else { out.vertex(vertices[ve].x) };
        let ends = if fwd { [start_v,end_v] } else { [end_v,start_v] };
        out.edges.push(Edge {curve:EdgeCurve::Curve((**curve).clone()),t:[0.,1.],v:ends,tol:0.});
        uses.push(Coedge {edge:(out.edges.len()-1) as u32,reversed:!fwd,pcurve:Pcurve::Curve(Arc::new(Curve::BSpline(Arc::new(pcurve))))});
    }
    // (the outer loop counter-clockwise in the surface's own parameters, a hole clockwise: its
    // pcurves' polygon's area says which way round it turns — an end pose's band may run down `ψ`)
    let mut ring: Vec<P> = Vec::new();
    for c in &uses {
        let Pcurve::Curve(pc) = &c.pcurve else { unreachable!("a pcurve of its own") };
        for j in 0..8 { let s = j as f64/8.; let q = pc.point(if c.reversed { 1.-s } else { s }); ring.push([q[0],q[1]]) }
    }
    let area: f64 = (0..ring.len()).map(|j| { let (p,q) = (ring[j],ring[(j+1)%ring.len()]); p[0]*q[1]-p[1]*q[0] }).sum();
    if (area < 0.) == (n == 0) { uses = crate::topo::reverse_loop(uses) }
    rings.push(uses);
    }
    out.faces.push(Face {surface,reversed:false,loops:rings,name:format!("{chart:?}")});
    Ok(out)
}

/// The most points a graded net is read at along either way (structural, as `fit_net`'s caps).
const MOST_GRADED: usize = 4096;
/// The share of the worst miss a graded net's round halves past (structural): a round halves only
/// the intervals missing by more than this share of the worst, and by more than the bar. Halving
/// every interval past the bar halves those the interpolant's ringing reaches from where the grid
/// does not yet follow the sheet, a few places each way of it, which the sheet itself never asked:
/// the crossed tori's forward member's sheets about its pinch graded to 279 × 156 and 299 × 155
/// poles so, 156 × 86 and 173 × 81 at a quarter (each to `FIT`).
const GRADED_SHARE: f64 = 0.25;

/// **How a sheet's net lies over its window `(w0, w1)`**: its `v` the roll's, `(t − a)/(b − a)`,
/// and its `s` along `σ = θ − κ (t − t*)` over `span`, the least `σ` of the window to the greatest.
/// The lean `κ` is nought but over a window holding the pinch `(θ*, t*)`, where it is the pinch's
/// own (`Lay::of`): the sheet's turns gather on the rows about the pinch, each row's nearest the
/// pinch at its own `θ` — `|(A, B)|` least along the row where `θ − θ* = κ (t − t*)`, `κ = −(A_θ ·
/// A_t)/|A_θ|²` of the Jacobian of `(A, B)` there — so a net over `(θ, t)` grades every column the
/// rows' nearest places spread over, and one over `(σ, t)` only those about `σ = θ*`. The crossed
/// tori's forward member's sheets about its pinch (`κ` −0.71), graded to the same bar (`FIT`):
/// 218 × 193 and 225 × 180 poles over `(θ, t)`, 156 × 86 and 173 × 81 over `(σ, t)`, its span half
/// again as wide; its STEP file 14.1 MB and 5.2 MB. Affine, so a pcurve in the chart's `(θ, t)` is one in
/// the net's `(s, v)`, its poles mapped. The net runs on past the window, and is the sheet there
/// too (`sheet_net`).
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Lay { pub window: (f64,f64),pub roll: [f64;2],pub lean: f64,pub at: f64,pub span: [f64;2] }

impl Lay {
    /// The window laid square: `s` the window's `θ`, `v` the roll's.
    pub fn plain(window: (f64,f64),roll: [f64;2]) -> Lay {
        Lay {window,roll,lean:0.,at:0.,span:[window.0,window.1]}
    }
    /// The window `(w0, w1)` of `tube`'s sheets laid along its pinch's rows where it holds the pinch,
    /// else square.
    pub fn of(tube: &Tube,(w0,w1): (f64,f64)) -> Lay {
        let [a,b] = tube.roll;
        let within = |th: f64| { let th = w0+(th-w0).rem_euclid(TAU); th < w1 };
        let lean = tube.pinch.as_ref().filter(|p| within(p.collar.at.1)).and_then(|p| {
            let (t0,th0) = p.collar.at;
            let j = p.rates(th0,t0)?;
            let (at,aa) = ([j[0][0],j[1][0]],[j[0][1],j[1][1]]);
            let k = -(at[0]*aa[0]+at[1]*aa[1])/(at[0]*at[0]+at[1]*at[1]);
            k.is_finite().then_some((k,t0))
        });
        // (a lean that moves the net over the roll by less than one of its first columns is none:
        // the crossed reverse's, −0.016, a third of one)
        let Some((lean,at)) = lean.filter(|(k,_)| k.abs()*(b-a) >= (w1-w0)/32.) else { return Lay::plain((w0,w1),tube.roll) };
        let (p,q) = (lean*(a-at),lean*(b-at));
        Lay {window:(w0,w1),roll:tube.roll,lean,at,span:[w0-p.max(q),w1-p.min(q)]}
    }
    /// The net's `(s, v)` of the chart's `(θ, t)`.
    pub fn net(&self,p: P) -> P {
        let ([c0,c1],[a,b]) = (self.span,self.roll);
        [(p[0]-self.lean*(p[1]-self.at)-c0)/(c1-c0),(p[1]-a)/(b-a)]
    }
    /// The chart's `(θ, t)` of the net's `(s, v)`.
    pub fn chart(&self,q: P) -> P {
        let ([c0,c1],[a,b]) = (self.span,self.roll);
        let t = a+(b-a)*q[1];
        [c0+(c1-c0)*q[0]+self.lean*(t-self.at),t]
    }
    /// Whether the chart's `(θ, t)` lies in the window (a net leaning past it reads the sheet
    /// beyond, which no face of the window lies on).
    pub fn holds(&self,p: P) -> bool { self.lean == 0. || p[0] >= self.window.0 && p[0] <= self.window.1 }
}

/// **A sheet's net over its window, laid as `lay` says**, fitted as a map (`nurbs::fit_net`) to
/// `bar`; where the window holds the pinch's hole, through the sheet with the hole filled smoothly
/// (`Tube::net_point`) and its parameters graded toward it (`fit_graded`): about the pinch the sheet
/// bends at the scale of `ρ₀` (its normal turns half round across the hole), which a uniform grid
/// over the whole roll meets only with `ρ₀` past the swallowtail points (measured: the bar held
/// outside the hole from `ρ₀` 0.14 forward and 0.12 reverse at 1024 × 512).
pub fn sheet_net(tube: &Tube,side: f64,lay: &Lay,bar: f64,stretches: &[&super::band::Band]) -> Result<crate::nurbs::Net,Unbuilt> {
    let (w0,w1) = lay.window;
    let within = |th: f64| { let th = w0+(th-w0).rem_euclid(TAU); th < w1 };
    // (the place of the net's parameters: through each stretch, the identity outside them all)
    let place = |q: P| stretches.iter().fold(q,|q,s| s.stretch(q));
    // (a window holding a band's seam graded as one holding the pinch's hole, its stand-in read and
    // held to the bar inside the seam as outside: the stand-in is smooth, and a net held to it
    // strays nowhere a Boolean reads it)
    let holds = tube.pinch.as_ref().is_some_and(|p| within(p.collar.at.1))
        || tube.bands.iter().any(|b| b.rim(64).iter().any(|q| within(q[0])))
        || stretches.iter().any(|s| s.stretch_outline(16).iter().any(|q| within(q[0])));
    // (every window graded, not only one holding a hole: a window's grid doubled whole wherever
    // any of it misses fitted the crossed tori's first window to 1.6e-8 at a bar of 4.4e-7, 65 × 129
    // poles where 33 × 33 grade to it)
    let fit = if holds {
        let f = |u: f64,v: f64| { let q = place(lay.chart([u,v])); tube.net_point(side,q[0],q[1]).unwrap_or([f64::NAN;3]) };
        // (read against the sheet where the net is kept: outside the hole, the faces' rim — and past
        // the window, where a leaning net runs on: a face's stretch of it reaches there, which a
        // Boolean reads for crossings, so it is the sheet there too)
        let kept = |u: f64,v: f64| tube.pinch.as_ref().is_none_or(|p| p.side(place(lay.chart([u,v]))) >= 0.);
        // (a leaning net's span wider than the window, its first columns as many to the radian)
        let nu = if lay.lean == 0. { 32 } else { (32.*(lay.span[1]-lay.span[0])/(w1-w0)).ceil() as usize };
        fit_graded(&f,&kept,DEGREE,nu,16,bar)
    } else {
        let f = |u: f64,v: f64| { let q = lay.chart([u,v]); tube.point(side,q[0],q[1]).map_or([f64::NAN;3],|p| p.x) };
        fit_graded(&f,&|_,_| true,DEGREE,32,16,bar)
    };
    let (net,err) = fit.ok_or_else(|| failed(format!("the sheet {side} not fitted over a window")))?;
    if debug() { eprintln!("tube: the sheet {side} over ({w0}, {w1}), leaning {}: {}×{} poles, fitted to {err:e}",lay.lean,net.poles.len(),net.poles[0].len()) }
    Ok(net)
}

/// **A net through `f` over the unit square, its parameters graded where it bends**: interpolated
/// row by row and then pole by pole (as `nurbs::fit_net`) at parameters that start even and are
/// halved, interval by interval, until the net holds `f` within `bar` wherever `kept` at the middle
/// of every interval of every row and column and at the middle of every cell — each neighbour of
/// an interval halved halved too where it would stand more than twice as wide (the grading kept
/// within a factor two, structural, so the interpolant does not ring). None where `f` is not read, a
/// way asks more than `MOST_GRADED` points, or a round finds nothing to halve.
///
/// **Which way a miss is the grid's**: on a row the net is that row's interpolant along `u`, on a
/// column the columns' along `v`, so a miss at an interval's middle there is that way's. A miss at a
/// cell's middle is both ways' at once, and is read apart: the interpolant along `u` through the
/// cell's middle row, read at its middle, is the miss `u`'s grid makes there, and along `v` through
/// its middle column `v`'s. Each way's intervals are halved where they miss by more than the bar
/// and by more than `GRADED_SHARE` of the worst miss (where the grid does not yet follow `f`, the
/// interpolant rings out from it, and a miss some places off is its echo); a cell missing neither
/// way alone, its two intervals. Held at its lines alone, a net about the crossed tori's pinch
/// missed the sheet by 8e-6 inside its cells at a bar of 4.4e-7 and a sewn edge stood 4e-5 off its
/// face; halving both of a cell's intervals for its miss grew it to 581 × 457.
fn fit_graded(f: &(dyn Fn(f64,f64) -> V + Sync),kept: &(dyn Fn(f64,f64) -> bool + Sync),degree: usize,nu: usize,nv: usize,bar: f64) -> Option<(crate::nurbs::Net,f64)> {
    use crate::util::par::indices;
    let even = |n: usize| (0..=n).map(|i| i as f64/n as f64).collect::<Vec<f64>>();
    let mids = |ts: &[f64]| ts.windows(2).map(|w| 0.5*(w[0]+w[1])).collect::<Vec<f64>>();
    let (mut us,mut vs) = (even(nu),even(nv));
    loop {
        // (each row read, interpolated and its error read on every core, `util::par`, in order: a
        // window about a pinch is graded to some 900 × 850, read a few times over, #1013)
        let grid: Vec<Vec<V>> = indices(vs.len(),|j| us.iter().map(|&u| f(u,vs[j])).collect());
        if grid.iter().flatten().flatten().any(|x| !x.is_finite()) { return None }
        let rows: Vec<BSpline> = indices(grid.len(),|j| crate::nurbs::interpolate(&grid[j],&us,degree)).into_iter().collect::<Option<_>>()?;
        let cols: Vec<BSpline> = indices(rows[0].poles.len(),|i| crate::nurbs::interpolate(&rows.iter().map(|r| r.poles[i]).collect::<Vec<_>>(),&vs,degree))
            .into_iter().collect::<Option<_>>()?;
        let net = crate::nurbs::Net {du:rows[0].degree,dv:cols[0].degree,uknots:rows[0].knots.clone(),vknots:cols[0].knots.clone(),
            poles:cols.into_iter().map(|c| c.poles).collect(),weights:None};
        let (mu,mv) = (mids(&us),mids(&vs));
        let off = |u: f64,v: f64| if kept(u,v) { distance(net.point(u,v),f(u,v)) } else { 0. };
        let mut eu: Vec<f64> = indices(mu.len(),|i| vs.iter().map(|&v| off(mu[i],v)).fold(0.,f64::max));
        let mut ev: Vec<f64> = indices(mv.len(),|j| us.iter().map(|&u| off(u,mv[j])).fold(0.,f64::max));
        // (each cell's middle: `f` there where kept, and the net's miss)
        let middle: Vec<Vec<Option<V>>> = indices(mu.len(),|i| mv.iter().map(|&v| kept(mu[i],v).then(|| f(mu[i],v))).collect());
        let cells: Vec<Vec<f64>> = indices(mu.len(),|i| mv.iter().zip(&middle[i]).map(|(&v,x)| x.map_or(0.,|x| distance(net.point(mu[i],v),x))).collect());
        let worst = eu.iter().chain(&ev).chain(cells.iter().flatten()).fold(0f64,|m,&e| m.max(e));
        if debug() {
            let hist = |ts: &[f64]| (0..20).map(|k| ts.iter().filter(|&&t| t >= k as f64/20. && t < (k+1) as f64/20.).count()).collect::<Vec<_>>();
            eprintln!("graded: {}×{}, worst {worst:e}; u {:?} v {:?}",us.len(),vs.len(),hist(&us),hist(&vs))
        }
        if !(worst > bar) { return Some((net,worst)) }
        if worst.is_finite() {
            // (a cell's miss read apart: along `u` through its middle row, along `v` through its
            // middle column)
            let across: Vec<Vec<f64>> = indices(mv.len(),|j| {
                let Some(c) = crate::nurbs::interpolate(&us.iter().map(|&u| f(u,mv[j])).collect::<Vec<_>>(),&us,degree) else { return vec![f64::INFINITY;mu.len()] };
                mu.iter().enumerate().map(|(i,&u)| middle[i][j].map_or(0.,|x| distance(c.point(u),x))).collect()
            });
            let along: Vec<Vec<f64>> = indices(mu.len(),|i| {
                let Some(c) = crate::nurbs::interpolate(&vs.iter().map(|&v| f(mu[i],v)).collect::<Vec<_>>(),&vs,degree) else { return vec![f64::INFINITY;mv.len()] };
                mv.iter().enumerate().map(|(j,&v)| middle[i][j].map_or(0.,|x| distance(c.point(v),x))).collect()
            });
            for row in &across { for (e,&m) in eu.iter_mut().zip(row) { *e = e.max(m); } }
            for col in &along { for (e,&m) in ev.iter_mut().zip(col) { *e = e.max(m); } }
        }
        // (only the intervals missing by a share of the worst: where it is not yet met, the
        // interpolant rings out from it, and a miss some places off is its echo, not the sheet's)
        let at = bar.max(GRADED_SHARE*worst);
        if !eu.iter().chain(&ev).any(|&e| e > at) {
            // (a cell missing neither way alone: its two intervals)
            for (i,row) in cells.iter().enumerate() { for (j,&m) in row.iter().enumerate() { if m > at { eu[i] = eu[i].max(m); ev[j] = ev[j].max(m); } } }
        }
        let (gu,gv) = (graded(&us,&eu,at),graded(&vs,&ev,at));
        if gu.len() > MOST_GRADED || gv.len() > MOST_GRADED || gu.len() == us.len() && gv.len() == vs.len() { return None }
        (us,vs) = (gu,gv);
    }
}

/// The parameters `ts` with each interval whose error is over `bar` halved, and each neighbour of a
/// halved one halved too where it would stand more than twice as wide.
fn graded(ts: &[f64],errors: &[f64],bar: f64) -> Vec<f64> {
    let n = errors.len();
    let mut split: Vec<bool> = errors.iter().map(|&e| e > bar).collect();
    loop {
        let width = |i: usize,s: &[bool]| (ts[i+1]-ts[i])*if s[i] { 0.5 } else { 1. };
        let more: Vec<usize> = (0..n).filter(|&i| !split[i] && ((i > 0 && width(i,&split) > 2.*width(i-1,&split))
            || (i+1 < n && width(i,&split) > 2.*width(i+1,&split)))).collect();
        if more.is_empty() { break }
        for i in more { split[i] = true }
    }
    let mut out = vec![ts[0]];
    for i in 0..n {
        if split[i] { out.push(0.5*(ts[i]+ts[i+1])) }
        out.push(ts[i+1]);
    }
    out
}

/// **The built solid's faces held to the sweep's boundary** (the design bible's fourth principle:
/// built right or refused with a witness): each face read at the points of a 4 × 4 grid over its
/// parameters that lie in it (`query::Located::face_place`), and at each the tool's least depth over
/// the roll — sampled at `HELD_SAMPLES`, each least refined by golden sections — within a hair of
/// nought (`1e-4` of the solid's size, or 64 tolerances). A face read deeper lies inside the sweep
/// or off it, and the solid is refused there (`Unbuilt::Wrong`): a chart whose depth's search missed
/// a pocket beside a fold kept its folded sheet, 0.15 inside the solid, every check of its own passed.
pub fn held(tube: &Tube,built: &Brep,tol: f64) -> Result<(),Unbuilt> {
    use crate::query::{Located,Place};
    let solid = Located::new(built,tol);
    let size = { let (lo,hi) = built.bounds(); distance(lo,hi) };
    let hair = (1e-4*size).max(64.*tol);
    let [a,b] = tube.roll;
    let depth = |x: V| -> f64 {
        let d = |t: f64| tube.depth_at(x,t).map_or(f64::INFINITY,|r| r.0);
        let n = HELD_SAMPLES;
        let h = (b-a)/n as f64;
        let vals: Vec<f64> = (0..=n).map(|k| d(a+h*k as f64)).collect();
        let mut best = vals.iter().copied().fold(f64::INFINITY,f64::min);
        for k in 0..=n {
            let (l,r) = (if k > 0 { vals[k-1] } else { f64::INFINITY },if k < n { vals[k+1] } else { f64::INFINITY });
            if vals[k] <= l && vals[k] <= r && vals[k] <= best+hair {
                let t = super::golden(&d,(a+h*(k as f64-1.)).max(a),(a+h*(k as f64+1.)).min(b));
                best = best.min(d(t));
            }
        }
        best
    };
    let points: Vec<(usize,V)> = (0..built.faces.len()).flat_map(|fi| {
        let [[u0,u1],[v0,v1]] = crate::boolean::face_parameters(built,fi);
        let f = &built.faces[fi];
        (1..5).flat_map(move |i| (1..5).map(move |j| [u0+(u1-u0)*(i as f64-0.5)/4.,v0+(v1-v0)*(j as f64-0.5)/4.]))
            .map(move |uv| (fi,f.surface.point(uv)))
    }).collect();
    // (each point read on the cores: placed in its face, then its depth)
    let off = crate::util::par::map(&points,|&(fi,p)| {
        if solid.face_place(fi,p) != Place::In { return None }
        let v = depth(p);
        (v.abs() > hair).then_some((fi,p,v))
    });
    if let Some((fi,p,v)) = off.into_iter().flatten().next() {
        if debug() { eprintln!("tube: face {fi} at {p:?} stands {v:e} from the sweep's boundary, past {hair:e}") }
        return Err(Unbuilt::Wrong {at:p,held:v < 0.})
    }
    Ok(())
}

/// The instants of the roll `held` reads a point's depth at before refining each least.
const HELD_SAMPLES: usize = 512;
