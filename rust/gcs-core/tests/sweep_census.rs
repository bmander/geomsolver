//! A census of continuous sweeps against the structure Adsul, Machchhar and Sohoni give a sweep's
//! envelope (docs/sweep-literature.md, #239's first step). Each sweep's contact set is read
//! instant by instant, as their seed surface is: at each of a grid of times, every ring of every
//! tool face is searched round for the roots of `f = n·v` (the outward normal against the
//! velocity), and every convex corner's fan for where its two faces' `f` differ in sign. At each
//! contact `p` of a face their invariant
//!
//! `θ(p) = l f_u + m f_v − f_t`, where `σ_t = l σ_u + m σ_v`,
//!
//! is read by differences (their Theorem 39: the second derivative, at the contact, of the point's
//! distance from the tool along its inverse trajectory). `θ < 0` is a contact the tool overruns an
//! instant later or earlier (their Proposition 42), `θ = 0` the cusp locus, and `θ > 0` everywhere
//! a sweep that splits into simple sweeps over short enough pieces of its roll (Theorem 33). Apart
//! from θ, every contact is asked whether the tool holds it strictly inside at some time of the
//! roll — the trim set's own definition, read off the tool's material field on a grid of the
//! roll — and how soon (their `ℓ`). A trim near its own time with `θ < 0` is local (a fold); one
//! far from it with `θ > 0` is global (a crossing of two pieces of the sheet, or the tool at an end
//! pose); the census counts each, over the whole sweep and within the blank it cuts.
//!
//! `theta_is_the_inverse_trajectorys_second_derivative` holds the reading to a closed form; the census
//! itself is a tool (`the_census`, ignored), whose table docs/sweep-literature.md records.
use gcs_core::envelope::{self,Motion};
use gcs_core::program::Elaborated;
use gcs_core::solid::{admission,SpatialField,SweepContacts};
use fixtures::{tools,motions::{self,Observer,ROLLED}};

type V = [f64;3];

fn dot(a: V,b: V) -> f64 { a[0]*b[0]+a[1]*b[1]+a[2]*b[2] }
fn cross(a: V,b: V) -> V { [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]] }
fn norm(a: V) -> f64 { dot(a,a).sqrt() }
fn add(a: V,b: V) -> V { [a[0]+b[0],a[1]+b[1],a[2]+b[2]] }
fn scale(a: V,s: f64) -> V { a.map(|x| x*s) }
fn unit(a: V) -> Option<V> { let n = norm(a); (n > 0. && n.is_finite()).then(|| scale(a,1./n)) }

/// How the census samples a sweep: instants over the roll, rings along each face, stations round
/// each ring, and times the trim test reads the tool at.
#[derive(Clone,Copy)]
struct Grid { instants: usize,rings: usize,stations: usize,trims: usize }

const GRID: Grid = Grid {instants:97,rings:40,stations:720,trims:1081};

/// A sweep read for the census: its tool's faces, material, motion and roll, which way each face's
/// chart normal points (+1 out of the tool, −1 into it, 0 a face nowhere on the tool's boundary),
/// and the blank it cuts, where it cuts one.
struct Census { c: SweepContacts,outward: Vec<f64>,blank: Option<SpatialField>,scale: f64 }

impl Census {
    fn read(e: &Elaborated,sweep: &str,body: Option<&str>) -> Census {
        let tolerance = admission::Options::default().axis_tolerance;
        let sk = &e.sketch;
        let c = SweepContacts::read(sk,fixtures::solid(e,sweep),tolerance).unwrap();
        let blank = body.map(|b| admission::static_remainder(sk,fixtures::solid(e,b),tolerance).unwrap().0);
        let tool = c.source_material();
        let mut size = 1_f64;
        let outward = c.patches().iter().map(|s| {
            let [_,vd] = s.domain();
            // a point of the face on the tool's boundary (a face partly hidden by the tool's own
            // Booleans is told by the part that is not), and the field either side of it
            for i in 1..40 { for j in 0..16 {
                let (u,v) = (i as f64/40.,vd[0]+(vd[1]-vd[0])*(j as f64+0.5)/16.);
                let Ok(p) = s.at(u,v) else { continue };
                size = size.max(1.+norm(p.position));
                let Some(n) = unit(cross(p.du,p.dv)) else { continue };
                if tool.value(p.position).abs() > 1e-7*size { continue; }
                let e = 1e-5*size;
                let side = tool.value(add(p.position,scale(n,e)))-tool.value(add(p.position,scale(n,-e)));
                return side.signum();
            }}
            0.
        }).collect();
        Census {c,outward,blank,scale:size}
    }

    fn pose(&self,t: f64) -> Motion { self.c.motion().at(t).unwrap() }

    /// Station `v` of face `patch` brought back into its chart: round a whole revolution it comes
    /// back, along anything else it stops at the ends.
    fn wrap(&self,patch: usize,v: f64) -> f64 {
        let [_,d] = self.c.patches()[patch].domain();
        if self.periodic(patch) { d[0]+(v-d[0]).rem_euclid(d[1]-d[0]) } else { v.clamp(d[0],d[1]) }
    }

    fn periodic(&self,patch: usize) -> bool {
        let s = &self.c.patches()[patch];
        s.periodic() && s.domain()[1] == [0.,1.]
    }

    /// `f(u, v, t)`: the outward unit normal against the velocity at the face's point (u, v) at
    /// time t.
    fn f(&self,patch: usize,u: f64,v: f64,t: f64) -> Option<f64> {
        let s = self.c.patches()[patch].at(u,self.wrap(patch,v)).ok()?;
        let c = envelope::contact(s,self.pose(t)).ok()?;
        Some(self.outward[patch]*c.normal_velocity)
    }

    /// θ at the contact (u, v, t) of face `patch`, and θ over the sum of its three terms' sizes
    /// (its sign, with how far from zero it stands).
    fn theta(&self,patch: usize,u: f64,v: f64,t: f64) -> Option<(f64,f64)> {
        let s = self.c.patches()[patch].at(u,v).ok()?;
        let m = self.pose(t);
        let (su,sv,vel) = (m.vector(s.du),m.vector(s.dv),m.velocity(s.position));
        // σ_t = l σ_u + m σ_v: σ_t lies in the tangent plane at a contact
        let (a,b,d) = (dot(su,su),dot(su,sv),dot(sv,sv));
        let det = a*d-b*b;
        if !(det.abs() > 0.) { return None; }
        let (ru,rv) = (dot(su,vel),dot(sv,vel));
        let (l,mm) = ((d*ru-b*rv)/det,(a*rv-b*ru)/det);
        let h = 1e-6;
        let (ua,ub) = ((u-h).max(0.),(u+h).min(1.));
        let fu = (self.f(patch,ub,v,t)?-self.f(patch,ua,v,t)?)/(ub-ua);
        let [_,vd] = self.c.patches()[patch].domain();
        let (va,vb) = if self.periodic(patch) { (v-h,v+h) } else { ((v-h).max(vd[0]),(v+h).min(vd[1])) };
        let fv = (self.f(patch,u,vb,t)?-self.f(patch,u,va,t)?)/(vb-va);
        let ft = (self.f(patch,u,v,t+h)?-self.f(patch,u,v,t-h)?)/(2.*h);
        let theta = l*fu+mm*fv-ft;
        let size = (l*fu).abs()+(mm*fv).abs()+ft.abs();
        Some((theta,if size > 0. { theta/size } else { 0. }))
    }
}

/// What the trim test says of a point of the contact set at time `t`: whether the tool holds it
/// strictly inside at some time of the roll, the least time from `t` it does (`ℓ`), and whether it
/// does at an end of the roll.
#[derive(Clone,Copy,Debug)]
struct Trim { trimmed: bool,ell: f64,at_end: bool }

/// The trim test against the poses' inverses `inverse[j]` at times `times[j]`.
fn trim(c: &Census,inverse: &[Motion],times: &[f64],y: V,t: f64) -> Trim {
    let tool = c.c.source_material();
    let depth = 1e-9*c.scale;
    let (mut trimmed,mut ell,mut at_end) = (false,f64::INFINITY,false);
    for (j,m) in inverse.iter().enumerate() {
        if tool.value(m.point(y)) < -depth {
            trimmed = true;
            ell = ell.min((times[j]-t).abs());
            at_end |= j == 0 || j+1 == inverse.len();
        }
    }
    Trim {trimmed,ell,at_end}
}

/// One contact sampled: where it is, a face's (its face and chart point, and θ signed and relative)
/// or a fan's (how far round the fan its normal of contact stands, 0 at its first face's normal and
/// 1 at its second's), its trim, and whether it lies in the blank.
#[derive(Clone,Copy,Debug)]
struct Sample {
    y: V,face: Option<(usize,f64,f64)>,theta: Option<(f64,f64)>,alpha: f64,trim: Trim,in_blank: bool,
}

/// The census's counts over one set of samples.
#[derive(Clone,Debug,Default)]
struct Tally {
    faces: usize,fans: usize,negative: usize,kept: usize,local: usize,near: usize,global: usize,
    global_end: usize,anomalies: usize,fan_trimmed: usize,fan_near: usize,fan_near_join: usize,
    ells: Vec<f64>,fan_ells: Vec<f64>,
}

impl Tally {
    fn of(samples: &[Sample],step: f64) -> Tally {
        let mut t = Tally::default();
        // beside a fold: overrun within a degree of its own time (or three of the trim grid's
        // steps, where those are coarser), where the singular trim curve runs from θ = 0 to the
        // crossing it ends on
        let near = (3.*step).max(1_f64.to_radians());
        for s in samples {
            let Some((_,rel)) = s.theta else {
                if s.face.is_some() { continue; }
                t.fans += 1;
                if s.trim.trimmed { t.fan_trimmed += 1; t.fan_ells.push(s.trim.ell); }
                // a convex corner's fan is never overrun from close by (their Proposition 19)
                if s.trim.trimmed && s.trim.ell <= near {
                    t.fan_near += 1;
                    if s.alpha.min(1.-s.alpha) < 0.05 { t.fan_near_join += 1; }
                }
                continue;
            };
            t.faces += 1;
            let negative = rel < 0.;
            if negative { t.negative += 1; }
            match (s.trim.trimmed,negative) {
                (false,false) => t.kept += 1,
                (false,true) => { t.kept += 1; t.anomalies += 1; }
                (true,true) => t.local += 1,
                (true,false) if s.trim.ell <= near => t.near += 1,
                (true,false) => {
                    t.global += 1;
                    if s.trim.at_end { t.global_end += 1; }
                    t.ells.push(s.trim.ell);
                }
            }
        }
        t
    }

    fn row(&self) -> String {
        let pc = |n: usize| if self.faces == 0 { "—".to_string() }
            else { format!("{:.1}",100.*n as f64/self.faces as f64) };
        let mut ells = self.ells.clone();
        ells.sort_by(f64::total_cmp);
        let ell = if ells.is_empty() { "—".to_string() }
            else { format!("{:.1}° ({:.1}°)",ells[0].to_degrees(),ells[ells.len()/2].to_degrees()) };
        let mut fan_ells = self.fan_ells.clone();
        fan_ells.sort_by(f64::total_cmp);
        let fan_ell = if fan_ells.is_empty() { String::new() }
            else { format!(", ℓ ≥ {:.2}° ({:.1}°)",fan_ells[0].to_degrees(),fan_ells[fan_ells.len()/2].to_degrees()) };
        let fans = if self.fans == 0 { "—".to_string() } else {
            format!("{} ({:.0}% trimmed, {} from close by, {} of them within 5% of a join{fan_ell})",self.fans,
                100.*self.fan_trimmed as f64/self.fans as f64,self.fan_near,self.fan_near_join)
        };
        format!("{} | {} | {} | {} | {} | {} ({} at an end) | {} | {} | {}",self.faces,pc(self.negative),
            pc(self.kept),pc(self.local),pc(self.near),pc(self.global),pc(self.global_end),ell,self.anomalies,fans)
    }
}

/// What the instant-by-instant search said of the rings: the most roots one ring had at one
/// instant, how many (ring, instant) pairs had more than two, and how many were in contact all
/// round (a flat face the motion keeps in its plane, or a ring in contact at every point).
#[derive(Clone,Copy,Debug,Default)]
struct Rings { most: usize,over_two: usize,all_round: usize }

/// Every contact of the sweep at instant `t`: each face's rings searched round, each convex
/// corner's fan.
fn instant(c: &Census,g: Grid,inverse: &[Motion],times: &[f64],t: f64) -> (Vec<Sample>,Rings) {
    let tool = c.c.source_material();
    let pose = c.pose(t);
    let mut out = Vec::new();
    let mut rings = Rings::default();
    let on = 1e-7*c.scale;
    let in_blank = |y: V| c.blank.as_ref().is_some_and(|b| b.value(y) < 0.);
    for (patch,surface) in c.c.patches().iter().enumerate() {
        if c.outward[patch] == 0. { continue; }
        let [_,vd] = surface.domain();
        let periodic = c.periodic(patch);
        let n = g.stations;
        let station = |j: usize| if periodic { vd[0]+(vd[1]-vd[0])*j as f64/n as f64 }
            else { vd[0]+(vd[1]-vd[0])*j as f64/(n-1) as f64 };
        for i in 0..g.rings {
            let u = (i as f64+0.5)/g.rings as f64;
            let values: Vec<Option<f64>> = (0..n).map(|j| c.f(patch,u,station(j),t)).collect();
            let size = values.iter().flatten().fold(0_f64,|a,x| a.max(x.abs()));
            if size <= 1e-10*c.scale {
                if values.iter().any(Option::is_some) { rings.all_round += 1; }
                continue;
            }
            let pairs = if periodic { n } else { n-1 };
            let mut roots = 0;
            for j in 0..pairs {
                let (Some(a),Some(b)) = (values[j],values[(j+1)%n]) else { continue };
                if !(a == 0. || a*b < 0.) { continue; }
                let (mut lo,mut hi) = (station(j),if j+1 == n { vd[1] } else { station(j+1) });
                let mut flo = a;
                for _ in 0..60 {
                    let mid = 0.5*(lo+hi);
                    let Some(fm) = c.f(patch,u,mid,t) else { break };
                    if (fm < 0.) == (flo < 0.) { lo = mid; flo = fm; } else { hi = mid; }
                }
                let v = c.wrap(patch,0.5*(lo+hi));
                let Ok(s) = surface.at(u,v) else { continue };
                // a point of a face the tool's Booleans hide is no point of its boundary
                if tool.value(s.position).abs() > on { continue; }
                roots += 1;
                let y = pose.point(s.position);
                out.push(Sample {y,face:Some((patch,u,v)),theta:c.theta(patch,u,v,t),alpha:f64::NAN,
                    trim:trim(c,inverse,times,y,t),in_blank:in_blank(y)});
            }
            rings.most = rings.most.max(roots);
            if roots > 2 { rings.over_two += 1; }
        }
    }
    // a convex corner's fan: the circle (or line) two faces meet on is in contact wherever one of
    // the normals between theirs is square to its velocity, where their own `f` differ in sign
    for edge in c.c.edges() {
        let Some((_,[pa,pb])) = c.c.corner(edge) else { continue };
        if !(tool.value(scale(add(pa,pb),0.5)) < 0.) { continue; }
        let ([a,b],[ua,ub]) = (edge.patches,edge.ends);
        if c.outward[a] == 0. || c.outward[b] == 0. { continue; }
        let (sa,sb) = (&c.c.patches()[a],&c.c.patches()[b]);
        let [_,vd] = sa.domain();
        for j in 0..g.stations {
            let v = vd[0]+(vd[1]-vd[0])*(j as f64+0.5)/g.stations as f64;
            let (Ok(p),Ok(q)) = (sa.at(ua,v),sb.at(ub,v)) else { continue };
            if norm(add(p.position,scale(q.position,-1.))) > on || tool.value(p.position).abs() > on { continue; }
            let normal = |s: &envelope::SurfacePoint,o: f64| unit(cross(s.du,s.dv)).map(|n| scale(n,o));
            let (Some(na),Some(nb)) = (normal(&p,c.outward[a]),normal(&q,c.outward[b])) else { continue };
            let vel = pose.velocity(p.position);
            let (ga,gb) = (dot(pose.vector(na),vel),dot(pose.vector(nb),vel));
            if ga*gb < 0. {
                let y = pose.point(p.position);
                // `f` is linear in the normal, so (1 − α) ga + α gb = 0 (their §4.1)
                out.push(Sample {y,face:None,theta:None,alpha:ga/(ga-gb),trim:trim(c,inverse,times,y,t),
                    in_blank:in_blank(y)});
            }
        }
    }
    (out,rings)
}

/// The census of one sweep: its samples over the roll and what the rings said.
fn census(c: &Census,g: Grid) -> (Vec<Sample>,Rings,f64) {
    let [a,b] = c.c.domain();
    let times: Vec<f64> = (0..g.trims).map(|j| a+(b-a)*j as f64/(g.trims-1) as f64).collect();
    let inverse: Vec<Motion> = times.iter().map(|&t| c.pose(t).inverse()).collect();
    let found = gcs_core::par::indices(g.instants,|k| {
        let t = a+(b-a)*k as f64/(g.instants-1) as f64;
        instant(c,g,&inverse,&times,t)
    });
    let mut samples = Vec::new();
    let mut rings = Rings::default();
    for (s,r) in found {
        samples.extend(s);
        rings.most = rings.most.max(r.most);
        rings.over_two += r.over_two;
        rings.all_round += r.all_round;
    }
    (samples,rings,(b-a)/(g.trims-1) as f64)
}

/// How many contact times one tool point has over the roll (E2's question), on a coarse grid of
/// the faces: the most, and the share of points with two or more; None under a constant twist,
/// whose contacts are fixed on the tool (every time or none).
fn times_per_point(c: &Census) -> Option<(usize,f64,usize)> {
    if c.c.motion().twist().is_some() { return None; }
    let (mut most,mut several,mut points,mut degenerate) = (0,0,0,0);
    for (patch,s) in c.c.patches().iter().enumerate() {
        if c.outward[patch] == 0. { continue; }
        let [_,vd] = s.domain();
        for i in 0..24 { for j in 0..96 {
            let (u,v) = ((i as f64+0.5)/24.,vd[0]+(vd[1]-vd[0])*(j as f64+0.5)/96.);
            match c.c.at_source_over(patch,u,v,c.c.domain(),1e-10) {
                Ok(r) => { points += 1; most = most.max(r.len()); if r.len() >= 2 { several += 1; } }
                Err(_) => degenerate += 1,
            }
        }}
    }
    Some((most,several as f64/points.max(1) as f64,degenerate))
}

/// The sphere of radius 1 turned about an axis half its radius from its centre: θ in closed form.
/// The sweep of a ball depends only on its centre's path, so θ is a translated ball's along the
/// circle of radius ½: `−⟨b̈, N⟩ + κ|ḃ|²` (Theorem 39, `κ = 1`), which at latitude φ from the turn's
/// plane is `(1 − 2 cos φ)/4` on the side toward the axis and `(1 + 2 cos φ)/4` away from it. The
/// side toward the axis overruns itself about its equator (`θ < 0` within 60° of it), and every
/// such contact is trimmed.
#[test]
fn theta_is_the_inverse_trajectorys_second_derivative() {
    let source = format!("{}{}{}",tools::sphere(0.),motions::TURN_OFFSET,motions::swept("turn",-60.,60.));
    let e = fixtures::read(&source);
    let c = Census::read(&e,"swept",None);
    let g = Grid {instants:3,rings:24,stations:720,trims:721};
    let [a,b] = c.c.domain();
    let times: Vec<f64> = (0..g.trims).map(|j| a+(b-a)*j as f64/(g.trims-1) as f64).collect();
    let inverse: Vec<Motion> = times.iter().map(|&t| c.pose(t).inverse()).collect();
    let (mut toward_axis,mut away,mut overrun) = (0,0,0);
    for t in [-0.3,0.,0.4] {
        let (samples,rings) = instant(&c,g,&inverse,&times,t);
        assert_eq!(rings.most,2,"a ring of a ball has two contacts at an instant");
        let pose = c.pose(t);
        let (pivot,centre) = (pose.point([2.5,0.,0.]),pose.point([3.,0.,0.]));
        for s in &samples {
            let normal = unit(add(s.y,scale(centre,-1.))).unwrap();
            let latitude = normal[2].asin();
            let toward = dot(normal,add(pivot,scale(centre,-1.))) > 0.;
            let expected = if toward { (1.-2.*latitude.cos())/4. } else { (1.+2.*latitude.cos())/4. };
            let (theta,_) = s.theta.unwrap();
            assert!((theta-expected).abs() < 1e-4,"θ {theta} where the closed form is {expected} (latitude {:.1}°, {})",
                latitude.to_degrees(),if toward { "toward the axis" } else { "away" });
            if theta < -1e-3 {
                assert!(s.trim.trimmed,"θ < 0 is overrun (Proposition 42)");
                overrun += 1;
            }
            if toward { toward_axis += 1 } else { away += 1 }
        }
    }
    assert!(toward_axis > 50 && away > 50 && overrun > 20,"{toward_axis} toward, {away} away, {overrun} overrun");
}

/// A dumbbell along the vertical through (3, 0): a bar of radius 0.25 from z = −1 to 1 and a ball
/// of radius 0.5 at each end, two concave creases where the bar enters the balls.
const DUMBBELL: &str = "\
unit mm
use std
in std.front {
construction centerline spindle := line(std.origin, hint((0, 1)))
fix((0, 1)) spindle.p2
private c0 := point
private c1 := point
private c2 := point
private c3 := point
fix((3, -1)) c0
fix((3.25, -1)) c1
fix((3.25, 1)) c2
fix((3, 1)) c3
private bottom := line(c0, c1)
private wall := line(c1, c2)
private top := line(c2, c3)
private ax := line(c3, c0)
private ca := point
private ba := point
private ta := point
fix((3, 1)) ca
fix((3, 0.5)) ba
fix((3, 1.5)) ta
private da := line(ba, ta)
private ma := arc(center: ca, start: ba, end: ta)
private cb := point
private bb := point
private tb := point
fix((3, -1)) cb
fix((3, -1.5)) bb
fix((3, -0.5)) tb
private db := line(bb, tb)
private mb := arc(center: cb, start: bb, end: tb)
}
construction bar := solid(face(bottom, wall, top, ax), about: ax)
construction ball_a := solid(face(ma, da), about: da)
construction ball_b := solid(face(mb, db), about: db)
construction tool := solid(bar)
ball_a union tool
ball_b union tool
";

/// One sweep the census reads: its name in the table, its document, the sweep and the body it cuts.
struct Case { name: &'static str,document: Document,sweep: &'static str,body: Option<&'static str> }

enum Document { Source(String),Example(&'static str),Gear([f64;3]) }

fn cases() -> Vec<Case> {
    use Document::*;
    // a tool rolled through `post` over (-roll, roll), and the part it cuts there
    let rolled = |name,tool: &str,motion: &str,roll: f64,post: &str| Case {name,sweep:"removal",body:Some("part"),
        document:Source(format!("{tool}{motion}construction removal := sweep(tool over t in ({}deg, {roll}deg), \
            {ROLLED})\n{post}",-roll))};
    // a tool swept over (-60°, 60°), cutting nothing
    let alone = |name,tool: &str,motion: &str,reading: &str| Case {name,sweep:"swept",body:None,
        document:Source(format!("{tool}{motion}{}",motions::swept(reading,-60.,60.)))};
    let example = |name,file,sweep,body| Case {name,document:Example(file),sweep,body};
    let pinion = |name,design| Case {name,document:Gear(design),sweep:"pair.pinion.cut.removal",
        body:Some("pair.pinion.body")};
    let skew = motions::cradle_roll(0.25,Observer::Skew);
    vec![
        example("torus, compound roll (`swept_tumble.sv`)","swept_tumble.sv","tumbled",None),
        rolled("torus, skew cradle roll (generating, admitted)",&tools::torus(2.),&skew,75.,&tools::skew_post(4.)),
        rolled("lens (two sharp rims), skew cradle roll",&tools::ring_lens(2.),&skew,75.,&tools::skew_post(4.)),
        rolled("torus, crossed roll (a ring in contact at every time)",&tools::torus(2.),motions::CROSSED_ROLL,60.,
            &tools::post(4.,0.4,-2.,2.)),
        alone("cylinder tumbled (a turn whose axis meets the tool's)",tools::CYLINDER,motions::TUMBLE,"turn"),
        alone("cylinder slid across (a slide)",tools::CYLINDER,&motions::slide_x(8.),"feed"),
        alone("cylinder tilted 30°, turned off its axis",&tools::tilted_cylinder(30.),
            &motions::turn_about(2.5,0.,2.5,5.),"turn"),
        alone("cylinder tilted 30°, roll",&tools::tilted_cylinder(30.),&motions::roll(0.25),ROLLED),
        alone("dumbbell, roll",DUMBBELL,&motions::roll(0.25),ROLLED),
        alone("thin plate (its sides), roll",&tools::thin_plate(0.2),&motions::roll(0.25),ROLLED),
        example("torus turned across a ring (`solid_sweeps/turn.sv`)","solid_sweeps/turn.sv","groove",Some("block")),
        example("V wheel screwed along a rod (`solid_sweeps/vthread.sv`)","solid_sweeps/vthread.sv","groove",
            Some("rod")),
        pinion("hypoid pinion's crown, bevel (offset 0, admitted)",[0.,0.,35.]),
        pinion("hypoid pinion's crown, offset 20 (refused, its flank folds)",[20.,0.,35.]),
        pinion("hypoid pinion's crown, offset 30 (refused, undercut)",[30.,0.,35.]),
    ]
}

/// The census of every case: `cargo test --manifest-path rust/Cargo.toml -p gcs-core --test core
/// sweep_census::the_census -- --ignored --nocapture` (`SOLVENT_CENSUS` a word of the cases'
/// names to read only those). Prints the table docs/sweep-literature.md records: for each case,
/// what the rings said, how many contact times a tool point has (E2's question), and the samples'
/// split by θ and trim over the whole sweep and within the blank.
#[test]
#[ignore = "a tool: the census docs/sweep-literature.md records"]
fn the_census() {
    let only = std::env::var("SOLVENT_CENSUS").ok();
    let mut whole = Vec::new();
    let mut blank = Vec::new();
    for case in cases() {
        if only.as_ref().is_some_and(|w| !case.name.contains(w.as_str())) { continue; }
        let started = std::time::Instant::now();
        let e = match &case.document {
            Document::Source(s) => fixtures::read(s),
            Document::Example(name) => fixtures::read(&std::fs::read_to_string(
                std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../examples").join(name)).unwrap()),
            Document::Gear(design) => fixtures::gear::read(&fixtures::gear::fabricated(&fixtures::gear::source(),
                *design,0.,0.,0.)),
        };
        let c = Census::read(&e,case.sweep,case.body);
        let [a,b] = c.c.domain();
        let (samples,rings,step) = census(&c,GRID);
        let per_point = match times_per_point(&c) {
            None => "fixed on the tool".to_string(),
            Some((most,several,degenerate)) => format!("≤ {most}, {:.0}% ≥ 2{}",100.*several,
                if degenerate > 0 { format!(", {degenerate} at every time") } else { String::new() }),
        };
        let head = format!("{} | {:.0}° | {} ({} > 2, {} all round) | {per_point}",case.name,(b-a).to_degrees(),
            rings.most,rings.over_two,rings.all_round);
        whole.push(format!("| {head} | {} |",Tally::of(&samples,step).row()));
        if c.blank.is_some() {
            let inside: Vec<Sample> = samples.iter().copied().filter(|s| s.in_blank).collect();
            blank.push(format!("| {} | {} |",case.name,Tally::of(&inside,step).row()));
        }
        eprintln!("{}: {:?}",case.name,started.elapsed());
    }
    let columns = "face samples | θ < 0 | kept | local (θ < 0) | beside a fold | global | \
        least ℓ global (median) | θ < 0 kept | fan samples";
    println!("\nThe whole sweep:\n\n| case | roll | roots a ring (> 2, all round) | contact times a point | {columns} |");
    println!("|{}", "---|".repeat(14));
    for r in &whole { println!("{r}"); }
    println!("\nWithin the blank:\n\n| case | {columns} |");
    println!("|{}", "---|".repeat(10));
    for r in &blank { println!("{r}"); }
}
