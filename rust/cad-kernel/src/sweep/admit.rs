//! The swept classes' sampled rows (docs/generating-sweeps.md in the solver this kernel was carved
//! from), asked of one placement of a sweep before any construction: a relative rotation's (the
//! generating class, rows T2, M2 and E0–E4, and T3 where a prism's cap reaches the blank) and a
//! screw's, a turn's or a slide's (the constant-twist class, T2, E1 and S1–S4, its characteristic
//! walked). They read a sweep's contacts (`tool::SweepContacts`) and the blank as a closure in the
//! sweep's frame, and nothing else: a caller reads its own document into those, says which sweep
//! a refusal names, and asks every placement it has. The rows are **sampled**, never certified, and
//! the evidence says so: a sweep they pass has shown no failure at the samples it names, which is
//! not a proof that none exists between them.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use super::condition::Condition;
use super::cutter::Cutter;
use super::characteristic::{Characteristic,Failure};
use super::envelope::{Motion,SurfacePoint};
use super::sheet::Capped;
use super::tool::{SweepContacts,TimedContact,ToolSurface};
use crate::util::space::{add,sub,dot,cross,norm,scale};
use std::{collections::BTreeMap,f64::consts::PI};

type V = [f64;3];

/// Sampling density. Each revolved tool face is sampled `rows` along its profile by `columns`
/// around the band of its revolution that reaches the blank, the band found first by a
/// `coarse_rows` by `coarse_columns` pass over the whole revolution.
#[derive(Clone,Copy,Debug)]
pub struct Options {
    pub rows: usize,
    pub columns: usize,
    pub coarse_rows: usize,
    pub coarse_columns: usize,
    pub axis_tolerance: f64,
    /// How far inside the blank's field a point must be to count as in the blank.
    pub margin: f64,
    pub root_tolerance: f64,
    /// A generated area factor below this, relative to the tool's own, is a fold.
    pub least_factor: f64,
}

impl Default for Options {
    fn default() -> Self {
        Options {rows:100,columns:400,coarse_rows:24,coarse_columns:1440,axis_tolerance:crate::sweep::AXIS_TOLERANCE,
            margin:1e-6,root_tolerance:1e-9,least_factor:1e-3}
    }
}

/// How the evidence was obtained. Only sampling exists; a certified basis would be a second
/// variant, and a caller matching on this one learns of it when it arrives.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Basis { Sampled { rows: usize,columns: usize } }

/// Which tool points the checks read the blank at, and whether each was inside: two placements
/// agreeing on all of them would compute the same checks to the last number.
#[derive(Default)]
pub struct Reads { pub points: Vec<V>,pub inside: Vec<bool> }

impl Reads {
    pub fn new() -> Reads { Reads {points:Vec::new(),inside:Vec::new()} }
    /// Read the blank at `p`, and remember that it was read and what it said.
    pub fn read(&mut self,inside: &dyn Fn(V) -> bool,p: V) -> bool { let i = inside(p); self.points.push(p); self.inside.push(i); i }
    pub fn extend(&mut self,other: Reads) { self.points.extend(other.points); self.inside.extend(other.inside); }
    /// Remember a read made elsewhere.
    pub fn push(&mut self,p: V,inside: bool) { self.points.push(p); self.inside.push(inside); }
}

/// What one sample of a tool face says of itself alone, for the fine pass: its point, the rate of its
/// contact condition, and — where its roots could be found — each root, whether it is in the blank,
/// and for one that is, the generated surface's area factor there (`area_factor`).
struct Sampled { s: SurfacePoint,rate: Option<crate::sweep::motion::NormalVelocity>,roots: Option<Vec<(TimedContact,bool,Option<f64>)>> }

/// E3's area factor at a contact `t` of the tool point at (u, v) of face `patch`: the generated
/// surface's oriented area factor on the same branch, by central differences, against the tool
/// surface's own. None where a neighbouring root or point cannot be read, or the tool's own area
/// vanishes.
fn area_factor(surface: &ToolSurface,patch: usize,
    roots: &(dyn Fn(usize,f64,f64) -> Result<Vec<TimedContact>,String>+Sync),t: &TimedContact,u: f64,v: f64) -> Option<f64> {
    let h = 1e-5;
    let (ua,ub) = ((u-h).max(0.),(u+h).min(1.));
    // round a revolution `v` comes back; along a prism it stops at the caps
    let wrap = |vv: f64| if surface.periodic() { vv.rem_euclid(1.) } else { vv };
    let (va,vb) = if surface.periodic() { (v-h,v+h) } else { ((v-h).max(0.),(v+h).min(1.)) };
    let near = |uu: f64,vv: f64| -> Option<V> {
        roots(patch,uu,wrap(vv)).ok()?.into_iter()
            .filter(|x| x.root.branch == t.root.branch)
            .min_by(|a,b| (a.root.time-t.root.time).abs().total_cmp(&(b.root.time-t.root.time).abs()))
            .map(|x| x.contact.position)
    };
    let (Some(fa),Some(fb),Some(ga),Some(gb)) = (near(ua,v),near(ub,v),near(u,va),near(u,vb)) else { return None };
    let at = |uu: f64,vv: f64| surface.at(uu,wrap(vv)).map(|s| s.position).ok();
    let (Some(sa),Some(sb),Some(ta),Some(tb)) = (at(ua,v),at(ub,v),at(u,va),at(u,vb)) else { return None };
    let (du,dv) = (ub-ua,vb-va);
    let area = norm(cross(sub(sb,sa).map(|x| x/du),sub(tb,ta).map(|x| x/dv)));
    if area == 0. { return None; }
    Some(dot(cross(sub(fb,fa).map(|x| x/du),sub(gb,ga).map(|x| x/dv)),t.contact.normal)/area)
}

/// The coarse pass's `j`-th of `n` stations of `v` over a face: round a revolution `n` steps of a
/// turn, along a prism from one cap to the other, both ends included.
fn coarse_v(surface: &ToolSurface,j: usize,n: usize) -> f64 {
    if surface.periodic() { j as f64/n as f64 } else { coarse_at(j as f64,n) }
}

/// Station `at` (fractional) of `n` from one cap to the other.
fn coarse_at(at: f64,n: usize) -> f64 { (at/(n.max(2)-1) as f64).clamp(0.,1.) }

/// What the fine pass over one tool face found.
struct Fine { reads: Reads,samples: usize,near_double_roots: usize,hits: Vec<(Hit,usize,usize)>,gaps: Vec<f64>,least: f64,
    signs: BTreeMap<(usize,usize),([usize;2],[Option<V>;2])> }

/// How many normals of a cap's rim's fan E4 reads at each of its points, ends excluded.
const FAN_READS: usize = 16;

/// One in-blank contact of the fine pass.
#[derive(Clone,Copy)]
struct Hit { source: V,position: V,normal: V }

/// A point of a convex edge between two of the tool's faces, which carries every normal between
/// theirs: each face's outward normal and the way into it from the edge, the patch the edge is
/// named by, and whether it is a crease where a drafted prism's sides fold (else a cap's rim).
#[derive(Clone,Copy)]
struct Fan { point: V,faces: [(V,V);2],patch: usize,crease: bool }


/// T2, E1 and T1 for a prism's caps for one placement, asked of every class before anything that
/// might be reported in their place: the blank's reads, the poses the roll is sampled at, and
/// whether each cap of `lone` (the tool, where it is a prism standing alone; its `start`, then its
/// `end`) reaches the blank. A whole turn has no ends to clear: the tool at either lies within
/// what the turn sweeps (`SweepContacts::whole`, #684 N).
fn ends(c: &SweepContacts,inside: &(dyn Fn(V) -> bool+Sync),options: &Options,lone: Option<&Lone>)
    -> Result<(Reads,Vec<Motion>,[bool;2]),Failure> {
    let tool = c.source_material();
    let on_tool = |p: V| tool.value(p).abs() < 1e-6;
    let domain = c.domain();
    let pose = |t: f64| c.motion().at(t).map_err(|m| (Condition::Motion,m,None));
    let limits = if c.whole() { Vec::new() } else { vec![pose(domain[0])?,pose(domain[1])?] };
    let mut reads = Reads::new();
    // T2: a concave corner carried through the blank trims two envelopes against each other.
    // Asked first, since the crossing it makes would otherwise be reported in its place.
    let steps = 64;
    let poses: Vec<Motion> = (0..=steps).map(|k| pose(domain[0]+(domain[1]-domain[0])*k as f64/steps as f64))
        .collect::<Result<_,_>>()?;
    for edge in c.edges() {
        let ([a,b],[ua,_]) = (edge.patches,edge.ends);
        let (Some(pa),Some(pb)) = (c.patches().get(a),c.patches().get(b)) else { continue };
        let Some((corner,[sa,sb])) = c.corner(edge) else { continue };
        if !on_tool(corner) { continue; }
        let (ta,tb) = (sub(sa,corner),sub(sb,corner));
        // Opposite tangents: the profile is smooth there.
        if dot(ta,tb) < -(1.-1e-9)*norm(ta)*norm(tb) { continue; }
        let chord: V = std::array::from_fn(|k| 0.5*(sa[k]+sb[k]));
        if tool.value(chord) <= 0. { continue; }
        for j in 0..options.coarse_columns {
            let Ok(point) = pa.at(ua,j as f64/options.coarse_columns as f64) else { continue };
            if let Some(p) = poses.iter().map(|m| m.point(point.position)).find(|&p| reads.read(inside,p)) {
                return Err((Condition::Corner,format!("`{}` meets `{}` at a concave corner",pa.name(),pb.name()),Some(p)));
            }
        }
    }
    // E1, over the whole tool, before anything that might be reported in its place: its caps
    // too, where one reaches the blank (below)
    let clear = |asked: &dyn Fn(&ToolSurface) -> bool| -> Result<Reads,Failure> {
        let mut reads = Reads::new();
        if limits.is_empty() { return Ok(reads) }
        let patches: Vec<&ToolSurface> = c.patches().iter().filter(|s| asked(s)).collect();
        for own in crate::util::par::map(&patches,|surface| -> Result<Reads,Failure> {
            let mut own = Reads::new();
            for i in 0..=options.coarse_rows { for j in 0..options.coarse_columns {
                let Ok(s) = surface.at(i as f64/options.coarse_rows as f64,coarse_v(surface,j,options.coarse_columns)) else { continue };
                if !on_tool(s.position) { continue; }
                for limit in &limits {
                    let p = limit.point(s.position);
                    if own.read(inside,p) { return Err((Condition::Clearance,
                        "the tool at a limit of its roll lies in the blank".into(),Some(p))); }
                }
            }}
            Ok(own)
        }) { reads.extend(own?); }
        Ok(reads)
    };
    reads.extend(clear(&|s| s.cap().is_none())?);
    // T1 for a prism: a cap passing through the blank is cut only by a prism standing alone as the
    // tool, its profile one loop, sectioned across (T3, #684 F); otherwise the class cuts with a
    // prism's sides only (under a planar sweep a cap, square to the turn, is in contact at every
    // time or none, and M2 refuses it). Each cap is asked by its rim, its sides' ends at `v` (a
    // drafted side's end at its parting plane is no rim, but the crease its halves meet in, #684 P).
    let mut capped = [false;2];
    for (end,v) in [(0,0.),(1,1.)] {
        let rim = c.patches().iter().enumerate()
            .filter(|&(k,s)| !s.periodic() && s.cap().is_none() && !c.creases().iter().any(|crease| crease[1-end] == k))
            .find_map(|(_,surface)| (0..=options.coarse_rows).find_map(|i| { let s = surface.at(i as f64/options.coarse_rows as f64,v).ok()?;
                poses.iter().map(|m| m.point(s.position)).find(|&p| reads.read(inside,p)) }).map(|p| (surface.name(),p)));
        let Some((name,p)) = rim else { continue };
        if lone.is_none() {
            return Err((Condition::Tool,format!("a cap of the prism `{name}` bounds passes through the blank, and the class cuts \
                with a cap only of a prism standing alone as the tool, its profile one loop: make the prism longer than the blank \
                is thick"),Some(p)));
        }
        capped[end] = true;
    }
    reads.extend(clear(&|s| s.cap().is_some_and(|end| capped[end]))?);
    Ok((reads,poses,capped))
}

/// A tool that is one prism standing alone, its profile one loop: what the class sections across
/// where its cap reaches the blank (T3, #684 F) — its cutter (millimetres) and the model's
/// millimetres a unit.
pub struct Lone { cutter: Cutter,scale: f64 }

impl Lone {
    /// The cutter of a prism standing alone as the tool (millimetres), its profile one loop, and
    /// `scale` millimetres a unit of the tool's contacts; none where the cutter is no prism's.
    pub fn new(cutter: Cutter,scale: f64) -> Option<Lone> {
        cutter.prism_profile()?;
        Some(Lone {cutter,scale})
    }

    /// T3: the direction across the profile whose sections the contacts in the blank (`hits`, at
    /// their tool points) keep farthest from the profile's corners and its ends — a band that
    /// meets none, with room for the sheet's margins: a quarter of its width and two of the
    /// reach's 96 stations either side. Directions are tried every 5 degrees, and the best taken; the
    /// sampled hits stand for the contacts, so the row is sampled too.
    fn sections(&self,hits: &[(Hit,usize,usize)]) -> Result<Capped,Failure> {
        let (cutter,mm) = (&self.cutter,self.scale);
        let (e1,e2) = crate::util::space::across(cutter.axis);
        // each direction's band, how far it keeps from what it comes nearest (an end, or the corner
        // at a point), which of the band's hits comes nearest it, and what the margins need
        let mut best: Option<(Capped,Option<V>,usize,f64)> = None;
        for k in 0..36 {
            let (sn,cs) = (PI*k as f64/36.).dsin_cos();
            let Some(spread) = cutter.spread(add(scale(e1,cs),scale(e2,sn))) else { continue };
            let at: Vec<f64> = hits.iter().map(|(h,..)| dot(spread.across,sub(scale(h.source,mm),cutter.origin))).collect();
            let (lo,hi) = (0..at.len()).fold((0,0),|(l,h),i| (if at[i] < at[l] { i } else { l },if at[i] > at[h] { i } else { h }));
            let band = [at[lo],at[hi]];
            let need = 0.25*(band[1]-band[0])+2.*(spread.range[1]-spread.range[0])/96.;
            let mut near = (band[0]-spread.range[0],None,lo);
            if spread.range[1]-band[1] < near.0 { near = (spread.range[1]-band[1],None,hi); }
            for &(c,p) in &spread.corners {
                let (gap,end) = if c < band[0] { (band[0]-c,lo) } else if c > band[1] { (c-band[1],hi) } else { (0.,lo) };
                if gap < near.0 { near = (gap,Some(p),end); }
            }
            if best.as_ref().is_none_or(|b| near.0-need > b.0.clearance-b.3) {
                best = Some((Capped {across:spread.across,band,clearance:near.0},near.1,near.2,need));
            }
        }
        let (capped,corner,at,need) = best.expect("a lone prism's profile is one loop of lines and arcs, sectioned every way");
        if capped.clearance < need {
            let what = match corner {
                Some(p) => format!("a corner of the profile, where a section's side leaps from one face's normal to the next's, at \
                    ({:.4}, {:.4}, {:.4}) on the tool",p[0]/mm,p[1]/mm,p[2]/mm),
                None => "the profile's end, where a section closes to a ruling of a side".into(),
            };
            return Err((Condition::Sections,format!("the prism's cap reaches the blank, and no direction across its profile \
                sections it clear of its corners and its ends: across the best, the contacts in the blank reach {:.4} to {:.4} mm, \
                {:.4} mm from {what}, against the {need:.4} mm the sheet's margins need",capped.band[0],capped.band[1],
                capped.clearance),Some(hits[at].0.position)))
        }
        Ok(capped)
    }
}

/// M2, E1–E4 and T2 for one placement, `inside` being the blank read in the sweep's frame, and T3
/// where a cap of `lone` reaches the blank. The tool's faces are sampled side by side, each on its
/// own core, and their findings taken in the faces' order: the first failure in that order is the
/// one reported, as one pass over them would.
pub fn check(c: &SweepContacts,inside: &(dyn Fn(V) -> bool+Sync),options: &Options,lone: Option<&Lone>)
    -> Result<(Evidence,Reads),Failure> {
    let tool = c.source_material();
    let on_tool = |p: V| tool.value(p).abs() < 1e-6;
    let domain = c.domain();
    let middle = 0.5*(domain[0]+domain[1]);
    let pose = |t: f64| c.motion().at(t).map_err(|m| (Condition::Motion,m,None));
    let times = [pose(domain[0])?,pose(middle)?,pose(domain[1])?];
    let roots = |patch: usize,u: f64,v: f64| c.at_source_over(patch,u,v,domain,options.root_tolerance);
    let (mut reads,poses,capped) = ends(c,inside,options,lone)?;
    // M2 at the poles: where a face's profile meets the tool's axis the surface has no normal of
    // its own, so the sampled checks below cannot evaluate it. Its normal is the axis, the limit
    // along the profile; a pole whose contact condition is zero at every time and whose path
    // enters the blank is refused, as a sampled point would be.
    for surface in c.patches() {
        for end in [0.,1.] {
            let (Ok(s),Ok(near)) = (surface.at(end,0.),surface.at(if end == 0. { 1e-6 } else { 1.-1e-6 },0.)) else { continue };
            if norm(s.dv) > 1e-9*(1.+norm(s.position)) || !on_tool(s.position) { continue; }
            // The pole's normal is the revolution axis, oriented as the surface beside it: the
            // axis is square to the small circle a point beside the pole turns on.
            let inward = if end == 0. { 1e-6 } else { 1.-1e-6 };
            let ring: Vec<V> = [0.,1./3.,2./3.].iter().filter_map(|&v| surface.at(inward,v).ok().map(|q| q.position)).collect();
            if ring.len() < 3 { continue; }
            let axis = cross(sub(ring[1],ring[0]),sub(ring[2],ring[0]));
            let beside = cross(near.du,near.dv);
            if norm(axis) == 0. || norm(beside) == 0. { continue; }
            let n = axis.map(|x| x/norm(axis)*dot(axis,beside).signum());
            let seed = if n[0].abs() < 0.9 { [1.,0.,0.] } else { [0.,1.,0.] };
            let a = cross(seed,n); let a = a.map(|x| x/norm(a));
            let pole = SurfacePoint {position:s.position,du:a,dv:cross(n,a)};
            let Ok(g) = c.motion().normal_velocity(pole) else { continue };
            let scale = 1.+norm(s.position);
            let values = [domain[0],middle,domain[1]].map(|t| g.at(t).unwrap_or(f64::NAN));
            if !values.iter().all(|x| x.abs() <= options.root_tolerance*scale) { continue; }
            if let Some(p) = poses.iter().map(|m| m.point(s.position)).find(|&p| reads.read(inside,p)) {
                return Err((Condition::Stationary,format!("the pole of `{}`, where its profile meets its axis, is in contact \
                    at every time",surface.name()),Some(p)));
            }
        }
    }
    // E3's sign: the area factor times the rate of the contact condition. The factor alone
    // runs through infinity and changes sign where a point's two contact times merge, which
    // is a fold of the tool's time chart and not of the surface; the product stays finite
    // there, and changes sign only where the generated surface itself folds.
    let fine = crate::util::par::indices(c.patches().len(),|patch| -> Result<Fine,Failure> {
        let mut own = Reads::new();
        let (mut samples,mut near_double_roots) = (0,0);
        let mut hits: Vec<(Hit,usize,usize)> = Vec::new();
        let mut gaps: Vec<f64> = Vec::new();
        let mut least = f64::INFINITY;
        let mut signs: BTreeMap<(usize,usize),([usize;2],[Option<V>;2])> = BTreeMap::new();
        let surface = &c.patches()[patch];
        // a cap is asked only where it reaches the blank (T1)
        if surface.cap().is_some_and(|end| !capped[end]) {
            return Ok(Fine {reads:own,samples,near_double_roots,hits,gaps,least,signs})
        }
        let (cu,cv) = (options.coarse_rows,options.coarse_columns);
        let mut reach = vec![false;cv];
        // the coarse rows on every core, each its reads and which angles reach
        for (row,reads) in crate::util::par::indices(cu+1,|i| {
            let mut own = Reads::new();
            let mut row = vec![false;cv];
            for j in 0..cv {
                let (u,v) = (i as f64/cu as f64,coarse_v(surface,j,cv));
                let Ok(s) = surface.at(u,v) else { continue };
                if !on_tool(s.position) { continue; }
                if let Ok(list) = roots(patch,u,v) {
                    if list.iter().any(|t| own.read(inside,t.contact.position)) { row[j] = true; }
                }
            }
            (row,own)
        }) {
            for (r,x) in reach.iter_mut().zip(row) { *r |= x; }
            own.extend(reads);
        }
        let own = std::cell::RefCell::new(own);
        let read = |p: V| own.borrow_mut().read(inside,p);
        // The shortest arc of the revolution holding every reaching angle, widened a step.
        let reached: Vec<usize> = (0..cv).filter(|&j| reach[j]).collect();
        let band = if reached.is_empty() { None } else if !surface.periodic() {
            // along a prism the reaching stretch runs between its least and greatest, widened
            let (lo,hi) = (reached[0].saturating_sub(2),(reached[reached.len()-1]+2).min(cv-1));
            Some((lo,hi-lo))
        } else {
            let mut gap = (0,0);
            for w in 0..reached.len() {
                let (a,b) = (reached[w],reached[(w+1)%reached.len()]);
                let g = match (b+cv-a)%cv { 0 => cv, g => g };
                if g > gap.1 { gap = (w,g); }
            }
            Some(((reached[(gap.0+1)%reached.len()]+cv-2)%cv,(cv-gap.1+4).min(cv)))
        };
        let (nu,nv) = (options.rows,options.columns);
        // Without a band nothing of this patch reaches the blank in the roll; its clearance
        // and stationarity are still asked over a band of the whole revolution.
        let (start,width) = band.unwrap_or((0,cv));
        let v_at = |j: usize| {
            let at = start as f64+width as f64*j as f64/nv as f64;
            if surface.periodic() { (at/cv as f64).rem_euclid(1.) } else { coarse_at(at,cv) }
        };
        let mut previous: Vec<Option<V>> = vec![None;nv+1];
        // Each sample's contact equation, to find stationary points between samples.
        let mut equations: Vec<Option<([f64;3],V)>> = vec![None;nv+1];
        // What each sample says of itself alone — its point, the rate of its contact condition, its
        // roots with whether each is in the blank, and each such root's area factor — worked out on
        // every core a row at a time; what samples say of their neighbours is asked below, in order.
        let sampled: Vec<Vec<Option<Sampled>>> = crate::util::par::indices(nu+1,|i| {
            let u = i as f64/nu as f64;
            (0..=nv).map(|j| {
                let v = v_at(j);
                let s = surface.at(u,v).ok()?;
                if !on_tool(s.position) { return None; }
                let rate = c.motion().normal_velocity(s).ok();
                let roots = roots(patch,u,v).ok().map(|list| list.into_iter().map(|t| {
                    let within = inside(t.contact.position);
                    let factor = if within { area_factor(surface,patch,&roots,&t,u,v) } else { None };
                    (t,within,factor)
                }).collect());
                Some(Sampled {s,rate,roots})
            }).collect()
        });
        for sampled in sampled {
            let mut row: Vec<Option<V>> = vec![None;nv+1];
            let mut here: Vec<Option<([f64;3],V)>> = vec![None;nv+1];
            for (j,sample) in sampled.into_iter().enumerate() {
                let Some(Sampled {s,rate,roots:list}) = sample else { continue };
                samples += 1;
                // M2: where a point's contact condition does not change over the roll, the
                // point is on the boundary at every time or at none, and its time is no
                // parameter of the generated surface. A single rotation is that everywhere.
                // M2 between samples: the equation is C + a cos + b sin in the roll. Where (a, b)
                // turns right round between neighbours while roots exist on both sides, it passed
                // through zero with C: a point between them is in contact at every time, as on a
                // curve of such points, which no single sample is on.
                if let Some(g) = &rate {
                    let k = g.variation();
                    here[j] = Some((k,s.position));
                    // An isolated such point lies inside a sample cell rather than on a segment
                    // between two samples: there (a, b) winds right round the cell's corners.
                    if j > 0 { if let (Some(p0),Some(p1),Some(p2)) = (equations[j-1],equations[j],here[j-1]) {
                        let corners = [p0,p1,(k,s.position),p2];
                        let reached = corners.iter().all(|(m,_)| m[0].abs() <= m[1].dhypot(m[2]));
                        let mut turn = 0.;
                        for w in 0..4 {
                            let (m,n) = (corners[w].0,corners[(w+1)%4].0);
                            turn += (m[1]*n[2]-m[2]*n[1]).datan2(m[1]*n[1]+m[2]*n[2]);
                        }
                        if reached && turn.abs() > PI {
                            let centre: V = std::array::from_fn(|n| corners.iter().map(|c| c.1[n]).sum::<f64>()/4.);
                            if let Some(p) = times.iter().map(|m| m.point(centre)).find(|&p| read(p)) {
                                return Err((Condition::Stationary,format!("a tool point of `{}` inside a sample cell is in contact \
                                    at every time",surface.name()),Some(p)));
                            }
                        }
                    } }
                    for neighbour in [equations[j],if j > 0 { here[j-1] } else { None }].into_iter().flatten() {
                        let (m,q) = neighbour;
                        let (a1,a2) = (m[1].dhypot(m[2]),k[1].dhypot(k[2]));
                        let turned = m[1]*k[1]+m[2]*k[2] < -0.5*a1*a2;
                        if turned && m[0].abs() <= a1 && k[0].abs() <= a2 {
                            let between: V = std::array::from_fn(|n| 0.5*(q[n]+s.position[n]));
                            if let Some(p) = times.iter().map(|m| m.point(between)).find(|&p| read(p)) {
                                return Err((Condition::Stationary,format!("tool points between samples of `{}` are in contact at every time",
                                    surface.name()),Some(p)));
                            }
                        }
                    }
                }
                // (one that holds the same nonzero value is in contact at no time, and bounds nothing:
                // a prism's side square to a rack's line of centres, whose cap cuts, #684 F)
                if let Some(g) = &rate {
                    let scale = 1.+norm(s.position);
                    let at = [domain[0],middle,domain[1]].map(|t| g.at(t).unwrap_or(f64::NAN));
                    if at.iter().all(|x| (x-at[1]).abs() <= options.root_tolerance*scale) && at[1].abs() <= options.root_tolerance*scale {
                        if let Some(p) = times.iter().map(|m| m.point(s.position)).find(|&p| read(p)) {
                            return Err((Condition::Stationary,
                                "a tool point's contact condition does not change over the roll: it is in contact at every time".into(),
                                Some(p)));
                        }
                    }
                }
                let Some(list) = list else { near_double_roots += 1; continue };
                // every root read, as a pass over them reads them
                for (t,within,_) in &list { own.borrow_mut().push(t.contact.position,*within); }
                let found: Vec<&(TimedContact,bool,Option<f64>)> = list.iter().filter(|x| x.1).collect();
                if found.len() > 1 {
                    return Err((Condition::Single,format!("a tool point touches the blank at roll {:.4} and again at {:.4}",
                        found[0].0.root.time,found[1].0.root.time),Some(found[1].0.contact.position)));
                }
                for &(t,_,factor) in found {
                    let hit = Hit {source:s.position,position:t.contact.position,normal:t.contact.normal};
                    row[j] = Some(hit.position);
                    for q in [previous[j],if j > 0 { row[j-1] } else { None }].into_iter().flatten() {
                        gaps.push(norm(sub(q,hit.position)));
                    }
                    hits.push((hit,patch,t.root.branch));
                    // E3: the generated surface's oriented area factor on the same branch
                    // (`area_factor`), against the tool surface's own.
                    let Some(factor) = factor else { continue };
                    if factor.abs() < options.least_factor {
                        return Err((Condition::Fold,format!("the generated surface's area factor is {factor:.3e}"),Some(t.contact.position)));
                    }
                    least = least.min(factor.abs());
                    let e = 1e-6;
                    let Some(g) = &rate else { continue };
                    let (Ok(g0),Ok(g1)) = (g.at(t.root.time-e),g.at(t.root.time+e)) else { continue };
                    let side = usize::from(factor*(g1-g0) < 0.);
                    let entry = signs.entry((patch,t.root.branch)).or_insert(([0,0],[None,None]));
                    entry.0[side] += 1;
                    if entry.1[side].is_none() { entry.1[side] = Some(t.contact.position); }
                }
            }
            previous = row;
            equations = here;
        }
        Ok(Fine {reads:own.into_inner(),samples,near_double_roots,hits,gaps,least,signs})
    });
    let (mut samples,mut near_double_roots) = (0,0);
    let mut hits: Vec<(Hit,usize,usize)> = Vec::new();
    let mut gaps: Vec<f64> = Vec::new();
    let mut least = f64::INFINITY;
    let mut signs: BTreeMap<(usize,usize),([usize;2],[Option<V>;2])> = BTreeMap::new();
    for found in fine {
        let found = found?;
        reads.extend(found.reads);
        samples += found.samples; near_double_roots += found.near_double_roots;
        hits.extend(found.hits); gaps.extend(found.gaps);
        least = least.min(found.least);
        signs.extend(found.signs);
    }
    for ((patch,_),(counts,at)) in &signs {
        if counts[0] > 0 && counts[1] > 0 {
            let minority = usize::from(counts[1] < counts[0]);
            return Err((Condition::Fold,format!("the generated surface of `{}` turns back on itself ({} samples one way, {} the other)",
                c.patches()[*patch].name(),counts[0],counts[1]),at[minority]));
        }
    }
    if hits.is_empty() {
        return Err((Condition::Reach,"no contact of the tool within the roll lies in the blank".into(),None));
    }
    // where two faces of the tool meet at a convex edge, the edge carries every normal between theirs,
    // a fan: a cap's rim, where a side meets it (#684 F), and a drafted prism's crease at its parting
    // plane, where its sides' halves meet (#684 P). Where a face's contact reaches the edge in the
    // blank, the generated surface carries on over the fan: E3 asks that it carry on, not turn back —
    // a face's image stepped toward the edge and the fan's stepped off it point the same way (a flat
    // end swung so its corners dig past it fails here, its envelope under the corners' paths); a fan
    // whose normals all touch at the face's instant, as a crease's under a motion keeping the
    // profile's plane, carries the contact nowhere, its step off the roots' rounding and no way at
    // all. E2 asks that each fan normal touch the blank once, and the fans' contacts are asked E4
    // with the faces' (an edge point's own fan is one tool point, never distant from itself)
    let mut fans: Vec<Fan> = Vec::new();
    if capped.contains(&true) { for (patch,surface) in c.patches().iter().enumerate() {
        if surface.periodic() || surface.cap().is_some() { continue }
        // a reaching cap's rim, its sides' end at `v`: the cap's normal along the side, and the side's
        for v in [0.,1.].into_iter().filter(|&v| capped[v as usize]) { for i in 0..=options.rows {
            let Ok(s) = surface.at(i as f64/options.rows as f64,v) else { continue };
            if !on_tool(s.position) { continue }
            let side = crate::sweep::characteristic::outward(tool,s.position,crate::util::space::unit(cross(s.du,s.dv)));
            let along = scale(crate::util::space::unit(s.dv),if v > 0.5 { 1. } else { -1. });
            fans.push(Fan {point:s.position,faces:[(along,side.map(|x| -x)),(side,along.map(|x| -x))],patch,crease:false});
        } }
    } }
    // each crease, the half toward the prism's `from` and the half toward its `to`
    for &[below,above] in c.creases() { for i in 0..=options.rows {
        let u = i as f64/options.rows as f64;
        let (Ok(a),Ok(b)) = (c.patches()[below].at(u,1.),c.patches()[above].at(u,0.)) else { continue };
        if !on_tool(a.position) { continue }
        let normal = |s: &SurfacePoint| crate::sweep::characteristic::outward(tool,a.position,crate::util::space::unit(cross(s.du,s.dv)));
        fans.push(Fan {point:a.position,faces:[(normal(&a),scale(crate::util::space::unit(a.dv),-1.)),(normal(&b),crate::util::space::unit(b.dv))],
            patch:below,crease:true});
    } }
    let read = crate::util::par::map(&fans,|&Fan {point:p,faces,patch,crease}| -> Result<(Vec<(Hit,usize,usize)>,Reads),Failure> {
        let (mut found,mut own) = (Vec::new(),Reads::new());
        let name = c.patches()[patch].name();
        let (edge,whose) = if crease { (format!("where `{name}` folds at its prism's parting plane"),["its `from` half's","its `to` half's"]) }
            else { (format!("at the rim where `{name}` meets its prism's cap"),["the cap's","the side's"]) };
        let h = 1e-5*(1.+norm(p));
        let near = |q: V,n: V,t: f64| c.at_point_normal_over(q,n,domain,options.root_tolerance).ok()?.into_iter()
            .min_by(|a,b| (a.root.time-t).abs().total_cmp(&(b.root.time-t).abs())).map(|x| x.contact.position);
        // each face meeting the edge, its normal, the way into it, and the fan's other end
        for k in 0..2 {
            let ((face,into),other) = (faces[k],faces[1-k].0);
            let Ok(list) = c.at_point_normal_over(p,face,domain,options.root_tolerance) else { continue };
            for t in list.iter().filter(|t| own.read(inside,t.contact.position)) {
                let x0 = t.contact.position;
                let stepped: V = std::array::from_fn(|k| p[k]+h*into[k]);
                let turned: V = std::array::from_fn(|k| face[k]+1e-4*other[k]);
                let (Some(xf),Some(xn)) = (near(stepped,face,t.root.time),near(p,turned,t.root.time)) else { continue };
                let (onto,off) = (sub(x0,xf),sub(xn,x0));
                if dot(onto,off) < -1e-3*norm(onto)*norm(off) && norm(off) > 1e-3*norm(onto) {
                    return Err((Condition::Fold,format!("the generated surface turns back {edge}: the {}'s fan sweeps back over {} \
                        envelope",if crease { "crease" } else { "rim" },whose[k]),Some(x0)))
                }
            }
        }
        // the fan's normals as a station reads them (`cutter::Cutter::sample`)
        for f in 1..FAN_READS {
            let f = f as f64/FAN_READS as f64;
            let n = crate::util::space::unit(add(scale(faces[0].0,1.-f),scale(faces[1].0,f)));
            let Ok(list) = c.at_point_normal_over(p,n,domain,options.root_tolerance) else { continue };
            let within: Vec<&TimedContact> = list.iter().filter(|t| own.read(inside,t.contact.position)).collect();
            if within.len() > 1 {
                return Err((Condition::Single,format!("a point of the tool {edge} touches the blank at roll {:.4} and again at {:.4}",
                    within[0].root.time,within[1].root.time),Some(within[1].contact.position)))
            }
            found.extend(within.into_iter().map(|t| (Hit {source:p,position:t.contact.position,normal:t.contact.normal},patch,usize::MAX)));
        }
        Ok((found,own))
    });
    for found in read { let (found,own) = found?; hits.extend(found); reads.extend(own); }
    // E4: contacts closer than the sample spacing whose tool points lie far apart.
    gaps.sort_by(f64::total_cmp);
    let spacing = gaps.get(gaps.len()*95/100).copied().unwrap_or(0.);
    let mut near_tangent_pairs = 0;
    if spacing > 0. {
        let mut grid: BTreeMap<[i64;3],Vec<usize>> = BTreeMap::new();
        for (k,(h,_,_)) in hits.iter().enumerate() { grid.entry(h.position.map(|x| (x/spacing).floor() as i64)).or_default().push(k); }
        for (k,(a,_,_)) in hits.iter().enumerate() {
            let key = a.position.map(|x| (x/spacing).floor() as i64);
            for dx in -1..=1 { for dy in -1..=1 { for dz in -1..=1 {
                let Some(list) = grid.get(&[key[0]+dx,key[1]+dy,key[2]+dz]) else { continue };
                for &m in list {
                    if m <= k { continue; }
                    let b = &hits[m].0;
                    if norm(sub(a.position,b.position)) > 0.5*spacing || norm(sub(a.source,b.source)) < 5.*spacing { continue; }
                    if dot(a.normal,b.normal).abs() > 0.999 { near_tangent_pairs += 1; continue; }
                    return Err((Condition::Crossing,"two distant parts of the tool's sweep meet here".into(),Some(a.position)));
                }
            }}}
        }
    }
    // T3: a cap in the blank, sectioned across its profile clear of its corners and its ends
    let class = match lone.filter(|_| capped.contains(&true)) {
        Some(lone) => Found::Capped(Box::new(lone.sections(&hits)?)),
        None => Found::Generating,
    };
    Ok((Evidence {samples,contacts:hits.len(),spacing,
        least_area_factor:least,near_double_roots,near_tangent_pairs,
        basis:Basis::Sampled {rows:options.rows,columns:options.columns},class},reads))
}

/// T2 and E1, then S1–S4 (`constant_twist`), for one placement of a sweep under a screw, a turn or
/// a slide, `inside` being the blank read in the sweep's frame, `heights` how far along the
/// motion's axis it reaches there. Their twist is constant, so M2 does not apply: a tool point is
/// on the boundary at every time or at none, and the boundary is the characteristic carried along.
pub fn check_screw(c: &SweepContacts,inside: &(dyn Fn(V) -> bool+Sync),extent: Option<([f64;2],f64)>,options: &Options)
    -> Result<(Evidence,Reads),Failure> {
    let (reads,..) = ends(c,inside,options,None)?;
    let reads = std::sync::Mutex::new(reads);
    // the blank read outside the lock, the faces' rings side by side
    let read = |p: V| { let i = inside(p); reads.lock().unwrap_or_else(|e| e.into_inner()).push(p,i); i };
    let ask = crate::sweep::characteristic::Ask {rows:options.rows,root_tolerance:options.root_tolerance,least_factor:options.least_factor,
        roll:c.domain(),heights:extent.map(|e| e.0),radius:extent.map(|e| e.1),inside:&read};
    let found = crate::sweep::characteristic::Characteristic::walk(c,&ask)?;
    let [first,last] = found.reach;
    let mut gaps: Vec<f64> = (first..last).map(|w| crate::util::space::distance(found.nodes[w].position,found.nodes[w+1].position)).collect();
    gaps.sort_by(f64::total_cmp);
    let evidence = Evidence {samples:found.samples,contacts:last-first+1,
        spacing:gaps.get(gaps.len()/2).copied().unwrap_or(0.),least_area_factor:found.least_factor,near_double_roots:0,
        near_tangent_pairs:0,basis:Basis::Sampled {rows:options.rows,columns:360},
        class:Found::ConstantTwist(Box::new(found))};
    Ok((evidence,reads.into_inner().unwrap_or_else(|e| e.into_inner())))
}


/// What one placement's rows saw: how many tool points were sampled and their contacts in the
/// blank, how far apart those were, the least area factor, the samples at a double root and the
/// near-tangent pairs, how the evidence was got, and the class the sweep is in with what it found.
#[derive(Clone,Debug)]
pub struct Evidence {
    pub samples: usize,
    pub contacts: usize,
    /// Most neighbouring contacts are this close (the 95th percentile of their gaps).
    pub spacing: f64,
    pub least_area_factor: f64,
    /// Samples whose two contact times nearly coincide: the edge of the envelope, where the
    /// contact equation has a double root and is not solved there.
    pub near_double_roots: usize,
    /// Pairs of contacts from distant tool points lying on one another with the same normal:
    /// sheets touching tangentially, which is where a crossing begins.
    pub near_tangent_pairs: usize,
    pub basis: Basis,
    pub class: Found,
}

/// The class a placement's rows admit it to: a relative rotation's (the generating class), with a
/// prism's cap sectioned across where one reaches the blank (T3), or a screw's (constant twist),
/// with the stretch of its characteristic that reaches the blank.
#[derive(Clone,Debug)]
pub enum Found { Generating,ConstantTwist(Box<Characteristic>),Capped(Box<Capped>) }

/// One placement of a sweep asked its class's rows: a screw's, a turn's or a slide's (`check_screw`,
/// `extent` how far along the screw's axis the blank reaches in the sweep's frame, and how far from
/// it) where the motion has a constant twist, else a relative rotation's (`check`, `lone` the tool
/// where it is a prism standing alone). `inside` is the blank read in the sweep's frame.
pub fn admit(c: &SweepContacts,inside: &(dyn Fn(V) -> bool+Sync),extent: Option<([f64;2],f64)>,options: &Options,
    lone: Option<&Lone>) -> Result<(Evidence,Reads),Failure> {
    match c.motion().twist() {
        Some(_) => check_screw(c,inside,extent,options),
        None => check(c,inside,options,lone),
    }
}
