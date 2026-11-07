//! **The generating class's rows, asked of one placement** (#721 F2c): M2, E1–E4 and T1–T3 for a
//! tool carried by a relative rotation (`check`), and T2, E1 and the constant twist's S1–S4 for one
//! carried by a screw, a turn or a slide (`check_screw`), each against the blank as `inside` reads
//! it in the sweep's frame. The tool's faces are sampled (`Sampling`), so the rows are sampled
//! too. A row that fails says so by `constant_twist::Row`, with the words and a point; the reads
//! made of the blank are returned (`Reads`), so the caller can tell whether another placement
//! reads it alike.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use crate::contact::SurfacePoint;
use crate::geom::Rigid;
use crate::trajectory::{Motion,Trajectory};
use crate::util::space::{add,cross,dot,norm,scale,sub};
use super::constant_twist::{Ask,Characteristic,Failure,Row,Sweep,outward};
use super::cutter::Cutter;
use super::generating::{Chart,Form,NormalVelocity,PointContactError,TimedContact};
use super::tool::{Material,PatchEdge,Tool,ToolSurface};
use crate::topo::Brep;
use std::collections::BTreeMap;
use std::f64::consts::{PI,TAU};

type V = [f64;3];

/// What a generating sweep's rows read: the tool as the kernel reads it, the trajectory it is
/// carried along, the roll, whether that roll carries a turn round a whole turn or more, read
/// once round (#684 N), and the model's millimetres a unit, what a row says a length in.
#[derive(Clone,Copy)]
pub struct Generating<'a> { pub tool: &'a Tool,pub path: &'a Trajectory,pub roll: [f64;2],pub whole: bool,pub mm: f64 }

impl Generating<'_> {
    fn patches(&self) -> &[ToolSurface] { &self.tool.faces }
    fn edges(&self) -> &[PatchEdge] { &self.tool.edges }
    fn creases(&self) -> &[[usize;2]] { &self.tool.creases }
    fn corner(&self,edge: &PatchEdge) -> Option<(V,[V;2])> { self.tool.corner(edge) }
    fn motion(&self) -> &Trajectory { self.path }
    fn source_material(&self) -> &dyn Material { self.tool.material() }
    fn domain(&self) -> [f64;2] { self.roll }
    fn whole(&self) -> bool { self.whole }
    /// The contact chart of a surface's point, where the trajectory has one.
    fn normal_velocity(&self,s: SurfacePoint) -> Option<NormalVelocity> {
        super::generating::Form::of(self.path).ok()?.normal_velocity(s).ok()
    }
    /// The contact times of face `patch`'s point at `(u, v)` over `interval`.
    fn at_source_over(&self,patch: usize,u: f64,v: f64,interval: [f64;2],tolerance: f64)
        -> Result<Vec<TimedContact>,PointContactError> {
        let surface = self.tool.faces.get(patch).ok_or_else(|| PointContactError::Failed("no such sweep patch".into()))?.at(u,v)
            .map_err(|e| PointContactError::Failed(format!("{e:?}")))?;
        self.chart()?.at_surface(surface,interval,tolerance)
    }
    /// The contact times of a point carrying the unit normal `normal` over `interval`.
    fn at_point_normal_over(&self,position: V,normal: V,interval: [f64;2],tolerance: f64)
        -> Result<Vec<TimedContact>,PointContactError> {
        self.chart()?.at_point_normal(position,normal,interval,tolerance)
    }
    fn chart(&self) -> Result<Chart<'_>,PointContactError> { Chart::new(self.path,self.roll).map_err(|u| PointContactError::Failed(u.to_string())) }
}

/// The trajectory's pose at the roll `t`, or why not, in the words a motion is read in.
fn pose_at(path: &Trajectory,t: f64) -> Result<Motion,String> {
    if !t.is_finite() { return Err("a motion angle must be finite".into()) }
    path.at(t).map_err(|e| e.msg)
}

/// How finely the rows sample the tool: the fine pass's rows along a face and columns round it, the
/// coarse pass's, the contact equation's root tolerance, and the least generated area factor E3
/// allows.
#[derive(Clone,Copy,Debug)]
pub struct Sampling { pub rows: usize,pub columns: usize,pub coarse_rows: usize,pub coarse_columns: usize,
    pub root_tolerance: f64,pub least_factor: f64 }

impl Default for Sampling {
    /// A revolved face sampled 100 rows along its profile by 400 round the band reaching the blank,
    /// the band found by a 24 by 1440 pass; roots to 1e-9; a generated area a thousandth of the
    /// tool's a fold.
    fn default() -> Sampling {
        Sampling {rows:100,columns:400,coarse_rows:24,coarse_columns:1440,root_tolerance:1e-9,least_factor:1e-3}
    }
}

/// What the generating class's rows found of one placement: the tool points sampled and their
/// contacts in the blank, how close neighbouring contacts are (the 95th percentile of their gaps),
/// the least area factor, the samples whose two contact times nearly coincide, the pairs of
/// contacts of distant tool points touching tangentially, and where a cap reaches the blank, how it
/// is sectioned across (T3).
#[derive(Clone,Debug)]
pub struct Found { pub samples: usize,pub contacts: usize,pub spacing: f64,pub least_area_factor: f64,
    pub near_double_roots: usize,pub near_tangent_pairs: usize,pub capped: Option<Capped> }

/// A generating sweep of a prism whose cap reaches the blank (#684 F, row T3), sectioned across:
/// the direction in the profile's plane a station is a distance along (the tool's frame, unit),
/// the stations the sampled contacts in the blank reach, and how far that band keeps from the
/// profile's corners and its ends (millimetres from the profile's origin).
#[derive(Clone,Debug)]
pub struct Capped { pub across: V,pub band: [f64;2],pub clearance: f64 }

/// Which tool points the checks read the blank at, and whether each was inside: two placements
/// agreeing on all of them would compute the same checks to the last number.
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
struct Sampled { s: SurfacePoint,rate: Option<NormalVelocity>,roots: Option<Vec<(TimedContact,bool,Option<f64>)>> }

/// E3's area factor at a contact `t` of the tool point at (u, v) of face `patch`: the generated
/// surface's oriented area factor on the same branch, by central differences, against the tool
/// surface's own. None where a neighbouring root or point cannot be read, or the tool's own area
/// vanishes.
fn area_factor(surface: &ToolSurface,patch: usize,
    roots: &(dyn Fn(usize,f64,f64) -> Result<Vec<TimedContact>,PointContactError>+Sync),t: &TimedContact,u: f64,v: f64) -> Option<f64> {
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

/// How many steps across a cap no rim bounds `ends` reads it at, each way.
const CAP_READS: usize = 16;

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


/// **A rack's straight flank undercutting the gear it cuts** (#822): E3's fold said for what it
/// is, where it is that. Under a slide seen from a turn, the slide square to the turn's axis (a
/// rack against the blank it cuts), the rack's pitch line rolls on the blank's pitch circle, its
/// radius R the slide's advance a radian over the turn's rate, and every section square to the
/// axis is a plane generation by a rack — a helical rack's too, leaned across the section. A flat
/// flank's trace in a section is a line at the transverse pressure angle αₜ (its normal's slant
/// from the slide, its part along the axis dropped). It generates the involute of the base circle
/// R cos αₜ down to where the line of action touches that circle, R sin²αₜ inside the pitch line;
/// a point of the flank deeper than that generates the involute's folded branch, the undercut.
/// Lengths are millimetres; `along` is whether the flank faces the way the rack slides as the roll
/// runs on.
#[derive(Clone,Debug,PartialEq)]
pub struct Undercut { pub flank: String,pub along: bool,pub pitch: f64,pub base: f64,pub angle: f64,pub reach: f64,pub depth: f64 }

impl std::fmt::Display for Undercut {
    fn fmt(&self,f: &mut std::fmt::Formatter) -> std::fmt::Result {
        let Undercut {flank,along,pitch,base,angle,reach,depth} = self;
        let (way,degrees,shift,more) = (if *along { "along" } else { "back along" },angle.to_degrees(),reach-depth,reach/depth);
        write!(f,"the flank `{flank}`, the rack tooth's side facing {way} its slide, undercuts the gear below its base circle \
            (r = {base:.3} mm): it reaches {reach:.3} mm past the rack's pitch line, where {depth:.3} mm is the most a straight \
            flank at this pressure angle ({degrees:.2}° across the gear's axis) takes on this pitch circle (r = {pitch:.3} mm) \
            without undercut. Shift the rack out {shift:.3} mm or more, take {more:.3} times as many teeth or more at this pitch \
            (a pitch circle of r = {:.3} mm), or end the straight flank within {depth:.3} mm of the pitch line. Its generated \
            flank folds",pitch*more)
    }
}

/// E3's fold of face `patch`, read as a rack's straight flank undercutting the gear (`Undercut`),
/// or none where it is not one: the face not flat, the motion no rack's roll, or the flank's
/// contacts in the blank (`inside`) reaching no deeper than the involute's end. How deep it
/// reaches is read off the face sampled as the fine pass samples it, its rows by the coarse
/// pass's columns.
fn undercut(c: &Generating,patch: usize,inside: &(dyn Fn(V) -> bool+Sync),options: &Sampling) -> Option<Undercut> {
    let ToolSurface::Extruded(flank) = c.patches().get(patch)? else { return None };
    if !flank.straight() { return None }
    let Ok(Form::Rack {slide,observer}) = Form::of(c.motion()) else { return None };
    let (d,s,w) = (slide.axis,slide.advance/TAU,scale(observer.axis,observer.ratio));
    let rate = norm(w);
    if !(rate > 0.) || dot(d,w).abs() > 1e-9*rate*norm(d) { return None }
    let axis = scale(w,1./rate);
    // from the axis to the pitch point, where the rack's point keeps still in the blank at roll 0
    let out = scale(cross(d,w),s/(rate*rate));
    let pitch = norm(out);
    if !(pitch > 0.) || !pitch.is_finite() { return None }
    let e = scale(out,1./pitch);
    let at = flank.at(0.5,0.5).ok()?;
    let normal = cross(at.du,at.dv);
    if !(norm(normal) > 0.) { return None }
    let n = outward(c.source_material(),at.position,crate::util::space::unit(normal));
    let across = sub(n,scale(axis,dot(n,axis)));
    if norm(across) < 1e-9 { return None }
    let cos = (dot(across,d)/(norm(across)*norm(d))).clamp(-1.,1.);
    let depth = pitch*(1.-cos*cos);
    let tool = c.source_material();
    let below = |p: V| pitch-dot(sub(p,observer.origin),e);
    let (nu,nv) = (options.rows,options.coarse_columns.max(2));
    let mut reach = f64::NEG_INFINITY;
    for i in 0..=nu { for j in 0..=nv {
        let (u,v) = (i as f64/nu as f64,j as f64/nv as f64);
        let Ok(q) = flank.at(u,v) else { continue };
        if tool.value(q.position).abs() >= 1e-6 || below(q.position) <= reach { continue }
        let Ok(list) = c.at_source_over(patch,u,v,c.domain(),options.root_tolerance) else { continue };
        if list.iter().any(|t| inside(t.contact.position)) { reach = below(q.position) }
    } }
    if !(reach-depth > 1e-6*(1.+pitch)) { return None }
    let mm = c.mm;
    Some(Undercut {flank:flank.name.clone(),along:dot(n,d)*s > 0.,pitch:pitch*mm,base:pitch*cos.abs()*mm,angle:cos.abs().acos(),
        reach:reach*mm,depth:depth*mm})
}

/// T2, E1 and T1 for a prism's caps for one placement, asked of every class before anything that
/// might be reported in their place: the blank's reads, the poses the roll is sampled at, and
/// whether each cap of `lone` (the tool, where it is a prism standing alone; its `start`, then its
/// `end`) reaches the blank. A whole turn has no ends to clear: the tool at either lies within
/// what the turn sweeps (`SweepContacts::whole`, #684 N). A plane is asked only where the motion's
/// contact lies on it at some pose of the roll sampled (#721 F1d).
fn ends(c: &Generating,inside: &(dyn Fn(V) -> bool+Sync),options: &Sampling,lone: Option<&Lone>,concave: bool)
    -> Result<(Reads,Vec<Motion>,[bool;2]),Failure> {
    let tool = c.source_material();
    let on_tool = |p: V| c.tool.on(p);
    let domain = c.domain();
    let pose = |t: f64| pose_at(c.motion(),t).map_err(|m| (Row::Motion,m,None));
    let limits = if c.whole() { Vec::new() } else { vec![pose(domain[0])?,pose(domain[1])?] };
    let mut reads = Reads::new();
    // T2: a concave corner carried through the blank trims two envelopes against each other.
    // Asked first, since the crossing it makes would otherwise be reported in its place.
    let steps = 64;
    let poses: Vec<Motion> = (0..=steps).map(|k| pose(domain[0]+(domain[1]-domain[0])*k as f64/steps as f64))
        .collect::<Result<_,_>>()?;
    // where tool point `x` is first carried into the blank over the roll, each read remembered
    let reaches = |reads: &mut Reads,x: V| poses.iter().map(|m| m.point(x)).find(|&p| reads.read(inside,p));
    for edge in c.edges().iter().filter(|_| concave) {
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
            // (where the corner is the tool's: a face read whole a Boolean trims, #721 F1b)
            if !on_tool(point.position) { continue }
            if let Some(p) = reaches(&mut reads,point.position) {
                return Err((Row::Corner,format!("`{}` meets `{}` at a concave corner",pa.name(),pb.name()),Some(p)));
            }
        }
    }
    // and a crease where a drafted prism's halves fold (#684 P), concave where it is drafted outward
    // both ways from its parting plane: the chord between a point of each half beside it out of the
    // tool's material
    for &[a,b] in c.creases() {
        let (Some(pa),Some(pb)) = (c.patches().get(a),c.patches().get(b)) else { continue };
        let h = 1e-4;
        for i in 1..options.coarse_rows {
            let u = i as f64/options.coarse_rows as f64;
            let (Ok(point),Ok(sa),Ok(sb)) = (pa.at(u,1.),pa.at(u,1.-h),pb.at(u,h)) else { continue };
            if !on_tool(point.position) { continue }
            let chord: V = std::array::from_fn(|k| 0.5*(sa.position[k]+sb.position[k]));
            if tool.value(chord) <= 0. { continue }
            if let Some(p) = reaches(&mut reads,point.position) {
                let said = if pa.name() == pb.name() { format!("the crease across `{}` is concave",pa.name()) }
                    else { format!("`{}` meets `{}` at a concave crease",pa.name(),pb.name()) };
                return Err((Row::Corner,said,Some(p)));
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
                    if own.read(inside,p) { return Err((Row::Clearance,
                        "the tool at a limit of its roll lies in the blank".into(),Some(p))); }
                }
            }}
            Ok(own)
        }) { reads.extend(own?); }
        Ok(reads)
    };
    reads.extend(clear(&|s| s.cap().is_none())?);
    // T1 for a prism: a cap passing through the blank is cut only by a prism standing alone as the
    // tool or placed, its profile one loop, sectioned across (T3, #684 F, AC) or, under a constant
    // twist, its characteristic walked over the cap (#684 AE); otherwise the class cuts with a
    // prism's sides only (under a planar sweep a cap, square to the turn, is in contact at every
    // time or none, and M2 refuses it). Each cap is asked by its rim, its sides'
    // ends at `v` (a drafted side's end at its parting plane is no rim, but the crease its halves
    // meet in, #684 P).
    let mut capped = [false;2];
    // (a side's rim that lies in a cap's plane, or a lone prism's flat's — a cap a round hole passes
    // through, read as flats of its wall; a rim on any other face is no cap's, and the walk reads
    // the contact there or refuses it by its own row, #721 F1d)
    let caps: Vec<(V,V)> = c.patches().iter().filter_map(|s| match s {
        ToolSurface::Revolved(r) => r.flat().filter(|_| lone.is_some()).map(|(o,a,..)| (o,a)),
        _ => s.cap().and_then(|_| s.at(0.,0.).ok()).map(|p| (p.position,cross(p.du,p.dv))),
    }).collect();
    for (end,v) in [(0,0.),(1,1.)] {
        let capped_rim = |s: &ToolSurface| caps.iter().any(|&(o,n)| (0..=4).all(|i| s.at(i as f64/4.,v)
            .is_ok_and(|p| dot(sub(p.position,o),n).abs() <= 1e-7*norm(n)*(1.+norm(p.position)))));
        let rim = c.patches().iter().enumerate()
            .filter(|&(k,s)| !s.periodic() && s.cap().is_none() && !c.creases().iter().any(|crease| crease[1-end] == k) && capped_rim(s))
            .find_map(|(_,surface)| (0..=options.coarse_rows).find_map(|i| {
                reaches(&mut reads,surface.at(i as f64/options.coarse_rows as f64,v).ok()?.position) }).map(|p| (surface.name(),p)));
        let Some((name,p)) = rim else { continue };
        if lone.is_none() {
            return Err((Row::Tool,format!("a cap of the prism `{name}` bounds passes through the blank, and the class cuts \
                with a cap only of a prism standing alone as the tool or placed, undrafted, its profile one loop: make the prism \
                longer than the blank is thick"),Some(p)));
        }
        capped[end] = true;
    }
    // and any other plane of the tool's boundary that is no ruled side — a partial revolution's
    // end, a plane a Boolean of the tool cuts it by — read as a cap (`Tool::read`, #721 F1b):
    // asked by its own points on the tool, since no rim bounds it
    let mut held_none: Vec<usize> = Vec::new();
    for (k,surface) in c.patches().iter().enumerate() {
        let Some(end) = surface.cap() else { continue };
        if capped[end] { continue }
        let reached = on_tool_grid(surface,&on_tool).find_map(|s| reaches(&mut reads,s.position));
        if let Some(p) = reached {
            // (a lone prism's cap its rim does not carry into the blank is cut as one its rim does)
            if lone.is_some() { capped[end] = true; continue }
            // a plane on whose points on the tool the contact keeps one sign at every pose sampled
            // holds none: nothing of it is the sweep's boundary but at the end poses (#721 F1d)
            if holds_none(surface,&on_tool,&poses) { held_none.push(k); continue }
            return Err((Row::Tool,format!("its plane face `{}` passes through the blank, and the class cuts with a plane \
                face only as a prism's side, or as a cap of a prism standing alone as the tool or placed, undrafted, its \
                profile one loop",surface.name()),Some(p)));
        }
    }
    // (E1 of the caps the class cuts, and of the planes that hold no contact, which the tool at the
    // roll's limits may still lay in the blank)
    let asked: Vec<*const ToolSurface> = held_none.iter().map(|&k| &c.patches()[k] as *const ToolSurface).collect();
    reads.extend(clear(&|s| s.cap().is_some_and(|end| capped[end]) || asked.contains(&(s as *const ToolSurface)))?);
    Ok((reads,poses,capped))
}

/// `surface`'s points on the tool (`on_tool`) on a `CAP_READS` grid of its chart.
fn on_tool_grid<'a>(surface: &'a ToolSurface,on_tool: &'a dyn Fn(V) -> bool) -> impl Iterator<Item=crate::contact::SurfacePoint>+'a {
    let n = CAP_READS;
    (0..=n).flat_map(move |i| (0..=n).map(move |j| (i,j)))
        .filter_map(move |(i,j)| surface.at(i as f64/n as f64,j as f64/n as f64).ok().filter(|s| on_tool(s.position)))
}

/// Whether `surface` holds no contact over the roll: `n·v` one sign at each of its points on the
/// tool (`on_tool_grid`) at every pose of `poses`, so no class's contact meets it (#721 F1d).
fn holds_none(surface: &ToolSurface,on_tool: &dyn Fn(V) -> bool,poses: &[Motion]) -> bool {
    let [lo,hi] = on_tool_grid(surface,on_tool).flat_map(|s| poses.iter().filter_map(move |&m| crate::contact::contact(s,m).ok()
        .map(|c| c.normal_velocity))).fold([f64::INFINITY,f64::NEG_INFINITY],|[lo,hi],x| [lo.min(x),hi.max(x)]);
    lo > 0. || hi < 0.
}

/// A tool that is one prism standing alone or placed, drafted or not, its profile any loops: what
/// the class sections across where its cap reaches the blank (T3, #684 F, AC), and what every sheet
/// of the tool's sides is sectioned by (#721 F1d) — its cutter (millimetres, carried by the
/// placements) and the model's millimetres a unit.
pub struct Lone { cutter: Cutter,scale: f64 }

impl std::fmt::Debug for Lone {
    fn fmt(&self,f: &mut std::fmt::Formatter) -> std::fmt::Result { write!(f,"Lone {{ axis: {:?},scale: {} }}",self.cutter.axis,self.scale) }
}

impl Lone {
    /// The tool's cutter (millimetres, carried by the placements) as a lone prism, `scale` the
    /// model's millimetres a unit, or none where it is no prism.
    fn new(cutter: Cutter,scale: f64) -> Option<Lone> {
        cutter.prism_walks()?;
        Some(Lone {cutter,scale})
    }

    /// Whether its sections across its profile are read where its cap reaches the blank (T3): an
    /// undrafted prism, or one drafted whose sides are planes, of any loops (a hole parts a section
    /// into chords, a draft creases it at the parting plane, #721 F1d).
    pub fn sectioned_across(&self) -> bool { self.cutter.spread(crate::util::space::across(self.cutter.axis).0).is_some() }

    /// **The tool `tool` read from its boundary `b` (millimetres) as one prism standing alone**
    /// (#721 F1b, F1d), `mm` the model's millimetres a unit: where every face it reads is a side
    /// running between two caps, the boundary's planes square to one direction, each side's
    /// rulings along that direction or leaning to it alike — one draft, all one way, or out to a
    /// parting plane and in again past it, where the sides' halves meet in creases (#684 G, P) —
    /// and each cap's loops of lines and arcs. Its cutter is that direction's prism of its widest
    /// section — the nearer cap's loops undrafted, the wider cap's drafted one way, the parting
    /// plane's drafted both ways — however the tool was drawn: a prism, one placed, one drafted, a
    /// box a Boolean leaves. None for any other tool.
    pub fn read(b: &Brep,tool: &Tool,mm: f64) -> Option<Lone> {
        use crate::build::Profile;
        use crate::geom::Surface;
        use crate::util::space::unit;
        // the caps' normal is the prism's direction: its two planes, each read as a cap or as the
        // flats of the revolutions it shares circles with (a cap a round hole passes through),
        // each by its outward normal and a point of it
        let flat = |f: &ToolSurface| matches!(f,ToolSurface::Revolved(s) if s.flat().is_some());
        let mut caps: Vec<(V,V)> = Vec::new();
        for f in tool.faces.iter().filter(|f| matches!(f,ToolSurface::Cap(_)) || flat(f)) {
            let Ok(p) = f.at(0.5,0.5) else { return None };
            let n = cross(p.du,p.dv);
            if !(norm(n) > 0.) { return None }
            let n = scale(n,1./norm(n));
            if !caps.iter().any(|&(m,q)| norm(sub(m,n)) <= 1e-9 && dot(sub(p.position,q),n).abs() <= 1e-9*(1.+norm(q))) {
                caps.push((n,p.position))
            }
        }
        let [(c0,_),(c1,_)] = caps[..] else { return None };
        if !(norm(c0) > 0.) || norm(cross(c0,c1)) > 1e-9*norm(c0)*norm(c1) || dot(c0,c1) >= 0. { return None }
        // (pointing the way the sides' rulings run from their edge, as they were read: the cutter
        // laid from the cap they leave, so its stations are measured from it)
        let ruled = tool.faces.iter().find_map(|f| match f {
            ToolSurface::Extruded(s) => Some(sub(s.at(0.5,1.).ok()?.position,s.at(0.5,0.).ok()?.position)),
            _ => None,
        })?;
        let axis = unit(if dot(c0,ruled) < 0. { scale(c0,-1.) } else { c0 });
        // each side's ruling: how far it leans out from the prism per unit along `axis` (above
        // nought where it grows going along), and the height of its middle; a side square to the
        // axis is no side of a prism
        let mut leans: Vec<(f64,f64)> = Vec::new();
        for f in &tool.faces {
            // each side's ruling at its middle: a strip's from its edge at `v = 0` to its far one;
            // a whole turn of a line about a line along the prism (a round hole's wall, a drum's
            // side) its meridian, a ruling of the cylinder or cone it is
            let ruling = match f {
                ToolSurface::Cap(_) => continue,
                f if flat(f) => continue,
                ToolSurface::Revolved(s) => {
                    if !matches!(s.meridian(),crate::swept::surface::Meridian::Line {..})
                        || norm(cross(s.axis_direction(),axis)) > 1e-9 { return None }
                    let (Ok(p),Ok(q)) = (s.at(0.,0.),s.at(1.,0.)) else { return None };
                    (p,q)
                }
                ToolSurface::Extruded(s) => { let (Ok(p),Ok(q)) = (s.at(0.5,0.),s.at(0.5,1.)) else { return None }; (p,q) }
            };
            let (p,q) = ruling;
            let r = sub(q.position,p.position);
            let h = dot(r,axis);
            if !(h.abs() > 1e-9*norm(r)) { return None }
            // how far the ruling runs out across the side's outward normal square to the
            // axis (it may slide along the side as well, its edge's ends moving apart or
            // together): the tangent of the side's draft
            let lateral = sub(r,scale(axis,h));
            let n = cross(p.du,p.dv);
            let out = sub(n,scale(axis,dot(n,axis)));
            if !(norm(out) > 1e-9*norm(n)) { return None }
            let out = scale(out,1./norm(out));
            leans.push((dot(lateral,out)/h,dot(add(p.position,scale(r,0.5)),axis)));
        }
        if leans.is_empty() { return None }
        // the boundary's planes square to the sides, each its loops: the caps, each a point of it
        // and its height along the sides, the nearer first
        let mut ends: Vec<(f64,V,usize)> = b.faces.iter().enumerate().filter_map(|(fi,f)| match &f.surface {
            Surface::Plane(frame) if norm(cross(frame.z,axis)) <= 1e-9 && !f.loops.is_empty() => {
                let p = b.vertices[b.edges[f.loops[0][0].edge as usize].v[0] as usize].p;
                Some((dot(p,axis),p,fi))
            }
            _ => None,
        }).collect();
        if ends.len() != 2 { return None }
        ends.sort_by(|x,y| x.0.total_cmp(&y.0));
        let [(h0,low,near),(h1,high,far)] = [ends[0],ends[1]];
        if !(h1-h0 > 0.) { return None }
        let cap = |fi: usize,normal: V,origin: V| -> Option<Profile> {
            let loops = b.faces[fi].loops.iter().map(|l| crate::offset::plane_loop(b,l,normal)).collect::<Result<Vec<_>,_>>().ok()?;
            let names = vec![Vec::new();loops.len()];
            Some(Profile {origin,normal,loops,names})
        };
        let size = h1-h0;
        let alike = |a: f64,c: f64| (a-c).abs() <= 1e-7*(1.+a.abs().max(c.abs()));
        let g = leans[0].0;
        let cutter = if tool.creases.is_empty() {
            if !leans.iter().all(|&(x,_)| alike(x,g)) { return None }
            if g.abs() <= 1e-9 {
                // undrafted: the nearer cap's loops carried along
                Cutter::extruded(cap(near,axis,low)?,0.,size,None)
            } else if g < 0. {
                // narrowing along the axis: the low cap's loops, drafted into the prism going up
                Cutter::extruded(cap(near,axis,low)?,0.,size,Some((-g).datan()))
            } else {
                // widening along it: the high cap's, drafted going down
                Cutter::extruded(cap(far,scale(axis,-1.),high)?,0.,size,Some(g.datan()))
            }
        } else {
            // drafted both ways from a parting plane: the sides below it widen to it, those above
            // narrow from it, all at one draft; its section is the low cap's moved out to it
            let [below,_] = tool.creases[0];
            let parting = dot(tool.faces.get(below)?.at(0.5,1.).ok()?.position,axis);
            if !(parting > h0 && parting < h1) || !(g.abs() > 1e-9) { return None }
            let taper = g.abs();
            let way = |&(x,h): &(f64,f64)| if h < parting { alike(x,taper) } else { alike(x,-taper) };
            if !leans.iter().all(way) { return None }
            let up = Rigid {r:[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]],t:scale(axis,parting-h0)};
            let profile = crate::build::offset(&cap(near,axis,low)?,-(parting-h0)*taper).ok()?.moved(&up);
            Cutter::extruded(profile,h0-parting,h1-parting,Some(taper.datan()))
        };
        Lone::new(cutter.ok()?,mm)
    }

    /// The prism's cutter, in millimetres.
    pub fn cutter(&self) -> &Cutter { &self.cutter }

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
        let Some((capped,corner,at,need)) = best else {
            return Err((Row::Sections,"the prism's cap reaches the blank, and the class sections a prism across its profile \
                only undrafted and of one loop (not built)".into(),hits.first().map(|h| h.0.position)))
        };
        if capped.clearance < need {
            let what = match corner {
                Some(p) => format!("a corner of the profile, where a section's side leaps from one face's normal to the next's, at \
                    ({:.4}, {:.4}, {:.4}) on the tool",p[0]/mm,p[1]/mm,p[2]/mm),
                None => "the profile's end, where a section closes to a ruling of a side".into(),
            };
            return Err((Row::Sections,format!("the prism's cap reaches the blank, and no direction across its profile \
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
pub fn check(c: &Generating,inside: &(dyn Fn(V) -> bool+Sync),options: &Sampling,lone: Option<&Lone>)
    -> Result<(Found,Reads),Failure> {
    let tool = c.source_material();
    let on_tool = |p: V| c.tool.on(p);
    let domain = c.domain();
    let middle = 0.5*(domain[0]+domain[1]);
    let pose = |t: f64| pose_at(c.motion(),t).map_err(|m| (Row::Motion,m,None));
    let times = [pose(domain[0])?,pose(middle)?,pose(domain[1])?];
    let roots = |patch: usize,u: f64,v: f64| c.at_source_over(patch,u,v,domain,options.root_tolerance);
    // (a lone prism's cap is cut where it is sectioned across: undrafted, one loop)
    let lone = lone.filter(|l| l.sectioned_across());
    let (mut reads,poses,capped) = ends(c,inside,options,lone,true)?;
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
            let Some(g) = c.normal_velocity(pole) else { continue };
            let scale = 1.+norm(s.position);
            let values = [domain[0],middle,domain[1]].map(|t| g.at(t).unwrap_or(f64::NAN));
            if !values.iter().all(|x| x.abs() <= options.root_tolerance*scale) { continue; }
            if let Some(p) = poses.iter().map(|m| m.point(s.position)).find(|&p| reads.read(inside,p)) {
                return Err((Row::Stationary,format!("the pole of `{}`, where its profile meets its axis, is in contact \
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
                let rate = c.normal_velocity(s);
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
                                return Err((Row::Stationary,format!("a tool point of `{}` inside a sample cell is in contact \
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
                                return Err((Row::Stationary,format!("tool points between samples of `{}` are in contact at every time",
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
                            return Err((Row::Stationary,
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
                    return Err((Row::Single,format!("a tool point touches the blank at roll {:.4} and again at {:.4}",
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
                        let said = undercut(c,patch,inside,options).map_or_else(
                            || format!("the generated surface's area factor is {factor:.3e}"),|u| u.to_string());
                        return Err((Row::Fold,said,Some(t.contact.position)));
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
            let said = undercut(c,*patch,inside,options).map_or_else(|| format!("the generated surface of `{}` turns back on \
                itself ({} samples one way, {} the other)",c.patches()[*patch].name(),counts[0],counts[1]),|u| u.to_string());
            return Err((Row::Fold,said,at[minority]));
        }
    }
    if hits.is_empty() {
        return Err((Row::Reach,"no contact of the tool within the roll lies in the blank".into(),None));
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
            let side = outward(tool,s.position,crate::util::space::unit(cross(s.du,s.dv)));
            let along = scale(crate::util::space::unit(s.dv),if v > 0.5 { 1. } else { -1. });
            fans.push(Fan {point:s.position,faces:[(along,side.map(|x| -x)),(side,along.map(|x| -x))],patch,crease:false});
        } }
    } }
    // each crease, the half toward the prism's `from` and the half toward its `to`
    for &[below,above] in c.creases() { for i in 0..=options.rows {
        let u = i as f64/options.rows as f64;
        let (Ok(a),Ok(b)) = (c.patches()[below].at(u,1.),c.patches()[above].at(u,0.)) else { continue };
        if !on_tool(a.position) { continue }
        let normal = |s: &SurfacePoint| outward(tool,a.position,crate::util::space::unit(cross(s.du,s.dv)));
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
                    return Err((Row::Fold,format!("the generated surface turns back {edge}: the {}'s fan sweeps back over {} \
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
                return Err((Row::Single,format!("a point of the tool {edge} touches the blank at roll {:.4} and again at {:.4}",
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
                    return Err((Row::Crossing,"two distant parts of the tool's sweep meet here".into(),Some(a.position)));
                }
            }}}
        }
    }
    // T3: a cap in the blank, sectioned across its profile clear of its corners and its ends
    let capped = match lone.filter(|_| capped.contains(&true)) {
        Some(lone) => Some(lone.sections(&hits)?),
        None => None,
    };
    Ok((Found {samples,contacts:hits.len(),spacing,least_area_factor:least,near_double_roots,near_tangent_pairs,capped},reads))
}

/// T2 and E1, then S1–S4 (`constant_twist`), for one placement of a sweep under a screw, a turn or
/// a slide, `inside` being the blank read in the sweep's frame, `heights` how far along the
/// motion's axis it reaches there. Their twist is constant, so M2 does not apply: a tool point is
/// on the boundary at every time or at none, and the boundary is the characteristic carried along.
/// A cap of `lone`, the tool where it is a prism standing alone or placed, its profile one loop,
/// may reach the blank, and the walk crosses it (#684 AE).
pub fn check_screw(c: &Generating,inside: &(dyn Fn(V) -> bool+Sync),extent: Option<([f64;2],f64)>,options: &Sampling,lone: Option<&Lone>)
    -> Result<(Characteristic,f64,Reads),Failure> {
    // (a lone prism's cap is walked, drafted or not and of any loops: each side's rim on it, a
    // straight side's whether it leans or not, and its contact line where it lies on the cap,
    // #721 F1d)
    // (a concave corner is the walk's to meet: it ends a stretch of the characteristic there, #721
    // F1d, and under a slide along the tool's own axis it is no part of the characteristic at all)
    // (a plane that is no lone prism's cap and holds no contact over the roll is no refusal:
    // `ends`, #721 F1d)
    let (reads,..) = ends(c,inside,options,lone,false)?;
    let reads = std::sync::Mutex::new(reads);
    // the blank read outside the lock, the faces' rings side by side
    let read = |p: V| { let i = inside(p); reads.lock().unwrap_or_else(|e| e.into_inner()).push(p,i); i };
    let ask = Ask {rows:options.rows,root_tolerance:options.root_tolerance,least_factor:options.least_factor,
        roll:c.domain(),heights:extent.map(|e| e.0),radius:extent.map(|e| e.1),inside:&read,caps:lone.is_some()};
    let found = Characteristic::walk(&Sweep {tool:c.tool,path:c.path,whole:c.whole},&ask)?;
    let [first,last] = found.reach;
    let mut gaps: Vec<f64> = (first..last).map(|w| crate::util::space::distance(found.nodes[w].position,found.nodes[w+1].position)).collect();
    gaps.sort_by(f64::total_cmp);
    let spacing = gaps.get(gaps.len()/2).copied().unwrap_or(0.);
    Ok((found,spacing,reads.into_inner().unwrap_or_else(|e| e.into_inner())))
}

