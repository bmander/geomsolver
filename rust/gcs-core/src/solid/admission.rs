//! The generating-sweep class (docs/generating-sweeps.md): which continuous sweeps a boundary
//! is constructed for, asked of the solved model before any construction. A tool that is a
//! revolution of lines and arcs (or an intersection of such), turned by rotations about fixed
//! axes at constant ratios, clear of the blank at both ends of its roll, touching each blank
//! point once, never folding and never crossing itself: that is what generates a bevel or a
//! hypoid, and everything else is refused by the condition it fails, with a point that fails
//! it. The conditions are **sampled**, never certified, and the evidence says so: a sweep this
//! admits has shown no failure at the samples it names, which is not a proof that none exists
//! between them. Material evaluation answers membership for every sweep, admitted or not; only
//! boundary construction is restricted.
use super::{SpatialField,SweepContacts,TimedContact,EdgeChart,cad};
use crate::{envelope::{Motion,SurfacePoint},model::{Sketch,SolidDef}};
use std::{collections::BTreeMap,f64::consts::TAU,fmt};

type V = [f64;3];
fn sub(a: V,b: V) -> V { std::array::from_fn(|k| a[k]-b[k]) }
fn dot(a: V,b: V) -> f64 { a[0]*b[0]+a[1]*b[1]+a[2]*b[2] }
fn cross(a: V,b: V) -> V { [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]] }
fn norm(a: V) -> f64 { dot(a,a).sqrt() }

/// The rows of the class, in the order they are asked.
#[derive(Clone,Copy,Debug,PartialEq,Eq,PartialOrd,Ord)]
pub enum Condition { Tool, Corner, Motion, Stationary, Clearance, Single, Fold, Crossing }

impl Condition {
    pub fn code(self) -> &'static str {
        match self { Condition::Tool => "T1", Condition::Corner => "T2", Condition::Motion => "M1",
            Condition::Stationary => "M2", Condition::Clearance => "E1", Condition::Single => "E2",
            Condition::Fold => "E3", Condition::Crossing => "E4" }
    }
    /// What the row asks, in the words the scope document uses.
    pub fn rule(self) -> &'static str {
        match self {
            Condition::Tool => "the tool is a full revolution of lines and arcs, or an intersection of such",
            Condition::Corner => "every profile corner the sweep carries into the blank is convex or tangent",
            Condition::Motion => "the motion is rotations about fixed axes at constant ratios",
            Condition::Stationary => "the contact condition depends on the motion",
            Condition::Clearance => "the tool is clear of the blank at both ends of the roll",
            Condition::Single => "each tool point touches the blank at most once in the roll",
            Condition::Fold => "the generated surface does not fold",
            Condition::Crossing => "the generated surface does not cross itself",
        }
    }
}

/// A sweep outside the class: which row, which sweep, what was seen, and where.
#[derive(Clone,Debug)]
pub struct Refusal {
    pub condition: Condition,
    pub sweep: String,
    pub message: String,
    /// A point in the body's coordinates where the condition fails, when there is one.
    pub witness: Option<V>,
}

impl fmt::Display for Refusal {
    fn fmt(&self,f: &mut fmt::Formatter) -> fmt::Result {
        write!(f,"`{}` is outside the generating-sweep class ({}: {}): {}",
            self.sweep,self.condition.code(),self.condition.rule(),self.message)?;
        if let Some(p) = self.witness { write!(f," at ({:.4}, {:.4}, {:.4})",p[0],p[1],p[2])?; }
        Ok(())
    }
}

#[derive(Clone,Debug)]
pub enum Error {
    /// Outside the class.
    Refused(Refusal),
    /// The body could not be read at all; nothing is said about the class.
    Unreadable(String),
}

impl fmt::Display for Error {
    fn fmt(&self,f: &mut fmt::Formatter) -> fmt::Result {
        match self { Error::Refused(r) => r.fmt(f), Error::Unreadable(m) => f.write_str(m) }
    }
}

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
        Options {rows:100,columns:400,coarse_rows:24,coarse_columns:1440,axis_tolerance:1e-10,
            margin:1e-6,root_tolerance:1e-9,least_factor:1e-3}
    }
}

/// How the evidence was obtained. Only sampling exists; a certified basis would be a second
/// variant, and a caller matching on this one learns of it when it arrives.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Basis { Sampled { rows: usize,columns: usize } }

/// One placement of a swept cut: its pose in the body, and whether its checks were skipped as
/// equivalent to an earlier placement's, which they are exactly when the blank is inside at
/// every point the earlier placement's checks read and outside at every other.
#[derive(Clone,Debug)]
pub struct Placement { pub pose: Motion, pub equivalent_to: Option<usize> }

#[derive(Clone,Debug)]
pub struct SweepEvidence {
    pub sweep: usize,
    pub name: String,
    pub placements: Vec<Placement>,
    /// Tool points sampled, and their contacts inside the blank, at the first placement.
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
}

#[derive(Clone,Debug,Default)]
pub struct Admission { pub sweeps: Vec<SweepEvidence> }

/// The body's static remainder as a field, and its swept cuts with their poses. Swept material
/// is read only where it is cut from the body itself, directly or through placements. A host
/// constructing the swept boundary reads the blank here too: exact, and microseconds a point.
pub fn static_remainder(sk: &Sketch,root: usize,axis_tolerance: f64) -> Result<(SpatialField,Vec<cad::SweptCut>),String> {
    super::validate(sk,root)?;
    let solid = &sk.solids[root];
    let SolidDef::Body {stock,on,through,bound} = &solid.def else {
        return Err(format!("`{}`: admission reads a body with swept cuts",solid.name));
    };
    if let Some(&o) = std::iter::once(stock).chain(on).chain(bound).find(|&&o| cad::contains_sweep(sk,o as usize)) {
        return Err(format!("`{}`: only a `cut` may hold swept material for now (`{}`)",solid.name,sk.solids[o as usize].name));
    }
    let read = |i: u32| SpatialField::read(sk,i as usize,axis_tolerance);
    let mut field = read(*stock)?;
    for &o in on { field = field.union(read(o)?).map_err(|e| format!("{e:?}"))?; }
    for &b in bound { field = field.intersection(read(b)?).map_err(|e| format!("{e:?}"))?; }
    let mut sweeps = Vec::new();
    for &c in through {
        match cad::swept_cut(sk,c as usize)? {
            Some(sweep) => sweeps.push(sweep),
            None => field = field.difference(read(c)?).map_err(|e| format!("{e:?}"))?,
        }
    }
    Ok((field,sweeps))
}

/// T1, as a question about the tool's solid graph.
fn tool(sk: &Sketch,id: usize) -> Result<(),String> {
    let solid = &sk.solids[id];
    match &solid.def {
        SolidDef::Revolve {sweep,..} => if (sweep.value.abs()-TAU).abs() > 1e-9 {
            Err(format!("`{}` is a partial revolution",solid.name)) } else { Ok(()) },
        SolidDef::Placed {source,..} => tool(sk,*source as usize),
        SolidDef::Body {stock,on,through,bound} => {
            if !on.is_empty() { return Err(format!("`{}` adds solids together; only intersections are in the class",solid.name)); }
            if !through.is_empty() { return Err(format!("`{}` cuts solids; only intersections are in the class",solid.name)); }
            std::iter::once(stock).chain(bound).try_for_each(|&o| tool(sk,o as usize))
        }
        SolidDef::Prism {..} => Err(format!("`{}` is a prism, not a revolution",solid.name)),
        SolidDef::Loft {..} => Err(format!("`{}` is a loft, not a revolution",solid.name)),
        SolidDef::Through {..} => Err(format!("`{}` is a through prism, not a revolution",solid.name)),
        SolidDef::Swept {..} => Err(format!("`{}` is itself a sweep",solid.name)),
    }
}

/// Which tool points the checks read the blank at, and whether each was inside: two placements
/// agreeing on all of them would compute the same checks to the last number.
struct Reads { points: Vec<V>,inside: Vec<bool> }

/// Admit or refuse every swept cut of the body `root`.
pub fn admit_body(sk: &Sketch,root: usize,options: &Options) -> Result<Admission,Error> {
    let (field,cuts) = static_remainder(sk,root,options.axis_tolerance).map_err(Error::Unreadable)?;
    let mut by_sweep: BTreeMap<usize,Vec<Motion>> = BTreeMap::new();
    let mut order = Vec::new();
    for cut in cuts {
        if !by_sweep.contains_key(&cut.swept) { order.push(cut.swept); }
        by_sweep.entry(cut.swept).or_default().push(cut.pose);
    }
    let mut admission = Admission::default();
    for swept in order {
        let name = sk.solids[swept].name.clone();
        let refuse = |condition,message: String,witness| Error::Refused(Refusal {condition,sweep:name.clone(),message,witness});
        let SolidDef::Swept {source,..} = &sk.solids[swept].def else { unreachable!("a swept cut is a sweep") };
        tool(sk,*source as usize).map_err(|m| refuse(Condition::Tool,m,None))?;
        let contacts = SweepContacts::read(sk,swept,options.axis_tolerance).map_err(|m| refuse(Condition::Tool,m,None))?;
        let probe = SurfacePoint {position:[1.,2.,3.],du:[1.,0.,0.],dv:[0.,1.,0.]};
        contacts.motion().normal_velocity(probe).map_err(|m| refuse(Condition::Motion,m,None))?;
        let poses = &by_sweep[&swept];
        let mut placements: Vec<Placement> = Vec::new();
        let mut checked: Vec<(usize,Reads)> = Vec::new();
        let mut evidence = None;
        for (k,pose) in poses.iter().enumerate() {
            let inside = |p: V| field.value(pose.point(p)) < -options.margin;
            let same = checked.iter().find(|(_,reads)| reads.points.iter().zip(&reads.inside).all(|(p,&i)| inside(*p) == i));
            if let Some((j,_)) = same { placements.push(Placement {pose:*pose,equivalent_to:Some(*j)}); continue; }
            let (found,reads) = check(&contacts,&inside,options).map_err(|(c,m,w)| refuse(c,m,w.map(|p| pose.point(p))))?;
            if evidence.is_none() { evidence = Some(found); }
            checked.push((k,reads));
            placements.push(Placement {pose:*pose,equivalent_to:None});
        }
        let mut found = evidence.expect("a swept cut has a placement");
        found.sweep = swept; found.name = name; found.placements = placements;
        admission.sweeps.push(found);
    }
    Ok(admission)
}

/// One in-blank contact of the fine pass.
#[derive(Clone,Copy)]
struct Hit { source: V,position: V,normal: V }

type Failure = (Condition,String,Option<V>);

/// M2, E1–E4 and T2 for one placement, `inside` being the blank read in the sweep's frame.
fn check(c: &SweepContacts,inside: &dyn Fn(V) -> bool,options: &Options) -> Result<(SweepEvidence,Reads),Failure> {
    let tool = c.source_material();
    let on_tool = |p: V| tool.value(p).abs() < 1e-6;
    let domain = c.domain();
    let middle = 0.5*(domain[0]+domain[1]);
    let pose = |t: f64| c.motion().at(t).map_err(|m| (Condition::Motion,m,None));
    let limits = [pose(domain[0])?,pose(domain[1])?];
    let times = [pose(domain[0])?,pose(middle)?,pose(domain[1])?];
    let roots = |patch: usize,u: f64,v: f64| c.at_source_over(patch,u,v,domain,options.root_tolerance);
    let mut reads = Reads {points:Vec::new(),inside:Vec::new()};
    let mut read = |p: V| { let i = inside(p); reads.points.push(p); reads.inside.push(i); i };
    // T2: a concave corner carried through the blank trims two envelopes against each other.
    // Asked first, since the crossing it makes would otherwise be reported in its place.
    let steps = 64;
    let poses: Vec<Motion> = (0..=steps).map(|k| pose(domain[0]+(domain[1]-domain[0])*k as f64/steps as f64))
        .collect::<Result<_,_>>()?;
    for edge in c.edges() {
        let [a,b] = edge.faces;
        let (Some(pa),Some(pb)) = (c.patches().get(a),c.patches().get(b)) else { continue };
        let (EdgeChart::FixedU(ua),EdgeChart::FixedU(ub)) = (edge.charts[0],edge.charts[1]) else { continue };
        let h = 1e-4;
        let inward = |u: f64| if u < 0.5 { u+h } else { u-h };
        let (Ok(corner),Ok(sa),Ok(sb)) = (pa.at(ua,0.),pa.at(inward(ua),0.),pb.at(inward(ub),0.)) else { continue };
        if !on_tool(corner.position) { continue; }
        let (ta,tb) = (sub(sa.position,corner.position),sub(sb.position,corner.position));
        // Opposite tangents: the profile is smooth there.
        if dot(ta,tb) < -(1.-1e-9)*norm(ta)*norm(tb) { continue; }
        let chord: V = std::array::from_fn(|k| 0.5*(sa.position[k]+sb.position[k]));
        if tool.value(chord) <= 0. { continue; }
        for j in 0..options.coarse_columns {
            let Ok(point) = pa.at(ua,j as f64/options.coarse_columns as f64) else { continue };
            if let Some(p) = poses.iter().map(|m| m.point(point.position)).find(|&p| read(p)) {
                return Err((Condition::Corner,format!("`{}` meets `{}` at a concave corner",pa.name,pb.name),Some(p)));
            }
        }
    }
    let (mut samples,mut near_double_roots) = (0,0);
    let mut hits: Vec<(Hit,usize,usize)> = Vec::new();
    let mut gaps: Vec<f64> = Vec::new();
    let mut least = f64::INFINITY;
    // E1, over the whole tool, before anything that might be reported in its place.
    for surface in c.patches() {
        for i in 0..=options.coarse_rows { for j in 0..options.coarse_columns {
            let Ok(s) = surface.at(i as f64/options.coarse_rows as f64,j as f64/options.coarse_columns as f64) else { continue };
            if !on_tool(s.position) { continue; }
            for limit in &limits {
                let p = limit.point(s.position);
                if read(p) { return Err((Condition::Clearance,
                    "the tool at a limit of its roll lies in the blank".into(),Some(p))); }
            }
        }}
    }
    // E3's sign: the area factor times the rate of the contact condition. The factor alone
    // runs through infinity and changes sign where a point's two contact times merge, which
    // is a fold of the tool's time chart and not of the surface; the product stays finite
    // there, and changes sign only where the generated surface itself folds.
    let mut signs: BTreeMap<(usize,usize),([usize;2],[Option<V>;2])> = BTreeMap::new();
    for patch in 0..c.patches().len() {
        let surface = &c.patches()[patch];
        let (cu,cv) = (options.coarse_rows,options.coarse_columns);
        let mut reach = vec![false;cv];
        for i in 0..=cu { for j in 0..cv {
            let (u,v) = (i as f64/cu as f64,j as f64/cv as f64);
            let Ok(s) = surface.at(u,v) else { continue };
            if !on_tool(s.position) { continue; }
            if let Ok(list) = roots(patch,u,v) {
                if list.iter().any(|t| read(t.contact.position)) { reach[j] = true; }
            }
        }}
        // The shortest arc of the revolution holding every reaching angle, widened a step.
        let reached: Vec<usize> = (0..cv).filter(|&j| reach[j]).collect();
        let band = if reached.is_empty() { None } else {
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
        let v_at = |j: usize| ((start as f64+width as f64*j as f64/nv as f64)/cv as f64).rem_euclid(1.);
        let mut previous: Vec<Option<V>> = vec![None;nv+1];
        for i in 0..=nu {
            let u = i as f64/nu as f64;
            let mut row: Vec<Option<V>> = vec![None;nv+1];
            for j in 0..=nv {
                let v = v_at(j);
                let Ok(s) = surface.at(u,v) else { continue };
                if !on_tool(s.position) { continue; }
                samples += 1;
                // M2: where a point's contact condition does not change over the roll, the
                // point is on the boundary at every time or at none, and its time is no
                // parameter of the generated surface. A single rotation is that everywhere.
                let rate = c.motion().normal_velocity(s).ok();
                if let Some(g) = &rate {
                    let scale = 1.+norm(s.position);
                    let at = [domain[0],middle,domain[1]].map(|t| g.at(t).unwrap_or(f64::NAN));
                    if at.iter().all(|x| (x-at[1]).abs() <= options.root_tolerance*scale) {
                        if let Some(p) = times.iter().map(|m| m.point(s.position)).find(|&p| read(p)) {
                            return Err((Condition::Stationary,
                                "a tool point's contact condition does not change over the roll".into(),Some(p)));
                        }
                    }
                }
                let list = match roots(patch,u,v) { Ok(l) => l, Err(_) => { near_double_roots += 1; continue } };
                let found: Vec<&TimedContact> = list.iter().filter(|t| read(t.contact.position)).collect();
                if found.len() > 1 {
                    return Err((Condition::Single,format!("a tool point touches the blank at roll {:.4} and again at {:.4}",
                        found[0].root.time,found[1].root.time),Some(found[1].contact.position)));
                }
                for t in found {
                    let hit = Hit {source:s.position,position:t.contact.position,normal:t.contact.normal};
                    row[j] = Some(hit.position);
                    for q in [previous[j],if j > 0 { row[j-1] } else { None }].into_iter().flatten() {
                        gaps.push(norm(sub(q,hit.position)));
                    }
                    hits.push((hit,patch,t.root.branch));
                    // E3: the generated surface's oriented area factor on the same branch,
                    // by central differences, against the tool surface's own.
                    let h = 1e-5;
                    let (ua,ub) = ((u-h).max(0.),(u+h).min(1.));
                    let near = |uu: f64,vv: f64| -> Option<V> {
                        roots(patch,uu,vv.rem_euclid(1.)).ok()?.into_iter()
                            .filter(|x| x.root.branch == t.root.branch)
                            .min_by(|a,b| (a.root.time-t.root.time).abs().total_cmp(&(b.root.time-t.root.time).abs()))
                            .map(|x| x.contact.position)
                    };
                    let (Some(fa),Some(fb),Some(ga),Some(gb)) = (near(ua,v),near(ub,v),near(u,v-h),near(u,v+h)) else { continue };
                    let at = |uu: f64,vv: f64| surface.at(uu,vv.rem_euclid(1.)).map(|s| s.position).ok();
                    let (Some(sa),Some(sb),Some(ta),Some(tb)) = (at(ua,v),at(ub,v),at(u,v-h),at(u,v+h)) else { continue };
                    let du = ub-ua;
                    let area = norm(cross(sub(sb,sa).map(|x| x/du),sub(tb,ta).map(|x| x/(2.*h))));
                    if area == 0. { continue; }
                    let factor = dot(cross(sub(fb,fa).map(|x| x/du),sub(gb,ga).map(|x| x/(2.*h))),t.contact.normal)/area;
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
        }
    }
    for ((patch,_),(counts,at)) in &signs {
        if counts[0] > 0 && counts[1] > 0 {
            let minority = usize::from(counts[1] < counts[0]);
            return Err((Condition::Fold,format!("the generated surface of `{}` turns back on itself ({} samples one way, {} the other)",
                c.patches()[*patch].name,counts[0],counts[1]),at[minority]));
        }
    }
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
    Ok((SweepEvidence {sweep:0,name:String::new(),placements:Vec::new(),samples,contacts:hits.len(),spacing,
        least_area_factor:least,near_double_roots,near_tangent_pairs,
        basis:Basis::Sampled {rows:options.rows,columns:options.columns}},reads))
}
