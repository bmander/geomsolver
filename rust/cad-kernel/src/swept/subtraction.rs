//! **The planar subtraction class, read from the tool's B-rep** (#606, #607, #721 F1c): a sweep of a
//! tool that stands along one direction under a motion that turns the plane square to it in itself,
//! cut from any blank. Within a slab of the blank that no face square to that direction crosses,
//! the tool's section is the same at every height (or, where a side leans, moves as the side
//! leans), and so is what the sweep removes: the region `R = ∪ₜ poseₜ(section)` over the roll
//! (`region::SweptRegion`), and in the slab the cut is the blank less `R`'s prism. The rows:
//!
//! - P1: the motion turns the plane in itself about axes square to it, the same face up;
//! - Q1: the tool stands along the plane's normal: every face of its B-rep a plane square to it
//!   (a cap), a plane or a cylinder along it (a side), or a plane or a cone about a line along it
//!   leaning to it (a drafted side) — however it was drawn, prisms joined, cut or drafted;
//! - Q2: the slabs are split at every cap within the blank, so a band of a stepped tool splits the
//!   blank itself; a slab whose sides lean is read at stations through it, and refused where its
//!   region changes shape between them;
//! - Q3: the roll is a whole period of the motion (the tool stands where it began, as the observer
//!   sees it), or the tool is clear of the blank at both its ends;
//! - Q4: the region's boundary is regular: pieces meeting at corners or leaving the blank's
//!   shadow, no fold, no crossing (`SweptRegion::read`).
//!
//! The tool is read only from its boundary (#721 F1): its section at a height is the face its
//! B-rep cut there by a half-space leaves on the plane (`section`), lines and arcs, its material
//! that loop's signed distance (`Loops`). Sampled where the other classes are: the region's
//! candidates on a grid of the roll, Q3's clearance on a grid of the section.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use crate::build::{walks,Profile,Seg};
use crate::geom::{Curve,Surface};
use crate::plane2::Edge;
use crate::topo::Brep;
use crate::trajectory::{Motion,Trajectory};
use crate::util::space::{across,add,cross,dot,norm,scale};
use super::planar::Region;
use super::region::{keeps,Plane,Section,Setting,SweptRegion,PER_DEGREE};
use std::collections::BTreeMap;
use std::sync::{Arc,Mutex};

type V = [f64;3];
type P = [f64;2];

/// A row that fails, what was seen, and a point of the body where, when there is one.
pub type Failure = (Row,String,Option<V>);

/// The class's rows are the dispatch's: P1, P3, Q1–Q4 and E0.
use super::dispatch::Row;

/// What a sweep is called, in the words a refusal says it in: the sweep, its tool and its motion.
#[derive(Clone,Debug,Default)]
pub struct Names { pub sweep: String,pub tool: String,pub motion: String }

/// How near square or along a direction a unit axis must be to be read as such.
const ALIGNED: f64 = 1e-9;

/// What a face of the tool is along the plane's normal.
enum Along { Cap,Side,Leaning }

/// Face `s`'s reading along unit `n`, none where it is no face of a tool standing along it.
fn along(s: &Surface,n: V) -> Option<Along> {
    let parallel = |a: V| norm(cross(a,n)) <= ALIGNED*norm(a);
    match s {
        Surface::Plane(f) if parallel(f.z) => Some(Along::Cap),
        Surface::Plane(f) if dot(f.z,n).abs() <= ALIGNED*norm(f.z) => Some(Along::Side),
        Surface::Plane(_) => Some(Along::Leaning),
        Surface::Cylinder(f,_) if parallel(f.z) => Some(Along::Side),
        Surface::Cone(f,..) if parallel(f.z) => Some(Along::Leaning),
        _ => None,
    }
}

/// Points along face `fi`'s loops (millimetres): each edge's ends and 31 places between.
fn face_points(b: &Brep,fi: usize) -> Vec<V> {
    b.faces[fi].loops.iter().flatten().flat_map(|c| {
        let e = &b.edges[c.edge as usize];
        (0..=32).map(move |k| e.point(e.t[0]+(e.t[1]-e.t[0])*k as f64/32.,&b.vertices))
    }).collect()
}

/// **A tool standing along the plane's normal, read from its B-rep** (Q1): the plane (a point of it
/// the world's origin, so a height is the normal's ordinate in space), the heights of its caps and
/// of the stretches its leaning sides span, how far along the normal it reaches (model units), and
/// its boundary (millimetres, `mm` a model unit) to section, each section kept once read.
pub struct Prismatic { pub plane: Plane,caps: Vec<f64>,leaning: Vec<[f64;2]>,span: [f64;2],tool: Brep,mm: f64,
    /// Each side's box (millimetres), about its edges' points; none for a cap.
    boxes: Vec<Option<(V,V)>>,
    sections: Mutex<BTreeMap<(u64,bool),Result<Vec<Vec<Edge>>,String>>> }

impl std::fmt::Debug for Prismatic {
    fn fmt(&self,f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f,"Prismatic {{ n: {:?},caps: {:?},leaning: {:?},span: {:?} }}",self.plane.n,self.caps,self.leaning,self.span)
    }
}

impl Prismatic {
    /// Tool `b` (millimetres, `mm` a model unit) read as standing along unit `n` — every face a cap,
    /// a side or a leaning side, the sides across any one height leaning alike — or the face it does
    /// not stand along by: its name and a point of it (model units).
    pub fn along(b: &Brep,n: V,mm: f64) -> Result<Prismatic,(String,V)> {
        let (u,v) = across(n);
        let plane = Plane {o:[0.;3],u,v,n};
        let (mut caps,mut leaning,mut span) = (Vec::new(),Vec::new(),[f64::INFINITY,f64::NEG_INFINITY]);
        // each side's heights, how far it moves into the material a unit up (a draft's rate; a
        // side standing along the normal's nought), its name and a point of it (model units)
        let mut sides: Vec<([f64;2],f64,String,V)> = Vec::new();
        let mut boxes = Vec::new();
        for (fi,f) in b.faces.iter().enumerate() {
            let points = face_points(b,fi);
            // (a hundredth of its diagonal past its edges' points, which an arc between bulges past)
            let (lo,hi) = points.iter().fold(([f64::INFINITY;3],[f64::NEG_INFINITY;3]),|(lo,hi),p|
                (std::array::from_fn(|k| lo[k].min(p[k])),std::array::from_fn(|k| hi[k].max(p[k]))));
            let pad = 0.01*crate::util::space::distance(lo,hi);
            let reading = along(&f.surface,n);
            boxes.push((!matches!(reading,Some(Along::Cap))).then(|| (lo.map(|x| x-pad),hi.map(|x| x+pad))));
            let heights = points.iter().fold([f64::INFINITY,f64::NEG_INFINITY],|[lo,hi],p| { let h = dot(*p,n)/mm; [lo.min(h),hi.max(h)] });
            span = [span[0].min(heights[0]),span[1].max(heights[1])];
            let at = points.first().map_or([0.;3],|p| scale(*p,1./mm));
            let name = crate::recipe::joined(&f.name);
            match reading {
                Some(Along::Cap) => caps.push(dot(f.surface.frame().o,n)/mm),
                Some(Along::Side) => sides.push((heights,0.,name,at)),
                Some(Along::Leaning) => {
                    let m = points.iter().find_map(|p| f.outward(*p)).ok_or_else(|| (name.clone(),at))?;
                    let up = dot(m,n);
                    sides.push((heights,-up/(1.-up*up).max(0.).sqrt(),name,at));
                    leaning.push(heights)
                }
                None => return Err((name,at)),
            }
        }
        // the sides across any one height lean alike, by one draft or none: the section there is
        // the section anywhere in the slab moved into its material, so its region is read at
        // stations through the slab; sides leaning unalike (a slanted cut, a side of a prism lying
        // across the plane) cross the slab as no draft does
        for (i,(hi,ri,..)) in sides.iter().enumerate() {
            for (hj,rj,name,at) in &sides[i+1..] {
                let overlap = hi[1].min(hj[1])-hi[0].max(hj[0]);
                if overlap > SAME*(1.+span[1].abs().max(span[0].abs())) && (ri-rj).abs() > 1e-9*(1.+ri.abs()) {
                    return Err((name.clone(),*at))
                }
            }
        }
        if !(span[1] > span[0]) { return Err(("the tool of no height".into(),[0.;3])) }
        caps.sort_by(f64::total_cmp);
        caps.dedup_by(|a,b| (*a-*b).abs() <= SAME*(1.+a.abs()));
        Ok(Prismatic {plane,caps,leaning,span,tool:b.clone(),mm,boxes,sections:Mutex::new(BTreeMap::new())})
    }

    /// The heights `[lo, hi]` split into slabs at every cap of the tool strictly within them,
    /// lowest first: over each the tool's section is one (or, where a side leans, moves as the side
    /// leans).
    pub fn split(&self,[lo,hi]: [f64;2]) -> Vec<[f64;2]> {
        let mut planes: Vec<f64> = vec![lo,hi];
        planes.extend(self.caps.iter().copied().filter(|&h| h > lo+SAME && h < hi-SAME));
        planes.sort_by(f64::total_cmp);
        planes.dedup_by(|a,b| (*a-*b).abs() <= SAME*(1.+a.abs()));
        planes.windows(2).map(|w| [w[0],w[1]]).collect()
    }

    /// The tool's own slabs, between its caps over its whole height (`split`).
    pub fn slabs(&self) -> Vec<[f64;2]> { self.split(self.span) }

    /// What the region the section at height `h` (from below or above) sweeps under `path` over
    /// `roll` is read from, the tool's reach the length its tolerances are taken against.
    pub fn setting(&self,h: f64,below: bool,path: &Trajectory,roll: [f64;2]) -> Result<Setting,String> {
        Ok(Setting {section:section_of(self.section(h,below)?),path:path.clone(),plane:self.plane,roll,scale:self.reach()})
    }

    /// Whether a side leans across height `h`.
    pub fn leans(&self,h: f64) -> bool { self.leaning.iter().any(|s| h > s[0] && h < s[1]) }

    /// How far the tool's points reach from the plane's normal through its origin (model units).
    pub fn reach(&self) -> f64 {
        self.tool.vertices.iter().map(|v| { let p = self.plane.page(scale(v.p,1./self.mm)); p[0].dhypot(p[1]) }).fold(0.,f64::max)
    }

    /// **The tool's section at height `h`** (model units, the plane's coordinates), its loops walked
    /// with the material on the left (outer loops counter-clockwise about the normal): the face the
    /// tool's B-rep cut by the plane there leaves on it, read from the side below (`below`) or above
    /// — where `h` is a cap's height, the section on that side of it. None where the tool does not
    /// reach `h`.
    pub fn section(&self,h: f64,below: bool) -> Result<Vec<Vec<Edge>>,String> {
        let key = (h.to_bits(),below);
        if let Some(s) = self.sections.lock().unwrap_or_else(|e| e.into_inner()).get(&key) { return s.clone() }
        let s = self.cut(h,below);
        self.sections.lock().unwrap_or_else(|e| e.into_inner()).insert(key,s.clone());
        s
    }

    /// The first of the tool's sides a section's step (millimetres) lies on: its middle on the
    /// face's surface.
    fn face_of(&self,s: &Seg) -> usize {
        let q = s.curve.point(0.5*(s.t[0]+s.t[1]));
        let tol = 1e-7*(1.+self.tool.size());
        // (on its surface and within the box its edges span: a surface two faces lie on, as the
        // sides of a plus's opposite arms do, is told apart by where each face is)
        self.tool.faces.iter().zip(&self.boxes).position(|(f,boxed)| boxed.is_some_and(|(lo,hi)|
            (0..3).all(|k| q[k] >= lo[k]-tol && q[k] <= hi[k]+tol) && crate::util::space::distance(f.surface.point(f.surface.inverse(q)),q) <= tol))
            .unwrap_or(usize::MAX)
    }

    fn cut(&self,h: f64,below: bool) -> Result<Vec<Vec<Edge>>,String> {
        let (mm,n) = (self.mm,self.plane.n);
        let reach = SAME*(1.+h.abs());
        if (below && h <= self.span[0]+reach) || (!below && h >= self.span[1]-reach) { return Ok(Vec::new()) }
        let keep = if below { scale(n,-1.) } else { n };
        let piece = crate::build::kept(&self.tool,scale(n,h*mm),keep,0.)?;
        // the faces the cut leaves on the plane, looking away from what is kept
        let on = 1e-7*(1.+(h*mm).abs()+self.tool.size());
        let coords = |q: V| self.plane.page(scale(q,1./mm));
        let mut loops: Vec<(usize,Vec<Edge>)> = Vec::new();
        for f in &piece.faces {
            let Surface::Plane(fr) = &f.surface else { continue };
            let out = if f.reversed { scale(fr.z,-1.) } else { fr.z };
            if norm(cross(out,n)) > 1e-9 || dot(out,keep) > 0. || (dot(fr.o,n)-h*mm).abs() > on { continue }
            for l in &f.loops {
                let edges = crate::offset::plane_loop(&piece,l,n)?;
                let profile = Profile {origin:fr.o,normal:n,loops:vec![edges],names:vec![Vec::new()]};
                let [walk] = &walks(&profile,&|q| coords(q))?[..] else { unreachable!("one loop walked") };
                // each step begun at the step on the tool's first face, so that a loop is walked in
                // one order at every height its section is read at (a drafted slab's stations are
                // matched piece for piece)
                let keys: Vec<usize> = walk.iter().map(|s| self.face_of(s)).collect();
                let first = (0..keys.len()).min_by_key(|&k| keys[k]).unwrap_or(0);
                let walk: Vec<&Seg> = walk[first..].iter().chain(&walk[..first]).collect();
                let mut edges: Vec<Edge> = walk.iter().map(|s| edge(s,n,mm,&coords)).collect::<Result<_,_>>()?;
                // each corner one point, shared by the edges either side of it, as a drawn loop's is
                for k in 0..edges.len() {
                    let j = (k+1)%edges.len();
                    let (a,b) = (edges[k].endpoints()[1],edges[j].endpoints()[0]);
                    let corner = [0.5*(a[0]+b[0]),0.5*(a[1]+b[1])];
                    set_end(&mut edges[k],1,corner);
                    set_end(&mut edges[j],0,corner);
                }
                loops.push((keys[first],edges));
            }
        }
        loops.sort_by_key(|(key,_)| *key);
        let mut loops: Vec<Vec<Edge>> = loops.into_iter().map(|(_,l)| l).collect();
        // each loop counter-clockwise; one within an odd number of others a hole, turned round
        let inner: Vec<bool> = (0..loops.len()).map(|i| {
            let p = loops[i][0].at(0.5);
            (0..loops.len()).filter(|&j| j != i && loops[j].iter().map(|e| e.crossings(p)).sum::<usize>()%2 == 1).count()%2 == 1
        }).collect();
        for (l,inner) in loops.iter_mut().zip(inner) {
            // (its first step still the one it began at)
            if inner { *l = l.drain(..).rev().map(Edge::reversed).collect(); l.rotate_right(1); }
        }
        Ok(loops)
    }
}

/// A walk's step (millimetres) as an edge of the plane's (`coords`, model units): a line, or an arc
/// turning about the normal `n` as its circle does.
fn edge(s: &Seg,n: V,mm: f64,coords: &dyn Fn(V) -> P) -> Result<Edge,String> {
    let ends = [coords(s.start()),coords(s.end())];
    match &s.curve {
        Curve::Line {..} => Ok(Edge::Line {a:ends[0],b:ends[1],axis:false}),
        Curve::Circle(frame,r) => {
            let center = coords(frame.o);
            let start = (ends[0][1]-center[1]).datan2(ends[0][0]-center[0]);
            let sweep = (s.t[1]-s.t[0])*dot(frame.z,n).signum();
            Ok(Edge::Arc {center,radius:r/mm,start,sweep,ends})
        }
        other => Err(format!("a section's edge on a {}",other.kind())),
    }
}

/// End `end` of edge `e` (0 its start, 1 its end) moved to `p`: a line's end, an arc's recorded end.
fn set_end(e: &mut Edge,end: usize,p: P) {
    match e {
        Edge::Line {a,b,..} => if end == 0 { *a = p } else { *b = p },
        Edge::Arc {ends,..} => ends[end] = p,
        Edge::Conic {a,b,..} => if end == 0 { *a = p } else { *b = p },
    }
}

/// **A section's material** in its plane: the signed distance to its loops, negative inside (an odd
/// number of them about the point), one-Lipschitz, zero on its boundary. Its edges are held in a
/// tree by their boxes, so a reading visits only the edges that may be nearest and those the ray
/// from the point can cross (a pocket's loop is thousands of chords, #721 F1d).
#[derive(Clone,Debug)]
pub struct Loops { edges: Vec<Edge>,tree: crate::util::bvh::Bvh<2> }

impl Loops {
    pub fn new(loops: &[Vec<Edge>]) -> Loops {
        let edges: Vec<Edge> = loops.iter().flatten().cloned().collect();
        let tree = crate::util::bvh::Bvh::new(edges.iter().map(|e| { let [lo,hi] = e.boxed(); crate::util::bvh::Bounds {lo,hi} }));
        Loops {edges,tree}
    }
}

impl Region for Loops {
    fn value(&self,p: P) -> f64 {
        let at = crate::util::bvh::Bounds {lo:p,hi:p};
        let nearest = std::cell::Cell::new(f64::INFINITY);
        self.tree.query_nearest(|b| b.gap(at),|| nearest.get(),|i| nearest.set(nearest.get().min(self.edges[i].distance(p))));
        // (the ray runs from the point toward +x: only an edge whose box it meets can cross it)
        let mut crossed = 0;
        self.tree.query(|b| b.overlaps(crate::util::bvh::Bounds {lo:p,hi:[f64::INFINITY,p[1]]}),|i| crossed += self.edges[i].crossings(p));
        if crossed%2 == 1 { -nearest.get() } else { nearest.get() }
    }
}

/// The normals of tool `b`'s plane faces (millimetres, `mm` a model unit), each once in the faces'
/// order, whose planes through the tool's middle every one of `poses` keeps (`region::keeps`, P1).
pub fn kept_normals(b: &Brep,poses: &[Motion],mm: f64) -> Vec<V> {
    let (lo,hi) = b.bounds();
    let o = scale(add(lo,hi),0.5/mm);
    let mut normals: Vec<V> = Vec::new();
    for f in &b.faces {
        let Surface::Plane(fr) = &f.surface else { continue };
        if !normals.iter().any(|m| norm(cross(*m,fr.z)) <= ALIGNED) { normals.push(fr.z) }
    }
    normals.into_iter().filter(|&n| poses.iter().all(|m| keeps(m,o,n,1.+norm(o)))).collect()
}

/// The section of `loops` as the region reads it: one member, its material their `Loops`.
pub fn section_of(loops: Vec<Vec<Edge>>) -> Section {
    if loops.is_empty() { return Section::new(vec![None],None) }
    let field = Loops::new(&loops);
    Section::new(vec![Some(loops)],Some(Arc::new(field)))
}

/// Heights within this of each other (relative) are one plane.
const SAME: f64 = 1e-9;

/// What the class read of one sweep's tool: the tool standing along the plane's normal, the motion,
/// the roll, the sweep's names and a length its tolerances are taken against.
#[derive(Debug)]
pub struct Subtraction { tool: Prismatic,path: Trajectory,roll: [f64;2],names: Names,scale: f64 }

/// One slab of the blank: its heights along the plane's normal, how far its cut reaches (a hair
/// past the blank where the slab ends at its face) and the region the section there sweeps —
/// drafted, the region at each station through it, the first at the lower end of its reach.
#[derive(Clone,Debug)]
pub struct Slab { pub heights: [f64;2],pub reach: [f64;2],pub stations: Vec<(f64,SweptRegion)> }

/// What the class found: the plane and each slab the tool reaches, lowest first.
#[derive(Clone,Debug)]
pub struct Found { pub plane: Plane,pub slabs: Vec<Slab> }

/// What a found sweep was read from: the candidates and the kept stretches its regions read, and
/// the roll's steps.
#[derive(Clone,Copy,Debug)]
pub struct Sampled { pub candidates: usize,pub runs: usize,pub steps: usize }

impl Subtraction {
    /// **Sweep of tool `b`** (millimetres, `mm` a model unit) along `path` over `roll`, read for the
    /// class: the plane its first face square to a direction the motion keeps is square to (P1, at
    /// five rolls), the tool standing along its normal (Q1). Refused at P1 where the motion keeps no
    /// plane a face of the tool is square to, and at Q1 naming the face that stands along no
    /// normal the motion keeps.
    pub fn read(b: &Brep,path: &Trajectory,roll: [f64;2],mm: f64,names: Names) -> Result<Subtraction,Failure> {
        let [a,z] = roll;
        let poses: Vec<Motion> = (0..=4).map(|k| path.at(a+(z-a)*k as f64/4.)).collect::<Result<_,_>>()
            .map_err(|e| (Row::Plane,e.msg,None))?;
        let kept = kept_normals(b,&poses,mm);
        let Some(&first) = kept.first() else {
            return Err((Row::Plane,format!("`{}` keeps no plane a face of `{}` is square to",names.motion,names.tool),None))
        };
        let mut refused = None;
        for n in kept {
            match Prismatic::along(b,n,mm) {
                Ok(tool) => {
                    let scale = tool.reach();
                    return Ok(Subtraction {tool,path:path.clone(),roll,names,scale})
                }
                Err(e) => { if n == first { refused = Some(e) } }
            }
        }
        let (face,at) = refused.expect("the first plane read");
        Err((Row::Members,format!("`{}`'s face `{face}` is no plane square to the plane `{}` keeps, nor a plane, a cylinder or a \
            cone standing along its normal leaning as every side beside it leans",names.tool,names.motion),Some(at)))
    }

    /// What the swept region at height `h` is read from: the section there (from below or above),
    /// the motion, the plane and the roll.
    fn setting(&self,h: f64,below: bool) -> Result<Setting,Failure> {
        self.tool.setting(h,below,&self.path,self.roll).map_err(|e| (Row::Members,format!("`{}`'s section: {e}",self.names.tool),None))
    }

    /// Q2–Q4 over every placement (`poses`, the sweep's frame in the body's), the blank read in the
    /// body by `inside` and boxed there by `bounds` (its least and greatest corner): the slabs the
    /// tool's caps split the blank into, Q3 at the roll's ends, and each slab's region within a box
    /// about the blank's shadow at every placement.
    pub fn check(&self,poses: &[Motion],inside: &(dyn Fn(V) -> bool+Sync),bounds: Option<(V,V)>) -> Result<Found,Failure> {
        let Some((lo,hi)) = bounds else { return Err((Row::Slabs,"the blank's extent is unknown".into(),None)) };
        let pl = self.tool.plane;
        let names = &self.names;
        // the blank's box in the sweep's frame, at every placement: its heights and its shadow
        let (mut heights,mut shadow) = ([f64::INFINITY,f64::NEG_INFINITY],[[f64::INFINITY;2],[f64::NEG_INFINITY;2]]);
        for pose in poses {
            let back = pose.inverse();
            for k in 0..8 {
                let c = back.point(std::array::from_fn(|i| if (k>>i)&1 == 0 { lo[i] } else { hi[i] }));
                let (h,p) = (pl.height(c),pl.page(c));
                heights = [heights[0].min(h),heights[1].max(h)];
                shadow = [[shadow[0][0].min(p[0]),shadow[0][1].min(p[1])],[shadow[1][0].max(p[0]),shadow[1][1].max(p[1])]];
            }
        }
        let size = (shadow[1][0]-shadow[0][0]).max(shadow[1][1]-shadow[0][1]).max(heights[1]-heights[0]);
        let boxed = |grown: f64| -> [P;2] {
            let pad = PAD*grown*size+1e-6*(1.+self.scale);
            [[shadow[0][0]-pad,shadow[0][1]-pad],[shadow[1][0]+pad,shadow[1][1]+pad]]
        };
        // Q2: the slabs, split at every cap within the blank; a slab the tool does not reach cuts
        // nothing
        // each slab the tool reaches: Q3, a whole period of its section or its tool clear of the
        // blank at both ends of the roll; then its region within the box, a drafted slab's at
        // stations through it (#607), from a little past the blank where the slab ends at its face
        let past = PAST*(heights[1]-heights[0]);
        let mut found = Vec::new();
        for slab in self.tool.split(heights) {
            let h = 0.5*(slab[0]+slab[1]);
            let setting = self.setting(h,true)?;
            if setting.section.is_empty() { continue }
            let [a,b] = self.roll;
            let whole = setting.alike(a,b);
            if !whole {
                let covered: Vec<V> = self.roll.iter().flat_map(|&t| setting.covered(t,48)).map(|p| pl.world(p,h)).collect();
                let hit = crate::util::par::map(&covered,|&w| poses.iter().map(|pose| pose.point(w)).find(|&x| inside(x)));
                if let Some((k,x)) = hit.into_iter().enumerate().find_map(|(k,x)| x.map(|x| (k,x))) {
                    let t = if k < covered.len()/2 { a } else { b };
                    let period = setting.period(a).map_or(String::new(),|p| format!(": {}",super::region::advice(a,p)));
                    return Err((Row::Closes,format!("`{}`'s tool `{}` lies in the blank at roll {:.4} degrees{period}",names.sweep,
                        names.tool,t.to_degrees()),Some(x)))
                }
            }
            let drafted = self.tool.leans(h);
            let reach = [slab[0]-if slab[0] == heights[0] { past } else { 0. },slab[1]+if slab[1] == heights[1] { past } else { 0. }];
            let read = |z: f64,bounds: [P;2]| SweptRegion::read(if drafted { self.setting(z,z > h)? } else { setting.clone() },bounds,whole)
                .map_err(|r| (row(r.seen.row()),r.words(&names.motion),r.witness.map(|p| poses[0].point(pl.world(p,z)))));
            let stations = if drafted { self.drafted(reach,&read,&boxed)? } else { vec![(h,read(h,boxed(1.))?)] };
            found.push(Slab {heights:slab,reach,stations});
        }
        if found.iter().all(|s| s.stations.iter().all(|(_,r)| r.loops.is_empty())) {
            return Err((Row::Reach,format!("no part of the region `{}` sweeps reaches the blank",names.sweep),None))
        }
        Ok(Found {plane:pl,slabs:found})
    }

    /// A drafted slab's stations through `reach`, each region read by `read` within the box `boxed`
    /// grows by each of `GROWN` in turn, until every station's loops are the first's piece for piece:
    /// where the region is cut by the box's sides is no part of the blank, so a change of shape only
    /// there is the box's (#612). A change that holds in every box is refused at the height it
    /// happens at, found between the stations.
    fn drafted(&self,reach: [f64;2],read: &(dyn Fn(f64,[P;2]) -> Result<SweptRegion,Failure>+Sync),boxed: &dyn Fn(f64) -> [P;2])
        -> Result<Vec<(f64,SweptRegion)>,Failure> {
        let at = stations(reach,STATIONS);
        let within = |bounds: [P;2]| crate::util::par::map(&at,|&z| read(z,bounds)).into_iter().collect::<Result<Vec<_>,_>>();
        let alike = |regions: &[SweptRegion]| regions.iter().all(|r| r.matching(&regions[0]).is_some());
        // the first box's refusal is the slab's; a grown box that refuses is passed over
        let bounds = boxed(GROWN[0]);
        let first = within(bounds)?;
        if alike(&first) { return Ok(at.into_iter().zip(first).collect()) }
        if let Some(regions) = GROWN[1..].iter().filter_map(|&grown| within(boxed(grown)).ok()).find(|r| alike(r)) {
            return Ok(at.into_iter().zip(regions).collect())
        }
        // the first box's stations: the last alike to the first and the next, and where between
        let j = (1..first.len()).find(|&j| first[j].matching(&first[0]).is_none()).expect("a station unlike the first");
        let unlike = |z: f64| read(z,bounds).map_or(true,|r| r.matching(&first[0]).is_none());
        let [mut lo,mut hi] = [at[j-1],at[j]];
        for _ in 0..2 {
            let zs: Vec<f64> = (1..8).map(|i| lo+(hi-lo)*i as f64/8.).collect();
            let marks = crate::util::par::map(&zs,|&z| unlike(z));
            let i = marks.iter().position(|&m| m).unwrap_or(zs.len());
            (lo,hi) = (if i == 0 { lo } else { zs[i-1] },if i == zs.len() { hi } else { zs[i] });
        }
        let shape = |r: &SweptRegion| {
            let (n,holes) = (r.loops.len(),r.loops.iter().filter(|l| l.area < 0.).count());
            let pieces: usize = r.loops.iter().map(|l| l.pieces.len()).sum();
            format!("{n} loop{} of {pieces} pieces{}",if n == 1 { "" } else { "s" },match holes { 0 => String::new(),1 => " (one a hole)".into(),
                h => format!(" ({h} of them holes)") })
        };
        let (a,b) = (shape(&first[j-1]),shape(&first[j]));
        let b = if a == b { format!("{b} of other stretches") } else { b };
        let names = &self.names;
        Err((Row::Slabs,format!("`{}`'s drafted tool `{}` sweeps a region whose shape changes within its slab, from {a} at height {:.4} \
            of `{}` to {b} at {:.4}: step the tool at about {:.4} (a cap there splits the slab), or draft it less",names.sweep,names.tool,
            at[j-1],names.tool,at[j],0.5*(lo+hi)),None))
    }

    /// What `found`'s regions were read from: their candidates and kept stretches, and the roll's
    /// steps.
    pub fn sampled(&self,found: &Found) -> Sampled {
        let regions = found.slabs.iter().flat_map(|s| s.stations.iter().map(|(_,r)| r));
        let (candidates,runs) = regions.fold((0,0),|(c,k),r| (c+r.read[0],k+r.read[1]));
        let steps = ((self.roll[1]-self.roll[0]).to_degrees()*PER_DEGREE).ceil() as usize;
        Sampled {candidates,runs,steps}
    }
}

/// The region's row as the class's.
fn row(r: super::region::Row) -> Row {
    match r { super::region::Row::Plane => Row::Plane,super::region::Row::Period => Row::Period,super::region::Row::Kept => Row::Kept }
}

/// How far past the blank's faces a cut through them reaches, as a share of the blank's height.
pub const PAST: f64 = 1e-3;

/// The pad about the blank's shadow a swept region is read within, as a share of the blank's size,
/// and what a drafted slab grows it by, in turn, until its stations' regions are alike (#612).
const PAD: f64 = 0.02;
const GROWN: [f64;3] = [1.,5.,25.];

/// How many stations a drafted slab is read at, its ends among them: every other one lofted through,
/// the rest holding the loft to the fit's bar (`region::lofted`).
pub const STATIONS: usize = 9;

/// `n` heights from a slab's lower end to its upper, both included.
pub fn stations(heights: [f64;2],n: usize) -> Vec<f64> {
    (0..n).map(|k| heights[0]+(heights[1]-heights[0])*k as f64/(n-1) as f64).collect()
}
