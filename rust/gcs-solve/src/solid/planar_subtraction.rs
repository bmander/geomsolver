//! The planar subtraction class (#606, #607): a sweep of prisms under a motion that turns their
//! plane in itself, cut from any blank. Within a slab of the blank that every member of the tool
//! spans or misses whole, the tool's section is the same at every height (or, drafted, moves as its
//! draft moves it), and so is what the sweep removes: the region `R = ∪ₜ poseₜ(section)` over the
//! roll (`envelope::swept_region`), and in the slab the cut is the blank less `R`'s prism. The rows:
//!
//! - P1: the motion turns the plane in itself about axes square to it, the same face up;
//! - Q1: the tool is prisms square to that plane, their profiles lines and arcs, joined by `union`
//!   and `common` (a drafted one read at stations through its slab, #607);
//! - Q2: every member's caps lie outside the blank, or on a plane the blank is split at — which
//!   the cap planes inside the blank are, so a band of a stepped tool splits the blank itself;
//! - Q3: the roll is a whole period of the motion (the tool stands where it began, as the
//!   observer sees it), or the tool is clear of the blank at both its ends;
//! - Q4: the region's boundary is regular: pieces meeting at corners or leaving the blank's
//!   shadow, no fold, no crossing (`SweptRegion::read`).
//!
//! T2, E2 and T1's refusal of a union are not asked: the region's keeping test does what they
//! guard against. A prism sweep the generating class admits (a rack) is built by its sheet; one it
//! refuses at a row this class does not ask is asked these instead (`admission::admit_body`).
//! Sampled where the other classes are: the region's candidates on a grid of the roll, Q3's
//! clearance on a grid of the section.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::admission::{Basis,Class,Condition,Equivalence,SweepEvidence};
use crate::envelope::Motion;
use crate::envelope::swept_region::{Plane,Section,Setting,SweptRegion,Tree};
use crate::model::{Sketch,SolidDef};
use crate::solid::surface::Edge;
use crate::space::{cross,dot,norm};

type V = [f64;3];
type P = [f64;2];
pub type Failure = (Condition,String,Option<V>);

/// How a member's face coordinates map into the plane's: `(a, b) ↦ o + a x + b y`, turned over
/// where the face looks the other way.
#[derive(Clone,Copy,Debug)]
struct Map { o: P,x: P,y: P }

impl Map {
    fn point(&self,p: P) -> P { [self.o[0]+p[0]*self.x[0]+p[1]*self.y[0],self.o[1]+p[0]*self.x[1]+p[1]*self.y[1]] }
    fn turned(&self) -> bool { self.x[0]*self.y[1]-self.x[1]*self.y[0] < 0. }
    /// An edge of the face in the plane's coordinates.
    fn edge(&self,e: &Edge) -> Edge {
        match *e {
            Edge::Line {a,b,axis} => Edge::Line {a:self.point(a),b:self.point(b),axis},
            Edge::Arc {center,radius,sweep,ends,..} => {
                let c = self.point(center);
                let s = self.point(e.at(0.));
                let start = (s[1]-c[1]).datan2(s[0]-c[0]);
                let sweep = if self.turned() { -sweep } else { sweep };
                Edge::Arc {center:c,radius,start,sweep,ends:ends.map(|q| self.point(q))}
            }
            Edge::Conic {a,m,b,w} => Edge::Conic {a:self.point(a),m:self.point(m),b:self.point(b),w},
        }
    }
}

/// One prism of the tool: its face, the face's loops in the plane's coordinates (each walked with
/// its material on its left), how far along the normal it reaches, and — drafted — its sections
/// and how a height is a depth of it.
#[derive(Clone,Debug)]
struct Member { face: usize,loops: Vec<Vec<Edge>>,span: [f64;2],drafted: Option<Drafted> }

#[derive(Clone,Debug)]
struct Drafted { sections: crate::solid::field::Sections,map: Map,at: f64,sign: f64 }

impl Member {
    /// Whether the member reaches height `h`.
    fn reaches(&self,h: f64) -> bool { h > self.span[0] && h < self.span[1] }

    /// The member's loops at height `h`, read where it reaches height `within` (a slab's middle:
    /// a drafted section is read at the slab's ends too, and a little past the blank's), or none.
    fn at(&self,h: f64,within: f64) -> Option<Vec<Vec<Edge>>> {
        if !self.reaches(within) { return None }
        let Some(d) = &self.drafted else { return Some(self.loops.clone()) };
        let loops = d.sections.section_at((h-d.at)*d.sign).ok()?;
        // a face turned over walks its loops the other way: walked back, its material on the left
        let mapped: Vec<Vec<Edge>> = loops.iter().map(|l| l.iter().map(|e| d.map.edge(e)).collect()).collect();
        crate::solid::field::oriented(&mapped).ok()
    }
}

/// What the class read of one sweep's tool: its plane, its members and how they combine, its motion
/// and its roll.
#[derive(Clone,Debug)]
pub struct Subtraction { plane: Plane,members: Vec<Member>,tree: Tree,family: crate::motion::Family,roll: [f64;2],name: String,
    tool: String,scale: f64 }

/// One slab of the blank: its heights along the plane's normal and the region the section there
/// sweeps — drafted, the region at each station through it (the first at its lower height).
#[derive(Clone,Debug)]
pub struct Slab { pub heights: [f64;2],pub stations: Vec<(f64,SweptRegion)> }

/// What the class found: the plane, how far along its normal the blank reaches (where a cut runs a
/// hair past it), and each slab a member reaches, lowest first.
#[derive(Clone,Debug)]
pub struct Found { pub plane: Plane,pub heights: [f64;2],pub slabs: Vec<Slab> }

/// Whether `swept`'s tool is prisms joined by union and common (Q1, structurally) under a motion
/// that keeps the first one's plane (P1, at a few rolls): the sweeps this class asks where the
/// generating class's rows would refuse them. Any other keeps the generating class's refusal.
pub fn asks(sk: &Sketch,swept: usize) -> bool {
    let SolidDef::Swept {source,motion,..} = &sk.solids[swept].def else { return false };
    let mut members = Vec::new();
    if tree(sk,*source as usize,&mut members).is_err() || members.is_empty() { return false }
    let SolidDef::Prism {face,..} = &sk.solids[members[0]].def else { return false };
    let Ok((basis,_)) = crate::solid::field::prism_loops(sk,*face as usize) else { return false };
    let n = basis.normal();
    let n = n.map(|x| x/norm(n));
    let (Ok(family),Ok([a,b])) = (crate::motion::Family::read(sk,*motion as usize),super::roll::interval(sk,swept)) else { return false };
    let size = 1.+norm(basis.o);
    (0..=4).all(|k| family.at(a+(b-a)*k as f64/4.).is_ok_and(|m| {
        let o = basis.o;
        dot(m.vector(n),n) > 1.-1e-9 && dot(crate::space::sub(m.point(o),o),n).abs() <= 1e-9*size && dot(m.velocity(o),n).abs() <= 1e-9*size
    }))
}

/// The tool's solid graph as a tree of prisms, each pushed to `members` (its id).
fn tree(sk: &Sketch,id: usize,members: &mut Vec<usize>) -> Result<Tree,String> {
    let solid = &sk.solids[id];
    match &solid.def {
        SolidDef::Prism {..} => { members.push(id); Ok(Tree::Member(members.len()-1)) }
        SolidDef::Body {stock,on,through,bound} => {
            if !through.is_empty() { return Err(format!("`{}` cuts solids; a planar tool's members are joined by union and common",
                solid.name)) }
            let mut united = vec![tree(sk,*stock as usize,members)?];
            for &o in on { united.push(tree(sk,o as usize,members)?) }
            let united = if united.len() == 1 { united.pop().expect("a stock") } else { Tree::Union(united) };
            if bound.is_empty() { return Ok(united) }
            let mut common = vec![united];
            for &b in bound { common.push(tree(sk,b as usize,members)?) }
            Ok(Tree::Common(common))
        }
        _ => Err(format!("`{}` is no prism: a planar tool's members are prisms",solid.name)),
    }
}

/// Heights within this of each other are one plane.
const SAME: f64 = 1e-9;

impl Subtraction {
    /// Read sweep `swept`'s tool for the class, refusing at Q1 by name and at P1 where the motion
    /// leaves the plane.
    pub fn read(sk: &Sketch,swept: usize) -> Result<Subtraction,Failure> {
        let SolidDef::Swept {source,motion,..} = &sk.solids[swept].def else {
            return Err((Condition::Members,"not a sweep".into(),None))
        };
        let tool = &sk.solids[*source as usize];
        let mut ids = Vec::new();
        let tree = tree(sk,*source as usize,&mut ids).map_err(|m| (Condition::Members,m,None))?;
        let q1 = |m: String| (Condition::Members,m,None);
        // the plane: the first member's face's, its axes made square
        let face_of = |id: usize| match &sk.solids[id].def { SolidDef::Prism {face,..} => *face as usize,_ => unreachable!("a member is a prism") };
        let (first,_) = crate::solid::field::prism_loops(sk,face_of(ids[0])).map_err(q1)?;
        let n = first.normal();
        let n = n.map(|x| x/norm(n));
        let u = first.u.map(|x| x/norm(first.u));
        let plane = Plane {o:first.o,u,v:cross(n,u),n};
        let mut members = Vec::new();
        for &id in &ids {
            let SolidDef::Prism {face,from,to,draft} = &sk.solids[id].def else { unreachable!("a member is a prism") };
            let name = sk.solids[id].name.clone();
            let (basis,loops) = crate::solid::field::prism_loops(sk,*face as usize).map_err(q1)?;
            if loops.iter().flatten().any(|e| matches!(e,Edge::Conic {..})) {
                return Err(q1(format!("`{name}`'s profile has an edge neither a line nor an arc")))
            }
            let m = basis.normal();
            let m = m.map(|x| x/norm(m));
            if norm(cross(m,n)) > 1e-9 || (norm(basis.u)-1.).abs() > 1e-9 || (norm(basis.v)-1.).abs() > 1e-9 || dot(basis.u,basis.v).abs() > 1e-9 {
                return Err(q1(format!("`{name}` stands on another plane than `{}`'s",sk.solids[ids[0]].name)))
            }
            let sign = dot(m,n);
            let o = plane.page(basis.o);
            let map = Map {o,x:[dot(basis.u,plane.u),dot(basis.u,plane.v)],y:[dot(basis.v,plane.u),dot(basis.v,plane.v)]};
            let at = plane.height(basis.o);
            let (a,b) = (at+sign*from.value,at+sign*to.value);
            let mapped: Vec<Vec<Edge>> = loops.iter().map(|l| l.iter().map(|e| map.edge(e)).collect()).collect();
            let oriented = crate::solid::field::oriented(&mapped).map_err(|e| q1(format!("`{name}`: {e}")))?;
            let drafted = match draft {
                Some(d) if d.value != 0. => Some(Drafted {sections:crate::solid::field::Sections::new(&loops,d.value,[from.value,to.value])
                    .map_err(|e| q1(format!("`{name}`: {e}")))?,map,at,sign}),
                _ => None,
            };
            members.push(Member {face:*face as usize,loops:oriented,span:[a.min(b),a.max(b)],drafted});
        }
        let family = crate::motion::Family::read(sk,*motion as usize).map_err(|e| (Condition::Plane,e,None))?;
        let roll = super::roll::interval(sk,swept).map_err(|m| (Condition::Closes,m,None))?;
        let scale = members.iter().flat_map(|m| m.loops.iter().flatten()).map(|e| match *e {
            Edge::Line {a,b,..} => a[0].dhypot(a[1]).max(b[0].dhypot(b[1])),
            Edge::Arc {center,radius,..} => center[0].dhypot(center[1])+radius,
            Edge::Conic {..} => 0.,
        }).fold(0.,f64::max);
        Ok(Subtraction {plane,members,tree,family,roll,name:sk.solids[swept].name.clone(),tool:tool.name.clone(),scale})
    }

    /// The section at height `h` of the slab whose middle is `within`.
    fn section(&self,h: f64,within: f64) -> Section {
        Section {members:self.members.iter().map(|m| m.at(h,within)).collect(),tree:self.tree.clone()}
    }

    fn setting(&self,h: f64,within: f64) -> Setting {
        Setting {section:self.section(h,within),family:self.family.clone(),plane:self.plane,roll:self.roll,scale:self.scale}
    }

    /// Q2–Q4 over every placement (`poses`, the sweep's frame in the body's), the blank read in the
    /// body by `inside` and boxed there by `bounds` (its least and greatest corner): the slabs the
    /// members' caps split the blank into, Q3 at the roll's ends, and each slab's region within a
    /// box about the blank's shadow at every placement.
    pub fn check(&self,poses: &[Motion],inside: &(dyn Fn(V) -> bool+Sync),bounds: Option<(V,V)>) -> Result<Found,Failure> {
        let Some((lo,hi)) = bounds else { return Err((Condition::Slabs,"the blank's extent is unknown".into(),None)) };
        let pl = self.plane;
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
        let pad = 0.02*size+1e-6*(1.+self.scale);
        let bounds: [P;2] = [[shadow[0][0]-pad,shadow[0][1]-pad],[shadow[1][0]+pad,shadow[1][1]+pad]];
        // Q2: the slabs, split at every cap within the blank; a slab no member reaches cuts nothing
        let mut planes: Vec<f64> = vec![heights[0],heights[1]];
        for m in &self.members { for h in m.span { if h > heights[0]+SAME && h < heights[1]-SAME { planes.push(h) } } }
        planes.sort_by(f64::total_cmp);
        planes.dedup_by(|a,b| (*a-*b).abs() <= SAME*(1.+self.scale));
        // each slab with the height its section is read at: its middle, or — merged with the one next
        // to it, whose section is its own (every member undrafted) — the first one's
        let mut slabs: Vec<([f64;2],f64)> = planes.windows(2).map(|w| ([w[0],w[1]],0.5*(w[0]+w[1]))).collect();
        let key = |h: f64| -> Option<String> {
            if self.members.iter().any(|m| m.drafted.is_some() && m.reaches(h)) { return None }
            Some(self.key(&self.tree,h).unwrap_or_default())
        };
        let mut merged: Vec<([f64;2],f64)> = Vec::new();
        for s in slabs.drain(..) {
            if let Some(last) = merged.last_mut() {
                let (a,b) = (key(last.1),key(s.1));
                if a.is_some() && a == b { last.0[1] = s.0[1]; continue }
            }
            merged.push(s);
        }
        // Q3: a whole period, or the tool clear of the blank at both ends of the roll
        let middle = 0.5*(heights[0]+heights[1]);
        let h = merged.iter().map(|s| s.1).find(|&h| !self.section(h,h).is_empty()).unwrap_or(middle);
        let probe = self.setting(h,h);
        let [a,b] = self.roll;
        let whole = probe.alike(a,b);
        if !whole {
            for &(_,h) in &merged {
                let setting = self.setting(h,h);
                if setting.section.is_empty() { continue }
                for t in self.roll {
                    for p in setting.covered(t,48) {
                        let w = pl.world(p,h);
                        if let Some(x) = poses.iter().map(|pose| pose.point(w)).find(|&x| inside(x)) {
                            let period = setting.period(a,8.).map_or(String::new(),|p| format!(": roll it a whole period, `over t in \
                                ({:.6}deg, {:.6}deg)`",a.to_degrees(),p.to_degrees()).replace(".000000deg","deg"));
                            return Err((Condition::Closes,format!("`{}`'s tool `{}` lies in the blank at roll {:.4} degrees{period}",self.name,
                                self.tool,t.to_degrees()),Some(x)))
                        }
                    }
                }
            }
        }
        // each slab's region, within the box; a drafted slab's at stations through it (#607), from a
        // little past the blank where the slab ends at its face
        let mut found = Vec::new();
        for (slab,h) in merged {
            if self.section(h,h).is_empty() { continue }
            let drafted = self.members.iter().any(|m| m.drafted.is_some() && m.reaches(h));
            let reach = [slab[0]-if slab[0] == heights[0] { PAST*(heights[1]-heights[0]) } else { 0. },
                slab[1]+if slab[1] == heights[1] { PAST*(heights[1]-heights[0]) } else { 0. }];
            let heights = slab;
            let at: Vec<f64> = if drafted { stations(reach,STATIONS) } else { vec![h] };
            let regions = crate::par::map(&at,|&z| SweptRegion::read(self.setting(z,h),bounds,whole)
                .map_err(|r| (r.row,r.message,r.witness.map(|p| poses[0].point(pl.world(p,z))))));
            let mut stations = Vec::new();
            for (z,r) in at.into_iter().zip(regions) { stations.push((z,r?)) }
            found.push(Slab {heights,stations});
        }
        if found.iter().all(|s| s.stations.iter().all(|(_,r)| r.loops.is_empty())) {
            return Err((Condition::Reach,format!("no part of the region `{}` sweeps reaches the blank",self.name),None))
        }
        Ok(Found {plane:pl,heights,slabs:found})
    }

    /// What the section at height `h` is made of, the same for two heights exactly where their
    /// sections are one: the faces of the members reaching it, as the tree joins them (each join's
    /// operands in order of their own words, a join of one its operand), none where it is empty.
    fn key(&self,t: &Tree,h: f64) -> Option<String> {
        let (ts,common) = match t {
            Tree::Member(m) => return self.members[*m].reaches(h).then(|| format!("{}",self.members[*m].face)),
            Tree::Union(ts) => (ts,false),
            Tree::Common(ts) => (ts,true),
        };
        let mut parts: Vec<String> = Vec::new();
        for t in ts {
            match self.key(t,h) {
                Some(k) => parts.push(k),
                // what is held in common with nothing is nothing
                None if common => return None,
                None => {}
            }
        }
        parts.sort(); parts.dedup();
        match parts.len() { 0 => None,1 => parts.pop(),_ => Some(format!("{}({})",if common { "C" } else { "U" },parts.join(","))) }
    }

    /// The sweep's evidence, the slabs its finding, sampled as `found` says.
    pub fn evidence(&self,found: Found) -> SweepEvidence {
        let regions = found.slabs.iter().flat_map(|s| s.stations.iter().map(|(_,r)| r));
        let (candidates,runs) = regions.fold((0,0),|(c,k),r| (c+r.read[0],k+r.read[1]));
        let steps = ((self.roll[1]-self.roll[0]).to_degrees()*4.).ceil() as usize;
        SweepEvidence {sweep:0,name:String::new(),placements:Vec::new(),samples:candidates,contacts:runs,spacing:0.25_f64.to_radians(),
            least_area_factor:1.,near_double_roots:0,near_tangent_pairs:0,basis:Basis::Sampled {rows:candidates,columns:steps},
            equivalence:Equivalence::Sampled,class:Class::Subtraction(Box::new(found))}
    }
}

/// How far past the blank's faces a cut through them reaches, as a share of the blank's height.
pub const PAST: f64 = 1e-3;

/// How many stations a drafted slab is read at, its ends among them: every other one lofted through,
/// the rest holding the loft to the fit's bar (`brep::sweep::subtraction`).
pub const STATIONS: usize = 9;

/// `n` heights from a slab's lower end to its upper, both included.
pub fn stations(heights: [f64;2],n: usize) -> Vec<f64> {
    (0..n).map(|k| heights[0]+(heights[1]-heights[0])*k as f64/(n-1) as f64).collect()
}
