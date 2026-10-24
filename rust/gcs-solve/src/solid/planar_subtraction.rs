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

/// One prism of the tool: its name and face, the face's loops in the plane's coordinates (each
/// walked with its material on its left), how far along the normal it reaches, and — drafted — its
/// sections and how a height is a depth of it.
#[derive(Clone,Debug)]
struct Member { name: String,face: usize,loops: Vec<Vec<Edge>>,span: [f64;2],drafted: Option<Drafted> }

/// A drafted member's sections (read in the plane's coordinates) and how a height is a depth of it:
/// the height of its face, and which way along the normal its depth runs.
#[derive(Clone,Debug)]
struct Drafted { sections: crate::solid::field::Sections,at: f64,sign: f64 }

impl Member {
    /// Whether the member reaches height `h`.
    fn reaches(&self,h: f64) -> bool { h > self.span[0] && h < self.span[1] }

    /// The member's loops at height `h`, read where it reaches height `within` (a slab's middle:
    /// a drafted section is read at the slab's ends too, and a little past the blank's), or none.
    fn at(&self,h: f64,within: f64) -> Option<Vec<Vec<Edge>>> {
        if !self.reaches(within) { return None }
        match &self.drafted { Some(d) => d.sections.section_at((h-d.at)*d.sign).ok(),None => Some(self.loops.clone()) }
    }
}

/// What the class read of one sweep's tool: its plane, its members and how they combine, its motion
/// and its roll.
#[derive(Clone,Debug)]
pub struct Subtraction { plane: Plane,members: Vec<Member>,tree: Tree,family: crate::motion::Family,roll: [f64;2],name: String,
    tool: String,scale: f64 }

/// One slab of the blank: its heights along the plane's normal, how far its cut reaches (a hair
/// past the blank where the slab ends at its face) and the region the section there sweeps —
/// drafted, the region at each station through it, the first at the lower end of its reach.
#[derive(Clone,Debug)]
pub struct Slab { pub heights: [f64;2],pub reach: [f64;2],pub stations: Vec<(f64,SweptRegion)> }

/// What the class found: the plane and each slab a member reaches, lowest first.
#[derive(Clone,Debug)]
pub struct Found { pub plane: Plane,pub slabs: Vec<Slab> }

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
    (0..=4).all(|k| family.at(a+(b-a)*k as f64/4.).is_ok_and(|m| crate::envelope::swept_region::keeps(&m,basis.o,n,1.+norm(basis.o))))
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
                Some(d) if d.value != 0. => Some(Drafted {sections:crate::solid::field::Sections::new(&mapped,d.value,[from.value,to.value])
                    .map_err(|e| q1(format!("`{name}`: {e}")))?,at,sign}),
                _ => None,
            };
            members.push(Member {name,face:*face as usize,loops:oriented,span:[a.min(b),a.max(b)],drafted});
        }
        let family = crate::motion::Family::read(sk,*motion as usize).map_err(|e| (Condition::Plane,e,None))?;
        let roll = super::roll::interval(sk,swept).map_err(|m| (Condition::Closes,m,None))?;
        let scale = members.iter().flat_map(|m| m.loops.iter().flatten()).map(Edge::reach).fold(0.,f64::max);
        Ok(Subtraction {plane,members,tree,family,roll,name:sk.solids[swept].name.clone(),tool:tool.name.clone(),scale})
    }

    /// What the swept region at height `h` of the slab whose middle is `within` is read from: the
    /// section there, the motion, the plane and the roll.
    fn setting(&self,h: f64,within: f64) -> Result<Setting,Failure> {
        let section = Section::new(self.members.iter().map(|m| m.at(h,within)).collect(),&self.tree)
            .map_err(|e| (Condition::Members,format!("`{}`'s section: {e}",self.tool),None))?;
        Ok(Setting {section,family:self.family.clone(),plane:self.plane,roll:self.roll,scale:self.scale})
    }

    /// Whether any member reaches height `h`.
    fn reached(&self,h: f64) -> bool { self.members.iter().any(|m| m.reaches(h)) }

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
        let boxed = |grown: f64| -> [P;2] {
            let pad = PAD*grown*size+1e-6*(1.+self.scale);
            [[shadow[0][0]-pad,shadow[0][1]-pad],[shadow[1][0]+pad,shadow[1][1]+pad]]
        };
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
        // each slab a member reaches: Q3, a whole period of its section or its tool clear of the blank
        // at both ends of the roll; then its region within the box, a drafted slab's at stations
        // through it (#607), from a little past the blank where the slab ends at its face
        let past = PAST*(heights[1]-heights[0]);
        let mut found = Vec::new();
        for (slab,h) in merged {
            if !self.reached(h) { continue }
            let setting = self.setting(h,h)?;
            let [a,b] = self.roll;
            let whole = setting.alike(a,b);
            if !whole {
                let covered: Vec<V> = self.roll.iter().flat_map(|&t| setting.covered(t,48)).map(|p| pl.world(p,h)).collect();
                let hit = crate::par::map(&covered,|&w| poses.iter().map(|pose| pose.point(w)).find(|&x| inside(x)));
                if let Some((k,x)) = hit.into_iter().enumerate().find_map(|(k,x)| x.map(|x| (k,x))) {
                    let t = if k < covered.len()/2 { a } else { b };
                    let period = setting.period(a).map_or(String::new(),|p| format!(": {}",crate::solid::roll::advice(a,p)));
                    return Err((Condition::Closes,format!("`{}`'s tool `{}` lies in the blank at roll {:.4} degrees{period}",self.name,
                        self.tool,t.to_degrees()),Some(x)))
                }
            }
            let drafted = self.members.iter().any(|m| m.drafted.is_some() && m.reaches(h));
            let reach = [slab[0]-if slab[0] == heights[0] { past } else { 0. },slab[1]+if slab[1] == heights[1] { past } else { 0. }];
            let read = |z: f64,bounds: [P;2]| SweptRegion::read(if drafted { self.setting(z,h)? } else { setting.clone() },bounds,whole)
                .map_err(|r| (r.row,r.message,r.witness.map(|p| poses[0].point(pl.world(p,z)))));
            let stations = if drafted { self.drafted(reach,h,&read,&boxed)? } else { vec![(h,read(h,boxed(1.))?)] };
            found.push(Slab {heights:slab,reach,stations});
        }
        if found.iter().all(|s| s.stations.iter().all(|(_,r)| r.loops.is_empty())) {
            return Err((Condition::Reach,format!("no part of the region `{}` sweeps reaches the blank",self.name),None))
        }
        Ok(Found {plane:pl,slabs:found})
    }

    /// A drafted slab's stations through `reach` (`h` its middle), each region read by `read`
    /// within the box `boxed` grows by each of `GROWN` in turn, until every station's loops are
    /// the first's piece for piece: where the region is cut by the box's sides is no part of the
    /// blank, so a change of shape only there is the box's (#612). A change that holds in every box
    /// is refused at the depth of the drafted member it happens at, found between the stations.
    fn drafted(&self,reach: [f64;2],h: f64,read: &(dyn Fn(f64,[P;2]) -> Result<SweptRegion,Failure>+Sync),boxed: &dyn Fn(f64) -> [P;2])
        -> Result<Vec<(f64,SweptRegion)>,Failure> {
        let at = stations(reach,STATIONS);
        let within = |bounds: [P;2]| crate::par::map(&at,|&z| read(z,bounds)).into_iter().collect::<Result<Vec<_>,_>>();
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
            let marks = crate::par::map(&zs,|&z| unlike(z));
            let i = marks.iter().position(|&m| m).unwrap_or(zs.len());
            (lo,hi) = (if i == 0 { lo } else { zs[i-1] },if i == zs.len() { hi } else { zs[i] });
        }
        let (member,d) = self.members.iter().find_map(|m| m.drafted.as_ref().filter(|_| m.reaches(h)).map(|d| (m,d)))
            .expect("a drafted member");
        let depth = |z: f64| (z-d.at)*d.sign;
        let shape = |r: &SweptRegion| {
            let (n,holes) = (r.loops.len(),r.loops.iter().filter(|l| l.area < 0.).count());
            let pieces: usize = r.loops.iter().map(|l| l.pieces.len()).sum();
            format!("{n} loop{} of {pieces} pieces{}",if n == 1 { "" } else { "s" },match holes { 0 => String::new(),1 => " (one a hole)".into(),
                h => format!(" ({h} of them holes)") })
        };
        let (a,b) = (shape(&first[j-1]),shape(&first[j]));
        let b = if a == b { format!("{b} of other stretches") } else { b };
        Err((Condition::Slabs,format!("`{}`'s drafted tool `{}` sweeps a region whose shape changes within its slab, from {a} at depth {:.4} \
            of `{}` to {b} at {:.4}: step the tool at about {:.4} (a cap there splits the slab), or draft it less",self.name,self.tool,
            depth(at[j-1]),member.name,depth(at[j]),depth(0.5*(lo+hi))),None))
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

    /// Sweep `swept`'s evidence over `poses`, every one checked, the slabs its finding, sampled as
    /// `found` says.
    fn evidence(&self,swept: usize,poses: &[Motion],found: Found) -> SweepEvidence {
        let regions = found.slabs.iter().flat_map(|s| s.stations.iter().map(|(_,r)| r));
        let (candidates,runs) = regions.fold((0,0),|(c,k),r| (c+r.read[0],k+r.read[1]));
        let per = crate::envelope::swept_region::PER_DEGREE;
        let steps = ((self.roll[1]-self.roll[0]).to_degrees()*per).ceil() as usize;
        let placements = poses.iter().map(|&pose| super::admission::Placement {pose,equivalent_to:None}).collect();
        SweepEvidence {sweep:swept,name:self.name.clone(),placements,samples:candidates,contacts:runs,spacing:(1./per).to_radians(),
            least_area_factor:1.,near_double_roots:0,near_tangent_pairs:0,basis:Basis::Sampled {rows:candidates,columns:steps},
            equivalence:Equivalence::Sampled,class:Class::Subtraction(Box::new(found))}
    }
}

/// Sweep `swept`, cut from a blank at `poses` (the sweep's frame in the body's), the blank read by
/// `inside` and boxed by `bounds`, admitted to the class (rows P1, Q1–Q4) with its evidence, or
/// refused by row: what admission asks of it, and what the accuracy meter reads.
pub fn admit(sk: &Sketch,swept: usize,poses: &[Motion],inside: &(dyn Fn(V) -> bool+Sync),bounds: Option<(V,V)>)
    -> Result<SweepEvidence,Failure> {
    let class = Subtraction::read(sk,swept)?;
    let found = class.check(poses,inside,bounds)?;
    Ok(class.evidence(swept,poses,found))
}

/// How far past the blank's faces a cut through them reaches, as a share of the blank's height.
pub const PAST: f64 = 1e-3;

/// The pad about the blank's shadow a swept region is read within, as a share of the blank's size,
/// and what a drafted slab grows it by, in turn, until its stations' regions are alike (#612).
const PAD: f64 = 0.02;
const GROWN: [f64;3] = [1.,5.,25.];

/// How many stations a drafted slab is read at, its ends among them: every other one lofted through,
/// the rest holding the loft to the fit's bar (`brep::sweep::subtraction`).
pub const STATIONS: usize = 9;

/// `n` heights from a slab's lower end to its upper, both included.
pub fn stations(heights: [f64;2],n: usize) -> Vec<f64> {
    (0..n).map(|k| heights[0]+(heights[1]-heights[0])*k as f64/(n-1) as f64).collect()
}
