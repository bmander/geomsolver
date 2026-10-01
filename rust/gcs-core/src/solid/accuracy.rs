//! How far an exported boundary lies from the exact one: each sample point's distance to a
//! body's exact surface, read two independent ways (docs/native-hypoid-plan.md).
//!
//! **The analytic route** knows what the surface is made of. A blank face is a turned line or
//! arc, and its distance is the exact meridian projection (`SurfaceProjector`). A generated face
//! is the envelope of a cutter face under the generating roll, at one tooth index; its nearest
//! point is where the sample's foot on the rolled cutter is also a contact. The foot of a point
//! `q` on the cutter posed at roll `t` is its meridian projection `f(t)`, at signed support
//! distance `s(t)` along the cutter normal `n(t)`, and the envelope point is the roll where
//! `g(t) = n · v(f)`, the normal velocity of the material point at the foot, vanishes: there
//! `q - f = s n` with `n` the envelope's own normal, so `f` is a critical point of the distance
//! to the envelope (and `t` one of `s`). The roll is scanned and each sign change of `g` refined
//! by false position; the foot must lie on the finite cutter face (its meridian parameter in
//! [0,1]), on the cutter's own boundary where the cutter is a Boolean, and within the blank. A
//! blank face's foot must lie on the blank's boundary and outside every cut (the nearest cutter
//! envelope reads it on the material's side). Where a point's foot on each of two faces lies off
//! the other — a point standing off a convex edge — the nearest point is on the edge the faces
//! share, found by alternating projections onto the two. Tooth indices are pruned by a table of
//! the cells the unindexed envelope passes through.
//!
//! **The field route** is `MaterialField::reading`: the body's one-Lipschitz field, exact below
//! its cap, which near a smooth face is the signed distance to it. Off a convex edge it is only
//! a lower bound (the greater of the two faces' values), and so it is well inside a cut, where it
//! is the deepest a single pose of the cutter reaches. The two routes share the solved snapshot
//! and nothing else: no projection, root or sweep search of one is read by the other.
//!
//! **A body with no swept cut** is read through the Rust kernel's exact B-rep of its CAD recipe
//! (`Exact`): the foot on each face where it lies within the face, else the nearest point of an
//! edge, signed by the B-rep's own classification of the point. A body the kernel refuses by name
//! falls back to its turned faces; a loft is not metered, its material field refusing it. That B-rep holds a stretch of a
//! traced or formula curve as the B-spline the recipe fits within `cad::FIT_MM` of it; the field
//! route reads the curve itself (as chords within `CHORD_SLACK` of the profile's reach).
//!
//! What this does not claim: a generated face is not trimmed by the other cuts (another index,
//! another stretch of the roll), so a sample where two cuts meet may be measured to one's
//! continuation inside the other, which reads short. The search is sampled (the roll's scan,
//! the tabulated cells) and says so: a sample farther than `reach` from every face is unmatched.
//! Signs follow the field's: positive outside the material.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::{cad,RevolvedSurface,SpatialField,SurfaceProjector,SweepContacts,MaterialField};
use crate::envelope::Motion;
use crate::model::{EntKind,Sketch,SolidDef};
use crate::motion::Family;
use crate::space::{add,sub,scale,dot,norm,distance,normalised,triangle_normal};
use std::collections::BTreeSet;

type V = [f64;3];

/// How the analytic route searches, in model units.
#[derive(Clone,Copy,Debug)]
pub struct Options {
    /// The farthest a candidate foot is taken: a sample farther from every face is unmatched.
    pub reach: f64,
    /// How far off its trims (the blank, the cutter's own Boolean) a foot may stand.
    pub trim: f64,
    /// Intervals the roll of every sweep is scanned in before each sign change is refined.
    pub roll_steps: usize,
    /// The least side of the cells the unindexed envelope is tabulated in.
    pub cell: f64,
}

impl Options {
    /// The defaults for a model whose length unit is `millimetres` millimetres: two
    /// millimetres of reach, a tenth of a micrometre of trim (a foot farther off its face is
    /// followed to the edge), millimetre cells, a roll in 64 steps.
    pub fn in_units(millimetres: f64) -> Self {
        let mm = 1./millimetres;
        Self {reach:2.*mm,trim:1e-4*mm,roll_steps:64,cell:mm}
    }
}

impl Default for Options { fn default() -> Self { Self::in_units(1.) } }

/// One exact face the body's surface is made of: a turned face of its blank, or a cutter face
/// whose envelope a sweep generates (at every tooth index at once).
#[derive(Clone,Debug)]
pub struct Surface { pub name: String, pub generated: bool }

/// The nearest exact face to a sample, by the analytic route.
#[derive(Clone,Copy,Debug)]
pub struct Nearest {
    /// Signed distance to the face: positive outside the material.
    pub distance: f64,
    /// Which of `Meter::surfaces`.
    pub surface: usize,
    pub foot: V,
    /// The body's outward unit normal at the foot.
    pub normal: V,
    /// For a generated face, which placement of its sweep (the tooth index) and the roll.
    pub placement: Option<usize>,
    pub roll: Option<f64>,
}

/// A sample's two readings: the analytic route's nearest face, and the material field's value.
#[derive(Clone,Copy,Debug)]
pub struct Measurement { pub analytic: Option<Nearest>, pub field: f64 }

#[derive(Clone,Copy,Debug,PartialEq,Eq,PartialOrd,Ord)]
pub enum SampleKind { Face, Vertex, Midpoint, Centroid }

impl SampleKind {
    pub fn name(self) -> &'static str {
        match self { SampleKind::Face => "face points",SampleKind::Vertex => "vertices",
            SampleKind::Midpoint => "edge midpoints",SampleKind::Centroid => "centroids" }
    }
    /// Whether the sample's normal is its own face's there: a face point's and a triangle's
    /// centroid's, not a vertex's or an edge midpoint's, which may stand on another face's edge.
    fn own_normal(self) -> bool { matches!(self,SampleKind::Face | SampleKind::Centroid) }
}

/// A point of an exported boundary: where it is, the exporter's normal if it has one, and which
/// of its faces (and which class of face) it lies on, numbered as the caller likes.
#[derive(Clone,Copy,Debug)]
pub struct Sample { pub position: V, pub normal: Option<V>, pub face: u32, pub class: u32, pub kind: SampleKind }

#[derive(Clone,Debug)]
struct Face {
    surface: usize,
    projector: SurfaceProjector,
    v: [f64;2],
    /// +1 where the projector's normal points out of the material, -1 where it points in.
    side: f64,
}

#[derive(Clone,Debug)]
struct Sweep {
    contacts: SweepContacts,
    faces: Vec<Face>,
    /// Each placement's pose and its inverse.
    placements: Vec<(Motion,Motion)>,
    cells: BTreeSet<[i64;3]>,
    cell: f64,
}

/// A body with no swept cut, as the Rust kernel's exact B-rep of its CAD recipe (millimetres,
/// `mm` of them a model unit).
#[derive(Clone,Debug)]
struct Exact { located: crate::brep::query::Located<'static>,mm: f64 }

impl Exact {
    fn read(sk: &Sketch,body: usize) -> Result<Self,String> {
        let b = crate::brep::recipe::build(&cad::recipe(sk,body)?)?;
        let tol = 1e-9*b.size().max(1.);
        Ok(Self {located:crate::brep::query::Located::owned(b,tol),mm:cad::millimetres(sk)?})
    }
    fn names(&self) -> Vec<Surface> {
        self.located.b.faces.iter().enumerate().map(|(i,f)| Surface {generated:false,
            name:if f.name.is_empty() { format!("{} {i}",f.surface.kind()) } else { f.name.clone() }}).collect()
    }
    /// The nearest point of the boundary: a face's foot within its trims, or an edge's.
    fn nearest(&self,p: V) -> Option<Nearest> {
        use crate::brep::{query::Place,topo::EdgeCurve};
        let b = &self.located.b;
        let q = p.map(|x| x*self.mm);
        let mut best: Option<(f64,usize,V,V)> = None;
        let mut keep = |d: f64,face: usize,foot: V,normal: V| if best.is_none_or(|b| d < b.0) { best = Some((d,face,foot,normal)) };
        for (fi,f) in b.faces.iter().enumerate() {
            let uv = f.surface.inverse(q);
            let foot = f.surface.point(uv);
            if self.located.face_place(fi,foot) == Place::Out { continue }
            let Some(n) = f.surface.normal(uv) else { continue };
            let n = if f.reversed { scale(n,-1.) } else { n };
            keep(distance(q,foot),fi,foot,n);
        }
        for (ei,e) in b.edges.iter().enumerate() {
            let EdgeCurve::Curve(c) = &e.curve else { continue };
            let mut t = c.inverse(q);
            if let Some(period) = c.period() { while t < e.t[0] { t += period; } }
            let foot = c.point(t.clamp(e.t[0],e.t[1]));
            let Some(face) = b.faces.iter().position(|f| f.loops.iter().flatten().any(|u| u.edge as usize == ei)) else { continue };
            let d = distance(q,foot);
            keep(d,face,foot,crate::space::normalised(sub(q,foot)).unwrap_or([0.;3]));
        }
        let (d,face,foot,normal) = best?;
        // signed as the field is: positive outside, by the boundary's own ray classification
        let outside = match self.located.solid_place(q) { Place::Out => true,Place::In => false,Place::On => return Some(Nearest {
            distance:0.,surface:face,foot:foot.map(|x| x/self.mm),normal,placement:None,roll:None}) };
        let normal = if outside == (dot(normal,sub(q,foot)) >= 0.) { normal } else { scale(normal,-1.) };
        Some(Nearest {distance:if outside { d } else { -d }/self.mm,surface:face,foot:foot.map(|x| x/self.mm),normal,placement:None,roll:None})
    }
}

/// The two routes over one body's solved snapshot. Re-read after the model changes.
#[derive(Clone,Debug)]
pub struct Meter {
    exact: Option<Exact>,
    surfaces: Vec<Surface>,
    blank: SpatialField,
    faces: Vec<Face>,
    sweeps: Vec<Sweep>,
    field: MaterialField,
    options: Options,
}

const PARAMETER_SLACK: f64 = 1e-9;

impl Face {
    /// Whether a projection's parameters lie on the finite face, to `PARAMETER_SLACK`.
    fn covers(&self,[u,v]: [f64;2]) -> bool {
        (-PARAMETER_SLACK..=1.+PARAMETER_SLACK).contains(&u) && v >= self.v[0]-PARAMETER_SLACK && v <= self.v[1]+PARAMETER_SLACK
    }
}

fn surface_face(surface: &RevolvedSurface,index: usize) -> Result<Face,String> {
    Ok(Face {surface:index,projector:surface.projector()?,v:surface.domain()[1],side:1.})
}

/// A diameter on the axis disappears in a full revolution (`SweepContacts::read`'s rule).
fn on_axis(surface: &RevolvedSurface) -> Result<bool,String> {
    let radius = |u| surface.at(u,surface.domain()[1][0]).map(|p| norm(p.dv)/surface.sweep().abs())
        .map_err(|e| format!("{e:?}"));
    Ok(radius(0.)? <= cad::AXIS_TOLERANCE && radius(1.)? <= cad::AXIS_TOLERANCE)
}

/// Every turned face of a static solid, placed.
fn turned_faces(sk: &Sketch,id: usize,pose: Motion,out: &mut Vec<RevolvedSurface>) -> Result<(),String> {
    let solid = &sk.solids[id];
    match &solid.def {
        SolidDef::Revolve {face,..} => for (edges,_) in sk.faces[*face as usize].boundaries() {
            for &edge in edges {
                let surface = RevolvedSurface::read(sk,id,edge)?.placed(pose);
                if edge.kind == EntKind::Line && on_axis(&surface)? { continue; }
                out.push(surface);
            }
        },
        SolidDef::Placed {source,motion,at} => turned_faces(sk,*source as usize,
            Family::read(sk,*motion as usize)?.at(at.value)?.then(pose),out)?,
        SolidDef::Body {..} => for o in solid.operands() { turned_faces(sk,o as usize,pose,out)? },
        _ => return Err(format!("`{}`: the accuracy meter reads turned blanks only",solid.name)),
    }
    Ok(())
}

fn field_error(e: crate::interval::Error) -> String { format!("{e:?}") }

/// Which way a blank face's projector normal points, read at the first point of a grid over the
/// face that lies on the blank's boundary with the blank's field changing sign across it there.
fn blank_side(blank: &SpatialField,surface: &RevolvedSurface,projector: &SurfaceProjector) -> Option<f64> {
    let [_,[v0,v1]] = surface.domain();
    for i in 0..=16 { for j in 0..=32 {
        let Ok(x) = surface.at(i as f64/16.,v0+(v1-v0)*j as f64/32.).map(|s| s.position) else { continue };
        let size = 1.+norm(x);
        if blank.value(x).abs() > 1e-9*size { continue; }
        let Ok(pr) = projector.project(x) else { continue };
        let h = 1e-6*size;
        let (up,down) = (blank.value(add(x,scale(pr.normal,h))),blank.value(sub(x,scale(pr.normal,h))));
        if up > 0. && down < 0. { return Some(1.); }
        if up < 0. && down > 0. { return Some(-1.); }
    } }
    None
}

impl Meter {
    /// Read `body` from the solved sketch: its static remainder (the blank) and the swept cuts
    /// subtracted from it, each at every placement.
    pub fn read(sk: &Sketch,body: usize,options: Options) -> Result<Self,String> {
        super::validate(sk,body)?;
        let tol = cad::AXIS_TOLERANCE;
        let read = |id: u32| -> Result<SpatialField,String> {
            if cad::contains_sweep(sk,id as usize) {
                return Err(format!("`{}`: the accuracy meter reads sweeps only as a body's cuts",sk.solids[id as usize].name));
            }
            SpatialField::read(sk,id as usize,tol)
        };
        let mut cuts = Vec::new();
        let mut statics = Vec::new();
        let blank = match &sk.solids[body].def {
            SolidDef::Body {stock,on,through,bound} => {
                let mut field = read(*stock)?;
                statics.push(*stock);
                for &o in on { field = field.union(read(o)?).map_err(field_error)?; statics.push(o); }
                for &b in bound { field = field.intersection(read(b)?).map_err(field_error)?; statics.push(b); }
                for &c in through {
                    match cad::swept_cut(sk,c as usize)? {
                        Some(cut) => cuts.push(cut),
                        None => { field = field.difference(read(c)?).map_err(field_error)?; statics.push(c); }
                    }
                }
                field
            }
            _ => { statics.push(body as u32); read(body as u32)? }
        };
        // a body with no swept cut: its exact B-rep is the analytic route, where the kernel builds
        // it (one it refuses by name is read through its turned faces, as before)
        if cuts.is_empty() {
            if let Ok(exact) = Exact::read(sk,body) {
                let field = MaterialField::read(sk,body,tol)?;
                return Ok(Self {surfaces:exact.names(),exact:Some(exact),blank,faces:Vec::new(),sweeps:Vec::new(),field,options})
            }
        }
        let mut surfaces = Vec::new();
        let mut faces = Vec::new();
        let mut turned = Vec::new();
        for id in statics { turned_faces(sk,id as usize,Motion::identity(),&mut turned)?; }
        for s in &turned {
            // a face's side, read where the face is on the blank's boundary; a face nowhere on it
            // is no face of the body
            let mut face = surface_face(s,surfaces.len())?;
            let Some(side) = blank_side(&blank,s,&face.projector) else { continue };
            face.side = side;
            faces.push(face);
            surfaces.push(Surface {name:s.name.clone(),generated:false});
        }
        let mut sweeps: Vec<(usize,Sweep)> = Vec::new();
        for cut in cuts {
            if let Some((_,s)) = sweeps.iter_mut().find(|(id,_)| *id == cut.swept) {
                s.placements.push((cut.pose,cut.pose.inverse()));
                continue;
            }
            let contacts = SweepContacts::read(sk,cut.swept,tol)?;
            let mut faces = Vec::new();
            for patch in contacts.patches() {
                let index = surfaces.iter().position(|s: &Surface| s.generated && s.name == patch.name)
                    .unwrap_or_else(|| { surfaces.push(Surface {name:patch.name.clone(),generated:true}); surfaces.len()-1 });
                faces.push(surface_face(patch,index)?);
            }
            sweeps.push((cut.swept,Sweep {contacts,faces,placements:vec![(cut.pose,cut.pose.inverse())],
                cells:BTreeSet::new(),cell:options.cell}));
        }
        let mut sweeps: Vec<Sweep> = sweeps.into_iter().map(|(_,s)| s).collect();
        let sphere = blank.support_bounds().map_err(field_error)?.map(|b| crate::space::box_centre_diagonal(&b));
        for s in &mut sweeps { s.tabulate(sphere,&options)?; }
        let field = MaterialField::read(sk,body,tol)?;
        Ok(Self {exact:None,surfaces,blank,faces,sweeps,field,options})
    }

    pub fn surfaces(&self) -> &[Surface] { &self.surfaces }

    /// Both readings of one point.
    pub fn measure(&self,p: V) -> Measurement { Measurement {analytic:self.nearest(p),field:self.field(p)} }

    /// The blank's own field at a point (its static remainder, no cut subtracted): negative
    /// inside it, and a lower bound on the distance to its boundary.
    pub fn blank(&self,p: V) -> f64 { self.blank.value(p) }

    /// The material field's value at a point: the field route.
    pub fn field(&self,p: V) -> f64 { self.field.reading(p).value }

    /// The nearest exact face within reach, by the analytic route: the nearest foot on a face
    /// within its trims, or — where a point stands off a convex edge, so that its foot on each
    /// face lies beyond the other — the nearest point of the edge the two faces share.
    pub fn nearest(&self,p: V) -> Option<Nearest> {
        if let Some(exact) = &self.exact { return exact.nearest(p).filter(|n| n.distance.abs() <= self.options.reach) }
        let mut candidates = Vec::new();
        for (i,face) in self.faces.iter().enumerate() {
            if let Some(mut c) = self.blank_face(i,face,p) {
                // a blank face is on the body only where no cut removes it
                if !c.trimmed && self.removed(c.near.foot) { c.trimmed = true; }
                candidates.push(c);
            }
        }
        self.generated(p,&mut candidates);
        let reach = self.options.reach;
        let mut best: Option<Nearest> = candidates.iter().filter(|c| !c.trimmed).map(|c| c.near)
            .min_by(|a,b| a.distance.abs().total_cmp(&b.distance.abs()));
        let bound = best.map_or(reach,|b| b.distance.abs());
        let mut trimmed: Vec<&Candidate> = candidates.iter().filter(|c| c.trimmed && c.near.distance.abs() < bound).collect();
        trimmed.sort_by(|a,b| a.near.distance.abs().total_cmp(&b.near.distance.abs()));
        trimmed.truncate(4);
        for (i,a) in trimmed.iter().enumerate() {
            for b in &trimmed[i+1..] {
                let Some(edge) = self.edge(p,a,b) else { continue };
                if best.is_none_or(|n| edge.distance.abs() < n.distance.abs()) { best = Some(edge); }
            }
        }
        best.filter(|b| b.distance.abs() <= reach)
    }

    /// Whether a point of the blank lies inside a cut: the nearest generated face reads it outside
    /// the material. Whether that face lies within the blank is not asked: the cut's boundary
    /// continues past the blank's, and what is asked is which side of the cut the point is on.
    fn removed(&self,p: V) -> bool {
        let mut found = Vec::new();
        self.generated(p,&mut found);
        found.iter().min_by(|a,b| a.near.distance.abs().total_cmp(&b.near.distance.abs()))
            .is_some_and(|c| c.near.distance > self.options.trim)
    }

    /// The point where two faces meet nearest `p`, by alternating projections onto them, if it
    /// lies on the blank's boundary; its distance is signed as the faces' are.
    fn edge(&self,p: V,a: &Candidate,b: &Candidate) -> Option<Nearest> {
        let size = 1.+norm(p);
        let (mut x,mut ta,mut tb) = (p,a.near.roll,b.near.roll);
        let mut converged = false;
        for _ in 0..200 {
            let (xa,ra) = self.project(&a.piece,x,ta)?;
            let (xb,rb) = self.project(&b.piece,xa,tb)?;
            (ta,tb) = (ra,rb);
            let moved = distance(xb,x);
            x = xb;
            if moved <= 1e-12*size { converged = true; break; }
        }
        if !converged || self.blank.value(x).abs() > self.options.trim { return None; }
        let d = distance(p,x);
        if d > self.options.reach { return None; }
        let side = if a.near.distance.max(b.near.distance) > 0. { 1. } else { -1. };
        let normal = normalised(sub(p,x)).map_or(a.near.normal,|n| scale(n,side));
        let (near,piece) = if a.near.distance.abs() >= b.near.distance.abs() { (a.near,&a.piece) } else { (b.near,&b.piece) };
        let placement = match piece { Piece::Generated {placement,..} => Some(*placement),Piece::Face(_) => None };
        Some(Nearest {distance:side*d,surface:near.surface,foot:x,normal,placement,
            roll:if placement.is_some() { near.roll } else { None }})
    }

    /// The foot of `x` on a face's support: a blank face's meridian projection, or the nearest
    /// envelope point of a cutter face, its roll found from `roll` outwards. The roll found.
    fn project(&self,piece: &Piece,x: V,roll: Option<f64>) -> Option<(V,Option<f64>)> {
        match *piece {
            Piece::Face(i) => {
                let pr = self.faces[i].projector.project(x).ok()?;
                Some((sub(x,scale(pr.normal,pr.signed_residual)),None))
            }
            Piece::Generated {sweep,placement,face} => {
                let s = &self.sweeps[sweep];
                let (pose,inverse) = s.placements[placement];
                let q = inverse.point(x);
                let at = |t: f64| s.posed(q,t);
                let [t0,t1] = s.contacts.domain();
                let h = (t1-t0)/self.options.roll_steps.max(2) as f64;
                let seed = roll?;
                let g = |t: f64| at(t).and_then(|(m,x)| trial(&s.faces[face],&m,x)).map(|(g,_)| g);
                let g0 = g(seed)?;
                let mut found = (g0 == 0.).then_some(seed);
                let (mut lo,mut hi) = ((seed,g0),(seed,g0));
                for _ in 0..4*self.options.roll_steps.max(2) {
                    if found.is_some() { break; }
                    let next = (hi.0+h).min(t1);
                    if next > hi.0 {
                        let gn = g(next)?;
                        if gn*hi.1 <= 0. { found = self.root(s,face,&at,[hi.0,next],[hi.1,gn]).map(|r| r.0); break; }
                        hi = (next,gn);
                    }
                    let next = (lo.0-h).max(t0);
                    if next < lo.0 {
                        let gn = g(next)?;
                        if gn*lo.1 <= 0. { found = self.root(s,face,&at,[next,lo.0],[gn,lo.1]).map(|r| r.0); break; }
                        lo = (next,gn);
                    }
                    if hi.0 >= t1 && lo.0 <= t0 { break; }
                }
                let t = found?;
                let (m,xs) = at(t)?;
                let (_,pr) = trial(&s.faces[face],&m,xs)?;
                let f = sub(xs,scale(pr.normal,pr.signed_residual));
                Some((pose.point(m.point(f)),Some(t)))
            }
        }
    }

    /// Which side of the blank's boundary a direction points: +1 where the blank's field rises.
    fn rises(field: &SpatialField,at: V,n: V,step: f64) -> f64 {
        let up = field.value(add(at,scale(n,step)));
        let down = field.value(sub(at,scale(n,step)));
        if up < down { -1. } else { 1. }
    }

    /// A blank face's foot, trimmed where it lies off the finite face or off the blank.
    fn blank_face(&self,index: usize,face: &Face,p: V) -> Option<Candidate> {
        let pr = face.projector.project(p).ok()?;
        if pr.signed_residual.abs() > self.options.reach { return None; }
        let on = face.covers(pr.parameters);
        let foot = sub(p,scale(pr.normal,pr.signed_residual));
        let side = face.side;
        let trimmed = !on || self.blank.value(foot).abs() > self.options.trim;
        Some(Candidate {near:Nearest {distance:side*pr.signed_residual,surface:face.surface,foot,
            normal:scale(pr.normal,side),placement:None,roll:None},piece:Piece::Face(index),trimmed})
    }

    /// Every critical point of the distance from `p` to the envelope of each cutter face at every
    /// placement near it, trimmed where it lies off the blank.
    fn generated(&self,p: V,out: &mut Vec<Candidate>) {
        for (si,sweep) in self.sweeps.iter().enumerate() {
            for (placement,&(pose,inverse)) in sweep.placements.iter().enumerate() {
                let q = inverse.point(p);
                if !sweep.near(q) { continue; }
                let [t0,t1] = sweep.contacts.domain();
                let steps = self.options.roll_steps.max(2);
                let at = |t: f64| sweep.posed(q,t);
                let mut previous: Vec<Option<(f64,f64)>> = vec![None;sweep.faces.len()];
                for i in 0..=steps {
                    let t = t0+(t1-t0)*i as f64/steps as f64;
                    let Some((m,x)) = at(t) else { previous.fill(None); continue; };
                    for (k,face) in sweep.faces.iter().enumerate() {
                        let g = trial(face,&m,x).map(|(g,_)| g);
                        if let (Some((ta,ga)),Some(gb)) = (previous[k],g) {
                            if ga*gb < 0. || gb == 0. {
                                if let Some(c) = self.root(sweep,k,&at,[ta,t],[ga,gb])
                                    .and_then(|r| self.envelope_point(sweep,si,k,r,placement,pose)) { out.push(c); }
                            }
                        }
                        previous[k] = g.map(|g| (t,g));
                    }
                }
            }
        }
    }

    /// A root of the normal velocity at the foot, by false position (the Illinois variant) on
    /// a bracket: the roll, the pose, the point in the cutter's frame and its projection.
    fn root(&self,sweep: &Sweep,k: usize,at: &impl Fn(f64) -> Option<(Motion,V)>,bracket: [f64;2],values: [f64;2])
        -> Option<(f64,Motion,V,super::SurfaceProjection)> {
        let face = &sweep.faces[k];
        let ([mut a,mut b],[mut fa,mut fb]) = (bracket,values);
        let mut side = 0;
        let mut found = None;
        for _ in 0..200 {
            let c = if fb == 0. { b } else {
                let c = (a*fb-b*fa)/(fb-fa);
                if c.is_finite() && c > a.min(b) && c < a.max(b) { c } else { 0.5*(a+b) }
            };
            let (m,x) = at(c)?;
            let (fc,pr) = trial(face,&m,x)?;
            found = Some((c,m,x,pr));
            if fc == 0. || (b-a).abs() <= 1e-13*(1.+c.abs()) { break; }
            if fc*fb > 0. {
                b = c; fb = fc;
                if side == -1 { fa *= 0.5; }
                side = -1;
            } else {
                a = c; fa = fc;
                if side == 1 { fb *= 0.5; }
                side = 1;
            }
        }
        found
    }

    /// The envelope point at a root, if it lies on the finite cutter face and on the cutter's own
    /// boundary; trimmed where it lies off the blank.
    fn envelope_point(&self,sweep: &Sweep,index: usize,k: usize,(t,m,x,pr): (f64,Motion,V,super::SurfaceProjection),
        placement: usize,pose: Motion) -> Option<Candidate> {
        let face = &sweep.faces[k];
        if !face.covers(pr.parameters) { return None; }
        let s = pr.signed_residual;
        if s.abs() > self.options.reach { return None; }
        let f = sub(x,scale(pr.normal,s));
        // on the cutter's own boundary, where the cutter is itself a Boolean
        let source = sweep.contacts.source_material();
        let trim = self.options.trim;
        if source.value(f).abs() > trim { return None; }
        let outward = Self::rises(source,f,pr.normal,trim.max(1e-9));
        let foot = pose.point(m.point(f));
        // the body is the blank less the cutter: its outward normal is into the cutter
        let normal = pose.vector(m.vector(scale(pr.normal,-outward)));
        Some(Candidate {near:Nearest {distance:-outward*s,surface:face.surface,foot,normal,placement:Some(placement),roll:Some(t)},
            piece:Piece::Generated {sweep:index,placement,face:k},trimmed:self.blank.value(foot) > trim})
    }
}

/// What a candidate foot lies on, for following it to an edge.
#[derive(Clone,Copy,Debug)]
enum Piece { Face(usize), Generated {sweep: usize,placement: usize,face: usize} }

/// A foot on one face, and whether it lies off that face's trims.
#[derive(Clone,Copy,Debug)]
struct Candidate { near: Nearest, piece: Piece, trimmed: bool }

/// The normal velocity at the foot of `x` on a cutter face posed by `m`, and the projection.
fn trial(face: &Face,m: &Motion,x: V) -> Option<(f64,super::SurfaceProjection)> {
    let pr = face.projector.project(x).ok()?;
    let f = sub(x,scale(pr.normal,pr.signed_residual));
    let g = dot(m.vector(pr.normal),m.velocity(f));
    g.is_finite().then_some((g,pr))
}

fn key(p: V,cell: f64) -> [i64;3] { p.map(|x| (x/cell).floor() as i64) }

impl Sweep {
    /// The generating motion's pose at roll `t`, and the point `q` of the sweep's own frame in the
    /// cutter's frame there.
    fn posed(&self,q: V,t: f64) -> Option<(Motion,V)> {
        let m = self.contacts.motion().at(t).ok()?;
        Some((m,m.inverse().point(q)))
    }

    /// Whether a point in the sweep's own frame is within a cell of the tabulated envelope.
    fn near(&self,q: V) -> bool {
        let [i,j,k] = key(q,self.cell);
        (-1..=1).any(|a| (-1..=1).any(|b| (-1..=1).any(|c| self.cells.contains(&[i+a,j+b,k+c]))))
    }

    /// Tabulate the unindexed envelope: each cutter face's contacts over a grid of its source
    /// parameters, kept where some placement puts them within reach of the blank's bounding
    /// sphere, in cells no smaller than the gaps between neighbouring contacts.
    fn tabulate(&mut self,sphere: Option<(V,f64)>,options: &Options) -> Result<(),String> {
        let roll = self.contacts.domain();
        let mut points = Vec::new();
        let mut gaps = Vec::new();
        for (k,patch) in self.contacts.patches().iter().enumerate() {
            let [_,[v0,v1]] = patch.domain();
            // the grid's spacing on the source at most half the least cell
            let mut speed: [f64;2] = [0.,0.];
            for i in 0..=4 { for j in 0..=4 {
                if let Ok(s) = patch.at(i as f64/4.,v0+(v1-v0)*j as f64/4.) {
                    speed[0] = speed[0].max(norm(s.du));
                    speed[1] = speed[1].max(norm(s.dv)*(v1-v0));
                }
            } }
            let count = |speed: f64,most: usize| ((2.*speed/options.cell).ceil() as usize).clamp(8,most);
            let (nu,nv) = (count(speed[0],256),count(speed[1],4096));
            let mut grid: Vec<Option<V>> = vec![None;(nu+1)*(nv+1)];
            for i in 0..=nu { for j in 0..=nv {
                let (u,v) = (i as f64/nu as f64,v0+(v1-v0)*j as f64/nv as f64);
                let Ok(roots) = self.contacts.at_source_over(k,u,v,roll,1e-9) else { continue; };
                for r in &roots { points.push(r.contact.position); }
                if roots.len() == 1 { grid[i*(nv+1)+j] = Some(roots[0].contact.position); }
            } }
            for i in 0..=nu { for j in 0..=nv {
                let Some(p) = grid[i*(nv+1)+j] else { continue; };
                if i < nu { if let Some(q) = grid[(i+1)*(nv+1)+j] { gaps.push(distance(p,q)); } }
                if j < nv { if let Some(q) = grid[i*(nv+1)+j+1] { gaps.push(distance(p,q)); } }
            } }
        }
        gaps.sort_by(f64::total_cmp);
        let gap = gaps.get((gaps.len()*99)/100).copied().unwrap_or(0.);
        self.cell = options.cell.max(gap);
        let within = |p: V| sphere.is_none_or(|(c,d)| self.placements.iter()
            .any(|(pose,_)| distance(pose.point(p),c) <= 0.5*d+options.reach+self.cell));
        self.cells = points.into_iter().filter(|&p| within(p)).map(|p| key(p,self.cell)).collect();
        Ok(())
    }
}

/// Distribution of one route's readings over some samples, in model units.
#[derive(Clone,Copy,Debug,Default)]
struct Stats {
    count: usize,
    /// Samples no exact face was found within reach of (analytic route).
    unmatched: usize,
    max: f64,
    p99: f64,
    mean: f64,
    /// The mean of the signed distances: an offset reads here, where faceting reads both ways.
    signed_mean: f64,
    field_max: f64,
    field_p99: f64,
    /// The routes' difference over matched samples.
    agreement_max: f64,
    agreement_p99: f64,
    /// The exporter's normal against the exact one, degrees (NaN where no sample has one).
    normal_max: f64,
    normal_p99: f64,
    /// The sample the analytic route reads farthest.
    worst: Option<usize>,
}

fn percentile(values: &mut [f64],fraction: f64) -> f64 {
    if values.is_empty() { return f64::NAN; }
    values.sort_by(f64::total_cmp);
    values[((values.len() as f64*fraction).ceil() as usize).clamp(1,values.len())-1]
}

/// The exporter's normal against the exact one at the nearest face, in degrees.
fn normal_error(sample: &Sample,measurement: &Measurement) -> Option<f64> {
    let (n,a) = (sample.normal?,measurement.analytic?);
    let c = dot(normalised(n)?,a.normal).clamp(-1.,1.);
    Some(c.dacos().to_degrees())
}

/// The distribution over the samples `which` picks.
fn stats(samples: &[Sample],measurements: &[Measurement],which: impl Fn(usize) -> bool) -> Stats {
    let (mut analytic,mut field,mut agreement,mut normal) = (Vec::new(),Vec::new(),Vec::new(),Vec::new());
    let mut s = Stats {max:0.,..Default::default()};
    let mut signed = 0.;
    for (i,(sample,m)) in samples.iter().zip(measurements).enumerate() {
        if !which(i) { continue; }
        s.count += 1;
        field.push(m.field.abs());
        match m.analytic {
            None => s.unmatched += 1,
            Some(a) => {
                if a.distance.abs() >= s.max { s.max = a.distance.abs(); s.worst = Some(i); }
                analytic.push(a.distance.abs());
                signed += a.distance;
                agreement.push((a.distance-m.field).abs());
                if let Some(e) = normal_error(sample,m) { normal.push(e); }
            }
        }
    }
    let matched = analytic.len();
    s.mean = if matched > 0 { analytic.iter().sum::<f64>()/matched as f64 } else { f64::NAN };
    s.signed_mean = if matched > 0 { signed/matched as f64 } else { f64::NAN };
    if matched == 0 { s.max = f64::NAN; }
    s.p99 = percentile(&mut analytic,0.99);
    s.field_max = field.iter().copied().fold(f64::NAN,f64::max);
    s.field_p99 = percentile(&mut field,0.99);
    s.agreement_max = agreement.iter().copied().fold(f64::NAN,f64::max);
    s.agreement_p99 = percentile(&mut agreement,0.99);
    s.normal_max = normal.iter().copied().fold(f64::NAN,f64::max);
    s.normal_p99 = percentile(&mut normal,0.99);
    s
}

/// A report of an export's accuracy, one line each, distances in micrometres: overall, by
/// kind of exact face, by exact face, by sample kind, by the exporter's class of face, and the
/// exporter's worst faces and locations. `classes` names the samples' `class` numbers;
/// `millimetres` is the model's length unit in millimetres.
pub fn report(meter: &Meter,samples: &[Sample],measurements: &[Measurement],classes: &[String],millimetres: f64) -> Vec<String> {
    let um = |x: f64| x*millimetres*1e3;
    let mm = |p: V| format!("({:.4}, {:.4}, {:.4})",p[0]*millimetres,p[1]*millimetres,p[2]*millimetres);
    let line = |label: &str,s: &Stats| {
        let mut text = format!("{label:<34} {:>7} samples  max {:>9.3}  p99 {:>9.3}  mean {:>8.3}  signed mean {:>8.3}  \
            field max {:>9.3}  routes differ max {:>8.3} p99 {:>8.3}",
            s.count,um(s.max),um(s.p99),um(s.mean),um(s.signed_mean),um(s.field_max),um(s.agreement_max),um(s.agreement_p99));
        if s.normal_max.is_finite() { text += &format!("  normal max {:.2}° p99 {:.2}°",s.normal_max,s.normal_p99); }
        if s.unmatched > 0 { text += &format!("  ({} unmatched)",s.unmatched); }
        text
    };
    let surface = |i: usize| measurements[i].analytic.map(|a| a.surface);
    let mut out = vec!["distances in µm; analytic route (nearest exact face) and field route (material field)".to_string()];
    out.push(line("all",&stats(samples,measurements,|_| true)));
    for (label,generated) in [("generated faces",true),("blank faces",false)] {
        let s = stats(samples,measurements,|i| surface(i).is_some_and(|k| meter.surfaces[k].generated == generated));
        if s.count > 0 { out.push(line(label,&s)); }
    }
    let kinds: BTreeSet<SampleKind> = samples.iter().map(|s| s.kind).collect();
    if kinds.len() > 1 {
        for kind in kinds { out.push(line(kind.name(),&stats(samples,measurements,|i| samples[i].kind == kind))); }
    }
    let used: BTreeSet<u32> = samples.iter().map(|s| s.class).collect();
    if used.len() > 1 {
        for class in used {
            let name = classes.get(class as usize).map_or("?",|s| s.as_str());
            out.push(line(&format!("exported {name}"),&stats(samples,measurements,|i| samples[i].class == class)));
        }
    }
    out.push("by exact face:".into());
    for (k,s) in meter.surfaces.iter().enumerate() {
        let st = stats(samples,measurements,|i| surface(i) == Some(k));
        if st.count > 0 { out.push(line(&format!("  {}{}",if s.generated { "generated " } else { "" },s.name),&st)); }
    }
    // the exporter's faces that read worst
    let faces: BTreeSet<u32> = samples.iter().map(|s| s.face).collect();
    if faces.len() > 1 && faces.len() < samples.len() {
        let mut worst: Vec<(f64,u32)> = faces.iter().map(|&f| (stats(samples,measurements,|i| samples[i].face == f).max,f))
            .filter(|(m,_)| m.is_finite()).collect();
        worst.sort_by(|a,b| b.0.total_cmp(&a.0));
        out.push(format!("worst of {} exported faces:",faces.len()));
        for &(_,f) in worst.iter().take(8) {
            let class = samples.iter().find(|s| s.face == f).map_or(0,|s| s.class);
            let name = classes.get(class as usize).map_or("?",|s| s.as_str());
            out.push(line(&format!("  face {f} ({name})"),&stats(samples,measurements,|i| samples[i].face == f)));
        }
    }
    let mut order: Vec<usize> = (0..samples.len()).filter(|&i| measurements[i].analytic.is_some()).collect();
    order.sort_by(|&a,&b| measurements[b].analytic.unwrap().distance.abs().total_cmp(&measurements[a].analytic.unwrap().distance.abs()));
    out.push("worst locations (mm):".into());
    for &i in order.iter().take(6) {
        let (p,m) = (samples[i].position,measurements[i]);
        let a = m.analytic.unwrap();
        let mut text = format!("  {} {} {}: analytic {:.3}, field {:.3} on {}",mm(p),samples[i].kind.name(),samples[i].face,
            um(a.distance),um(m.field),meter.surfaces[a.surface].name);
        if let (Some(index),Some(roll)) = (a.placement,a.roll) { text += &format!(" at index {index}, roll {:.3}°",roll.to_degrees()); }
        out.push(text);
    }
    // where the exporter's normal is farthest from the exact one, among samples whose own face is
    // the one they are nearest (a vertex or an edge midpoint beside another face reads that one's)
    let mut bent: Vec<(f64,usize)> = (0..samples.len()).filter(|&i| samples[i].kind.own_normal())
        .filter_map(|i| normal_error(&samples[i],&measurements[i]).map(|e| (e,i))).collect();
    bent.sort_by(|a,b| b.0.total_cmp(&a.0));
    if bent.first().is_some_and(|b| b.0 > 0.) {
        out.push("worst normals (mm):".into());
        for &(e,i) in bent.iter().take(4) {
            let (p,a) = (samples[i].position,measurements[i].analytic.unwrap());
            out.push(format!("  {} {} {}: {e:.2}° at {:.3} on {}",mm(p),samples[i].kind.name(),samples[i].face,um(a.distance),
                meter.surfaces[a.surface].name));
        }
    }
    let differ = |i: usize| measurements[i].analytic.map_or(f64::NAN,|a| (a.distance-measurements[i].field).abs());
    if let Some(i) = (0..samples.len()).filter(|&i| differ(i).is_finite()).max_by(|&a,&b| differ(a).total_cmp(&differ(b))) {
        let (p,a) = (samples[i].position,measurements[i].analytic.unwrap());
        out.push(format!("the routes differ most at {}, {} {}: analytic {:.3} on {}, field {:.3}",mm(p),samples[i].kind.name(),
            samples[i].face,um(a.distance),meter.surfaces[a.surface].name,um(measurements[i].field)));
    }
    let unmatched: Vec<usize> = (0..samples.len()).filter(|&i| measurements[i].analytic.is_none()).collect();
    if let Some(&i) = unmatched.first() {
        let p = samples[i].position;
        out.push(format!("{} samples unmatched within {:.3} mm, the first at {}, field {:.3}",unmatched.len(),
            meter.options.reach*millimetres,mm(p),um(measurements[i].field)));
    }
    out
}

/// The verdict against an export tolerance (model units): every exact face's farthest sample on
/// the analytic route within it, and no sample unmatched; the line says which faces exceed it
/// and by how much, or the worst face when none does.
pub fn within(meter: &Meter,samples: &[Sample],measurements: &[Measurement],tolerance: f64,millimetres: f64) -> (bool,String) {
    let um = |x: f64| x*millimetres*1e3;
    let mut faces: Vec<(f64,&str,usize)> = meter.surfaces.iter().enumerate().filter_map(|(k,s)| {
        let st = stats(samples,measurements,|i| measurements[i].analytic.is_some_and(|a| a.surface == k));
        (st.count > 0).then_some((st.max,s.name.as_str(),st.count))
    }).collect();
    faces.sort_by(|a,b| b.0.total_cmp(&a.0));
    let unmatched = measurements.iter().filter(|m| m.analytic.is_none()).count();
    let over: Vec<String> = faces.iter().filter(|f| !(f.0 <= tolerance)).map(|f| format!("{} {:.3}",f.1,um(f.0))).collect();
    let ok = over.is_empty() && unmatched == 0 && !faces.is_empty();
    let text = if ok {
        format!("tolerance {:.3} µm: every exact face within it, the worst {} at {:.3}",um(tolerance),faces[0].1,um(faces[0].0))
    } else {
        format!("tolerance {:.3} µm: exceeded on {} of {} exact faces ({}), {unmatched} samples unmatched",um(tolerance),over.len(),
            faces.len(),over.join(", "))
    };
    (ok,text)
}

/// Samples of a triangle mesh: at most `triangles` triangles chosen by area (as the field
/// agreement chooses them), each giving its centroid, its edges' midpoints and its corners,
/// all carrying the triangle's own normal and its index as their face.
pub fn mesh_samples(vertices: &[V],triangles: &[[u32;3]],most: usize) -> Vec<Sample> {
    let options = super::agreement::Options {triangles:most.max(1),by_area:true,..Default::default()};
    let mut out = Vec::new();
    for index in super::agreement::chosen(vertices,triangles,&options) {
        let (face,t) = (index as u32,triangles[index]);
        let [a,b,c] = t.map(|i| vertices[i as usize]);
        let normal = triangle_normal(a,b,c);
        let mut push = |position,kind| out.push(Sample {position,normal,face,class:0,kind});
        push(scale(add(add(a,b),c),1./3.),SampleKind::Centroid);
        for (p,q) in [(a,b),(b,c),(c,a)] { push(scale(add(p,q),0.5),SampleKind::Midpoint); }
        for p in [a,b,c] { push(p,SampleKind::Vertex); }
    }
    out
}
