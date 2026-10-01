//! A solid made of revolutions about parallel lines, cut by a half-plane through one of them: the
//! section is a planar region (`brep::planar`) whose every step is a piece of one revolution's
//! meridian, plain where the revolution is about the half-plane's own line and carried in by a
//! `Chart` where it is about another. A spiral bevel gear's cutter is its crown bounded by its
//! neighbour, the inner crown turned about the gear's axis, which is parallel to the cutter's own:
//! its sections are exact this way, where a kernel cuts the solid with a plane at each station.
//! Each step is tagged with the face it bounds (its revolution and its meridian's edge), the same
//! at every station, and read exactly: its point and its surface's outward normal in space.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::build::{Profile,ProfileEdge};
use super::geom::{Rigid,V};
use super::planar::{boolean,Chart,Op,Region,Seg,Step};
use crate::json::Json;
use crate::space::{add,cross,dot,norm,scale,sub};
use std::collections::BTreeMap;

fn vec3(j: &Json) -> V { let a = j.arr(); [a[0].as_f64(),a[1].as_f64(),a[2].as_f64()] }
fn field<'a>(j: &'a Json,k: &str) -> Result<&'a Json,String> { j.get(k).ok_or(format!("recipe: no `{k}`")) }
fn unit(a: V) -> V { scale(a,1./norm(a)) }

/// One revolution: a point of its line and its direction (`±` the section's), and its meridian
/// region in its own `(r, z)` (radius about its line, distance along it from that point).
#[derive(Clone,Debug)]
struct Leaf { o: V,a: V,region: Region }

/// What each recipe node is: a revolution (by its index among the leaves), or a body.
#[derive(Clone,Debug)]
enum Node { Leaf(usize),Body { stock: i64,ops: Vec<(Op,i64)> } }

/// A recipe of revolutions about lines parallel to its first's, ready to be cut by half-planes of
/// that line.
#[derive(Clone,Debug)]
pub struct Sectioned { pub origin: V,pub axis: V,leaves: Vec<Leaf>,nodes: Vec<(i64,Node)>,root: i64,tol: f64 }

/// The tag a revolution's meridian step bounds: its revolution, loop and place.
fn tag(leaf: usize,l: usize,k: usize) -> u32 { ((leaf as u32+1) << 16) | ((l as u32) << 10) | k as u32 }

impl Sectioned {
    /// The recipe read, or `Ok(Err)` naming why it is not of this kind.
    pub fn read(recipe: &Json) -> Result<Result<Sectioned,String>,String> {
        let nodes = field(recipe,"nodes")?.arr();
        let mut size: f64 = 1.;
        for n in nodes.iter().filter(|n| n.get("kind").is_some_and(|k| k.as_str() == "revolve")) {
            size = size.max(norm(vec3(field(n,"origin")?))+norm(vec3(field(n,"axis")?)));
        }
        let tol = 1e-9*size;
        // each node's leaf (a revolution, as placed so far)
        // each revolution's line as placed, and the line and profile it was drawn with
        let mut placed: BTreeMap<i64,usize> = BTreeMap::new();
        let mut profiles: Vec<(V,V,V,V,Profile)> = Vec::new();
        let mut out: Vec<(i64,Node)> = Vec::new();
        for n in nodes {
            let (id,name) = (field(n,"id")?.as_i64(),n.get("name").map_or("",|j| j.as_str()));
            match field(n,"kind")?.as_str() {
                "revolve" => {
                    if (field(n,"angle")?.as_f64().abs()-std::f64::consts::TAU).abs() > 1e-9 { return Ok(Err(format!("`{name}` is a partial revolution"))) }
                    let (o,a) = (vec3(field(n,"origin")?),unit(vec3(field(n,"axis")?)));
                    profiles.push((o,a,o,a,Profile::from_json(field(n,"profile")?)?));
                    placed.insert(id,profiles.len()-1);
                    out.push((id,Node::Leaf(profiles.len()-1)));
                }
                "placed" => {
                    let src = field(n,"source")?.as_i64();
                    let Some(&k) = placed.get(&src) else { return Ok(Err(format!("`{name}` places something not a revolution"))) };
                    let mm: Vec<f64> = field(n,"matrix")?.arr().iter().map(Json::as_f64).collect();
                    let m = Rigid::from_rows(&mm);
                    let (o,a,so,sa,p) = profiles[k].clone();
                    profiles.push((m.point(o),m.vector(a),so,sa,p));
                    placed.insert(id,profiles.len()-1);
                    out.push((id,Node::Leaf(profiles.len()-1)));
                }
                "body" => {
                    let mut ops = Vec::new();
                    for (key,op) in [("on",Op::Union),("cut",Op::Cut),("bound",Op::Common)] {
                        for x in field(n,key)?.arr() { ops.push((op,x.as_i64())); }
                    }
                    out.push((id,Node::Body {stock:field(n,"stock")?.as_i64(),ops}));
                }
                kind => return Ok(Err(format!("`{name}` is a {kind}, not a revolution"))),
            }
        }
        let Some(&(origin,axis,..)) = profiles.first() else { return Ok(Err("no revolution".into())) };
        let mut leaves = Vec::new();
        for (o,a,so,sa,profile) in profiles {
            if norm(cross(a,axis)) > 1e-12 { return Ok(Err("a revolution about a line not parallel to the first's".into())) }
            // the meridian in its own (r, z): the side of its line every point of the profile is on
            let arc_at = |f: &super::geom::Frame,r: f64,w: f64| add(f.o,add(scale(f.x,r*w.dcos()),scale(f.y,r*w.dsin())));
            let mut side: Option<V> = None;
            for e in profile.loops.iter().flatten() {
                let pts: Vec<V> = match *e {
                    ProfileEdge::Line {a: p,b: q} => vec![p,q],
                    ProfileEdge::Arc {frame,r,span} => { let [w0,w1] = span.unwrap_or([0.,std::f64::consts::TAU]);
                        (0..=16).map(|k| arc_at(&frame,r,w0+(w1-w0)*k as f64/16.)).collect() }
                    ProfileEdge::Spline(_) => return Ok(Err("a profile with a spline".into())),
                };
                for p in pts {
                    let d = sub(p,so);
                    let d = sub(d,scale(sa,dot(d,sa)));
                    if norm(d) <= tol { continue }
                    let d = unit(d);
                    match side { None => side = Some(d), Some(s) => if dot(s,d) < 1.-1e-9 { return Ok(Err("a profile crossing its line".into())) } }
                }
            }
            // (read where it was drawn: a placement is rigid, and moves only the line)
            let Some(side) = side else { return Ok(Err("a profile on its line".into())) };
            let q = |p: V| { let d = sub(p,so); [dot(d,side),dot(d,sa)] };
            let region = Region::plain(profile.loops.iter().map(|l| l.iter().map(|e| match *e {
                ProfileEdge::Line {a: p,b: r} => Seg::Line {a:q(p),b:q(r)},
                ProfileEdge::Arc {frame,r,span} => {
                    let [w0,w1] = span.unwrap_or([0.,std::f64::consts::TAU]);
                    let c = q(frame.o);
                    let p0 = q(arc_at(&frame,r,w0));
                    let (x,y) = ([dot(frame.x,side),dot(frame.x,sa)],[dot(frame.y,side),dot(frame.y,sa)]);
                    Seg::Arc {c,r,a0:(p0[1]-c[1]).datan2(p0[0]-c[0]),sweep:(x[0]*y[1]-x[1]*y[0]).signum()*(w1-w0)}
                }
                ProfileEdge::Spline(_) => unreachable!("refused above"),
            }).collect()).collect());
            leaves.push((o,a,region));
        }
        let leaves = leaves.into_iter().map(|(o,a,region)| Leaf {o,a,region}).collect();
        Ok(Ok(Sectioned {origin,axis,leaves,nodes:out,root:field(recipe,"root")?.as_i64(),tol}))
    }

    /// The section by the half-plane of the line towards `side` (unit, square to the line), in its
    /// `(s, z)`: along `side` from the line, and along the line from `origin`.
    pub fn section(&self,side: V) -> Result<Region,String> {
        let mut regions: BTreeMap<i64,Region> = BTreeMap::new();
        for (id,node) in &self.nodes {
            let region = match node {
                Node::Leaf(i) => {
                    let leaf = &self.leaves[*i];
                    let d = sub(self.origin,leaf.o);
                    let along = dot(d,self.axis);
                    let perp = sub(d,scale(self.axis,along));
                    let sign = dot(leaf.a,self.axis).signum();
                    let t = |l: usize,k: usize| tag(*i,l,k);
                    if norm(perp) <= self.tol && sign > 0. && along.abs() <= self.tol { leaf.region.tagged(&t) }
                    else { leaf.region.charted(Chart {beta:dot(side,perp),delta2:dot(perp,perp),shift:along,sign},&t) }
                }
                Node::Body {stock,ops} => {
                    let mut region = regions.get(stock).ok_or("recipe: a stock before its body")?.clone();
                    for (op,x) in ops {
                        region = boolean(&region,regions.get(x).ok_or("recipe: an operand before its body")?,*op,self.tol)?;
                    }
                    region
                }
            };
            regions.insert(*id,region);
        }
        regions.remove(&self.root).ok_or("recipe: no root".into())
    }

    /// A section step's point at `t` in space, and its revolution's outward normal there.
    pub fn at(&self,side: V,step: &Step,t: f64) -> (V,V) {
        let p = step.point(t);
        let x = add(self.origin,add(scale(side,p[0]),scale(self.axis,p[1])));
        let [nr,nz] = step.own_normal(t);
        let leaf = &self.leaves[((step.tag >> 16) as usize).saturating_sub(1).min(self.leaves.len()-1)];
        // the radial direction about the step's own line, and that line's direction
        let d = sub(x,leaf.o);
        let radial = sub(d,scale(self.axis,dot(d,self.axis)));
        let radial = if norm(radial) > 0. { unit(radial) } else { side };
        let a = if dot(leaf.a,self.axis) < 0. { scale(self.axis,-1.) } else { self.axis };
        (x,add(scale(radial,nr),scale(a,nz)))
    }
}
