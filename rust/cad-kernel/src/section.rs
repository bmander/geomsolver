//! A solid made of revolutions about parallel lines, cut by a half-plane through one of them: the
//! section is a planar region (`brep::planar`) whose every step is a piece of one revolution's
//! meridian, plain where the revolution is about the half-plane's own line and carried in by a
//! `Chart` where it is about another. A spiral bevel gear's cutter is its crown bounded by its
//! neighbour, the inner crown turned about the gear's axis, which is parallel to the cutter's own:
//! its sections are exact this way, where a kernel cuts the solid with a plane at each station.
//! Each step is tagged with the face it bounds (its revolution and its meridian's edge), the same
//! at every station, and read exactly: its point and its surface's outward normal in space.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use super::build::Profile;
use super::geom::{Rigid,V};
use super::planar::{boolean,Chart,Op,Region,Step};
use super::recipe::{meridian_of,refused,revolutions_tolerance,whole_turn,Named,Node,Recipe,Unread};
use super::KernelError;
use crate::util::json::Json;
use crate::util::space::{add,cross,dot,norm,scale,sub,unit};
use std::collections::BTreeMap;

/// One revolution: a point of its line and its direction (`±` the section's), and its meridian
/// region in its own `(r, z)` (radius about its line, distance along it from that point), each
/// step tagged by its profile edge (`edge_tag`); the recipe node's name, and its edges' (#212).
#[derive(Clone,Debug)]
struct Leaf { o: V,a: V,region: Region,name: String,edges: Vec<Vec<String>> }

/// A profile edge's place, loop `l` and edge `k`, as a leaf's steps carry it and `tag` packs it.
fn edge_tag(l: usize,k: usize) -> u32 { ((l as u32) << 10) | k as u32 }

/// What each recipe node is: a revolution (by its index among the leaves), or a body.
#[derive(Clone,Debug)]
enum Term { Leaf(usize),Body { stock: i64,ops: Vec<(Op,i64)> } }

/// A recipe of revolutions about lines parallel to its first's, ready to be cut by half-planes of
/// that line.
#[derive(Clone,Debug)]
pub struct Sectioned { pub origin: V,pub axis: V,leaves: Vec<Leaf>,nodes: Vec<(i64,Term)>,root: i64,tol: f64,
    /// Read from a B-rep (`of_boundary`): each step's face, by its solid's key and its own name,
    /// at the step's place (its tag's).
    faces: Vec<(String,String)> }

/// The tag a revolution's meridian step bounds: its revolution, loop and place.
fn tag(leaf: usize,l: usize,k: usize) -> u32 { ((leaf as u32+1) << 16) | edge_tag(l,k) }

/// How far a meridian `region` reaches from a point `at` from its line's origin: its farthest
/// point in `(r, z)`, read at each step's ends and seven places between (an arc's farthest point
/// between two of them at most 8% farther), plus `at`.
fn reach(region: &Region,at: f64) -> f64 {
    let far = region.loops.iter().flatten()
        .flat_map(|st| (0..=8).map(move |i| st.point(i as f64/8.)))
        .map(|p| p[0].dhypot(p[1])).fold(0.,f64::max);
    far+at
}

/// `tag`'s inverse: the revolution (`None` for no leaf's), loop and place.
fn untag(tag: u32) -> (Option<usize>,usize,usize) {
    ((((tag >> 16) & 0x3ff) as usize).checked_sub(1),((tag >> 10) & 0x3f) as usize,(tag & 0x3ff) as usize)
}

impl Sectioned {
    /// The recipe's JSON read (`of`).
    pub fn read(recipe: &Json) -> Result<Sectioned,KernelError> { Sectioned::of(&Recipe::read(recipe)?) }

    /// The recipe read, or a `Kind::Refused` error naming why it is not of this kind.
    pub fn of(recipe: &Recipe) -> Result<Sectioned,KernelError> {
        let tol = revolutions_tolerance(&recipe.nodes);
        // each node's leaf (a revolution, as placed so far)
        // each revolution's line as placed, and the line and profile it was drawn with
        let mut placed: BTreeMap<i64,usize> = BTreeMap::new();
        let mut profiles: Vec<(V,V,V,V,Profile,String)> = Vec::new();
        let mut out: Vec<(i64,Term)> = Vec::new();
        for Named {id,name,node,..} in &recipe.nodes {
            match node {
                Node::Revolve {profile,origin,axis,angle} => {
                    whole_turn(*angle,name)?;
                    let (o,a) = (*origin,unit(*axis));
                    profiles.push((o,a,o,a,Profile::of(profile)?,name.to_string()));
                    placed.insert(*id,profiles.len()-1);
                    out.push((*id,Term::Leaf(profiles.len()-1)));
                }
                Node::Placed {source,matrix} => {
                    let Some(&k) = placed.get(source) else { return Err(refused(format!("`{name}` places something not a revolution"))) };
                    let m = Rigid::from_rows(matrix);
                    let (o,a,so,sa,p,_) = profiles[k].clone();
                    profiles.push((m.point(o),m.vector(a),so,sa,p,name.to_string()));
                    placed.insert(*id,profiles.len()-1);
                    out.push((*id,Term::Leaf(profiles.len()-1)));
                }
                Node::Body(b) => {
                    let mut ops = Vec::new();
                    for ((_,list),op) in b.sets().into_iter().zip([Op::Union,Op::Cut,Op::Common]) {
                        for x in list { ops.push((op,*x)); }
                    }
                    out.push((*id,Term::Body {stock:b.stock,ops}));
                }
                node => return Err(refused(format!("`{name}` is a {}, not a revolution",node.kind()))),
            }
        }
        let Some(&(origin,axis,..)) = profiles.first() else { return Err(refused("no revolution")) };
        let mut leaves = Vec::new();
        for (o,a,so,sa,profile,name) in profiles {
            // the meridian in its own (r, z), read where it was drawn: a placement is rigid, and
            // moves only the line
            let loops = match meridian_of(&profile,so,sa,tol,false) {
                Ok(loops) => loops,
                Err(Unread::Spline) => return Err(refused("a profile with a spline")),
                Err(Unread::Crosses) => return Err(refused("a profile crossing its line")),
                Err(Unread::OnLine) => return Err(refused("a profile on its line")),
            };
            let region = Region::new(loops.into_iter().enumerate()
                .map(|(li,l)| l.into_iter().enumerate().map(|(k,seg)| Step {seg,chart:None,tag:edge_tag(li,k)}).collect()).collect());
            // parallel to the tolerance: read as about a line parallel to the first's, each point
            // of the revolution moves by the sine between the lines times its reach from where they
            // are compared — a solved line is parallel to its rounding, not exactly
            if norm(cross(a,axis))*reach(&region,norm(sub(o,origin))) > tol {
                return Err(refused("a revolution about a line not parallel to the first's"))
            }
            leaves.push(Leaf {o,a,region,name,edges:profile.names.clone()});
        }
        Ok(Sectioned {origin,axis,leaves,nodes:out,root:recipe.root,tol,faces:Vec::new()})
    }

    /// **A cutter of revolution read from its B-rep** (#721 F1d), not its recipe: every face of
    /// it a whole turn of a cylinder, cone, sphere, torus or revolution of a line or a circle about
    /// one line, or a plane square to it bounded by circles about it (a disc, a ring). Its section
    /// by any half-plane of the line is the same, the faces' meridians over the spans of them the
    /// B-rep's faces reach, laid in the half-plane in `(r, z)` and joined end to end, the line
    /// closing them where they end on it; each step names its face. Refused, saying why, for any
    /// other boundary: a face about another line (a cutter of revolutions about parallel lines, as
    /// a spiral bevel's, is sectioned off its recipe), a face over part of a turn, a plane along
    /// the line, a face no closed form reads.
    pub fn of_boundary(b: &crate::topo::Brep) -> Result<Sectioned,String> {
        use super::geom::{Curve,Surface};
        use super::planar::Seg;
        use super::recipe::{joined,named_parts};
        use crate::topo::{unwrap,EdgeCurve};
        let tol = super::geom::tolerance(b.size());
        let frame = b.faces.iter().find_map(|f| match &f.surface {
            Surface::Cylinder(fr,..) | Surface::Cone(fr,..) | Surface::Sphere(fr,..) | Surface::Torus(fr,..) | Surface::Revolution(fr,..) => Some(*fr),
            _ => None,
        }).ok_or("it has no face of revolution")?;
        let (origin,axis) = (frame.o,unit(frame.z));
        let along = |p: V| dot(sub(p,origin),axis);
        let on_axis = |p: V| norm(cross(sub(p,origin),axis)) <= 1e2*tol;
        // each face's meridian piece in the half-plane, and its name
        let mut pieces: Vec<(Seg,String)> = Vec::new();
        for f in &b.faces {
            let named = joined(&f.name);
            // the face's parameters over its loops, each loop unwrapped along itself
            let mut span = [[f64::INFINITY,f64::NEG_INFINITY];2];
            for l in &f.loops {
                let mut last: Option<[f64;2]> = None;
                for c in l {
                    let e = &b.edges[c.edge as usize];
                    for k in 0..=8 {
                        let s = k as f64/8.;
                        let t = if c.reversed { e.t[1]-(e.t[1]-e.t[0])*s } else { e.t[0]+(e.t[1]-e.t[0])*s };
                        let mut uv = c.pcurve.at(t,e,&f.surface,&b.vertices);
                        if let Some(near) = last { uv = unwrap(uv,near,f.surface.periods()) }
                        for i in 0..2 { span[i] = [span[i][0].min(uv[i]),span[i][1].max(uv[i])] }
                        last = Some(uv);
                    }
                }
            }
            let [[u0,u1],[v0,v1]] = span;
            let piece = match &f.surface {
                Surface::Plane(fr) => {
                    if norm(cross(fr.z,axis)) > 1e-9 { return Err(format!("its face `{named}` is a plane not square to its axis")) }
                    let mut radii: Vec<f64> = Vec::new();
                    for c in f.loops.iter().flatten() {
                        match &b.edges[c.edge as usize].curve {
                            EdgeCurve::Curve(Curve::Circle(cf,r)) if norm(cross(cf.z,axis)) <= 1e-9 && on_axis(cf.o) => radii.push(*r),
                            _ => return Err(format!("its face `{named}` is a plane bounded by more than circles about its axis")),
                        }
                    }
                    let inner = if f.loops.len() == 1 { 0. } else { radii.iter().copied().fold(f64::INFINITY,f64::min) };
                    let outer = radii.iter().copied().fold(0.,f64::max);
                    let z = along(fr.o);
                    Seg::Line {a:[inner,z],b:[outer,z]}
                }
                Surface::Cylinder(fr,..) | Surface::Cone(fr,..) | Surface::Sphere(fr,..) | Surface::Torus(fr,..) | Surface::Revolution(fr,..) => {
                    if norm(cross(fr.z,axis)) > 1e-9 || !on_axis(fr.o) {
                        return Err(format!("its face `{named}` turns about another line than its first face's"))
                    }
                    if u1-u0 < std::f64::consts::TAU*(1.-1e-9) { return Err(format!("its face `{named}` turns only part of the way round")) }
                    let (s,z0) = (dot(fr.z,axis).signum(),along(fr.o));
                    let lift = |x: f64,z: f64| [x,z0+s*z];
                    match &f.surface {
                        Surface::Cylinder(_,r) => Seg::Line {a:lift(*r,v0),b:lift(*r,v1)},
                        Surface::Cone(_,r,alpha) => {
                            let (sn,cs) = alpha.dsin_cos();
                            Seg::Line {a:lift(r+v0*sn,v0*cs),b:lift(r+v1*sn,v1*cs)}
                        }
                        // `(r cos v, r sin v)` about the centre, `z` turned with the frame
                        Surface::Sphere(_,r) => Seg::Arc {c:lift(0.,0.),r:*r,a0:s*v0,sweep:s*(v1-v0)},
                        Surface::Torus(_,big,r) => Seg::Arc {c:lift(*big,0.),r:*r,a0:s*v0,sweep:s*(v1-v0)},
                        Surface::Revolution(_,c) => {
                            let at = |v: f64| { let l = fr.local(c.point(v)); lift(l[0],l[2]) };
                            match **c {
                                Curve::Line {..} => Seg::Line {a:at(v0),b:at(v1)},
                                Curve::Circle(cf,r) => {
                                    let l = fr.local(cf.o);
                                    let centre = lift(l[0],l[2]);
                                    let (p,q) = (at(v0),at(v0+1e-3*(v1-v0)));
                                    let (dp,dq) = ([p[0]-centre[0],p[1]-centre[1]],[q[0]-centre[0],q[1]-centre[1]]);
                                    let sense = (dp[0]*dq[1]-dp[1]*dq[0]).signum();
                                    Seg::Arc {c:centre,r,a0:dp[1].datan2(dp[0]),sweep:sense*(v1-v0)}
                                }
                                _ => return Err(format!("its face `{named}` turns {} {}, not a line or a circle",
                                    if c.kind().starts_with(['a','e','i','o','u']) { "an" } else { "a" },c.kind())),
                            }
                        }
                        _ => unreachable!("a face of revolution"),
                    }
                }
                other => return Err(format!("its face `{named}` is a {}, which no closed form sections",other.kind())),
            };
            // its ends on the circles about the axis its edges lie on, exactly: the span read off
            // its loops' parameter curves is only as near as they are
            let rims: Vec<[f64;2]> = f.loops.iter().flatten().filter_map(|c| match &b.edges[c.edge as usize].curve {
                EdgeCurve::Curve(Curve::Circle(cf,r)) if norm(cross(cf.z,axis)) <= 1e-9 && on_axis(cf.o) => Some([*r,along(cf.o)]),
                _ => None,
            }).collect();
            let snap = |p: [f64;2]| rims.iter().copied().filter(|q| (p[0]-q[0]).dhypot(p[1]-q[1]) <= 1e-4*(1.+b.size()))
                .min_by(|x,y| (p[0]-x[0]).dhypot(p[1]-x[1]).total_cmp(&(p[0]-y[0]).dhypot(p[1]-y[1]))).unwrap_or(p);
            let piece = match piece {
                Seg::Line {a,b} => Seg::Line {a:snap(a),b:snap(b)},
                Seg::Arc {c,r,a0,sweep} => {
                    let st = super::planar::Step::from(piece);
                    let (p,q) = (snap(st.start()),snap(st.end()));
                    let (w0,w1) = ((p[1]-c[1]).datan2(p[0]-c[0]),(q[1]-c[1]).datan2(q[0]-c[0]));
                    // the turn from end to end the way and about as far as it ran
                    let turn = w1-w0+std::f64::consts::TAU*((sweep-(w1-w0))/std::f64::consts::TAU).round();
                    let _ = a0;
                    Seg::Arc {c,r,a0:w0,sweep:turn}
                }
                other => other,
            };
            pieces.push((piece,f.name.clone()));
        }
        if pieces.len() >= 1 << 10 { return Err("its faces are too many to tag".into()) }
        // the line closing the meridians where they end on it: each end on it no other piece
        // shares, paired up the line
        let close = 1e2*tol;
        let near = |p: [f64;2],q: [f64;2]| (p[0]-q[0]).dhypot(p[1]-q[1]) <= close;
        let ends: Vec<[f64;2]> = pieces.iter().flat_map(|(s,_)| { let st = super::planar::Step::from(*s); [st.start(),st.end()] }).collect();
        let mut alone: Vec<f64> = ends.iter().filter(|&&p| p[0] <= close && ends.iter().filter(|&&q| near(p,q)).count() == 1)
            .map(|p| p[1]).collect();
        alone.sort_by(f64::total_cmp);
        if alone.len()%2 == 1 { return Err("its meridians do not close on its axis".into()) }
        for w in alone.chunks(2) { pieces.push((Seg::Line {a:[0.,w[0]],b:[0.,w[1]]},String::new())) }
        // the pieces parted into loops, end to end
        let steps: Vec<super::planar::Step> = pieces.iter().enumerate()
            .map(|(k,(s,_))| super::planar::Step {seg:*s,chart:None,tag:edge_tag(0,k)}).collect();
        let mut group: Vec<usize> = (0..steps.len()).collect();
        let root = |g: &Vec<usize>,mut i: usize| { while g[i] != i { i = g[i] } i };
        for i in 0..steps.len() { for j in i+1..steps.len() {
            let (a,c) = (&steps[i],&steps[j]);
            if [a.start(),a.end()].iter().any(|&p| near(p,c.start()) || near(p,c.end())) {
                let (x,y) = (root(&group,i),root(&group,j));
                group[x] = y;
            }
        } }
        let mut loops: BTreeMap<usize,Vec<super::planar::Step>> = BTreeMap::new();
        for (i,st) in steps.iter().enumerate() { loops.entry(root(&group,i)).or_default().push(*st) }
        let region = Region::new(loops.into_values().collect());
        let faces = pieces.iter().map(|(_,name)| { let (key,face) = named_parts(name); (key.to_string(),face.to_string()) }).collect();
        Ok(Sectioned {origin,axis,leaves:vec![Leaf {o:origin,a:axis,region,name:String::new(),edges:Vec::new()}],nodes:vec![(0,Term::Leaf(0))],
            root:0,tol,faces})
    }

    /// The section by the half-plane of the line towards `side` (unit, square to the line), in its
    /// `(s, z)`: along `side` from the line, and along the line from `origin`.
    pub fn section(&self,side: V) -> Result<Region,String> {
        let mut regions: BTreeMap<i64,Region> = BTreeMap::new();
        for (id,node) in &self.nodes {
            let region = match node {
                Term::Leaf(i) => {
                    let leaf = &self.leaves[*i];
                    let d = sub(self.origin,leaf.o);
                    let along = dot(d,self.axis);
                    let perp = sub(d,scale(self.axis,along));
                    let sign = dot(leaf.a,self.axis).signum();
                    // each step its revolution's and its profile edge's (`edge_tag`)
                    let t = |l: usize,k: usize| tag(*i,0,0) | leaf.region.loops[l][k].tag;
                    if norm(perp) <= self.tol && sign > 0. && along.abs() <= self.tol { leaf.region.tagged(&t) }
                    else { leaf.region.charted(Chart {beta:dot(side,perp),delta2:dot(perp,perp),shift:along,sign},&t) }
                }
                Term::Body {stock,ops} => {
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

    /// The face a section step's tag bounds (`tag`, its meeting bits aside): the recipe node of its
    /// revolution, by name, and its profile edge's name — empty where the profile names none.
    pub fn face_of(&self,tag: u32) -> Option<(String,String)> {
        let (leaf,l,k) = untag(tag);
        // read from a B-rep, each step's face its own (`of_boundary`): none for the axis
        if !self.faces.is_empty() { return self.faces.get(k).filter(|f| !f.0.is_empty() || !f.1.is_empty()).cloned() }
        let leaf = self.leaves.get(leaf?)?;
        Some((leaf.name.clone(),leaf.edges.get(l).and_then(|e| e.get(k)).cloned().unwrap_or_default()))
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
