//! The CAD recipe (`solid::cad::recipe`) built by this kernel: the same millimetre-valued DAG the
//! native host is handed, node by node, so the two kernels can be held to each other.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::boolean::{combined,Op};
use super::build::{cut_before,drafted,kept,kept_by_solid,prism,revolve,Profile};
use super::geom::{tolerance,tolerance_over,widen,Rigid,V,EMPTY};
use super::topo::Brep;
use super::{KernelError,Kind};
use crate::json::Json;
use std::collections::BTreeMap;

/// A node built, or why not.
type Built = Result<Brep,KernelError>;

mod node;
mod writer;
pub use node::{Body,Corner,Edge,Family,FamilyWall,Kept,Lowered,Named,Node,Recipe,Rolled,Run,Section,Stroke};
pub use writer::Writer;

/// Between a face's solid and its own name while a recipe is built (`build`, `meridian`,
/// `solid::exact`): the solid half keyed as `document::operand_primitives` keys it, the face half
/// as its primitive names it (`pocket` and `near`). A placement renames the solid half and leaves
/// the face half alone, and neither half may hold it.
pub const NAMED: char = '\u{1}';

/// The key of a face's solid and its own name, as a recipe's build names it (`NAMED`): the whole
/// name and nothing where it was not built so.
pub fn named_parts(name: &str) -> (&str,&str) { name.split_once(NAMED).unwrap_or((name,"")) }

/// A face's name under its solid's key (`NAMED`), the inverse of `named_parts`.
pub fn name_under(key: &str,face: &str) -> String { format!("{key}{NAMED}{face}") }

/// One node of a recipe built from the nodes before it; `Err` names what this kernel does not
/// build yet.
pub fn node(n: &Json,built: &BTreeMap<i64,Brep>) -> Result<Brep,String> { node_named(n,built,&BTreeMap::new(),0.) }

/// `node`, naming each operand in what it says, its Booleans' tolerance never under `floor` (mm):
/// the rounding a recipe read off coordinates far from its origin carries (a part solved at 1e9 mm
/// is placed to 1e-7 mm, wherever it is built about).
pub fn node_named(n: &Json,built: &BTreeMap<i64,Brep>,built_names: &BTreeMap<i64,String>,floor: f64) -> Result<Brep,String> {
    Ok(Node::read(n)?.built(built,built_names,floor)?)
}

impl Node {
    /// The node built from the nodes before it (`built`, named by `built_names` in what it says),
    /// its Booleans' tolerance never under `floor` (mm) (`node_named`).
    pub fn built(&self,built: &BTreeMap<i64,Brep>,built_names: &BTreeMap<i64,String>,floor: f64) -> Built {
        match self {
            Node::Prism {profile,from,to,draft} => Ok(drafted(&Profile::of(profile)?,*from,*to,draft.unwrap_or(0.))?),
            Node::Revolve {profile,origin,axis,angle} => Ok(revolve(&Profile::of(profile)?,*origin,*axis,*angle)?),
            Node::Placed {source,matrix} => {
                let source = built.get(source).ok_or("recipe: `source` not built before its user")?;
                Ok(source.moved(&Rigid::from_rows(matrix)))
            }
            Node::Through {profile,sources} => through_node(profile,sources,built),
            Node::Body(b) => body(b,built,built_names,floor),
            // a ball swept by a turn or a screw (`solid::canal`, #238): the tube about its centre's path
            Node::Canal {centre,radius,origin,axis,angle,advance,face} =>
                Ok(super::build::canal(*centre,*radius,*origin,*axis,*angle,*advance,face)?),
            Node::Region(pieces) => region_node(pieces,built,built_names,floor),
            Node::Lowered(l) => lowered_node(l,built,built_names,floor),
            Node::Family(f) => family_node(f),
        }
    }
}

/// A `family` node: each wall's sheet over the pieces of its interval, the solid they bound.
fn family_node(f: &Family) -> Built {
    use super::family::{Sheet,Stations,Wall};
    let stations = std::sync::Arc::new(Stations::new(f.spans.clone(),f.at.clone())?);
    let loops = f.loops.iter().map(|l| l.iter().map(|w| Ok(Wall {
        sheet:std::sync::Arc::new(Sheet::new(w.run.clone(),stations.clone(),w.nodes.clone(),w.closed)?),name:w.name.clone()}))
        .collect::<Result<Vec<_>,String>>()).collect::<Result<Vec<_>,String>>()?;
    Ok(super::family::family(&loops,f.vanish,f.carry)?)
}

/// A `through` node: a prism spanning its sources' material along the profile's normal (their
/// `bounds` enclose it, issue #59), padded as the native host pads it.
fn through_node(profile: &Section,sources: &[i64],built: &BTreeMap<i64,Brep>) -> Built {
    let profile = Profile::of(profile)?;
    let mut bounds = EMPTY;
    for s in sources {
        widen(&mut bounds,built.get(s).ok_or("recipe: a `through` source not built before it")?.bounds());
    }
    let (lo,hi) = bounds;
    let (nrm,o) = (crate::space::scale(profile.normal,1./crate::space::norm(profile.normal)),profile.origin);
    let (mut from,mut to,mut diagonal) = (0.,0.,0.);
    for k in 0..3 {
        let (a,z) = (nrm[k]*(lo[k]-o[k]),nrm[k]*(hi[k]-o[k]));
        from += a.min(z); to += a.max(z);
        diagonal += (hi[k]-lo[k]).dpowi(2);
    }
    let pad = diagonal.sqrt()*4e-5;
    Ok(prism(&profile,from-pad,to+pad)?)
}

/// A `region` node, a region in general position (`solid::bounded`, #238): the common of the
/// primitives it keeps within, less those it keeps outside, each bounded past its material.
fn region_node(pieces: &[Kept],built: &BTreeMap<i64,Brep>,built_names: &BTreeMap<i64,String>,floor: f64) -> Built {
    let mut solid: Option<Brep> = None;
    for piece in pieces {
        let b = piece.node.built(built,built_names,floor)?;
        solid = Some(match solid {
            None if piece.keep => b,
            None => return Err("recipe: a region's first piece is one it keeps within".into()),
            Some(s) => combined(&s,&b,if piece.keep { Op::Common } else { Op::Cut },floor)?,
        });
    }
    solid.ok_or_else(|| "recipe: a region of no pieces".into())
}

/// A `lowered` node, a region set's pieces (`solid::lowering`, #194): each run a prism or a
/// revolution, each corner the ball's patch built where it stands, each rolled run the ball rolled
/// along it; those joined (a run ending in the section a corner's cell begins in, meeting nowhere
/// else) glued there, never intersected; the chains unioned (`chained`).
fn lowered_node(n: &Lowered,built: &BTreeMap<i64,Brep>,built_names: &BTreeMap<i64,String>,floor: f64) -> Built {
    let mut pieces = n.pieces.iter().map(|piece| lowered_run(piece,built,built_names,floor)).collect::<Result<Vec<_>,_>>()?;
    for c in &n.corners {
        let (vertex,centre,radius) = (c.vertex,c.centre,c.radius);
        let [a,b,s] = &c.faces[..] else { return Err("recipe: a corner stands between three faces".into()) };
        let tol = tolerance_over(1.+radius+crate::space::norm(vertex),floor);
        pieces.push(super::fillet::corner_among(vertex,centre,radius,[a,b,s],tol).map_err(|e| format!("a corner: {e}"))?);
    }
    // a ball rolled along a traced run (#194): against the body the region rounds as its terms
    // but the late ones make it (the stratum), each piece the material it adds or takes
    let first_rolled = pieces.len();
    if !n.rolled.is_empty() {
        let stratum = body(n.stratum.as_ref().ok_or("recipe: no `stratum`")?,built,built_names,floor)?;
        let tol = tolerance_over(stratum.size(),floor);
        for k in &n.rolled {
            pieces.push(rolled_run(k,&stratum,tol,floor)?);
        }
    }
    // a ring's section beyond a seam's normal surface (#422), fitted: glued to its ring as a rolled
    // piece is to a corner's cell (a chamfer's on the seam's normal surface, a round's at its ends)
    for b in &n.beyond {
        let tol = tolerance_over(1.+crate::space::norm(b.ends[0])+b.section.size(),floor);
        pieces.push(match b.section {
            super::spill::Across::Round { .. } => super::spill::round_beyond(b,tol)
                .map_err(|e| format!("a round beyond a seam: {e}"))?,
            super::spill::Across::Flat { .. } => super::spill::flat_beyond(b,tol)
                .map_err(|e| format!("a flat beyond a seam: {e}"))?,
        });
    }
    chained(&pieces,&n.joins,first_rolled,floor)
}

/// One rolled run of a `lowered` node: the ball rolled along it against the `stratum`, or a
/// chamfer's flat cut along it instead (#218's 6), cut at the section of each corner standing at
/// an end of its run, which takes the rest (#218's 7).
fn rolled_run(k: &Rolled,stratum: &Brep,tol: f64,floor: f64) -> Built {
    let mut piece = match k.flat {
        Some((legs,on)) => super::fillet::flat_along(stratum,k.at,legs,Some(on),tol).map_err(|e| format!("a flat along a run: {e}"))?,
        None => super::fillet::roll_across(stratum,k.at,k.radius,tol,&k.wedge)
            .map_err(|e| format!("a rolled round: {e}"))?.piece,
    };
    // the stretch a piece beyond a seam takes (#422): what lies past both planes of the wedge, cut
    // away — a box about the piece kept past each
    if !k.wedge.is_empty() {
        let (lo,hi) = piece.bounds();
        let (middle,half) = (crate::space::lerp(lo,hi,0.5),crate::space::distance(lo,hi));
        let mut block = super::build::prism(&super::build::square(crate::space::sub(middle,[0.,0.,half]),[0.,0.,1.],half),0.,2.*half)?;
        for &(at,normal) in &k.wedge {
            block = kept(&block,at,normal,floor).map_err(|e| format!("a rolled round's wedge: {e}"))?;
        }
        piece = super::boolean::combined(&piece,&block,super::boolean::Op::Cut,floor)
            .map_err(|e| format!("a rolled round cut short of a seam: {e}"))?;
    }
    for &(at,normal,reach) in &k.cuts {
        piece = cut_before(&piece,at,normal,reach,floor).map_err(|e| format!("a rolled round cut at a corner: {e}"))?;
    }
    // short of a seam's normal surface, a piece beyond taking the rest (#422)
    for (surface,inside) in &k.kept {
        piece = match surface {
            super::geom::Surface::Plane(f) => kept(&piece,f.o,if *inside { f.z.map(|x| -x) } else { f.z },floor),
            _ => kept_by_solid(&piece,surface,*inside,floor),
        }.map_err(|e| format!("a flat along a run kept short of a seam: {e}"))?;
    }
    Ok(piece)
}

/// One run of a `lowered` node, a prism or a revolution, cut off where its edge ends: at a slant
/// (`within`), or carried on into a curved face (`kept`).
fn lowered_run(piece: &Run,built: &BTreeMap<i64,Brep>,built_names: &BTreeMap<i64,String>,floor: f64) -> Built {
    let mut b = piece.node.built(built,built_names,floor)?;
    // a run cut off at a slant where its edge ends (a run-out): kept behind each face's
    // plane (through `at`, its `normal` away from the run)
    for &(at,normal) in &piece.within {
        let keep = normal.map(|x| -x);
        b = kept(&b,at,keep,floor).map_err(|e| format!("a run cut off at its end: {e}"))?;
    }
    // one carried on into a curved face it ends on (#218): kept within or outside the
    // cylinder or the ball its surface bounds
    for (surface,inside) in &piece.kept {
        b = kept_by_solid(&b,surface,*inside,floor).map_err(|e| format!("a run cut off at a curved end: {e}"))?;
    }
    Ok(b)
}

/// A lowered region's pieces, those `joins` names glued at their shared sections into chains, each
/// walked from its first piece breadth first at the tolerance of its pieces (looser at a rolled
/// piece's, those from `first_rolled` on), and the chains set beside one another (`beside`).
fn chained(pieces: &[Brep],joins: &[[usize;2]],first_rolled: usize,floor: f64) -> Built {
    if joins.iter().flatten().any(|&k| k >= pieces.len()) { return Err("recipe: a join names no piece".into()) }
    let mut next: Vec<Vec<usize>> = vec![Vec::new();pieces.len()];
    for &[i,j] in joins { next[i].push(j); next[j].push(i); }
    let tol = tolerance_over(pieces.iter().map(Brep::size).fold(1.,f64::max),floor);
    let mut solid: Option<Brep> = None;
    let mut done = vec![false;pieces.len()];
    for first in 0..pieces.len() {
        if done[first] { continue }
        done[first] = true;
        let mut chain = pieces[first].clone();
        let mut rolled = false;
        let mut queue = std::collections::VecDeque::from([first]);
        while let Some(i) = queue.pop_front() {
            for &k in &next[i] {
                if done[k] { continue }
                done[k] = true;
                // a rolled piece's cap is cut from its fitted canal: its edges are the cell's
                // to within the canal's fit (`fillet::roll`'s bar, a ten-millionth of the
                // boundary), and the shared edges are measured once the chain is whole
                let rolled_join = [i,k].iter().any(|&p| p >= first_rolled);
                let bar = if rolled_join { (2e-7*chain.size().max(pieces[k].size())).max(8.*tol) } else { 8.*tol };
                chain = super::boolean::glued(&chain,&pieces[k],bar)
                    .ok_or("two pieces joined at a section share no face there")?;
                rolled |= rolled_join;
                queue.push_back(k);
            }
        }
        if rolled { chain.measure_tolerances(); }
        solid = Some(match solid { None => chain,Some(s) => beside(&s,&chain,floor)? });
    }
    solid.ok_or_else(|| "recipe: a lowered region of no pieces".into())
}

/// Two chains of a lowered region unioned, or — where they only touch, along a line their ends
/// share (a rim's run and a ring's stretch each side of a reflex corner, kept off one another by
/// the walls they end on) — side by side as two shells: their union is no manifold there, but
/// nothing of either lies in the other, so a body taking each takes both.
fn beside(a: &Brep,b: &Brep,floor: f64) -> Built {
    let union = match combined(a,b,Op::Union,floor) { Ok(u) => return Ok(u),Err(e) => e };
    let size = a.size().max(b.size());
    let apart = combined(a,b,Op::Common,floor)
        .is_ok_and(|c| c.faces.is_empty() || super::props::volume(&c).abs() <= 1e-9*size.dpowi(3));
    if apart { Ok(a.joined(b)) } else { Err(union) }
}

/// A body: the stock, plus everything on it, minus everything that cuts it, within everything that
/// bounds it — in that order, as the native host combines them; then what reads the faces that
/// makes (a round's fills, then its cuts: the stratum, #183 F6), where `n` lists them.
fn body(n: &Body,built: &BTreeMap<i64,Brep>,built_names: &BTreeMap<i64,String>,floor: f64) -> Built {
    let mut solid = built.get(&n.stock).ok_or("recipe: `stock` not built before its user")?.clone();
    for ((key,list),op) in n.sets().into_iter().zip([Op::Union,Op::Cut,Op::Common,Op::Union,Op::Cut]) {
        // the late sets (fills, then cuts) are what reads the faces the rest make
        let written = !key.ends_with("_late");
        for x in list {
            let operand = built.get(x).ok_or(format!("recipe: an operand not built before its body"))?;
            let name = built_names.get(x).cloned().unwrap_or_default();
            solid = combined(&solid,operand,op,floor).map_err(|e| {
                // **what the stratum leaves**: each round or chamfer is read off the body as the
                // terms before them leave it, never off another, so two of separate statements
                // meeting at a vertex (a fill and a run of other radii) or across a corner a fill
                // rounds (a rim's run and a ring's stretch) leave material touching itself along a
                // line, which no manifold boundary holds (#197, #218). An open boundary is no such
                // meeting: the kernel lost a face applying the round
                match (written,e.kind) {
                    (false,Kind::NotManifold) => e.saying(|e| format!("{key}: rounds or chamfers of separate statements meet where \
                        the body they each read leaves them touching along a line, not built yet (#218): {e}")),
                    (false,Kind::Open) => e.saying(|e| format!("{key}: the kernel left the body open applying a round or \
                        chamfer to it, a failure of its own: {e}")),
                    _ => e.saying(|e| format!("{key} `{name}`: {e}")),
                }
            })?;
        }
    }
    Ok(solid)
}

/// The recipe's root, built, each face named by its node (`NAMED`): a primitive's by the node's
/// name, a placement's copies by the placement and the route to the primitive in its source —
/// the walk `solid::document::operand_primitives` makes over the solids, made over the nodes, so
/// the names are `solid::exact`'s.
pub fn build(recipe: &Json) -> Result<Brep,String> { Ok(Recipe::read(recipe)?.build()?) }

impl Recipe {
    /// The root, built (`build`).
    pub fn build(&self) -> Built {
        let mut built = BTreeMap::new();
        // each node's primitives, keyed as its faces are named, and their routes from it
        let mut routes: BTreeMap<i64,(String,BTreeMap<String,String>)> = BTreeMap::new();
        for Named {id,name,node} in &self.nodes {
            let mut b = node.built(&built,&BTreeMap::new(),0.).map_err(|e| e.saying(|e| format!("`{name}`: {e}")))?;
            let mut own = BTreeMap::new();
            match node {
                Node::Lowered(_) => {}
                Node::Body(body) => for id in std::iter::once(&body.stock).chain(body.sets().into_iter().flat_map(|(_,l)| l)) {
                    let Some((operand,paths)) = routes.get(id) else { continue };
                    let short = crate::solid::route_step(operand);
                    for (primitive,path) in paths {
                        // a swept solid's own sweep, split off as its stock, adds no step
                        let route = if operand == name { path.clone() } else { placed_name(&short,path) };
                        own.entry(primitive.clone()).or_insert(route);
                    }
                },
                Node::Placed {source,..} => {
                    let source = routes.get(source).map(|(_,p)| p);
                    for f in &mut b.faces {
                        let (of,face) = named_parts(&f.name);
                        let path = source.and_then(|p| p.get(of)).map_or(of,String::as_str);
                        f.name = name_under(&placed_name(name,path),face);
                    }
                    for path in source.into_iter().flat_map(|p| p.values()) {
                        own.insert(placed_name(name,path),path.clone());
                    }
                }
                _ => {
                    for f in &mut b.faces { f.name = name_under(name,&f.name); }
                    own.insert(name.to_string(),String::new());
                }
            }
            routes.insert(*id,(name.to_string(),own));
            built.insert(*id,b);
        }
        built.remove(&self.root).ok_or("recipe: no root".into())
    }
}

/// A route one step further on (`solid::document::placed_name`).
fn placed_name(name: &str,path: &str) -> String {
    if path.is_empty() { name.to_string() } else { format!("{name}.{path}") }
}

impl Recipe {
    /// A recipe whose every operand is a whole revolution about one line, or a ball centred on it,
    /// built as its meridian region turned once about that line: the region in the half-plane of
    /// the line towards `seam`, so every face of revolution starts its parameters there. Each
    /// operand's region is a thin prism of it standing on that half-plane (a ball's, its
    /// half-disc), the bodies combine them by this kernel's Booleans (lines and arcs swept: planes
    /// and cylinders, exact), and the root's cap is the region. A `Kind::Refused` error says why a
    /// recipe is not such a solid, so a caller builds it by its Booleans instead.
    pub fn meridian(&self,seam: V) -> Result<(Brep,V,V),KernelError> {
        let (profile,origin,axis) = meridian_region(self,seam)?;
        Ok((revolve(&profile,origin,axis,std::f64::consts::TAU)?,origin,axis))
    }
}

/// Not of the kind a route builds, saying why (`Kind::Refused`).
pub(super) fn refused(why: impl Into<String>) -> KernelError { KernelError::new(Kind::Refused,why) }

impl Node {
    /// A ball, where the node is one: a whole revolution of a profile of lines on its axis and arcs
    /// of one circle centred on it (`tol` mm) — its centre and its radius.
    pub fn ball(&self,tol: f64) -> Result<Option<(V,f64)>,String> {
        use crate::space::{cross,norm,scale,sub};
        use super::build::ProfileEdge;
        let &Node::Revolve {ref profile,origin,axis,angle} = self else { return Ok(None) };
        if (angle.abs()-std::f64::consts::TAU).abs() > 1e-9 { return Ok(None) }
        let (o,a) = (origin,scale(axis,1./norm(axis)));
        let on = |p: V| norm(cross(sub(p,o),a)) <= tol;
        let mut found: Option<(V,f64)> = None;
        for e in Profile::of(profile)?.loops.iter().flatten() {
            match *e {
                ProfileEdge::Line {a: p,b: q} => if !on(p) || !on(q) { return Ok(None) },
                ProfileEdge::Arc {frame,r,..} => {
                    if !on(frame.o) { return Ok(None) }
                    match found {
                        None => found = Some((frame.o,r)),
                        Some((c,q)) => if norm(sub(c,frame.o)) > tol || (q-r).abs() > tol { return Ok(None) },
                    }
                }
                ProfileEdge::Spline(_) => return Ok(None),
            }
        }
        Ok(found)
    }
}

/// The tolerance a recipe of revolutions is read to: of the reach of its revolutions' lines.
pub(super) fn revolutions_tolerance(nodes: &[Named]) -> f64 {
    let mut size: f64 = 1.;
    for n in nodes {
        if let Node::Revolve {origin,axis,..} = &n.node { size = size.max(crate::space::norm(*origin)+crate::space::norm(*axis)); }
    }
    tolerance(size)
}

/// Whether a revolution turning by `angle` (called `name`) is a whole turn: `Err` refuses it, by
/// name.
pub(super) fn whole_turn(angle: f64,name: &str) -> Result<(),KernelError> {
    if (angle.abs()-std::f64::consts::TAU).abs() > 1e-9 { Err(refused(format!("`{name}` is a partial revolution"))) } else { Ok(()) }
}
/// Why a revolution's profile is no meridian (`meridian_of`).
pub(super) enum Unread {
    /// A spline (one not a conic's arc, where conics are read).
    Spline,
    /// Points of it either side of its line.
    Crosses,
    /// Every point of it within the tolerance of its line.
    OnLine,
}

/// A revolution's profile as its meridian: each loop's edges as steps in the half-plane of the
/// line through `o` along the unit `a` that the profile lies on, in `(r, z)` — along that side
/// from the line, and along the line from `o` — loop by loop and edge by edge as the profile lists
/// them; a spline that is a conic's arc read as that conic where `conics`.
pub(super) fn meridian_of(profile: &Profile,o: V,a: V,tol: f64,conics: bool) -> Result<Vec<Vec<super::planar::Seg>>,Unread> {
    use crate::space::{add,dot,norm,scale,sub,unit};
    use super::build::ProfileEdge;
    use super::geom::Frame;
    use super::planar::Seg;
    // a conic's arc: a rational quadratic Bézier, its ends weighed alike
    let conic = |b: &super::nurbs::BSpline| -> Option<([V;3],f64)> {
        if !conics || b.degree != 2 || b.poles.len() != 3 || b.knots.len() != 6 { return None }
        let w = b.weights.as_deref().unwrap_or(&[1.;3]);
        if (w[0]-w[2]).abs() > 1e-12*w[0] { return None }
        Some(([b.poles[0],b.poles[1],b.poles[2]],w[1]/w[0]))
    };
    // the side of the line every point of it is on
    let mut side: Option<V> = None;
    let arc_at = |frame: &Frame,r: f64,w: f64| add(frame.o,add(scale(frame.x,r*w.dcos()),scale(frame.y,r*w.dsin())));
    for e in profile.loops.iter().flatten() {
        let pts: Vec<V> = match *e {
            ProfileEdge::Line {a: p,b: q} => vec![p,q],
            ProfileEdge::Arc {frame,r,span} => {
                let [a0,a1] = span.unwrap_or([0.,std::f64::consts::TAU]);
                (0..=16).map(|k| arc_at(&frame,r,a0+(a1-a0)*k as f64/16.)).collect()
            }
            ProfileEdge::Spline(ref b) if conic(b).is_some() => b.poles.clone(),
            ProfileEdge::Spline(_) => return Err(Unread::Spline),
        };
        for p in pts {
            let d = sub(p,o);
            let d = sub(d,scale(a,dot(d,a)));
            if norm(d) <= tol { continue }
            let d = unit(d);
            match side { None => side = Some(d), Some(s) => if dot(s,d) < 1.-1e-9 { return Err(Unread::Crosses) } }
        }
    }
    let Some(side) = side else { return Err(Unread::OnLine) };
    let q = |p: V| { let d = sub(p,o); [dot(d,side),dot(d,a)] };
    Ok(profile.loops.iter().map(|l| l.iter().map(|e| match *e {
        ProfileEdge::Line {a: p,b: r} => Seg::Line {a:q(p),b:q(r)},
        ProfileEdge::Arc {frame,r,span} => {
            let [w0,w1] = span.unwrap_or([0.,std::f64::consts::TAU]);
            let c = q(frame.o);
            let p0 = q(arc_at(&frame,r,w0));
            // the frame's turn as the half-plane sees it
            let (x,y) = ([dot(frame.x,side),dot(frame.x,a)],[dot(frame.y,side),dot(frame.y,a)]);
            let sense = (x[0]*y[1]-x[1]*y[0]).signum();
            Seg::Arc {c,r,a0:(p0[1]-c[1]).datan2(p0[0]-c[0]),sweep:sense*(w1-w0)}
        }
        ProfileEdge::Spline(ref b) => {
            let ([a,m,c],w) = conic(b).expect("refused above");
            Seg::Conic {a:q(a),m:q(m),b:q(c),w}
        }
    }).collect()).collect())
}

/// `meridian`'s region: a profile in the half-plane of its line towards `seam` (its outer loop
/// first), with the line's point and direction.
fn meridian_region(recipe: &Recipe,seam: V) -> Result<(Profile,V,V),KernelError> {
    use crate::space::{cross,dot,norm,scale,sub};
    use super::build::ProfileEdge;
    use super::planar::Region;
    let nodes = &recipe.nodes;
    let unit = |a: V| scale(a,1./norm(a));
    let tol = revolutions_tolerance(nodes);
    let ball = |n: &Node| n.ball(tol);
    let mut line = None;
    for Named {node,..} in nodes {
        if let Node::Revolve {origin,axis,..} = node { if ball(node)?.is_none() { line = Some((*origin,unit(*axis))); break } }
    }
    let Some((origin,axis)) = line else { return Err(refused("no revolution of it is about a line of its own")) };
    let on_line = |p: V| norm(cross(sub(p,origin),axis)) <= tol;
    let seam = unit(sub(seam,scale(axis,dot(seam,axis))));
    // the half-plane's coordinates: along `seam` from the line, and along it
    let mut regions: BTreeMap<i64,Region> = BTreeMap::new();
    // each step tagged by its face's name (`NAMED`), one past its place here: the Booleans keep a
    // step's tag
    let mut names: Vec<String> = Vec::new();
    for Named {id,name,node} in nodes {
        let region = match node {
            Node::Revolve {profile,origin: o,axis: a,angle} => {
                whole_turn(*angle,name)?;
                let profile = Profile::of(profile)?;
                if let Some((c,r)) = ball(node)? {
                    if !on_line(c) { return Err(refused(format!("`{name}` is a ball off the line"))) }
                    // its half-disc: from the axis round through the half-plane and back along it
                    let z = dot(sub(c,origin),axis);
                    let arc = (0..profile.loops.len()).flat_map(|l| (0..profile.loops[l].len()).map(move |k| (l,k)))
                        .find(|&(l,k)| matches!(profile.loops[l][k],ProfileEdge::Arc {..})).map_or(String::new(),|(l,k)| profile.name(l,k));
                    names.push(name_under(name,&arc));
                    let tag = names.len() as u32;
                    Region::half_disc(z,r).tagged(&|_,_| tag)
                } else {
                    let a = unit(*a);
                    if norm(cross(a,axis)) > 1e-12 || !on_line(*o) {
                        return Err(refused(format!("`{name}` is a revolution about another line")))
                    }
                    let loops = match meridian_of(&profile,origin,axis,tol,true) {
                        Ok(loops) => loops,
                        Err(Unread::Spline) => return Err(refused(format!("`{name}`'s profile has a spline"))),
                        Err(Unread::Crosses) => return Err(refused(format!("`{name}`'s section crosses its axis"))),
                        Err(Unread::OnLine) => return Err(refused(format!("`{name}` has no section off its axis"))),
                    };
                    let first = names.len() as u32+1;
                    for (l,edges) in profile.loops.iter().enumerate() {
                        for k in 0..edges.len() { names.push(name_under(name,&profile.name(l,k))); }
                    }
                    let mut tag = first;
                    Region::new(loops.into_iter().map(|l| l.into_iter()
                        .map(|seg| { let step = super::planar::Step {seg,chart:None,tag}; tag += 1; step }).collect()).collect())
                }
            }
            Node::Body(b) => {
                let mut region = regions.get(&b.stock).ok_or("recipe: a stock not built before its body")?.clone();
                // (no late set: a body of revolutions has no readers of its faces)
                for ((key,list),op) in b.sets().into_iter().zip([super::planar::Op::Union,super::planar::Op::Cut,super::planar::Op::Common]) {
                    for x in list {
                        let operand = regions.get(x).ok_or("recipe: an operand not built before its body")?;
                        region = super::planar::boolean(&region,operand,op,tol).map_err(|e| format!("`{name}`'s meridian, {key}: {e}"))?;
                    }
                }
                region
            }
            node => return Err(refused(format!("`{name}` is a {}, not a revolution",node.kind()))),
        };
        regions.insert(*id,region);
    }
    let root = regions.remove(&recipe.root).ok_or("recipe: no root")?;
    if root.loops.is_empty() { return Err(refused("the meridian region is empty")) }
    let name = |t: u32| if t == 0 { String::new() } else { names.get(t as usize-1).cloned().unwrap_or_default() };
    Ok((region_profile(&root,origin,seam,axis,&name),origin,axis))
}

/// A meridian region as the profile a revolution turns: back into the half-plane through `origin`
/// along `axis`, towards `seam`, the outer loop first (the one enclosing most), each edge named by
/// its step's tag.
pub(crate) fn region_profile(
    root: &super::planar::Region,
    origin: V,
    seam: V,
    axis: V,
    name: &dyn Fn(u32) -> String,
) -> Profile {
    use crate::space::{add,cross,scale};
    use super::build::ProfileEdge;
    use super::geom::Frame;
    use super::planar::Seg;
    let at = |p: [f64;2]| add(origin,add(scale(seam,p[0]),scale(axis,p[1])));
    let normal = cross(seam,axis);
    let loops = root.outer_first();
    let names = loops.iter().map(|l| l.iter().map(|s| name(s.tag)).collect()).collect();
    let loops = loops.iter().map(|l| l.iter().map(|s| match s.seg {
        Seg::Line {a,b} => ProfileEdge::Line {a:at(a),b:at(b)},
        Seg::Arc {c,r,a0,sweep} => ProfileEdge::Arc {frame:Frame::new(at(c),normal,seam),r,
            span:Some(if sweep > 0. { [a0,a0+sweep] } else { [a0+sweep,a0] })},
        // a conic's arc is the rational quadratic B-spline over the same three points, exactly
        Seg::Conic {a,m,b,w} => ProfileEdge::Spline(std::sync::Arc::new(super::nurbs::BSpline {degree:2,
            knots:vec![0.,0.,0.,1.,1.,1.],poles:vec![at(a),at(m),at(b)],
            weights:(w != 1.).then(|| vec![1.,w,1.])})),
    }).collect()).collect();
    Profile {origin,normal,loops,names}
}
