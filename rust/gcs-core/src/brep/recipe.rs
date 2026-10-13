//! The CAD recipe (`solid::cad::recipe`) built by this kernel: the same millimetre-valued DAG the
//! native host is handed, node by node, so the two kernels can be held to each other.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::boolean::{boolean,Op};
use super::build::{drafted,loft,prism,revolve,Guide,Profile};
use super::geom::{widen,Rigid,V,EMPTY};
use super::topo::Brep;
use crate::json::Json;
use std::collections::BTreeMap;

fn vec3(j: &Json) -> V { let a = j.arr(); [a[0].as_f64(),a[1].as_f64(),a[2].as_f64()] }

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
fn field<'a>(j: &'a Json,k: &str) -> Result<&'a Json,String> { j.get(k).ok_or(format!("recipe: no `{k}`")) }

/// One step of the body rule: `solid` combined with `operand` by `op`, to a tolerance of the two's
/// size (never under `floor`, mm), and checked.
pub fn combined(solid: &Brep,operand: &Brep,op: Op,floor: f64) -> Result<Brep,String> {
    let tol = (1e-9*solid.size().max(operand.size())).max(floor);
    let out = boolean(solid,operand,op,tol)?;
    out.check(10.*tol)?;
    Ok(out)
}

/// One node of a recipe built from the nodes before it; `Err` names what this kernel does not
/// build yet.
pub fn node(n: &Json,built: &BTreeMap<i64,Brep>) -> Result<Brep,String> { node_named(n,built,&BTreeMap::new(),0.) }

/// `node`, naming each operand in what it says, its Booleans' tolerance never under `floor` (mm):
/// the rounding a recipe read off coordinates far from its origin carries (a part solved at 1e9 mm
/// is placed to 1e-7 mm, wherever it is built about).
pub fn node_named(n: &Json,built: &BTreeMap<i64,Brep>,built_names: &BTreeMap<i64,String>,floor: f64) -> Result<Brep,String> {
    let kind = field(n,"kind")?.as_str();
    let source = |k: &str| -> Result<&Brep,String> {
        built.get(&field(n,k)?.as_i64()).ok_or(format!("recipe: `{k}` not built before its user"))
    };
    match kind {
        "prism" => drafted(&Profile::from_json(field(n,"profile")?)?,field(n,"from")?.as_f64(),field(n,"to")?.as_f64(),
            n.get("draft").map_or(0.,Json::as_f64)),
        "revolve" => revolve(&Profile::from_json(field(n,"profile")?)?,vec3(field(n,"origin")?),
            vec3(field(n,"axis")?),field(n,"angle")?.as_f64()),
        "loft" => {
            let g = field(n,"guide")?;
            let guide = match field(g,"kind")?.as_str() {
                "line" => Guide::Line {delta:vec3(field(g,"delta")?)},
                "arc" => Guide::Arc {center:vec3(field(g,"center")?),axis:vec3(field(g,"axis")?),angle:field(g,"angle")?.as_f64()},
                k => return Err(format!("recipe: a guide of kind `{k}`")),
            };
            let end = n.get("end").map(Profile::from_json).transpose()?;
            loft(&Profile::from_json(field(n,"profile")?)?,end.as_ref(),&guide)
        }
        "placed" => {
            let m: Vec<f64> = field(n,"matrix")?.arr().iter().map(Json::as_f64).collect();
            Ok(source("source")?.moved(&Rigid::from_rows(&m)))
        }
        "through" => {
            // a prism spanning its sources' material along the profile's normal (their `bounds`
            // enclose it, issue #59), padded as the native host pads it
            let profile = Profile::from_json(field(n,"profile")?)?;
            let mut sources = EMPTY;
            for s in field(n,"sources")?.arr() {
                widen(&mut sources,built.get(&s.as_i64()).ok_or("recipe: a `through` source not built before it")?.bounds());
            }
            let (lo,hi) = sources;
            let (nrm,o) = (crate::space::scale(profile.normal,1./crate::space::norm(profile.normal)),profile.origin);
            let (mut from,mut to,mut diagonal) = (0.,0.,0.);
            for k in 0..3 {
                let (a,z) = (nrm[k]*(lo[k]-o[k]),nrm[k]*(hi[k]-o[k]));
                from += a.min(z); to += a.max(z);
                diagonal += (hi[k]-lo[k]).dpowi(2);
            }
            let pad = diagonal.sqrt()*4e-5;
            prism(&profile,from-pad,to+pad)
        }
        "body" => body(n,built,built_names,floor),
        "lowered" => {
            // a region set's pieces (`solid::lowering`, #194): each run a prism or a revolution,
            // each corner the ball's patch built where it stands; those joined (a run ending in the section a
            // corner's cell begins in, meeting nowhere else) glued there, never intersected; the
            // chains unioned
            let mut pieces = field(n,"pieces")?.arr().iter().map(|piece| {
                let mut b = node_named(piece,built,built_names,floor)?;
                // a run cut off at a slant where its edge ends (a run-out): kept behind each face's
                // plane (through `at`, its `normal` away from the run)
                for h in piece.get("within").map_or(&[][..],|w| w.arr()) {
                    let keep = vec3(field(h,"normal")?).map(|x| -x);
                    b = kept(&b,vec3(field(h,"at")?),keep,floor).map_err(|e| format!("a run cut off at its end: {e}"))?;
                }
                // one carried on into a curved face it ends on (#218): kept within or outside the
                // cylinder or the ball its surface bounds
                for k in piece.get("kept").map_or(&[][..],|w| w.arr()) {
                    let inside = matches!(field(k,"inside")?,Json::Bool(true));
                    b = kept_by_solid(&b,&surface_of(k)?,inside,floor).map_err(|e| format!("a run cut off at a curved end: {e}"))?;
                }
                Ok::<_,String>(b)
            }).collect::<Result<Vec<_>,_>>()?;
            for c in field(n,"corners")?.arr() {
                let (vertex,centre,radius) = (vec3(field(c,"vertex")?),vec3(field(c,"centre")?),field(c,"radius")?.as_f64());
                let surfaces = field(c,"faces")?.arr().iter().map(surface_of).collect::<Result<Vec<_>,_>>()?;
                let [a,b,s] = &surfaces[..] else { return Err("recipe: a corner stands between three faces".into()) };
                let tol = (1e-9*(1.+radius+crate::space::norm(vertex))).max(floor);
                pieces.push(super::fillet::corner_among(vertex,centre,radius,[a,b,s],tol).map_err(|e| format!("a corner: {e}"))?);
            }
            // a ball rolled along a traced run (#194): against the body the region rounds as its terms
            // but the late ones make it (the stratum), each piece the material it adds or takes
            let rolled = n.get("rolled").map_or(&[][..],|r| r.arr());
            if !rolled.is_empty() {
                let stratum = body(field(n,"stratum")?,built,built_names,floor)?;
                let tol = (1e-9*stratum.size()).max(floor);
                for k in rolled {
                    let (at,r) = (vec3(field(k,"at")?),field(k,"radius")?.as_f64());
                    pieces.push(super::fillet::roll(&stratum,at,r,tol).map_err(|e| format!("a rolled round: {e}"))?.piece);
                }
            }
            let joins: Vec<[usize;2]> = field(n,"joins")?.arr().iter()
                .map(|j| { let j = j.arr(); [j[0].as_i64() as usize,j[1].as_i64() as usize] }).collect();
            if joins.iter().flatten().any(|&k| k >= pieces.len()) { return Err("recipe: a join names no piece".into()) }
            // each chain walked from its first piece, breadth first, at the tolerance of its pieces
            let mut next: Vec<Vec<usize>> = vec![Vec::new();pieces.len()];
            for &[i,j] in &joins { next[i].push(j); next[j].push(i); }
            let tol = (1e-9*pieces.iter().map(Brep::size).fold(1.,f64::max)).max(floor);
            let mut solid: Option<Brep> = None;
            let mut done = vec![false;pieces.len()];
            for first in 0..pieces.len() {
                if done[first] { continue }
                done[first] = true;
                let mut chain = pieces[first].clone();
                let mut queue = std::collections::VecDeque::from([first]);
                while let Some(i) = queue.pop_front() {
                    for &k in &next[i] {
                        if done[k] { continue }
                        done[k] = true;
                        chain = super::boolean::glued(&chain,&pieces[k],8.*tol)
                            .ok_or("two pieces joined at a section share no face there")?;
                        queue.push_back(k);
                    }
                }
                solid = Some(match solid { None => chain,Some(s) => beside(&s,&chain,floor)? });
            }
            solid.ok_or_else(|| "recipe: a lowered region of no pieces".to_string())
        }
        k => Err(format!("recipe: a node of kind `{k}`")),
    }
}

/// Two chains of a lowered region unioned, or — where they only touch, along a line their ends
/// share (a rim's run and a ring's stretch each side of a reflex corner, kept off one another by
/// the walls they end on) — side by side as two shells: their union is no manifold there, but
/// nothing of either lies in the other, so a body taking each takes both.
fn beside(a: &Brep,b: &Brep,floor: f64) -> Result<Brep,String> {
    let union = match combined(a,b,Op::Union,floor) { Ok(u) => return Ok(u),Err(e) => e };
    let size = a.size().max(b.size());
    let apart = combined(a,b,Op::Common,floor)
        .is_ok_and(|c| c.faces.is_empty() || super::props::volume(&c).abs() <= 1e-9*size.dpowi(3));
    if apart { Ok(a.joined(b)) } else { Err(union) }
}

/// A body: the stock, plus everything on it, minus everything that cuts it, within everything that
/// bounds it — in that order, as the native host combines them; then what reads the faces that
/// makes (a round's fills, then its cuts: the stratum, #183 F6), where `n` lists them.
fn body(n: &Json,built: &BTreeMap<i64,Brep>,built_names: &BTreeMap<i64,String>,floor: f64) -> Result<Brep,String> {
    let mut solid = built.get(&field(n,"stock")?.as_i64()).ok_or("recipe: `stock` not built before its user")?.clone();
    for (key,op,written) in [("on",Op::Union,true),("cut",Op::Cut,true),("bound",Op::Common,true),
        ("on_late",Op::Union,false),("cut_late",Op::Cut,false)] {
        // the late sets only where a body has readers of its faces
        let list = match n.get(key) { Some(l) => l.arr(),None if !written => continue,None => field(n,key)?.arr() };
        for x in list {
            let operand = built.get(&x.as_i64()).ok_or(format!("recipe: an operand not built before its body"))?;
            let name = built_names.get(&x.as_i64()).cloned().unwrap_or_default();
            solid = combined(&solid,operand,op,floor).map_err(|e| {
                // **what the stratum leaves**: each round or chamfer is read off the body as the
                // terms before them leave it, never off another, so two of separate statements
                // meeting at a vertex (a fill and a run of other radii) or across a corner a fill
                // rounds (a rim's run and a ring's stretch) leave material touching itself along a
                // line, which no manifold boundary holds (#197, #218)
                if !written && e.starts_with(super::topo::NOT_MANIFOLD) {
                    format!("{key}: rounds or chamfers of separate statements meet where the body they each read leaves \
                        them touching along a line, not built yet (#218): {e}")
                } else { format!("{key} `{name}`: {e}") }
            })?;
        }
    }
    Ok(solid)
}

/// `piece` kept on the side `keep` (a unit normal) of the plane through `o`: in common with a block
/// standing on the plane under the piece's middle, reaching past the piece on every other side.
pub fn kept(piece: &Brep,o: V,keep: V,floor: f64) -> Result<Brep,String> {
    use crate::space::{add,distance,dot,scale,sub};
    use super::build::ProfileEdge;
    let (lo,hi) = piece.bounds();
    if !lo[0].is_finite() { return Err("a piece of nothing cut off at a plane".into()) }
    let middle = scale(add(lo,hi),0.5);
    let foot = sub(middle,scale(keep,dot(sub(middle,o),keep)));
    let size = 2.*distance(lo,hi)+distance(middle,foot);
    let f = super::geom::Frame::about(foot,keep);
    let at = |x: f64,y: f64| f.at([x,y,0.]);
    let corners = [at(-size,-size),at(size,-size),at(size,size),at(-size,size)];
    let edges = (0..4).map(|k| ProfileEdge::Line {a:corners[k],b:corners[(k+1)%4]}).collect();
    let block = prism(&Profile {origin:foot,normal:keep,loops:vec![edges],names:Vec::new()},0.,size)?;
    combined(piece,&block,Op::Common,floor)
}

/// A lowered piece's end face or a corner's face, as `solid::lowering` writes it (millimetres): a
/// plane through `origin` square to `normal`, a cylinder of `radius` about the line through `origin`
/// along `axis`, or a sphere of `radius` about `origin`.
fn surface_of(j: &Json) -> Result<super::geom::Surface,String> {
    use super::geom::{Frame,Surface};
    let radius = || field(j,"radius").map(Json::as_f64);
    Ok(match field(j,"kind")?.as_str() {
        "plane" => Surface::Plane(Frame::about(vec3(field(j,"origin")?),vec3(field(j,"normal")?))),
        "cylinder" => Surface::Cylinder(Frame::about(vec3(field(j,"origin")?),vec3(field(j,"axis")?)),radius()?),
        "cone" => Surface::Cone(Frame::about(vec3(field(j,"origin")?),vec3(field(j,"axis")?)),radius()?,field(j,"half")?.as_f64()),
        "sphere" => Surface::Sphere(Frame::about(vec3(field(j,"origin")?),[0.,0.,1.]),radius()?),
        k => return Err(format!("recipe: a face on a surface of kind `{k}`")),
    })
}

/// `piece` kept within (`inside`) or outside the solid a cylinder, a cone or a sphere bounds: in
/// common with it, or less it, the cylinder or the cone long enough to reach past the piece either
/// way (a cone short of its apex).
pub fn kept_by_solid(piece: &Brep,surface: &super::geom::Surface,inside: bool,floor: f64) -> Result<Brep,String> {
    use crate::space::{add,distance,dot,scale,sub};
    use super::build::ProfileEdge;
    use super::geom::{Frame,Surface};
    let (lo,hi) = piece.bounds();
    if !lo[0].is_finite() { return Err("a piece of nothing cut off at a curved face".into()) }
    let middle = scale(add(lo,hi),0.5);
    let solid = match *surface {
        Surface::Cylinder(f,radius) => {
            // the circle in the plane square to the axis through the piece's middle's foot
            let foot = add(f.o,scale(f.z,dot(sub(middle,f.o),f.z)));
            let reach = distance(lo,hi)+radius;
            let circle = ProfileEdge::Arc {frame:Frame::about(foot,f.z),r:radius,span:None};
            prism(&Profile {origin:foot,normal:f.z,loops:vec![vec![circle]],names:Vec::new()},-reach,reach)?
        }
        Surface::Cone(f,radius,half) => {
            // a frustum about the axis, lofted between its circles either side of the middle's foot
            let h = dot(sub(middle,f.o),f.z);
            let reach = distance(lo,hi)+radius;
            let at = |k: f64| -> Result<Profile,String> {
                let (o,r) = (add(f.o,scale(f.z,h+k)),radius+(h+k)*half.dtan());
                if !(r > 0.) { return Err("a piece kept by a cone reaching past its apex".into()) }
                Ok(Profile {origin:o,normal:f.z,loops:vec![vec![ProfileEdge::Arc {frame:Frame::about(o,f.z),r,span:None}]],
                    names:Vec::new()})
            };
            loft(&at(-reach)?,Some(&at(reach)?),&Guide::Line {delta:scale(f.z,2.*reach)})?
        }
        Surface::Sphere(f,radius) => {
            // a half disc turned once about a line through the centre
            let (top,bottom) = (f.at([0.,radius,0.]),f.at([0.,-radius,0.]));
            let arc = ProfileEdge::Arc {frame:Frame::new(f.o,f.z,f.y),r:radius,span:Some([0.,std::f64::consts::PI])};
            let shut = ProfileEdge::Line {a:bottom,b:top};
            let half = Profile {origin:f.o,normal:f.z,loops:vec![vec![arc,shut]],names:Vec::new()};
            super::build::revolve(&half,f.o,f.y,std::f64::consts::TAU)?
        }
        _ => return Err(format!("a piece kept within a {}",surface.kind())),
    };
    combined(piece,&solid,if inside { Op::Common } else { Op::Cut },floor)
}

/// The recipe's root, built, each face named by its node (`NAMED`): a primitive's by the node's
/// name, a placement's copies by the placement and the route to the primitive in its source —
/// the walk `solid::document::operand_primitives` makes over the solids, made over the nodes, so
/// the names are `solid::exact`'s.
pub fn build(recipe: &Json) -> Result<Brep,String> {
    let mut built = BTreeMap::new();
    // each node's primitives, keyed as its faces are named, and their routes from it
    let mut routes: BTreeMap<i64,(String,BTreeMap<String,String>)> = BTreeMap::new();
    for n in field(recipe,"nodes")?.arr() {
        let name = n.get("name").map_or("",|j| j.as_str());
        let mut b = node(n,&built).map_err(|e| format!("`{name}`: {e}"))?;
        let mut own = BTreeMap::new();
        match field(n,"kind")?.as_str() {
            "lowered" => {}
            "body" => for key in ["stock","on","cut","bound","on_late","cut_late"] {
                let ids = match n.get(key) { Some(Json::Arr(l)) => l.iter().map(Json::as_i64).collect(),
                    Some(x) => vec![x.as_i64()], None => vec![] };
                for id in ids {
                    let Some((operand,paths)) = routes.get(&id) else { continue };
                    let short = operand.rsplit('.').next().unwrap_or(operand);
                    for (primitive,path) in paths {
                        // a swept solid's own sweep, split off as its stock, adds no step
                        let route = if operand == name { path.clone() } else { placed_name(short,path) };
                        own.entry(primitive.clone()).or_insert(route);
                    }
                }
            },
            "placed" => {
                let source = routes.get(&field(n,"source")?.as_i64()).map(|(_,p)| p);
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
        let id = field(n,"id")?.as_i64();
        routes.insert(id,(name.to_string(),own));
        built.insert(id,b);
    }
    built.remove(&field(recipe,"root")?.as_i64()).ok_or("recipe: no root".into())
}

/// A route one step further on (`solid::document::placed_name`).
fn placed_name(name: &str,path: &str) -> String {
    if path.is_empty() { name.to_string() } else { format!("{name}.{path}") }
}

/// A recipe whose every operand is a whole revolution about one line, or a ball centred on it,
/// built as its meridian region turned once about that line: the region in the half-plane of the
/// line towards `seam`, so every face of revolution starts its parameters there. Each operand's
/// region is a thin prism of it standing on that half-plane (a ball's, its half-disc), the bodies
/// combine them by this kernel's Booleans (lines and arcs swept: planes and cylinders, exact), and
/// the root's cap is the region. `Ok(Err)` names why a recipe is not such a solid, so a caller
/// builds it by its Booleans instead.
pub fn meridian(recipe: &Json,seam: V) -> Result<Result<(Brep,V,V),String>,String> {
    Ok(match meridian_region(recipe,seam)? {
        Ok((profile,origin,axis)) => Ok((revolve(&profile,origin,axis,std::f64::consts::TAU)?,origin,axis)),
        Err(e) => Err(e),
    })
}

/// `meridian`'s region: a profile in the half-plane of its line towards `seam` (its outer loop
/// first), with the line's point and direction.
pub fn meridian_region(recipe: &Json,seam: V) -> Result<Result<(Profile,V,V),String>,String> {
    use crate::space::{add,cross,dot,norm,scale,sub};
    use super::build::ProfileEdge;
    use super::geom::Frame;
    use super::planar::{Region,Seg};
    let nodes = field(recipe,"nodes")?.arr();
    let unit = |a: V| scale(a,1./norm(a));
    let mut size: f64 = 1.;
    for n in nodes.iter().filter(|n| n.get("kind").is_some_and(|k| k.as_str() == "revolve")) {
        size = size.max(norm(vec3(field(n,"origin")?))+norm(vec3(field(n,"axis")?)));
    }
    let tol = 1e-9*size;
    // a ball: a revolution of a profile of lines on its axis and arcs of one circle centred on it
    let ball = |n: &Json| -> Result<Option<(V,f64)>,String> {
        let (o,a) = (vec3(field(n,"origin")?),unit(vec3(field(n,"axis")?)));
        let on = |p: V| norm(cross(sub(p,o),a)) <= tol;
        let mut found: Option<(V,f64)> = None;
        for e in Profile::from_json(field(n,"profile")?)?.loops.iter().flatten() {
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
    };
    // a conic's arc: a rational quadratic Bézier, its ends weighed alike
    let conic = |b: &super::nurbs::BSpline| -> Option<([V;3],f64)> {
        if b.degree != 2 || b.poles.len() != 3 || b.knots.len() != 6 { return None }
        let w = b.weights.as_deref().unwrap_or(&[1.;3]);
        if (w[0]-w[2]).abs() > 1e-12*w[0] { return None }
        Some(([b.poles[0],b.poles[1],b.poles[2]],w[1]/w[0]))
    };
    let mut line = None;
    for n in nodes {
        if field(n,"kind")?.as_str() == "revolve" && ball(n)?.is_none() { line = Some((vec3(field(n,"origin")?),unit(vec3(field(n,"axis")?)))); break }
    }
    let Some((origin,axis)) = line else { return Ok(Err("no revolution of it is about a line of its own".into())) };
    let on_line = |p: V| norm(cross(sub(p,origin),axis)) <= tol;
    let seam = unit(sub(seam,scale(axis,dot(seam,axis))));
    // the half-plane's coordinates: along `seam` from the line, and along it
    let mut regions: BTreeMap<i64,Region> = BTreeMap::new();
    // each step tagged by its face's name (`NAMED`), one past its place here: the Booleans keep a
    // step's tag
    let mut names: Vec<String> = Vec::new();
    for n in nodes {
        let name = n.get("name").map_or("",|j| j.as_str());
        let region = match field(n,"kind")?.as_str() {
            "revolve" => {
                if (field(n,"angle")?.as_f64().abs()-std::f64::consts::TAU).abs() > 1e-9 { return Ok(Err(format!("`{name}` is a partial revolution"))) }
                if let Some((c,r)) = ball(n)? {
                    if !on_line(c) { return Ok(Err(format!("`{name}` is a ball off the line"))) }
                    // its half-disc: from the axis round through the half-plane and back along it
                    let z = dot(sub(c,origin),axis);
                    let profile = Profile::from_json(field(n,"profile")?)?;
                    let arc = (0..profile.loops.len()).flat_map(|l| (0..profile.loops[l].len()).map(move |k| (l,k)))
                        .find(|&(l,k)| matches!(profile.loops[l][k],ProfileEdge::Arc {..})).map_or(String::new(),|(l,k)| profile.name(l,k));
                    names.push(name_under(name,&arc));
                    let tag = names.len() as u32;
                    Region::half_disc(z,r).tagged(&|_,_| tag)
                } else {
                    let (o,a) = (vec3(field(n,"origin")?),unit(vec3(field(n,"axis")?)));
                    if norm(cross(a,axis)) > 1e-12 || !on_line(o) { return Ok(Err(format!("`{name}` is a revolution about another line"))) }
                    let profile = Profile::from_json(field(n,"profile")?)?;
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
                        ProfileEdge::Spline(_) => return Ok(Err(format!("`{name}`'s profile has a spline"))),
                        };
                        for p in pts {
                            let d = sub(p,origin);
                            let d = sub(d,scale(axis,dot(d,axis)));
                            if norm(d) <= tol { continue }
                            let d = unit(d);
                            match side { None => side = Some(d), Some(s) => if dot(s,d) < 1.-1e-9 { return Ok(Err(format!("`{name}`'s section crosses its axis"))) } }
                        }
                    }
                    let Some(side) = side else { return Ok(Err(format!("`{name}` has no section off its axis"))) };
                    let q = |p: V| { let d = sub(p,origin); [dot(d,side),dot(d,axis)] };
                    let first = names.len() as u32+1;
                    for (l,edges) in profile.loops.iter().enumerate() {
                        for k in 0..edges.len() { names.push(name_under(name,&profile.name(l,k))); }
                    }
                    let mut tag = first;
                    Region::new(profile.loops.iter().map(|l| l.iter().map(|e| match *e {
                        ProfileEdge::Line {a: p,b: r} => Seg::Line {a:q(p),b:q(r)},
                        ProfileEdge::Arc {frame,r,span} => {
                            let [w0,w1] = span.unwrap_or([0.,std::f64::consts::TAU]);
                            let c = q(frame.o);
                            let p0 = q(arc_at(&frame,r,w0));
                            // the frame's turn as the half-plane sees it
                            let (x,y) = ([dot(frame.x,side),dot(frame.x,axis)],[dot(frame.y,side),dot(frame.y,axis)]);
                            let sense = (x[0]*y[1]-x[1]*y[0]).signum();
                            Seg::Arc {c,r,a0:(p0[1]-c[1]).datan2(p0[0]-c[0]),sweep:sense*(w1-w0)}
                        }
                        ProfileEdge::Spline(ref b) => {
                            let ([a,m,c],w) = conic(b).expect("refused above");
                            Seg::Conic {a:q(a),m:q(m),b:q(c),w}
                        }
                    }).map(|seg| { let step = super::planar::Step {seg,chart:None,tag}; tag += 1; step }).collect()).collect())
                }
            }
            "body" => {
                let mut region = regions.get(&field(n,"stock")?.as_i64()).ok_or("recipe: a stock not built before its body")?.clone();
                for (key,op) in [("on",super::planar::Op::Union),("cut",super::planar::Op::Cut),("bound",super::planar::Op::Common)] {
                    for x in field(n,key)?.arr() {
                        let operand = regions.get(&x.as_i64()).ok_or("recipe: an operand not built before its body")?;
                        region = super::planar::boolean(&region,operand,op,tol).map_err(|e| format!("`{name}`'s meridian, {key}: {e}"))?;
                    }
                }
                region
            }
            kind => return Ok(Err(format!("`{name}` is a {kind}, not a revolution"))),
        };
        regions.insert(field(n,"id")?.as_i64(),region);
    }
    let root = regions.remove(&field(recipe,"root")?.as_i64()).ok_or("recipe: no root")?;
    if root.loops.is_empty() { return Ok(Err("the meridian region is empty".into())) }
    let name = |t: u32| if t == 0 { String::new() } else { names.get(t as usize-1).cloned().unwrap_or_default() };
    Ok(Ok((region_profile(&root,origin,seam,axis,&name),origin,axis)))
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
