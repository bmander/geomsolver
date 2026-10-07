//! The CAD recipe (`solid::cad::recipe`) built by this kernel: the same millimetre-valued DAG the
//! native host is handed, node by node, so the two kernels can be held to each other.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::boolean::{boolean,Op};
use super::build::{loft,prism,revolve,Guide,Profile};
use super::geom::{widen,Rigid,V,EMPTY};
use super::topo::Brep;
use crate::json::Json;
use std::collections::BTreeMap;

fn vec3(j: &Json) -> V { let a = j.arr(); [a[0].as_f64(),a[1].as_f64(),a[2].as_f64()] }
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
        "prism" => prism(&Profile::from_json(field(n,"profile")?)?,field(n,"from")?.as_f64(),field(n,"to")?.as_f64()),
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
        "body" => {
            // the stock, plus everything on it, minus everything that cuts it, within everything
            // that bounds it — in that order, as the native host combines them
            let mut solid = source("stock")?.clone();
            for (key,op) in [("on",Op::Union),("cut",Op::Cut),("bound",Op::Common)] {
                for x in field(n,key)?.arr() {
                    let operand = built.get(&x.as_i64()).ok_or(format!("recipe: an operand not built before its body"))?;
                    let name = built_names.get(&x.as_i64()).cloned().unwrap_or_default();
                    solid = combined(&solid,operand,op,floor).map_err(|e| format!("{key} `{name}`: {e}"))?;
                }
            }
            Ok(solid)
        }
        "fillet" => {
            // a fillet's pieces, one per edge it rounds, each a prism or a revolution: those its
            // derivation joined (a chain's, each ending in the section the next begins in, meeting
            // nowhere else) glued there, a cap to a cap, never intersected; the chains unioned
            let mut pieces = field(n,"pieces")?.arr().iter().map(|piece| node_named(piece,built,built_names,floor)
                .map_err(|e| format!("fillet: {e}"))).collect::<Result<Vec<_>,_>>()?;
            // a corner's patch of the ball, after the pieces (joins name it so)
            for c in n.get("corners").map(Json::arr).unwrap_or_default() {
                let (vertex,centre,radius) = (vec3(field(c,"vertex")?),vec3(field(c,"centre")?),field(c,"radius")?.as_f64());
                let toward: Vec<V> = field(c,"toward")?.arr().iter().map(vec3).collect();
                let toward: [V;3] = toward.try_into().map_err(|_| "recipe: a corner stands between three faces")?;
                let tol = (1e-9*(1.+radius)).max(floor);
                pieces.push(super::fillet::corner(vertex,centre,radius,toward,tol).map_err(|e| format!("fillet: a corner: {e}"))?);
            }
            let joins: Vec<[usize;2]> = n.get("joins").map(Json::arr).unwrap_or_default().iter()
                .map(|j| { let j = j.arr(); [j[0].as_i64() as usize,j[1].as_i64() as usize] }).collect();
            if joins.iter().flatten().any(|&k| k >= pieces.len()) { return Err("recipe: a fillet's join names no piece".into()) }
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
                            .ok_or("fillet: two pieces joined at a section share no face there")?;
                        queue.push_back(k);
                    }
                }
                solid = Some(match solid { None => chain,Some(s) => combined(&s,&chain,Op::Union,floor)? });
            }
            // a ball rolled along a traced loop of the operands' union, picked by a point of it (the
            // loops of one fillet share their operands: their union is made once)
            let mut unions: Vec<(Vec<i64>,Brep)> = Vec::new();
            for rolled in n.get("rolled").map(Json::arr).unwrap_or_default() {
                let ids: Vec<i64> = field(rolled,"operands")?.arr().iter().map(|id| id.as_i64()).collect();
                if !unions.iter().any(|(k,_)| *k == ids) {
                    let mut union: Option<Brep> = None;
                    for id in &ids {
                        let b = built.get(id).ok_or("recipe: a fillet's operand not built before it")?;
                        union = Some(match union { None => b.clone(),Some(s) => combined(&s,b,Op::Union,floor)? });
                    }
                    unions.push((ids.clone(),union.ok_or("recipe: a rolled fillet with no operands")?));
                }
                let union = &unions.iter().find(|(k,_)| *k == ids).unwrap().1;
                let tol = (1e-9*union.size().max(1.)).max(floor);
                let b = super::fillet::roll(union,vec3(field(rolled,"point")?),field(rolled,"radius")?.as_f64(),tol)
                    .map_err(|e| format!("fillet: {e}"))?.piece;
                solid = Some(match solid { None => b,Some(s) => combined(&s,&b,Op::Union,floor)? });
            }
            solid.ok_or_else(|| "recipe: a fillet with no pieces".to_string())
        }
        k => Err(format!("recipe: a node of kind `{k}`")),
    }
}

/// The recipe's root, built.
pub fn build(recipe: &Json) -> Result<Brep,String> {
    let mut built = BTreeMap::new();
    for n in field(recipe,"nodes")?.arr() {
        let b = node(n,&built).map_err(|e| format!("`{}`: {e}",n.get("name").map_or("",|j| j.as_str())))?;
        built.insert(field(n,"id")?.as_i64(),b);
    }
    built.remove(&field(recipe,"root")?.as_i64()).ok_or("recipe: no root".into())
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
    let mut line = None;
    for n in nodes {
        if field(n,"kind")?.as_str() == "revolve" && ball(n)?.is_none() { line = Some((vec3(field(n,"origin")?),unit(vec3(field(n,"axis")?)))); break }
    }
    let Some((origin,axis)) = line else { return Ok(Err("no revolution of it is about a line of its own".into())) };
    let on_line = |p: V| norm(cross(sub(p,origin),axis)) <= tol;
    let seam = unit(sub(seam,scale(axis,dot(seam,axis))));
    // the half-plane's coordinates: along `seam` from the line, and along it
    let mut regions: BTreeMap<i64,Region> = BTreeMap::new();
    for n in nodes {
        let name = n.get("name").map_or("",|j| j.as_str());
        let region = match field(n,"kind")?.as_str() {
            "revolve" => {
                if (field(n,"angle")?.as_f64().abs()-std::f64::consts::TAU).abs() > 1e-9 { return Ok(Err(format!("`{name}` is a partial revolution"))) }
                if let Some((c,r)) = ball(n)? {
                    if !on_line(c) { return Ok(Err(format!("`{name}` is a ball off the line"))) }
                    // its half-disc: from the axis round through the half-plane and back along it
                    let z = dot(sub(c,origin),axis);
                    Region::plain(vec![vec![Seg::Arc {c:[0.,z],r,a0:-std::f64::consts::FRAC_PI_2,sweep:std::f64::consts::PI},
                        Seg::Line {a:[0.,z+r],b:[0.,z-r]}]])
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
                    Region::plain(profile.loops.iter().map(|l| l.iter().map(|e| match *e {
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
                        ProfileEdge::Spline(_) => unreachable!("refused above"),
                    }).collect()).collect())
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
    // back into the half-plane's space, the outer loop first (the one enclosing most)
    let at = |p: [f64;2]| add(origin,add(scale(seam,p[0]),scale(axis,p[1])));
    let normal = cross(seam,axis);
    let mut loops: Vec<(f64,Vec<ProfileEdge>)> = root.loops.iter().map(|l| {
        let area = Region {loops:vec![l.clone()]}.area().abs();
        (area,l.iter().map(|s| match s.seg {
            Seg::Line {a,b} => ProfileEdge::Line {a:at(a),b:at(b)},
            Seg::Arc {c,r,a0,sweep} => ProfileEdge::Arc {frame:Frame::new(at(c),normal,seam),r,
                span:Some(if sweep > 0. { [a0,a0+sweep] } else { [a0+sweep,a0] })},
        }).collect())
    }).collect();
    loops.sort_by(|a,b| b.0.total_cmp(&a.0));
    Ok(Ok((Profile {origin,normal,loops:loops.into_iter().map(|l| l.1).collect(),names:Vec::new()},origin,axis)))
}
