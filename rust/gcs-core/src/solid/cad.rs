//! Solved analytic construction data for a native CAD host. No meshed surfaces,
//! filesystem access or kernel dependency enters the core through this interface.
use crate::{json::{object,Json},model::{EntKind,EntRef,Sense,Sketch,SolidDef},plane};
use std::collections::BTreeSet;

fn vector(v: [f64;3]) -> Json { Json::Arr(v.into_iter().map(Json::from).collect()) }
fn ids(v: &[u32]) -> Json { Json::Arr(v.iter().copied().map(Json::from).collect()) }

/// An immutable, millimetre-valued DAG of the selected solid's current solved
/// geometry. Unsupported operations fail before a host receives a partial recipe.
/// Native construction must additionally validate its analytic profiles and solids.
pub fn recipe(sk: &Sketch,root: usize) -> Result<Json,String> {
    super::validate(sk,root)?;
    let scale = sk.units.length.ok_or("CAD export requires an explicit model length unit")?.1;
    let mut pending = vec![(root,false)];
    let mut seen = BTreeSet::new();
    let mut nodes = Vec::new();
    while let Some((i,ready)) = pending.pop() {
        if seen.contains(&i) { continue; }
        let dependencies = super::evaluation_operands(sk,i)?;
        if !ready {
            pending.push((i,true));
            pending.extend(dependencies.iter().rev().map(|&i| (i as usize,false)));
            continue;
        }
        let solid = &sk.solids[i];
        let mut node = match &solid.def {
            SolidDef::Body {stock,on,through} => object([
                ("kind","body".into()),("stock",(*stock).into()),
                ("on",ids(on)),("cut",ids(through)),
            ]),
            SolidDef::Prism {face,from,to} => object([
                ("kind","prism".into()),("profile",profile(sk,*face as usize,scale)?),
                ("from",(from.value*scale).into()),("to",(to.value*scale).into()),
            ]),
            SolidDef::Through {face,..} => object([
                ("kind","through".into()),("profile",profile(sk,*face as usize,scale)?),
                ("sources",ids(&dependencies)),
            ]),
            SolidDef::Revolve {face,axis,sweep,sense} => {
                // Use the profile's frame for both axis endpoints, as the solid
                // evaluator does; a line need not have its own plane membership.
                let p = super::face_poly(sk,*face as usize,super::REPORT_UNIT)
                    .ok_or("invalid revolution profile")?;
                let lift = |i| {
                    let q = plane::in_view(p.pose.0,p.pose.1,p.pose.2,sk.point_xy(i));
                    p.basis.lift(q.0,q.1)
                };
                let line = &sk.lines[*axis as usize];
                let a = lift(line.p1 as usize);
                let b = lift(line.p2 as usize);
                object([
                    ("kind","revolve".into()),("profile",profile(sk,*face as usize,scale)?),
                    ("origin",vector(a.map(|v| v*scale))),
                    ("axis",vector(std::array::from_fn(|k| b[k]-a[k]))),
                    ("angle",(sweep.value.min(std::f64::consts::TAU)
                        * if *sense == Sense::Cw { -1. } else { 1. }).into()),
                ])
            }
            SolidDef::Loft {..} => return Err(format!(
                "`{}`: native CAD export does not yet support along-guide lofts",solid.name)),
        };
        node.set("id",i.into());
        node.set("name",solid.name.clone().into());
        nodes.push(node);
        seen.insert(i);
    }
    Ok(object([("schema",1.into()),("units","mm".into()),
        ("root",root.into()),("nodes",Json::Arr(nodes))]))
}

fn profile(sk: &Sketch,index: usize,scale: f64) -> Result<Json,String> {
    let face = &sk.faces[index];
    let p = super::face_poly(sk,index,super::REPORT_UNIT).ok_or("invalid CAD profile")?;
    let lift = |q| {
        let q = plane::in_view(p.pose.0,p.pose.1,p.pose.2,q);
        p.basis.lift(q.0,q.1).map(|v| v*scale)
    };
    let x_dir = std::array::from_fn(|k| p.pose.0*p.basis.u[k]-p.pose.1*p.basis.v[k]);
    let edge = |e: EntRef| -> Result<Json,String> {
        Ok(match e.kind {
            EntKind::Line => {
                let l = &sk.lines[e.i()];
                object([("kind","line".into()),
                    ("start",vector(lift(sk.point_xy(l.p1 as usize)))),
                    ("end",vector(lift(sk.point_xy(l.p2 as usize))))])
            }
            EntKind::Circle | EntKind::Arc => {
                let (center,radius,angles) = if e.kind == EntKind::Circle {
                    let c = &sk.circles[e.i()];
                    (c.center,c.radius,None)
                } else {
                    let a = &sk.arcs[e.i()];
                    (a.center,a.radius,Some(sk.arc_angles(e.i())))
                };
                let mut result = object([("kind","circle".into()),
                    ("center",vector(lift(sk.point_xy(center as usize)))),
                    ("normal",vector(p.basis.normal())),("x_dir",vector(x_dir)),
                    ("radius",(sk.params[radius as usize].value*scale).into())]);
                if let Some((start,end)) = angles {
                    result.set("angles",Json::Arr(vec![start.into(),end.into()]));
                }
                result
            }
            _ => return Err(format!("`{}`: CAD profiles currently require lines, arcs or circles",face.name)),
        })
    };
    let loops = face.boundaries().map(|(edges,_)|
        edges.iter().copied().map(edge).collect::<Result<Vec<_>,_>>().map(Json::Arr))
        .collect::<Result<Vec<_>,_>>()?;
    Ok(object([("loops",Json::Arr(loops)),("origin",vector(p.basis.o.map(|v| v*scale))),
        ("normal",vector(p.basis.normal()))]))
}
