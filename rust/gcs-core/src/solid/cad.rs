//! Solved analytic construction data for a native CAD host. No meshed surfaces,
//! filesystem access or kernel dependency enters the core through this interface.
use crate::{json::{object,Json},model::{EntKind,EntRef,Sense,Sketch,SolidDef},plane};
use std::collections::BTreeSet;

/// How far from a revolution's axis an endpoint is read as on it, whenever an export reads the
/// solved model into fields, contacts or its admission (the tolerance a revolved snapshot takes).
pub const AXIS_TOLERANCE: f64 = 1e-10;
/// Poses a material evaluator caches per sweep while an export judges its cells and its mesh.
pub const POSE_CACHE: usize = 4096;

/// Model length units to millimetres, which every native export is written in.
pub fn millimetres(sk: &Sketch) -> Result<f64,String> {
    Ok(sk.units.length.ok_or("CAD export requires an explicit model length unit")?.1)
}

fn vector(v: [f64;3]) -> Json { Json::Arr(v.into_iter().map(Json::from).collect()) }
fn ids(v: &[u32]) -> Json { Json::Arr(v.iter().copied().map(Json::from).collect()) }

/// Row-major 3x4 native placement. Rotation is dimensionless; only the
/// translation changes from model length units to millimetres.
pub fn placement_matrix(pose: crate::envelope::Motion,scale: f64) -> [f64;12] {
    let columns: [[f64;3];3] = std::array::from_fn(|i| {
        let mut v = [0.;3]; v[i] = 1.; pose.vector(v)
    });
    let origin = pose.point([0.;3]);
    std::array::from_fn(|i| {
        let (row,column) = (i/4,i%4);
        if column == 3 { origin[row]*scale } else { columns[column][row] }
    })
}

/// An immutable, millimetre-valued DAG of the selected solid's current solved
/// geometry. Unsupported operations fail before a host receives a partial recipe.
/// Native construction must additionally validate its analytic profiles and solids.
pub fn recipe(sk: &Sketch,root: usize) -> Result<Json,String> {
    Ok(recipe_with(sk,root,false)?.recipe)
}

/// One continuous sweep a static recipe left out of a body's cuts: the swept
/// solid and the pose (model units) at which the body subtracts it, composed
/// from the placements between the body and the sweep.
#[derive(Clone,Debug)]
pub struct SweptCut {
    pub swept: usize,
    pub pose: crate::envelope::Motion,
}

/// A recipe with every cut that is a continuous sweep, or a placed copy of one,
/// left out and listed, so a host that constructs swept boundaries another way
/// can build the static remainder first. The root and its stock must be static,
/// and a sweep may not be added to a body, since material this host classifies
/// is only ever removed from a static blank.
#[derive(Clone,Debug)]
pub struct StaticRecipe {
    pub recipe: Json,
    pub sweeps: Vec<SweptCut>,
}

pub fn recipe_static(sk: &Sketch,root: usize) -> Result<StaticRecipe,String> {
    recipe_with(sk,root,true)
}

pub(crate) fn contains_sweep(sk: &Sketch,i: usize) -> bool {
    matches!(sk.solids[i].def,SolidDef::Swept {..})
        || sk.solids[i].operands().into_iter().any(|o| contains_sweep(sk,o as usize))
}

/// A cut operand that is a sweep or a chain of placements ending in one.
pub(crate) fn swept_cut(sk: &Sketch,i: usize) -> Result<Option<SweptCut>,String> {
    let solid = &sk.solids[i];
    match &solid.def {
        SolidDef::Swept {source,..} => {
            if contains_sweep(sk,*source as usize) {
                return Err(format!("`{}`: nested continuous sweeps are not yet supported",solid.name));
            }
            Ok(Some(SweptCut {swept:i,pose:crate::envelope::Motion::identity()}))
        }
        SolidDef::Placed {source,motion,at} => match swept_cut(sk,*source as usize)? {
            Some(inner) => {
                let outer = crate::motion::Family::read(sk,*motion as usize)?.at(at.value)?;
                Ok(Some(SweptCut {swept:inner.swept,pose:inner.pose.then(outer)}))
            }
            None => Ok(None),
        },
        _ if contains_sweep(sk,i) => Err(format!(
            "`{}`: a continuous sweep may only be cut from a body directly or through placements",solid.name)),
        _ => Ok(None),
    }
}

fn recipe_with(sk: &Sketch,root: usize,static_only: bool) -> Result<StaticRecipe,String> {
    super::validate(sk,root)?;
    let scale = millimetres(sk)?;
    let mut pending = vec![(root,false)];
    let mut seen = BTreeSet::new();
    let mut nodes = Vec::new();
    let mut sweeps = Vec::new();
    while let Some((i,ready)) = pending.pop() {
        if seen.contains(&i) { continue; }
        let mut dependencies = super::evaluation_operands(sk,i)?;
        // The static remainder of a body: its swept cuts are listed, not built.
        let mut kept_cuts = None;
        if static_only { if let SolidDef::Body {stock,on,through,bound} = &sk.solids[i].def {
            if contains_sweep(sk,*stock as usize) {
                return Err(format!("`{}`: a body's stock may not contain a continuous sweep",sk.solids[i].name));
            }
            if let Some(&o) = on.iter().chain(bound).find(|&&o| contains_sweep(sk,o as usize)) {
                return Err(format!("`{}`: only a `cut` may hold swept material for now (`{}`)",
                    sk.solids[i].name,sk.solids[o as usize].name));
            }
            let mut kept = Vec::new();
            for &cut in through {
                match swept_cut(sk,cut as usize)? {
                    // The node is visited twice (dependencies first); list each
                    // sweep once, on the emitting visit.
                    Some(sweep) => if ready { sweeps.push(sweep) },
                    None => kept.push(cut),
                }
            }
            dependencies.retain(|d| !through.contains(d) || kept.contains(d));
            kept_cuts = Some(kept);
        } }
        if !ready {
            pending.push((i,true));
            pending.extend(dependencies.iter().rev().map(|&i| (i as usize,false)));
            continue;
        }
        let solid = &sk.solids[i];
        let mut node = match &solid.def {
            SolidDef::Swept {..} => return Err(format!(
                "`{}`: native CAD boundary construction for continuous motion sweeps is not yet supported",solid.name)),
            SolidDef::Body {stock,on,through,bound} if kept_cuts.is_some() => object([
                ("kind","body".into()),("stock",(*stock).into()),
                ("on",ids(on)),("cut",ids(kept_cuts.as_deref().unwrap_or(through))),("bound",ids(bound)),
            ]),
            SolidDef::Placed { source, motion, at } => {
                let pose = crate::motion::Family::read(sk,*motion as usize)?.at(at.value)?;
                let matrix = placement_matrix(pose,scale).into_iter().map(Json::from).collect();
                object([("kind","placed".into()),("source",(*source).into()),
                    ("matrix",Json::Arr(matrix))])
            }
            SolidDef::Body {stock,on,through,bound} => object([
                ("kind","body".into()),("stock",(*stock).into()),
                ("on",ids(on)),("cut",ids(through)),("bound",ids(bound)),
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
    Ok(StaticRecipe {recipe:object([("schema",1.into()),("units","mm".into()),
        ("root",root.into()),("nodes",Json::Arr(nodes))]),sweeps})
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
