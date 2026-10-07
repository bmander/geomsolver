//! Solved analytic construction data for a native CAD host. No meshed surfaces,
//! filesystem access or kernel dependency enters the core through this interface.
use crate::{json::{object,Json},model::{EntKind,EntRef,Sense,Sketch,SolidDef}};
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

/// The operands of body `i` that hold swept material where a sweep is not cut: its stock, what is
/// put on it and what bounds it. A body with any is built from swept material (`drill :=
/// solid(fluted)`), not one a sweep is cut from.
pub(crate) fn swept_operands(sk: &Sketch,i: usize) -> Vec<usize> {
    let SolidDef::Body {stock,on,bound,..} = &sk.solids[i].def else { return Vec::new() };
    std::iter::once(stock).chain(on).chain(bound).map(|&o| o as usize).filter(|&o| contains_sweep(sk,o)).collect()
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
            // each edge's piece, a prism or a revolution of its section: a recipe a kernel builds
            // as it builds any other, unioned
            SolidDef::Fillet {..} => {
                let blend = sk.fillet_blend(i).map_err(|e| format!("`{}`: {e}",solid.name))?;
                // a ball rolled along a traced loop is rolled again by the kernel building it, from
                // the operands it rounds, the loop picked by a point of it
                let operands = ids(&super::fillet::operands(sk,i));
                let rolled = blend.rolls.iter().map(|roll| object([("operands",operands.clone()),
                    ("radius",(roll.rolled.r/roll.mm*scale).into()),
                    ("point",vector(roll.edge_point().map(|v| v*scale))),("concave",roll.rolled.concave.into())])).collect();
                // a corner, the ball's patch there, built by the kernel from where it stands
                let corners = blend.corners.iter().map(|c| object([("vertex",vector(c.vertex.map(|v| v*scale))),
                    ("centre",vector(c.centre.map(|v| v*scale))),("radius",(c.r*scale).into()),
                    ("toward",Json::Arr(c.toward.iter().map(|&m| vector(m)).collect()))])).collect();
                object([("kind","fillet".into()),
                    ("pieces",Json::Arr(blend.pieces.iter().map(|p| fillet_piece(p,scale)).collect())),
                    ("corners",Json::Arr(corners)),
                    ("joins",Json::Arr(blend.joins.iter().map(|j| Json::Arr(j.iter().map(|&k| k.into()).collect())).collect())),
                    ("rolled",Json::Arr(rolled))])
            }
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
                let lift = |i| { let q = sk.point_xy(i); p.basis.lift(q.0,q.1) };
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
            SolidDef::Loft {face,end,guide} => {
                // the start section, the end section (at the guide's end, where there is one) and the
                // guide; each section's loops in written order, each loop's edges in written order —
                // the order a loft pairs them in
                let mut node = object([("kind","loft".into()),("profile",profile(sk,*face as usize,scale)?),
                    ("guide",super::loft::guide_json(sk,*guide,scale)?)]);
                if let Some(end) = end { node.set("end",profile(sk,*end as usize,scale)?); }
                node
            }
        };
        node.set("id",i.into());
        node.set("name",solid.name.clone().into());
        nodes.push(node);
        seen.insert(i);
    }
    Ok(StaticRecipe {recipe:object([("schema",1.into()),("units","mm".into()),
        ("root",root.into()),("nodes",Json::Arr(nodes))]),sweeps})
}

/// A recipe written about another origin: every point it carries less `by` (millimetres) — a
/// profile's edges' ends, centres and poles, a guide's start and centre, a placement's translation
/// (`t + R by − by`) — and every direction as it was. A point that only places a plane or a line (a
/// profile's origin, a revolution's) is put where its plane or line comes nearest the new origin:
/// moved within it, it says the same, and the frames built about it keep parameters of the solid's
/// own size. A kernel building a solid far from the world's origin builds it about its own instead,
/// where a double still carries the solid's sizes (a part at 1e9 mm is good to 1e-7 mm there).
pub fn shifted(recipe: &Json,by: [f64;3]) -> Json {
    let triple = |j: &Json| -> Option<[f64;3]> { match j { Json::Arr(a) if a.len() == 3 => Some(std::array::from_fn(|k| a[k].as_f64())),_ => None } };
    let point = |j: &Json| match triple(j) { Some(p) => vector(std::array::from_fn(|k| p[k]-by[k])),None => j.clone() };
    let points = |j: &Json,keys: &[&str]| {
        let mut j = j.clone();
        for &k in keys { if let Some(p) = j.get(k).map(&point) { j.set(k,p); } }
        if let Some(Json::Arr(poles)) = j.get("poles") { let poles = Json::Arr(poles.iter().map(&point).collect()); j.set("poles",poles); }
        j
    };
    // the point of the plane through `key` square to `along` (or of the line through it along
    // `along`) nearest the new origin
    let nearest = |j: &mut Json,key: &str,along: &str,plane: bool| {
        let (Some(o),Some(d)) = (j.get(key).and_then(triple),j.get(along).and_then(triple)) else { return };
        let o: [f64;3] = std::array::from_fn(|k| o[k]-by[k]);
        let dd: f64 = d.iter().map(|x| x*x).sum();
        if !(dd > 0.) { return }
        let t = (0..3).map(|k| o[k]*d[k]).sum::<f64>()/dd;
        j.set(key,vector(std::array::from_fn(|k| if plane { t*d[k] } else { o[k]-t*d[k] })));
    };
    let profile = |p: &Json| {
        let mut p = p.clone();
        nearest(&mut p,"origin","normal",true);
        if let Some(Json::Arr(loops)) = p.get("loops") {
            let loops = Json::Arr(loops.iter().map(|l| match l {
                Json::Arr(edges) => Json::Arr(edges.iter().map(|e| points(e,&["start","end","center"])).collect()),
                other => other.clone(),
            }).collect());
            p.set("loops",loops);
        }
        p
    };
    let mut out = recipe.clone();
    if let Some(Json::Arr(nodes)) = recipe.get("nodes") {
        let node = |n: &Json| {
            let mut n = n.clone();
            nearest(&mut n,"origin","axis",false);
            for k in ["profile","end"] { if let Some(p) = n.get(k).filter(|p| matches!(p,Json::Obj(_))).map(&profile) { n.set(k,p); } }
            if let Some(g) = n.get("guide").map(|g| points(g,&["start","center"])) { n.set("guide",g); }
            if let Some(Json::Arr(m)) = n.get("matrix") {
                let m: Vec<f64> = m.iter().map(Json::as_f64).collect();
                if m.len() == 12 {
                    let mut moved = m.clone();
                    for r in 0..3 { moved[4*r+3] = m[4*r+3]+(0..3).map(|c| m[4*r+c]*by[c]).sum::<f64>()-by[r]; }
                    n.set("matrix",Json::Arr(moved.into_iter().map(Json::from).collect()));
                }
            }
            n
        };
        let nodes = nodes.iter().map(|n| {
            let mut n = node(n);
            // a fillet's pieces are nodes of their own, and a rolled loop is picked by a point
            if let Some(Json::Arr(pieces)) = n.get("pieces") {
                let pieces = Json::Arr(pieces.iter().map(&node).collect());
                n.set("pieces",pieces);
            }
            if let Some(Json::Arr(rolled)) = n.get("rolled") {
                let rolled = Json::Arr(rolled.iter().map(|r| points(r,&["point"])).collect());
                n.set("rolled",rolled);
            }
            if let Some(Json::Arr(corners)) = n.get("corners") {
                let corners = Json::Arr(corners.iter().map(|c| points(c,&["vertex","centre"])).collect());
                n.set("corners",corners);
            }
            n
        }).collect();
        out.set("nodes",Json::Arr(nodes));
    }
    out
}

/// How near its curve a fitted stretch must pass (millimetres).
pub const FIT_MM: f64 = 1e-4;

/// The cubic B-spline through a curve's samples (Piegl and Tiller's global interpolation,
/// `curve::interpolating_ctrl_at`) in the curve's own coordinates, over its interval from first
/// end to second, its parameter the fraction of the curve's at every sample (so a loft pairs the
/// fit where it pairs the curve): the samples doubled until every sample halfway between two it
/// passes through is within `tol` of it, measured by projecting that sample onto the fit. A
/// trimmed stretch's first and last poles are its end points themselves, which its neighbours
/// share. The poles, the knots and the error measured; `None` past 4096 samples.
pub fn fit_curve(sk: &Sketch,i: usize,tol: f64) -> Option<(Vec<(f64,f64)>,Vec<f64>,f64)> {
    let (a,b) = sk.curve_domain(i);
    let ends = sk.curves[i].trim.map(|t| (sk.point_xy(t.from as usize),sk.point_xy(t.to as usize)));
    fit_stretch(sk,i,(a,b),ends,tol)
}

/// The curve over `(a, b)` (decreasing where the stretch runs against the curve) as a cubic
/// B-spline within `tol`, its two ends put exactly at `ends` where given.
fn fit_stretch(sk: &Sketch,i: usize,(a,b): (f64,f64),ends: Option<((f64,f64),(f64,f64))>,tol: f64)
    -> Option<(Vec<(f64,f64)>,Vec<f64>,f64)> {
    fit_sampled(&|lo,hi,n| sk.curve_sweep(i,lo,hi,n),(a,b),ends,tol)
}

/// A planar stretch read by `sweep` (`n` chords over `[lo, hi]`, `lo < hi`) over `(a, b)`
/// (decreasing where the stretch runs against the parameter) as a cubic B-spline within `tol`,
/// its ends put exactly at `ends` where given: its poles, knots and the error measured.
pub fn fit_sampled(sweep: &dyn Fn(f64,f64,usize) -> Vec<(f64,f64)>,(a,b): (f64,f64),ends: Option<((f64,f64),(f64,f64))>,tol: f64)
    -> Option<(Vec<(f64,f64)>,Vec<f64>,f64)> {
    use crate::brep::nurbs::BSpline;
    let (lo,hi) = (a.min(b),a.max(b));
    let mut n = 8;
    loop {
        let mut fine = sweep(lo,hi,2*n);
        if a > b { fine.reverse(); }
        let mut through: Vec<(f64,f64)> = fine.iter().step_by(2).copied().collect();
        let last = through.len()-1;
        if let Some(ends) = ends {
            (through[0],through[last]) = ends;
        }
        let fractions = (0..=n).map(|k| k as f64/n as f64).collect();
        let (ctrl,knots,t) = crate::curve::interpolating_ctrl_at(&through,fractions)?;
        let s = BSpline::new(crate::curve::DEGREE,knots.clone(),ctrl.iter().map(|&(x,y)| [x,y,0.]).collect()).ok()?;
        // each halfway sample's distance to the fit, from the fit's own halfway parameter
        let err = (0..n).map(|k| {
            let q = [fine[2*k+1].0,fine[2*k+1].1,0.];
            let mut u = (t[k]+t[k+1])/2.;
            for _ in 0..8 {
                let (x,d,dd) = s.d2(u);
                let e = crate::space::sub(x,q);
                let h = crate::space::dot(d,d)+crate::space::dot(e,dd);
                if !(h > 0.) { break }
                u = (u-crate::space::dot(e,d)/h).clamp(t[k],t[k+1]);
            }
            crate::space::distance(s.point(u),q)
        }).fold(0.,f64::max);
        if err <= tol { return Some((ctrl,knots,err)) }
        n *= 2;
        if n > 2048 { return None }
    }
}

/// A fillet's piece as a recipe node (millimetres): its section's loop — along the first face from
/// the corner, the ball's arc, back along the second — swept along the edge, or turned once about
/// the axis.
fn fillet_piece(p: &super::fillet::Piece,scale: f64) -> Json {
    use super::fillet::Stroke;
    let lift = |q: [f64;2]| p.section.lift(q[0],q[1]).map(|v| v*scale);
    let normal = p.section.normal();
    let edge = |stroke: Stroke,name: &str| match stroke {
        Stroke::Line {from,to} => object([("kind","line".into()),
            ("start",vector(lift(from))),("end",vector(lift(to))),("name",name.into())]),
        // a recipe's arc turns counter-clockwise about its normal; a loop's edges go either way round
        Stroke::Arc {centre,radius,start,sweep,..} => object([("kind","circle".into()),
            ("center",vector(lift(centre))),("normal",vector(normal)),("x_dir",vector(p.section.u)),
            ("radius",(radius*scale).into()),
            ("angles",Json::Arr(vec![start.min(start+sweep).into(),start.max(start+sweep).into()])),
            ("name",name.into())]),
    };
    let edges = p.wedge.strokes().into_iter().zip(super::fillet::EDGE_NAMES).map(|(s,n)| edge(s,n)).collect();
    let profile = object([("loops",Json::Arr(vec![Json::Arr(edges)])),
        ("origin",vector(p.section.o.map(|v| v*scale))),("normal",vector(normal))]);
    match p.carry {
        super::fillet::Carry::Prism {length} => object([("kind","prism".into()),("profile",profile),
            ("from",0.0.into()),("to",(length*scale).into())]),
        super::fillet::Carry::Turn {sweep} => object([("kind","revolve".into()),("profile",profile),
            ("origin",vector(p.section.o.map(|v| v*scale))),("axis",vector(p.section.v)),
            ("angle",sweep.into())]),
    }
}

/// A profile's cubic B-spline edge as a recipe writes it: its knots, its poles lifted into space
/// (millimetres) and the fit it was measured within (mm).
pub(crate) fn bspline_edge(poles: &[(f64,f64)],knots: &[f64],fit_mm: f64,lift: &dyn Fn((f64,f64)) -> [f64;3]) -> Json {
    object([("kind","bspline".into()),("degree",crate::curve::DEGREE.into()),
        ("knots",Json::Arr(knots.iter().map(|&k| k.into()).collect())),
        ("poles",Json::Arr(poles.iter().map(|&c| vector(lift(c))).collect())),
        ("fit",fit_mm.into())])
}

fn profile(sk: &Sketch,index: usize,scale: f64) -> Result<Json,String> {
    let face = &sk.faces[index];
    let p = super::face_poly(sk,index,super::REPORT_UNIT).ok_or("invalid CAD profile")?;
    let lift = |q: (f64,f64)| p.basis.lift(q.0,q.1).map(|v| v*scale);
    let x_dir = p.basis.u;
    // an edge's pieces in a profile: one, or a closed curve's smooth stretches
    let edge = |e: EntRef| -> Result<Vec<Json>,String> {
        Ok(vec![match e.kind {
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
            EntKind::Spline => {
                // a clamped cubic B-spline: its knots and its control points, lifted
                let sp = &sk.splines[e.i()];
                let mut o = object([("kind","bspline".into()),("degree",crate::curve::DEGREE.into()),
                    ("knots",Json::Arr(sp.knots.iter().map(|&k| k.into()).collect())),
                    ("poles",Json::Arr(sp.ctrl.iter().map(|&c| vector(lift(sk.point_xy(c as usize)))).collect()))]);
                // a rational one's weights: the same curve exactly, in either kernel
                if let Some(w) = &sp.weights { o.set("weights",Json::Arr(w.iter().map(|&x| x.into()).collect())); }
                o
            }
            EntKind::Curve => {
                // a stretch of a traced or formula curve: the cubic B-spline through it, fitted
                // until it is within `FIT_MM` of the curve, and what the fit measured
                let fitted = |(poles,knots,err): (Vec<(f64,f64)>,Vec<f64>,f64)| bspline_edge(&poles,&knots,err*scale,&lift);
                let unfitted = || format!("`{}`: a curve in its profile could not be fitted within {FIT_MM} mm",face.name);
                if sk.curves[e.i()].trim.is_none() {
                    // a closed curve standing alone: two halves meeting at two vertices, so no side
                    // of the prism is a closed surface with a seam for a Boolean to arrange
                    if !sk.curve_closed(e.i()) {
                        return Err(format!("`{}`: its curve does not come back to where it started",face.name));
                    }
                    let (a,b) = sk.curve_domain(e.i());
                    let m = 0.5*(a+b);
                    let (start,middle) = (sk.curve_point(e.i(),a),sk.curve_point(e.i(),m));
                    let half = |stretch,ends| fit_stretch(sk,e.i(),stretch,Some(ends),FIT_MM/scale).ok_or_else(unfitted).map(fitted);
                    return Ok(vec![half((a,m),(start,middle))?,half((m,b),(middle,start))?]);
                }
                fitted(fit_curve(sk,e.i(),FIT_MM/scale).ok_or_else(unfitted)?)
            }
            _ => return Err(format!("`{}`: CAD profiles currently require lines, arcs, circles, splines or stretches of curves",face.name)),
        }])
    };
    // each edge with its name, where the document gives it one: the name of the face a sweep makes
    // of it, which a kernel building the solid gives that face (`brep::build`)
    // (a closed curve's stretches each named as the curve is)
    let loops = face.boundaries().map(|(edges,names)|
        edges.iter().enumerate().map(|(k,&e)| edge(e).map(|pieces| pieces.into_iter().map(|mut j| {
            if let Some(n) = names.get(k).filter(|n| !n.is_empty()) { j.set("name",n.clone().into()); }
            j
        }).collect::<Vec<_>>())).collect::<Result<Vec<_>,_>>().map(|l| Json::Arr(l.concat())))
        .collect::<Result<Vec<_>,_>>()?;
    Ok(object([("loops",Json::Arr(loops)),("origin",vector(p.basis.o.map(|v| v*scale))),
        ("normal",vector(p.basis.normal()))]))
}
