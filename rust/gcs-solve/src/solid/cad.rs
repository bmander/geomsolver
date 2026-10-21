//! Solved analytic construction data for a native CAD host. No meshed surfaces,
//! filesystem access or kernel dependency enters the core through this interface.
use crate::{json::{object,Json},model::{EntKind,EntRef,Sense,Sketch,SolidDef}};
use crate::brep::recipe::Writer;
use std::collections::{BTreeMap,BTreeSet};

/// How far from a revolution's axis an endpoint is read as on it, whenever an export reads the
/// solved model into fields, contacts or its admission (the tolerance a revolved snapshot takes).
pub const AXIS_TOLERANCE: f64 = 1e-10;
/// Poses a material evaluator caches per sweep while an export judges its cells and its mesh.
pub const POSE_CACHE: usize = 4096;

/// Model length units to millimetres, which every native export is written in.
pub fn millimetres(sk: &Sketch) -> f64 { sk.units.mm_per() }

pub(super) fn vector(v: [f64;3]) -> Json { Json::Arr(v.into_iter().map(Json::from).collect()) }
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
    Ok(recipe_with(sk,root,false,None)?.recipe)
}

/// `recipe` written about another origin: every point it carries less `by` (millimetres) — a
/// profile's edges' ends, centres and poles, a placement's translation (`t + R by − by`) — and
/// every direction as it was; a point that only places a plane or a line put where its plane or
/// line comes nearest the new origin (`brep::recipe::Writer`). A kernel building a solid far from
/// the world's origin builds it about its own instead.
pub fn recipe_about(sk: &Sketch,root: usize,by: [f64;3]) -> Result<Json,String> {
    Ok(recipe_with(sk,root,false,Some(by))?.recipe)
}

/// One continuous sweep a static recipe left out of a body's cuts: the swept
/// solid and the pose (model units) at which the body subtracts it, composed
/// from the placements between the body and the sweep.
#[derive(Clone,Debug)]
pub struct SweptCut {
    pub swept: usize,
    pub pose: crate::envelope::Motion,
    /// What the faces this cut leaves are named under (`brep::recipe::NAMED`): the cut operand's
    /// own name, the outermost placement's, as `solid::exact` keys a placed copy (#212).
    pub key: String,
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
    recipe_with(sk,root,true,None)
}

/// Whether solid `i` holds a continuous sweep among its operands that only the generating class
/// or the field builds: a ball's sweep built as its tube (`canal`, #238) is static material.
pub(crate) fn contains_sweep(sk: &Sketch,i: usize) -> bool {
    matches!(sk.solids[i].def,SolidDef::Swept {..}) && !super::canal::is_canal(sk,i)
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
        SolidDef::Swept {source,..} if !super::canal::is_canal(sk,i) => {
            if contains_sweep(sk,*source as usize) {
                return Err(format!("`{}`: nested continuous sweeps are not yet supported",solid.name));
            }
            Ok(Some(SweptCut {swept:i,pose:crate::envelope::Motion::identity(),key:solid.name.clone()}))
        }
        SolidDef::Placed {source,motion,at} => match swept_cut(sk,*source as usize)? {
            Some(inner) => {
                let outer = crate::motion::Family::read(sk,*motion as usize)?.at(at.value)?;
                Ok(Some(SweptCut {swept:inner.swept,pose:inner.pose.then(outer),key:solid.name.clone()}))
            }
            None => Ok(None),
        },
        _ if contains_sweep(sk,i) => Err(format!(
            "`{}`: a continuous sweep may only be cut from a body directly or through placements",solid.name)),
        _ => Ok(None),
    }
}

fn recipe_with(sk: &Sketch,root: usize,static_only: bool,about: Option<[f64;3]>) -> Result<StaticRecipe,String> {
    super::validate(sk,root)?;
    let scale = millimetres(sk);
    let w = Writer {scale,about};
    let mut walk = Walk::new(sk,static_only);
    let mut pending = vec![(root,false)];
    let mut seen = BTreeSet::new();
    let mut nodes = Vec::new();
    while let Some((i,ready)) = pending.pop() {
        if seen.contains(&i) { continue; }
        let dependencies = match &sk.solids[i].def {
            SolidDef::Body {..} if !ready => walk.sets(i)?,
            SolidDef::Body {..} => Vec::new(),
            // a tool through a body reads its material, and a region lowered to nothing is none of
            // it: a convex edge's fill beside a flat cut `through:` the body it rounds
            _ => super::evaluation_operands(sk,i)?.into_iter().filter(|o| !walk.nowhere(o)).collect(),
        };
        if !ready {
            pending.push((i,true));
            pending.extend(dependencies.iter().rev().map(|&i| (i as usize,false)));
            continue;
        }
        let mut node = walk.node(i,&dependencies,&w)?;
        node.set("id",i.into());
        node.set("name",sk.solids[i].name.clone().into());
        if sk.solids[i].copy.is_some() { node.set("step",sk.solids[i].step().into()); }
        nodes.push(node);
        seen.insert(i);
    }
    Ok(StaticRecipe {recipe:object([("schema",1.into()),("units","mm".into()),
        ("root",root.into()),("nodes",Json::Arr(nodes))]),sweeps:walk.sweeps})
}

/// A recipe's walk over the solids (`recipe_with`): each body's sets gathered on its first visit
/// (`sets`), and each solid written as its node on its second (`node`).
struct Walk<'a> {
    sk: &'a Sketch,
    /// Whether the body's swept cuts are listed (`sweeps`) and left out of the recipe.
    static_only: bool,
    /// How many solids take each as an operand.
    users: BTreeMap<u32,usize>,
    /// The lowered regions the first of each group stands for.
    groups: BTreeMap<usize,Vec<u32>>,
    /// Each region's route (`region::route`, #194), asked once a recipe; none for any other solid.
    routes: std::cell::RefCell<BTreeMap<usize,Option<super::region::Route>>>,
    /// Each body's sets, read on its first visit and written on its second.
    bodies: BTreeMap<usize,([Vec<u32>;4],Vec<SweptCut>)>,
    /// The swept cuts left out, each listed once, on the emitting visit.
    sweeps: Vec<SweptCut>,
}

impl<'a> Walk<'a> {
    fn new(sk: &'a Sketch,static_only: bool) -> Walk<'a> {
        let mut users: BTreeMap<u32,usize> = Default::default();
        for s in &sk.solids { for o in s.operands() { *users.entry(o).or_default() += 1; } }
        Walk {sk,static_only,users,groups:Default::default(),routes:Default::default(),bodies:Default::default(),sweeps:Vec::new()}
    }

    /// Solid `o`'s route, where it is a region.
    fn route(&self,o: usize) -> Option<super::region::Route> {
        self.routes.borrow_mut().entry(o).or_insert_with(|| super::region::route(self.sk,o).ok()).clone()
    }
    /// The lowering table's pieces of region `o`; none for any other solid.
    fn pieces_of(&self,o: u32) -> std::rc::Rc<Vec<super::lowering::Piece>> {
        self.route(o as usize).as_ref().and_then(|r| r.pieces().cloned()).unwrap_or_default()
    }
    /// Whether `o` is a region lowered to nothing, a term of no body: a round over faces that never
    /// meet.
    fn nowhere(&self,o: &u32) -> bool {
        self.route(*o as usize).as_ref().and_then(|r| r.pieces().map(|p| p.is_empty())).unwrap_or(false)
    }

    /// Body `i`'s sets, read on its first visit: its swept cuts listed apart where only static
    /// material is asked for, what reads its faces split off to act last (the stratum), its lowered
    /// regions in each late set lumped as one; and the solids it depends on, in order.
    fn sets(&mut self,i: usize) -> Result<Vec<u32>,String> {
        let sk = self.sk;
        let SolidDef::Body {stock,on,through,bound} = &sk.solids[i].def else { unreachable!("a body's sets") };
        if let Some(o) = std::iter::once(stock).chain(bound).find(|&o| self.nowhere(o)) {
            return Err(format!("`{}`: `{}` holds nowhere, so the body has no material",sk.solids[i].name,
                sk.solid_name(*o as usize)));
        }
        let mut cut: Vec<u32> = through.iter().copied().filter(|o| !self.nowhere(o)).collect();
        // The static remainder of a body: its swept cuts are listed, not built.
        let mut swept = Vec::new();
        if self.static_only {
            if contains_sweep(sk,*stock as usize) {
                return Err(format!("`{}`: a body's stock may not contain a continuous sweep",sk.solids[i].name));
            }
            if let Some(&o) = on.iter().chain(bound).find(|&&o| contains_sweep(sk,o as usize)) {
                return Err(format!("`{}`: only a `cut` may hold swept material for now (`{}`)",
                    sk.solids[i].name,sk.solids[o as usize].name));
            }
            let mut kept = Vec::new();
            for c in cut {
                match swept_cut(sk,c as usize)? { Some(sweep) => swept.push(sweep),None => kept.push(c) }
            }
            cut = kept;
        }
        // **the stratum** (#183 F6): what reads the body's faces acts on the body its other
        // terms make, after them, fills then cuts, as the field applies them
        let [on,cut,on_late,cut_late] = super::faces::stratum(sk,i,on.iter().copied().filter(|o| !self.nowhere(o)),cut);
        let (on_late,cut_late) = (self.lump(on_late),self.lump(cut_late));
        let dependencies = std::iter::once(*stock).chain(on.iter().copied()).chain(cut.iter().copied())
            .chain(bound.iter().copied()).chain(on_late.iter().copied()).chain(cut_late.iter().copied()).collect();
        self.bodies.insert(i,([on,cut,on_late,cut_late],swept));
        Ok(dependencies)
    }

    /// A body's late set with its lowered regions built as one (`lowering::assemble`): the first
    /// stands for the rest, which are no operands of their own.  Lumped by the labels their faces
    /// are named under (`route_step`, [0.149]): a named round's pieces stand apart from another's,
    /// so each keeps its faces' names, and those of the rounds no name was given stand as one.
    fn lump(&mut self,v: Vec<u32>) -> Vec<u32> {
        let mut by: BTreeMap<String,BTreeSet<u32>> = BTreeMap::new();
        for &o in &v {
            if self.users.get(&o) == Some(&1) && !self.pieces_of(o).is_empty() {
                let step = super::route_step(&self.sk.solids[o as usize].name);
                let labels = step.rsplit_once('.').map_or("",|(l,_)| l).to_string();
                by.entry(labels).or_default().insert(o);
            }
        }
        let mut stood_for = BTreeSet::new();
        for low in by.into_values() {
            let Some(&first) = low.first().filter(|_| low.len() > 1) else { continue };
            self.groups.insert(first as usize,low.iter().copied().collect());
            stood_for.extend(low.into_iter().filter(|&o| o != first));
        }
        v.into_iter().filter(|o| !stood_for.contains(o)).collect()
    }

    /// Solid `i` written as its recipe node by `w`, every solid it depends on (`dependencies`)
    /// written before it.
    fn node(&mut self,i: usize,dependencies: &[u32],w: &Writer) -> Result<Json,String> {
        let (sk,scale) = (self.sk,w.scale);
        let solid = &sk.solids[i];
        Ok(match &solid.def {
            // a ball under one turn: the tube about its centre's path (#238)
            SolidDef::Swept {..} if let Some(c) = super::canal::built(sk,i) => c?.node(w),
            SolidDef::Swept {..} => return Err(format!(
                "`{}`: native CAD boundary construction for continuous motion sweeps is not yet supported",solid.name)),
            SolidDef::Placed { source, motion, at } => {
                let pose = crate::motion::Family::read(sk,*motion as usize)?.at(at.value)?;
                object([("kind","placed".into()),("source",(*source).into()),
                    ("matrix",w.matrix(placement_matrix(pose,scale)))])
            }
            SolidDef::Body {stock,bound,..} => {
                let ([on,cut,on_late,cut_late],swept) = self.bodies.remove(&i).expect("a body's sets, read on its first visit");
                // (each sweep listed once, on the emitting visit)
                self.sweeps.extend(swept);
                let mut node = body_terms(*stock,&on,&cut,bound);
                node.set("kind","body".into());
                if !on_late.is_empty() || !cut_late.is_empty() {
                    node.set("on_late",ids(&on_late));
                    node.set("cut_late",ids(&cut_late));
                }
                node
            }
            // a region: built as its route says (`region::route`)
            SolidDef::Region {..} => match self.route(i).map_or_else(|| super::region::route(sk,i),Ok)? {
                // over a body's faces, an entry of the lowering table (#194): its pieces, with those
                // of the regions it stands for in its body's set
                super::region::Route::Lowered(l) => {
                    if l.map_err(|e| format!("`{}`: {e}",sk.solid_name(i)))?.is_empty() {
                        return Err(format!("`{}` holds nowhere",sk.solid_name(i)))
                    }
                    let members = self.groups.get(&i).cloned().unwrap_or_else(|| vec![i as u32]);
                    let pieces: Vec<_> = members.into_iter().map(|o| self.pieces_of(o)).collect();
                    let assembly = super::lowering::assemble(pieces.iter().flat_map(|p| p.iter()));
                    let mut node = assembly.node(w)?;
                    // a ball rolled along a traced run is rolled against the body as its terms but the
                    // late ones make it, all built before this (they come first in its dependencies)
                    if !assembly.rolled.is_empty() {
                        let late = |sets: &[Vec<u32>;4]| sets[2..].iter().any(|s| s.contains(&(i as u32)));
                        let Some((&b,(sets,_))) = self.bodies.iter().find(|(_,(sets,_))| late(sets)) else {
                            return Err(format!("`{}`: a rolled round is built against the body it rounds",sk.solid_name(i)));
                        };
                        let SolidDef::Body {stock,bound,..} = &sk.solids[b].def else { unreachable!("a body's sets") };
                        node.set("stratum",body_terms(*stock,&sets[0],&sets[1],bound));
                    }
                    node
                }
                // in general position: the common of its terms' primitives, where they are ones that
                // construction makes (#238)
                super::region::Route::Bounded(b) => b.map_err(|e| format!("`{}`: {e}",sk.solid_name(i)))?.node(w)?,
                super::region::Route::Field(why) => return Err(format!("`{}`: {why}",sk.solid_name(i))),
                // on one axis: its meridian turned once about it (§6.21)
                super::region::Route::Meridian(m) => {
                    let name = |t: u32| m.name(t);
                    let p = crate::brep::recipe::region_profile(&m.region,m.origin,m.seam,m.axis,&name);
                    object([
                        ("kind","revolve".into()),("profile",p.to_json(w)?),
                        ("origin",w.line_origin(m.origin.map(|v| v*scale),m.axis)),("axis",vector(m.axis)),
                        ("angle",std::f64::consts::TAU.into()),
                    ])
                }
            },
            SolidDef::Prism {face,from,to,draft} => {
                let mut node = object([
                    ("kind","prism".into()),("profile",profile(sk,*face as usize,w)?),
                    ("from",(from.value*scale).into()),("to",(to.value*scale).into()),
                ]);
                // its sides tapered from the profile's plane (#245), an angle in radians
                if let Some(d) = draft { node.set("draft",d.value.into()); }
                node
            }
            SolidDef::Through {face,..} => object([
                ("kind","through".into()),("profile",profile(sk,*face as usize,w)?),
                ("sources",ids(dependencies)),
            ]),
            SolidDef::Revolve {face,axis,sweep,sense} => {
                // Use the profile's frame for both axis endpoints, as the solid
                // evaluator does; a line need not have its own plane membership.
                let p = super::face_poly(sk,*face as usize,super::REPORT_UNIT)
                    .ok_or("invalid revolution profile")?;
                let (a2,b2) = sk.revolve_axis(*face,*axis)
                    .ok_or("the revolution axis does not lie in the profile's plane")?;
                let a = p.basis.lift(a2.0,a2.1);
                let b = p.basis.lift(b2.0,b2.1);
                let along = std::array::from_fn(|k| b[k]-a[k]);
                object([
                    ("kind","revolve".into()),("profile",profile(sk,*face as usize,w)?),
                    ("origin",w.line_origin(a.map(|v| v*scale),along)),
                    ("axis",vector(along)),
                    ("angle",(sweep.value.min(std::f64::consts::TAU)
                        * if *sense == Sense::Cw { -1. } else { 1. }).into()),
                ])
            }
            // its walls exact (#243 L2): the interval's pieces, the stations each is solved at, and
            // each wall's edge's numbers at every one of them
            SolidDef::Family {..} => super::family::json(&super::family::walls(sk,i)
                .map_err(|e| format!("`{}`: {e}",solid.name))?,w),
        })
    }
}

/// A body's terms but the late ones — its stock, what is on it, what cuts it, what bounds it — as
/// the recipe's `body` reads them: the body node's, and the stratum a rolled round is rolled against.
fn body_terms(stock: u32,on: &[u32],cut: &[u32],bound: &[u32]) -> Json {
    object([("stock",stock.into()),("on",ids(on)),("cut",ids(cut)),("bound",ids(bound))])
}

/// How near its curve a fitted stretch must pass (millimetres).
pub const FIT_MM: f64 = 1e-4;

/// The cubic B-spline through a curve's samples (Piegl and Tiller's global interpolation,
/// `curve::interpolating_ctrl_at`) in the curve's own coordinates, over its interval from first end
/// to second, its parameter the fraction of the curve's at every sample: the samples doubled until
/// every sample halfway between two it passes through is within `tol` of it, measured by projecting
/// that sample onto the fit. A trimmed stretch's or a free curve's first and last poles are its end
/// points themselves, which its neighbours share. The poles, the knots and the error measured;
/// `None` past 4096 samples.
pub fn fit_curve(sk: &Sketch,i: usize,tol: f64) -> Option<(Vec<(f64,f64)>,Vec<f64>,f64)> {
    let (a,b) = sk.curve_domain(i);
    let ends = sk.curve_ends(i).map(|(p,q)| (sk.point_xy(p as usize),sk.point_xy(q as usize)));
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

/// A profile's cubic B-spline edge as a recipe writes it: its knots, its poles lifted into space
/// (millimetres) and the fit it was measured within (mm).
pub(crate) fn bspline_edge(poles: &[(f64,f64)],knots: &[f64],fit_mm: f64,lift: &dyn Fn((f64,f64)) -> [f64;3]) -> Json {
    object([("kind","bspline".into()),("degree",crate::curve::DEGREE.into()),
        ("knots",Json::Arr(knots.iter().map(|&k| k.into()).collect())),
        ("poles",Json::Arr(poles.iter().map(|&c| vector(lift(c))).collect())),
        ("fit",fit_mm.into())])
}

fn profile(sk: &Sketch,index: usize,w: &Writer) -> Result<Json,String> {
    let face = &sk.faces[index];
    let p = super::face_poly(sk,index,super::REPORT_UNIT).ok_or("invalid CAD profile")?;
    let scale = w.scale;
    // a point of the profile, in millimetres as written
    let lift = |q: (f64,f64)| w.moved(p.basis.lift(q.0,q.1).map(|v| v*scale));
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
                if sk.curve_ends(e.i()).is_none() {
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
    Ok(object([("loops",Json::Arr(loops)),("origin",w.plane_origin(p.basis.o.map(|v| v*scale),p.basis.normal())),
        ("normal",vector(p.basis.normal()))]))
}
