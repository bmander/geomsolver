//! Immutable material snapshots of ordinary Solvent solid definitions.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::{SpatialField,MaterialField,SweptField,ExtrudedField,PlanarField,RevolvedField,I,V};
use crate::model::{EntKind,Sketch,SolidDef};
use crate::{motion::Family,plane::Basis,syntax::BodyWord};
use crate::solid::{surface::Edge,RevolvedRegion};
use std::collections::BTreeMap;
use std::f64::consts::TAU;

#[derive(Clone)]
enum Snapshot { Static(SpatialField), Swept(MaterialField) }

impl Snapshot {
    fn material(self) -> MaterialField {
        match self { Self::Static(f) => f.into(),Self::Swept(f) => f }
    }
    fn static_field(self) -> Result<SpatialField,String> {
        match self {
            Self::Static(f) => Ok(f),
            Self::Swept(_) => Err("a continuous sweep cannot be used as a static source; nested continuous sweeps are not yet supported".into()),
        }
    }
    fn transformed(self,under: &Family,at: f64) -> Result<Self,crate::interval::Error> {
        Ok(match self {
            Self::Static(f) => Self::Static(f.transformed(under,at)?),
            Self::Swept(f) => Self::Swept(f.transformed(under,at)?),
        })
    }
    fn support_bounds(&self) -> Result<Option<V>,crate::interval::Error> {
        match self { Self::Static(f) => f.support_bounds(),Self::Swept(f) => f.support_bounds() }
    }
    fn combine(self,other: Self,word: BodyWord) -> Result<Self,crate::interval::Error> {
        Ok(match (self,other) {
            (Self::Static(a),Self::Static(b)) => Self::Static(match word {
                BodyWord::Union => a.union(b)?,
                BodyWord::Cut => a.difference(b)?,
                _ => a.intersection(b)?,
            }),
            (a,b) => {
                let (a,b) = (a.material(),b.material());
                Self::Swept(match word {
                    BodyWord::Union => a.union(b)?,
                    BodyWord::Cut => a.difference(b)?,
                    _ => a.intersection(b)?,
                })
            }
        })
    }
}

/// How near its curve a profile's chord passes, a share of the profile's reach.
pub const CHORD_SLACK: f64 = 1e-7;

/// A curve from `a` to `b` as chords whose middles and quarters lie within `tol` of it (a cubic
/// bends through its middle symmetrically, so the quarters are asked too), from 32 pieces.
fn within(at: &dyn Fn(f64) -> (f64,f64),a: f64,b: f64,tol: f64) -> Vec<(f64,f64)> {
    let off = |p: (f64,f64),q: (f64,f64),m: (f64,f64)| {
        let (dx,dy) = (q.0-p.0,q.1-p.1);
        let l = dx.dhypot(dy);
        if l > 0. { ((m.0-p.0)*dy-(m.1-p.1)*dx).abs()/l } else { (m.0-p.0).dhypot(m.1-p.1) }
    };
    let mut out = vec![at(a)];
    let mut stack: Vec<(f64,f64,u32)> = (0..32).rev().map(|k| (a+(b-a)*k as f64/32.,a+(b-a)*(k+1) as f64/32.,0)).collect();
    while let Some((x,z,depth)) = stack.pop() {
        let (p,q) = (at(x),at(z));
        let strays = [0.25,0.5,0.75].iter().any(|&f| off(p,q,at(x+(z-x)*f)) > tol);
        if depth < 30 && strays { let m = (x+z)/2.; stack.push((m,z,depth+1)); stack.push((x,m,depth+1)); }
        else { out.push(q); }
    }
    out
}

/// A planar face's basis and its loops as exact lines, arcs and circles in the
/// plane's own view coordinates: the same reading the faceted kernel tessellates
/// from, without the tessellation. The outer loop comes first, then each hole.
/// A spline or a stretch of a curve has no closed form here and enters as chords
/// within `CHORD_SLACK` of the loop's size of it, its ends the exact points its
/// neighbours share: the field is exact for those chords (the conversion from the
/// solved curve is not certified, as a solved line's is not).
fn face_loops(sk: &Sketch,face: usize) -> Result<(Basis,Vec<Vec<Edge>>),String> {
    let f = sk.faces.get(face).ok_or("no such face")?;
    let basis = match f.plane()? {
        Some(p) => {
            sk.planes.get(p as usize).ok_or("no such plane")?;
            sk.basis(p as usize)
        }
        None => Basis::page(),
    };
    let at = |q: (f64,f64)| [q.0,q.1];
    let view = |i: u32| at(sk.point_xy(i as usize));
    // how near its curve a chord must pass: a share of the whole face's reach
    let reach = f.boundaries().flat_map(|(edges,_)| edges.iter().flat_map(|&e| sk.children(e)))
        .filter(|c| c.kind == EntKind::Point).map(|c| { let q = view(c.idx); q[0].abs().max(q[1].abs()) })
        .fold(0_f64,f64::max);
    let slack = CHORD_SLACK*(1.+reach);
    let chords = |pts: Vec<(f64,f64)>,ends: [u32;2]| -> Vec<Edge> {
        let mut pts: Vec<[f64;2]> = pts.into_iter().map(at).collect();
        let n = pts.len();
        if n >= 2 { pts[0] = view(ends[0]); pts[n-1] = view(ends[1]); }
        pts.windows(2).map(|w| Edge::Line {a:w[0],b:w[1],axis:false}).collect()
    };
    let loops = f.boundaries().map(|(edges,_)| edges.iter().map(|e| Ok(match e.kind {
        EntKind::Spline => {
            let sp = &sk.splines[e.i()];
            let (a,b) = crate::curve::domain(sk,e.i());
            return Ok(chords(within(&|t| crate::curve::point_at(sk,e.i(),t),a,b,slack),[sp.ctrl[0],*sp.ctrl.last().unwrap()]))
        }
        EntKind::Curve => {
            let pts: Vec<(f64,f64)> = sk.curve_polyline_within(e.i(),slack).into_iter().map(|(_,q)| q).collect();
            let Some(t) = sk.curves[e.i()].trim else {
                // a closed curve is a whole loop: its chords end where they began
                if !sk.curve_closed(e.i()) { return Err("material fields read a curve only where a face trims it, or whole where it closes".into()) }
                let mut pts: Vec<[f64;2]> = pts.into_iter().map(at).collect();
                if let Some(&first) = pts.first() { *pts.last_mut().unwrap() = first; }
                return Ok(pts.windows(2).map(|w| Edge::Line {a:w[0],b:w[1],axis:false}).collect())
            };
            return Ok(chords(pts,[t.from,t.to]))
        }
        EntKind::Line => {
            let l = &sk.lines[e.i()];
            vec![Edge::Line {a:view(l.p1),b:view(l.p2),axis:false}]
        }
        EntKind::Arc => {
            let arc = &sk.arcs[e.i()];
            let center = view(arc.center);
            let ends = [view(arc.start),view(arc.end)];
            let angle = |p: [f64;2]| (p[1]-center[1]).datan2(p[0]-center[0]);
            let start = angle(ends[0]);
            // An arc runs counter-clockwise from its start to its end.
            let mut sweep = angle(ends[1])-start;
            while sweep <= 0. { sweep += TAU; }
            let radius = (ends[0][0]-center[0]).dhypot(ends[0][1]-center[1]);
            vec![Edge::Arc {center,radius,start,sweep,ends}]
        }
        EntKind::Circle => {
            let c = &sk.circles[e.i()];
            let center = view(c.center);
            let radius = sk.params[c.radius as usize].value.abs();
            vec![Edge::Arc {center,radius,start:0.,sweep:TAU,ends:[[center[0]+radius,center[1]];2]}]
        }
        _ => return Err("material fields read planar lines, arcs, circles, splines and stretches of curves \
            as profile edges".to_string()),
    })).collect::<Result<Vec<Vec<Edge>>,String>>().map(|l| l.concat())).collect::<Result<Vec<_>,_>>()?;
    Ok((basis,loops))
}

fn extruded(sk: &Sketch,face: usize,range: [f64;2]) -> Result<ExtrudedField,String> {
    let (basis,loops) = face_loops(sk,face)?;
    ExtrudedField::new(PlanarField::from_loops(&loops,0.)?,basis.o,basis.u,basis.v,range)
        .map_err(|e| format!("material field: {e:?}"))
}

// The ordinates along a face's normal spanned by every material source's support:
// where a through cut has to reach. Padded as the faceted kernel pads its own.
fn through_range(basis: &Basis,supports: &[Option<V>]) -> Result<[f64;2],String> {
    let mut lo = [f64::INFINITY;3]; let mut hi = [f64::NEG_INFINITY;3];
    for support in supports {
        let support = support.ok_or("a through extent needs every material source bounded")?;
        for k in 0..3 {
            lo[k] = lo[k].min(support[k].bounds()[0]); hi[k] = hi[k].max(support[k].bounds()[1]);
        }
    }
    if lo.iter().any(|v| !v.is_finite()) {
        return Err("a through extent needs a material source".into());
    }
    let n = basis.normal();
    let (mut a,mut b) = (0_f64,0_f64);
    for k in 0..3 {
        let (p,q) = (n[k]*(lo[k]-basis.o[k]),n[k]*(hi[k]-basis.o[k]));
        a += p.min(q); b += p.max(q);
    }
    let diagonal = (0..3).map(|k| (hi[k]-lo[k]).dpowi(2)).sum::<f64>().sqrt();
    let pad = diagonal*crate::solid::EPS*4.;
    Ok([a-pad,b+pad])
}

// Preserve static subgraphs as SpatialFields and promote only where a sweep
// appears. Both public readers share dependency ordering, naming and refusals.
fn read(sk: &Sketch,solid: usize,axis_tolerance: f64) -> Result<Snapshot,String> {
    crate::solid::validate(sk,solid)?;
    let mut done: BTreeMap<usize,Snapshot> = BTreeMap::new();
    let mut pending = vec![(solid,false)];
    while let Some((i,ready)) = pending.pop() {
        if done.contains_key(&i) { continue; }
        let s = &sk.solids[i];
        if !ready {
            pending.push((i,true));
            let operands = crate::solid::document::evaluation_operands(sk,i)?;
            pending.extend(operands.into_iter().rev().map(|o| (o as usize,false)));
            continue;
        }
        let field = (|| {
            let get = |i: u32| done[&(i as usize)].clone();
            let error = |e| format!("material field: {e:?}");
            Ok(match &s.def {
                SolidDef::Revolve {..} => Snapshot::Static(RevolvedRegion::read(sk,i,axis_tolerance)?.field()?.into()),
                SolidDef::Prism {face,from,to} =>
                    Snapshot::Static(extruded(sk,*face as usize,[from.value,to.value])?.into()),
                SolidDef::Through {face,..} => {
                    let supports = crate::solid::document::evaluation_operands(sk,i)?.into_iter()
                        .map(|o| get(o).support_bounds()).collect::<Result<Vec<_>,_>>()
                        .map_err(error)?;
                    let (basis,_) = face_loops(sk,*face as usize)?;
                    let range = through_range(&basis,&supports)?;
                    Snapshot::Static(extruded(sk,*face as usize,range)?.into())
                }
                SolidDef::Placed {source,motion,at} => get(*source)
                    .transformed(&Family::read(sk,*motion as usize)?,at.value).map_err(error)?,
                SolidDef::Swept {source,motion,from,to} => Snapshot::Swept(SweptField::new(
                    get(*source).static_field()?,Family::read(sk,*motion as usize)?,
                    I::new(from.value,to.value).map_err(error)?).into()),
                SolidDef::Body {stock,on,through,bound} => {
                    let mut body = get(*stock);
                    for &i in on { body = body.combine(get(i),BodyWord::Union).map_err(error)?; }
                    // Everything that cuts is one union, subtracted once: the same field as the
                    // cuts taken away in turn, max(max(s, -a), -b) being max(s, -min(a, b)), with
                    // the leaves in the same order; but the union keeps an indexed cut's copies
                    // together, where a mesher's query asks only those near it (`Spread`), and
                    // is folded in pairs, so a gear's forty-eight copies stand six unions deep.
                    if let Some(cut) = in_pairs(through.iter().map(|&i| get(i)).collect(),BodyWord::Union)? {
                        body = body.combine(cut,BodyWord::Cut).map_err(error)?;
                    }
                    for &i in bound { body = body.combine(get(i),BodyWord::Bound).map_err(error)?; }
                    body
                }
                // each piece's section exactly, swept or turned as the kernels build it
                SolidDef::Fillet {..} => {
                    let blend = sk.fillet_blend(i)?;
                    // every piece's material, unioned in pairs (a casting's many edges and corners
                    // stand few unions deep)
                    let mut leaves: Vec<Snapshot> = Vec::new();
                    for p in &blend.pieces {
                        use crate::solid::fillet::Stroke;
                        let edges: Vec<Edge> = p.wedge.strokes().into_iter().map(|s| match s {
                            Stroke::Line {from,to} => Edge::Line {a:from,b:to,axis:false},
                            Stroke::Arc {centre,radius,start,sweep,from,to} =>
                                Edge::Arc {center:centre,radius,start,sweep,ends:[from,to]},
                        }).collect();
                        let profile = PlanarField::from_loop(&edges,0.)?;
                        let leaf: SpatialField = match p.carry {
                            crate::solid::fillet::Carry::Prism {length} => ExtrudedField::new(profile,
                                p.section.o,p.section.u,p.section.v,[0.,length]).map_err(error)?.into(),
                            crate::solid::fillet::Carry::Turn {..} =>
                                RevolvedField::new(profile,p.section.o,p.section.v).map_err(error)?.into(),
                        };
                        let mut leaf = Snapshot::Static(leaf);
                        // a partial ring: the whole ring within its sector — bounded by a pie slice
                        // about the axis reaching past the section, carried along the axis past it,
                        // or less the slice of the rest of the turn where that is the convex one
                        if let (crate::solid::fillet::Carry::Turn {sweep},false) = (p.carry,p.whole()) {
                            let reach = edges.iter().map(|e| match *e {
                                Edge::Line {a,b,..} => a[0].hypot(a[1]).max(b[0].hypot(b[1])),
                                Edge::Arc {center,radius,..} => center[0].hypot(center[1])+radius,
                            }).fold(0_f64,f64::max);
                            let (word,from,turn) = if sweep <= std::f64::consts::PI { (BodyWord::Bound,0.,sweep) }
                                else { (BodyWord::Cut,sweep,TAU-sweep) };
                            let radius = 2.*reach;
                            let at = |a: f64| [radius*a.dcos(),radius*a.dsin()];
                            let slice = [Edge::Line {a:[0.,0.],b:at(from),axis:false},
                                Edge::Arc {center:[0.,0.],radius,start:from,sweep:turn,ends:[at(from),at(from+turn)]},
                                Edge::Line {a:at(from+turn),b:[0.,0.],axis:false}];
                            let quarter = crate::space::cross(p.section.v,p.section.u);
                            let sector = ExtrudedField::new(PlanarField::from_loop(&slice,0.)?,p.section.o,p.section.u,quarter,
                                [-radius,radius]).map_err(error)?;
                            leaf = leaf.combine(Snapshot::Static(sector.into()),word).map_err(error)?;
                        }
                        leaves.push(leaf);
                    }
                    // a corner: its cell — on the ball's side of each face, on the vertex's of each
                    // section through the ball's centre, each a half-space (a slab of a large square)
                    // — less the ball (a half disc turned about a line through its centre)
                    for c in &blend.corners {
                        use crate::space::{norm,scale,sub};
                        let reach = 4.*(norm(sub(c.centre,c.vertex))+c.r);
                        let square = PlanarField::from_loop(&[
                            Edge::Line {a:[-reach,-reach],b:[reach,-reach],axis:false},Edge::Line {a:[reach,-reach],b:[reach,reach],axis:false},
                            Edge::Line {a:[reach,reach],b:[-reach,reach],axis:false},Edge::Line {a:[-reach,reach],b:[-reach,-reach],axis:false}],0.)?;
                        // the half-space through `o` behind `n` (where (p − o)·n ≤ 0)
                        let behind = |o: [f64;3],n: [f64;3]| -> Result<Snapshot,String> {
                            let f = crate::brep::geom::Frame::about(o,n);
                            Ok(Snapshot::Static(ExtrudedField::new(square.clone(),o,f.x,f.y,[-reach,0.]).map_err(error)?.into()))
                        };
                        let mut slabs = Vec::new();
                        for e in crate::brep::fillet::corner_sections(c.vertex,c.centre,c.toward) { slabs.push(behind(c.centre,e)?); }
                        for m in c.toward { slabs.push(behind(c.vertex,scale(m,-1.))?); }
                        let cell = in_pairs(slabs,BodyWord::Bound)?.ok_or("a corner with no cell")?;
                        let ball = Snapshot::Static(RevolvedField::new(PlanarField::disk([0.,0.],c.r).map_err(error)?,c.centre,c.toward[0]).map_err(error)?.into());
                        leaves.push(cell.combine(ball,BodyWord::Cut).map_err(error)?);
                    }
                    // a ball rolled along a traced loop: the canal's material less the operands' (at
                    // a concave edge) or within it (convex), read from the spine and contacts
                    if !blend.rolls.is_empty() {
                        let mut union: Option<Snapshot> = None;
                        for o in crate::solid::fillet::operands(sk,i) {
                            let f = get(o);
                            union = Some(match union { None => f,Some(u) => u.combine(f,BodyWord::Union).map_err(error)? });
                        }
                        let union = union.ok_or("a fillet with no operands")?;
                        for roll in &blend.rolls {
                            let model = |c: &crate::brep::nurbs::BSpline| crate::brep::nurbs::BSpline {
                                poles:c.poles.iter().map(|&p| roll.model(p)).collect(),..c.clone() };
                            let rolled = &roll.rolled;
                            let leaf = super::CanalField::new(model(&rolled.spine),[model(&rolled.contacts[0]),model(&rolled.contacts[1])],
                                rolled.r/roll.mm,rolled.reach/roll.mm)?;
                            let leaf = Snapshot::Static(SpatialField::from(leaf));
                            let word = if rolled.concave { BodyWord::Cut } else { BodyWord::Bound };
                            leaves.push(leaf.combine(union.clone(),word).map_err(error)?);
                        }
                    }
                    in_pairs(leaves,BodyWord::Union)?.ok_or("a fillet with no pieces")?
                }
                SolidDef::Loft {..} => return Err("material fields currently require prisms, \
                    full revolutions, Boolean bodies or named motions".into()),
            })
        })().map_err(|e: String| format!("`{}`: {e}",s.name))?;
        done.insert(i,field);
    }
    Ok(done.remove(&solid).unwrap())
}

/// Fields combined by `word`, in pairs and then pairs of those: as many leaves as a long fold, but
/// their depth (which a field bounds) only the logarithm of their count. `None` for none.
fn in_pairs(mut fields: Vec<Snapshot>,word: BodyWord) -> Result<Option<Snapshot>,String> {
    while fields.len() > 1 {
        let mut paired = Vec::with_capacity(fields.len().div_ceil(2));
        let mut it = fields.into_iter();
        while let Some(a) = it.next() {
            paired.push(match it.next() { Some(b) => a.combine(b,word).map_err(|e| format!("material field: {e:?}"))?, None => a });
        }
        fields = paired;
    }
    Ok(fields.pop())
}

impl SpatialField {
    /// Read prisms and full revolutions over analytic profile loops, their Boolean
    /// compositions and fixed motion instances. Unsupported sources fail rather
    /// than substituting a faceted shape. Values use the model's length units.
    pub fn read(sk: &Sketch,solid: usize,axis_tolerance: f64) -> Result<Self,String> {
        read(sk,solid,axis_tolerance)?.static_field()
    }
}

impl MaterialField {
    /// Read ordinary static solids, continuous motion sweeps and their placed or
    /// Boolean combinations. Swept sources must themselves be static. This owns
    /// one solved snapshot; no query rereads source parameters or samples a mesh.
    pub fn read(sk: &Sketch,solid: usize,axis_tolerance: f64) -> Result<Self,String> {
        Ok(read(sk,solid,axis_tolerance)?.material())
    }
}

impl SweptField {
    /// Sweep an ordinary source solid continuously under a named model motion.
    /// Roll bounds are radians. Evaluator budgets remain separate from geometry.
    pub fn read(sk: &Sketch,source: usize,motion: usize,domain: I,axis_tolerance: f64)
        -> Result<Self,String> {
        Ok(Self::new(SpatialField::read(sk,source,axis_tolerance)?,Family::read(sk,motion)?,domain))
    }
}
