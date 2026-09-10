//! Immutable material snapshots of ordinary Solvent solid definitions.
use super::{SpatialField,MaterialField,SweptField,ExtrudedField,PlanarField,I,V};
use crate::model::{EntKind,Sketch,SolidDef};
use crate::{motion::Family,plane::{self,Basis},syntax::BodyWord};
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
                BodyWord::On => a.union(b)?,
                BodyWord::Cut => a.difference(b)?,
                _ => a.intersection(b)?,
            }),
            (a,b) => {
                let (a,b) = (a.material(),b.material());
                Self::Swept(match word {
                    BodyWord::On => a.union(b)?,
                    BodyWord::Cut => a.difference(b)?,
                    _ => a.intersection(b)?,
                })
            }
        })
    }
}

/// A planar face's basis and its loops as exact lines, arcs and circles in the
/// plane's own view coordinates: the same reading the faceted kernel tessellates
/// from, without the tessellation. The outer loop comes first, then each hole.
fn face_loops(sk: &Sketch,face: usize) -> Result<(Basis,Vec<Vec<Edge>>),String> {
    let f = sk.faces.get(face).ok_or("no such face")?;
    let (basis,pose) = match f.plane()? {
        Some(p) => {
            let pl = sk.planes.get(p as usize).ok_or("no such plane")?;
            (pl.basis,(sk.params[pl.frame.c as usize].value,sk.params[pl.frame.s as usize].value,
                sk.point_xy(pl.frame.origin as usize)))
        }
        None => (Basis::page(),(1.,0.,(0.,0.))),
    };
    let view = |i: u32| {
        let (a,b) = plane::in_view(pose.0,pose.1,pose.2,sk.point_xy(i as usize));
        [a,b]
    };
    let loops = f.boundaries().map(|(edges,_)| edges.iter().map(|e| Ok(match e.kind {
        EntKind::Line => {
            let l = &sk.lines[e.i()];
            Edge::Line {a:view(l.p1),b:view(l.p2),axis:false}
        }
        EntKind::Arc => {
            let arc = &sk.arcs[e.i()];
            let center = view(arc.center);
            let ends = [view(arc.start),view(arc.end)];
            let angle = |p: [f64;2]| (p[1]-center[1]).atan2(p[0]-center[0]);
            let start = angle(ends[0]);
            // An arc runs counter-clockwise from its start to its end.
            let mut sweep = angle(ends[1])-start;
            while sweep <= 0. { sweep += TAU; }
            let radius = (ends[0][0]-center[0]).hypot(ends[0][1]-center[1]);
            Edge::Arc {center,radius,start,sweep,ends}
        }
        EntKind::Circle => {
            let c = &sk.circles[e.i()];
            let center = view(c.center);
            let radius = sk.params[c.radius as usize].value.abs();
            Edge::Arc {center,radius,start:0.,sweep:TAU,ends:[[center[0]+radius,center[1]];2]}
        }
        _ => return Err("material fields read planar lines, arcs and circles as profile edges"
            .to_string()),
    })).collect::<Result<Vec<_>,String>>()).collect::<Result<Vec<_>,_>>()?;
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
    let diagonal = (0..3).map(|k| (hi[k]-lo[k]).powi(2)).sum::<f64>().sqrt();
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
                    for &i in on { body = body.combine(get(i),BodyWord::On).map_err(error)?; }
                    for &i in through { body = body.combine(get(i),BodyWord::Cut).map_err(error)?; }
                    for &i in bound { body = body.combine(get(i),BodyWord::Bound).map_err(error)?; }
                    body
                }
                SolidDef::Loft {..} => return Err("material fields currently require prisms, \
                    full revolutions, Boolean bodies or named motions".into()),
            })
        })().map_err(|e: String| format!("`{}`: {e}",s.name))?;
        done.insert(i,field);
    }
    Ok(done.remove(&solid).unwrap())
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
