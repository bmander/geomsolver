//! Immutable material snapshots of ordinary Solvent solid definitions.
use super::{SpatialField,MaterialField,SweptField,I};
use crate::{model::{Sketch,SolidDef},motion::Family,solid::RevolvedRegion,syntax::BodyWord};
use std::collections::BTreeMap;

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
            pending.extend(s.operands().into_iter().rev().map(|o| (o as usize,false)));
            continue;
        }
        let field = (|| {
            let get = |i: u32| done[&(i as usize)].clone();
            let error = |e| format!("material field: {e:?}");
            Ok(match &s.def {
                SolidDef::Revolve {..} => Snapshot::Static(RevolvedRegion::read(sk,i,axis_tolerance)?.field()?.into()),
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
                _ => return Err("material fields currently require full revolutions, Boolean bodies or named motions".into()),
            })
        })().map_err(|e: String| format!("`{}`: {e}",s.name))?;
        done.insert(i,field);
    }
    Ok(done.remove(&solid).unwrap())
}

impl SpatialField {
    /// Read full revolutions with convex analytic profile loops, their Boolean
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
