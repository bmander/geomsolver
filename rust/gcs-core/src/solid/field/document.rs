//! Immutable material snapshots of ordinary Solvent solid definitions.
use super::{SpatialField,SweptField,I};
use crate::{model::{Sketch,SolidDef},motion::Family,solid::RevolvedRegion};
use std::collections::BTreeMap;

impl SpatialField {
    /// Read full revolutions with convex analytic profile loops, their Boolean
    /// compositions and fixed motion instances. Unsupported sources fail rather
    /// than substituting a faceted shape. Values use the model's length units.
    pub fn read(sk: &Sketch,solid: usize,axis_tolerance: f64) -> Result<Self,String> {
        crate::solid::validate(sk,solid)?;
        let mut done: BTreeMap<usize,Self> = BTreeMap::new();
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
                    SolidDef::Revolve {..} => RevolvedRegion::read(sk,i,axis_tolerance)?.field()?.into(),
                    SolidDef::Placed {source,motion,at} => get(*source)
                        .transformed(&Family::read(sk,*motion as usize)?,at.value).map_err(error)?,
                    SolidDef::Body {stock,on,through} => {
                        let mut body = get(*stock);
                        for &i in on { body = body.union(get(i)).map_err(error)?; }
                        for &i in through { body = body.difference(get(i)).map_err(error)?; }
                        body
                    }
                    _ => return Err("material fields currently require full revolutions, Boolean bodies or motion placements".into()),
                })
            })().map_err(|e: String| format!("`{}`: {e}",s.name))?;
            done.insert(i,field);
        }
        Ok(done.remove(&solid).unwrap())
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
