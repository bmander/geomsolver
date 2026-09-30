//! The CAD recipe (`solid::cad::recipe`) built by this kernel: the same millimetre-valued DAG the
//! native host is handed, node by node, so the two kernels can be held to each other.
use super::boolean::{boolean,Op};
use super::build::{loft,prism,revolve,Guide,Profile};
use super::geom::{Rigid,V};
use super::topo::Brep;
use crate::json::Json;
use std::collections::BTreeMap;

fn vec3(j: &Json) -> V { let a = j.arr(); [a[0].as_f64(),a[1].as_f64(),a[2].as_f64()] }
fn field<'a>(j: &'a Json,k: &str) -> Result<&'a Json,String> { j.get(k).ok_or(format!("recipe: no `{k}`")) }

/// One node of a recipe built from the nodes before it; `Err` names what this kernel does not
/// build yet.
pub fn node(n: &Json,built: &BTreeMap<i64,Brep>) -> Result<Brep,String> { node_named(n,built,&BTreeMap::new()) }

/// `node`, naming each operand in what it says.
pub fn node_named(n: &Json,built: &BTreeMap<i64,Brep>,built_names: &BTreeMap<i64,String>) -> Result<Brep,String> {
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
            // a prism spanning its sources' material along the profile's normal, padded as the
            // native host pads it
            let profile = Profile::from_json(field(n,"profile")?)?;
            let (mut lo,mut hi) = ([f64::INFINITY;3],[f64::NEG_INFINITY;3]);
            for s in field(n,"sources")?.arr() {
                let b = built.get(&s.as_i64()).ok_or("recipe: a `through` source not built before it")?;
                let (a,z) = b.bounds();
                for k in 0..3 { lo[k] = lo[k].min(a[k]); hi[k] = hi[k].max(z[k]); }
            }
            let (nrm,o) = (crate::space::scale(profile.normal,1./crate::space::norm(profile.normal)),profile.origin);
            let (mut from,mut to,mut diagonal) = (0.,0.,0.);
            for k in 0..3 {
                let (a,z) = (nrm[k]*(lo[k]-o[k]),nrm[k]*(hi[k]-o[k]));
                from += a.min(z); to += a.max(z);
                diagonal += (hi[k]-lo[k]).powi(2);
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
                    let tol = 1e-9*solid.size().max(operand.size());
                    let name = built_names.get(&x.as_i64()).cloned().unwrap_or_default();
                    solid = boolean(&solid,operand,op,tol).map_err(|e| format!("{key} `{name}`: {e}"))?;
                    solid.check(10.*tol).map_err(|e| format!("{key} `{name}`: {e}"))?;
                }
            }
            Ok(solid)
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
