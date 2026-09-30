//! The CAD recipe (`solid::cad::recipe`) built by this kernel: the same millimetre-valued DAG the
//! native host is handed, node by node, so the two kernels can be held to each other.
use super::build::{prism,revolve,Profile};
use super::geom::{Rigid,V};
use super::topo::Brep;
use crate::json::Json;
use std::collections::BTreeMap;

fn vec3(j: &Json) -> V { let a = j.arr(); [a[0].as_f64(),a[1].as_f64(),a[2].as_f64()] }
fn field<'a>(j: &'a Json,k: &str) -> Result<&'a Json,String> { j.get(k).ok_or(format!("recipe: no `{k}`")) }

/// A box about every point of the boundary: the corners of its faces' parameter boxes and a
/// grid across each (for a face of revolution, the bulge between its edges), so it may be larger
/// than the solid but is never smaller than its vertices and edges.
pub fn bounds(b: &Brep) -> ([f64;3],[f64;3]) {
    let (mut lo,mut hi) = ([f64::INFINITY;3],[f64::NEG_INFINITY;3]);
    let mut grow = |p: V| for k in 0..3 { lo[k] = lo[k].min(p[k]); hi[k] = hi[k].max(p[k]); };
    for v in &b.vertices { grow(v.p); }
    for f in &b.faces {
        let (mut ulo,mut uhi) = ([f64::INFINITY;2],[f64::NEG_INFINITY;2]);
        for l in &f.loops { for c in l {
            let e = &b.edges[c.edge as usize];
            for j in 0..=16 {
                let t = e.t[0]+(e.t[1]-e.t[0])*j as f64/16.;
                grow(e.point(t,&b.vertices));
                let uv = c.pcurve.at(t,e,&f.surface,&b.vertices);
                for k in 0..2 { ulo[k] = ulo[k].min(uv[k]); uhi[k] = uhi[k].max(uv[k]); }
            }
        } }
        for i in 0..=16 { for j in 0..=16 {
            let uv = [ulo[0]+(uhi[0]-ulo[0])*i as f64/16.,ulo[1]+(uhi[1]-ulo[1])*j as f64/16.];
            grow(f.surface.point(uv));
        } }
    }
    (lo,hi)
}

/// One node of a recipe built from the nodes before it; `Err` names what this kernel does not
/// build yet.
pub fn node(n: &Json,built: &BTreeMap<i64,Brep>) -> Result<Brep,String> {
    let kind = field(n,"kind")?.as_str();
    let source = |k: &str| -> Result<&Brep,String> {
        built.get(&field(n,k)?.as_i64()).ok_or(format!("recipe: `{k}` not built before its user"))
    };
    match kind {
        "prism" => prism(&Profile::from_json(field(n,"profile")?)?,field(n,"from")?.as_f64(),field(n,"to")?.as_f64()),
        "revolve" => revolve(&Profile::from_json(field(n,"profile")?)?,vec3(field(n,"origin")?),
            vec3(field(n,"axis")?),field(n,"angle")?.as_f64()),
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
                let (a,z) = bounds(b);
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
        "body" => Err("Booleans are not built by this kernel yet".into()),
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
