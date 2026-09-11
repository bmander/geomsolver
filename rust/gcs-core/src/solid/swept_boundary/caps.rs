//! The caps: the tool's own boundary at the two end poses, the receding side
//! at the start of the roll and the advancing side at its end. The tool is
//! tessellated to the construction's sagitta, each facet posed and kept by
//! the sign of its normal velocity at its centroid, so the cap's rim is
//! ragged to a facet about the contact curve; the field judges the cap's
//! vertices like any sheet's, and the stitch zips the rim to the sheet's end
//! column, which lies exactly on the contact curve.
use crate::model::{Sketch,SolidDef};
use crate::motion::Family;
use crate::solid::{SweepPatch,indexed,static_solid_at_unit};

type V3 = [f64;3];

/// Which end of the roll a cap stands at.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum End { From, To }

/// The two caps of a sweep as seed sheets (one column each, at the end
/// parameter), cut from the tool's welded indexed mesh so that adjacent
/// facets share vertices and no T-junction is left between them.
pub fn caps(sk: &Sketch,swept: usize,sagitta: f64) -> Result<[SweepPatch;2],String> {
    let SolidDef::Swept {source,motion,from,to} = &sk.solids[swept].def else { return Err("not a continuous sweep".into()) };
    let unit = sagitta/crate::curve::FLATNESS_PX;
    let tool = static_solid_at_unit(sk,*source as usize,unit,0)?;
    let (vertices,triangles) = indexed(&tool.boundary()?);
    let family = Family::read(sk,*motion as usize)?;
    let normal = |t: &[u32;3]| -> Option<V3> {
        let [a,b,c] = t.map(|v| vertices[v as usize]);
        let (u,w) = ([b[0]-a[0],b[1]-a[1],b[2]-a[2]],[c[0]-a[0],c[1]-a[1],c[2]-a[2]]);
        let n = [u[1]*w[2]-u[2]*w[1],u[2]*w[0]-u[0]*w[2],u[0]*w[1]-u[1]*w[0]];
        let len = (n[0]*n[0]+n[1]*n[1]+n[2]*n[2]).sqrt();
        (len > 0.).then(|| n.map(|x| x/len))
    };
    let mut out = Vec::with_capacity(2);
    for (end,t) in [(End::From,from.value),(End::To,to.value)] {
        let pose = family.at(t).map_err(|e| format!("{e:?}"))?;
        let mut patch = SweepPatch {points:Vec::new(),normals:Vec::new(),triangles:Vec::new(),column:Vec::new(),times:vec![t],closed:false};
        let mut remap: std::collections::BTreeMap<u32,u32> = Default::default();
        for t in &triangles {
            let Some(n) = normal(t) else { continue };
            let centroid: V3 = std::array::from_fn(|j| t.iter().map(|&v| vertices[v as usize][j]).sum::<f64>()/3.);
            let v = pose.velocity(centroid);
            let n_world = pose.vector(n);
            let along = n_world[0]*v[0]+n_world[1]*v[1]+n_world[2]*v[2];
            // receding at the start, advancing at the end; a facet whose
            // normal velocity vanishes is on the contact curve itself and
            // belongs to the sheet
            let keep = match end { End::From => along < 0.,End::To => along > 0. };
            if !keep { continue; }
            let ids = t.map(|v| *remap.entry(v).or_insert_with(|| {
                patch.points.push(pose.point(vertices[v as usize])); patch.normals.push([0.;3]); patch.column.push(0); (patch.points.len()-1) as u32 }));
            for id in ids { for k in 0..3 { patch.normals[id as usize][k] += n_world[k]; } }
            patch.triangles.push(ids);
        }
        for n in patch.normals.iter_mut() { let len = (n[0]*n[0]+n[1]*n[1]+n[2]*n[2]).sqrt(); if len > 0. { *n = n.map(|x| x/len); } }
        out.push(patch);
    }
    Ok([out.remove(0),out.remove(0)])
}
