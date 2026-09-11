//! The static remainder of a body: its facet primitives and Boolean term with
//! every continuous sweep it cuts left out, for a host that arranges meshes.
use super::{document::{frame_origin,resolve_at},mesh_unit,Csg,Term};
use crate::{csg::Piece,mesh,model::Sketch};

/// Primitives are in solid-local coordinates (world minus `origin`), cut at
/// `unit`. A sweep resolves to an empty term in the facet evaluator; those
/// empties are pruned from the body's Booleans, and any other empty operand is
/// an error as it is for a full evaluation.
pub struct StaticSolid {
    pub csg: Csg,
    pub origin: [f64;3],
    pub unit: f64,
    pub sweeps_omitted: usize,
}

impl StaticSolid {
    /// Whether a world point lies inside the static remainder, by the facet
    /// evaluator's own ray classification.
    pub fn contains(&self,p: [f64;3]) -> bool {
        self.csg.inside(std::array::from_fn(|k| p[k]-self.origin[k]))
    }
    /// The evaluated boundary in world coordinates, by the facet CSG kernel.
    /// Slow on finely cut revolutions; a mesh host evaluates the term itself.
    pub fn boundary(&self) -> Result<Vec<Piece>,String> {
        let epsilon = self.csg.epsilon();
        if !epsilon.is_finite() || epsilon <= 0. { return Err("solid scale is not representable".into()); }
        let mut pieces = crate::csg::boundary(&self.csg,epsilon);
        for piece in &mut pieces {
            if !piece.area().is_finite() { return Err("solid boundary exceeds numerical precision".into()); }
            for p in &mut piece.pts { for k in 0..3 { p[k] += self.origin[k]; } }
        }
        Ok(pieces)
    }
}

fn prune(term: Term,omitted: &mut usize) -> Term {
    match term {
        Term::Union(a,b) | Term::Diff(a,b) if matches!(*b,Term::Empty) => { *omitted += 1; prune(*a,omitted) }
        Term::Union(a,b) => Term::Union(Box::new(prune(*a,omitted)),Box::new(prune(*b,omitted))),
        Term::Diff(a,b) => Term::Diff(Box::new(prune(*a,omitted)),Box::new(prune(*b,omitted))),
        Term::Inter(a,b) => Term::Inter(Box::new(prune(*a,omitted)),Box::new(prune(*b,omitted))),
        other => other,
    }
}

/// The static remainder of `root`, cut at the mesh unit of the solid's own
/// extent. `expected_sweeps` is the number of swept cut operands the caller
/// knows about; a mismatch means some other operand failed to build.
pub fn static_solid(sk: &Sketch,root: usize,expected_sweeps: usize) -> Result<StaticSolid,String> {
    static_solid_at_unit(sk,root,mesh_unit(sk,root),expected_sweeps)
}

/// The same at a caller's mesh unit (the world length its facets are cut to
/// within `curve::FLATNESS_PX` of), for a boundary meshed to a stated sagitta.
pub fn static_solid_at_unit(sk: &Sketch,root: usize,unit: f64,expected_sweeps: usize) -> Result<StaticSolid,String> {
    let name = sk.solid_name(root);
    let origin = frame_origin(sk,root,unit);
    let mut csg = resolve_at(sk,root,unit,origin);
    let mut omitted = 0;
    csg.term = prune(std::mem::replace(&mut csg.term,Term::Empty),&mut omitted);
    if omitted != expected_sweeps {
        return Err(format!("`{name}`: {omitted} operands could not be evaluated where {expected_sweeps} sweeps were expected"));
    }
    let mut terms = vec![&csg.term];
    while let Some(term) = terms.pop() {
        match term {
            Term::Empty => return Err(format!("`{name}`: an operand could not be evaluated at this approximation")),
            Term::Prim(_) => {}
            Term::Union(a,b) | Term::Diff(a,b) | Term::Inter(a,b) => { terms.push(a); terms.push(b); }
        }
    }
    if csg.prims.is_empty() || csg.prims.iter().any(|p| p.facets.is_empty()
        || p.facets.iter().any(|f| f.pts.len() < 3 || f.pts.iter().flatten().chain(&f.n).any(|x| !x.is_finite()))) {
        return Err(format!("`{name}`: cannot evaluate finite solid geometry at this approximation"));
    }
    Ok(StaticSolid {csg,origin,unit,sweeps_omitted:omitted})
}

/// An indexed triangle mesh of welded pieces: vertices shared by exact
/// coordinate identity after the weld and T-junction stitch.
pub fn indexed(pieces: &[Piece]) -> (Vec<[f64;3]>,Vec<[u32;3]>) {
    let grouped = mesh::grouped(pieces);
    let mut vertices: Vec<[f64;3]> = Vec::new();
    let mut index: std::collections::HashMap<[u64;3],u32> = std::collections::HashMap::new();
    let mut triangles = Vec::with_capacity(grouped.positions.len()/9);
    for triangle in grouped.positions.chunks_exact(9) {
        let mut ids = [0u32;3];
        for (k,id) in ids.iter_mut().enumerate() {
            let p = [triangle[3*k],triangle[3*k+1],triangle[3*k+2]];
            let key = p.map(|v| if v == 0. { 0f64.to_bits() } else { v.to_bits() });
            *id = *index.entry(key).or_insert_with(|| { vertices.push(p); (vertices.len()-1) as u32 });
        }
        if ids[0] != ids[1] && ids[1] != ids[2] && ids[0] != ids[2] { triangles.push(ids); }
    }
    (vertices,triangles)
}

/// A primitive's facets as an indexed triangle mesh in world coordinates:
/// convex facets fanned, vertices shared by exact coordinate identity.
pub fn primitive_triangles(prim: &super::Prim,origin: [f64;3]) -> (Vec<[f64;3]>,Vec<[u32;3]>) {
    let mut vertices: Vec<[f64;3]> = Vec::new();
    let mut index: std::collections::HashMap<[u64;3],u32> = std::collections::HashMap::new();
    let mut triangles = Vec::new();
    for facet in &prim.facets {
        let ids: Vec<u32> = facet.pts.iter().map(|p| {
            let p: [f64;3] = std::array::from_fn(|k| p[k]+origin[k]);
            let key = p.map(|v| if v == 0. { 0f64.to_bits() } else { v.to_bits() });
            *index.entry(key).or_insert_with(|| { vertices.push(p); (vertices.len()-1) as u32 })
        }).collect();
        for k in 1..ids.len().saturating_sub(1) {
            if ids[0] != ids[k] && ids[k] != ids[k+1] && ids[0] != ids[k+1] { triangles.push([ids[0],ids[k],ids[k+1]]); }
        }
    }
    (vertices,triangles)
}
