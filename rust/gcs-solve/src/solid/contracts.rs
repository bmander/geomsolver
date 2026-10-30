//! The contracts a constructed body's pieces answer to before its field agreement is asked
//! (`agreement.rs`): judgements about geometry a kernel produced, made here so every host
//! judges alike. A host measures (triangles, cell volumes) and the core decides.
use crate::space::{sub,cross,norm};

type V = [f64;3];

/// A triangle under this many square millimetres is microscopic.
pub const TINY_AREA: f64 = 1e-6;
/// More microscopic triangles than this in one millimetre cube is a crumpled or folded patch,
/// not a tessellator meeting a short edge.
pub const MOST_TINY: usize = 100;

/// The microscopic triangles of a mesh (millimetres): how many, their area, and the box of
/// their first corners.
#[derive(Clone,Copy,Debug)]
pub struct TinyTriangles {
    pub count: usize,pub area: f64,pub low: V,pub high: V,pub total: usize,
    /// The area under which a triangle counted (mm²), and the most of them in any one
    /// millimetre cube, with that cube's least corner.
    pub under: f64,pub densest: usize,pub at: V,
}

/// The mesh contract's measure over a mesh in model units at `scale` millimetres a unit.
pub fn tiny_triangles(vertices: &[V],triangles: &[[u32;3]],scale: f64) -> TinyTriangles {
    tiny_triangles_under(vertices,triangles,scale,TINY_AREA)
}

/// The same, a triangle counting as microscopic under `under` square millimetres.
pub fn tiny_triangles_under(vertices: &[V],triangles: &[[u32;3]],scale: f64,under: f64) -> TinyTriangles {
    let (mut count,mut area,mut low,mut high) = (0,0.,[f64::INFINITY;3],[f64::NEG_INFINITY;3]);
    let mut cubes: std::collections::BTreeMap<[i64;3],usize> = Default::default();
    for t in triangles {
        let [a,b,c] = t.map(|i| vertices[i as usize].map(|x| x*scale));
        let size = 0.5*norm(cross(sub(b,a),sub(c,a)));
        if size >= under { continue; }
        count += 1; area += size;
        for k in 0..3 { low[k] = low[k].min(a[k]); high[k] = high[k].max(a[k]); }
        *cubes.entry(a.map(|x| x.floor() as i64)).or_default() += 1;
    }
    let (at,densest) = cubes.into_iter().max_by_key(|(_,n)| *n).map_or(([0.;3],0),|(k,n)| (k.map(|x| x as f64),n));
    TinyTriangles {count,area,low,high,total:triangles.len(),under,densest,at}
}

impl TinyTriangles {
    /// The mesh contract: no cluster of microscopic triangles (under a square micrometre, or held
    /// to a tolerance a square a tenth of the mesher's deflection on a side). A few come of a
    /// tessellator meeting a short edge or a gently curved one (an ear of three boundary points),
    /// along every trimmed edge of every tooth, so a gear's count grows with its teeth; more than
    /// `MOST_TINY` in one millimetre cube is a crumpled or folded patch of surface, where the
    /// gear's first folded sheet put eighteen thousand, whatever a probe sampled by area happens
    /// to find there.
    pub fn clustered(&self) -> Result<(),String> {
        if self.densest > MOST_TINY {
            return Err(format!("the mesh has {} triangles under {:.1e} mm² within the millimetre cube at {:?} ({} in all, {:.2e} mm²): \
                a crumpled or folded patch of surface",self.densest,self.under,self.at,self.count,self.area));
        }
        Ok(())
    }
}

/// The cell contract after classification, and what it reads of a cell: the kernel's
/// (`cad_kernel::sweep::cells`), where the split it judges is made.
pub use cad_kernel::sweep::cells::{CellVolume,LEAST_CELL,cells};
