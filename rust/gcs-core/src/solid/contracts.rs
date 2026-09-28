//! The contracts a constructed body's pieces answer to before its field agreement is asked
//! (`agreement.rs`): judgements about geometry a kernel produced, made here so every host
//! judges alike. A host measures (triangles, cell volumes) and the core decides.
use crate::space::{sub,cross,norm};

type V = [f64;3];

/// A triangle under this many square millimetres is microscopic.
pub const TINY_AREA: f64 = 1e-6;
/// More microscopic triangles than this is a crumpled or folded patch, not a tessellator
/// meeting a short edge.
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
    /// The mesh contract: no cluster of microscopic triangles. A few may come of a tessellator
    /// meeting a short edge; a hundred under a square micrometre is a crumpled or folded patch of
    /// surface, whatever a probe sampled by area happens to find there.
    pub fn verdict(&self) -> Result<(),String> {
        if self.count > MOST_TINY {
            return Err(format!("the mesh has {} triangles under {TINY_AREA} mm² ({:.2e} mm² in all) between {:?} and {:?}: \
                a crumpled or folded patch of surface",self.count,self.area,self.low.map(|x| (x*1e3).round()/1e3),
                self.high.map(|x| (x*1e3).round()/1e3)));
        }
        Ok(())
    }

    /// The mesh contract held to a tolerance, whose finer mesh has more short edges for a
    /// tessellator to meet (along every trimmed edge of every tooth): a triangle is microscopic
    /// under a square a tenth of the mesher's deflection on a side, and a crumpled or folded patch
    /// is more than `MOST_TINY` of them in one millimetre cube, where the fold phase 2 found put
    /// eighteen thousand.
    pub fn clustered(&self) -> Result<(),String> {
        if self.densest > MOST_TINY {
            return Err(format!("the mesh has {} triangles under {:.1e} mm² within the millimetre cube at {:?} ({} in all, {:.2e} mm²): \
                a crumpled or folded patch of surface",self.densest,self.under,self.at,self.count,self.area));
        }
        Ok(())
    }
}

/// A cell of a blank's partition as the cell contract reads it: its volume (mm³) and a point
/// inside it (mm).
#[derive(Clone,Copy,Debug)]
pub struct CellVolume { pub volume: f64,pub point: V }

/// No cell of a partition may be smaller than this (mm³): a sliver of near-tangent surfaces.
pub const LEAST_CELL: f64 = 1e-3;

/// The cell contract after classification: one removed cell per placement of each sweep, the
/// cells of one sweep congruent (its placements are rigid copies, so a cell that differs was
/// cut by something else: a neighbour, a tangent face), and no cell below a size floor (a
/// sliver the kernel made of near-tangent surfaces). The removed volumes are compared as a
/// set, since a cell does not say which placement cut it; a body whose placements cut unequal
/// amounts from an asymmetric blank would need them told apart first.
pub fn cells(kept: &[CellVolume],removed: &[CellVolume],placements: usize) -> Result<(),String> {
    if let Some(cell) = kept.iter().chain(removed).find(|c| c.volume < LEAST_CELL) {
        return Err(format!("the split left a cell of {:.3e} mm³, under the {LEAST_CELL} mm³ floor, near {:?}",
            cell.volume,cell.point.map(|x| (x*1e3).round()/1e3)));
    }
    if removed.len() != placements {
        return Err(format!("the split removed {} cells for {} placements",removed.len(),placements));
    }
    let (least,most) = removed.iter().fold((f64::INFINITY,0_f64),|(l,m),c| (l.min(c.volume),m.max(c.volume)));
    if most > least*(1.+1e-4) {
        return Err(format!("the removed cells are not congruent: {least:.6} to {most:.6} mm³"));
    }
    Ok(())
}
