//! The traced sheets a construction starts from: the tracer's candidate
//! sheets over exactly the declared interval, no overrun, so the first and
//! last columns are the contact curves at the two end poses.
use super::caps::Cap;
use super::grazing::{GrazingFace,PlaneMotion,grazing_faces,swept_region};
use crate::model::Sketch;
use crate::solid::{SweepContacts,SweepPatch};
use crate::space::{cross,dot,normalised};

/// A grazing face's own plane, and the region it sweeps there.
#[derive(Clone,Debug)]
pub struct Grazing { pub face: GrazingFace, pub patch: SweepPatch, pub rim: Vec<bool> }

/// A seed of the construction: a traced sheet, a cap cut from the tool at an end of the roll, or
/// the region a grazing face sweeps in its own plane.
#[derive(Clone,Debug)]
pub enum Seed { Traced(SweepPatch), Cap(Cap), Grazing(Grazing) }

impl Seed {
    pub fn patch(&self) -> &SweepPatch {
        match self { Seed::Traced(p) => p,Seed::Cap(c) => &c.patch,Seed::Grazing(g) => &g.patch }
    }
    /// A cap's vertices on an edge of the tool, where a vertex's own label cannot say which face
    /// it speaks for and the corner is judged in from instead. A traced sheet's labels decide,
    /// and so do a region's: its rim is the swept boundary there exactly, by construction, with
    /// nothing about it for a corner to be in doubt over.
    pub fn tool_edges(&self) -> Option<&[bool]> {
        match self { Seed::Traced(_) | Seed::Grazing(_) => None,Seed::Cap(c) => Some(&c.tool_edges) }
    }
}

/// Read the sweep's tool and motion, put aside every grazing planar face, and trace the rest.
pub fn seeds(sk: &Sketch,swept: usize,spacing: f64,sagitta: f64,progress: &dyn Fn(&str))
    -> Result<(SweepContacts,Vec<SweepPatch>,Vec<GrazingFace>),String> {
    let mut sweep = SweepContacts::read(sk,swept,1e-10)?;
    let grazing = grazing_faces(&sweep)?;
    sweep.leave_faces(grazing.iter().map(|g| g.face).collect());
    let sheets = sweep.characteristic_sheets(spacing,sagitta,0.,1e-9,None,progress)?;
    Ok((sweep,sheets,grazing))
}

/// The region each grazing face sweeps in its own plane, as a seed: its rim sampled at the
/// tracer's own times, where the sheets that meet it there end.
pub fn grazing_seeds(faces: &[GrazingFace],times: &[f64],roll: [f64;2],sagitta: f64,spacing: f64) -> Result<Vec<Grazing>,String> {
    let [from,to] = roll;
    let mut out = Vec::with_capacity(faces.len());
    for face in faces {
        // the motion's own parameter at each time, measured from the roll's start
        let reach = match face.motion { PlaneMotion::Turn {sweep,..} => sweep,PlaneMotion::Slide {advance} => advance[0].hypot(advance[1]) };
        let poses: Vec<f64> = times.iter().map(|t| reach*(t-from)/(to-from)).collect();
        let region = swept_region(&face.outer,&face.holes,face.motion,sagitta,spacing,&poses)?;
        // wound so that every triangle faces the way the face does
        let flip = normalised(cross(face.u,face.v)).is_some_and(|n| dot(n,face.outward) < 0.);
        let patch = SweepPatch {
            points:region.points.iter().map(|p| face.lift(*p)).collect(),
            normals:vec![face.outward;region.points.len()],
            triangles:region.triangles.iter().map(|t| if flip { [t[0],t[2],t[1]] } else { *t }).collect(),
            column:region.row.clone(),times:vec![from,to],closed:false,provenance:None,
        };
        out.push(Grazing {face:face.clone(),patch,rim:region.rim});
    }
    Ok(out)
}
