//! The contacts of a continuous sweep's tool: the tool read from the B-rep the kernel builds of it
//! (#721 F1b), its motion and roll from the swept-solid definition. The drawing's own reading of the
//! tool, the oracle the B-rep reading was held to while the classes switched, is retired (#721 F1c).
use super::SpatialField;
use crate::{model::{Sketch,SolidDef},motion::Family};

type V = [f64;3];

/// A face of a sweep's tool, its kinds and the edges between them are the kernel's (#721 F2b): what
/// is read here is the drawing they are read off.
pub use crate::brep::swept::tool::{CapSurface,ExtrudedSurface,Material,PatchEdge,Tool,ToolSurface};

impl Material for SpatialField {
    fn value(&self,p: V) -> f64 { SpatialField::value(self,p) }
}

/// The faces of a continuous sweep's tool and the edges between them, with the motion that carries
/// them. Every source face is retained, including faces hidden by source Booleans; the source
/// material is what rejects hidden portions.
#[derive(Clone,Debug)]
pub struct SweepContacts {
    pub(super) source: std::sync::Arc<SpatialField>,
    /// The tool as the kernel reads it: its faces, where adjacent faces meet, where a drafted
    /// prism's sides fold, and its material.
    pub(super) tool: Tool,
    pub(super) motion: Family,
    /// The roll it is read over: the sweep's, or a whole turn's once round (`whole`).
    pub(super) roll: [f64;2],
    /// Whether the motion is a turn the sweep's roll carries round a whole turn or more (#684 N).
    pub(super) whole: bool,
    /// The tool as one prism standing alone, where its boundary is one (`Lone::read`): what the
    /// generating class sections across where its cap reaches the blank (#684 F).
    pub(super) lone: Option<std::sync::Arc<Lone>>,
}

/// A point's contact times, and why it has none, are the kernel's (`brep::swept::generating`, #721
/// F2c).
pub use crate::brep::swept::generating::{Chart,PointContactError,TimedContact};
use crate::brep::swept::admission::Lone;

impl SweepContacts {
    /// **The sweep `solid`'s tool read from its B-rep** (#721 F1b): the boundary the kernel builds
    /// of the tool, read by its faces' surfaces and the edges they share (`Tool::read`), never by
    /// how it was drawn, and its material that boundary's. A tool whose boundary has a face no
    /// reading takes is refused, naming the face and why. The motion and the roll are the
    /// drawing's.
    pub fn read(sk: &Sketch,solid: usize,axis_tolerance: f64) -> Result<Self,String> {
        let drawn = Self::motion_read(sk,solid,axis_tolerance)?;
        let SolidDef::Swept {source,..} = &sk.solids[solid].def else { unreachable!("read as a sweep") };
        let name = &sk.solids[*source as usize].name;
        let mm = super::cad::millimetres(sk);
        let b = super::cad::tool_brep(sk,solid)?;
        let tool = Tool::read(&b,mm).map_err(|why| format!("`{name}` is read by its boundary, but {why}"))?;
        let lone = Lone::read(&b,&tool,mm).map(std::sync::Arc::new);
        Ok(Self {tool,lone,..drawn})
    }

    /// The tool as one prism standing alone, where its boundary is one.
    pub fn lone(&self) -> Option<&Lone> { self.lone.as_deref() }

    /// The sweep `solid` with its motion, its roll (a whole turn's once round, #684 N) and its
    /// source's field read, and no face of its tool yet: what `read` reads the tool into.
    fn motion_read(sk: &Sketch,solid: usize,axis_tolerance: f64) -> Result<Self,String> {
        super::validate(sk,solid)?;
        let SolidDef::Swept {source,motion,..} = &sk.solids[solid].def else {
            return Err("contact construction requires a continuous swept solid".into());
        };
        let source_field = SpatialField::read(sk,*source as usize,axis_tolerance)?;
        // a whole turn or more sweeps what one whole turn does, a solid of revolution (#684 N): every
        // reader of its contacts reads it once round
        let (motion,roll) = (Family::read(sk,*motion as usize)?,super::roll::interval(sk,solid)?);
        let once = motion.twist().and_then(|screw| super::constant_twist::once_round(&screw,roll));
        let source = std::sync::Arc::new(source_field);
        let tool = Tool {faces:Vec::new(),edges:Vec::new(),creases:Vec::new(),material:source.clone()};
        Ok(Self {source,tool,motion,roll:once.unwrap_or(roll),whole:once.is_some(),lone:None})
    }

    pub fn edges(&self) -> &[PatchEdge] { &self.tool.edges }
    /// The tool as the kernel reads it.
    pub fn tool(&self) -> &Tool { &self.tool }
    /// Where `edge`'s two faces meet, and a point just inside each face beside it (`Tool::corner`).
    pub fn corner(&self,edge: &PatchEdge) -> Option<(V,[V;2])> { self.tool.corner(edge) }
    pub fn patches(&self) -> &[ToolSurface] { &self.tool.faces }
    /// Where a drafted prism's sides fold at its parting plane (#684 P): each pair the side below
    /// it, ending there at `v = 1`, and the side above, starting there at `v = 0`.
    pub fn creases(&self) -> &[[usize;2]] { &self.tool.creases }
    pub fn motion(&self) -> &Family { &self.motion }
    pub fn source_material(&self) -> &SpatialField { &self.source }
    /// The roll the contacts are read over: the sweep's, or a whole turn's once round from its start.
    pub fn domain(&self) -> [f64;2] { self.roll }
    /// Whether the sweep is a whole turn or more, read once round (`domain`): it has no ends, the
    /// tool at either lying within what it sweeps (#684 N).
    pub fn whole(&self) -> bool { self.whole }
    /// Contact times of a point carrying a given unit normal, over a motion
    /// interval. This is the sharp-edge sweep condition: a convex source edge
    /// carries every normal between its incident faces', and each such normal
    /// contacts where it is perpendicular to the point's velocity. The caller
    /// supplies the normal; nothing here checks that the edge actually carries it.
    pub fn at_point_normal_over(&self,position: [f64;3],normal: [f64;3],interval: [f64;2],tolerance: f64)
        -> Result<Vec<TimedContact>,PointContactError> {
        self.chart().map_err(PointContactError::Failed)?.at_point_normal(position,normal,interval,tolerance)
    }

    /// The same chart over a caller-supplied motion interval. Contacts outside the
    /// declared roll lie on the envelope of a longer motion, not on this sweep's
    /// boundary; a caller may use them as an extended candidate sheet only where a
    /// separate material classification decides what is exposed.
    pub fn at_source_over(&self,patch: usize,u: f64,v: f64,interval: [f64;2],tolerance: f64)
        -> Result<Vec<TimedContact>,String> {
        let surface = self.tool.faces.get(patch).ok_or("no such sweep patch")?.at(u,v)
            .map_err(|e| format!("{e:?}"))?;
        self.chart()?.at_surface(surface,interval,tolerance).map_err(|e| match e {
            PointContactError::Unsatisfied => "temporal contact failed the original normal-velocity equation".into(),
            e => e.to_string(),
        })
    }

    /// The sweep's contact times as the kernel reads them: the lowered motion and the roll
    /// (`domain`), or why the motion has none, in the drawing's words.
    pub fn chart(&self) -> Result<Chart<'_>,String> {
        Chart::new(self.motion.path()?,self.roll).map_err(|_| self.motion.uncharted())
    }
}
