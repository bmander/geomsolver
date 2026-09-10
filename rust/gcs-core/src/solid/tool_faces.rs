//! The boundary of a swept tool as faces and edges, by surface family. The
//! language builds solids bounded by planes, cylinders, cones, spheres and
//! tori: revolved faces, and the planar and extruded faces of a prism. Each
//! family has an exact contact equation along one station of the face under
//! an instantaneous motion, which is what the tracer in `sweep_candidates`
//! walks; an edge is a shared boundary curve of two faces, charted on both.
use super::{RevolvedSurface,surface::contact::sinusoid_roots};
use crate::envelope::{self,Error,Motion,SurfacePoint};

type V3 = [f64;3];

/// One face of the tool's boundary, in the tool's own frame.
#[derive(Clone,Debug)]
pub enum ToolFace {
    /// A profile edge revolved about an axis: `u` along the meridian, `v` the
    /// fraction of the revolution.
    Revolved(RevolvedSurface),
}

/// The contact condition `n·v = 0` along one station of a face, with the
/// station's own angular chart.
#[derive(Clone,Copy,Debug)]
pub enum StationEquation {
    /// `a cos θ + b sin θ + c = 0`, `θ = v·sweep`, over `domain` in `v`.
    Sinusoid { a: f64, b: f64, c: f64, sweep: f64, domain: [f64;2] },
}

impl StationEquation {
    /// The amplitude of the equation's varying part and its constant: a zero
    /// amplitude makes the whole station stationary or nowhere in contact.
    pub fn amplitude_and_constant(&self) -> (f64,f64) {
        match *self { Self::Sinusoid {a,b,c,..} => (a.hypot(b),c) }
    }

    /// Isolated roots as `(branch, v)`.
    pub fn roots(&self,tolerance: f64) -> Result<Vec<(usize,f64)>,Error> {
        match *self {
            Self::Sinusoid {a,b,c,sweep,domain} => sinusoid_roots(a,b,c,sweep,domain,tolerance),
        }
    }
}

impl ToolFace {
    pub fn at(&self,u: f64,v: f64) -> Result<SurfacePoint,Error> {
        match self { Self::Revolved(s) => s.at(u,v) }
    }

    /// The face's own domain in `(u, v)`.
    pub fn domain(&self) -> [[f64;2];2] {
        match self { Self::Revolved(s) => s.domain() }
    }

    /// The contact equation along the station at `u` under `motion`.
    /// `Degenerate` where the station has no angular extent (a pole).
    pub fn station(&self,u: f64,motion: Motion) -> Result<StationEquation,Error> {
        match self {
            Self::Revolved(s) => {
                let (a,b,c) = s.contact_coefficients(u,motion)?;
                let [_,domain] = s.domain();
                Ok(StationEquation::Sinusoid {a,b,c,sweep:s.sweep(),domain})
            }
        }
    }

    /// The outward-oriented unit normal at `(u, v)` given the face's sign.
    pub fn normal(&self,u: f64,v: f64,sign: f64) -> Result<(V3,V3),Error> {
        let s = self.at(u,v)?;
        let n = envelope::contact(s,Motion::identity())?.normal;
        Ok((s.position,n.map(|x| x*sign)))
    }

    /// Where a meridian end lies on the revolution axis the station there is a
    /// pole every branch meets: its position and the axis direction, which is
    /// the tangent plane's normal there.
    pub fn pole(&self,end: f64,scale: f64) -> Result<Option<(V3,V3)>,Error> {
        match self {
            Self::Revolved(s) => {
                let p = s.at(end,0.)?;
                let dv = p.dv[0].hypot(p.dv[1]).hypot(p.dv[2]);
                Ok((dv <= scale*1e-9).then(|| (p.position,s.axis_direction())))
            }
        }
    }
}

/// Where along a face's boundary an edge runs: the chart from the edge's
/// parameter `t` in `[0, 1]` to the face's `(u, v)`.
#[derive(Clone,Copy,Debug)]
pub enum EdgeChart {
    /// `u` fixed, `v` running over the face's `v` domain.
    FixedU(f64),
    /// `v` fixed, `u` running over `[0, 1]`.
    FixedV(f64),
}

impl EdgeChart {
    pub fn at(self,t: f64,domain: [[f64;2];2]) -> (f64,f64) {
        match self {
            Self::FixedU(u) => (u,domain[1][0]+t*(domain[1][1]-domain[1][0])),
            Self::FixedV(v) => (t,v),
        }
    }
}

/// A boundary curve shared by two faces, charted on each so both incident
/// normals are read at one parameter.
#[derive(Clone,Copy,Debug)]
pub struct ToolEdge {
    pub faces: [usize;2],
    pub charts: [EdgeChart;2],
}
