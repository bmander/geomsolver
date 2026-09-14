//! Structural circle extraction from native revolved charts. No sampled fit.
use super::{RevolvedSurface,Meridian};
use crate::{interval::exact_difference,space::coordinate_axis};

#[derive(Clone,Copy,Debug,PartialEq)]
pub(crate) struct CoordinateCircle {
    pub axis: usize,
    pub direction: f64,
    pub centre: [f64;3],
    pub radial: [f64;3],
}
impl RevolvedSurface {
    pub(crate) fn coordinate_circle(&self,u: f64) -> Option<CoordinateCircle> {
        let (axis,direction) = coordinate_axis(self.axis)?;
        let Meridian::Line {start,delta} = self.meridian else { return None; };
        // Endpoints have an algebraic position in the stored line snapshot.
        // General u and round meridians need an additional exact representation.
        let mut point = start;
        if u == 1. {
            for k in 0..3 { point[k] = exact_difference(start[k],-delta[k])?; }
        } else if u != 0. { return None; }
        let mut centre = self.origin; centre[axis] = point[axis];
        let mut radial = [0.;3];
        for k in 0..3 { if k != axis { radial[k] = exact_difference(point[k],centre[k])?; } }
        if radial == [0.;3] { return None; }
        Some(CoordinateCircle {axis,direction,centre,radial})
    }
}
