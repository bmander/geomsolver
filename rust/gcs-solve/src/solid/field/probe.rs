//! Material-side evidence for boundary candidates, using the complete solid DAG.
use super::{MaterialEvaluator,MaterialBounds,SweepError,I,V,norm,point};
use crate::interval::minimum::{Options,Stop};

#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum ProbeState {
    /// Every closed ball of the requested radius about the input box is material.
    InteriorBall,
    /// Every such closed ball is exterior to material.
    ExteriorBall,
    /// Minus is material and plus is exterior along the supplied direction.
    OutwardBracket,
    /// Minus is exterior and plus is material.
    InwardBracket,
    /// The enclosures establish none of the above; this never means exposed.
    Unresolved,
}

/// A bracket guarantees at least one material boundary on each corresponding
/// offset segment, within `distance` of its origin. It does not establish a
/// unique crossing, correct topology, surface normal or a whole-face error bound.
#[derive(Clone,Debug)]
pub struct MaterialProbe {
    pub state: ProbeState,
    pub center: MaterialBounds,
    /// Minus/plus queries, absent when the center already proves a ball margin.
    pub sides: Option<[MaterialBounds;2]>,
    /// The translated point boxes used for minus/plus, including arithmetic error.
    pub side_boxes: [V;2],
}

impl MaterialEvaluator {
    /// Check a candidate box against all static, swept and Boolean material.
    /// The one-Lipschitz field provides ball margins. Otherwise probe at +/-
    /// `distance` along the normalized direction, with outward-rounded offsets.
    /// All returned evidence applies to the entire input/translated boxes.
    /// Options apply per swept leaf per query, with at most three root queries.
    /// A field-zero alone or an exhausted budget cannot imply a bracket.
    pub fn probe(&mut self,position: V,direction: [f64;3],distance: f64,options: Options)
        -> Result<MaterialProbe,SweepError> {
        if !options.valid() || !distance.is_finite() || distance <= 0.
            || !direction.iter().all(|v| v.is_finite()) { return Err(SweepError::InvalidOptions); }
        let scale = direction.iter().map(|v| v.abs()).fold(0.,f64::max);
        if scale == 0. { return Err(SweepError::InvalidOptions); }
        let side_boxes = (|| {
            let mut direction = point(direction)?;
            let scale = I::point(scale)?;
            for x in &mut direction { *x = x.div(scale)?; }
            let length = norm(direction)?;
            let distance = I::point(distance)?;
            let mut sides = [position;2];
            for k in 0..3 {
                let delta = direction[k].div(length)?.mul(distance)?;
                sides[0][k] = position[k].sub(delta)?;
                sides[1][k] = position[k].add(delta)?;
            }
            Ok(sides)
        })().map_err(SweepError::Oracle)?;
        let center = self.query(position,Stop::Outside(I::new(-distance,distance).unwrap()),options,None)?;
        let [lo,hi] = center.value.bounds();
        if hi < -distance || lo > distance {
            return Ok(MaterialProbe {state:if hi < -distance { ProbeState::InteriorBall }
                else { ProbeState::ExteriorBall },center,sides:None,side_boxes});
        }
        let minus = self.query(side_boxes[0],Stop::Outside(I::ZERO),options,None)?;
        let plus = self.query(side_boxes[1],Stop::Outside(I::ZERO),options,None)?;
        let [ml,mh] = minus.value.bounds(); let [pl,ph] = plus.value.bounds();
        let state = if mh < 0. && pl > 0. { ProbeState::OutwardBracket }
            else if ml > 0. && ph < 0. { ProbeState::InwardBracket }
            else { ProbeState::Unresolved };
        Ok(MaterialProbe {state,center,sides:Some([minus,plus]),side_boxes})
    }
}
