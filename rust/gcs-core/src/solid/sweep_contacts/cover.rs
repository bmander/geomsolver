//! Conservative source/contact domain partition and regular temporal charts.
use super::SweepContacts;
use crate::interval::{Interval as I,Error};

#[derive(Clone,Copy,Debug)]
pub struct ContactCoverOptions {
    /// Binary subdivision depth over the three parameters together, at most 48.
    pub max_depth: u8,
    /// Maximum evaluated cells across every source patch. Pending domains remain
    /// explicit if this limit is reached; no partial cover is called complete.
    pub max_cells: usize,
}
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum ContactLimit { Depth, Budget, Resolution }
#[derive(Clone,Copy,Debug)]
pub enum ContactEvidence {
    /// The entire box excludes zero of the unnormalized contact equation.
    Excluded {value:I},
    /// Every (u,v) in the box has exactly one contact time in its time interval:
    /// the endpoint signs straddle and the time derivative never vanishes.
    /// The source normal is nonzero throughout. This does not prove the mapped
    /// envelope is regular, exposed, within source trims, or accurately fitted.
    TemporalChart {ends:[I;2],derivative:I},
    /// Includes possible source poles, chart folds and time-domain boundaries.
    /// A missing value means the cell was not evaluated before budget exhaustion.
    Unresolved {value:Option<I>,limit:ContactLimit},
}
#[derive(Clone,Copy,Debug)]
pub struct ContactCell {
    pub patch: usize,
    /// Original source u, original source v, and declared motion time.
    pub parameters: [I;3],
    pub evidence: ContactEvidence,
}
#[derive(Clone,Debug)]
pub struct ContactCover {
    /// Closed cells partition every patch's full parameter domain. Cell interiors
    /// are disjoint; all shared boundaries are retained. Unresolved cells cannot
    /// be dropped when constructing a swept boundary.
    pub cells: Vec<ContactCell>,
    pub evaluations: usize,
}
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum ContactCoverError { InvalidOptions, Arithmetic(Error) }
impl From<Error> for ContactCoverError { fn from(error: Error) -> Self { Self::Arithmetic(error) } }

impl SweepContacts {
    pub fn cover(&self,options: ContactCoverOptions) -> Result<ContactCover,ContactCoverError> {
        if options.max_depth > 48 || options.max_cells == 0 { return Err(ContactCoverError::InvalidOptions); }
        let mut pending = Vec::new();
        let time = I::new(self.roll[0],self.roll[1])?;
        for surface in &self.patches {
            let [u,v] = surface.domain();
            pending.push(vec![([I::new(u[0],u[1])?,I::new(v[0],v[1])?,time],0_u8)]);
        }
        let mut result = ContactCover {cells:Vec::new(),evaluations:0};
        // Depth-first refinement reaches small charts promptly, while rotating
        // among patches prevents one difficult face consuming every query.
        let mut cursor = 0;
        loop {
            let Some(offset) = (0..pending.len()).find(|i| !pending[(cursor+i)%pending.len()].is_empty()) else { break; };
            let patch = (cursor+offset)%pending.len(); cursor = (patch+1)%pending.len();
            let (parameters,depth) = pending[patch].pop().unwrap();
            let unresolved = |value,limit| ContactCell {patch,parameters,
                evidence:ContactEvidence::Unresolved {value,limit}};
            if result.evaluations >= options.max_cells {
                result.cells.push(unresolved(None,ContactLimit::Budget)); continue;
            }
            result.evaluations += 1;
            let [u,v,time] = parameters;
            let surface = self.patches[patch].bounds(u,v)?;
            let equation = self.motion.normal_velocity_bounds(surface.position,surface.normal)?;
            let value = equation.at(time)?;
            if !value.contains(0.) {
                result.cells.push(ContactCell {patch,parameters,evidence:ContactEvidence::Excluded {value}});
                continue;
            }
            let derivative = equation.derivative(time)?;
            if !derivative.contains(0.) && surface.normal.iter().any(|x| !x.contains(0.)) {
                let [a,b] = time.bounds();
                let ends = [equation.at(I::point(a)?)?,equation.at(I::point(b)?)?];
                let opposite = |a: I,b: I| a.bounds()[1] < 0. && b.bounds()[0] > 0.;
                if opposite(ends[0],ends[1]) || opposite(ends[1],ends[0]) {
                    result.cells.push(ContactCell {patch,parameters,
                        evidence:ContactEvidence::TemporalChart {ends,derivative}});
                    continue;
                }
            }
            if depth == options.max_depth {
                result.cells.push(unresolved(Some(value),ContactLimit::Depth)); continue;
            }
            // Once time is monotone, refine the free source parameters instead
            // of making the temporal root bracket narrower. Otherwise split the
            // widest fraction of the original domain. Exact time independence
            // also prevents useless time splits. Sampling never discards a box.
            let [u,v] = self.patches[patch].domain();
            let original = [u,v,self.roll];
            let fraction = |axis: usize| {
                if axis == 2 && (equation.is_time_independent() || !derivative.contains(0.)) { return 0.; }
                let span = original[axis][1]-original[axis][0];
                let [a,b] = parameters[axis].bounds();
                if span > 0. { (b-a)/span } else { 0. }
            };
            let axis = (0..3).max_by(|a,b| fraction(*a).total_cmp(&fraction(*b))).unwrap();
            let [a,b] = parameters[axis].bounds(); let mid = a*0.5+b*0.5;
            if mid <= a || mid >= b {
                result.cells.push(unresolved(Some(value),ContactLimit::Resolution)); continue;
            }
            for interval in [I::new(mid,b)?,I::new(a,mid)?] {
                let mut child = parameters; child[axis] = interval;
                pending[patch].push((child,depth+1));
            }
        }
        Ok(result)
    }
}
