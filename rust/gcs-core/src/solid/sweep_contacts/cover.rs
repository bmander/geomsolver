//! Conservative source/contact domain partition and regular source/time charts.
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
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum ContactParameter {
    /// Normalized position along the generating profile, u.
    Meridian,
    /// Normalized source revolution coordinate v, rather than an angle in radians.
    Angle,
    /// The shared generating-motion parameter (roll), rather than elapsed seconds.
    Time,
}
impl ContactParameter {
    pub fn index(self) -> usize { match self { Self::Meridian => 0,Self::Angle => 1,Self::Time => 2 } }
    pub fn free(self) -> [usize;2] { match self { Self::Meridian => [1,2],Self::Angle => [0,2],Self::Time => [0,1] } }
}
#[derive(Clone,Copy,Debug)]
pub struct ContactChart {
    pub dependent: ContactParameter,
    /// May extend beyond its partition cell, including across a full revolution's
    /// coordinate seam, but never beyond the declared motion or a partial source
    /// revolution. Contains the cell's dependent interval. Neighboring charts may
    /// overlap. Every free parameter pair has exactly one root in this interval.
    pub range: I,
    pub ends: [I;2],
    pub derivative: I,
}
#[derive(Clone,Copy,Debug)]
pub enum ContactEvidence {
    /// The complete source patch box is strictly inside or outside the cutter's
    /// material. Its contact roots cannot contribute to the cutter boundary.
    OffSource {material:I},
    /// The entire box excludes zero of the unnormalized contact equation.
    Excluded {value:I},
    /// Every free parameter pair has exactly one dependent parameter value:
    /// the endpoint signs straddle and its derivative never vanishes.
    /// The source normal is nonzero throughout. This does not prove the mapped
    /// envelope is regular, exposed, within source trims, or accurately fitted.
    Chart(ContactChart),
    /// Includes possible source poles, chart folds and time-domain boundaries.
    /// A missing value means the cell was not evaluated before budget exhaustion.
    Unresolved {value:Option<I>,limit:ContactLimit},
}
#[derive(Clone,Copy,Debug)]
pub struct ContactCell {
    pub patch: usize,
    /// Original source u, original source v, and motion parameter. A fixed-time
    /// cover has a point interval in the third coordinate.
    pub parameters: [I;3],
    pub evidence: ContactEvidence,
}
#[derive(Clone,Debug)]
pub struct ContactCover {
    /// Closed cells partition every patch's full source domain over the requested
    /// motion interval or fixed time. Relative cell interiors are disjoint; shared
    /// boundaries are retained. Unresolved cells cannot be dropped when constructing
    /// a swept boundary.
    pub cells: Vec<ContactCell>,
    pub evaluations: usize,
}
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum ContactCoverError { InvalidOptions, Arithmetic(Error) }
impl From<Error> for ContactCoverError { fn from(error: Error) -> Self { Self::Arithmetic(error) } }

enum ChartAttempt {
    Unresolved,
    Monotone,
    Proven(ContactChart),
}

impl SweepContacts {
    /// Sample a chart from this same solved snapshot. Inputs are normalized free
    /// coordinates, in the order given by `ContactParameter::free`. A missing or
    /// ambiguous root is an error; no branch is guessed from neighboring samples.
    pub fn at_chart(&self,cell: &ContactCell,a: f64,b: f64,tolerance: f64)
        -> Result<crate::envelope::Contact,String> {
        if ![a,b].iter().all(|x| x.is_finite() && (0. ..=1.).contains(x)) {
            return Err("chart coordinates must lie in [0,1]".into());
        }
        let ContactEvidence::Chart(chart) = cell.evidence else { return Err("contact cell is not a chart".into()); };
        let surface = self.patches.get(cell.patch).ok_or("no such contact patch")?;
        let [u,v] = surface.domain(); let full = [u,v,self.roll];
        for (p,domain) in cell.parameters.iter().zip(full) {
            let [a,b] = p.bounds();
            if a < domain[0] || b > domain[1] { return Err("contact cell leaves the source domain".into()); }
        }
        let axis = chart.dependent.index(); let [lo,hi] = chart.range.bounds();
        let [a0,b0] = cell.parameters[axis].bounds();
        let allowed = if chart.dependent == ContactParameter::Angle {
            surface.angular_chart_domain()
        } else { full[axis] };
        if lo < allowed[0] || hi > allowed[1] || lo > a0 || hi < b0 {
            return Err("contact chart has an invalid dependent interval".into());
        }
        let free = chart.dependent.free();
        let map = |axis: usize,s: f64| { let [a,b] = cell.parameters[axis].bounds(); (a+(b-a)*s).clamp(a,b) };
        let (a,b) = (map(free[0],a),map(free[1],b));
        let roots: Vec<_> = match chart.dependent {
            ContactParameter::Meridian => self.at_angle(cell.patch,a,b,tolerance)
                .map_err(|e| format!("{e:?}"))?.into_iter()
                .filter(|c| chart.range.contains(c.u)).map(|c| c.contact).collect(),
            ContactParameter::Time => self.at_source(cell.patch,a,b,tolerance)?.into_iter()
                .filter(|c| chart.range.contains(c.root.time)).map(|c| c.contact).collect(),
            ContactParameter::Angle => surface.angular_chart(chart.range.bounds())
                .map_err(|e| format!("{e:?}"))?
                .contacts(a,self.motion.at(b)?,tolerance).map_err(|e| format!("{e:?}"))?.into_iter()
                .map(|c| c.contact).collect(),
        };
        if roots.len() != 1 { return Err(format!("contact chart has {} roots at its free coordinates",roots.len())); }
        Ok(roots[0])
    }

    fn contact_chart(&self,patch: usize,mut parameters: [I;3],dependent: ContactParameter,range: I)
        -> Result<ChartAttempt,Error> {
        parameters[dependent.index()] = range;
        let [u,v,time] = parameters;
        let surface = self.patches[patch].angular_chart(v.bounds()).map_err(|_| Error::OutsideDomain)?;
        let bounds = surface.bounds(u,v)?;
        if bounds.normal.iter().all(|x| x.contains(0.)) { return Ok(ChartAttempt::Unresolved); }
        let equation = self.motion.normal_velocity_moment_bounds(bounds.normal,bounds.moment)?;
        let derivative = match dependent {
            ContactParameter::Meridian => self.motion.normal_velocity_moment_bounds(bounds.normal_du,bounds.moment_du)?.at(time)?,
            ContactParameter::Time => equation.derivative(time)?,
            ContactParameter::Angle => self.motion.normal_velocity_moment_bounds(bounds.normal_dv,bounds.moment_dv)?.at(time)?,
        };
        if derivative.contains(0.) { return Ok(ChartAttempt::Unresolved); }
        let [a,b] = range.bounds();
        let mut ends = [I::ZERO;2];
        for (i,endpoint) in [a,b].into_iter().enumerate() {
            ends[i] = match dependent {
                ContactParameter::Time => equation.at(I::point(endpoint)?)?,
                ContactParameter::Angle | ContactParameter::Meridian => {
                    let mut source = [u,v]; source[dependent.index()] = I::point(endpoint)?;
                    let bounds = surface.bounds(source[0],source[1])?;
                    self.motion.normal_velocity_moment_bounds(bounds.normal,bounds.moment)?.at(time)?
                }
            };
        }
        let opposite = |a: I,b: I| a.bounds()[1] < 0. && b.bounds()[0] > 0.;
        Ok(if opposite(ends[0],ends[1]) || opposite(ends[1],ends[0]) {
            ChartAttempt::Proven(ContactChart {dependent,range,ends,derivative})
        } else { ChartAttempt::Monotone })
    }

    pub fn cover(&self,options: ContactCoverOptions) -> Result<ContactCover,ContactCoverError> {
        self.cover_domain(self.roll,options)
    }

    /// Contact-curve charts at one declared motion parameter, for endpoint caps
    /// or intermediate sections. Time is fixed, so only source coordinates can
    /// be dependent. The complete source domain, including events, is retained.
    pub fn cover_at(&self,time: f64,options: ContactCoverOptions) -> Result<ContactCover,ContactCoverError> {
        if !time.is_finite() { return Err(Error::InvalidBounds.into()); }
        if time < self.roll[0] || time > self.roll[1] { return Err(Error::OutsideDomain.into()); }
        self.cover_domain([time,time],options)
    }

    fn cover_domain(&self,domain: [f64;2],options: ContactCoverOptions) -> Result<ContactCover,ContactCoverError> {
        if options.max_depth > 48 || options.max_cells == 0 { return Err(ContactCoverError::InvalidOptions); }
        let mut pending = Vec::new();
        let time = I::new(domain[0],domain[1])?;
        for surface in &self.patches {
            let [u,v] = surface.domain();
            pending.push(std::collections::VecDeque::from([
                ([I::new(u[0],u[1])?,I::new(v[0],v[1])?,time],0_u8)]));
        }
        self.cover_pending(domain,options,pending,Vec::new())
    }

    /// Refine selected unresolved cells, preserving every other partition cell.
    /// max_depth counts additional subdivisions and max_cells bounds additional
    /// evaluations. The returned evaluation count includes the previous work.
    pub(super) fn refine_cells(&self,cover: ContactCover,selected: &[usize],options: ContactCoverOptions)
        -> Result<ContactCover,ContactCoverError> {
        if options.max_depth > 48 || options.max_cells == 0 { return Err(ContactCoverError::InvalidOptions); }
        let mut pending = vec![std::collections::VecDeque::new();self.patches.len()];
        let selected: std::collections::BTreeSet<_> = selected.iter().copied().collect();
        if selected.iter().any(|&i| i >= cover.cells.len()) { return Err(ContactCoverError::InvalidOptions); }
        let mut retained = Vec::new();
        let domain = cover.cells.first().map(|c| c.parameters[2].bounds()).unwrap_or(self.roll);
        if domain[0] != domain[1] || cover.cells.iter().any(|c| c.parameters[2].bounds() != domain) {
            return Err(ContactCoverError::InvalidOptions);
        }
        for (i,cell) in cover.cells.into_iter().enumerate() {
            if selected.contains(&i) {
                if cell.patch >= pending.len() || !matches!(cell.evidence,ContactEvidence::Unresolved {..}) {
                    return Err(ContactCoverError::InvalidOptions);
                }
                pending[cell.patch].push_back((cell.parameters,0));
            } else { retained.push(cell); }
        }
        let mut refined = self.cover_pending(domain,options,pending,retained)?;
        refined.evaluations += cover.evaluations;
        Ok(refined)
    }

    fn cover_pending(&self,domain: [f64;2],options: ContactCoverOptions,
        mut pending: Vec<std::collections::VecDeque<([I;3],u8)>>,cells: Vec<ContactCell>) -> Result<ContactCover,ContactCoverError> {
        let mut result = ContactCover {cells,evaluations:0};
        // Rotate among patches. Fixed-time curves use breadth-first subdivision
        // so a difficult branch cannot leave half a face unvisited. Full motion
        // uses depth-first refinement to reach regular surface charts early.
        let mut cursor = 0;
        loop {
            let Some(offset) = (0..pending.len()).find(|i| !pending[(cursor+i)%pending.len()].is_empty()) else { break; };
            let patch = (cursor+offset)%pending.len(); cursor = (patch+1)%pending.len();
            let (parameters,depth) = pending[patch].pop_front().unwrap();
            let unresolved = |value,limit| ContactCell {patch,parameters,
                evidence:ContactEvidence::Unresolved {value,limit}};
            if result.evaluations >= options.max_cells {
                result.cells.push(unresolved(None,ContactLimit::Budget)); continue;
            }
            result.evaluations += 1;
            let [u,v,time] = parameters;
            let surface = self.patches[patch].bounds(u,v)?;
            let equation = self.motion.normal_velocity_moment_bounds(surface.normal,surface.moment)?;
            let value = equation.at(time)?;
            if !value.contains(0.) {
                result.cells.push(ContactCell {patch,parameters,evidence:ContactEvidence::Excluded {value}});
                continue;
            }
            let material = self.source.bounds(surface.position)?;
            if !material.contains(0.) {
                result.cells.push(ContactCell {patch,parameters,evidence:ContactEvidence::OffSource {material}});
                continue;
            }
            let derivative = if domain[0] == domain[1] { I::ZERO } else { equation.derivative(time)? };
            let angular = self.motion.normal_velocity_moment_bounds(surface.normal_dv,surface.moment_dv)?.at(time)?;
            let meridian = self.motion.normal_velocity_moment_bounds(surface.normal_du,surface.moment_du)?.at(time)?;
            let slopes = [meridian,angular,derivative];
            let [source_u,source_v] = self.patches[patch].domain();
            let original = [source_u,source_v,domain];
            let order = [ContactParameter::Time,ContactParameter::Angle,ContactParameter::Meridian];
            let mut chart = None;
            let mut monotone = slopes.map(|s| !s.contains(0.));
            for dependent in order {
                if dependent == ContactParameter::Time && equation.is_time_independent() { continue; }
                let slope = slopes[dependent.index()];
                if slope.contains(0.) || surface.normal.iter().all(|x| x.contains(0.)) { continue; }
                let axis = dependent.index();
                let range = parameters[axis];
                let mut attempt = self.contact_chart(patch,parameters,dependent,range)?;
                if !matches!(attempt,ChartAttempt::Proven(_)) {
                    // A root on an internal partition plane needs an overlapping
                    // chart. Full revolutions also permit local continuation
                    // through their coordinate seam. Physical endpoints do not.
                    let full = match dependent {
                        ContactParameter::Time => domain,
                        ContactParameter::Angle => self.patches[patch].angular_chart_domain(),
                        ContactParameter::Meridian => self.patches[patch].domain()[0],
                    };
                    let [a,b] = range.bounds(); let half = (b-a)*0.5;
                    let expanded = I::new((a-half).max(full[0]),(b+half).min(full[1]))?;
                    if expanded != range && (dependent != ContactParameter::Angle || expanded.bounds()[1]-expanded.bounds()[0] <= 1.) {
                        attempt = self.contact_chart(patch,parameters,dependent,expanded)?;
                    }
                }
                // Monotonicity of the original box alone cannot justify always
                // refining free coordinates when the overlapping box loses it.
                monotone[axis] = !matches!(attempt,ChartAttempt::Unresolved);
                if let ChartAttempt::Proven(proven) = attempt { chart = Some(proven); }
                if chart.is_some() { break; }
            }
            if let Some(chart) = chart {
                result.cells.push(ContactCell {patch,parameters,evidence:ContactEvidence::Chart(chart)});
                continue;
            }
            if depth == options.max_depth {
                result.cells.push(unresolved(Some(value),ContactLimit::Depth)); continue;
            }
            // Once a coordinate is monotone, refine its free parameters instead
            // of making the root bracket narrower. Otherwise split the
            // widest fraction of the original domain. Exact time independence
            // also prevents useless time splits. Sampling never discards a box.
            let dependent = order.into_iter().map(|p| p.index()).find(|&axis| monotone[axis]);
            let fraction = |axis: usize| {
                if axis == 2 && equation.is_time_independent() { return 0.; }
                if dependent == Some(axis) { return 0.; }
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
                if domain[0] == domain[1] { pending[patch].push_back((child,depth+1)); }
                else { pending[patch].push_front((child,depth+1)); }
            }
        }
        Ok(result)
    }
}
