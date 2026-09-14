//! Analytically established common spherical carriers for coordinate-axis circle
//! sweeps. Identity uses structural/exact checks, never a sampled sphere fit.
//! Regular local maps are certified over native boxes. Point inversion remains
//! a numerical solve, not a certified point enclosure or a visibility decision.
use crate::{envelope,interval::{Interval as I,Error as Arithmetic},solid::{SweepContacts,
    ToolFace,EdgeChart,sweep_source::{Source,SourcePoint,Evaluation}},space::distance};
type V = [f64;3];
type Box2 = [[f64;2];2];

#[derive(Clone,Debug,PartialEq)]
pub enum Error {
    InvalidInput, MissingEdge, UnsupportedSource, UnsupportedMotion,
    UnestablishedCarrier, OutsideDomain, NonFinite,
    Arithmetic(Arithmetic), Source(crate::solid::sweep_source::Error), Solve(envelope::Error),
}
impl From<Arithmetic> for Error { fn from(e: Arithmetic) -> Self { Self::Arithmetic(e) } }
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Band { pub edge: usize, pub domain: Box2 }
#[derive(Clone,Copy,Debug)]
pub struct Options { pub max_depth: u8, pub max_cells: usize }
/// Reasons regularity could not be established, not classified geometric events.
#[derive(Clone,Debug,PartialEq)]
pub enum Limit { FoldOrPole, AngularChart, Budget, Bounds(Error) }
#[derive(Clone,Debug)]
pub struct Unresolved { pub band: usize, pub domain: Box2, pub reason: Limit }

/// A projection chart uses the motion-axis coordinate and one transverse
/// coordinate. The omitted coordinate has a fixed sign relative to the centre.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub struct Chart { pub omitted: usize, pub positive: bool }
#[derive(Clone,Debug)]
pub struct Region {
    band: usize,
    domain: Box2,
    chart: Chart,
    axial_derivative: I,
    angular_derivative: I,
}
impl Region {
    pub fn band(&self) -> usize { self.band }
    pub fn domain(&self) -> Box2 { self.domain }
    pub fn chart(&self) -> Chart { self.chart }
    /// Nonzero derivatives establish global injectivity on the native rectangle:
    /// axial coordinate is monotone in edge t and independent of roll; with t
    /// fixed, the other projected coordinate is monotone in roll.
    pub fn derivatives(&self) -> [I;2] { [self.axial_derivative,self.angular_derivative] }
    /// Orientation into the projection chart, not an outward material normal.
    pub fn orientation(&self) -> i8 {
        if (self.axial_derivative.bounds()[0] > 0.) ==
            (self.angular_derivative.bounds()[0] > 0.) { 1 } else { -1 }
    }
}
#[derive(Clone,Debug)]
pub struct Mapped {
    pub native: [f64;2],
    pub chart: Chart,
    /// Absolute world coordinates [axial, retained transverse].
    pub coordinates: [f64;2],
    pub source: SourcePoint,
    pub evaluation: Evaluation,
}
#[derive(Clone,Copy,Debug,PartialEq)]
struct Sphere { centre: V, terms: V }
struct Circle { face: usize, u: f64, angular: [f64;2] }

pub struct Correspondence<'a> {
    sweep: &'a SweepContacts,
    bands: Vec<Band>,
    circles: Vec<Circle>,
    sphere: Sphere,
    axis: usize,
    rate: f64,
    regions: Vec<Region>,
    unresolved: Vec<Unresolved>,
    evaluations: usize,
}
impl<'a> Correspondence<'a> {
    pub fn new(sweep: &'a SweepContacts,bands: &[Band],options: Options) -> Result<Self,Error> {
        if bands.is_empty() || options.max_depth > 48 { return Err(Error::InvalidInput); }
        let motion = sweep.motion().coordinate_rotation().ok_or(Error::UnsupportedMotion)?;
        let mut circles = Vec::new(); let mut sphere = None;
        for band in bands {
            let d = band.domain; let time = sweep.domain();
            if d.iter().any(|b| !b.iter().all(|x| x.is_finite()) || b[0] >= b[1])
                || d[0][0] < 0. || d[0][1] > 1. || d[1][0] < time[0] || d[1][1] > time[1] {
                return Err(Error::InvalidInput);
            }
            let edge = sweep.edges().get(band.edge).ok_or(Error::MissingEdge)?;
            let mut common = None;
            for k in 0..2 {
                let ToolFace::Revolved(f) = &sweep.faces()[edge.faces[k]] else {
                    return Err(Error::UnsupportedSource);
                };
                let EdgeChart::FixedU(u) = edge.charts[k] else { return Err(Error::UnsupportedSource); };
                let circle = f.coordinate_circle(u).ok_or(Error::UnsupportedSource)?;
                if circle.axis == motion.axis { return Err(Error::UnestablishedCarrier); }
                let transverse = 3-circle.axis-motion.axis;
                // The two axis lines must intersect exactly. Otherwise this is
                // generally a different surface of revolution, however close.
                if circle.centre[transverse] != motion.origin[transverse] {
                    return Err(Error::UnestablishedCarrier);
                }
                let mut centre = circle.centre; centre[circle.axis] = motion.origin[circle.axis];
                let mut terms = circle.radial;
                terms[circle.axis] = crate::interval::exact_difference(circle.centre[circle.axis],centre[circle.axis])
                    .ok_or(Error::UnestablishedCarrier)?;
                let mut terms = terms.map(f64::abs); terms.sort_by(f64::total_cmp);
                let key = Sphere {centre,terms};
                // Equal unordered squared terms is a sufficient exact identity
                // test; unequal decompositions may still describe the same sphere
                // and intentionally refuse. Never round a sum of squares to compare.
                if sphere.is_some_and(|old| old != key) { return Err(Error::UnestablishedCarrier); }
                sphere = Some(key);
                let signature = (circle,f.sweep(),f.domain()[1]);
                if common.is_some_and(|old| old != signature) {
                    return Err(Error::UnestablishedCarrier);
                }
                common = Some(signature);
                if k == 0 { circles.push(Circle {face:edge.faces[0],u,angular:f.domain()[1]}); }
            }
        }
        let mut out = Self {sweep,bands:bands.to_vec(),circles,sphere:sphere.unwrap(),
            axis:motion.axis,rate:motion.rate,regions:Vec::new(),unresolved:Vec::new(),evaluations:0};
        for (band,b) in bands.iter().enumerate() {
            let mut pending = vec![(b.domain,0u8)];
            while let Some((domain,depth)) = pending.pop() {
                if out.evaluations >= options.max_cells {
                    out.unresolved.push(Unresolved {band,domain,reason:Limit::Budget}); continue;
                }
                out.evaluations += 1;
                let (region,reason) = match out.regular(band,domain) {
                    Ok(result) => result,
                    Err(error) => (None,Limit::Bounds(error)),
                };
                if let Some(region) = region { out.regions.push(region); continue; }
                let axis = if reason == Limit::FoldOrPole { 0 } else {
                    if depth%2 == 0 { 1 } else { 0 }
                };
                let [a,b] = domain[axis]; let mid = a*0.5+b*0.5;
                if depth >= options.max_depth || mid == a || mid == b {
                    out.unresolved.push(Unresolved {band,domain,reason}); continue;
                }
                let (mut lo,mut hi) = (domain,domain); lo[axis][1] = mid; hi[axis][0] = mid;
                pending.push((hi,depth+1)); pending.push((lo,depth+1));
            }
        }
        Ok(out)
    }
    pub fn centre(&self) -> V { self.sphere.centre }
    /// Radius squared is the exact sum of squares of these binary64 terms.
    pub fn radius_terms(&self) -> V { self.sphere.terms }
    pub fn regions(&self) -> &[Region] { &self.regions }
    pub fn unresolved(&self) -> &[Unresolved] { &self.unresolved }
    pub fn bands(&self) -> &[Band] { &self.bands }
    /// Native-cell bound attempts; numerical inverse solves are separate work.
    pub fn evaluations(&self) -> usize { self.evaluations }

    fn regular(&self,band: usize,domain: Box2) -> Result<(Option<Region>,Limit),Error> {
        let c = &self.circles[band];
        let ToolFace::Revolved(f) = &self.sweep.faces()[c.face] else { unreachable!() };
        let t = I::new(domain[0][0],domain[0][1])?;
        let scale = I::point(c.angular[1])?.sub(I::point(c.angular[0])?)?;
        let v = I::point(c.angular[0])?.add(t.mul(scale)?)?;
        let f = f.angular_chart(v.bounds()).map_err(Error::Solve)?;
        let source = f.bounds(I::point(c.u)?,v)?;
        let axial = source.dv[self.axis].mul(scale)?;
        if axial.contains(0.) { return Ok((None,Limit::FoldOrPole)); }
        let roll = I::new(domain[1][0],domain[1][1])?;
        let position = self.sweep.motion().bounds(roll)?.point(source.position)?;
        for omitted in 0..3 { if omitted != self.axis {
            let z = position[omitted].sub(I::point(self.sphere.centre[omitted])?)?;
            if z.contains(0.) { continue; }
            let retained = 3-self.axis-omitted;
            let sign = if (self.axis+1)%3 == retained { -1. } else { 1. };
            let angular = z.mul(I::point(sign*self.rate)?)?;
            if angular.contains(0.) { continue; }
            return Ok((Some(Region {band,domain,chart:Chart {omitted,positive:z.bounds()[0] > 0.},
                axial_derivative:axial,angular_derivative:angular}),Limit::AngularChart));
        } }
        Ok((None,Limit::AngularChart))
    }
    pub fn forward(&self,region: usize,native: [f64;2]) -> Result<Mapped,Error> {
        let r = self.regions.get(region).ok_or(Error::InvalidInput)?;
        if native.iter().enumerate().any(|(k,x)| !x.is_finite() ||
            !(r.domain[k][0]..=r.domain[k][1]).contains(x)) { return Err(Error::OutsideDomain); }
        let source = SourcePoint {source:Source::Edge(self.bands[r.band].edge),parameters:[native[0],0.]};
        let evaluation = self.sweep.source_at(source,native[1]).map_err(Error::Source)?;
        if !evaluation.position.iter().all(|x| x.is_finite()) { return Err(Error::NonFinite); }
        let retained = 3-self.axis-r.chart.omitted;
        Ok(Mapped {native,chart:r.chart,coordinates:[evaluation.position[self.axis],evaluation.position[retained]],
            source,evaluation})
    }
    /// Invert this region's projection chart. Injectivity follows from its
    /// interval derivative evidence. Successful numerical inversion is checked
    /// on the original edge; failure is unresolved, not proof of nonmembership.
    pub fn inverse(&self,region: usize,coordinates: [f64;2],seed: [f64;2],
        agreement: f64) -> Result<Mapped,Error> {
        let r = self.regions.get(region).ok_or(Error::InvalidInput)?;
        if !coordinates.iter().all(|x| x.is_finite()) || !agreement.is_finite() || agreement <= 0. {
            return Err(Error::InvalidInput);
        }
        let retained = 3-self.axis-r.chart.omitted;
        let a = coordinates[0]-self.sphere.centre[self.axis];
        let b = coordinates[1]-self.sphere.centre[retained];
        let radius2: f64 = self.sphere.terms.iter().map(|x| x*x).sum();
        let square = radius2-a*a-b*b;
        if !square.is_finite() { return Err(Error::NonFinite); }
        if square <= 0. { return Err(Error::Solve(envelope::Error::NotConverged)); }
        let mut target = self.sphere.centre;
        target[self.axis] = coordinates[0]; target[retained] = coordinates[1];
        target[r.chart.omitted] += square.sqrt()*if r.chart.positive { 1. } else { -1. };
        let options = envelope::IntersectionOptions {bounds:[r.domain[0],r.domain[1],[0.,0.]],
            parameter_scale:[r.domain[0][1]-r.domain[0][0],r.domain[1][1]-r.domain[1][0],1.],
            residual_tolerance:[agreement/3_f64.sqrt();3],max_iterations:80};
        let solved = crate::intersection::solve(|p| {
            let m = self.forward(region,[p[0],p[1]]).map_err(|_| envelope::Error::OutsideDomain)?;
            let residual = std::array::from_fn(|k| m.evaluation.position[k]-target[k]);
            Ok((m,residual))
        },[seed[0],seed[1],0.],options).map_err(Error::Solve)?.value;
        if distance(solved.evaluation.position,target) > agreement { return Err(Error::Solve(envelope::Error::NotConverged)); }
        Ok(solved)
    }
    /// Transfer a point between two native regions, preserving each original
    /// evaluation. Regions on opposite hemispheres must not silently alias.
    pub fn transfer(&self,from: usize,to: usize,native: [f64;2],seed: [f64;2],
        agreement: f64) -> Result<[Mapped;2],Error> {
        let a = self.forward(from,native)?;
        let r = self.regions.get(to).ok_or(Error::InvalidInput)?;
        let z = a.evaluation.position[r.chart.omitted]-self.sphere.centre[r.chart.omitted];
        if z == 0. || (z > 0.) != r.chart.positive { return Err(Error::OutsideDomain); }
        let retained = 3-self.axis-r.chart.omitted;
        let b = self.inverse(to,[a.evaluation.position[self.axis],a.evaluation.position[retained]],seed,agreement)?;
        if distance(a.evaluation.position,b.evaluation.position) > agreement {
            return Err(Error::Solve(envelope::Error::NotConverged));
        }
        Ok([a,b])
    }
}
