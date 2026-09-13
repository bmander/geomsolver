//! The field as judge: strict signs, brackets and the projection of a point
//! onto the boundary along a direction, every query counted and timed.
use crate::interval::{Interval as I,minimum::{Options,Stop}};
use crate::solid::{MaterialEvaluator,MaterialField};
use std::time::{Duration,Instant};

type V3 = [f64;3];

/// Strict signs, a small/zero-containing field enclosure, or exhausted refinement.
/// `Near` bounds a field value only. It proves neither a root nor spatial proximity.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Sign { Material, Exterior, Near { value_bound: f64 }, Unresolved }

/// Opposed strict signs at actual spatial points. Only the judge creates witnesses.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct BoundaryBracket {
    inside: [f64;3],
    outside: [f64;3],
    inside_value: [f64;2],
    outside_value: [f64;2],
}
impl BoundaryBracket {
    pub fn inside(&self) -> (V3,[f64;2]) { (self.inside,self.inside_value) }
    pub fn outside(&self) -> (V3,[f64;2]) { (self.outside,self.outside_value) }
    /// Every boundary on the witness segment is within this distance of p.
    pub fn radius_from(&self,p: V3) -> Result<f64,JudgeError> {
        let distance = |q: V3| -> Result<f64,crate::interval::Error> {
            let mut sum = I::ZERO;
            for k in 0..3 { sum = sum.add(I::point(q[k])?.sub(I::point(p[k])?)?.square()?)?; }
            Ok(I::new(sum.bounds()[0].max(0.),sum.bounds()[1])?.sqrt()?.bounds()[1])
        };
        Ok(distance(self.inside).map_err(arithmetic)?.max(distance(self.outside).map_err(arithmetic)?))
    }
}
fn arithmetic(e: crate::interval::Error) -> JudgeError { JudgeError::Field(format!("{e:?}")) }

/// Where a point went when projected along its direction.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Projection {
    /// The boundary lies within `radius` of the point as given.
    Kept { radius: f64, bracket: BoundaryBracket },
    /// The boundary was found within `radius` of `point`, which is the given
    /// point moved along its direction by `by` (outward positive).
    Moved { point: V3, radius: f64, by: f64, bracket: BoundaryBracket },
    /// Strict material at the sampled search points; no global absence claim.
    Inner,
    /// Strict exterior at the sampled search points; no global absence claim.
    Positive,
    /// Search or refinement could not establish the requested spatial accuracy.
    /// `Some(bracket)` retains opposed witnesses; `None` has no spatial guarantee.
    /// `radius` is the retained bracket radius or the signed search offset.
    Unresolved { radius: f64, enclosure: [f64;2], bracket: Option<BoundaryBracket> },
}

#[derive(Clone,Debug,PartialEq)]
pub enum JudgeError {
    /// Material outward and exterior inward of the point: its direction is
    /// not the boundary's outward normal there.
    ReversedNormal { point: V3, direction: V3 },
    /// An enclosure straddling zero at the full budget, at `radius` along the
    /// direction from the point.
    Unresolved { point: V3, radius: f64, enclosure: [f64;2] },
    Field(String),
}

/// What a query asks of the field: the strict sign, resolved as finely as a near query may; the
/// strict sign of a point expected beyond a band; or whether a point is deeper than a band.
#[derive(Clone,Copy)]
enum Ask { Sign, Beyond(f64), Deep(f64) }

/// Counts and times of what the judge asked the field.
#[derive(Clone,Copy,Debug,Default)]
pub struct QueryStats {
    pub near: usize,
    pub near_time: Duration,
    pub near_slowest: Duration,
    pub far: usize,
    pub far_time: Duration,
    pub roll_evaluations: usize,
    /// Queries whose enclosure contained zero without exhausting refinement.
    pub near_hits: usize,
    pub unresolved: usize,
    /// Of the unresolved queries, how many had a sweep stop on its budget, and
    /// how many on the roll's resolution limit.
    pub exhausted: usize,
    pub resolution_limited: usize,
}

impl QueryStats {
    pub fn report(&self) -> String {
        let per = |n: usize,t: Duration| if n == 0 { 0. } else { t.as_secs_f64()*1e3/n as f64 };
        format!("{} near queries ({:.2} ms each, slowest {:.2} ms), {} far ({:.2} ms each), {} roll evaluations, {} near hits, {} unresolved ({} exhausted, {} at the resolution limit)",
            self.near,per(self.near,self.near_time),self.near_slowest.as_secs_f64()*1e3,self.far,per(self.far,self.far_time),
            self.roll_evaluations,self.near_hits,self.unresolved,self.exhausted,self.resolution_limited)
    }
}

pub struct FieldJudge {
    evaluator: MaterialEvaluator,
    near: Options,
    far: Options,
    pub stats: QueryStats,
    /// Every `sign_beyond` asked so far, by point and band: the trim asks a triangle's
    /// certificate probes first, and the certificate asks them again of every triangle the
    /// stitch left as it was. A query's answer depends on nothing but its point and options.
    beyond: std::collections::HashMap<([u64;3],u64),(Sign,[f64;2])>,
}

impl FieldJudge {
    /// `tolerance` is the value width a near query resolves to: half the
    /// vertex tolerance. This controls value refinement, not spatial accuracy.
    pub fn new(field: MaterialField,tolerance: f64,near_budget: usize,far_budget: usize,cached_poses: usize) -> Self {
        Self {
            evaluator: field.evaluator(cached_poses),
            near: Options {value_tolerance:tolerance,max_evaluations:near_budget.max(4)},
            far: Options {value_tolerance:tolerance,max_evaluations:far_budget.max(4)},
            stats: QueryStats::default(),
            beyond: Default::default(),
        }
    }

    fn query(&mut self,p: V3,ask: Ask) -> Result<(Sign,[f64;2]),JudgeError> {
        let started = Instant::now();
        let point = p.map(|x| I::point(x).map_err(|e| JudgeError::Field(format!("{e:?}")))).into_iter().collect::<Result<Vec<_>,_>>()?;
        let point: [I;3] = [point[0],point[1],point[2]];
        let band = |b: f64| I::new(-b,b).map_err(|e| JudgeError::Field(format!("{e:?}")));
        let (options,near,stop) = match ask {
            // Stop only on a strict sign or the stated value-refinement limit.
            Ask::Sign => (self.near,true,Stop::Outside(I::ZERO)),
            Ask::Beyond(b) => (self.far,false,Stop::Outside(band(b)?)),
            Ask::Deep(d) => (self.far,false,Stop::Decided(band(d)?)),
        };
        let bounds = self.evaluator.bounds_stopping(point,stop,options).map_err(|e| JudgeError::Field(format!("{e:?}")))?;
        let elapsed = started.elapsed();
        self.stats.roll_evaluations += bounds.sweeps.iter().map(|q| q.minimum.evaluations).sum::<usize>();
        if near {
            self.stats.near += 1; self.stats.near_time += elapsed;
            if elapsed > self.stats.near_slowest { self.stats.near_slowest = elapsed; }
        } else { self.stats.far += 1; self.stats.far_time += elapsed; }
        let [lo,hi] = bounds.value.bounds();
        let sign = if hi < 0. { Sign::Material } else if lo > 0. { Sign::Exterior } else {
            use crate::interval::minimum::Status;
            let exhausted = bounds.sweeps.iter().any(|q| q.minimum.status == Status::BudgetExhausted);
            let limited = bounds.sweeps.iter().any(|q| q.minimum.status == Status::ResolutionLimit);
            if exhausted || limited {
                self.stats.unresolved += 1;
                if exhausted { self.stats.exhausted += 1; }
                if limited { self.stats.resolution_limited += 1; }
                Sign::Unresolved
            } else { self.stats.near_hits += 1; Sign::Near {value_bound:lo.abs().max(hi.abs())} }
        };
        Ok((sign,[lo,hi]))
    }

    /// The strict sign at a point, resolved as finely as a near query may.
    pub fn sign(&mut self,p: V3) -> Result<(Sign,[f64;2]),JudgeError> { self.query(p,Ask::Sign) }

    /// The strict sign at a point expected at least `band` from the boundary:
    /// the search stops as soon as the enclosure clears the band.
    pub fn sign_beyond(&mut self,p: V3,band: f64) -> Result<(Sign,[f64;2]),JudgeError> {
        let key = (p.map(f64::to_bits),band.to_bits());
        if let Some(&answer) = self.beyond.get(&key) { return Ok(answer); }
        let answer = self.query(p,Ask::Beyond(band))?;
        self.beyond.insert(key,answer);
        Ok(answer)
    }

    /// The sign at a point deeper than `depth`: material only when the
    /// enclosure lies below `-depth`, exterior only above `depth`, and
    /// otherwise near, whatever the enclosure's width. The search stops as
    /// soon as that is decided, inside the band as well as outside it: a
    /// point on the boundary no longer refines to the full value tolerance.
    pub fn deep_sign(&mut self,p: V3,depth: f64) -> Result<(Sign,[f64;2]),JudgeError> {
        let (sign,[lo,hi]) = self.query(p,Ask::Deep(depth))?;
        Ok(match sign {
            Sign::Material if hi >= -depth => (Sign::Near {value_bound:lo.abs().max(hi.abs())},[lo,hi]),
            Sign::Exterior if lo <= depth => (Sign::Near {value_bound:lo.abs().max(hi.abs())},[lo,hi]),
            s => (s,[lo,hi]),
        })
    }

    /// Read a spatial box without interpreting a residual as a distance.
    pub(crate) fn enclose(&mut self,points: [I;3]) -> Result<I,JudgeError> {
        let started = Instant::now();
        let result = self.evaluator.bounds_stopping(points,Stop::Outside(I::ZERO),self.far)
            .map_err(|e| JudgeError::Field(format!("{e:?}")))?;
        self.stats.far += 1;
        self.stats.far_time += started.elapsed();
        self.stats.roll_evaluations += result.sweeps.iter().map(|q| q.minimum.evaluations).sum::<usize>();
        Ok(result.value)
    }

    /// Opposing point witnesses, without a distance or uniqueness assumption.
    pub fn bracket(&mut self,inside: V3,outside: V3) -> Result<Option<BoundaryBracket>,JudgeError> {
        let (a,av) = self.sign(inside)?;
        let (b,bv) = self.sign(outside)?;
        Ok((a == Sign::Material && b == Sign::Exterior).then_some(BoundaryBracket {
            inside,outside,inside_value:av,outside_value:bv,
        }))
    }

    /// Search along a normalized direction. Near/unknown values never supply a
    /// witness. Inner/Positive describe the sampled search, not global absence.
    pub fn project(&mut self,p: V3,m: V3,epsilon: f64,reach: f64) -> Result<Projection,JudgeError> {
        let length = crate::space::norm(m);
        if !epsilon.is_finite() || epsilon*0.5 <= 0. || !reach.is_finite() || reach < epsilon
            || !length.is_finite() || length <= 0. || p.iter().any(|x| !x.is_finite()) {
            return Err(JudgeError::Field("invalid projection options or direction".into()));
        }
        let m = m.map(|x| x/length);
        let along = |r: f64| -> V3 { std::array::from_fn(|k| p[k]+r*m[k]) };
        let first = |judge: &mut Self,sense: f64| -> Result<(Sign,f64,[f64;2]),JudgeError> {
            let mut r = epsilon*0.5;
            loop {
                let (s,value) = judge.sign(along(sense*r))?;
                if matches!(s,Sign::Material | Sign::Exterior) || r >= reach { return Ok((s,sense*r,value)); }
                r = (2.*r).min(reach);
            }
        };
        let (outer,r_out,v_out) = first(self,1.)?;
        let (inner,r_in,v_in) = first(self,-1.)?;
        let unresolved = |r,value| Projection::Unresolved {radius:r,enclosure:value,bracket:None};
        if !matches!(outer,Sign::Material | Sign::Exterior) { return Ok(unresolved(r_out,v_out)); }
        if !matches!(inner,Sign::Material | Sign::Exterior) { return Ok(unresolved(r_in,v_in)); }
        if inner == Sign::Exterior && outer == Sign::Material {
            return Err(JudgeError::ReversedNormal {point:p,direction:m});
        }
        let mut low = (r_in,v_in);
        let mut high = (r_out,v_out);
        if inner == outer {
            let sense = if outer == Sign::Material { 1. } else { -1. };
            let mut r = if sense > 0. { r_out } else { -r_in };
            let mut unknown = None;
            loop {
                if r >= reach {
                    return Ok(match unknown { Some((r,v)) => unresolved(r,v),None =>
                        if outer == Sign::Material { Projection::Inner } else { Projection::Positive } });
                }
                r = (2.*r).min(reach);
                let (s,value) = self.sign(along(sense*r))?;
                if s == outer {
                    if sense > 0. { low = (r,value); } else { high = (-r,value); }
                } else if matches!(s,Sign::Material | Sign::Exterior) {
                    if sense > 0. { high = (r,value); } else { low = (-r,value); }
                    break;
                } else { unknown = Some((sense*r,value)); }
            }
        }
        self.bisect(p,m,low,high,epsilon)
    }

    /// Keep the spatial witnesses throughout refinement. An ambiguous midpoint
    /// may be bypassed by quarter probes; it never becomes a residual-sized root.
    fn bisect(&mut self,p: V3,m: V3,mut low: (f64,[f64;2]),mut high: (f64,[f64;2]),epsilon: f64)
        -> Result<Projection,JudgeError> {
        let along = |r: f64| -> V3 { std::array::from_fn(|k| p[k]+r*m[k]) };
        loop {
            let bracket = BoundaryBracket {inside:along(low.0),outside:along(high.0),inside_value:low.1,outside_value:high.1};
            let radius = bracket.radius_from(p)?;
            if radius <= epsilon { return Ok(Projection::Kept {radius,bracket}); }
            let by = low.0*0.5+high.0*0.5;
            let point = along(by);
            let radius = bracket.radius_from(point)?;
            if radius <= epsilon { return Ok(Projection::Moved {point,radius,by,bracket}); }
            let before = (low.0,high.0);
            let mut ambiguous = [0.;2];
            for fraction in [0.5,0.25,0.75] {
                let r = before.0+(before.1-before.0)*fraction;
                if r <= low.0 || r >= high.0 { continue; }
                let (sign,value) = self.sign(along(r))?;
                match sign {
                    Sign::Material => low = (r,value),
                    Sign::Exterior => high = (r,value),
                    _ => ambiguous = value,
                }
            }
            if before == (low.0,high.0) {
                return Ok(Projection::Unresolved {radius,enclosure:ambiguous,bracket:Some(bracket)});
            }
        }
    }

    /// Both sides of a triangle: material at `c - d·n`, exterior at `c + d·n`,
    /// each asked only to clear a band half that distance wide.
    pub fn sides(&mut self,c: V3,n: V3,d: f64) -> Result<(Sign,Sign),JudgeError> {
        let inside: V3 = std::array::from_fn(|k| c[k]-d*n[k]);
        let outside: V3 = std::array::from_fn(|k| c[k]+d*n[k]);
        Ok((self.sign_beyond(inside,d/2.)?.0,self.sign_beyond(outside,d/2.)?.0))
    }

    /// The value width a near query resolves to: the width of the
    /// value enclosure; it is not a spatial certificate.
    pub fn near_tolerance(&self) -> f64 { self.near.value_tolerance }
}
