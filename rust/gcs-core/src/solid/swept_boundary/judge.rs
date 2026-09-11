//! The field as judge: strict signs, brackets and the projection of a point
//! onto the boundary along a direction, every query counted and timed.
use crate::interval::{Interval as I,minimum::{Options,Stop}};
use crate::solid::{MaterialEvaluator,MaterialField};
use std::time::{Duration,Instant};

type V3 = [f64;3];

/// What the field says at a point: strictly material, strictly exterior,
/// near (a converged enclosure containing zero: the field is one-Lipschitz,
/// so the boundary lies within the enclosure's larger magnitude of the point,
/// which is a certificate and not a sign), or unresolved (an enclosure
/// containing zero at the budget or the roll's resolution limit, which is
/// neither).
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Sign { Material, Exterior, Near { within: f64 }, Unresolved }

/// Where a point went when projected along its direction.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Projection {
    /// The boundary lies within `radius` of the point as given.
    Kept { radius: f64 },
    /// The boundary was found within `radius` of `point`, which is the given
    /// point moved along its direction by `by` (outward positive).
    Moved { point: V3, radius: f64, by: f64 },
    /// Material on both sides as far as the search went: the point is on a
    /// branch of the envelope the sweep covers at another time.
    Inner,
    /// Exterior on both sides as far as the search went: the point is off
    /// the swept material.
    Positive,
    /// An enclosure straddling zero at the full budget, `radius` along the
    /// direction from the point: no sign, so no bracket, and never rounded.
    Unresolved { radius: f64, enclosure: [f64;2] },
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
    /// Queries whose converged enclosure contained zero: the boundary within
    /// the enclosure's width of the point.
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
    /// vertex tolerance, so a point that far from the boundary reads strictly.
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
            // refined until the enclosure clears the tolerance or converges, so one within it
            // is read as near (below) whichever side of zero it fell
            Ask::Sign => (self.near,true,Stop::Outside(band(self.near.value_tolerance)?)),
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
        // A near query's enclosure lying wholly within its tolerance of zero says what one
        // containing zero says, the boundary within that width: read strictly, a point on an
        // exactly computed face (a plane, a grazing end face) took its sign from which side of
        // the face rounding put it
        let within_tolerance = near && lo >= -options.value_tolerance && hi <= options.value_tolerance;
        let sign = if within_tolerance && (hi < 0. || lo > 0.) { self.stats.near_hits += 1; Sign::Near {within:lo.abs().max(hi.abs())} }
            else if hi < 0. { Sign::Material } else if lo > 0. { Sign::Exterior } else {
            use crate::interval::minimum::Status;
            let exhausted = bounds.sweeps.iter().any(|q| q.minimum.status == Status::BudgetExhausted);
            let limited = bounds.sweeps.iter().any(|q| q.minimum.status == Status::ResolutionLimit);
            if exhausted || limited {
                self.stats.unresolved += 1;
                if exhausted { self.stats.exhausted += 1; }
                if limited { self.stats.resolution_limited += 1; }
                Sign::Unresolved
            } else { self.stats.near_hits += 1; Sign::Near {within:lo.abs().max(hi.abs())} }
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
            Sign::Material if hi >= -depth => (Sign::Near {within:lo.abs().max(hi.abs())},[lo,hi]),
            Sign::Exterior if lo <= depth => (Sign::Near {within:lo.abs().max(hi.abs())},[lo,hi]),
            s => (s,[lo,hi]),
        })
    }

    /// Project a point onto the boundary along `m` (the outward direction):
    /// kept where the field brackets the boundary within `epsilon` of it,
    /// moved where the bracket is found within `reach` on one side, `Inner`
    /// or `Positive` where the same sign holds as far as `reach`.
    pub fn project(&mut self,p: V3,m: V3,epsilon: f64,reach: f64) -> Result<Projection,JudgeError> {
        let along = |r: f64| -> V3 { std::array::from_fn(|k| p[k]+r*m[k]) };
        // the first strict sign found at r, 2r, 4r ... up to reach (signed r)
        // a near hit at offset r: the boundary within `within` of that point
        let near = |r: f64,within: f64| -> Projection {
            if r.abs() <= epsilon { Projection::Kept {radius:epsilon+within} } else { Projection::Moved {point:along(r),radius:within,by:r} }
        };
        // the first strict sign found at r, 2r, 4r ... up to reach (signed r)
        let first = |judge: &mut Self,sign_of: f64| -> Result<(Sign,f64,[f64;2]),JudgeError> {
            let mut r = epsilon;
            loop {
                let (s,enclosure) = judge.sign(along(sign_of*r))?;
                if s != Sign::Unresolved { return Ok((s,sign_of*r,enclosure)); }
                if r >= reach { return Ok((Sign::Unresolved,sign_of*r,enclosure)); }
                r = (2.*r).min(reach);
            }
        };
        let (outer,r_out,enclosure_out) = first(self,1.)?;
        match outer {
            Sign::Unresolved => return Ok(Projection::Unresolved {radius:r_out,enclosure:enclosure_out}),
            Sign::Near {within} => return Ok(near(r_out,within)),
            _ => {}
        }
        let (inner,r_in,enclosure_in) = first(self,-1.)?;
        match inner {
            Sign::Unresolved => return Ok(Projection::Unresolved {radius:r_in,enclosure:enclosure_in}),
            Sign::Near {within} => return Ok(near(r_in,within)),
            _ => {}
        }
        match (inner,outer) {
            (Sign::Material,Sign::Exterior) => {
                if r_out <= epsilon && -r_in <= epsilon { return Ok(Projection::Kept {radius:epsilon}); }
                self.bisect(p,m,r_in,r_out,epsilon)
            }
            (Sign::Exterior,Sign::Material) => Err(JudgeError::ReversedNormal {point:p,direction:m}),
            (Sign::Material,Sign::Material) => {
                // material outward: widen outward until exterior or reach
                let mut material = r_out;
                loop {
                    if material >= reach { return Ok(Projection::Inner); }
                    let r = (2.*material).min(reach);
                    let (s,enclosure) = self.sign(along(r))?;
                    match s {
                        Sign::Exterior => return self.bisect(p,m,material,r,epsilon),
                        Sign::Material => material = r,
                        Sign::Near {within} => return Ok(near(r,within)),
                        Sign::Unresolved => return Ok(Projection::Unresolved {radius:r,enclosure}),
                    }
                }
            }
            (Sign::Exterior,Sign::Exterior) => {
                // exterior inward: widen inward until material or reach
                let mut exterior = -r_in;
                loop {
                    if exterior >= reach { return Ok(Projection::Positive); }
                    let r = (2.*exterior).min(reach);
                    let (s,enclosure) = self.sign(along(-r))?;
                    match s {
                        Sign::Material => return self.bisect(p,m,-r,-exterior,epsilon),
                        Sign::Exterior => exterior = r,
                        Sign::Near {within} => return Ok(near(-r,within)),
                        Sign::Unresolved => return Ok(Projection::Unresolved {radius:-r,enclosure}),
                    }
                }
            }
            _ => unreachable!("near and unresolved first signs return above"),
        }
    }

    /// Bisect between a material offset `r_in` and an exterior offset `r_out`
    /// along `m` until the bracket is `2·epsilon` wide; the midpoint is the
    /// moved point, within `epsilon` of the boundary.
    fn bisect(&mut self,p: V3,m: V3,mut r_in: f64,mut r_out: f64,epsilon: f64) -> Result<Projection,JudgeError> {
        let along = |r: f64| -> V3 { std::array::from_fn(|k| p[k]+r*m[k]) };
        while r_out-r_in > 2.*epsilon {
            let mid = 0.5*(r_in+r_out);
            let (s,enclosure) = self.sign(along(mid))?;
            match s {
                Sign::Material => r_in = mid,
                Sign::Exterior => r_out = mid,
                Sign::Near {within} => return Ok(Projection::Moved {point:along(mid),radius:within,by:mid}),
                Sign::Unresolved => return Ok(Projection::Unresolved {radius:mid,enclosure}),
            }
        }
        let by = 0.5*(r_in+r_out);
        Ok(Projection::Moved {point:along(by),radius:0.5*(r_out-r_in),by})
    }

    /// Both sides of a triangle: material at `c - d·n`, exterior at `c + d·n`,
    /// each asked only to clear a band half that distance wide.
    pub fn sides(&mut self,c: V3,n: V3,d: f64) -> Result<(Sign,Sign),JudgeError> {
        let inside: V3 = std::array::from_fn(|k| c[k]-d*n[k]);
        let outside: V3 = std::array::from_fn(|k| c[k]+d*n[k]);
        Ok((self.sign_beyond(inside,d/2.)?.0,self.sign_beyond(outside,d/2.)?.0))
    }

    /// The value width a near query resolves to: the width of the
    /// certificate a Near sign carries.
    pub fn near_tolerance(&self) -> f64 { self.near.value_tolerance }
}
