//! Shared bounded local intersection adapter over the existing DogLeg solver.
//! The payload is geometric data from the same residual evaluation, not a second
//! source evaluation. A zero residual still needs full rank in every free direction.
use crate::{envelope::{check_iterations,Error,IntersectionOptions},
    linalg::{min_norm_solve,Mat},newton::{self,TrustRegion}};
type V3 = [f64;3];

pub(crate) struct Root<T> {
    pub parameters: V3,
    pub value: T,
    pub residuals: V3,
    pub iterations: u32,
}

pub(crate) fn solve<T>(sample: impl Fn(V3) -> Result<(T,V3),Error>,
    seed: V3,options: IntersectionOptions) -> Result<Root<T>,Error> {
    check_iterations(options.max_iterations)?;
    if (0..3).any(|i| !options.parameter_scale[i].is_finite()
            || options.parameter_scale[i] <= 0. || !options.residual_tolerance[i].is_finite()
            || options.residual_tolerance[i] <= 0.
            || !options.bounds[i].iter().all(|v| v.is_finite()
                && (v/options.parameter_scale[i]).is_finite())
            || options.bounds[i][0] > options.bounds[i][1]) {
        return Err(Error::InvalidOptions);
    }
    if !seed.iter().all(|x| x.is_finite()) { return Err(Error::NonFinite); }
    if (0..3).any(|i| seed[i] < options.bounds[i][0] || seed[i] > options.bounds[i][1]) {
        return Err(Error::OutsideDomain);
    }
    let free: Vec<usize> = (0..3).filter(|&i| options.bounds[i][0] < options.bounds[i][1]).collect();
    let evaluate = |z: &[f64]| -> Result<(V3, T, V3), Error> {
        let mut p = seed;
        for (k,&i) in free.iter().enumerate() { p[i] = z[k]*options.parameter_scale[i]; }
        if !p.iter().all(|x| x.is_finite()) { return Err(Error::NonFinite); }
        if (0..3).any(|i| p[i] < options.bounds[i][0] || p[i] > options.bounds[i][1]) {
            return Err(Error::OutsideDomain);
        }
        let (c,r) = sample(p)?;
        if !r.iter().all(|x| x.is_finite()) { return Err(Error::NonFinite); }
        Ok((p, c, r))
    };
    let mut z: Vec<f64> = free.iter().map(|&i| seed[i]/options.parameter_scale[i]).collect();
    evaluate(&z)?;
    if free.is_empty() {
        let (parameters,contact,residuals) = evaluate(&z)?;
        if (0..3).any(|i| residuals[i].abs() > options.residual_tolerance[i]) {
            return Err(Error::NotConverged);
        }
        return Ok(Root { parameters,value:contact,residuals,iterations: 0 });
    }
    let residual = |z: &[f64]| -> V3 {
        match evaluate(z) {
            Ok((_, _, r)) => std::array::from_fn(|i| r[i]/options.residual_tolerance[i]),
            // DogLeg rejects non-finite trials and shrinks the region. Never clamp a
            // parameter to a trim boundary and pretend it satisfies the original equations.
            Err(_) => [f64::INFINITY; 3],
        }
    };
    let bounds = free.iter().map(|&i|
        options.bounds[i].map(|v| v/options.parameter_scale[i])).collect();
    let mut tr = IntersectionSystem { residual, bounds, j: Mat::zeros(3, free.len()) };
    let mut r = (tr.residual)(&z);
    let info = newton::dogleg(&mut tr, &mut z, &mut r,
        newton::Tol { ftol: 1., xtol: 1e-14, gtol: 1e-14 },
        options.max_iterations as i32, options.max_iterations as i32 * 4);
    let (parameters, contact, residuals) = evaluate(&z)?;
    if (0..3).any(|i| residuals[i].abs() > options.residual_tolerance[i]) {
        return Err(Error::NotConverged);
    }
    // Even a zero-residual seed must define a regular intersection. DogLeg's early
    // residual exit does not factor its Jacobian, so establish rank explicitly here.
    tr.jacobian_at(&z);
    if !tr.j.data.iter().all(|x| x.is_finite()) { return Err(Error::SingularIntersection); }
    let (_, rank) = min_norm_solve(&tr.j, &[0.;3], 1e-10);
    if rank != free.len() { return Err(Error::SingularIntersection); }
    Ok(Root { parameters,value:contact,residuals,iterations:info.iterations as u32 })
}

struct IntersectionSystem<F> {
    residual: F,
    bounds: Vec<[f64;2]>,
    j: Mat,
}

impl<F: Fn(&[f64]) -> V3> TrustRegion for IntersectionSystem<F> {
    fn n(&self) -> usize { self.bounds.len() }
    fn m(&self) -> usize { 3 }
    fn residuals_into(&mut self, z: &[f64], out: &mut [f64]) {
        out.copy_from_slice(&(self.residual)(z));
    }
    fn jacobian_at(&mut self, z: &[f64]) {
        let r = (self.residual)(z);
        for k in 0..self.n() {
            let h = (f64::EPSILON.cbrt() * z[k].abs().max(1.))
                .min((self.bounds[k][1]-self.bounds[k][0])*0.25);
            let mut lo = z.to_vec();
            let mut hi = z.to_vec();
            lo[k] -= h;
            hi[k] += h;
            let a = (self.residual)(&lo);
            let b = (self.residual)(&hi);
            let af = a.iter().all(|x| x.is_finite());
            let bf = b.iter().all(|x| x.is_finite());
            for i in 0..3 {
                let d = match (af, bf) {
                    (true, true) => (b[i]-a[i])/(2.*h),
                    (false, true) => (b[i]-r[i])/h,
                    (true, false) => (r[i]-a[i])/h,
                    _ => f64::NAN,
                };
                self.j.set(i, k, d);
            }
        }
    }
    fn jt_mul(&mut self, v: &[f64], out: &mut [f64]) {
        out.copy_from_slice(&self.j.mul_t_vec(v));
    }
    fn j_mul(&mut self, v: &[f64], out: &mut [f64]) {
        out.copy_from_slice(&self.j.mul_vec(v));
    }
    fn restrict_step(&self, z: &[f64], p: &mut [f64]) {
        // Project the trial step into the box without increasing its norm. Keep the
        // feasible components: one coordinate already on an edge must not freeze the
        // other parameters. Every original equation and the final rank are still checked.
        for i in 0..p.len() {
            p[i] = p[i].clamp(self.bounds[i][0]-z[i],self.bounds[i][1]-z[i]);
        }
    }

    fn gn_step(&mut self, r: &[f64], _g: &[f64], p: &mut [f64]) {
        if !self.j.data.iter().all(|x| x.is_finite()) { p.fill(0.); return; }
        let neg: Vec<f64> = r.iter().map(|v| -v).collect();
        let (step, _) = min_norm_solve(&self.j, &neg, 1e-12);
        p.copy_from_slice(&step);
    }
}
