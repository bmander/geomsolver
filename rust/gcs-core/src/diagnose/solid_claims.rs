//! Sample coverage and geometric evidence are retained; outcomes are always derived.
use crate::{
    clear,
    model::{Sketch, SolidClaim, Sweep},
};

/// Number of uniform intervals; both endpoints are attempted.
pub const SWEEP_STEPS: usize = 36;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SolidOutcome {
    Holds,
    SampledSuccess,
    Refuted,
    Indeterminate,
}
impl SolidOutcome {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Holds => "holds",
            Self::SampledSuccess => "sampled-success",
            Self::Refuted => "refuted",
            Self::Indeterminate => "undecided",
        }
    }
}

/// A solved pose with valid geometric evidence, or a disclosed solve/evaluation failure.
#[derive(Clone, Debug)]
pub struct SolidPose {
    parameter: Option<f64>,
    evaluation: Result<clear::GeometricEvidence, String>,
}
impl SolidPose {
    pub fn parameter(&self) -> Option<f64> {
        self.parameter
    }
    pub fn evaluation(&self) -> Result<&clear::GeometricEvidence, &str> {
        self.evaluation.as_ref().map_err(String::as_str)
    }
    pub fn holds(&self) -> Option<bool> {
        self.evaluation.as_ref().ok()?.holds()
    }
}

/// Immutable result: attempted coverage, failures and the witness cannot disagree with the verdict.
///
/// ```compile_fail
/// fn erase_failures(v: &mut gcs_core::diagnose::SolidVerdict) {
///     v.poses.clear();
/// }
/// ```
#[derive(Clone, Debug)]
pub struct SolidVerdict {
    stmt: u32,
    text: String,
    sweep: Option<Sweep>,
    poses: Vec<SolidPose>,
}
impl SolidVerdict {
    pub fn stmt(&self) -> u32 {
        self.stmt
    }
    pub fn text(&self) -> &str {
        &self.text
    }
    pub fn sweep(&self) -> Option<&Sweep> {
        self.sweep.as_ref()
    }
    pub fn poses(&self) -> &[SolidPose] {
        &self.poses
    }
    pub fn samples(&self) -> usize {
        if self.sweep.is_some() {
            self.poses.len()
        } else {
            0
        }
    }
    pub fn valid_samples(&self) -> impl Iterator<Item = &SolidPose> {
        self.poses.iter().filter(|p| p.evaluation.is_ok())
    }
    pub fn failures(&self) -> impl Iterator<Item = &SolidPose> {
        self.poses.iter().filter(|p| p.evaluation.is_err())
    }
    pub fn failed_samples(&self) -> Vec<f64> {
        self.failures().filter_map(SolidPose::parameter).collect()
    }
    pub fn outcome(&self) -> SolidOutcome {
        if self.counterexample().is_some() {
            return SolidOutcome::Refuted;
        }
        if self.poses.is_empty() || self.poses.iter().any(|p| p.holds() != Some(true)) {
            return SolidOutcome::Indeterminate;
        }
        if self.sweep.is_some() {
            SolidOutcome::SampledSuccess
        } else {
            SolidOutcome::Holds
        }
    }
    pub fn holds(&self) -> Option<bool> {
        match self.outcome() {
            SolidOutcome::Holds | SolidOutcome::SampledSuccess => Some(true),
            SolidOutcome::Refuted => Some(false),
            SolidOutcome::Indeterminate => None,
        }
    }
    pub fn counterexample(&self) -> Option<&SolidPose> {
        self.poses.iter().find(|p| p.holds() == Some(false))
    }
    /// Refutations always select an actual counterexample, never an unrelated uncertain pose.
    pub fn representative(&self) -> Option<&SolidPose> {
        self.counterexample()
            .or_else(|| {
                self.valid_samples()
                    .filter_map(|pose| Some((pose, pose.evaluation().ok()?.measured()?)))
                    .min_by(|(_, a), (_, b)| a.total_cmp(b))
                    .map(|(pose, _)| pose)
            })
            .or_else(|| self.valid_samples().next())
    }
    pub fn worst(&self) -> Option<f64> {
        let p = self.representative()?;
        if p.holds() != Some(false) && p.evaluation.as_ref().ok()?.measured().is_none() {
            return None;
        }
        p.parameter
    }
    pub fn measurement(&self) -> Option<&clear::Measurement> {
        self.representative()?.evaluation.as_ref().ok().map(clear::GeometricEvidence::measurement)
    }
    pub fn measured(&self) -> Option<f64> {
        self.measurement()?.interval().map(clear::Interval::midpoint)
    }
    pub fn tolerance(&self) -> Option<f64> {
        self.measurement()?.interval().map(clear::Interval::uncertainty)
    }
}

fn geometry(sk: &Sketch, c: &SolidClaim) -> Result<clear::GeometricEvidence, String> {
    clear::evaluate(sk, &c.requirement, c.a as usize, c.b as usize, crate::solid::REPORT_UNIT)
        .into_evidence()
}

fn sampled_pose(sk: &Sketch, c: &SolidClaim, sw: &Sweep, t: f64) -> SolidPose {
    let evaluate = || -> Result<clear::GeometricEvidence, String> {
        let mut scratch = sk.clone();
        if scratch.free_dimensions.get(&sw.name) != Some(&sw.dimension) {
            return Err("sweep parameter dimension changed since validation".into());
        }
        let p = *scratch.free_vars.get(&sw.name).ok_or("sweep parameter no longer exists")?;
        let p = scratch.params.get_mut(p as usize).ok_or("invalid sweep parameter reference")?;
        p.value = t;
        p.fixed = true;
        scratch.solid_cache.borrow_mut().clear();
        let solved = crate::solve::solve(&mut scratch, Default::default());
        if !solved.success {
            return Err(format!("solve failed: {}", solved.message));
        }
        geometry(&scratch, c)
    };
    SolidPose { parameter: Some(t), evaluation: evaluate() }
}

pub fn judge_solids(sk: &Sketch) -> Vec<SolidVerdict> {
    // A single-pose claim must describe the current drawing, not a newly solved copy.
    // Check once for all such claims; sweeps solve and validate their own attempted poses.
    let current_pose = if sk.solid_claims.iter().any(|c| c.over.is_none()) {
        let mut sys = crate::system::System::new(sk);
        let z = sys.z0(sk);
        let residual = sys.max_relative_residual(&z);
        if residual.is_finite() && residual < super::DiagnoseOptions::default().tol {
            Ok(())
        } else {
            Err("current pose does not satisfy the drawing's constraints".to_string())
        }
    } else {
        Ok(())
    };
    sk.solid_claims
        .iter()
        .map(|c| {
            let poses = match &c.over {
                None => vec![SolidPose {
                    parameter: None,
                    evaluation: current_pose.clone().and_then(|()| geometry(sk, c)),
                }],
                Some(sw) => (0..=SWEEP_STEPS)
                    .map(|k| {
                        let t =
                            sw.sample(k, SWEEP_STEPS).expect("inclusive nonzero sweep intervals");
                        sampled_pose(sk, c, sw, t)
                    })
                    .collect(),
            };
            SolidVerdict {
                stmt: c.stmt,
                text: crate::io::describe_solid_claim(sk, c),
                sweep: c.over.clone(),
                poses,
            }
        })
        .collect()
}
