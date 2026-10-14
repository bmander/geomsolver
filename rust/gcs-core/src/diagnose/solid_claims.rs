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

/// How the claim in hand is judged, pose by pose: what it read once, before its first pose.
enum Poses {
    /// The drawing's own pose, once — a claim over no sweep.
    Current,
    /// Along a motion's roll with one operand the only placement it moves: both operands evaluated
    /// once and that placement moved rigidly at each roll (`clear::evaluate_moved`).
    Rigid {
        family: crate::motion::Family,
        home: crate::envelope::Motion,
        at: f64,
        a_moves: bool,
        a: std::rc::Rc<crate::solid::EvaluatedSolid>,
        b: std::rc::Rc<crate::solid::EvaluatedSolid>,
    },
    /// Along a motion's roll otherwise: a copy of the drawing re-placed per roll, every solid
    /// placed under the motion at its `at:` advanced by the roll and nothing solved again, keeping
    /// its evaluated solids against what they read.
    Replaced { scratch: Sketch, placed: Vec<(usize, f64)> },
    /// Over a free parameter: a copy solved again at each sample.
    Solved,
    /// Every pose fails, for this reason: what the roll reads could not be read.
    Failing(String),
}

/// What the claim in hand reads once, before its first pose.
fn poses_of(sk: &Sketch, c: &SolidClaim) -> Poses {
    use crate::model::SolidDef;
    let Some(sw) = &c.over else { return Poses::Current };
    let Some(m) = sw.motion() else { return Poses::Solved };
    let placed: Vec<(usize, f64)> = sk.solids.iter().enumerate().filter_map(|(i, s)| match &s.def {
        SolidDef::Placed { motion, at, .. } if *motion == m => Some((i, at.value)),
        _ => None,
    }).collect();
    // how many placements under the motion solid `i` reads, itself included
    let moving = |i: usize| {
        let (mut pending, mut seen, mut count) = (vec![i], std::collections::BTreeSet::new(), 0);
        while let Some(j) = pending.pop() {
            if !seen.insert(j) { continue; }
            count += usize::from(placed.iter().any(|&(p, _)| p == j));
            pending.extend(sk.solids[j].operands().into_iter().map(|o| o as usize));
        }
        count
    };
    let (a, b) = (c.a as usize, c.b as usize);
    let rigid = match (moving(a), moving(b)) {
        (1, 0) => placed.iter().find(|&&(p, _)| p == a).map(|&(_, at)| (true, at)),
        (0, 1) => placed.iter().find(|&&(p, _)| p == b).map(|&(_, at)| (false, at)),
        _ => None,
    };
    let Some((a_moves, at)) = rigid else { return Poses::Replaced { scratch: sk.clone(), placed } };
    let policy = crate::solid::ApproximationPolicy::from_unit(crate::solid::REPORT_UNIT);
    let read = || -> Result<Poses, String> {
        let family = crate::motion::Family::read(sk, m as usize)?;
        let home = family.at(at)?.inverse();
        let (a, b) = (sk.evaluated_solid(a, policy)?, sk.evaluated_solid(b, policy)?);
        Ok(Poses::Rigid { family, home, at, a_moves, a, b })
    };
    read().unwrap_or_else(Poses::Failing)
}

/// The pose of claim `c` at sample `t` of its sweep, solved again on a copy of the drawing.
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
        scratch.caches.solid.borrow_mut().clear();
        let solved = crate::solve::solve(&mut scratch, Default::default());
        if !solved.success {
            return Err(format!("solve failed: {}", solved.message));
        }
        geometry(&scratch, c)
    };
    SolidPose { parameter: Some(t), evaluation: evaluate() }
}

/// How many poses claim `c` is judged at: the drawing's own, or each sample of its sweep.
fn pose_count(c: &SolidClaim) -> usize {
    if c.over.is_some() { SWEEP_STEPS + 1 } else { 1 }
}

/// **The drawing's claims about solids, judged a pose at a time** (`step`), so a host can say
/// where the judging stands between poses and drop it for a newer drawing: a claim over a sweep
/// is 37 poses, each a solid evaluated, and a drawing's claims take seconds.  `judge_solids` is
/// this run to the end, so a claim is judged one way whoever asks.
///
/// The judge holds no drawing: each step is handed the one it was made for.
pub struct ClaimJudge {
    /// Whether the drawing's own pose satisfies its constraints — what a claim over no sweep reads.
    current_pose: Result<(), String>,
    /// The claim in hand, by its index in `Sketch::solid_claims`.
    claim: usize,
    /// What the claim in hand read before its first pose.
    reading: Option<Poses>,
    /// Its poses judged so far.
    poses: Vec<SolidPose>,
    verdicts: Vec<SolidVerdict>,
}

impl ClaimJudge {
    pub fn new(sk: &Sketch) -> ClaimJudge {
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
        ClaimJudge {
            current_pose, claim: 0, reading: None, poses: Vec::new(), verdicts: Vec::new(),
        }
    }

    /// Whether every claim of `sk` is judged.
    pub fn done(&self, sk: &Sketch) -> bool {
        self.claim >= sk.solid_claims.len()
    }

    /// Judge the next pose of `sk`, the drawing the judge was made for: whether every claim is now
    /// judged.
    pub fn step(&mut self, sk: &Sketch) -> bool {
        let Some(c) = sk.solid_claims.get(self.claim) else { return true };
        let k = self.poses.len();
        let reading = self.reading.get_or_insert_with(|| poses_of(sk, c));
        let sample = |k: usize| c.over.as_ref()
            .map(|sw| sw.sample(k, SWEEP_STEPS).expect("inclusive nonzero sweep intervals"));
        let pose = match reading {
            Poses::Current => SolidPose {
                parameter: None,
                evaluation: self.current_pose.clone().and_then(|()| geometry(sk, c)),
            },
            Poses::Rigid { family, home, at, a_moves, a, b } => {
                let t = sample(k).expect("a roll is a sweep");
                let evaluation = family.at(*at + t.to_radians()).map(|now| home.then(now))
                    .and_then(|pose| {
                        let pose = if *a_moves { pose.inverse() } else { pose };
                        clear::evaluate_moved(&c.requirement, a, b, pose)
                    });
                SolidPose { parameter: Some(t), evaluation }
            }
            Poses::Replaced { scratch, placed } => {
                let t = sample(k).expect("a roll is a sweep");
                for &(i, at) in placed.iter() {
                    let def = &mut scratch.solids[i].def;
                    if let crate::model::SolidDef::Placed { at: a, .. } = def {
                        a.value = at + t.to_radians();
                    }
                }
                SolidPose { parameter: Some(t), evaluation: geometry(scratch, c) }
            }
            Poses::Solved => {
                let t = sample(k).expect("a free parameter's claim is a sweep");
                sampled_pose(sk, c, c.over.as_ref().expect("a sweep"), t)
            }
            Poses::Failing(e) => SolidPose { parameter: sample(k), evaluation: Err(e.clone()) },
        };
        self.poses.push(pose);
        if self.poses.len() >= pose_count(c) {
            self.verdicts.push(SolidVerdict {
                stmt: c.stmt,
                text: crate::io::describe_solid_claim(sk, c),
                sweep: c.over.clone(),
                poses: std::mem::take(&mut self.poses),
            });
            self.reading = None;
            self.claim += 1;
        }
        self.done(sk)
    }

    /// Judge the rest of the claim in hand: whether every claim is now judged.
    fn finish_claim(&mut self, sk: &Sketch) -> bool {
        let claim = self.claim;
        while self.claim == claim {
            if self.step(sk) { return true; }
        }
        false
    }

    /// Where the judging stands: the claim in hand (counted from 0) of how many, and its poses
    /// judged of how many — `claim` equal to `claims` once every one is.
    pub fn progress(&self, sk: &Sketch) -> ClaimProgress {
        let c = sk.solid_claims.get(self.claim);
        ClaimProgress {
            claim: self.claim,
            claims: sk.solid_claims.len(),
            pose: self.poses.len(),
            poses: c.map_or(0, pose_count),
            text: c.map(|c| crate::io::describe_solid_claim(sk, c)),
            stmt: c.map(|c| c.stmt),
        }
    }

    /// The verdicts so far, in the drawing's order.
    pub fn verdicts(&self) -> &[SolidVerdict] {
        &self.verdicts
    }

    pub fn into_verdicts(self) -> Vec<SolidVerdict> {
        self.verdicts
    }
}

/// Where a `ClaimJudge` stands.
#[derive(Clone, Debug, PartialEq)]
pub struct ClaimProgress {
    /// The claim in hand, counted from 0, and how many there are.
    pub claim: usize,
    pub claims: usize,
    /// Its poses judged, and how many it is judged at.
    pub pose: usize,
    pub poses: usize,
    /// The claim in hand as the report words it (`claim a clear(1mm) b`), and its statement.
    pub text: Option<String>,
    pub stmt: Option<u32>,
}

impl ClaimProgress {
    /// The fraction of the judging done, every claim weighed alike.
    pub fn fraction(&self) -> f64 {
        if self.claims == 0 { return 1.0; }
        let within = if self.poses == 0 { 0.0 } else { self.pose as f64 / self.poses as f64 };
        ((self.claim as f64 + within) / self.claims as f64).min(1.0)
    }

    /// What is being done, as a host says it: `judging claim a clear(1mm) b (2 of 3), pose 5 of
    /// 37`, and `judged 3 claims` once all are.
    pub fn doing(&self) -> String {
        let Some(text) = &self.text else {
            let s = if self.claims == 1 { "" } else { "s" };
            return format!("judged {} claim{s}", self.claims);
        };
        let mut said = format!("judging {text}");
        if self.claims > 1 { said += &format!(" ({} of {})", self.claim + 1, self.claims); }
        if self.poses > 1 { said += &format!(", pose {} of {}", self.pose + 1, self.poses); }
        said
    }
}

/// Every claim of the drawing judged, each said to whoever listens (`progress::during`).
pub fn judge_solids(sk: &Sketch) -> Vec<SolidVerdict> {
    let mut judge = ClaimJudge::new(sk);
    while !judge.done(sk) {
        let c = &sk.solid_claims[judge.claim];
        let judged =
            || crate::progress::Work::claim(crate::io::describe_solid_claim(sk, c), c.stmt);
        crate::progress::during(judged, || judge.finish_claim(sk));
    }
    judge.into_verdicts()
}
