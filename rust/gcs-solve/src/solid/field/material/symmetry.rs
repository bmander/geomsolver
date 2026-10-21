//! The rigid maps a material field reads the same under: an indexed cut's placements, checked
//! against the whole field at sampled points.
use super::MaterialField;
use super::plan::{Kind,OpId,Plan};
use crate::motion::MotionBounds;

/// Points spread over the support at which a symmetry is checked, beside the caller's.
const SYMMETRY_SAMPLES: usize = 6;

/// A rigid map the field reads the same under (`MaterialField::symmetries`): one placement's
/// inverse, then another's.
#[derive(Clone,Copy,Debug)]
pub struct Symmetry { from: MotionBounds,to: MotionBounds }

impl Symmetry {
    pub fn apply(&self,p: [f64;3]) -> [f64;3] { self.to.point_mid(self.from.inverse_point_mid(p)) }
}

impl MaterialField {
    /// Rigid maps the field reads the same under, from the first union of placed copies of one
    /// operand it holds (an indexed cut, `Spread`): the map carrying the first copy's placement to
    /// each other's. Kept only if the whole field reads alike at `samples` (a surface's vertices,
    /// say), at points spread over its support, and at all their images, since the rest of the
    /// body (a gear's blank) must be alike under it too: sampled, a reading and never a claim,
    /// and empty where anything disagrees.
    pub(crate) fn symmetries(&self,samples: &[[f64;3]]) -> Vec<Symmetry> {
        // the largest set of one operand's placements among a union's operands (the rest, if any,
        // must be alike under the maps too, which the samples check)
        // (one operand placed twice is one op, its source, under two poses)
        fn copies(plan: &Plan,i: OpId) -> Option<Vec<MotionBounds>> {
            match &plan.op(i).kind {
                Kind::Union {flat,..} => {
                    let mut groups: Vec<(OpId,Vec<MotionBounds>)> = Vec::new();
                    for &(op,_) in flat {
                        let Kind::Transformed {source,pose} = &plan.op(op).kind else { continue };
                        match groups.iter_mut().find(|g| g.0 == *source) {
                            Some(g) => g.1.push(*pose),
                            None => groups.push((*source,vec![*pose])),
                        }
                    }
                    groups.into_iter().map(|g| g.1).max_by_key(Vec::len).filter(|g| g.len() > 1)
                }
                Kind::Difference(a,b) | Kind::Intersection(a,b) => copies(plan,*b).or_else(|| copies(plan,*a)),
                Kind::Transformed {..} | Kind::Static(_) | Kind::Swept(_) => None,
            }
        }
        let plan = self.plan();
        let Some(poses) = copies(plan,plan.root()) else { return Vec::new() };
        let maps: Vec<Symmetry> = poses[1..].iter().map(|&to| Symmetry {from:poses[0],to}).collect();
        let Some(support) = self.tight_support(4,512).ok().flatten() else { return Vec::new() };
        let [lo,hi] = [0,1].map(|k| support.map(|x| x.bounds()[k]));
        let size = crate::space::box_centre_diagonal(&support).1;
        let mut rng = crate::rng::Rng::new(0x5e11);
        let samples: Vec<[f64;3]> = (0..SYMMETRY_SAMPLES).map(|_| std::array::from_fn(|k| rng.uniform(lo[k],hi[k])))
            .chain(samples.iter().copied()).collect();
        let exact = |p: [f64;3]| self.query(p,&mut super::super::Query {relative:0.,..super::super::Query::at(p)}).value;
        // the first map at every sample, the rest at the points spread over the support and a few
        // of the caller's: a map carrying one tooth space onto the next already says most of it
        let alike = |(k,m): (usize,&Symmetry)| samples.iter().take(if k == 0 { usize::MAX } else { 2*SYMMETRY_SAMPLES }).all(|&p| {
            let q = m.apply(p);
            let (a,b) = (exact(p),exact(q));
            (a-b).abs() <= 1e-9*size
        });
        if maps.iter().enumerate().all(alike) { maps } else { Vec::new() }
    }
}
