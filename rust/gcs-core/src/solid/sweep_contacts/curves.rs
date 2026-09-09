//! Join proven fixed-time chart intervals without interpolating across gaps.
use super::{SweepContacts,ContactCover,ContactEvidence,ContactParameter};
use crate::envelope::Contact;
use std::collections::BTreeMap;

#[derive(Clone,Debug)]
pub struct ContactCurve {
    pub patch: usize,
    pub dependent: ContactParameter,
    /// Algebraic root label in this source parameterization only.
    pub branch: usize,
    pub time: f64,
    /// Original free source coordinate, not an arc-length parameter.
    pub range: [f64;2],
    /// Every contributing chart in ContactCurves::cover, including overlaps.
    pub cells: Vec<usize>,
}

#[derive(Clone,Debug)]
pub struct ContactCurves {
    /// Keep the complete partition: unresolved regions have not disappeared
    /// just because its regular chart pieces could be joined.
    pub cover: ContactCover,
    pub curves: Vec<ContactCurve>,
}

#[derive(Clone,Copy,Debug)]
pub struct ContactCurvePoint {
    /// Canonical source (u,v); a periodic v may wrap while position is continuous.
    pub parameters: [f64;2],
    pub contact: Contact,
}

impl SweepContacts {
    /// Merge touching/overlapping free intervals of the same patch, source chart
    /// direction and analytic root. No proximity weld or gap tolerance is used.
    /// Chart-direction changes, source-patch joins and periodic range joins remain
    /// separate curves. This is neither source trim classification nor a complete
    /// contact contour, and the retained cover must still be inspected.
    pub fn join_contact_curves(&self,cover: ContactCover,tolerance: f64) -> Result<ContactCurves,String> {
        if !tolerance.is_finite() || tolerance <= 0. { return Err("contact tolerance must be finite and positive".into()); }
        let mut time = None;
        let mut groups = BTreeMap::<(usize,usize,usize),Vec<(f64,f64,usize)>>::new();
        for (index,cell) in cover.cells.iter().enumerate() {
            let [t,end] = cell.parameters[2].bounds();
            if t != end || time.is_some_and(|time| time != t) {
                return Err("contact curves require one fixed motion parameter".into());
            }
            time = Some(t);
            let ContactEvidence::Chart(chart) = cell.evidence else { continue; };
            if chart.dependent == ContactParameter::Time {
                return Err("a fixed-time curve cannot solve for time".into());
            }
            let free = chart.dependent.free()[0];
            let [lo,hi] = cell.parameters[free].bounds();
            if lo >= hi { return Err("a contact curve needs a nonempty free interval".into()); }
            let mut branch = None;
            // Validate the chart with the existing root evaluator; labels at
            // both ends also guard against a parameter-domain event being hidden
            // by a midpoint-only branch lookup.
            for fraction in [0.,0.5,1.] {
                self.at_chart(cell,fraction,0.,tolerance)?;
                let s = if fraction == 0. { lo } else if fraction == 1. { hi } else { lo+(hi-lo)*fraction };
                let labels: Vec<_> = match chart.dependent {
                    ContactParameter::Angle => self.patches[cell.patch].angular_chart(chart.range.bounds())
                        .map_err(|e| format!("{e:?}"))?.contacts(s,self.motion.at(t)?,tolerance)
                        .map_err(|e| format!("{e:?}"))?.into_iter().map(|c| c.branch).collect(),
                    ContactParameter::Meridian => self.at_angle(cell.patch,s,t,tolerance)
                        .map_err(|e| format!("{e:?}"))?.into_iter().filter(|c| chart.range.contains(c.u))
                        .map(|c| c.branch).collect(),
                    ContactParameter::Time => unreachable!(),
                };
                if labels.len() != 1 || branch.is_some_and(|b| b != labels[0]) {
                    return Err("contact chart crosses an analytic branch event".into());
                }
                branch = Some(labels[0]);
            }
            groups.entry((cell.patch,chart.dependent.index(),branch.unwrap())).or_default().push((lo,hi,index));
        }
        let mut curves: Vec<ContactCurve> = Vec::new();
        for ((patch,axis,branch),mut intervals) in groups {
            intervals.sort_by(|a,b| a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1)).then(a.2.cmp(&b.2)));
            let dependent = if axis == 0 { ContactParameter::Meridian } else { ContactParameter::Angle };
            let mut current: Option<ContactCurve> = None;
            for (lo,hi,index) in intervals {
                if let Some(curve) = &mut current {
                    if lo <= curve.range[1] {
                        curve.range[1] = curve.range[1].max(hi);
                        curve.cells.push(index);
                        continue;
                    }
                }
                if let Some(curve) = current.take() { curves.push(curve); }
                current = Some(ContactCurve {patch,dependent,branch,time:time.unwrap(),range:[lo,hi],cells:vec![index]});
            }
            if let Some(curve) = current { curves.push(curve); }
        }
        Ok(ContactCurves {cover,curves})
    }

    /// Evaluate the original analytic branch over a joined interval from this
    /// same solved snapshot. This does not evaluate a fitted polyline or spline.
    pub fn at_contact_curve(&self,curve: &ContactCurve,s: f64,tolerance: f64) -> Result<ContactCurvePoint,String> {
        if !s.is_finite() || !(0. ..=1.).contains(&s) {
            return Err("curve coordinate must lie in [0,1]".into());
        }
        let surface = self.patches.get(curve.patch).ok_or("no such contact patch")?;
        let free = match curve.dependent {
            ContactParameter::Angle => 0,
            ContactParameter::Meridian => 1,
            ContactParameter::Time => return Err("a fixed-time curve cannot solve for time".into()),
        };
        let [lo,hi] = curve.range; let domain = surface.domain()[free];
        if ![lo,hi].iter().all(|v| v.is_finite()) || lo >= hi || lo < domain[0] || hi > domain[1] {
            return Err("contact curve leaves its source domain".into());
        }
        let p = if s == 0. { lo } else if s == 1. { hi } else { (lo+(hi-lo)*s).clamp(lo,hi) };
        match curve.dependent {
            ContactParameter::Angle => self.at(curve.patch,p,curve.time,tolerance).map_err(|e| format!("{e:?}"))?
                .into_iter().find(|c| c.branch == curve.branch)
                .map(|c| ContactCurvePoint {parameters:[p,c.v],contact:c.contact}),
            ContactParameter::Meridian => self.at_angle(curve.patch,p,curve.time,tolerance).map_err(|e| format!("{e:?}"))?
                .into_iter().find(|c| c.branch == curve.branch)
                .map(|c| ContactCurvePoint {parameters:[c.u,p],contact:c.contact}),
            ContactParameter::Time => unreachable!(),
        }.ok_or("joined contact branch is absent".into())
    }
}
