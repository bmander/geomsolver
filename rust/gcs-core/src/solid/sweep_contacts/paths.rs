//! Follow established chart transitions while retaining analytic source segments.
use super::{SweepContacts,ContactCurves};

#[derive(Clone,Copy,Debug)]
pub struct ContactPathSegment {
    pub curve: usize,
    /// Normalized coordinates within this joined curve; decreasing means reversed.
    pub range: [f64;2],
}

#[derive(Clone,Debug)]
pub struct ContactPath {
    pub segments: Vec<ContactPathSegment>,
    /// Closed by a repeated oriented interval. Periodic seam endpoints alone
    /// are not yet connected by this tracer.
    pub closed: bool,
    /// Measured mismatch between analytic evaluators at chart handoffs. This
    /// accounts for numeric reparameterization, not spline/export accuracy.
    /// Measured in the original source length units.
    pub join_error: f64,
}

impl SweepContacts {
    /// Follow both ends of a seed interval to obtain its maximal covered path.
    /// The seed is included once. A closed traversal already covers its loop;
    /// otherwise reverse the backward traversal and append the forward one.
    /// This still stops at unresolved links and does not identify periodic seams.
    pub fn trace_contact_component(&self,joined: &ContactCurves,start: usize,
        tolerance: f64,distance: f64) -> Result<ContactPath,String> {
        let right = self.trace_contact_path(joined,start,true,tolerance,distance)?;
        if right.closed { return Ok(right); }
        let mut left = self.trace_contact_path(joined,start,false,tolerance,distance)?;
        if left.closed { return Ok(left); }
        left.segments.reverse();
        for segment in &mut left.segments { segment.range.reverse(); }
        left.segments.pop(); // The forward path includes this same seed interval.
        left.segments.extend(right.segments);
        left.join_error = left.join_error.max(right.join_error);
        Ok(left)
    }

    /// Trace one connected candidate from a joined curve's low/high end. A
    /// missing transition stops the path; it never extrapolates through a gap.
    /// A repeated oriented source interval closes a loop and removes any prefix
    /// already covered by the closing overlap. Other components, the retained
    /// cover, source trims and material visibility still need separate treatment.
    pub fn trace_contact_path(&self,joined: &ContactCurves,start: usize,forward: bool,
        tolerance: f64,distance: f64) -> Result<ContactPath,String> {
        if start >= joined.curves.len() { return Err("no such contact curve".into()); }
        if !tolerance.is_finite() || tolerance <= 0. { return Err("contact tolerance must be finite and positive".into()); }
        if !distance.is_finite() || distance <= 0. { return Err("contact join distance must be finite and positive".into()); }
        let transitions = self.contact_transitions(joined)?;
        let mut path = ContactPath {segments:Vec::new(),closed:false,join_error:0.};
        let (mut curve,mut forward,mut entry) = (start,forward,if forward { 0. } else { 1. });
        for _ in 0..=2*joined.curves.len() {
            let end = if forward { 1. } else { 0. };
            if let Some(index) = path.segments.iter().position(|s| s.curve == curve) {
                let previous = path.segments[index].range;
                if (previous[1] > previous[0]) != forward { return Err("contact path reverses an earlier interval".into()); }
                path.segments.drain(..index);
                if entry >= previous[0].min(previous[1]) && entry <= previous[0].max(previous[1]) {
                    // The incoming chart already traversed this first segment's
                    // prefix, so start the closed path at the shared point.
                    path.segments[0].range[0] = entry;
                } else {
                    path.segments.push(ContactPathSegment {curve,range:[entry,previous[0]]});
                }
                path.closed = true;
                return Ok(path);
            }
            path.segments.push(ContactPathSegment {curve,range:[entry,end]});
            let Some(link) = transitions[curve][usize::from(forward)] else { return Ok(path); };
            let p = self.at_contact_curve(&joined.curves[curve],end,tolerance)?.contact.position;
            let other = &joined.curves[link.curve];
            let [lo,hi] = link.coordinate.bounds();
            entry = (lo*0.5+hi*0.5-other.range[0])/(other.range[1]-other.range[0]);
            let q = self.at_contact_curve(other,entry,tolerance)?.contact.position;
            let gap = (p[0]-q[0]).hypot(p[1]-q[1]).hypot(p[2]-q[2]);
            if gap > distance { return Err("contact chart handoff exceeds join distance".into()); }
            path.join_error = path.join_error.max(gap);
            curve = link.curve; forward = link.forward;
        }
        Err("contact path did not terminate".into())
    }
}
