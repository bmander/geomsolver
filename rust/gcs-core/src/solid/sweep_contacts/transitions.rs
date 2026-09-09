//! Reparameterization at curve endpoints, using shared implicit-root enclosures.
use super::{SweepContacts,ContactCurves,ContactCurve,ContactEvidence,ContactParameter,ContactCover,ContactCoverOptions};
use crate::interval::{Interval as I,Error};

#[derive(Clone,Copy,Debug)]
pub struct ContactTransition {
    /// Destination index in the same ContactCurves snapshot.
    pub curve: usize,
    /// Enclosure of the shared point's original free destination coordinate.
    /// This is not normalized over the destination interval.
    pub coordinate: I,
    /// Direction along the destination after leaving the source endpoint.
    pub forward: bool,
}

impl SweepContacts {
    /// Spend an additional bounded discovery budget only on unresolved cells
    /// touching unconnected curve endpoints. The whole partition remains intact;
    /// rejoin the returned cover to obtain updated curves and transitions.
    pub fn refine_contact_ends(&self,joined: ContactCurves,options: ContactCoverOptions)
        -> Result<ContactCover,String> {
        let links = self.contact_transitions(&joined)?;
        let mut selected = std::collections::BTreeSet::new();
        for (curve,ends) in joined.curves.iter().zip(links) {
            for (end,link) in ends.iter().enumerate() {
                if link.is_some() { continue; }
                let parameters = self.contact_endpoint_box(&joined,curve,curve.range[end]).map_err(|e| format!("{e:?}"))?;
                for (i,cell) in joined.cover.cells.iter().enumerate() {
                    if cell.patch != curve.patch || !matches!(cell.evidence,ContactEvidence::Unresolved {..}) { continue; }
                    if cell.parameters.iter().zip(parameters).all(|(a,b)| {
                        let [a,b0] = a.bounds(); let [c,d] = b.bounds(); a <= d && c <= b0
                    }) { selected.insert(i); }
                }
            }
        }
        self.refine_cells(joined.cover,&selected.into_iter().collect::<Vec<_>>(),options).map_err(|e| format!("{e:?}"))
    }

    /// For each joined interval's low/high endpoint, find an alternate chart
    /// containing its entire root enclosure and extending past that endpoint.
    /// The source patch and time must agree. Uniqueness in the destination's
    /// retained charts establishes branch identity; no spatial weld is used.
    /// None retains an unresolved end, including seams and source-patch changes.
    pub fn contact_transitions(&self,joined: &ContactCurves)
        -> Result<Vec<[Option<ContactTransition>;2]>,String> {
        let mut result = Vec::new();
        for (index,curve) in joined.curves.iter().enumerate() {
            let mut ends = [None;2];
            for (end,&value) in curve.range.iter().enumerate() {
                let parameters = self.contact_endpoint_box(joined,curve,value)
                    .map_err(|e| format!("contact endpoint enclosure: {e:?}"))?;
                let surface = self.patches[curve.patch].angular_chart(parameters[1].bounds())
                    .map_err(|e| format!("{e:?}"))?;
                let bounds = surface.bounds(parameters[0],parameters[1]).map_err(|e| format!("{e:?}"))?;
                let slopes = [(bounds.normal_du,bounds.moment_du),(bounds.normal_dv,bounds.moment_dv)]
                    .map(|(n,m)| self.motion.normal_velocity_moment_bounds(n,m)?.at(parameters[2]));
                let [du,dv] = slopes;
                let slopes = [du.map_err(|e| format!("{e:?}"))?,dv.map_err(|e| format!("{e:?}"))?];
                if slopes.iter().any(|d| d.contains(0.)) { continue; }
                // d(dependent)/d(free) = -g_free/g_dependent. Both are
                // nonzero here, so the orientation is established by signs.
                let increasing = (slopes[0].bounds()[0] > 0.) != (slopes[1].bounds()[0] > 0.);
                let forward = (end == 1) == increasing;
                let coordinate = parameters[curve.dependent.index()];
                for (target,other) in joined.curves.iter().enumerate() {
                    if target == index || other.patch != curve.patch || other.time != curve.time
                        || other.dependent == curve.dependent { continue; }
                    let [lo,hi] = coordinate.bounds();
                    // Keep the whole enclosure in the alternate curve, and
                    // require room to continue in the outgoing direction.
                    if lo < other.range[0] || hi > other.range[1]
                        || (forward && hi >= other.range[1]) || (!forward && lo <= other.range[0]) { continue; }
                    let mut intervals = Vec::new();
                    for &cell in &other.cells {
                        let cell = joined.cover.cells.get(cell).ok_or("invalid contact chart index")?;
                        let ContactEvidence::Chart(chart) = cell.evidence else { return Err("contact curve contains a non-chart".into()); };
                        if chart.range.contains(value) {
                            intervals.push(cell.parameters[other.dependent.free()[0]].bounds());
                        }
                    }
                    intervals.sort_by(|a,b| a[0].total_cmp(&b[0]));
                    let mut covered = lo;
                    let mut found = false;
                    for [a,b] in intervals {
                        if a > covered { break; }
                        if b >= covered { covered = b; found = true; }
                    }
                    if !found || covered < hi { continue; }
                    if ends[end].is_some() { return Err("ambiguous alternate contact chart".into()); }
                    ends[end] = Some(ContactTransition {curve:target,coordinate,forward});
                }
            }
            result.push(ends);
        }
        Ok(result)
    }

    fn contact_endpoint_box(&self,joined: &ContactCurves,curve: &ContactCurve,value: f64)
        -> Result<[I;3],Error> {
        let free = curve.dependent.free()[0];
        if curve.dependent == ContactParameter::Time { return Err(Error::OutsideDomain); }
        let cell = curve.cells.iter().filter_map(|&i| joined.cover.cells.get(i))
            .find(|c| c.parameters[free].contains(value)).ok_or(Error::OutsideDomain)?;
        let ContactEvidence::Chart(chart) = cell.evidence else { return Err(Error::OutsideDomain); };
        let mut parameters = cell.parameters;
        parameters[free] = I::point(value)?;
        parameters[2] = I::point(curve.time)?;
        let axis = curve.dependent.index();
        let mut root = chart.range;
        for _ in 0..64 {
            let [lo,hi] = root.bounds();
            let mid = lo*0.5+hi*0.5;
            if mid == lo || mid == hi { break; }
            parameters[axis] = I::point(mid)?;
            let surface = self.patches.get(curve.patch).ok_or(Error::OutsideDomain)?
                .angular_chart(parameters[1].bounds()).map_err(|_| Error::OutsideDomain)?;
            let b = surface.bounds(parameters[0],parameters[1])?;
            let f = self.motion.normal_velocity_moment_bounds(b.normal,b.moment)?.at(parameters[2])?;
            // Interval Newton retains the unique root even when the midpoint's
            // rounded equation cannot resolve a sign. The chart derivative
            // encloses every intermediate point between midpoint and root.
            let [a,b] = I::point(mid)?.sub(f.div(chart.derivative)?)?.bounds();
            let next = I::new(lo.max(a),hi.min(b))?;
            if next == root { break; }
            root = next;
        }
        parameters[axis] = root;
        Ok(parameters)
    }
}
