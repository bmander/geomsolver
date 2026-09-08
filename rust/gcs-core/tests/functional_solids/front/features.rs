//! Local field-only feature probes. These are candidate measurements, not a
//! completeness or nonsmoothness proof. Front predictions can request a probe
//! after projection fails or turns sharply away from the incoming normal.
use super::*;
use gcs_core::linalg::{Mat,min_norm_solve};

#[derive(Debug)]
pub(super) struct Feature {pub(super) p:V,pub(super) rank:usize,pub(super) normals:Vec<V>,pub(super) inside:V,pub(super) outside:V}

impl Surface {
    pub(super) fn local_projection(&mut self,p:V) -> Option<(V,V,V,V)> {
        self.correct_with_step(p,self.point_tolerance*0.1)
    }

    pub(super) fn feature_vertex(&mut self,guess:V,radius:f64) -> Option<Vertex> {
        // An unchanged frontier can retry this exact neighborhood. Cache both
        // success and refusal; a different radius must run its own probe.
        let key = (guess.map(f64::to_bits),radius.to_bits());
        if let Some(result) = self.features.get(&key) { return result.clone(); }
        let result = self.feature_vertex_uncached(guess,radius);
        if self.cache && self.features.len() < 16384 { self.features.insert(key,result.clone()); }
        result
    }

    fn feature_vertex_uncached(&mut self,guess:V,radius:f64) -> Option<Vertex> {
        let feature = self.local_feature(guess,radius)?;
        let n = unit(feature.normals.iter().copied().fold([0.;3],add));
        let mut vertex = self.vertex(feature.p,n,feature.inside,feature.outside);
        vertex.branches = feature.normals;
        Some(vertex)
    }

    pub(super) fn differential(&mut self,p:V,h:f64) -> Option<V> {
        let (g,error) = self.gradient_measurement(p,h);
        (length(g) > 1e-10 && error < length(g)*0.01).then(|| unit(g))
    }

    pub(super) fn branch_normal(&mut self,p:V,mut radius:f64) -> Option<(V,f64)> {
        // Mixed finite differences at a crease must not become an extra face.
        // Keep only normals that are locally stable under small perturbations.
        for _ in 0..8 {
            let h = radius*0.01;
            if h < self.point_tolerance*0.01 { break; }
            let (g,error) = self.gradient_measurement(p,h); let magnitude = length(g);
            if magnitude < 1e-10 || error > magnitude*0.01 { return None; }
            let n = mul(g,1./magnitude);
            let mut stable = true;
            for k in 0..3 { for sign in [-1.,1.] {
                let mut q = p; q[k] += sign*radius*0.05;
                if length(sub(self.differential(q,h)?,n)) > 0.08 { stable = false; break; }
            }}
            if stable { return Some((n,magnitude)); }
            radius *= 0.25;
        }
        None
    }

    pub(super) fn local_feature(&mut self,guess:V,mut radius:f64) -> Option<Feature> {
        assert!(radius.is_finite() && radius > 0.);
        let mut center = self.local_projection(guess).map_or(guess,|(p,_,_,_)| p);
        let mut previous:Option<Feature> = None;
        for level in 0..4 {
            if radius < self.point_tolerance*8. { return None; }
            let mut samples = vec![];
            // Fixed directions sample space, not known model planes or edges.
            for x in -1_i32..=1 { for y in -1_i32..=1 { for z in -1_i32..=1 {
                if x.abs()+y.abs()+z.abs() != 1 && x.abs()+y.abs()+z.abs() != 3 { continue; }
                let direction = unit([x as f64,y as f64,z as f64]);
                // Projection can fail outside a narrow material cone. Its
                // stable field gradient can still supply a candidate plane;
                // only the final intersection needs boundary witnesses.
                let p = add(center,mul(direction,radius));
                if let Some((n,magnitude)) = self.branch_normal(p,radius) {
                    // Linearize the field at this sample, including its
                    // residual. For an affine branch this recovers the plane
                    // even when the sample could not project onto material.
                    let [lo,hi] = self.query(p,true).bounds();
                    let corrected = sub(p,mul(n,(lo*0.5+hi*0.5)/magnitude));
                    samples.push((corrected,n));
                }
            }}}
            let mut groups:Vec<(V,f64)> = vec![];
            for &(p,n) in &samples {
                let offset = dot(sub(p,guess),n);
                if let Some((sum,rhs)) = groups.iter_mut().find(|(m,_)| length(sub(unit(*m),n)) < 0.12) {
                    *sum = add(*sum,n); *rhs += offset;
                } else { groups.push((n,offset)); }
            }
            let normals:Vec<_> = groups.iter().map(|&(n,_)| unit(n)).collect();
            if normals.len() < 2 { return None; }
            // Work relative to the neighborhood center. The minimum-norm
            // solution anchors the free direction of a rank-two intersection.
            // Fit one equation per normal group. Curvature within a sampled
            // branch must not force an artificially large rank cutoff that
            // also discards the axial direction of a thin genuine corner.
            let matrix = Mat::from_vec(normals.len(),3,normals.iter().flatten().copied().collect());
            let rhs:Vec<_> = groups.iter().map(|&(n,rhs)| rhs/length(n)).collect();
            let (delta,rank) = min_norm_solve(&matrix,&rhs,1e-10);
            if rank < 2 { return None; }
            let delta = [delta[0],delta[1],delta[2]];
            // Least squares also returns a compromise for nonincident faces
            // (for example both walls of a thin strip). That point can lie on
            // another real boundary, so sign witnesses alone do not establish
            // a common feature. Shrink around the current local point rather
            // than moving the next neighborhood to an inconsistent fit.
            if normals.iter().zip(&rhs).any(|(&n,&rhs)|
                (dot(n,delta)-rhs).abs() > self.point_tolerance) {
                previous = None; radius *= 0.25; continue;
            }
            let proposed = add(guess,delta);
            if length(sub(proposed,center)) > radius {
                // A wide neighborhood can see several sheets whose common
                // intersection is distant. Shrink around the local surface
                // point to isolate a nearer crease, within the same work cap.
                previous = None; radius *= 0.25; continue;
            }
            // A central-difference normal at a sharp tip need not point into
            // its material cone. Try a direction positive against every
            // recovered branch, and accept it only with strict field signs.
            let (direction,_) = min_norm_solve(&matrix,&vec![1.;normals.len()],1e-10);
            let direction = [direction[0],direction[1],direction[2]];
            let crossing = if length(direction) > 1e-10 {
                let offset = mul(unit(direction),self.point_tolerance*0.5);
                let inside = sub(proposed,offset); let outside = add(proposed,offset);
                (self.value(inside).bounds()[1] < 0. && self.value(outside).bounds()[0] > 0.)
                    .then_some((proposed,inside,outside))
            } else { None };
            let (p,inside,outside) = crossing.or_else(|| self.local_projection(proposed).map(|(p,_,a,b)| (p,a,b)))?;
            if length(sub(p,proposed)) > radius*0.1 { return None; }
            let candidate = Feature {p,rank,normals,inside,outside};
            if level == 3 {
                let previous = previous?;
                let matches = |a:&[V],b:&[V]| a.iter().all(|&n| b.iter().any(|&m| length(sub(n,m)) < 0.12));
                return (previous.rank == rank && matches(&previous.normals,&candidate.normals) &&
                    matches(&candidate.normals,&previous.normals)).then_some(candidate);
            }
            center = p; previous = Some(candidate); radius *= 0.25;
        }
        None
    }
}
