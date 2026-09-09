//! Native coordinate-seam splits of an already sampled on-face spatial curve.
use super::*;

extern "C" {
    fn solvent_cad_face_seams(cad: *mut c_void,face: c_int,axes: *mut c_int) -> c_int;
}

#[derive(Debug)]
pub(crate) struct FaceTrace {
    pub points: Vec<[f64;2]>,
    pub closed: bool,
    /// Source parameter intervals, in traversal order. A closed trace rebased
    /// at a seam can contain its final interval followed by its initial interval.
    pub intervals: Vec<[f64;2]>,
}

#[derive(Clone,Copy)]
struct Sample { s: f64,point: [f64;3],uv: [f64;2] }
fn distance(a: [f64;3],b: [f64;3]) -> f64 { (a[0]-b[0]).hypot(a[1]-b[1]).hypot(a[2]-b[2]) }
fn jumps(a: [f64;2],b: [f64;2]) -> Vec<usize> { (0..2).filter(|&k| (a[k]-b[k]).abs() > 0.5).collect() }

impl Session {
    pub(crate) fn face_seams(&self,face: c_int) -> Result<[bool;2],String> {
        let mut axes = [0;2];
        self.result(unsafe { solvent_cad_face_seams(self.0,face,axes.as_mut_ptr()) })?;
        Ok(axes.map(|v| v != 0))
    }

    fn seam_pair(&self,face: c_int,p: Sample,axis: usize,tolerance: f64)
        -> Result<Option<[[f64;2];2]>,String> {
        let mut uv = [p.uv;2]; uv[0][axis] = 0.; uv[1][axis] = 1.;
        let mut positions = [[0.;3];2];
        for k in 0..2 {
            let Some(q) = self.face_point(face,uv[k][0],uv[k][1],1e-9)? else { return Ok(None); };
            if !q.on_trim || distance(q.position,p.point) > tolerance { return Ok(None); }
            positions[k] = q.position;
        }
        Ok((distance(positions[0],positions[1]) <= tolerance).then_some(uv))
    }

    /// Split observed coordinate jumps, not discover all events of an arbitrary
    /// curve. Seeds must resolve its winding and on-face runs. Every supplied and
    /// refinement sample must lie on this face. Nonperiodic jumps, simultaneous
    /// seam crossings and unresolved incidence are errors. No missing run is filled.
    /// Tolerance is spatial mm; this does not bound interpolation between samples.
    pub(crate) fn face_traces(&self,face: c_int,at: &impl Fn(f64)->Result<[f64;3],String>,
        seeds: &[f64],closed: bool,tolerance: f64) -> Result<Vec<FaceTrace>,String> {
        if seeds.len() < 2 || seeds.len() > 4096 || seeds.iter().any(|s| !s.is_finite())
            || seeds.windows(2).any(|s| s[0] >= s[1]) {
            return Err("face trace needs 2..4096 increasing finite parameters".into());
        }
        let seams = self.face_seams(face)?;
        let project = |s| -> Result<Sample,String> {
            let point = at(s)?;
            let (uv,_) = self.face_parameters(face,point,tolerance)?.ok_or("trace leaves the native face")?;
            Ok(Sample {s,point,uv})
        };
        let mut samples = seeds.iter().map(|&s| project(s)).collect::<Result<Vec<_>,_>>()?;
        let last = samples.len()-1;
        if closed && distance(samples[0].point,samples[last].point) > tolerance {
            return Err("closed face trace has distinct spatial endpoints".into());
        }
        // A seed exactly at a seam should use the representative on the same
        // side as its neighbor, avoiding a spurious zero-length first/last piece.
        for (i,j) in [(0,1),(last,last-1)] {
            for axis in jumps(samples[i].uv,samples[j].uv) {
                if seams[axis] {
                    if let Some(pair) = self.seam_pair(face,samples[i],axis,tolerance)? {
                        samples[i].uv = pair[usize::from(samples[j].uv[axis] > 0.5)];
                    }
                }
            }
        }
        let mut parts: Vec<Vec<Sample>> = Vec::new();
        let mut current = vec![samples[0]];
        let push = |points: &mut Vec<Sample>,p: Sample| {
            if points.last().is_some_and(|q| q.s == p.s) { *points.last_mut().unwrap() = p; }
            else { points.push(p); }
        };
        for pair in samples.windows(2) {
            let axes = jumps(pair[0].uv,pair[1].uv);
            if !axes.is_empty() {
                if axes.len() != 1 { return Err("trace crosses simultaneous seams; refine its parameters".into()); }
                let axis = axes[0];
                if !seams[axis] { return Err("trace has a nonperiodic coordinate jump; refine its parameters".into()); }
                let high = pair[0].uv[axis] > 0.5;
                let (mut left,mut right) = (pair[0],pair[1]);
                let mut crossing = None;
                // Prefer an existing seed when it already reaches the seam.
                for p in [left,right] {
                    if let Some(uv) = self.seam_pair(face,p,axis,tolerance)? { crossing = Some((p,uv)); break; }
                }
                for _ in 0..64 {
                    if crossing.is_some() { break; }
                    let s = left.s*0.5+right.s*0.5;
                    if s == left.s || s == right.s { break; }
                    let p = project(s)?;
                    if let Some(uv) = self.seam_pair(face,p,axis,tolerance)? { crossing = Some((p,uv)); break; }
                    if (p.uv[axis] > 0.5) == high { left = p; } else { right = p; }
                }
                let (p,uv) = crossing.ok_or("native seam crossing did not converge")?;
                push(&mut current,Sample {uv:uv[usize::from(high)],..p});
                if current.len() > 1 { parts.push(current); }
                current = vec![Sample {uv:uv[usize::from(!high)],..p}];
            }
            if current.last().unwrap().s != pair[1].s { push(&mut current,pair[1]); }
        }
        if current.len() > 1 { parts.push(current); }
        let mut result: Vec<_> = parts.into_iter().map(|p| FaceTrace {
            intervals:vec![[p[0].s,p[p.len()-1].s]],points:p.into_iter().map(|p| p.uv).collect(),closed:false,
        }).collect();
        if closed && !result.is_empty() {
            let first = result[0].points[0];
            let last = *result.last().unwrap().points.last().unwrap();
            if jumps(first,last).is_empty() {
                if result.len() == 1 {
                    result[0].points.pop(); result[0].closed = true;
                } else {
                    let first = result.remove(0);
                    let last = result.last_mut().unwrap();
                    last.points.pop();
                    last.points.extend(first.points);
                    last.intervals.extend(first.intervals);
                }
            }
        }
        if result.iter().any(|p| p.points.windows(2).any(|p| !jumps(p[0],p[1]).is_empty())) {
            return Err("face trace retains an unresolved coordinate jump".into());
        }
        Ok(result)
    }
}
