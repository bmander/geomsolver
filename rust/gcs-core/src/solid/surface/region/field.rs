//! Read convex analytic meridians into the continuous material-field algebra.
use super::{Edge,P,RevolvedRegion,distance,PI,TAU};
use crate::solid::{PlanarField,RevolvedField};

fn cross(a: P,b: P) -> f64 { a[0]*b[1]-a[1]*b[0] }
fn delta(a: P,b: P) -> P { [a[0]-b[0],a[1]-b[1]] }
fn failure(e: crate::interval::Error) -> String { format!("profile field: {e:?}") }

impl Edge {
    fn reversed(mut self) -> Self {
        match &mut self {
            Self::Line {a,b,..} => std::mem::swap(a,b),
            Self::Arc {start,sweep,ends,..} => {
                *start += *sweep; *sweep = -*sweep; ends.swap(0,1);
            }
        }
        self
    }
    fn tangent(&self,t: f64) -> P {
        match *self {
            Self::Line {a,b,..} => delta(b,a),
            Self::Arc {start,sweep,..} => {
                let (s,c) = (start+t*sweep).sin_cos();
                [-s*sweep.signum(),c*sweep.signum()]
            }
        }
    }
    fn area_twice(&self) -> f64 {
        match *self {
            Self::Line {a,b,..} => cross(a,b),
            Self::Arc {center,radius,sweep,..} =>
                cross(center,delta(self.at(1.),self.at(0.)))+radius*radius*sweep,
        }
    }
}

/// Boundary order and individual edge direction need not match a face's written
/// operand order. Reconstruct one loop before checking turning and convexity.
fn ordered(edges: &[Edge]) -> Result<Vec<Edge>,String> {
    let scale = edges.iter().map(|e| match *e {
        Edge::Line {a,b,..} => a[0].hypot(a[1]).max(b[0].hypot(b[1])),
        Edge::Arc {center,radius,..} => center[0].hypot(center[1])+radius,
    }).fold(0_f64,f64::max);
    let tolerance = scale*f64::EPSILON*128.;
    let mut remaining = edges.to_vec();
    if remaining.is_empty() { return Err("empty material profile".into()); }
    let mut out = vec![remaining.remove(0)];
    while !remaining.is_empty() {
        let end = out.last().unwrap().at(1.);
        let mut next = None;
        for (i,e) in remaining.iter().enumerate() {
            for (t,reverse) in [(0.,false),(1.,true)] {
                if distance(end,e.at(t)) <= tolerance {
                    if next.is_some() { return Err("ambiguous material profile junction".into()); }
                    next = Some((i,reverse));
                }
            }
        }
        let (i,reverse) = next.ok_or("material profile endpoints do not meet")?;
        let edge = remaining.remove(i);
        out.push(if reverse { edge.reversed() } else { edge });
    }
    if distance(out[0].at(0.),out.last().unwrap().at(1.)) > tolerance {
        return Err("material profile is not closed".into());
    }
    Ok(out)
}

fn profile(edges: &[Edge]) -> Result<PlanarField,String> {
    let edges = ordered(edges)?;
    let area = edges.iter().map(Edge::area_twice).sum::<f64>();
    if !area.is_finite() || area == 0. { return Err("degenerate material profile".into()); }
    let direction = area.signum();
    let mut turning = 0.;
    for (i,e) in edges.iter().enumerate() {
        if let Edge::Arc {sweep,..} = *e {
            if sweep*direction <= 0. { return Err("material fields require convex profile loops".into()); }
            turning += sweep.abs();
        }
        let a = e.tangent(1.); let b = edges[(i+1)%edges.len()].tangent(0.);
        let turn = (direction*cross(a,b)).atan2(a[0]*b[0]+a[1]*b[1]);
        // Tangent junctions inherit the solved source's floating-point residual.
        if turn < -1e-9 { return Err("material fields require convex profile loops".into()); }
        turning += turn.max(0.);
    }
    if (turning-TAU).abs() > 1e-8 {
        return Err("material profile does not make one convex turn".into());
    }
    let mut constraints = Vec::new();
    let mut radius = 0_f64;
    let circular_section = edges.iter().filter(|e| !matches!(e,Edge::Line {axis:true,..})).count() == 1;
    for e in &edges {
        let field = match *e {
            Edge::Line {a,b,axis} => {
                radius = radius.max(a[0].hypot(a[1])).max(b[0].hypot(b[1]));
                // Cylindrical radius is already nonnegative. The construction
                // diameter of a sphere/cylinder must not become a material wall.
                if axis { continue; }
                PlanarField::half_plane(a,[direction*(b[1]-a[1]),direction*(a[0]-b[0])])
                    .map_err(failure)?
            }
            Edge::Arc {center,radius:r,sweep,..} => {
                radius = radius.max(center[0].hypot(center[1])+r);
                let disk = PlanarField::disk(center,r).map_err(failure)?;
                if sweep.abs() >= TAU || circular_section { disk }
                else {
                    let (a,b) = if direction > 0. { (e.at(0.),e.at(1.)) }
                        else { (e.at(1.),e.at(0.)) };
                    let a = delta(a,center); let b = delta(b,center);
                    let before = PlanarField::half_plane(center,[-a[1],a[0]]).map_err(failure)?;
                    let after = PlanarField::half_plane(center,[b[1],-b[0]]).map_err(failure)?;
                    // The circle constrains its angular sector only. Extending
                    // the whole disk would cut away distant parts of a fillet's profile.
                    let outside = if sweep.abs() <= PI { before.union(after) }
                        else { before.intersection(after) }.map_err(failure)?;
                    disk.union(outside).map_err(failure)?
                }
            }
        };
        constraints.push(field);
    }
    // Convex material lies in the convex hull of its boundary, hence this disk
    // contains it strictly. It also bounds profiles made solely of half-planes.
    let mut field = PlanarField::disk([0.;2],radius*2.).map_err(failure)?;
    for constraint in constraints { field = field.intersection(constraint).map_err(failure)?; }
    Ok(field)
}

impl RevolvedRegion {
    /// Convert supported solved profiles to an interval-evaluable material field.
    /// Each outer/hole loop must be convex and made of lines and circular arcs.
    /// Hole loops are subtracted, regardless of winding. Concave loops fail explicitly.
    /// The field represents this snapshot; source-solve and axis-snapping error
    /// are separate from interval evaluation and are not certified by this conversion.
    pub fn field(&self) -> Result<RevolvedField,String> {
        let mut loops = self.loops.iter();
        let mut field = profile(loops.next().ok_or("empty material profile")?)?;
        for hole in loops { field = field.difference(profile(hole)?).map_err(failure)?; }
        RevolvedField::new(field,self.origin,self.axis).map_err(failure)
    }
}
