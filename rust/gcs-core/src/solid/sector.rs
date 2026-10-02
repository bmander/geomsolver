//! One sector of an indexed body (docs/native-speed-plan.md). A body whose swept cuts are rigid
//! copies of one another turned about one axis, from a blank that turn leaves alike, is as many
//! copies of one sector as there are placements — when the sector can be bounded where no cut
//! reaches, so that nothing a cut makes is divided between two copies.
//!
//! Two readings, both of data a host already has. `indexing` reads the turn off the placements'
//! poses: every placement of every sweep one turn of that sweep's first by a whole number of
//! pitches about one line, the pitches covering the whole turn once. `Boundary::choose` reads the
//! sector's side off the cuts' own contacts: sliced by spheres about a centre on the axis (or by
//! planes square to it), each slice of a cut spans an arc about the axis; its turns by every pitch
//! leave a gap between neighbouring placements in that slice, and the boundary runs through the
//! middle of it. The side is a surface of the axis's sectors — at one angle in each slice, not one
//! angle overall, since a spiral tooth space turns across its face by as much as a pitch and no
//! flat half-plane clears it.
//!
//! Sampled, like the admission, and says so: a gap is read at the contacts the host traced, and a
//! host checks the surface it builds from the boundary against the gaps (`clearance_of`) and the
//! material field before it is used.
#[allow(unused_imports)]
use crate::fmath::Det;
use crate::envelope::Motion;
use crate::space::{sub,add,dot,cross,norm,scale};
use std::f64::consts::{PI,TAU};

type V = [f64;3];

/// The turn every placement is a whole number of pitches of: its line (a point and a unit axis),
/// how many placements make the whole turn, and each sweep's placements' indices.
#[derive(Clone,Debug)]
pub struct Indexing {
    pub origin: V,
    pub axis: V,
    pub count: usize,
    /// Per sweep, per placement in the order given: how many pitches it is turned from the
    /// sweep's first placement.
    pub indices: Vec<Vec<usize>>,
}

impl Indexing {
    /// One pitch, radians.
    pub fn pitch(&self) -> f64 { TAU/self.count as f64 }
    /// The turn by `k` pitches about the axis.
    pub fn turn(&self,k: usize) -> Motion {
        let about = Motion::rotation(self.axis,(k % self.count) as f64*self.pitch(),0.).expect("a unit axis");
        Motion::translation(scale(self.origin,-1.),[0.;3]).unwrap().then(about)
            .then(Motion::translation(self.origin,[0.;3]).unwrap())
    }
}

/// The rotation part of a motion, as rows.
fn matrix(m: Motion) -> [[f64;3];3] {
    let columns: [V;3] = std::array::from_fn(|j| { let mut e = [0.;3]; e[j] = 1.; m.vector(e) });
    std::array::from_fn(|i| std::array::from_fn(|j| columns[j][i]))
}

/// A unit vector square to `a`.
fn square_to(a: V) -> V {
    let seed = if a[0].abs() < 0.6 { [1.,0.,0.] } else { [0.,1.,0.] };
    let e = cross(a,seed);
    scale(e,1./norm(e))
}

/// Read the indexing off each sweep's placement poses (`size` is the body's scale, for the
/// tolerance: a pose off its turn by more than 1e-9 of it is not one). Refused, with the reason,
/// where the placements are not one indexing of at least three: a body cut once, or twice (the
/// two halves then cannot be told apart by their volume), is not worth a sector.
pub fn indexing(sweeps: &[Vec<Motion>],size: f64) -> Result<Indexing,String> {
    let tolerance = 1e-9*size.max(1.);
    let Some(first) = sweeps.first() else { return Err("the body has no swept cut".into()) };
    let count = first.len();
    if count < 3 { return Err(format!("the sweep is placed {count} times; a sector needs three or more")); }
    if let Some(s) = sweeps.iter().find(|s| s.len() != count) {
        return Err(format!("the sweeps are placed {count} and {} times: no one indexing",s.len()));
    }
    let pitch = TAU/count as f64;
    // the axis from the placement turned furthest from a half turn's ambiguity
    let relative = |s: &[Motion],j: usize| s[0].inverse().then(s[j]);
    let (mut axis,mut best) = ([0.;3],0.);
    for j in 1..count {
        let r = matrix(relative(first,j));
        let w = [r[2][1]-r[1][2],r[0][2]-r[2][0],r[1][0]-r[0][1]];
        if norm(w) > best { best = norm(w); axis = scale(w,1./norm(w)); }
    }
    if best < 1e-6 { return Err("the placements do not turn".into()); }
    // the line: (I - R) o = t on the plane square to the axis, for that placement
    let e1 = square_to(axis);
    let e2 = cross(axis,e1);
    let line = |m: Motion| -> Result<(V,f64,f64),String> {
        let r = matrix(m);
        let re1 = [dot(r[0],e1),dot(r[1],e1),dot(r[2],e1)];
        let (c,s) = (dot(re1,e1),dot(re1,e2));
        let angle = s.datan2(c).rem_euclid(TAU);
        let t = m.point([0.;3]);
        let slide = dot(t,axis);
        let (tx,ty) = (dot(t,e1),dot(t,e2));
        let det = (1.-c)*(1.-c)+s*s;
        if det < 1e-18 { return Ok(([0.;3],angle,slide)); }
        // (1-c) x + s y = tx, -s x + (1-c) y = ty
        let x = ((1.-c)*tx-s*ty)/det;
        let y = (s*tx+(1.-c)*ty)/det;
        Ok((add(scale(e1,x),scale(e2,y)),angle,slide))
    };
    let widest = (1..count).max_by(|&a,&b| {
        let sa = line(relative(first,a)).map(|l| l.1.dsin().abs()).unwrap_or(0.);
        let sb = line(relative(first,b)).map(|l| l.1.dsin().abs()).unwrap_or(0.);
        sa.total_cmp(&sb)
    }).unwrap();
    let (origin,_,_) = line(relative(first,widest))?;
    let mut indexing = Indexing {origin,axis,count,indices:Vec::new()};
    let probes = |o: V| [o,add(o,scale(axis,size)),add(o,scale(e1,size)),add(o,scale(e2,size))];
    for (n,s) in sweeps.iter().enumerate() {
        let mut seen = vec![false;count];
        let mut indices = Vec::new();
        for j in 0..count {
            let m = relative(s,j);
            let (_,angle,slide) = line(m)?;
            if slide.abs() > tolerance {
                return Err(format!("placement {j} of sweep {n} slides {slide:.3e} along the axis as it turns"));
            }
            let k = ((angle/pitch).round() as usize) % count;
            if (angle-k as f64*pitch).abs().min((angle-k as f64*pitch-TAU).abs()) > 1e-9 {
                return Err(format!("placement {j} of sweep {n} turns {:.6} degrees, not a whole number of {:.6} degree pitches",
                    angle.to_degrees(),pitch.to_degrees()));
            }
            let expected = indexing.turn(k);
            if let Some(p) = probes(origin).into_iter().find(|&p| norm(sub(m.point(p),expected.point(p))) > tolerance) {
                return Err(format!("placement {j} of sweep {n} is not a turn about the indexing axis (off by {:.3e} at {p:?})",
                    norm(sub(m.point(p),expected.point(p)))));
            }
            if std::mem::replace(&mut seen[k],true) {
                return Err(format!("sweep {n} is placed twice at {k} pitches"));
            }
            indices.push(k);
        }
        indexing.indices.push(indices);
    }
    Ok(indexing)
}

/// How a sector's side is sliced: by spheres about a centre on the axis (the slices a bevel
/// gear's teeth cross squarely, its end spheres being about its apex) or by planes square to it.
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Slices { Spheres(V),Planes }

/// A cylindrical frame about the indexing axis.
#[derive(Clone,Copy,Debug)]
pub struct Frame { pub origin: V,pub axis: V,e1: V,e2: V }

impl Frame {
    pub fn new(origin: V,axis: V) -> Frame {
        let axis = scale(axis,1./norm(axis));
        let e1 = square_to(axis);
        Frame {origin,axis,e1,e2:cross(axis,e1)}
    }
    /// A point's slice (its distance from the sphere's centre, or its height along the axis),
    /// its place in that slice (its angle from the axis, or its radius), and its angle about the
    /// axis.
    pub fn coordinates(&self,slices: Slices,p: V) -> [f64;3] {
        let d = sub(p,self.origin);
        let (x,y) = (dot(d,self.e1),dot(d,self.e2));
        let phi = y.datan2(x);
        match slices {
            Slices::Spheres(c) => {
                let d = sub(p,c);
                let (z,r) = (dot(d,self.axis),x.dhypot(y));
                [norm(d),r.datan2(z),phi]
            }
            Slices::Planes => [dot(d,self.axis),x.dhypot(y),phi],
        }
    }
    /// The point at a slice, a place in it and an angle.
    pub fn point(&self,slices: Slices,s: f64,w: f64,phi: f64) -> V {
        let radial = add(scale(self.e1,phi.dcos()),scale(self.e2,phi.dsin()));
        match slices {
            Slices::Spheres(c) => add(c,scale(add(scale(self.axis,w.dcos()),scale(radial,w.dsin())),s)),
            Slices::Planes => add(self.origin,add(scale(self.axis,s),scale(radial,w))),
        }
    }
    /// The direction square to the axis at angle `phi`.
    pub fn radial(&self,phi: f64) -> V { add(scale(self.e1,phi.dcos()),scale(self.e2,phi.dsin())) }
    /// A point's distance from the axis.
    pub fn radius(&self,p: V) -> f64 {
        let d = sub(p,self.origin);
        dot(d,self.e1).dhypot(dot(d,self.e2))
    }
}

/// How many slices the cuts' span is read in.
pub const SLICES: usize = 16;

/// A sector's side: in each slice of the cuts' span, the angle midway across the gap its cuts
/// leave, and the gap. Its turn by one pitch is the sector's other side.
#[derive(Clone,Debug)]
pub struct Boundary {
    pub frame: Frame,
    pub slices: Slices,
    pub pitch: f64,
    /// The slices' middles and the side's angle there.
    pub knots: Vec<f64>,
    pub phase: Vec<f64>,
    /// The side's angle over the slices: a cubic in the slice, scaled to [-1, 1] over the span,
    /// fitted to the gaps' middles — smooth, since a surface interpolating a wavering angle
    /// swings between its rows by more than the gap — and straight beyond the span.
    curve: [f64;4],
    /// Each slice's gap about the side, radians from it: the cuts in the slice and half a slice
    /// either side, read with the side's own turn across them taken out.
    pub gaps: Vec<[f64;2]>,
    /// The span of slices the cuts reach.
    pub span: [f64;2],
    /// The least gap either side of the side, as a length: its angle times the least radius of
    /// the contacts beside it.
    pub clearance: f64,
}

/// An angle wrapped into [-pitch/2, pitch/2).
fn wrapped(a: f64,pitch: f64) -> f64 { (a+pitch/2.).rem_euclid(pitch)-pitch/2. }

/// The widest gap between angles taken modulo `pitch`: its start and width.
fn widest(angles: impl Iterator<Item=f64>,pitch: f64) -> (f64,f64) {
    let mut a: Vec<f64> = angles.map(|x| x.rem_euclid(pitch)).collect();
    a.sort_by(f64::total_cmp);
    let (mut gap,mut at) = (0_f64,0_f64);
    for k in 0..a.len() {
        let next = if k+1 < a.len() { a[k+1] } else { a[0]+pitch };
        if next-a[k] > gap { gap = next-a[k]; at = a[k]; }
    }
    (at,gap)
}

impl Boundary {
    /// The side through the gaps `cuts` leave (each sweep's contacts at one of its placements,
    /// the ones in the blank), about `indexing`'s axis, read in each of `candidates` slicings; the
    /// one with the widest least gap. Refused where no slicing leaves a gap in every slice.
    pub fn choose(indexing: &Indexing,cuts: &[V],candidates: &[Slices]) -> Result<Boundary,String> {
        if cuts.is_empty() { return Err("no contact of any cut lies in the blank".into()); }
        let frame = Frame::new(indexing.origin,indexing.axis);
        let pitch = indexing.pitch();
        let mut best: Option<Boundary> = None;
        let mut reasons = Vec::new();
        for &slices in candidates {
            match Boundary::read(frame,slices,pitch,cuts) {
                Ok(b) => if best.as_ref().is_none_or(|a| b.clearance > a.clearance) { best = Some(b) },
                Err(e) => reasons.push(format!("{slices:?}: {e}")),
            }
        }
        best.ok_or_else(|| reasons.join("; "))
    }

    fn read(frame: Frame,slices: Slices,pitch: f64,cuts: &[V]) -> Result<Boundary,String> {
        let read: Vec<(f64,f64,f64)> = cuts.iter().map(|&p| { let [s,_,phi] = frame.coordinates(slices,p); (s,phi,frame.radius(p)) }).collect();
        let lo = read.iter().map(|r| r.0).fold(f64::INFINITY,f64::min);
        let hi = read.iter().map(|r| r.0).fold(f64::NEG_INFINITY,f64::max);
        if !(hi > lo) { return Err("the cuts span no slices".into()); }
        let h = (hi-lo)/SLICES as f64;
        // First the middle of each slice's widest gap, unwrapped from slice to slice.
        let (mut knots,mut phase): (Vec<f64>,Vec<f64>) = (Vec::new(),Vec::new());
        for i in 0..SLICES {
            let middle = lo+(i as f64+0.5)*h;
            let own: Vec<f64> = read.iter().filter(|r| (r.0-middle).abs() <= h/2.).map(|r| r.1).collect();
            if own.is_empty() { continue; }
            let (at,gap) = widest(own.into_iter(),pitch);
            let angle = at+gap/2.;
            knots.push(middle);
            phase.push(match phase.last() { Some(&previous) => angle+((previous-angle)/pitch).round()*pitch, None => angle });
        }
        if knots.is_empty() { return Err("no slice holds a contact".into()); }
        let mut b = Boundary {frame,slices,pitch,knots,phase,curve:[0.;4],gaps:Vec::new(),span:[lo,hi],clearance:f64::INFINITY};
        b.fit();
        // Then the gaps again, over each slice and half its neighbours, with the side's turn across
        // them taken out — a spiral cut turns across a slice by much of its gap — and the side moved
        // to their middles, twice; the third reading is the gaps kept.
        for round in 0..3 {
            let mut gaps = Vec::new();
            let mut clearance = f64::INFINITY;
            for &k in &b.knots {
                let window: Vec<&(f64,f64,f64)> = read.iter().filter(|r| (r.0-k).abs() <= h).collect();
                let (at,gap) = widest(window.iter().map(|r| r.1-b.angle_at(r.0)),pitch);
                let middle = wrapped(at+gap/2.,pitch);
                let gap = [middle-gap/2.,middle+gap/2.];
                let least = window.iter().map(|r| r.2).fold(f64::INFINITY,f64::min);
                clearance = clearance.min((-gap[0]).min(gap[1])*least);
                gaps.push(gap);
            }
            if round < 2 {
                let moved: Vec<f64> = b.knots.iter().zip(&gaps).map(|(&k,g)| b.angle_at(k)+(g[0]+g[1])/2.).collect();
                b.phase = moved;
                b.fit();
            } else { b.gaps = gaps; b.clearance = clearance; }
        }
        b.phase = b.knots.iter().map(|&k| b.angle_at(k)).collect();
        Ok(b)
    }

    /// The span scaled to [-1, 1].
    fn scaled(&self,s: f64) -> f64 { (2.*s-self.span[0]-self.span[1])/(self.span[1]-self.span[0]) }

    /// Fit the cubic to the knots' angles, least squares (a lower degree where there are too few).
    fn fit(&mut self) {
        let degree = (self.knots.len()-1).min(3);
        let mut a = [[0.;5];4];
        for (&k,&p) in self.knots.iter().zip(&self.phase) {
            let t = self.scaled(k);
            let powers: [f64;4] = std::array::from_fn(|i| if i <= degree { t.dpowi(i as i32) } else { 0. });
            for i in 0..=degree { for j in 0..=degree { a[i][j] += powers[i]*powers[j]; } a[i][4] += powers[i]*p; }
        }
        // Gaussian elimination with partial pivoting on the normal equations, well conditioned on [-1, 1]
        let n = degree+1;
        for c in 0..n {
            let pivot = (c..n).max_by(|&x,&y| a[x][c].abs().total_cmp(&a[y][c].abs())).unwrap();
            a.swap(c,pivot);
            for r in 0..n { if r != c {
                let f = a[r][c]/a[c][c];
                for k in c..5 { a[r][k] -= f*a[c][k]; }
            } }
        }
        self.curve = std::array::from_fn(|i| if i < n { a[i][4]/a[i][i] } else { 0. });
    }


    /// The side's angle at slice `s`.
    pub fn angle_at(&self,s: f64) -> f64 {
        let c = &self.curve;
        let value = |t: f64| c[0]+t*(c[1]+t*(c[2]+t*c[3]));
        let t = self.scaled(s);
        if t.abs() <= 1. { return value(t); }
        let end = t.signum();
        value(end)+(t-end)*(c[1]+end*2.*c[2]+3.*c[3])
    }

    /// The side as a grid of points over the slices `s` and the places across them `w`: rows
    /// evenly along the slices, a slice's width apart at most, and `columns` across them.
    pub fn grid(&self,s: [f64;2],w: [f64;2],columns: usize) -> (Vec<V>,usize) {
        let h = (self.span[1]-self.span[0])/SLICES as f64;
        let rows = (((s[1]-s[0])/h).ceil() as usize+1).max(4);
        let points = (0..rows).flat_map(|i| {
            let si = s[0]+(s[1]-s[0])*i as f64/(rows-1) as f64;
            (0..columns).map(move |j| {
                let wj = w[0]+(w[1]-w[0])*j as f64/(columns-1) as f64;
                self.frame.point(self.slices,si,wj,self.angle_at(si))
            })
        }).collect();
        (points,rows)
    }

    /// The least clearance of `points` (the side as built, where it is in the blank) from the
    /// cuts, as a length: how far inside its slice's gap each lies, times its radius. A point
    /// outside the cuts' span is clear of every cut. Refused at a point outside a gap.
    pub fn clearance_of(&self,points: &[V]) -> Result<f64,String> {
        let h = (self.span[1]-self.span[0])/SLICES as f64;
        let mut least = f64::INFINITY;
        for &p in points {
            let [s,_,phi] = self.frame.coordinates(self.slices,p);
            let off = wrapped(phi-self.angle_at(s),self.pitch);
            let (mut worst,mut any) = (f64::INFINITY,false);
            // every slice whose window holds this point
            for (k,g) in self.knots.iter().zip(&self.gaps) {
                if (s-k).abs() > h/2. { continue; }
                any = true;
                worst = worst.min((off-g[0]).min(g[1]-off));
            }
            if !any { continue; }
            let clear = worst*self.frame.radius(p);
            if clear <= 0. {
                return Err(format!("the side at {:?} lies {:.4} mm into a cut's slice ({:.3} degrees off its angle there, at slice {s:.3})",
                    p.map(|x| (x*1e3).round()/1e3),-clear,off.to_degrees()));
            }
            least = least.min(clear);
        }
        Ok(least)
    }

    /// How many pitches past this side a point is: 0 between it and its turn by one pitch.
    pub fn sector(&self,p: V) -> usize {
        let [s,_,phi] = self.frame.coordinates(self.slices,p);
        let count = (TAU/self.pitch).round() as usize;
        (((phi-self.angle_at(s)).rem_euclid(TAU)/self.pitch).floor() as usize).min(count-1)
    }
}

/// The span of slices and of places within them a blank covers, read in its section at angle
/// zero (the blank reads alike at every angle about the axis, which the host has checked), over
/// the box `bounds`, and widened by a twentieth and two samples each way. Refused where the
/// widened span reaches the axis: the sides of neighbouring sectors would meet there, in the
/// blank.
pub fn section_span(frame: Frame,slices: Slices,bounds: [V;2],inside: &dyn Fn(V) -> bool) -> Result<([f64;2],[f64;2]),String> {
    let corners: Vec<V> = (0..8).map(|k| std::array::from_fn(|i| bounds[(k >> i) & 1][i])).collect();
    let (s,w) = match slices {
        Slices::Spheres(c) => ([0.,corners.iter().map(|&p| norm(sub(p,c))).fold(0.,f64::max)],[0.,PI]),
        Slices::Planes => {
            let h: Vec<f64> = corners.iter().map(|&p| dot(sub(p,frame.origin),frame.axis)).collect();
            ([h.iter().copied().fold(f64::INFINITY,f64::min),h.iter().copied().fold(f64::NEG_INFINITY,f64::max)],
                [0.,corners.iter().map(|&p| frame.radius(p)).fold(0.,f64::max)])
        }
    };
    const N: usize = 160;
    let (mut s_in,mut w_in) = ([f64::INFINITY,f64::NEG_INFINITY],[f64::INFINITY,f64::NEG_INFINITY]);
    for i in 0..=N { for j in 0..=N {
        let (si,wj) = (s[0]+(s[1]-s[0])*i as f64/N as f64,w[0]+(w[1]-w[0])*j as f64/N as f64);
        if inside(frame.point(slices,si,wj,0.)) {
            s_in = [s_in[0].min(si),s_in[1].max(si)];
            w_in = [w_in[0].min(wj),w_in[1].max(wj)];
        }
    } }
    if !(s_in[1] >= s_in[0]) { return Err("the blank's section holds no sample".into()); }
    let pad = |r: [f64;2],step: f64| { let d = 0.05*(r[1]-r[0])+2.*step; [r[0]-d,r[1]+d] };
    let (s_out,w_out) = (pad(s_in,(s[1]-s[0])/N as f64),pad(w_in,(w[1]-w[0])/N as f64));
    if w_out[0] <= 0. { return Err("the blank reaches its axis: neighbouring sectors' sides would meet in it".into()); }
    if matches!(slices,Slices::Spheres(_)) && (w_out[1] >= PI || s_out[0] <= 0.) {
        return Err("the blank reaches its axis or the slices' centre".into());
    }
    Ok((s_out,w_out))
}
