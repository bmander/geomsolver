//! Whether a triangle mesh of a solid agrees with the solid's own material field: a point a
//! little inside each sampled triangle must not be exterior by the field, nor one a little
//! outside be material. This is the check that found the traced-sheet arrangement wrong on a
//! fifth to a third of its surface while its volume and its closed shell passed, so it is the
//! acceptance of every swept export (docs/generating-sweeps.md, Acceptance). It is sampled,
//! and a probe the field cannot decide is counted apart and never taken as agreement.
use super::MaterialEvaluator;
use crate::interval::{Interval,minimum::{self,Stop}};
use crate::space::{sub,cross,norm};

type V = [f64;3];

#[derive(Clone,Copy,Debug)]
pub struct Options {
    /// How far off each triangle, along its normal, a probe stands (model units).
    pub offset: f64,
    /// How far past zero the field at the triangle's own centroid must read, on the side the
    /// disagreeing probe found, for the disagreement to stand. A triangle lying on the true
    /// boundary has its centroid within its chordal deviation of zero, so a probe beside it that
    /// disagrees has passed through another face (beside an edge, across a thin wall); a
    /// triangle standing off the boundary has its centroid on the wrong side itself. Keep it
    /// above the mesh's chordal deviation, or faceting reads as a fault.
    pub confirm: f64,
    /// Triangles probed at most.
    pub triangles: usize,
    /// Choose them by area, stratified over the surface, so each probe answers for an equal
    /// share of it; by count, a patch of microscopic triangles draws probes out of all
    /// proportion to the surface it is. By index when false.
    pub by_area: bool,
    pub value_tolerance: f64,
    pub max_evaluations: usize,
}

impl Default for Options {
    fn default() -> Self { Options {offset:0.1,confirm:0.025,triangles:1000,by_area:true,value_tolerance:0.02,max_evaluations:20000} }
}

/// A probe the field decides the other way from the mesh.
#[derive(Clone,Copy,Debug)]
pub struct Disagreement {
    pub point: V,
    /// Which side of the mesh the probe stands on.
    pub inside_mesh: bool,
    pub field: [f64;2],
}

#[derive(Clone,Debug,Default)]
pub struct Agreement {
    pub triangles: usize,
    pub probed_triangles: usize,
    pub probes: usize,
    /// Probes whose field enclosure contains zero: the field could not say.
    pub unresolved: usize,
    /// Disagreements whose triangle's centroid reads on the boundary: a probe through another face.
    pub withdrawn: usize,
    pub disagreements: Vec<Disagreement>,
}

impl Agreement {
    pub fn agrees(&self) -> bool { self.disagreements.is_empty() }
}

/// Probe `triangles` over `vertices` against `material`, both in the same units, the triangles
/// wound outward.
pub fn of_triangles(vertices: &[V],triangles: &[[u32;3]],material: &mut MaterialEvaluator,options: &Options)
    -> Result<Agreement,String> {
    of_triangles_observed(vertices,triangles,material,options,&mut |_| {})
}

/// The same, telling `observe` the report so far after every triangle probed, for a caller
/// that shows progress; the core itself prints nothing.
pub fn of_triangles_observed(vertices: &[V],triangles: &[[u32;3]],material: &mut MaterialEvaluator,options: &Options,
    observe: &mut dyn FnMut(&Agreement)) -> Result<Agreement,String> {
    let mut report = Agreement {triangles:triangles.len(),..Default::default()};
    for t in sample(vertices,triangles,options) {
        observe(&report);
        report.add(probe(vertices,t,material,options)?);
    }
    Ok(report)
}

/// The same report, the triangles probed on every core, each thread reading `field` through an
/// evaluator of its own (poses cached `cache` a sweep): what an evaluator has cached changes how
/// fast it answers, never what, so the report is the one a single evaluator gives.
pub fn of_triangles_parallel(vertices: &[V],triangles: &[[u32;3]],field: &super::MaterialField,cache: usize,options: &Options)
    -> Result<Agreement,String> {
    let chosen = sample(vertices,triangles,options);
    let outcomes = crate::par::indices_with(chosen.len(),|| field.evaluator(cache),
        |material,i| probe(vertices,chosen[i],material,options));
    let mut report = Agreement {triangles:triangles.len(),..Default::default()};
    for outcome in outcomes { report.add(outcome?); }
    Ok(report)
}

/// What probing one triangle found.
#[derive(Default)]
struct Probed { probed: bool,probes: usize,unresolved: usize,withdrawn: bool,disagreements: Vec<Disagreement> }

impl Agreement {
    fn add(&mut self,p: Probed) {
        self.probed_triangles += usize::from(p.probed);
        self.probes += p.probes; self.unresolved += p.unresolved;
        self.withdrawn += usize::from(p.withdrawn);
        self.disagreements.extend(p.disagreements);
    }
}

/// Probe one triangle: a point `offset` inside it and one outside, and, where one side alone
/// disagrees, its centroid.
fn probe(vertices: &[V],t: &[u32;3],material: &mut MaterialEvaluator,options: &Options) -> Result<Probed,String> {
    let mut report = Probed::default();
    {
        let [a,b,c] = t.map(|i| vertices[i as usize]);
        let n = cross(sub(b,a),sub(c,a));
        let length = norm(n);
        if !(length > 0.) || !length.is_finite() { return Ok(report); }
        report.probed = true;
        let centroid: V = std::array::from_fn(|k| (a[k]+b[k]+c[k])/3.);
        // A probe asks only a sign, so every swept operand stops once its enclosure leaves
        // zero (a far sweep at once); the centroid asks only whether it lies within `confirm`.
        // Minima and maxima of enclosures each clear of a band are clear of it, so the answer
        // is the one full convergence would give, without refining what cannot change it.
        let mut ask = |distance: f64,stop: Stop,tolerance: f64| -> Result<(V,[f64;2]),String> {
            let point: V = std::array::from_fn(|k| centroid[k]+distance*n[k]/length);
            let bounds = material.query(point.map(|x| Interval::point(x).unwrap()),stop,
                minimum::Options {value_tolerance:tolerance,max_evaluations:options.max_evaluations},None)
                .map_err(|e| format!("{e:?}"))?;
            Ok((point,bounds.value.bounds()))
        };
        let mut found = Vec::new();
        for (distance,inside_mesh) in [(-options.offset,true),(options.offset,false)] {
            let (point,[lo,hi]) = ask(distance,Stop::Outside(Interval::ZERO),options.value_tolerance)?;
            report.probes += 1;
            if lo <= 0. && hi >= 0. { report.unresolved += 1; continue; }
            if (inside_mesh && lo > 0.) || (!inside_mesh && hi < 0.) {
                found.push(Disagreement {point,inside_mesh,field:[lo,hi]});
            }
        }
        // Both sides wrong is the triangle facing the wrong way, and stands whatever its
        // centroid reads. One side wrong stands only if the centroid is off the boundary too;
        // withdrawn only on the field's word, which an enclosure wider than `confirm` does not give.
        if found.len() == 1 {
            let band = Interval::new(-options.confirm,options.confirm).map_err(|e| format!("{e:?}"))?;
            let (_,[c_lo,c_hi]) = ask(0.,Stop::Decided(band),options.confirm/4.)?;
            if c_lo >= -options.confirm && c_hi <= options.confirm { report.withdrawn = true; return Ok(report); }
        }
        report.disagreements.extend(found);
    }
    Ok(report)
}

/// The triangles to probe: stratified by area (the one whose share of the cumulative area
/// holds each of `options.triangles` evenly spaced marks, each at most once), or every n-th.
fn sample<'a>(vertices: &[V],triangles: &'a [[u32;3]],options: &Options) -> Vec<&'a [u32;3]> {
    chosen(vertices,triangles,options).into_iter().map(|i| &triangles[i]).collect()
}

/// `sample`'s choice, as indices into `triangles`.
pub(crate) fn chosen(vertices: &[V],triangles: &[[u32;3]],options: &Options) -> Vec<usize> {
    let wanted = options.triangles.max(1);
    if !options.by_area {
        let step = (triangles.len()/wanted).max(1);
        return (0..triangles.len()).step_by(step).collect();
    }
    let mut cumulative = Vec::with_capacity(triangles.len());
    let mut total = 0.;
    for t in triangles {
        let [a,b,c] = t.map(|i| vertices[i as usize]);
        let area = 0.5*norm(cross(sub(b,a),sub(c,a)));
        if area.is_finite() { total += area; }
        cumulative.push(total);
    }
    if !(total > 0.) { return Vec::new(); }
    let mut chosen: Vec<usize> = (0..wanted).map(|k| {
        let mark = total*(k as f64+0.5)/wanted as f64;
        cumulative.partition_point(|&c| c < mark).min(triangles.len()-1)
    }).collect();
    chosen.dedup();
    chosen
}

/// The triangles of a binary STL, its coordinates divided by `scale` into model units.
pub fn stl_triangles(bytes: &[u8],scale: f64) -> Result<(Vec<V>,Vec<[u32;3]>),String> {
    if bytes.len() < 84 { return Err("an STL shorter than its header".into()); }
    let count = u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize;
    if bytes.len() != 84+50*count { return Err("an STL whose length does not match its triangle count".into()); }
    let mut vertices = Vec::with_capacity(3*count);
    let mut triangles = Vec::with_capacity(count);
    for i in 0..count {
        let base = 84+50*i+12;
        let mut t = [0u32;3];
        for (k,corner) in t.iter_mut().enumerate() {
            let at = |j: usize| f32::from_le_bytes(bytes[base+12*k+4*j..base+12*k+4*j+4].try_into().unwrap()) as f64/scale;
            *corner = vertices.len() as u32;
            vertices.push([at(0),at(1),at(2)]);
        }
        triangles.push(t);
    }
    Ok((vertices,triangles))
}
