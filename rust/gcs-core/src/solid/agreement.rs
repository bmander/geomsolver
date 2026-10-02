//! Whether a triangle mesh of a solid agrees with the solid's own material field: a point a
//! little inside each sampled triangle must not be exterior by the field, nor one a little
//! outside be material. This is the check that found the traced-sheet arrangement wrong on a
//! fifth to a third of its surface while its volume and its closed shell passed, so it is the
//! acceptance of every swept export (docs/generating-sweeps.md, Acceptance). It is sampled,
//! and a probe the field cannot decide is counted apart and never taken as agreement.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::MaterialEvaluator;
use crate::interval::{Interval,minimum::{self,Stop}};
use crate::space::{sub,cross,norm};
use std::collections::BTreeMap;

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

/// An indexed body's turn: the line it turns about (a point and a unit direction) and how many
/// turns by a pitch make the whole, as the body's construction read it off its placements.
#[derive(Clone,Copy,Debug)]
pub struct Indexed { pub origin: V,pub axis: V,pub count: usize }

/// How the probes of an indexed body were read (`of_triangles_indexed`).
#[derive(Clone,Debug,Default)]
pub struct Folding {
    /// Whether the field read alike under the turn at every point it was checked at; when not,
    /// every probe read the whole field where it stands.
    pub alike: bool,
    /// The cut operands, and those kept for the sector every probe is turned into.
    pub operands: usize,
    pub kept: usize,
    /// The boxes covering that sector, over which each operand left out was proved positive.
    pub cells: usize,
    /// Probes read with the whole field where they stand, being outside those boxes.
    pub whole: usize,
}

/// The side of the grid of boxes the turned probes are read in, in boxes across the body's support.
const COVER: usize = 96;
/// The evaluations a proof that an operand is positive over a box may take: an operand that reaches
/// the box is never proved so, and one not proved within them is kept.
const PROOF_EVALUATIONS: usize = 400;
/// Points the field is compared at with their turns, to take it as indexed.
const ALIKE_SAMPLES: usize = 16;

/// `of_triangles_parallel` for an indexed body: each probe turned by whole pitches into one sector
/// and read there, by the field with the cut operands left out that are proved positive over the box
/// of a grid it is in (`Sector`). The triangles probed, and each probe's point, are
/// `of_triangles_parallel`'s; where the field does not read alike under the turn, every probe reads
/// the whole field where it stands.
pub fn of_triangles_indexed(vertices: &[V],triangles: &[[u32;3]],field: &super::MaterialField,indexed: Indexed,cache: usize,
    options: &Options) -> Result<(Agreement,Folding),String> {
    let chosen = sample(vertices,triangles,options);
    let points: Vec<V> = chosen.iter().flat_map(|t| probe_points(vertices,t,options)).collect();
    let Some(sector) = Sector::new(field,indexed,&points,cache,options)? else {
        return of_triangles_parallel(vertices,triangles,field,cache,options).map(|a| (a,Folding::default()));
    };
    let clock = crate::clock::Instant::now();
    let outcomes = crate::par::indices_with(chosen.len(),|| sector.state(),|state,i| {
        let before = state.whole;
        let probed = probe_with(vertices,chosen[i],options,&mut |p,stop,tolerance| sector.read(state,p,stop,tolerance));
        probed.map(|p| (p,state.whole-before))
    });
    if std::env::var_os("SOLVENT_AGREEMENT_TIMES").is_some() { eprintln!("agreement: probed in {:?}",clock.elapsed()); }
    let mut report = Agreement {triangles:triangles.len(),..Default::default()};
    let mut folding = sector.folding.clone();
    for outcome in outcomes { let (p,whole) = outcome?; report.add(p); folding.whole += whole; }
    Ok((report,folding))
}

/// An indexed body's field read in one sector (`of_triangles_indexed`): a point turned by whole
/// pitches into the sector about angle zero, and read there by the field with the cut operands left
/// out (`MaterialField::without_cuts`) that are proved positive over the box of a grid over the
/// support the turned point is in — each box its own set, boxes with one set sharing a field. A turned
/// point reads what it would where it stands only where the field is alike under the turn, which is
/// checked first at points spread over the body's support and at their turns (a reading, sampled):
/// where it is not, there is no `Sector`. An operand left out that is positive over the box a point
/// is in changes no verdict — the field left is the same wherever the operand is positive, and
/// nowhere higher — so a point in no box of the grid reads the whole field where it stands.
pub struct Sector {
    field: super::MaterialField,
    cache: usize,
    max_evaluations: usize,
    origin: V,axis: V,e1: V,e2: V,pitch: f64,
    lo: V,step: V,
    /// Each box the points given were in, the set of operands kept there, and each set's field.
    boxes: BTreeMap<[usize;3],usize>,
    fields: Vec<super::MaterialField>,
    pub folding: Folding,
}

/// One thread's evaluators for a `Sector`, and how many points it read where they stand.
pub struct SectorState { sets: BTreeMap<usize,MaterialEvaluator>,full: MaterialEvaluator,pub whole: usize }

impl Sector {
    /// The sector the points `points` are read in once turned into it, its boxes the grid's that hold
    /// them; none where the field is not alike under the turn or has no support.
    pub fn new(field: &super::MaterialField,indexed: Indexed,points: &[V],cache: usize,options: &Options) -> Result<Option<Sector>,String> {
        let mut folding = Folding::default();
        let Indexed {origin,axis,count} = indexed;
        let n = norm(axis);
        if count < 2 || !(n > 0.) { return Ok(None); }
        let axis = axis.map(|x| x/n);
        let pitch = std::f64::consts::TAU/count as f64;
        let Some(support) = field.support_bounds().map_err(|e| format!("{e:?}"))? else { return Ok(None) };
        let [lo,hi] = [0,1].map(|k| support.map(|x| x.bounds()[k]));
        let size = crate::space::box_centre_diagonal(&support).1;
        let e1 = { let seed = if axis[0].abs() < 0.6 { [1.,0.,0.] } else { [0.,1.,0.] }; let c = cross(axis,seed); c.map(|x| x/norm(c)) };
        let e2 = cross(axis,e1);
        let pad = options.offset*2.;
        let (lo,hi): (V,V) = (std::array::from_fn(|k| lo[k]-pad),std::array::from_fn(|k| hi[k]+pad));
        let step: V = std::array::from_fn(|k| (hi[k]-lo[k])/COVER as f64);
        let mut sector = Sector {field:field.clone(),cache,max_evaluations:options.max_evaluations,origin,axis,e1,e2,pitch,lo,step,
            boxes:BTreeMap::new(),fields:Vec::new(),folding:Folding::default()};
        // the field alike under the turn
        let mut rng = crate::rng::Rng::new(0xf01d);
        let alike = (0..ALIKE_SAMPLES).all(|s| {
            let p: V = std::array::from_fn(|k| rng.uniform(lo[k],hi[k]));
            let k = 1+(s as i64)%(count as i64-1);
            let (a,b) = (field.reading(p).value,field.reading(sector.turn(p,k)).value);
            (a-b).abs() <= 1e-9*size.max(1.)
        });
        if !alike { return Ok(None); }
        folding.alike = true;
        let clock = crate::clock::Instant::now();
        let mut cells: Vec<[usize;3]> = Vec::new();
        for &p in points { if let Some(c) = sector.cell(sector.fold(p)) { if !cells.contains(&c) { cells.push(c); } } }
        folding.cells = cells.len();
        let boxed = |c: [usize;3]| -> Result<[Interval;3],String> {
            let mut out = [Interval::ZERO;3];
            for k in 0..3 {
                let a = lo[k]+step[k]*c[k] as f64;
                out[k] = Interval::new(a,a+step[k]).map_err(|e| format!("{e:?}"))?;
            }
            Ok(out)
        };
        let operands = field.cut_operands();
        folding.operands = operands.len();
        // an operand kept in a box unless proved positive over it, every operand and box on every core
        let pairs: Vec<(usize,usize)> = (0..cells.len()).flat_map(|c| (0..operands.len()).map(move |o| (c,o))).collect();
        let keep = crate::par::indices_with(pairs.len(),|| BTreeMap::<usize,MaterialEvaluator>::new(),|evaluators,i| -> Result<bool,String> {
            let (c,o) = pairs[i];
            let material = evaluators.entry(o).or_insert_with(|| operands[o].evaluator(cache));
            let bounds = material.query(boxed(cells[c])?,Stop::Outside(Interval::ZERO),
                minimum::Options {value_tolerance:options.value_tolerance,max_evaluations:PROOF_EVALUATIONS},None)
                .map_err(|e| format!("{e:?}"))?;
            Ok(!(bounds.value.bounds()[0] > 0.))
        }).into_iter().collect::<Result<Vec<bool>,String>>()?;
        // one field for each set of operands kept, and the set of each box
        let mut sets: Vec<Vec<bool>> = Vec::new();
        for (c,&cell) in cells.iter().enumerate() {
            let kept = keep[c*operands.len()..(c+1)*operands.len()].to_vec();
            let k = match sets.iter().position(|s| *s == kept) { Some(k) => k,None => { sets.push(kept); sets.len()-1 } };
            sector.boxes.insert(cell,k);
        }
        folding.kept = sets.iter().map(|s| s.iter().filter(|k| **k).count()).max().unwrap_or(0);
        sector.fields = sets.iter().map(|kept| {
            let mut next = kept.iter();
            field.without_cuts(&mut |_| *next.next().expect("one answer an operand")).map_err(|e| format!("{e:?}"))
        }).collect::<Result<Vec<_>,String>>()?;
        if std::env::var_os("SOLVENT_AGREEMENT_TIMES").is_some() {
            eprintln!("agreement: {} boxes, {} sets, proved in {:?}",cells.len(),sets.len(),clock.elapsed());
        }
        sector.folding = folding;
        Ok(Some(sector))
    }

    /// `p` turned by `k` pitches about the axis.
    fn turn(&self,p: V,k: i64) -> V {
        crate::envelope::Motion::rotation(self.axis,k as f64*self.pitch,0.).map(|m| {
            let q = m.point(sub(p,self.origin)); std::array::from_fn(|i| q[i]+self.origin[i])
        }).unwrap_or(p)
    }

    /// `p` turned by whole pitches into the sector about angle zero.
    pub fn fold(&self,p: V) -> V {
        let d = sub(p,self.origin);
        let angle = crate::space::dot(d,self.e2).datan2(crate::space::dot(d,self.e1));
        self.turn(p,-(angle/self.pitch).round() as i64)
    }

    /// The box of the grid a point is in, if any.
    fn cell(&self,q: V) -> Option<[usize;3]> {
        let c: [f64;3] = std::array::from_fn(|k| ((q[k]-self.lo[k])/self.step[k]).floor());
        c.iter().all(|&x| x >= 0. && x < COVER as f64).then(|| c.map(|x| x as usize))
    }

    /// A thread's evaluators.
    pub fn state(&self) -> SectorState { SectorState {sets:BTreeMap::new(),full:self.field.evaluator(self.cache),whole:0} }

    /// The field's enclosure at `p`, read turned into the sector where its box has a set, and where
    /// it stands otherwise, each sweep stopping as `stop` says at `tolerance`.
    pub fn read(&self,state: &mut SectorState,p: V,stop: Stop,tolerance: f64) -> Result<[f64;2],String> {
        let q = self.fold(p);
        let set = self.cell(q).and_then(|c| self.boxes.get(&c).copied());
        let (at,evaluator) = match set {
            Some(k) => (q,state.sets.entry(k).or_insert_with(|| self.fields[k].evaluator(self.cache))),
            None => { state.whole += 1; (p,&mut state.full) }
        };
        let bounds = evaluator.query(at.map(|x| Interval::point(x).unwrap()),stop,
            minimum::Options {value_tolerance:tolerance,max_evaluations:self.max_evaluations},None)
            .map_err(|e| format!("{e:?}"))?;
        Ok(bounds.value.bounds())
    }
}

/// Every point `probe_with` may read at a triangle: its centroid and the points off each side.
fn probe_points(vertices: &[V],t: &[u32;3],options: &Options) -> Vec<V> {
    let [a,b,c] = t.map(|i| vertices[i as usize]);
    let n = cross(sub(b,a),sub(c,a));
    let length = norm(n);
    if !(length > 0.) || !length.is_finite() { return Vec::new(); }
    let centroid: V = std::array::from_fn(|k| (a[k]+b[k]+c[k])/3.);
    [0.,-options.offset,options.offset].iter().map(|&d| std::array::from_fn(|k| centroid[k]+d*n[k]/length)).collect()
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
    probe_with(vertices,t,options,&mut |point,stop,tolerance| {
        let bounds = material.query(point.map(|x| Interval::point(x).unwrap()),stop,
            minimum::Options {value_tolerance:tolerance,max_evaluations:options.max_evaluations},None)
            .map_err(|e| format!("{e:?}"))?;
        Ok(bounds.value.bounds())
    })
}

/// `probe`, the field's enclosure at a point read by `read` (the point, the stop, the value tolerance).
fn probe_with(vertices: &[V],t: &[u32;3],options: &Options,read: &mut dyn FnMut(V,Stop,f64) -> Result<[f64;2],String>)
    -> Result<Probed,String> {
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
            Ok((point,read(point,stop,tolerance)?))
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
