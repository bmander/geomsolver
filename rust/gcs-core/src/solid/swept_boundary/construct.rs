//! The construction end to end: the sheets traced, the caps cut, every vertex judged, the sheets
//! clipped at their creases and the rims merged, the triangles kept by their centroids, unioned
//! in their planes, cleared of double coverage, welded, split at their T-junctions, zipped, and
//! every triangle certified. Every tolerance a stage uses comes from `SweptBoundaryOptions`,
//! and each stage is shown to an observer as it finishes, with the judge's counts so far.
use super::{Cap,Certificate,FieldJudge,Grazing,JudgeError,KeptMesh,Labelled,QueryStats,Rim,Seed,SweptBoundaryOptions};
use crate::model::Sketch;
use crate::solid::{MaterialField,SweepContacts,SweepPatch};

/// A stage just finished: what it made, and what it counted.
pub enum Stage<'a> {
    /// The traced sheets.
    Seeded { sheets: &'a [SweepPatch] },
    /// The two caps cut from the tool at the ends of the roll.
    Capped { caps: &'a [Cap] },
    /// The region each grazing planar face sweeps within its own plane.
    Grazed { regions: &'a [Grazing] },
    /// Every vertex of every seed judged.
    Labelled { seeds: &'a [Seed], labelled: &'a [Labelled] },
    /// The seeds' kept triangles, clipped at their creases, and the rims the clipping made.
    Clipped { mesh: &'a KeptMesh, rims: &'a [Rim] },
    /// The rims of different sheets merged along the creases they share.
    Merged { mesh: &'a KeptMesh, rims: &'a [Rim], welded: usize, split: usize },
    /// The triangles judged by their centroids: `keep` over `before`, and what was `kept`.
    Kept { before: &'a KeptMesh, keep: &'a [bool], kept: &'a KeptMesh, inside: usize, outside: usize },
    /// Each plane's triangles unioned; `replaced` of them went into the union.
    Unioned { mesh: &'a KeptMesh, replaced: usize },
    /// Triangles an earlier sheet covers clipped away; `dropped` of them were touched.
    Uncovered { mesh: &'a KeptMesh, dropped: usize },
    /// Coincident vertices welded, short edges collapsed, doubled slivers dropped.
    Welded { mesh: &'a KeptMesh, collapsed: usize, doubled: usize },
    /// T-junctions split.
    Split { mesh: &'a KeptMesh },
    /// Rims zipped in pairs, and the boundary loops left unpaired.
    Zipped { mesh: &'a KeptMesh, pairs: usize, unpaired: &'a [Vec<u32>] },
    /// Every triangle certified (or not).
    Certified { mesh: &'a KeptMesh, certificate: &'a Certificate },
}

/// Inspectable construction output. A candidate makes no surface-acceptance claim.
/// The report is available even with open rims or failed/unknown centroid checks.
pub struct BoundaryCandidate {
    pub mesh: KeptMesh,
    pub certificate: Certificate,
    pub stats: QueryStats,
    pub unpaired: Vec<Vec<u32>>,
    pub topology: Result<crate::topology::ClosedShell,crate::topology::Error>,
    field: MaterialField,
}

/// Immutable accepted single shell for one material snapshot. The spatial audit
/// bounds both distance directions; it does not establish isotopy, geometric
/// embedding, source-solve error or the accuracy of a later STL encoding.
#[derive(Debug)]
pub struct SweptBoundary {
    mesh: KeptMesh,
    certificate: Certificate,
    shell: crate::topology::ClosedShell,
    spatial: super::SpatialAudit,
    stats: QueryStats,
}
impl SweptBoundary {
    pub fn mesh(&self) -> &KeptMesh { &self.mesh }
    pub fn certificate(&self) -> &Certificate { &self.certificate }
    pub fn shell(&self) -> &crate::topology::ClosedShell { &self.shell }
    pub fn spatial(&self) -> &super::SpatialAudit { &self.spatial }
    /// Field work performed by acceptance; construction has its own candidate stats.
    pub fn stats(&self) -> &QueryStats { &self.stats }
}

impl BoundaryCandidate {
    /// Diagnose independent obligations even when topology or centroid checks fail.
    pub fn inspect(&self,options: &SweptBoundaryOptions,audit: super::AuditOptions) -> BoundaryReport {
        inspect(self.field.clone(),self.mesh.clone(),options,audit)
    }
    /// Revalidate the actual candidate, since callers may have edited its mesh or
    /// report. Only fresh evidence can create an accepted result.
    pub fn accept(&self,options: &SweptBoundaryOptions,audit: super::AuditOptions) -> Result<SweptBoundary,ConstructError> {
        validate(self.field.clone(),self.mesh.clone(),options,audit)
    }
}

/// Whether an independent check was attempted. Invalid mesh input prevents all
/// checks; spatial setup limitations are recorded inside the audit report.
#[derive(Clone,Debug)]
pub enum Check<T> { Attempted(T), NotAttempted(ConstructError) }

/// Immutable diagnostics of the actual mesh, independent of earlier candidate
/// labels. A report can only become accepted by satisfying every gate.
#[derive(Debug)]
pub struct BoundaryReport {
    mesh: KeptMesh,
    topology: Check<Result<crate::topology::ClosedShell,crate::topology::Error>>,
    certificate: Check<Result<Certificate,JudgeError>>,
    spatial: Check<super::AuditReport>,
    stats: QueryStats,
}
impl BoundaryReport {
    pub fn mesh(&self) -> &KeptMesh { &self.mesh }
    pub fn topology(&self) -> &Check<Result<crate::topology::ClosedShell,crate::topology::Error>> { &self.topology }
    pub fn certificate(&self) -> &Check<Result<Certificate,JudgeError>> { &self.certificate }
    pub fn spatial(&self) -> &Check<super::AuditReport> { &self.spatial }
    pub fn stats(&self) -> &QueryStats { &self.stats }
    pub fn into_accepted(self) -> Result<SweptBoundary,ConstructError> {
        let shell = match self.topology { Check::Attempted(r) => r.map_err(ConstructError::Topology)?,Check::NotAttempted(e) => return Err(e) };
        let certificate = match self.certificate { Check::Attempted(r) => r?,Check::NotAttempted(e) => return Err(e) };
        if !certificate.is_complete() { return Err(ConstructError::Certificate {failed:certificate.failures.len(),unresolved:certificate.unresolved.len()}); }
        let spatial = match self.spatial { Check::Attempted(r) => r.into_result().map_err(ConstructError::Spatial)?,Check::NotAttempted(e) => return Err(e) };
        Ok(SweptBoundary {mesh:self.mesh,certificate,shell,spatial,stats:self.stats})
    }
}

fn valid_mesh(mesh: &KeptMesh) -> bool {
    mesh.vertices.iter().flatten().all(|x| x.is_finite())
        && mesh.triangles.iter().flatten().all(|&i| (i as usize) < mesh.vertices.len())
        && mesh.sheet.len() == mesh.triangles.len()
}

// Check before downstream operations can hide malformed ownership by retaining
// or zipping arrays. Semantic inheritance is covered by parent-domain regressions.
fn check_stage_mesh(mesh: &KeptMesh,sources: usize,stage: &'static str) -> Result<(),ConstructError> {
    if !valid_mesh(mesh) || mesh.sheet.iter().any(|&s| s != u32::MAX && s as usize >= sources) {
        return Err(ConstructError::InvalidStageMesh {stage});
    }
    Ok(())
}

/// Bounded diagnostic mode. Topology failure does not suppress centroid or
/// spatial work. Only invalid input prevents these independent checks.
pub fn inspect(field: MaterialField,mesh: KeptMesh,options: &SweptBoundaryOptions,audit: super::AuditOptions) -> BoundaryReport {
    if !valid_mesh(&mesh) {
        return BoundaryReport {mesh,topology:Check::NotAttempted(ConstructError::InvalidMesh),
            certificate:Check::NotAttempted(ConstructError::InvalidMesh),spatial:Check::NotAttempted(ConstructError::InvalidMesh),stats:QueryStats::default()};
    }
    let mesh = mesh.compact();
    let triangles: Vec<_> = mesh.triangles.iter().map(|t| t.map(|i| i as usize)).collect();
    let topology = crate::topology::ClosedShell::from_triangles(mesh.vertices.len(),&triangles);
    let mut judge = FieldJudge::new(field.clone(),options.judge_tolerance(),options.near_budget,options.far_budget,options.cached_poses);
    let certificate = super::certify(&mut judge,&mesh.vertices,&mesh.triangles,options.probe_distance(),options.least_probe());
    let spatial = super::audit::report(&field,&mut judge,&mesh,audit);
    BoundaryReport {mesh,topology:Check::Attempted(topology),certificate:Check::Attempted(certificate),spatial:Check::Attempted(spatial),stats:judge.stats}
}

/// Validate a proposed mesh without trusting caller-supplied labels or reports.
pub fn validate(field: MaterialField,mesh: KeptMesh,options: &SweptBoundaryOptions,audit: super::AuditOptions)
    -> Result<SweptBoundary,ConstructError> {
    if !valid_mesh(&mesh) { return Err(ConstructError::InvalidMesh); }
    let mesh = mesh.compact();
    // Range checks precede compaction; topology then checks the complete vertex links.
    let triangles: Vec<_> = mesh.triangles.iter().map(|t| t.map(|i| i as usize)).collect();
    let shell = crate::topology::ClosedShell::from_triangles(mesh.vertices.len(),&triangles).map_err(ConstructError::Topology)?;
    let mut judge = FieldJudge::new(field.clone(),options.judge_tolerance(),options.near_budget,options.far_budget,options.cached_poses);
    let certificate = super::certify(&mut judge,&mesh.vertices,&mesh.triangles,options.probe_distance(),options.least_probe())?;
    if !certificate.is_complete() {
        return Err(ConstructError::Certificate {failed:certificate.failures.len(),unresolved:certificate.unresolved.len()});
    }
    let spatial = super::audit::audit(&field,&mut judge,&mesh,audit).map_err(ConstructError::Spatial)?;
    Ok(SweptBoundary {mesh,certificate,shell,spatial,stats:judge.stats})
}

/// Why a construction stopped.
#[derive(Clone,Debug,PartialEq)]
pub enum ConstructError {
    /// The tracer declined.
    Seeds(String),
    InvalidMesh,
    /// A construction operation produced malformed geometry or source metadata.
    InvalidStageMesh { stage: &'static str },
    Topology(crate::topology::Error),
    Certificate { failed: usize, unresolved: usize },
    Spatial(super::AuditError),
    /// A cap could not be cut from the tool.
    Caps(String),
    /// A grazing face's region could not be swept.
    Grazing(String),
    /// The sweep's field could not be read.
    Field(String),
    /// The field refused a judgement.
    Judge(JudgeError),
}

impl From<JudgeError> for ConstructError { fn from(e: JudgeError) -> Self { ConstructError::Judge(e) } }

/// The certified boundary of the swept solid `swept`, made as `options` say; `progress` hears
/// the tracer, and `observe` each stage as it finishes.
pub fn construct(sk: &Sketch,swept: usize,options: &SweptBoundaryOptions,progress: &dyn Fn(&str),
    observe: &mut dyn FnMut(Stage<'_>,&QueryStats)) -> Result<SweptBoundary,ConstructError> {
    let candidate = candidate(sk,swept,options,progress,observe)?;
    candidate.accept(options,options.audit())
}

/// `construct` from sheets already traced (the tracer's, or a test's own: moved, thinned,
/// one withheld), from the caps on.
pub fn construct_from(sk: &Sketch,swept: usize,options: &SweptBoundaryOptions,sheets: Vec<SweepPatch>,
    observe: &mut dyn FnMut(Stage<'_>,&QueryStats)) -> Result<SweptBoundary,ConstructError> {
    let candidate = candidate_from(sk,swept,options,sheets,observe)?;
    candidate.accept(options,options.audit())
}

/// Build an inspectable candidate without claiming surface acceptance.
pub fn candidate(sk: &Sketch,swept: usize,options: &SweptBoundaryOptions,progress: &dyn Fn(&str),
    observe: &mut dyn FnMut(Stage<'_>,&QueryStats)) -> Result<BoundaryCandidate,ConstructError> {
    let (_,sheets,_) = super::seeds(sk,swept,options.spacing,options.sagitta,progress).map_err(ConstructError::Seeds)?;
    candidate_from(sk,swept,options,sheets,observe)
}

pub fn candidate_from(sk: &Sketch,swept: usize,options: &SweptBoundaryOptions,sheets: Vec<SweepPatch>,
    observe: &mut dyn FnMut(Stage<'_>,&QueryStats)) -> Result<BoundaryCandidate,ConstructError> {
    let none = QueryStats::default();
    observe(Stage::Seeded {sheets:&sheets},&none);
    // the faces the motion carries within their own planes, which the tracer leaves alone
    let sweep = SweepContacts::read(sk,swept,options.axis_tolerance()).map_err(ConstructError::Seeds)?;
    let faces = super::grazing_faces(&sweep).map_err(ConstructError::Grazing)?;
    let planes: Vec<([f64;3],[f64;3])> = faces.iter().map(|f| (f.origin,f.outward)).collect();
    let caps = super::caps(sk,swept,&sheets,options.sagitta,options.snap(),options.spacing,&planes).map_err(ConstructError::Caps)?;
    observe(Stage::Capped {caps:&caps},&none);
    let mut times: Vec<f64> = sheets.iter().flat_map(|s| s.times.iter().copied()).collect();
    times.sort_by(f64::total_cmp); times.dedup();
    if times.is_empty() { times = sweep.domain().to_vec(); }
    let regions = super::grazing_seeds(&faces,&times,sweep.domain(),options.sagitta,options.spacing).map_err(ConstructError::Grazing)?;
    observe(Stage::Grazed {regions:&regions},&none);
    let mut seeds: Vec<Seed> = sheets.into_iter().map(Seed::Traced).collect();
    seeds.extend(caps.into_iter().map(Seed::Cap));
    seeds.extend(regions.into_iter().map(Seed::Grazing));
    let field = MaterialField::read(sk,swept,options.axis_tolerance()).map_err(ConstructError::Field)?;
    let mut judge = FieldJudge::new(field.clone(),options.judge_tolerance(),options.near_budget,options.far_budget,options.cached_poses);
    let (epsilon,reach) = (options.vertex_tolerance(),options.reach());
    let labelled = super::label_seeds(&mut judge,&seeds,epsilon,reach)?;
    observe(Stage::Labelled {seeds:&seeds,labelled:&labelled},&judge.stats);
    let source_count = seeds.len();
    let (mut mesh,rims) = super::clip_sheets(&mut judge,&seeds,&labelled,epsilon,reach)?;
    check_stage_mesh(&mesh,source_count,"clipped")?;
    observe(Stage::Clipped {mesh:&mesh,rims:&rims},&judge.stats);
    let (distance,snap) = options.crease_merge();
    let (welded,split) = super::merge_creases(&mut mesh,&rims,distance,snap);
    check_stage_mesh(&mesh,source_count,"merged")?;
    observe(Stage::Merged {mesh:&mesh,rims:&rims,welded,split},&judge.stats);
    let (probe,least) = (options.probe_distance(),options.least_probe());
    let (keep,inside,outside) = super::centroid_kept(&mut judge,&mesh,probe,least)?;
    let kept = super::retained(&mesh,&keep);
    check_stage_mesh(&kept,source_count,"kept")?;
    observe(Stage::Kept {before:&mesh,keep:&keep,kept:&kept,inside,outside},&judge.stats);
    let (mesh,replaced) = super::planar_union(&kept,options.coincidence());
    check_stage_mesh(&mesh,source_count,"unioned")?;
    observe(Stage::Unioned {mesh:&mesh,replaced},&judge.stats);
    let (mut mesh,dropped) = super::clip_overlaps(&mesh,options.coverage());
    check_stage_mesh(&mesh,source_count,"uncovered")?;
    observe(Stage::Uncovered {mesh:&mesh,dropped},&judge.stats);
    super::weld(&mut mesh,options.coincidence());
    check_stage_mesh(&mesh,source_count,"weld")?;
    let collapsed = super::collapse_short_edges(&mut mesh,options.shortest_edge());
    check_stage_mesh(&mesh,source_count,"collapse_short_edges")?;
    // a sliver doubled over another source's edge is one thinner than the certificate's least
    // probe distance
    let doubled = super::drop_doubled_slivers(&mut mesh,least);
    check_stage_mesh(&mesh,source_count,"welded")?;
    observe(Stage::Welded {mesh:&mesh,collapsed,doubled},&judge.stats);
    super::split_at_vertices(&mut mesh,options.junction());
    check_stage_mesh(&mesh,source_count,"split")?;
    observe(Stage::Split {mesh:&mesh},&judge.stats);
    // a loop nothing could pair is filled only where the field says the boundary spans it
    let mut closeable = |points: &[[f64;3]],normals: &[[f64;3]]| -> bool {
        matches!(super::loop_span(&mut judge,points,normals,epsilon,reach),Ok(super::Span::Spanned))
    };
    let (pairs,unpaired) = super::rim_zip(&mut mesh,options.spacing,options.junction(),least,&mut closeable);
    check_stage_mesh(&mesh,source_count,"zipped")?;
    observe(Stage::Zipped {mesh:&mesh,pairs,unpaired:&unpaired},&judge.stats);
    let certificate = super::certify(&mut judge,&mesh.vertices,&mesh.triangles,probe,least)?;
    observe(Stage::Certified {mesh:&mesh,certificate:&certificate},&judge.stats);
    let compact = mesh.compact();
    let triangles: Vec<_> = compact.triangles.iter().map(|t| t.map(|v| v as usize)).collect();
    let topology = crate::topology::ClosedShell::from_triangles(compact.vertices.len(),&triangles);
    Ok(BoundaryCandidate {mesh,certificate,stats:judge.stats,unpaired,topology,field})
}
