//! The construction end to end: the sheets traced, the caps cut, every vertex judged, the sheets
//! clipped at their creases and the rims merged, the triangles kept by their centroids, unioned
//! in their planes, cleared of double coverage, welded, split at their T-junctions, zipped, and
//! every triangle certified. Every tolerance a stage uses comes from `SweptBoundaryOptions`,
//! and each stage is shown to an observer as it finishes, with the judge's counts so far.
use super::{Cap,Certificate,FieldJudge,JudgeError,KeptMesh,Labelled,QueryStats,Rim,Seed,SweptBoundaryOptions};
use crate::model::Sketch;
use crate::solid::{MaterialField,SweepPatch};

/// A stage just finished: what it made, and what it counted.
pub enum Stage<'a> {
    /// The traced sheets.
    Seeded { sheets: &'a [SweepPatch] },
    /// The two caps cut from the tool at the ends of the roll.
    Capped { caps: &'a [Cap] },
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

/// A finished construction: the mesh, its certificate, and what the judge asked of the field.
pub struct SweptBoundary { pub mesh: KeptMesh, pub certificate: Certificate, pub stats: QueryStats }

/// Why a construction stopped.
#[derive(Clone,Debug,PartialEq)]
pub enum ConstructError {
    /// The tracer declined.
    Seeds(String),
    /// A cap could not be cut from the tool.
    Caps(String),
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
    let (_,sheets) = super::seeds(sk,swept,options.spacing,options.sagitta,progress).map_err(ConstructError::Seeds)?;
    construct_from(sk,swept,options,sheets,observe)
}

/// `construct` from sheets already traced (the tracer's, or a test's own: moved, thinned,
/// one withheld), from the caps on.
pub fn construct_from(sk: &Sketch,swept: usize,options: &SweptBoundaryOptions,sheets: Vec<SweepPatch>,
    observe: &mut dyn FnMut(Stage<'_>,&QueryStats)) -> Result<SweptBoundary,ConstructError> {
    let none = QueryStats::default();
    observe(Stage::Seeded {sheets:&sheets},&none);
    let caps = super::caps(sk,swept,&sheets,options.sagitta,options.snap(),options.spacing).map_err(ConstructError::Caps)?;
    observe(Stage::Capped {caps:&caps},&none);
    let mut seeds: Vec<Seed> = sheets.into_iter().map(Seed::Traced).collect();
    seeds.extend(caps.into_iter().map(Seed::Cap));
    let field = MaterialField::read(sk,swept,options.axis_tolerance()).map_err(ConstructError::Field)?;
    let mut judge = FieldJudge::new(field,options.judge_tolerance(),options.near_budget,options.far_budget,options.cached_poses);
    let (epsilon,reach) = (options.vertex_tolerance(),options.reach());
    let labelled = super::label_seeds(&mut judge,&seeds,epsilon,reach)?;
    observe(Stage::Labelled {seeds:&seeds,labelled:&labelled},&judge.stats);
    let (mut mesh,rims) = super::clip_sheets(&mut judge,&seeds,&labelled,epsilon,reach)?;
    observe(Stage::Clipped {mesh:&mesh,rims:&rims},&judge.stats);
    let (distance,snap) = options.crease_merge();
    let (welded,split) = super::merge_creases(&mut mesh,&rims,distance,snap);
    observe(Stage::Merged {mesh:&mesh,rims:&rims,welded,split},&judge.stats);
    let (probe,least) = (options.probe_distance(),options.least_probe());
    let (keep,inside,outside) = super::centroid_kept(&mut judge,&mesh,probe,least)?;
    let kept = super::retained(&mesh,&keep);
    observe(Stage::Kept {before:&mesh,keep:&keep,kept:&kept,inside,outside},&judge.stats);
    let (mesh,replaced) = super::planar_union(&kept,options.coincidence());
    observe(Stage::Unioned {mesh:&mesh,replaced},&judge.stats);
    let (mut mesh,dropped) = super::clip_overlaps(&mesh,options.coverage());
    observe(Stage::Uncovered {mesh:&mesh,dropped},&judge.stats);
    super::weld(&mut mesh,options.coincidence());
    let collapsed = super::collapse_short_edges(&mut mesh,options.shortest_edge());
    // a sliver doubled over another source's edge is one thinner than the certificate's least
    // probe distance
    let doubled = super::drop_doubled_slivers(&mut mesh,least);
    observe(Stage::Welded {mesh:&mesh,collapsed,doubled},&judge.stats);
    super::split_at_vertices(&mut mesh,options.junction());
    observe(Stage::Split {mesh:&mesh},&judge.stats);
    let (pairs,unpaired) = super::rim_zip(&mut mesh,options.spacing,options.junction());
    observe(Stage::Zipped {mesh:&mesh,pairs,unpaired:&unpaired},&judge.stats);
    let certificate = super::certify(&mut judge,&mesh.vertices,&mesh.triangles,probe,least)?;
    observe(Stage::Certified {mesh:&mesh,certificate:&certificate},&judge.stats);
    Ok(SweptBoundary {mesh,certificate,stats:judge.stats})
}
