//! A solid's exact export by this kernel, as every host makes it (phase 5 of
//! docs/rust-kernel-plan.md): the terminal's `solventc` and the browser's worker call the same
//! functions, so a file is one file wherever it is written. A body is built from its CAD recipe,
//! or — with swept cuts of the generating class, admitted — as one sector patterned
//! (`brep::sweep`); its STEP is written and parsed back against it (`step_check`); its STL is meshed
//! within the tolerance's deflection (the measured sag, a bound and not a control), its shells
//! checked, and a swept body's judged against its material field (`agreement`), nothing given back
//! unless it agrees.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::pattern::Built as Patterned;
use super::sweep::Say;
use super::boolean::Op;
use super::topo::Brep;
use crate::model::Sketch;
use crate::solid::export::{AtStage,ExportRefusal,Stage,Tolerance};
use crate::solid::{admission,agreement,cad,contracts,MaterialField};

/// The mesh's deflection with no tolerance given: the export's gross bar (mm).
pub const GROSS: f64 = 0.01;
/// The mesh's angular control (radians).
pub const ANGULAR: f64 = 0.2;

/// A body built exactly: its B-rep, and, built as one sector, the pattern that meshes it and the
/// turn its field agreement reads each probe in.
pub struct Exact { pub solid: Brep,pub pattern: Option<Patterned>,pub indexed: Option<([f64;3],[f64;3],usize)>,
    /// Whether the body cuts sweeps, so its mesh is held to its material field.
    pub swept: bool }

/// Solid `body` of the sketch built by this kernel. A body with swept cuts is admitted to the
/// generating-sweep class first (`admission` where the host made it, else here).
pub fn exact(sk: &Sketch,body: usize,admitted: Option<&admission::Admission>,tolerance: Option<Tolerance>,say: &Say) -> Result<Exact,ExportRefusal> {
    if holds_sweep(sk,body) { return compose(sk,body,tolerance,say) }
    let recipe = cad::recipe_static(sk,body).at(Stage::Blank)?;
    if !recipe.sweeps.is_empty() {
        let made;
        let admission = match admitted {
            Some(a) => a,
            None => { made = admission::admit_body(sk,body,&admission::Options::default())?; (say.mark)(Stage::Admission); &made }
        };
        return Ok(match super::sweep::build(sk,body,&recipe,admission,tolerance,say)? {
            super::sweep::Built::Sector {sector,pattern} => {
                (say.stage)(&format!("the solid by the Rust kernel: {} sectors, {} faces, {} edges",sector.count,pattern.solid.faces.len(),
                    pattern.solid.edges.len()));
                Exact {solid:pattern.solid.clone(),pattern:Some(pattern),indexed:Some((sector.origin,sector.axis,sector.count)),swept:true}
            }
            super::sweep::Built::Whole(solid) => {
                (say.stage)(&format!("the solid by the Rust kernel: {} faces, {} edges",solid.faces.len(),solid.edges.len()));
                Exact {solid,pattern:None,indexed:None,swept:true}
            }
        })
    }
    let started = crate::clock::Instant::now();
    let solid = super::recipe::build(&recipe.recipe).at(Stage::Blank)?;
    let tol = 1e-9*solid.size().max(1.);
    solid.check(10.*tol).map_err(|e| ExportRefusal::at(Stage::Blank,format!("the built boundary is invalid: {e}")))?;
    if let Some(p) = solid.pinches().first() {
        return Err(ExportRefusal::at(Stage::Blank,format!("the solid pinches to a point at [{:.4}, {:.4}, {:.4}] mm (two of its \
            walls touch there without an edge between them): no manifold file describes it",p[0],p[1],p[2])))
    }
    (say.stage)(&format!("built the solid by the Rust kernel: {:.6} mm³, {} faces, {} edges ({:?})",super::props::volume(&solid),
        solid.faces.len(),solid.edges.len(),started.elapsed()));
    Ok(Exact {solid,pattern:None,indexed:None,swept:false})
}

/// Whether `body` is a body whose stock, or a solid put on it or bounding it, holds a sweep: a
/// body built from swept material, not one a sweep is cut from.
fn holds_sweep(sk: &Sketch,body: usize) -> bool {
    let crate::model::SolidDef::Body {stock,on,bound,..} = &sk.solids[body].def else { return false };
    std::iter::once(stock).chain(on).chain(bound).any(|&o| cad::contains_sweep(sk,o as usize))
}

/// A body built from swept material (`holds_sweep`): each operand built exactly — a body with
/// swept cuts by its own construction, the rest by its recipe — and combined by the body rule:
/// the stock, plus what is put on it, minus what cuts it, within what bounds it. A sweep cut from
/// it directly as well is refused: the swept material is named apart, as `fluted` is from `drill`.
fn compose(sk: &Sketch,body: usize,tolerance: Option<Tolerance>,say: &Say) -> Result<Exact,ExportRefusal> {
    let started = crate::clock::Instant::now();
    let built = combine(sk,body,&|o| if cad::contains_sweep(sk,o) { Ok(exact(sk,o,None,tolerance,say)?.solid) } else { built_static(sk,o) })?;
    (say.stage)(&format!("`{}` built from its swept material by this kernel: {:.6} mm³, {} faces ({:?})",sk.solids[body].name,
        super::props::volume(&built),built.faces.len(),started.elapsed()));
    (say.mark)(Stage::Fuse);
    Ok(Exact {solid:built,pattern:None,indexed:None,swept:true})
}

/// A static solid built from its recipe by this kernel.
fn built_static(sk: &Sketch,o: usize) -> Result<Brep,ExportRefusal> {
    super::recipe::build(&cad::recipe_static(sk,o).at(Stage::Blank)?.recipe).at(Stage::Blank)
}

/// The body rule over operands `part` builds (millimetres): the stock, plus what is put on it, minus
/// what cuts it, within what bounds it, by this kernel's Booleans. A sweep cut from it directly is
/// refused: the swept material is named apart, as `fluted` is from `drill`.
fn combine(sk: &Sketch,body: usize,part: &dyn Fn(usize) -> Result<Brep,ExportRefusal>) -> Result<Brep,ExportRefusal> {
    let solid = &sk.solids[body];
    let crate::model::SolidDef::Body {stock,on,through,bound} = &solid.def else { unreachable!("a composed solid is a body") };
    if let Some(&c) = through.iter().find(|&&c| cad::contains_sweep(sk,c as usize)) {
        return Err(ExportRefusal::at(Stage::Blank,format!("`{}` cuts the sweep `{}` from swept material: name the body the sweep is \
            cut from, and build `{}` from it",solid.name,sk.solids[c as usize].name,solid.name)))
    }
    let mut built = part(*stock as usize)?;
    for (operands,op,word) in [(on,Op::Union,"union"),(through,Op::Cut,"cut"),(bound,Op::Common,"bound")] {
        for &o in operands {
            let operand = part(o as usize)?;
            let tol = 1e-9*built.size().max(operand.size());
            let name = &sk.solids[o as usize].name;
            built = super::boolean::boolean(&built,&operand,op,tol)
                .map_err(|e| ExportRefusal::at(Stage::Fuse,format!("`{}`: {word} `{name}`: {e}",solid.name)))?;
            built.check(10.*tol).map_err(|e| ExportRefusal::at(Stage::Fuse,format!("`{}`: {word} `{name}`: {e}",solid.name)))?;
        }
    }
    Ok(built)
}

/// The STEP file of a built solid, parsed back and held to it (every reference, the topology's
/// counts, units, and each face's surface and its numbers), with what the check counted.
pub fn step(exact: &Exact,name: &str,tolerance: Option<Tolerance>,say: &Say) -> Result<String,ExportRefusal> {
    let started = crate::clock::Instant::now();
    let text = super::step::write(&exact.solid,name,tolerance.map_or(1e-4,|t| t.deflection()*0.1)).at(Stage::Step)?;
    let verified = super::step_check::verify(&text,&super::step_check::Solid::of(&exact.solid))
        .map_err(|e| ExportRefusal::at(Stage::Step,format!("the STEP file does not describe the solid: {e}")))?;
    (say.stage)(&format!("the STEP output: {} entities; {} faces, {} edges and {} vertices, each the solid's ({:?})",
        verified.entities,verified.faces,verified.edges,verified.vertices,started.elapsed()));
    (say.mark)(Stage::Step);
    Ok(text)
}

/// The binary STL of a built solid: meshed within the tolerance's deflection (a sector's mesh
/// turned into its copies), no triangle turned against its surface, its shells closed — and a
/// swept body's held to its material field, refused where they disagree.
pub fn stl(sk: &Sketch,body: usize,exact: &Exact,tolerance: Option<Tolerance>,say: &Say) -> Result<Vec<u8>,ExportRefusal> {
    let started = crate::clock::Instant::now();
    let bar = tolerance.map_or(GROSS,|t| t.deflection());
    // meshed within what float32 coordinates leave of the bar, so the written file is within it
    let (lo,hi) = exact.solid.bounds();
    let sag = bar-crate::mesh::f32_rounding_within(lo,hi);
    let m = match &exact.pattern { Some(p) => p.mesh(sag,ANGULAR),None => super::mesh::mesh(&exact.solid,sag,ANGULAR) }.at(Stage::Mesh)?;
    if m.turned > 0 { return Err(ExportRefusal::at(Stage::Mesh,format!("the mesh has {} triangles facing against their surfaces",m.turned))) }
    if m.sag > bar { return Err(ExportRefusal::at(Stage::Mesh,format!("the mesh sags {:.3} µm against {:.3} µm",m.sag*1e3,bar*1e3))) }
    let bytes = crate::mesh::stl_of(&m.triangles(),&sk.solids[body].name);
    let rounding = written_within(&bytes,m.sag,bar)?;
    crate::mesh::stl_shells(&bytes).at(Stage::Stl)?;
    (say.stage)(&format!("the STL output: {} triangles, sagging {:.3} µm at most and moved {:.3} µm at most by float32, against \
        {:.3} µm, its shells checked ({:?})",m.tris.len(),m.sag*1e3,rounding*1e3,bar*1e3,started.elapsed()));
    (say.mark)(Stage::Stl);
    if exact.swept { field_agreement(sk,body,&bytes,tolerance,exact.indexed,say)?; }
    Ok(bytes)
}

/// How far float32 encoding may have moved the written STL (`mesh::stl_rounding`), refused where
/// that and the mesh's `sag` together pass the STL's share of the tolerance (`bar`): the bar is
/// about the written surface, and binary STL cannot place a vertex far from the origin finely.
pub fn written_within(bytes: &[u8],sag: f64,bar: f64) -> Result<f64,ExportRefusal> {
    let rounding = crate::mesh::stl_rounding(bytes).at(Stage::Stl)?;
    if sag+rounding > bar {
        return Err(ExportRefusal::at(Stage::Stl,format!("float32 STL coordinates may move the surface {:.3} µm at this \
            position, which with the mesh's sag of {:.3} µm passes {:.3} µm; move the solid nearer the origin or ask a coarser \
            tolerance",rounding*1e3,sag*1e3,bar*1e3)))
    }
    Ok(rounding)
}

/// A swept body's STL held to its material field: no cluster of microscopic triangles, and every
/// probe a little off each side of the mesh read on the side the mesh says (an indexed body's turned
/// into one sector and read by the cuts that reach it).
pub fn field_agreement(sk: &Sketch,body: usize,stl: &[u8],tolerance: Option<Tolerance>,indexed: Option<([f64;3],[f64;3],usize)>,say: &Say)
    -> Result<(),ExportRefusal> {
    let mesh = |message: String| ExportRefusal::at(Stage::Mesh,message);
    let refused = |message: String| ExportRefusal::at(Stage::Agreement,message);
    let scale = cad::millimetres(sk).map_err(mesh)?;
    let started = crate::clock::Instant::now();
    let (vertices,triangles) = agreement::stl_triangles(stl,scale).map_err(mesh)?;
    let verdict = match tolerance {
        None => {
            let tiny = contracts::tiny_triangles(&vertices,&triangles,scale);
            (say.stage)(&format!("mesh: {} of {} triangles under {} mm²",tiny.count,tiny.total,contracts::TINY_AREA));
            tiny.verdict()
        }
        Some(t) => {
            let tiny = contracts::tiny_triangles_under(&vertices,&triangles,scale,(t.deflection()/10.).dpowi(2));
            (say.stage)(&format!("mesh: {} of {} triangles under {:.1e} mm², at most {} in a millimetre cube",tiny.count,tiny.total,
                tiny.under,tiny.densest));
            tiny.clustered()
        }
    };
    verdict.map_err(mesh)?;
    (say.mark)(Stage::Mesh);
    let field = MaterialField::read(sk,body,cad::AXIS_TOLERANCE).map_err(refused)?;
    let [offset,confirm,value_tolerance] = tolerance.map_or([0.1,0.025,0.02],|t| t.probe());
    let options = agreement::Options {offset:offset/scale,confirm:confirm/scale,value_tolerance:value_tolerance/scale,..Default::default()};
    let (report,how) = match indexed {
        Some((origin,axis,count)) => {
            let at = agreement::Indexed {origin:origin.map(|x| x/scale),axis,count};
            let (report,folding) = agreement::of_triangles_indexed(&vertices,&triangles,&field,at,cad::POSE_CACHE,&options).map_err(refused)?;
            (report,if folding.alike {
                format!(", each turned into one sector and read there by {} of its {} cuts (the rest proved clear of {} boxes about it; \
                    {} probes outside them read where they stand)",folding.kept,folding.operands,folding.cells,folding.whole)
            } else { ", each where it stands (the field did not read alike turned by a pitch)".into() })
        }
        None => (agreement::of_triangles_parallel(&vertices,&triangles,&field,cad::POSE_CACHE,&options).map_err(refused)?,String::new()),
    };
    let off = if tolerance.is_some() { format!("{:.4}",options.offset*scale) } else { format!("{:.2}",options.offset*scale) };
    (say.stage)(&format!("field agreement: {} of {} triangles probed {off} mm off each side{how}, {} probes unresolved, \
        {} withdrawn beside another face, {} disagree ({:?})",report.probed_triangles,report.triangles,
        report.unresolved,report.withdrawn,report.disagreements.len(),started.elapsed()));
    if report.agrees() { (say.mark)(Stage::Agreement); return Ok(()) }
    Err(ExportRefusal {stage:Stage::Agreement,condition:None,witness:report.disagreements.first().map(|d| d.point),
        message:format!("`{}`: the exported surface disagrees with the material field at {} of {} probes; nothing was written",
        sk.solids[body].name,report.disagreements.len(),report.probes)})
}

/// A swept body's exact surface as the page draws it: its patterned (or whole) B-rep meshed at the
/// solid's own mesh unit, in model units and world coordinates, each triangle with the B-rep face
/// it is of (`of`, the sector's for a pattern, so every copy of a face is one face), whether each
/// face is curved, and the solid's exact volume (model units cubed).
#[derive(Clone,Debug,Default)]
pub struct Display { pub vertices: Vec<[f64;3]>,pub triangles: Vec<[u32;3]>,pub of: Vec<u32>,pub smooth: Vec<bool>,pub volume: f64 }

/// Where a swept body's exact build stands for its display, a stage at a time (`Builder::step`).
enum State {
    Admit,
    Prepare(admission::Admission),
    Sheets(super::sweep::Prepared,Vec<super::sweep::sheet::Fitted>),
    Plan(super::sweep::Prepared,Vec<super::sweep::sheet::Fitted>),
    Cut(super::sweep::Prepared,Vec<super::sweep::sheet::Fitted>,super::sweep::Plan),
    Finish(super::sweep::Cut),
    Compose(super::sweep::Built),
    Mesh(super::sweep::Built),
    Done(Display),
    Failed,
}

/// **A swept body's exact surface, built a stage at a time** — admitted, its blank, each sweep's
/// sheet, its sector's premises, the cut, the pattern, the mesh — so a host that cannot hear from
/// a call while it runs (a page's worker: the core's module has no imports) can say between stages
/// what it is doing, and stop between them for a newer drawing. The stages are `export::exact`'s,
/// without a file written or the field's agreement asked: what it builds is for looking at.
pub struct Builder { sk: Sketch,body: usize,recipe: cad::StaticRecipe,state: State,lines: std::sync::Mutex<Vec<String>>,done: usize,
    /// The body built from the swept body, when `body` is that (`holds_sweep`): the stages build
    /// the swept body, and a last one combines it with the rest by the body rule.
    outer: Option<usize> }

impl Builder {
    /// The build of swept solid `body` of `sk` (a copy of the sketch is kept), or why it has none:
    /// a solid with no swept cut is the static path's.
    pub fn new(sk: &Sketch,body: usize) -> Result<Builder,ExportRefusal> {
        if body >= sk.solids.len() { return Err(ExportRefusal::at(Stage::Blank,"no such solid in this drawing")) }
        // a body built from swept material: its one operand holding sweeps built by the stages
        let (body,outer) = if holds_sweep(sk,body) {
            let crate::model::SolidDef::Body {stock,on,bound,..} = &sk.solids[body].def else { unreachable!("a body") };
            let swept: Vec<usize> = std::iter::once(stock).chain(on).chain(bound).map(|&o| o as usize).filter(|&o| cad::contains_sweep(sk,o)).collect();
            let [inner] = swept[..] else {
                return Err(ExportRefusal::at(Stage::Blank,format!("`{}` is built from {} swept bodies; its display is built from one",
                    sk.solids[body].name,swept.len())))
            };
            (inner,Some(body))
        } else { (body,None) };
        let recipe = cad::recipe_static(sk,body).at(Stage::Blank)?;
        if recipe.sweeps.is_empty() {
            return Err(ExportRefusal::at(Stage::Blank,format!("`{}` cuts no sweep",sk.solids[body].name)))
        }
        Ok(Builder {sk:sk.clone(),body,recipe,state:State::Admit,lines:Default::default(),done:0,outer})
    }
    /// Run the next stage: `Ok(true)` once the surface is built, or the refusal that stopped it.
    pub fn step(&mut self) -> Result<bool,ExportRefusal> {
        let lines = &self.lines;
        let say = Say {stage:&|line: &str| lines.lock().unwrap_or_else(|e| e.into_inner()).push(line.to_string()),mark:&|_| {}};
        let (sk,body,recipe) = (&self.sk,self.body,&self.recipe);
        let state = std::mem::replace(&mut self.state,State::Failed);
        let next = (|| -> Result<State,ExportRefusal> { Ok(match state {
            State::Admit => State::Prepare(admission::admit_body(sk,body,&DISPLAY_ADMISSION)?),
            State::Prepare(admitted) => State::Sheets(super::sweep::prepare(sk,body,recipe,&admitted,&say)?,Vec::new()),
            State::Sheets(prepared,mut sheets) => {
                sheets.push(super::sweep::sheet(&prepared,sheets.len(),None,&say)?);
                if sheets.len() == prepared.sweeps() { State::Plan(prepared,sheets) } else { State::Sheets(prepared,sheets) }
            }
            State::Plan(prepared,sheets) => { let plan = super::sweep::plan(sk,body,recipe,&prepared,&sheets,&say); State::Cut(prepared,sheets,plan) }
            State::Cut(prepared,sheets,plan) => State::Finish(super::sweep::cut(sk,body,recipe,&prepared,&sheets,plan,&say)?),
            State::Finish(cut) => {
                let built = super::sweep::finish(cut,&say)?;
                if self.outer.is_some() { State::Compose(built) } else { State::Mesh(built) }
            }
            State::Compose(built) => {
                let outer = self.outer.expect("a composed build has its body");
                let inner = std::cell::RefCell::new(Some(match built { super::sweep::Built::Sector {pattern,..} => pattern.solid,
                    super::sweep::Built::Whole(s) => s }));
                let solid = combine(sk,outer,&|o| if o == body {
                    inner.borrow_mut().take().ok_or_else(|| ExportRefusal::at(Stage::Fuse,"the swept body is used twice")) } else { built_static(sk,o) })?;
                (say.stage)(&format!("`{}` combined from its swept material",sk.solids[outer].name));
                State::Mesh(super::sweep::Built::Whole(solid))
            }
            State::Mesh(built) => State::Done(display(sk,&built)?),
            done @ State::Done(_) => done,
            State::Failed => return Err(ExportRefusal::at(Stage::Blank,"the build has failed already")),
        }) })();
        match next {
            Ok(state) => { self.done += 1; self.state = state; Ok(matches!(self.state,State::Done(_))) }
            Err(refusal) => Err(refusal),
        }
    }
    /// What the next stage does, in a page's words.
    pub fn doing(&self) -> String {
        let name = |k: usize,p: &super::sweep::Prepared| format!("fitting the sheet of `{}` ({} of {})",p.sweep_name(&self.sk,k),k+1,p.sweeps());
        match &self.state {
            State::Admit => "admitting its sweeps to the generating class".into(),
            State::Prepare(_) => "building the blank".into(),
            State::Sheets(p,sheets) => name(sheets.len(),p),
            State::Plan(..) => "laying out the sector".into(),
            State::Cut(_,_,super::sweep::Plan::Sector(_)) => "cutting the sector by its sheets".into(),
            State::Cut(_,_,super::sweep::Plan::Whole(_)) => "cutting the blank by its sheets".into(),
            State::Finish(_) => "turning the sector round".into(),
            State::Compose(_) => "adding the rest of the body".into(),
            State::Mesh(_) => "meshing the exact surface".into(),
            State::Done(_) => "built".into(),
            State::Failed => "failed".into(),
        }
    }
    /// The stages run, and their number in all: admission, the blank, a sheet a distinct sweep, the
    /// plan, the cut, the pattern and the mesh.
    pub fn stages(&self) -> (usize,usize) {
        let mut sweeps: Vec<usize> = self.recipe.sweeps.iter().map(|s| s.swept).collect();
        sweeps.sort(); sweeps.dedup();
        (self.done,6+sweeps.len()+usize::from(self.outer.is_some()))
    }
    /// The last thing a stage said (its own words: what it built, and in what time).
    pub fn said(&self) -> Option<String> { self.lines.lock().unwrap_or_else(|e| e.into_inner()).last().cloned() }
    /// The surface, once built.
    pub fn display(&self) -> Option<&Display> { match &self.state { State::Done(d) => Some(d),_ => None } }
}

/// The admission a display asks: a quarter of the export's samples each way, on both its grids. A
/// display is looked at, not made: a design the export's sampling would refuse is still refused
/// there, and the build's own stages refuse what they cannot construct. On a gear member it is a
/// second in the browser's core where the export's took six.
const DISPLAY_ADMISSION: admission::Options = admission::Options {rows:25,columns:100,coarse_rows:6,coarse_columns:360,
    axis_tolerance:cad::AXIS_TOLERANCE,margin:1e-6,root_tolerance:1e-9,least_factor:1e-3};

/// How far a display's mesh may sag, as a fraction of the solid's diagonal (a gear's 0.2 mm), and
/// how far its chords and facets may turn (radians): a gear's half-millimetre fillets are what an
/// export's 0.2 radians cut finest, and what a screen shows least. A gear member is then 17–26
/// thousand triangles, as many as its field's preview: at 0.05 mm and 0.5 rad it was 39–63
/// thousand, and a page panning and zooming the pair could not keep up.
const DISPLAY_SAG: f64 = 2e-3;
const DISPLAY_ANGULAR: f64 = 0.8;

/// A built swept body meshed for display, within `DISPLAY_SAG` of its size.
fn display(sk: &Sketch,built: &super::sweep::Built) -> Result<Display,ExportRefusal> {
    let mm = cad::millimetres(sk).at(Stage::Mesh)?;
    let size = match built { super::sweep::Built::Sector {pattern,..} => pattern.solid.size(),super::sweep::Built::Whole(solid) => solid.size() };
    let bar = DISPLAY_SAG*size;
    // (a pattern's volume is its sector's times its copies, which are turns of it: measured whole,
    // a gear's 291 faces took seconds where its sector's 11 take a few hundredths)
    let (m,faces,volume) = match built {
        super::sweep::Built::Sector {sector,pattern} => (pattern.mesh(bar,DISPLAY_ANGULAR).at(Stage::Mesh)?,&pattern.sector().faces,
            super::props::volume(&sector.piece)*sector.count as f64),
        super::sweep::Built::Whole(solid) => (super::mesh::mesh(solid,bar,DISPLAY_ANGULAR).at(Stage::Mesh)?,&solid.faces,
            super::props::volume(solid)),
    };
    Ok(Display {
        vertices: m.pts.iter().map(|p| p.map(|x| x/mm)).collect(),
        triangles: m.tris.clone(),
        of: m.of.clone(),
        smooth: faces.iter().map(|f| !matches!(f.surface,super::geom::Surface::Plane(_))).collect(),
        volume: volume/(mm*mm*mm),
    })
}
