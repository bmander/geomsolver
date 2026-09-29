//! A body with continuous swept cuts, by arrangement and classification.
//!
//! The static remainder of the body is constructed as usual and is the blank.
//! For each distinct sweep, the native cutter is sectioned by meridian half-planes
//! of its reference axis; each section loop is the profile, and a row of the
//! candidate sheet is a section point carrying its face's outward normal, whose
//! contact position under the declared motion is exact. Sharp convex corners of
//! the section contribute a fan of normals at zero position change, the sweep of
//! the cutter's edge. Rows are spaced in augmented arc length (position length
//! plus turning across corners), so a flat bottom narrowing into a crease keeps a
//! smooth row structure. The sheet is placed at every pose the body cuts it at,
//! the kernel splits the blank by all of them, every cell is judged by the
//! declared material field at points a measured distance inside it, and the
//! material cells are united. Nothing here trims, selects visibility or knows a
//! cutter's shape; an unresolved or mixed cell refuses the build. The cutter's sections are
//! `sections`', and the fit contract a sheet passes before it is placed is `fit`'s.
use super::*;
use super::kernel::{Cell,Patterned};
use gcs_core::{interval::{Interval,minimum::Options},model::{Sketch,SolidDef},motion::Family,
    solid::{admission::Admission,cad,contracts,MaterialEvaluator,MaterialField,ProbeState,SweepContacts}};
use gcs_core::solid::contact_trace::{Band,Layout,Sample,Station,TraceError,Tracer,Withheld,marked};
pub(crate) use gcs_core::solid::contact_trace::{Inside,Sheet};
use gcs_core::solid::contact_trace::Rows;
use gcs_core::solid::export::{AtStage,ExportRefusal,Stage,Tolerance};
use gcs_core::space::{sub,dot,cross,norm,scale as scaled,distance};
use std::f64::consts::TAU;

#[path="sweep_boundary/sections.rs"]
mod sections;
#[path="sweep_boundary/fit.rs"]
mod fit;
#[path="sweep_boundary/sector.rs"]
mod sector;
use fit::{Judged,judged};

/// `SOLVENT_TRACE_DEBUG=1` prints where a station's contact curve ends, leaves the root window or
/// runs away, the edges of a section that does not close, and each withheld contact a fit held to
/// a tolerance misses: the instruments that located the faults of the traced construction, kept
/// for the next one.
fn tracing() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var_os("SOLVENT_TRACE_DEBUG").is_some())
}

/// Classify every cell of a partition against the material field. A cell is
/// judged at several interior points with measured boundary distances; every
/// point needs a ball certificate within that distance and all must agree, so
/// an unresolved point or a cell that a leaking sheet failed to separate
/// refuses the build instead of guessing.
pub(crate) fn classify(session: &Session,partition: c_int,material: &mut MaterialEvaluator)
    -> Result<(Vec<Cell>,Vec<Cell>),String> {
    let (mut kept,mut removed) = (Vec::new(),Vec::new());
    let (mut sampling,mut probing,mut probes) = (0.,0.,0);
    // Each cell's volume was measured when the partition was validated; its point is the
    // deepest interior sample measured here, every cell's on its own core.
    let solids = session.solids(partition)?;
    let clock = std::time::Instant::now();
    let sampled = session.samples_of(&solids,4,12)?;
    sampling += clock.elapsed().as_secs_f64();
    for (solid,samples) in solids.into_iter().zip(sampled) {
        let volume = session.volume(solid)?;
        if samples.is_empty() { return Err(format!("a cell of volume {volume} has no interior sample")); }
        let cell = Cell {solid,point:samples[0].0,volume};
        let mut verdict = None;
        let deepest = samples[0];
        for (point,boundary) in samples {
            let distance = (boundary*0.5).min(0.05);
            if distance <= 1e-4 { continue; }
            let clock = std::time::Instant::now();
            probes += 1;
            let probe = material.probe(point.map(|x| Interval::point(x).unwrap()),[1.,0.,0.],distance,
                Options {value_tolerance:distance/4.,max_evaluations:40000}).map_err(|e| format!("{e:?}"))?;
            probing += clock.elapsed().as_secs_f64();
            let inside = match probe.state {
                ProbeState::InteriorBall => true,
                ProbeState::ExteriorBall => false,
                state => return Err(format!("the material at {point:?} ({boundary:.3} mm from a cell boundary) is {state:?}")),
            };
            match verdict {
                None => verdict = Some((inside,point)),
                Some((previous,at)) if previous != inside => return Err(format!(
                    "a cell of volume {} reads both material and removed: a sheet did not separate it \
                    ({} at {:?}, {} at {:?})",cell.volume,if previous { "material" } else { "removed" },
                    at.map(|x| (x*1e4).round()/1e4),if inside { "material" } else { "removed" },
                    point.map(|x| (x*1e4).round()/1e4))),
                _ => {}
            }
        }
        match verdict {
            Some((true,_)) => kept.push(cell),
            Some((false,_)) => removed.push(cell),
            None => return Err(format!("a cell of volume {} has no sample clear of its boundary (the deepest {:.1e} mm in, at {:?})",
                cell.volume,deepest.1,deepest.0.map(|x| (x*1e4).round()/1e4))),
        }
    }
    stage(&format!("classification: interior samples {sampling:.1} s, {probes} field probes {probing:.1} s"));
    Ok((kept,removed))
}

/// A sheet grid fitted as a native face and judged: the face, the grid, and its withheld error.
pub(crate) type Fitted = (c_int,Sheet,f64);

/// How many times a sheet's grid is refined toward its tolerance, and the most nodes it may
/// have each way (the kernel interpolates at most 512): past either it is refused.
const MOST_REFINEMENTS: usize = 4;
const MOST_ROWS: usize = 480;
const MOST_COLUMNS: usize = 400;

/// The candidate sheet grid of one swept solid whose contacts enter the blank
/// `inside` describes, widened until its boundary lies outside: the core traces
/// (`solid::contact_trace`), the native cutter's sections are what it traces. Each row
/// placement in turn is handed to `fitted`; the first it accepts is the sheet, and a sheet it
/// refuses at the withheld contacts (a fit that misses or folds) passes to the next placement.
/// Held to a `tolerance`, a sheet also withholds the middles of its cells' sides, and one that
/// misses is refined where it misses (`contact_trace::marked`, `Grid::refined`) and fitted again,
/// until it fits or its refinement budget is spent.
fn swept_sheets(session: &Session,sk: &Sketch,swept: usize,inside: Inside,placements: &[Rows],
    tolerance: Option<Tolerance>,fitted: &mut dyn FnMut(&Sheet) -> Result<Judged,ExportRefusal>) -> Result<Fitted,ExportRefusal> {
    let SolidDef::Swept {source,..} = &sk.solids[swept].def else {
        return Err(ExportRefusal::at(Stage::Reach,format!("`{}` is not a continuous sweep",sk.solids[swept].name)));
    };
    let scale = cad::millimetres(sk).at(Stage::Reach)?;
    let name = &sk.solids[swept].name;
    let cutter = session.cutter(sk,*source as usize).at(Stage::Reach)?;
    let sweep = SweepContacts::read(sk,swept,cad::AXIS_TOLERANCE).at(Stage::Reach)?;
    let tracer = Tracer {sweep:&sweep,scale,inside,debug:tracing()};
    let started = std::time::Instant::now();
    let reach = session.reach(&cutter,&tracer).at(Stage::Reach)?;
    stage(&format!("`{name}`: contacts reach the blank over {:.1} degrees of stations and {} profile faces ({:?}: \
        sections {:.1} s, contacts {:.1} s, {} blank queries {:.1} s)",
        (reach.stations[1]-reach.stations[0]).to_degrees(),reach.faces,started.elapsed(),
        reach.spent[0],reach.spent[1],reach.queried,reach.spent[2]));
    mark(Stage::Reach);
    // A station: the section loop carrying the reach's walk, and the window between its anchors.
    let station_at = |angle: f64| -> Result<Station,TraceError> {
        let loops = session.profile(&cutter,angle)?;
        let profile = loops.into_iter().find(|l| l.index_of(reach.start.face).is_some() && l.index_of(reach.end.face).is_some())
            .ok_or("no section loop carries the profile at a station of the band")?;
        let window = Session::range(&profile,&reach,0.)?;
        let cutter = &cutter;
        Ok(Station {length:profile.augmented_length(),window,sample:Box::new(move |s| session.sample(cutter,&profile,s))})
    };
    let band = Band {stations:reach.stations,radius:reach.radius};
    let widened = |placement: Rows| -> Result<(Layout,Sheet),String> {
        // The margins carry the sheet's edge out of the blank; then the grid must be one chart.
        let (mut margin,mut station_margin) = (1.,(reach.stations[1]-reach.stations[0])*0.15);
        for _ in 0..6 {
            let layout = tracer.layout(&station_at,band,margin,station_margin,placement)?;
            let sheet = layout.sheet(&layout.grid,Withheld::Centres)?;
            let edges = [(0..sheet.columns).map(|c| sheet.points[c]).collect::<Vec<_>>(),
                (0..sheet.columns).map(|c| sheet.points[(sheet.rows-1)*sheet.columns+c]).collect(),
                (0..sheet.rows).map(|r| sheet.points[r*sheet.columns]).collect(),
                (0..sheet.rows).map(|r| sheet.points[r*sheet.columns+sheet.columns-1]).collect()];
            let within: Vec<usize> = edges.iter().map(|e| inside(e).map(|v| v.iter().filter(|b| **b).count())).collect::<Result<_,_>>()?;
            if within.iter().any(|&n| n > 0) {
                // An edge point in the blank at a time within the declared roll is where the sheet is
                // too small, and widening helps. One at a time outside it is the sheet, extended past
                // the roll so that it is rectangular, coming back into the blank: the cap at the roll's
                // limit is clear of the blank (admission's E1), so the true boundary is not there, and
                // widening only carries the extension further. That needs a sheet trimmed at the roll's
                // limits in its parameter space, which this construction does not build.
                let declared = sweep.domain();
                let mut late = None;
                let mut early = 0;
                for r in 0..sheet.rows { for c in 0..sheet.columns {
                    if !(r == 0 || r+1 == sheet.rows || c == 0 || c+1 == sheet.columns) { continue }
                    let k = r*sheet.columns+c;
                    if !inside(&[sheet.points[k]])?[0] { continue }
                    let t = sheet.times[k];
                    if t < declared[0] || t > declared[1] { late.get_or_insert((t,sheet.points[k])); } else { early += 1; }
                } }
                if early == 0 { if let Some((t,p)) = late {
                    return Err(format!("`{name}`: the sheet, extended past the declared roll, comes back into the blank at {:?} \
                        (time {:.1} degrees, the roll being {:.1} to {:.1}); it needs trimming at the roll's limits",
                        p.map(|x| (x*1e4).round()/1e4),t.to_degrees(),declared[0].to_degrees(),declared[1].to_degrees()));
                } }
                stage(&format!("`{name}`: the sheet's edge is in the blank (first row {}, last row {}, first column {}, last column {} \
                    points); widening its margins",within[0],within[1],within[2],within[3]));
                margin *= 1.6; station_margin *= 1.6; continue;
            }
            if let Some(fault) = sheet.chart_fault(inside)? {
                return Err(format!("`{name}`: the sheet is not one regular chart: {fault}"));
            }
            stage(&format!("`{name}`: sheet {}x{}, one chart ({:?})",sheet.rows,sheet.columns,started.elapsed()));
            return Ok((layout,sheet));
        }
        Err(format!("`{name}`: the candidate sheet cannot be widened out of the blank"))
    };
    let mut refused: Option<ExportRefusal> = None;
    for &placement in placements {
        let by = match placement { Rows::Walk => "walk length", Rows::Length => "length in space" };
        if let Some(refusal) = &refused {
            stage(&format!("{}; placing the sheet's rows by {by} instead",refusal.message));
        }
        let (layout,first) = widened(placement).at(Stage::Sheet)?;
        mark(Stage::Sheet);
        if tolerance.is_none() {
            match fitted(&first) {
                Ok(Judged::Fits(face,error)) => return Ok((face,first,error)),
                Ok(Judged::Misses {refusal,..}) => refused = Some(refusal),
                Err(refusal) if refusal.stage == Stage::Withheld => refused = Some(refusal),
                Err(refusal) => return Err(refusal),
            }
            continue;
        }
        // Held to a tolerance: refined where it misses until it fits, or refused.
        let mut grid = layout.grid.clone();
        for round in 0.. {
            let clock = std::time::Instant::now();
            let sheet = layout.sheet(&grid,Withheld::Sides).map_err(String::from).at(Stage::Sheet)?;
            stage(&format!("`{name}`: sheet {}x{} and {} withheld contacts read ({:?})",sheet.rows,sheet.columns,
                sheet.withheld.len(),clock.elapsed()));
            if let Some(fault) = sheet.chart_fault(inside).at(Stage::Sheet)? {
                return Err(ExportRefusal::at(Stage::Sheet,format!("`{name}`: the refined sheet is not one regular chart: {fault}")));
            }
            let (over,mut refusal) = match fitted(&sheet) {
                Ok(Judged::Fits(face,error)) => return Ok((face,sheet,error)),
                Ok(Judged::Misses {over,refusal}) => (over,refusal),
                Err(refusal) if refusal.stage == Stage::Withheld => { refused = Some(refusal); break }
                Err(refusal) => return Err(refusal),
            };
            let (rows,columns) = marked(&sheet.sites,&over,sheet.rows,sheet.columns);
            let next = grid.refined(&rows,&columns);
            if round >= MOST_REFINEMENTS || next.rows.len() > MOST_ROWS || next.columns.len() > MOST_COLUMNS {
                refusal.message = format!("{} with rows by {by}, after {round} refinements to {}x{} nodes (at most {MOST_REFINEMENTS} \
                    refinements and {MOST_ROWS}x{MOST_COLUMNS} nodes)",refusal.message,sheet.rows,sheet.columns);
                refused = Some(refusal);
                break;
            }
            stage(&format!("`{name}`: refining the sheet where it misses: {} of {} row intervals and {} of {} column intervals \
                split, {}x{} nodes",rows.iter().filter(|m| **m).count(),rows.len(),columns.iter().filter(|m| **m).count(),
                columns.len(),next.rows.len(),next.columns.len()));
            grid = next;
        }
    }
    Err(refused.unwrap_or_else(|| ExportRefusal::at(Stage::Sheet,format!("`{name}`: no row placement was offered"))))
}

/// The candidate sheet of one swept solid against a native blank: the roll must
/// carry the cutter clear of the blank at both limits (no caps on this path),
/// and the grid is fitted as a native face with its withheld contact error.
/// Where contacts reach the blank is asked of `field`, the same blank as the
/// core's analytic field, not of the kernel: a point there is microseconds where
/// the kernel's classifier took 30 ms, and this question only sizes the sheet.
pub(crate) fn swept_sheet(session: &Session,sk: &Sketch,swept: usize,blank: c_int,field: &gcs_core::solid::SpatialField,
    tolerance: Option<Tolerance>) -> Result<Fitted,ExportRefusal> {
    let name = &sk.solids[swept].name;
    let scale = cad::millimetres(sk).at(Stage::Clearance)?;
    let clear = || -> Result<(),String> {
        let SolidDef::Swept {source,motion,from,to} = &sk.solids[swept].def else {
            return Err(format!("`{name}` is not a continuous sweep"));
        };
        let cutter = session.cutter(sk,*source as usize)?;
        let family = Family::read(sk,*motion as usize)?;
        let started = std::time::Instant::now();
        for (label,limit) in [("start",from.value),("end",to.value)] {
            let placed = session.place(cutter.solid,family.at(limit)?,scale)?;
            let overlap = session.common_volume(placed,blank)?;
            if overlap > 0. {
                return Err(format!("`{name}`: the declared roll leaves the cutter inside the blank at its {label} \
                    ({:.1} degrees, {overlap:.3} mm³ overlap); declare a roll that carries it clear",limit.to_degrees()));
            }
        }
        stage(&format!("`{name}`: the roll carries the cutter clear of the blank at both limits ({:?})",started.elapsed()));
        Ok(())
    };
    clear().at(Stage::Clearance)?;
    mark(Stage::Clearance);
    let inside = |points: &[[f64;3]]| Ok(points.iter().map(|p| field.value(p.map(|x| x/scale)) < 0.).collect());
    let near = |p: [f64;3]| field.value(p.map(|x| x/scale))*scale;
    // Rows by walk length first, the placement the bevel pair and the pinion were recorded with;
    // by length where that fit misses or folds.
    swept_sheets(session,sk,swept,&inside,&[Rows::Walk,Rows::Length],tolerance,
        &mut |sheet| judged(session,name,sheet,scale,&near,tolerance))
}

/// How a body with swept cuts is built: as one sector patterned round its indexing axis where the
/// premise holds (`sector`), or whole, split by every placement's sheet at once.
#[derive(Clone,Copy,Debug,PartialEq)]
pub(crate) enum Construction { Sector,Whole }

/// A body constructed natively: the solid handle in its session, how it was built, and — built as
/// one sector patterned — the sector it was patterned from.
pub(crate) struct Built {
    pub solid: c_int,
    /// Read by the tests that build one body both ways.
    #[cfg_attr(not(test),allow(dead_code))]
    pub how: Construction,
    pub sector: Option<Patterned>,
}

/// Construct a body whose cuts include continuous sweeps, which only its admission to the
/// generating-sweep class allows: as `asked` where it applies, whole otherwise.
pub(crate) fn construct_swept_body(session: &Session,sk: &Sketch,body: usize,recipe: &cad::StaticRecipe,
    admission: &Admission,tolerance: Option<Tolerance>,asked: Construction) -> Result<Built,ExportRefusal> {
    if admission.body() != body {
        return Err(ExportRefusal::at(Stage::Admission,format!("`{}`: the admission presented is another body's",sk.solids[body].name)));
    }
    let scale = cad::millimetres(sk).at(Stage::Blank)?;
    let blank = session.construct(&recipe.recipe).at(Stage::Blank)?;
    let (field,_) = gcs_core::solid::admission::static_remainder(sk,body,cad::AXIS_TOLERANCE).at(Stage::Blank)?;
    stage(&format!("`{}`: static blank of {} operations",sk.solids[body].name,recipe.recipe.get("nodes").unwrap().arr().len()));
    mark(Stage::Blank);
    let mut distinct: Vec<usize> = recipe.sweeps.iter().map(|s| s.swept).collect();
    distinct.sort(); distinct.dedup();
    let sheets = distinct.iter().map(|&swept| swept_sheet(session,sk,swept,blank,&field,tolerance)).collect::<Result<Vec<_>,_>>()?;
    if asked == Construction::Sector {
        match sector::construct(session,sk,body,recipe,blank,&field,&distinct,&sheets,scale) {
            Ok((solid,sector)) => return Ok(Built {solid,how:Construction::Sector,sector:Some(sector)}),
            Err(reason) => stage(&format!("`{}` is built whole: the sector construction does not apply ({reason})",
                sk.solids[body].name)),
        }
    }
    let mut tools = Vec::new();
    for (&swept,(face,_,_)) in distinct.iter().zip(&sheets) {
        for cut in recipe.sweeps.iter().filter(|c| c.swept == swept) {
            tools.push(session.place(*face,cut.pose,scale).at(Stage::Split)?);
        }
    }
    let started = std::time::Instant::now();
    let partition = session.split_solid(blank,&tools).at(Stage::Split)?;
    stage(&format!("split the blank by {} sheets into {} cells ({:?})",tools.len(),
        session.solids(partition).at(Stage::Split)?.len(),started.elapsed()));
    mark(Stage::Split);
    let started = std::time::Instant::now();
    let classified = || -> Result<(Vec<Cell>,Vec<Cell>),String> {
        let mut material = MaterialField::read(sk,body,cad::AXIS_TOLERANCE)?.evaluator(cad::POSE_CACHE);
        let (kept,removed) = classify(session,partition,&mut material)?;
        stage(&format!("classified {} material and {} removed cells ({:?})",kept.len(),removed.len(),started.elapsed()));
        if kept.is_empty() { return Err("no cell of the blank is material".into()); }
        let volumes = |cells: &[Cell]| cells.iter().map(|c| contracts::CellVolume {volume:c.volume,point:c.point}).collect::<Vec<_>>();
        let placements: Vec<usize> = distinct.iter().map(|&s| recipe.sweeps.iter().filter(|c| c.swept == s).count()).collect();
        contracts::cells(&volumes(&kept),&volumes(&removed),&placements)?;
        Ok((kept,removed))
    };
    let (kept,_) = classified().at(Stage::Classify)?;
    mark(Stage::Classify);
    let started = std::time::Instant::now();
    let fused = || -> Result<c_int,String> {
        let part = session.fuse(&kept.iter().map(|c| c.solid).collect::<Vec<_>>())?;
        let [vertex,edge,_] = session.tolerances(part)?;
        stage(&format!("united the material: {:.6} mm³, {} faces, tolerances {vertex:.1e}/{edge:.1e} mm ({:?})",
            session.volume(part)?,session.faces(part)?.len(),started.elapsed()));
        Ok(part)
    };
    let part = fused().at(Stage::Fuse)?;
    sector::debug_faces(session,"the body built whole",part);
    mark(Stage::Fuse);
    Ok(Built {solid:part,how:Construction::Whole,sector:None})
}

/// `construct_built`'s solid.
#[cfg(test)]
pub(crate) fn construct_solid(session: &Session,sk: &Sketch,solid: usize,recipe: &cad::StaticRecipe,
    admission: Option<&Admission>,tolerance: Option<Tolerance>) -> Result<c_int,ExportRefusal> {
    construct_built(session,sk,solid,recipe,admission,tolerance).map(|built| built.solid)
}

/// Native construction of a solid for export: the static recipe when it is
/// complete, the swept path when the body cuts continuous sweeps and was admitted — as one sector
/// where that applies, unless `SOLVENT_SECTOR=off` asks for the whole construction.
pub(crate) fn construct_built(session: &Session,sk: &Sketch,solid: usize,recipe: &cad::StaticRecipe,
    admission: Option<&Admission>,tolerance: Option<Tolerance>) -> Result<Built,ExportRefusal> {
    let asked = if sector::wanted() { Construction::Sector } else { Construction::Whole };
    built_as(session,sk,solid,recipe,admission,tolerance,asked)
}

/// Built as `asked` where it applies; and how it was built.
#[cfg(test)]
pub(crate) fn construct_solid_as(session: &Session,sk: &Sketch,solid: usize,recipe: &cad::StaticRecipe,
    admission: Option<&Admission>,tolerance: Option<Tolerance>,asked: Construction) -> Result<(c_int,Construction),ExportRefusal> {
    built_as(session,sk,solid,recipe,admission,tolerance,asked).map(|built| (built.solid,built.how))
}

fn built_as(session: &Session,sk: &Sketch,solid: usize,recipe: &cad::StaticRecipe,
    admission: Option<&Admission>,tolerance: Option<Tolerance>,asked: Construction) -> Result<Built,ExportRefusal> {
    if recipe.sweeps.is_empty() {
        return session.construct(&recipe.recipe).at(Stage::Blank).map(|solid| Built {solid,how:Construction::Whole,sector:None});
    }
    let admission = admission.ok_or_else(|| ExportRefusal::at(Stage::Admission,format!("`{}`: a body with swept cuts is \
        built only once admitted to the generating-sweep class",sk.solids[solid].name)))?;
    construct_swept_body(session,sk,solid,recipe,admission,tolerance,asked)
}
