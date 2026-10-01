//! A body built as one sector, patterned (docs/native-speed-plan.md). Where every swept cut is a
//! turn of one placement about one axis and the blank reads alike under that turn, the body is its
//! count of turned copies of one sector: the blank between a side through the gaps the cuts leave
//! (`solid::sector`) and that side turned by one pitch, cut by the placements that fall in it. The
//! blank is split by the two sides, the sector by its own sheets, its cells judged by the material
//! field as the whole body's would be, and the material copied round and united on the faces the
//! copies share. Whatever the premise fails at — an indexing, a blank alike under it, a gap, a side
//! the field reads as material, a sector of the blank's share of volume — is returned as the reason,
//! and the caller builds the whole body instead.
use super::*;
use gcs_core::solid::sector::{self,Boundary,Frame,Slices};
use gcs_core::envelope::Motion;

/// `SOLVENT_SECTOR=off` builds the whole body, the construction the sector is checked against.
pub(super) fn wanted() -> bool { std::env::var("SOLVENT_SECTOR").map_or(true,|v| v != "off") }

/// The side's grid across its slices, and how finely the built side is read back.
const COLUMNS: usize = 9;
const READ_BACK: usize = 4;
/// The least clearance the side keeps from every cut beyond twice the sheets' fit error (mm).
const MARGIN: f64 = 0.05;
/// The most points of the side the material field is asked about.
const PROBES: usize = 160;
/// The fuzzy value the copies are identified to (mm): the split's own.
const FUZZY: f64 = 1e-5;
/// The tolerance the core's split works to (mm), phase 3 of docs/rust-kernel-plan.md.
const SPLIT_TOL: f64 = 1e-6;

/// The sector split by the core's kernel and not the native one (phase 3 of the Rust kernel plan):
/// with the core's writer (`SOLVENT_WRITER=rust`), unless `SOLVENT_SPLIT=occt`.
pub(crate) fn core_split() -> bool {
    std::env::var("SOLVENT_WRITER").is_ok_and(|v| v == "rust") && std::env::var("SOLVENT_SPLIT").map_or(true,|v| v != "occt")
}

/// The centres of the spheres the recipe's revolutions make, where they are on the axis: the
/// slicings a bevel gear's end spheres suggest. Millimetres.
fn sphere_centres(recipe: &Json,frame: Frame,size: f64) -> Vec<[f64;3]> {
    let on_axis = |p: [f64;3]| { let d = sub(p,frame.origin); norm(sub(d,scaled(frame.axis,dot(d,frame.axis)))) <= 1e-6*size };
    let mut centres: Vec<[f64;3]> = Vec::new();
    for node in recipe.get("nodes").map(Json::arr).unwrap_or_default() {
        if node.get("kind").map(Json::as_str) != Some("revolve") { continue }
        let (Some(o),Some(a)) = (node.get("origin"),node.get("axis")) else { continue };
        let [o,a]: [[f64;3];2] = [o,a].map(|j| std::array::from_fn(|i| j.arr()[i].as_f64()));
        // a circle about a point of the revolution's own axis turns into a sphere about it
        let on_revolution = |p: [f64;3]| norm(cross(sub(p,o),a)) <= 1e-6*size*norm(a);
        let Some(profile) = node.get("profile") else { continue };
        for edges in profile.get("loops").map(Json::arr).unwrap_or_default() { for edge in edges.arr() {
            if edge.get("kind").map(Json::as_str) != Some("circle") { continue }
            let Some(c) = edge.get("center") else { continue };
            let c: [f64;3] = std::array::from_fn(|i| c.arr()[i].as_f64());
            if on_revolution(c) && on_axis(c) && !centres.iter().any(|&k| distance(k,c) <= 1e-6*size) { centres.push(c); }
        } }
    }
    centres
}

/// `SOLVENT_SECTOR_DEBUG`: the faces of a solid by the kind of their support.
pub(super) fn debug_faces(session: &Session,what: &str,solid: c_int) {
    if std::env::var_os("SOLVENT_SECTOR_DEBUG").is_none() { return }
    let mut kinds = std::collections::BTreeMap::new();
    for f in session.faces(solid).unwrap_or_default() { *kinds.entry(session.face_kind(f).unwrap_or(-1)).or_insert(0) += 1; }
    eprintln!("sector: {what}: faces by kind {kinds:?}");
}

/// Build `body` as one sector patterned, or say why the premise fails: the sector, and its union to
/// be made and checked. `sheets` are the distinct sweeps' fitted sheets, in
/// the order of `distinct`.
#[allow(clippy::too_many_arguments)]
pub(super) fn construct(session: &Session,sk: &Sketch,body: usize,recipe: &cad::StaticRecipe,blank: c_int,
    meridian: Option<super::super::Meridian>,field: &gcs_core::solid::SpatialField,distinct: &[usize],sheets: &[Fitted],scale: f64)
    -> Result<(Patterned,Union),String> {
    let started = std::time::Instant::now();
    let poses: Vec<Vec<Motion>> = distinct.iter().map(|&s| recipe.sweeps.iter().filter(|c| c.swept == s).map(|c| c.pose).collect()).collect();
    let bounds = session.bounds(&[blank])?;
    let size = distance(bounds[0],bounds[1]);
    let indexing = sector::indexing(&poses,size/scale)?;
    let n = indexing.count;
    // Millimetres: the frame and the turns as the kernel is given them.
    let frame = Frame::new(indexing.origin.map(|x| x*scale),indexing.axis);
    let mm = |m: Motion,p: [f64;3]| m.point(p.map(|x| x/scale)).map(|x| x*scale);
    let inside = |p: [f64;3]| field.value(p.map(|x| x/scale)) < 0.;
    // The blank reads alike under one pitch's turn, at points spread over its box.
    let turn = indexing.turn(1);
    let mut rng = gcs_core::rng::Rng::new(0x5ec7);
    for _ in 0..256 {
        let p: [f64;3] = std::array::from_fn(|k| rng.uniform(bounds[0][k],bounds[1][k]));
        let (a,b) = (field.value(p.map(|x| x/scale)),field.value(mm(turn,p).map(|x| x/scale)));
        if (a-b).abs()*scale > 1e-9*size {
            return Err(format!("the blank does not read alike turned by one pitch (at {p:?}, {a} against {b})"));
        }
    }
    // Each sweep's contacts in the blank at its first placement.
    let mut cuts: Vec<Vec<[f64;3]>> = Vec::new();
    for (k,(_,sheet,_)) in sheets.iter().enumerate() {
        cuts.push(sheet.points.iter().chain(&sheet.withheld).map(|&p| mm(poses[k][0],p)).filter(|&p| inside(p)).collect());
    }
    let all: Vec<[f64;3]> = cuts.iter().flatten().copied().collect();
    let mut candidates: Vec<Slices> = sphere_centres(&recipe.recipe,frame,size).into_iter().map(Slices::Spheres).collect();
    candidates.push(Slices::Planes);
    if std::env::var_os("SOLVENT_SECTOR_DEBUG").is_some() {
        for &c in &candidates {
            match Boundary::choose(&indexing,&all,&[c]) {
                Ok(b) => eprintln!("sector: {c:?}: clearance {:.4} mm; span {:?}; gaps (degrees) {:?}; phase {:?}",b.clearance,b.span,
                    b.gaps.iter().map(|g| g.map(|x| (x.to_degrees()*1e3).round()/1e3)).collect::<Vec<_>>(),
                    b.phase.iter().map(|g| (g.to_degrees()*1e3).round()/1e3).collect::<Vec<_>>()),
                Err(e) => eprintln!("sector: {c:?}: {e}"),
            }
        }
    }
    let chosen = Boundary::choose(&indexing,&all,&candidates)?;
    let fit = sheets.iter().map(|s| s.2).fold(0.,f64::max);
    let margin = MARGIN+2.*fit;
    stage(&format!("the sector: {n} placements a {:.4} degree pitch apart; its side sliced by {}, {:.4} mm clear of the cuts' \
        {} contacts at least (against {margin:.4} mm)",indexing.pitch().to_degrees(),
        match chosen.slices { Slices::Spheres(_) => "spheres about the axis", Slices::Planes => "planes square to the axis" },
        chosen.clearance,all.len()));
    if chosen.clearance < margin {
        return Err(format!("the cuts leave a gap of {:.4} mm between neighbours at least, under the {margin:.4} mm the side keeps",chosen.clearance));
    }
    // The placements whose cuts fall in the sector: one of each sweep.
    let mut tools = Vec::new();
    for (k,points) in cuts.iter().enumerate() {
        let sectors: std::collections::BTreeSet<usize> = points.iter().map(|&p| chosen.sector(p)).collect();
        let [first] = sectors.iter().copied().collect::<Vec<_>>()[..] else {
            return Err(format!("the cut of `{}` lies in {} sectors",sk.solids[distinct[k]].name,sectors.len()));
        };
        let index = (n-first) % n;
        let j = indexing.indices[k].iter().position(|&i| i == index).ok_or("no placement in the sector")?;
        tools.push(session.place(sheets[k].0,poses[k][j],scale)?);
    }
    // Their sheets split by each other on a thread of their own while the side and the sector's blank
    // are made: the sector's split then skips the sheets' intersection with each other, the longest
    // of its intersections — where every sheet fits its contacts as a fabrication export holds it (its
    // fit share of 10 µm). A looser sheet (the pinion's at the gross bars, 15 µm) carries 0.6 µm
    // tolerances into the union, and intersected apart they moved its volume by 2.7e-6 of itself.
    // `SOLVENT_SECTOR_FUSE=off` splits by the sheets as they are.
    let fusing = tools.len() > 1 && !core_split() && sheets.iter().all(|s| s.2 <= Tolerance::FABRICATION.fit())
        && std::env::var("SOLVENT_SECTOR_FUSE").map_or(true,|v| v != "off");
    std::thread::scope(|scope| {
        let fused = fusing.then(|| scope.spawn(|| session.fused_tools(&tools)));
        // The side, built and read back against the gaps and the material field.
        let (s,w) = sector::section_span(frame,chosen.slices,bounds,&inside)?;
        let (grid,rows) = chosen.grid(s,w,COLUMNS);
        let side = session.fit_sheet(&grid,rows,COLUMNS)?;
        let read: Vec<[f64;3]> = session.surface_grid(side,READ_BACK*rows,READ_BACK*COLUMNS)?.into_iter().map(|(p,_)| p).filter(|&p| inside(p)).collect();
        let clear = chosen.clearance_of(&read)?;
        if clear < margin { return Err(format!("the side as built passes {clear:.4} mm from a cut, under {margin:.4} mm")); }
        let body_field = MaterialField::read(sk,body,cad::AXIS_TOLERANCE)?;
        let radius = margin/2.;
        let deep: Vec<[f64;3]> = read.iter().copied().filter(|p| field.value(p.map(|x| x/scale))*scale < -radius).collect();
        let step = deep.len().div_ceil(PROBES).max(1);
        let probed_at: Vec<[f64;3]> = deep.iter().copied().step_by(step).collect();
        // each probe on its own core, an evaluator a thread; the first to fail, in order, is the reason
        let probes = gcs_core::par::indices_with(probed_at.len(),|| body_field.evaluator(cad::POSE_CACHE),|material,i| {
            material.probe(probed_at[i].map(|x| Interval::point(x/scale).unwrap()),[1.,0.,0.],radius/scale,
                Options {value_tolerance:radius/scale/4.,max_evaluations:40000}).map_err(|e| format!("{e:?}"))
        });
        let mut probed = 0;
        for (&p,probe) in probed_at.iter().zip(probes) {
            let probe = probe?;
            if probe.state != ProbeState::InteriorBall {
                return Err(format!("the material field reads the side at {:?} as {:?}, not material",p.map(|x| (x*1e3).round()/1e3),probe.state));
            }
            probed += 1;
        }
        stage(&format!("the side: {rows}x{COLUMNS} nodes, read back {clear:.4} mm clear of the cuts at {} points in the blank, \
            {probed} of them material by the field ({:?})",read.len(),started.elapsed()));
        // The blank between the side and its turn: its share of the blank's volume.
        let clock = std::time::Instant::now();
        let other = session.place(side,turn,scale)?;
        // The blank made again about the axis, so the copies' faces on it are one parameterization
        // turned: its volume is the blank's
        let whole = session.volume(blank)?;
        // its faces' parameters starting opposite the sector, so none of its faces crosses where they start
        let across = chosen.angle_at((chosen.span[0]+chosen.span[1])/2.)+indexing.pitch()/2.+std::f64::consts::PI;
        // A blank built from its meridian region is that region turned again, from the half-plane its
        // parameters are to start on; any other is sectioned by that half-plane first.
        let turned = match meridian {
            Some(m) => {
                let seam = frame.radial(across);
                let angle = dot(frame.axis,cross(m.seam,seam)).atan2(dot(m.seam,seam));
                let region = session.place(m.region,super::super::turned_about(m.origin,m.axis,angle)?,1.)?;
                session.revolve_region(region,frame.origin,frame.axis)?
            }
            None => session.revolved(blank,frame.origin,frame.axis,frame.radial(across))?,
        };
        let again = session.volume(turned)?;
        if (again-whole).abs() > 1e-7*whole {
            return Err(format!("the blank turned about its axis again is {again:.9} mm³, not its {whole:.9} mm³"));
        }
        let debug = std::env::var_os("SOLVENT_SECTOR_DEBUG").is_some();
        if debug { eprintln!("sector: the blank made again about its axis ({:?})",clock.elapsed()); }
        // `SOLVENT_SPLIT_DUMP=DIR`: what the split takes and makes, as the core's B-rep reads it, for
        // the core's split to be developed against (phase 3 of docs/rust-kernel-plan.md)
        let dump_dir = std::env::var("SOLVENT_SPLIT_DUMP").ok();
        let dump = |shape: c_int,name: &str| -> Result<(),String> {
            let Some(dir) = &dump_dir else { return Ok(()) };
            // (a shape the core holds has no kernel's reading to dump)
            if session.held(shape).is_some() { return Ok(()) }
            std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
            std::fs::write(format!("{dir}/{name}.json"),session.brep_json(shape)?).map_err(|e| e.to_string())
        };
        dump(turned,"blank")?; dump(side,"side")?; dump(other,"other")?;
        for (k,&t) in tools.iter().enumerate() { dump(t,&format!("sheet{k}"))?; }
        if core_split() {
            let piece = core_sector(session,turned,[side,other],&tools,n,&body_field,distinct.len(),clock,
                &recipe.recipe,frame.origin,frame.axis,frame.radial(across))?;
            let one = gcs_core::brep::props::volume(&piece);
            let sector = Patterned {piece:-1,sides:[side,other],fuzzy:FUZZY,origin:frame.origin,axis:frame.axis,count:n,pitch:indexing.pitch()};
            return Ok((sector,Union {sector,one,clock,core:Some(piece)}))
        }
        let halves = session.split_solid(turned,&[side,other])?;
        if debug { eprintln!("sector: the blank split by the sides ({:?})",clock.elapsed()); }
        let cells = session.solids(halves)?;
        let volumes = cells.iter().map(|&c| session.volume(c)).collect::<Result<Vec<_>,_>>()?;
        let share = whole/n as f64;
        if debug { eprintln!("sector: blank {whole:.6}, share {share:.6}, cells {volumes:.6?} ({:?})",clock.elapsed()); }
        let Some(wedge) = cells.iter().zip(&volumes).find(|(_,v)| (*v-share).abs() <= 1e-5*whole).map(|(c,_)| *c) else {
            return Err(format!("the sides split the blank ({whole:.6} mm³) into {} cells of {volumes:.6?} mm³, none its {share:.6} mm³ share",
                cells.len()));
        };
        if cells.len() != 2 { return Err(format!("the sides split the blank into {} cells, not two",cells.len())); }
        let tool = match fused { Some(fused) => vec![fused.join().unwrap_or_else(|e| std::panic::resume_unwind(e))?],None => tools.clone() };
        dump(wedge,"wedge")?;
        let partition = session.split_solid(wedge,&tool)?;
        for (k,&c) in session.solids(partition)?.iter().enumerate() { dump(c,&format!("cell{k}"))?; }
        stage(&format!("split the blank by the sides into the sector of {share:.6} mm³, and it by {} sheets into {} cells ({:?})",
            tools.len(),session.solids(partition)?.len(),clock.elapsed()));
        mark(Stage::Split);
        let clock = std::time::Instant::now();
        let (kept,removed) = classify(session,partition,&body_field)?;
        stage(&format!("classified {} material and {} removed cells ({:?})",kept.len(),removed.len(),clock.elapsed()));
        if kept.is_empty() { return Err("no cell of the sector is material".into()); }
        let volumes = |cells: &[Cell]| cells.iter().map(|c| contracts::CellVolume {volume:c.volume,point:c.point}).collect::<Vec<_>>();
        contracts::cells(&volumes(&kept),&volumes(&removed),&vec![1;distinct.len()])?;
        mark(Stage::Classify);
        let clock = std::time::Instant::now();
        let piece = session.fuse(&kept.iter().map(|c| c.solid).collect::<Vec<_>>())?;
        dump(piece,"piece")?;
        let one = session.volume(piece)?;
        debug_faces(session,"the sector's material",piece);
        let sector = Patterned {piece,sides:[side,other],fuzzy:FUZZY,origin:frame.origin,axis:frame.axis,count:n,pitch:indexing.pitch()};
        Ok((sector,Union {sector,one,clock,core:None}))
    })
}

/// A sector's material to be turned into its copies and united (`Union::make`, `Session::pattern`)
/// and the union checked (`Union::check`), which a caller may run beside the files written from the
/// union and the sector.
pub(crate) struct Union { sector: Patterned,one: f64,clock: std::time::Instant,
    /// The sector's material as the core's split made it, where it made it (`core_split`): its
    /// `sector.piece` is then no native solid, and the core turns it into the copies.
    pub core: Option<gcs_core::brep::topo::Brep> }

impl Union {
    /// The sector and its copies united, unchecked.
    pub(crate) fn make(&self,session: &Session) -> Result<c_int,String> {
        let s = &self.sector;
        let angles: Vec<f64> = (1..s.count).map(|k| k as f64*s.pitch).collect();
        session.pattern(s.piece,s.origin,s.axis,&angles,s.sides,s.fuzzy)
    }
    /// The union `made` checked and measured, against the sector's own volume: the solid to keep.
    pub(crate) fn check(&self,session: &Session,made: c_int) -> Result<c_int,String> {
        let (n,one) = (self.sector.count,self.one);
        let part = session.pattern_check(made)?;
        let volume = session.volume(part)?;
        debug_faces(session,"the sectors united",part);
        let [vertex,edge,_] = session.tolerances(part)?;
        stage(&format!("united the material: {volume:.6} mm³, {} faces, tolerances {vertex:.1e}/{edge:.1e} mm, {n} sectors of {one:.6} mm³ \
            ({:?})",session.faces(part)?.len(),self.clock.elapsed()));
        // The sector's own measure carries its two sides, spline faces whose trims the split leaves
        // within the sheets' tolerance (6e-4 mm) of meeting: a part in 1e5 of its volume.
        if (volume-n as f64*one).abs() > 1e-4*volume {
            return Err(format!("the sectors united are {volume:.6} mm³, not {n} times {one:.6}"));
        }
        mark(Stage::Fuse);
        Ok(part)
    }
}

/// The sector by the core's kernel: the blank turned about its axis split by the two sides, the
/// share of its volume taken, that split by each sheet in turn, its cells judged by the material
/// field at interior samples the core measures (`brep::query::interior`), and the material cell
/// returned as the core's B-rep.
#[allow(clippy::too_many_arguments)]
fn core_sector(session: &Session,turned: c_int,sides: [c_int;2],tools: &[c_int],n: usize,field: &MaterialField,
    sweeps: usize,clock: std::time::Instant,recipe: &gcs_core::json::Json,origin: [f64;3],axis: [f64;3],seam: [f64;3])
    -> Result<gcs_core::brep::topo::Brep,String> {
    use gcs_core::brep::{boolean::split,json,props::volume,query::interior};
    let read = |s: c_int| session.brep(s).map(|b| (*b).clone());
    // the blank: the core's (phase 4), its recipe built and turned about the axis till its faces'
    // seams lie in the half-plane opposite the sector, or the kernel's (`SOLVENT_BLANK=occt`)
    let blank = if std::env::var("SOLVENT_BLANK").is_ok_and(|v| v == "occt") { read(turned)? } else {
        let started = std::time::Instant::now();
        // its meridian region turned once, from the half-plane opposite the sector; where it is not a
        // solid of revolution, its Booleans turned till its seams lie there
        if let Ok(dir) = std::env::var("SOLVENT_RECIPE_DUMP") { let _ = std::fs::write(format!("{dir}/blank.json"),recipe.dump(None)); }
        let blank = match gcs_core::brep::recipe::meridian(recipe,seam)? {
            Ok((b,o,a)) => {
                if gcs_core::space::norm(cross(a,axis)) > 1e-9 || gcs_core::space::norm(cross(sub(o,origin),axis)) > 1e-6 {
                    return Err("the blank's meridian is about another line than the indexing's".into())
                }
                b
            }
            Err(_) => turned_to(&gcs_core::brep::recipe::build(recipe)?,origin,axis,seam)?,
        };
        if std::env::var_os("SOLVENT_BREP_DEBUG").is_some() {
            for (fi,f) in blank.faces.iter().enumerate() {
                let (mut lo,mut hi) = ([f64::INFINITY;2],[f64::NEG_INFINITY;2]);
                for c in f.loops.iter().flatten() {
                    let e = &blank.edges[c.edge as usize];
                    for j in 0..=8 { let q = c.pcurve.at(e.t[0]+(e.t[1]-e.t[0])*j as f64/8.,e,&f.surface,&blank.vertices);
                        for k in 0..2 { lo[k] = lo[k].min(q[k]); hi[k] = hi[k].max(q[k]); } }
                }
                eprintln!("blank: face {fi} {} {:?}: {} loops, {} uses, u {:.4}..{:.4} v {:.4}..{:.4}",f.surface.kind(),f.name,f.loops.len(),
                    f.loops.iter().map(|l| l.len()).sum::<usize>(),lo[0],hi[0],lo[1],hi[1]);
            }
        }
        let kinds = |b: &gcs_core::brep::topo::Brep| b.faces.iter().fold(std::collections::BTreeMap::new(),|mut m,f| { *m.entry(f.surface.kind()).or_insert(0) += 1; m });
        let (ours,theirs) = (volume(&blank),session.volume(turned)?);
        stage(&format!("the blank by the core: {ours:.6} mm³ against the kernel's {theirs:.6}, faces {:?} against {:?} ({:?})",kinds(&blank),
            kinds(&read(turned)?),started.elapsed()));
        if (ours-theirs).abs() > 1e-7*theirs { return Err(format!("the core's blank is {ours:.9} mm³, the kernel's {theirs:.9}")) }
        blank
    };
    let whole = volume(&blank);
    let halves = split(&blank,&read(sides[0])?.joined(&read(sides[1])?),SPLIT_TOL)?;
    let volumes: Vec<f64> = halves.iter().map(volume).collect();
    let share = whole/n as f64;
    let Some(wedge) = halves.iter().zip(&volumes).find(|(_,v)| (*v-share).abs() <= 1e-5*whole).map(|(c,_)| c.clone()) else {
        return Err(format!("the sides split the blank ({whole:.6} mm³) by the core into {} cells of {volumes:.6?} mm³, none its \
            {share:.6} mm³ share",halves.len()));
    };
    if halves.len() != 2 { return Err(format!("the sides split the blank by the core into {} cells, not two",halves.len())); }
    let mut cells = vec![wedge];
    for &t in tools {
        let sheet = read(t)?;
        let split = gcs_core::par::map(&cells,|c| split(c,&sheet,SPLIT_TOL));
        cells = split.into_iter().collect::<Result<Vec<_>,_>>()?.into_iter().flatten().collect();
    }
    for c in &cells { c.check(1e-6).map_err(|e| format!("a cell of the core's split is invalid: {e}"))?; }
    let volumes: Vec<f64> = cells.iter().map(volume).collect();
    stage(&format!("split the blank by the sides into the sector of {share:.6} mm³, and it by {} sheets into {} cells, by the core \
        ({:?})",tools.len(),cells.len(),clock.elapsed()));
    mark(Stage::Split);
    let clock = std::time::Instant::now();
    let sampled = gcs_core::par::map(&cells,|c| interior(c,4)).into_iter().collect::<Result<Vec<_>,_>>()?;
    let (kept,removed) = judge(&volumes,sampled,field,clock.elapsed().as_secs_f64())?;
    stage(&format!("classified {} material and {} removed cells ({:?})",kept.len(),removed.len(),clock.elapsed()));
    let measured = |cells: &[Cell]| cells.iter().map(|c| contracts::CellVolume {volume:c.volume,point:c.point}).collect::<Vec<_>>();
    contracts::cells(&measured(&kept),&measured(&removed),&vec![1;sweeps])?;
    mark(Stage::Classify);
    let [kept] = &kept[..] else { return Err(format!("{} cells of the sector are material: the core does not unite them",kept.len())) };
    // each edge's tolerance measured, as a kernel's shape read into the core's B-rep has
    let mut piece = cells.swap_remove(kept.solid as usize);
    json::measure(&mut piece);
    Ok(piece)
}

/// A solid of revolution about the line through `origin` along `axis` turned about it until its
/// faces' seams (an edge used twice by one face) lie in the half-plane along `seam`; refused where
/// its seams are not in one half-plane.
fn turned_to(b: &gcs_core::brep::topo::Brep,origin: [f64;3],axis: [f64;3],seam: [f64;3]) -> Result<gcs_core::brep::topo::Brep,String> {
    let mut found: Option<[f64;3]> = None;
    for f in &b.faces {
        let uses: Vec<u32> = f.loops.iter().flatten().map(|c| c.edge).collect();
        for (k,&e) in uses.iter().enumerate() {
            if !uses[k+1..].contains(&e) { continue }
            let edge = &b.edges[e as usize];
            let gcs_core::brep::topo::EdgeCurve::Curve(_) = &edge.curve else { continue };
            let p = edge.point((edge.t[0]+edge.t[1])/2.,&b.vertices);
            let r = sub(p,origin);
            let r = sub(r,gcs_core::space::scale(axis,dot(r,axis)));
            let l = norm(r);
            if l < 1e-9 { continue }
            let r = r.map(|x| x/l);
            match found {
                None => found = Some(r),
                Some(q) if dot(q,r) < 1.-1e-12 => return Err("the blank's seams are not in one half-plane".into()),
                _ => {}
            }
        }
    }
    let Some(r) = found else { return Ok(b.clone()) };
    let angle = dot(axis,cross(r,seam)).atan2(dot(r,seam));
    Ok(b.moved(&gcs_core::brep::geom::Rigid::turn(origin,axis,angle)))
}
