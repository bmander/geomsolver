//! **One stepped exact build for every object** (#670, #671): `brep::export::Builder` builds a
//! static body as the export builds it, a stage at a time like a swept one, and hands back its
//! `Display` — the mesh, its faces named, the B-rep's edges — which crosses between cores as one
//! buffer and is supplied to a sketch that builds no object of its own (`FieldMeshing::Supplied`):
//! one job a binding (`Sketch::object_jobs`), an object on its way until its build arrives, its
//! refusal said in the build's words, and an earlier drawing's build drawn, stale, till the new.
use gcs_core::brep::export::{self,Builder,Display};
use gcs_core::brep::sweep::Say;
use gcs_core::model::{objects,Sketch};
use gcs_core::solid::{ApproximationPolicy,ObjectState};
use std::f64::consts::PI;

fn solved(name: &str) -> Option<Sketch> {
    let e = fixtures::example(name)?;
    let mut sk = e.sketch;
    gcs_core::solve::solve(&mut sk,gcs_core::solve::SolveOpts::default());
    Some(sk)
}

fn document(text: &str) -> Sketch { fixtures::read(text).sketch }

/// Solid `i` built by the stepped builder to the end: its display, or the refusal's words.
fn stepped(sk: &Sketch,i: usize) -> Result<(Display,Vec<String>),String> {
    let mut b = Builder::new(sk,i).map_err(|r| r.message)?;
    let mut doing = vec![b.doing()];
    while !b.step().map_err(|r| r.message)? { doing.push(b.doing()); }
    let (done,total) = b.stages();
    assert_eq!(done,total,"`{}`: {doing:?}",sk.solids[i].name);
    Ok((b.display().unwrap().clone(),doing))
}

/// What the export builds of solid `i`: its face count and volume (model units), or its refusal.
fn exported(sk: &Sketch,i: usize) -> Result<(usize,f64),String> {
    let say = Say {stage:&|_: &str| {},mark:&|_| {}};
    let x = export::exact(sk,i,None,None,&say).map_err(|r| r.message)?;
    let mm = sk.units.mm_per();
    Ok((x.solid.faces.len(),gcs_core::brep::props::volume(&x.solid)/(mm*mm*mm)))
}

/// A display's surface area (model units squared).
fn area(d: &Display) -> f64 {
    use gcs_core::space::{cross,norm,sub};
    d.triangles.iter().map(|t| { let [a,b,c] = t.map(|k| d.vertices[k as usize]); 0.5*norm(cross(sub(b,a),sub(c,a))) }).sum()
}

/// Every object the documents `names` hold (swept or not, as `swept` says): built by the
/// stepped builder as the export builds it, with as many faces (a static one), each named, and the
/// same volume — a swept one's within the export's gross bar, since the display admits it at fewer
/// samples; or refused in the same words.  How many were built.
fn corpus(names: &[String],swept: bool) -> usize {
    let mut built = 0;
    for name in names {
        let Some(sk) = solved(name) else { continue };
        for i in objects(&sk).into_iter().filter(|&i| sk.is_swept(i) == swept) {
            let solid = &sk.solids[i].name;
            match (exported(&sk,i),stepped(&sk,i)) {
                (Ok((faces,volume)),Ok((d,doing))) => {
                    assert_eq!(d.faces.names.len(),d.faces.smooth.len(),"{name}: `{solid}`'s faces named");
                    if !swept {
                        assert_eq!(doing[0],"building the solid from its recipe","{name}: `{solid}`");
                        assert_eq!(d.faces.smooth.len(),faces,"{name}: `{solid}`'s faces");
                    }
                    // a static body is the export's own build; a swept one is admitted at the
                    // display's samples, and comes out within the export's gross bar of it: no
                    // more volume than its surface swept that far
                    let within = if swept { export::GROSS/sk.units.mm_per()*area(&d) } else { 1e-9*volume.abs().max(1.) };
                    assert!((d.faces.volume-volume).abs() <= within,"{name}: `{solid}`: {} against {volume}, within {within}",d.faces.volume);
                    built += 1;
                }
                (Err(a),Err(b)) => assert_eq!(a,b,"{name}: `{solid}`"),
                (a,b) => panic!("{name}: `{solid}`: the export {:?}, the builder {:?}",a.map(|_| ()),b.map(|_| ())),
            }
        }
    }
    built
}

/// **Every object the export builds, the builder builds the same; one it refuses, it refuses in
/// the same words** — here over documents of every kind of static body, and over the whole case
/// library in the slow tier (`every_object_of_the_library_builds_as_the_export_builds_it`).
#[test]
fn every_static_object_builds_as_the_export_builds_it() {
    let names: Vec<String> = ["solid_flange.sv","turned_shaft.sv","star_handle.sv","rounded_bracket.sv","solid_tray.sv",
        "pin_lug.sv","solid_elbow.sv"].map(String::from).to_vec();
    let built = corpus(&names,false);
    assert!(built >= names.len(),"{built} built");
}

/// The whole case library, static objects and swept ones (`every_static_object_builds_as_the_export_builds_it`).
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, many minutes: every object of the library built twice")]
fn every_object_of_the_library_builds_as_the_export_builds_it() {
    let names = fixtures::example_names();
    assert!(corpus(&names,false) > 50);
    corpus(&names,true);
}

/// **A sharp rim's fan builds at the display's samples** (`?example=solid_sweeps`, `vthread.sv`):
/// the rod's end planes meet the fan the V wheel's rim sweeps in a curve 0.09 mm long, shorter than
/// a march's step there (0.22 mm). Dropping every point within a quarter of a step of an end cut it
/// to a chord across the rim's tip, the meeting was traced again from a seed that chord missed, and
/// the split took the copy whose fit ran 3.4e-6 off the fan, refusing a cell. Each of the trace's
/// three rules builds it alone (`docs/contract/brep.md`): the turn's steps kept, the fit held to
/// both surfaces between its points, and one curve kept of a meeting traced twice.
#[test]
fn a_sharp_rims_fan_builds_at_the_displays_samples() {
    let sk = document(&crate::solid_sweeps_examples::text("vthread.sv"));
    let rod = sk.solids.iter().position(|s| s.name == "rod").unwrap();
    let (d,_) = stepped(&sk,rod).unwrap_or_else(|e| panic!("{e}"));
    let (_,volume) = exported(&sk,rod).unwrap();
    let within = export::GROSS/sk.units.mm_per()*area(&d);
    assert!((d.faces.volume-volume).abs() <= within,"{} against the export's {volume}, within {within}",d.faces.volume);
}

const BLOCK: &str = "unit mm\nuse std\nuse std.rounds (round)\nin std.front {\n  a := point hint((0, 0))\n  b := point hint((40, 0))\n  \
    c := point hint((40, 20))\n  d := point hint((0, 20))\n  fix((0, 0)) a\n  l1 := line(a, b)\n  l2 := line(b, c)\n  \
    l3 := line(c, d)\n  l4 := line(d, a)\n  horizontal l1\n  horizontal l3\n  vertical l2\n  vertical l4\n  \
    length(40) l1\n  length(20) l2\n}\nblock := solid(face(l1, l2, l3, l4), depth: 10)\n";

/// A static body is a builder of two stages — its recipe built, then meshed — which says what it
/// is doing as a swept one does, and its display names each face and draws each edge: a block's
/// twelve, every one a crease.
#[test]
fn a_static_body_is_built_by_stages_with_its_faces_named_and_its_edges_drawn() {
    let sk = document(BLOCK);
    let block = objects(&sk)[0];
    let mut b = Builder::new(&sk,block).unwrap();
    assert_eq!(b.stages(),(0,2));
    assert_eq!(b.doing(),"building the solid from its recipe");
    assert!(!b.step().unwrap());
    assert_eq!(b.doing(),"meshing the exact surface");
    assert!(b.said().unwrap().contains("built the solid by the Rust kernel"),"{:?}",b.said());
    assert!(b.step().unwrap());
    assert_eq!(b.stages(),(2,2));
    let d = b.display().unwrap();
    assert!((d.faces.volume-8000.).abs() < 1e-6,"{}",d.faces.volume);
    assert_eq!(d.faces.smooth,vec![false;6]);
    assert_eq!(d.faces.names.len(),6);
    assert!(d.faces.names.iter().all(|n| n.starts_with("block.")),"{:?}",d.faces.names);
    assert_eq!(d.faces.edges.len(),12,"{:?}",d.faces.edges);
    assert!(d.faces.edges.iter().all(|e| !e.smooth && e.faces[0] < e.faces[1] && e.points.len() >= 2));
    // each edge runs between two corners of the block, 40, 20 or 10 long
    for e in &d.faces.edges {
        let [p,q] = [e.points[0],*e.points.last().unwrap()].map(|k| d.vertices[k as usize]);
        let l = gcs_core::space::distance(p,q);
        assert!([40.,20.,10.].iter().any(|w| (l-w).abs() < 1e-9),"{l}");
    }
}

/// A round's edges meet its faces tangent, and are drawn smooth; the rest of the body's are
/// creases.
#[test]
fn a_rounds_tangent_edges_are_smooth() {
    let sk = document(&format!("{BLOCK}block.near round(r: 2mm) block\n"));
    let body = objects(&sk)[0];
    let (d,_) = stepped(&sk,body).unwrap();
    assert!(d.faces.edges.iter().any(|e| e.smooth),"{:?}",d.faces.edges.iter().map(|e| e.smooth).collect::<Vec<_>>());
    assert!(d.faces.edges.iter().any(|e| !e.smooth));
}

/// A display crosses between cores as one buffer, read back as it was written; a buffer cut short,
/// of another format or naming a vertex that is not there is refused, never read as a surface.
#[test]
fn a_display_crosses_as_one_buffer_and_a_bad_one_is_refused() {
    let sk = document(BLOCK);
    let (d,_) = stepped(&sk,objects(&sk)[0]).unwrap();
    let bytes = d.bytes();
    let back = Display::read(&bytes).unwrap();
    assert_eq!(back.vertices,d.vertices);
    assert_eq!(back.triangles,d.triangles);
    assert_eq!(back.faces.of,d.faces.of);
    assert_eq!(back.faces.smooth,d.faces.smooth);
    assert_eq!(back.faces.names,d.faces.names);
    assert_eq!(back.faces.edges,d.faces.edges);
    assert_eq!(back.faces.volume,d.faces.volume);
    assert!(Display::read(&bytes[..bytes.len()-1]).unwrap_err().contains("not as long as it says"));
    let mut other = bytes.clone();
    other[..8].copy_from_slice(&99f64.to_le_bytes());
    assert!(Display::read(&other).unwrap_err().contains("another format"));
    let mut wild = d.clone();
    wild.triangles[0][0] = wild.vertices.len() as u32;
    assert!(Display::read(&wild.bytes()).unwrap_err().contains("vertex past"));
    assert!(Display::read(&[]).is_err());
}

/// Two closed instances of one component, standing apart: one binding, placed twice.
const PAIR: &str = "unit mm\nuse std\nuse std.place (offset)\ncomponent Plate(w: Length) {\n  in std.front {\n    \
    a := point hint((1, 1))\n    b := point hint((w, 1))\n    c := point hint((w, w))\n    e := point hint((1, w))\n    \
    a offset(dx: 1mm, dy: 1mm) std.origin\n    b offset(dx: w, dy: 1mm) std.origin\n    c offset(dx: w, dy: w) std.origin\n    \
    e offset(dx: 1mm, dy: w) std.origin\n    ab := line(a, b)\n    bc := line(b, c)\n    ce := line(c, e)\n    \
    ea := line(e, a)\n    body := solid(face(ab, bc, ce, ea), from: 0mm, to: 2mm)\n  }\n}\n\
    in (f := frame) { one := Plate(w: 11mm) }\nfix(dir == (1, 0, 0)) f.u\nfix(dir == (0, 1, 0)) f.v\nfix(origin == (0, 0, 0)) f\n\
    in (g := frame) { two := Plate(w: 11mm) }\nfix(dir == (1, 0, 0)) g.u\nfix(dir == (0, 1, 0)) g.v\nfix(origin == (30, 0, 0)) g\n";

/// **A sketch that builds no object of its own** (#671): every object is on its way until its
/// build is supplied — refused here, never built on the thread that draws, and not unbuilt — then
/// drawn from the build; a refused build is unbuilt in its words; a changed drawing draws the
/// earlier build, stale, till the new one arrives.
#[test]
fn a_sketch_supplied_its_objects_builds_none_of_its_own() {
    let sk = document(BLOCK);
    let block = objects(&sk)[0];
    sk.supply_objects(true);
    let jobs = sk.object_jobs();
    assert_eq!(jobs.len(),1);
    assert_eq!((jobs[0].solid,jobs[0].objects.clone(),jobs[0].swept),(block,vec![block],false));
    assert_eq!(sk.object_state(block),ObjectState::Building);
    let refused = sk.evaluated_solid(block,ApproximationPolicy::Mesh).err().unwrap();
    assert!(refused.contains("still being built"),"{refused}");
    assert!(gcs_core::overview::unbuilt(&sk).is_empty(),"on its way, not unbuilt");
    // its build supplied, as a worker hands it over
    let (d,_) = stepped(&sk,block).unwrap();
    assert!(sk.supply_object(block,Display::read(&d.bytes()).unwrap()).unwrap());
    assert_eq!(sk.object_state(block),ObjectState::Built);
    let solid = sk.evaluated_solid(block,ApproximationPolicy::Mesh).unwrap();
    assert!((solid.volume()-8000.).abs() < 1e-6);
    assert!(!solid.stale());
    assert!(solid.surviving_faces().iter().any(|f| f.starts_with("block.")),"{:?}",solid.surviving_faces());
    assert!(solid.edges().iter().any(|e| !e.smooth));
    // the drawing changed: the earlier build is drawn till the new one arrives
    let mut moved = sk.clone();
    let x: Vec<f64> = moved.get_x();
    moved.set_x(&x.iter().map(|v| v*1.5).collect::<Vec<_>>());
    if gcs_core::solid::reads(&moved,block,0.) != gcs_core::solid::reads(&sk,block,0.) {
        moved.supply_objects(true);
        assert_eq!(moved.object_state(block),ObjectState::Rebuilding);
        assert!(moved.evaluated_solid(block,ApproximationPolicy::Mesh).unwrap().stale());
    }
    // a refusal is the build's words
    sk.refuse_object(block,"`block`: the built boundary is invalid").unwrap();
    assert_eq!(sk.object_state(block),ObjectState::Refused("`block`: the built boundary is invalid".into()));
    assert_eq!(gcs_core::overview::unbuilt(&sk),vec![(block,"`block`: the built boundary is invalid".to_string())]);
}

/// **One job a binding** (#671): two placements of one component's solid are one build, of the
/// first, and each is drawn from it where it stands.
#[test]
fn the_placements_of_one_binding_are_one_job() {
    let sk = document(PAIR);
    let objs = objects(&sk);
    assert_eq!(objs.len(),2,"{:?}",objs.iter().map(|&i| &sk.solids[i].name).collect::<Vec<_>>());
    sk.supply_objects(true);
    let jobs = sk.object_jobs();
    assert_eq!(jobs.len(),1,"{jobs:?}");
    assert_eq!(jobs[0].objects,objs);
    let (d,_) = stepped(&sk,jobs[0].solid).unwrap();
    assert!(sk.supply_object(jobs[0].solid,d).unwrap());
    let boxes: Vec<_> = objs.iter().map(|&i| {
        assert_eq!(sk.object_state(i),ObjectState::Built,"`{}`",sk.solids[i].name);
        let s = sk.evaluated_solid(i,ApproximationPolicy::Mesh).unwrap();
        assert!((s.volume()-200.).abs() < 1e-6,"{}",s.volume());
        s.world_bounds()
    }).collect();
    assert!((boxes[1].lo[0]-boxes[0].lo[0]-30.).abs() < 1e-6,"{boxes:?}");
}

/// **A face family's job keeps its key through the page's write-back** (#671): the page solves,
/// writes the drawing back into its source (`edit::reconcile`, which here drops the seeds the
/// fixed frame points do not need) and hands a worker that text with its values.  The family's
/// key reads its running number's name, so that name is the family's count and not where it
/// stands: the worker names the job as the page did, though the write-back moved the `sweep`.
#[test]
fn a_face_familys_job_keeps_its_key_through_the_write_back() {
    let text = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"),"/../examples/paths/coil.sv")).unwrap();
    let mut page = fixtures::read(&text);
    gcs_core::solve::solve(&mut page.sketch,gcs_core::solve::SolveOpts::default());
    page.sketch.supply_objects(true);
    let jobs = page.sketch.object_jobs();
    assert_eq!(jobs.iter().map(|j| j.name.as_str()).collect::<Vec<_>>(),["coil"]);
    let sk = std::mem::take(&mut page.sketch);
    let written = gcs_core::edit::reconcile(&mut page,&sk);
    assert_eq!(written.refused,None);
    let sweep = |t: &str| t.find("coil := sweep").unwrap();
    assert_ne!(sweep(&written.text),sweep(&text),"the write-back moved the family:\n{}",written.text);
    let mut worker = document(&written.text);
    assert!(worker.set_x(&sk.get_x()));
    assert_eq!(worker.object_jobs(),jobs,"the worker's job is the page's");
}

/// **`exact_solid` and the builder agree** (#536): a body a generating sweep cuts is built by
/// `exact_solid` as the export builds it, where the sketch builds its own solids; where a host
/// builds them, the refusal names the builder that does.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 15 s: a generated body built twice")]
fn exact_solid_builds_a_generated_body_as_the_export_does() {
    let sk = solved("swept_torus.sv").unwrap();
    let part = (0..sk.solids.len()).find(|&i| sk.solids[i].name == "part").unwrap();
    let x = sk.exact_solid(part).unwrap();
    let (faces,volume) = exported(&sk,part).unwrap();
    assert_eq!(x.brep.faces.len(),faces);
    assert!((gcs_core::brep::props::volume(&x.brep)/x.mm.powi(3)-volume).abs() < 1e-9*volume);
    let hosted = solved("swept_torus.sv").unwrap();
    hosted.supply_objects(true);
    let why = hosted.exact_solid(part).unwrap_err();
    assert!(why.contains("`brep::export::Builder`"),"{why}");
}

/// **A build is current while what its solid reads is what the build read, within a solve's own
/// tolerance** (#671): a solve settling the drawing again changes nothing a build reads, and the
/// supply says it is current, so a host files it under the job's key and asks nothing again; an
/// edit leaves the build stale, drawn till the new one arrives.
#[test]
fn a_build_is_current_within_a_solves_tolerance() {
    let sk = document(BLOCK);
    let block = objects(&sk)[0];
    sk.supply_objects(true);
    let key = sk.object_jobs()[0].key;
    let (d,_) = stepped(&sk,block).unwrap();
    assert_eq!(gcs_core::solid::reads_key(&d.reads),key,"the build read what the drawing reads");
    // what a solve settling the block again leaves: its numbers a hair from the build's
    let nudged = Display {reads:d.reads.iter().map(|x| x*(1.+1e-14)).collect(),..d.clone()};
    assert!(sk.supply_object(block,nudged.clone()).unwrap());
    assert_eq!(sk.object_state(block),ObjectState::Built);
    assert_eq!(sk.object_jobs()[0].key,key,"the drawing's own key");
    // an earlier drawing's build: stale, and the drawing's own key asked
    let earlier = Display {reads:d.reads.iter().map(|x| x*1.01+1.).collect(),..d.clone()};
    assert!(!sk.supply_object(block,earlier).unwrap(),"an earlier drawing's");
    assert_eq!(sk.object_state(block),ObjectState::Rebuilding);
    assert!(sk.evaluated_solid(block,ApproximationPolicy::Mesh).unwrap().stale());
}

/// **A body built from swept material is built as the export composes it, to any depth** (the
/// radiator bot's compounds): a shaped gear united with a copy of itself screwed along its axis —
/// two swept operands, one a placement — and a body made of that compound, whose stock holds the
/// sweeps a level down; each operand built by its own stages, the compound combined, then meshed.
#[test]
fn a_body_of_swept_bodies_is_built_by_stages_as_the_export_composes_it() {
    let src = crate::rack_cut::SPACE.to_string() + "\
screw := frame
use std.motion (screws)
screw screws(48mm) turn.axis
construction lifted := gear at(under: screw) 30deg
construction compound := solid(gear)
lifted union compound
nested := solid(compound)
";
    let mut e = crate::common::build(&src);
    let r = gcs_core::solve::solve(&mut e.sketch,gcs_core::solve::SolveOpts::default());
    assert!(r.success,"{}",r.message);
    let sk = e.sketch;
    let nested = e.map.ent_named("nested").unwrap().i();
    let (faces,volume) = exported(&sk,nested).unwrap();
    let (d,doing) = stepped(&sk,nested).unwrap();
    assert!(doing.iter().filter(|d| d.starts_with("admitting its sweeps")).count() == 2,"{doing:?}");
    assert!(doing.iter().any(|d| d == "adding the rest of the body"),"{doing:?}");
    let within = export::GROSS/sk.units.mm_per()*area(&d);
    assert!((d.faces.volume-volume).abs() <= within,"{} against {volume}, within {within}",d.faces.volume);
    // the two gears overlap 2 mm of their 6, each turned off the other's tooth space
    let gear = PI*(22.*22.-0.)*6.;
    assert!(volume > gear && volume < 2.*gear,"{volume} ({faces} faces)");
    // each gear's tooth space named for the sweep that cut it
    assert_eq!(d.faces.names.iter().filter(|n| *n == "space").count(),2,"{:?}",d.faces.names);
}
