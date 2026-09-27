//! The generating-sweep harness (docs/generating-sweeps-robustness.md): every case runs the real
//! `solventc` in a child process with a wall-clock budget and a stage trace
//! (`SOLVENT_STAGE_TRACE`), so a failure is located at the stage that met it and a stuck stage
//! is a recorded result, not a hung suite. A case either exports with agreement, or is refused
//! at a named stage before the final gate; a failure at the final gate, a timeout, or an error
//! at no named stage is a fault of the construction's contracts, and the harness reports it.
//!
//! All three tests are ignored (minutes each): `cargo test -p gcs-cli --features occt
//! --test generating_harness -- --ignored --nocapture`. `SOLVENT_HARNESS_ONLY=<substring>`
//! runs the matching cases. The matrix is written to `build/harness/<test>.md`.
use std::{io::Read,path::{Path,PathBuf},process::{Command,Stdio},time::{Duration,Instant}};

use gcs_core::solid::export::Stage;
use fixtures::{tools::{lens,post,ring_lens,sphere,torus},motions::{Observer,cradle_roll}};

type V = [f64;3];

#[derive(Clone,Copy,PartialEq,Debug)]
enum Expect {
    /// Exports, with the independent truth agreeing where the case has one.
    Export,
    /// Refused, at exactly this stage.
    Refuse(&'static str),
    /// Either exports, or is refused at a named stage before the final gate.
    Either,
}

/// An independent answer: a one-Lipschitz function negative in material, and the volume.
struct Truth { sd: Box<dyn Fn(V) -> f64+Sync>, volume: Option<f64> }

struct Case {
    name: String,
    dir: PathBuf,
    entry: String,
    solid: String,
    arguments: Vec<String>,
    expect: Expect,
    truth: Option<Truth>,
    budget: Duration,
}

#[derive(Default,Debug)]
struct Mesh { triangles: usize,area: f64,tiny: usize,tiny_area: f64 }

#[derive(Default)]
struct Record {
    completed: Vec<(String,f64)>,
    failed_at: Option<String>,
    /// The stage the export said it refused at, when it said.
    refused: Option<String>,
    exit: Option<i32>,
    timed_out: bool,
    seconds: f64,
    message: String,
    volume: Option<f64>,
    agreement: Option<String>,
    mesh: Option<Mesh>,
    truth: Option<(usize,usize,usize)>,
    fault: Option<String>,
}

fn workspace() -> PathBuf { Path::new(env!("CARGO_MANIFEST_DIR")).join("../..") }

fn case_dir(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join("solvent-harness").join(name);
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// Run one case: the child process, its trace, its output mesh or the refused one.
fn run(case: &Case) -> Record {
    let trace = case.dir.join("trace.tsv");
    let output = case.dir.join("out.stl");
    let rejected = case.dir.join("rejected.stl");
    let _ = std::fs::remove_file(&trace);
    let mut command = Command::new(env!("CARGO_BIN_EXE_solventc"));
    command.arg(case.dir.join(&case.entry)).args(["--no-diagnose","--solid",&case.solid,"--stl"]).arg(&output)
        .args(&case.arguments)
        // `SOLVENT_HARNESS_BACKEND` runs every case through another STL backend (`refine`, `mesh`).
        .args(std::env::var("SOLVENT_HARNESS_BACKEND").ok().map(|b| vec!["--stl-backend".to_string(),b]).unwrap_or_default())
        .env("SOLVENT_STAGE_TRACE",&trace).env("SOLVENT_KEEP_REJECTED",&rejected)
        .stdout(Stdio::null()).stderr(Stdio::piped());
    let started = Instant::now();
    let mut child = command.spawn().expect("solventc starts");
    let mut stderr = child.stderr.take().unwrap();
    let reader = std::thread::spawn(move || { let mut s = String::new(); let _ = stderr.read_to_string(&mut s); s });
    let mut record = Record::default();
    loop {
        if let Some(status) = child.try_wait().unwrap() { record.exit = status.code(); break; }
        if started.elapsed() > case.budget { let _ = child.kill(); let _ = child.wait(); record.timed_out = true; break; }
        std::thread::sleep(Duration::from_millis(200));
    }
    record.seconds = started.elapsed().as_secs_f64();
    let text = reader.join().unwrap_or_default();
    if let Ok(t) = std::fs::read_to_string(&trace) {
        for (k,at) in t.lines().filter_map(|l| l.split_once('\t')) {
            match k.strip_prefix("refused:") {
                Some(stage) => record.refused = Some(stage.to_string()),
                None => record.completed.push((k.to_string(),at.parse().unwrap_or(f64::NAN))),
            }
        }
    }
    for line in text.lines() {
        let body = line.trim_start_matches("solventc: ");
        let body = body.split_once("] ").map(|(_,b)| b).filter(|_| body.starts_with('[')).unwrap_or(body);
        if let Some(rest) = body.strip_prefix("united the material: ") {
            record.volume = rest.split_whitespace().next().and_then(|v| v.parse().ok());
        }
        if body.starts_with("field agreement:") { record.agreement = Some(body.to_string()); }
        if !line.starts_with("solventc: [") && line.starts_with("solventc: ") && !body.contains("is in the generating-sweep class") {
            record.message = body.to_string();
        }
    }
    if record.timed_out { record.message = format!("no result within {:?}",case.budget); }
    // Where it stopped: the stage the refusal was recorded at (`refused:` in the trace); for a
    // run that said nothing (timed out, crashed), the first stage in order that did not complete.
    if record.exit != Some(0) {
        record.failed_at = Some(record.refused.clone().unwrap_or_else(|| Stage::ORDER.iter()
            .filter(|s| **s != Stage::Step).map(|s| s.key())
            .find(|k| !record.completed.iter().any(|(c,_)| c == k)).unwrap_or("?").to_string()));
    }
    let mesh_path = if output.exists() && record.exit == Some(0) { Some(output) } else if rejected.exists() { Some(rejected) } else { None };
    if let Some(path) = mesh_path {
        let triangles = triangles(&std::fs::read(&path).unwrap());
        record.mesh = Some(mesh_stats(&triangles));
        if let Some(truth) = &case.truth { record.truth = Some(truth_agreement(&triangles,&*truth.sd)); }
    }
    record.fault = fault(case,&record);
    record
}

/// What the case's expectation and the harness's property make of a record.
fn fault(case: &Case,r: &Record) -> Option<String> {
    let exported = r.exit == Some(0) && r.completed.iter().any(|(k,_)| k == "written");
    if r.timed_out { return Some(format!("timed out after {}",r.completed.last().map_or("nothing",|(k,_)| k))); }
    if let (Some(t),true) = (&r.truth,exported) { if t.2 > 0 { return Some(format!("{} truth probes disagree",t.2)); } }
    if let (Some(truth),Some(v),true) = (&case.truth,r.volume,exported) {
        if let Some(expected) = truth.volume {
            if (v-expected).abs() > 1e-3*expected.abs().max(1.) { return Some(format!("volume {v:.6} against {expected:.6}")); }
        }
    }
    match (case.expect,r.failed_at.as_deref()) {
        (Expect::Export,None) if exported => None,
        (Expect::Export,_) => Some(format!("expected an export, refused at {}",r.failed_at.as_deref().unwrap_or("?"))),
        (Expect::Refuse(stage),Some(at)) if at == stage => None,
        (Expect::Refuse(stage),at) => Some(format!("expected a refusal at {stage}, {}",at.map_or("exported".into(),|a| format!("refused at {a}")))),
        (Expect::Either,None) if exported => None,
        (Expect::Either,Some("agreement")) => Some("refused only at the final gate: a stage contract is missing".into()),
        (Expect::Either,Some(_)) if !r.message.is_empty() => None,
        (Expect::Either,_) => Some("refused with no message".into()),
    }
}

/// The triangles of a binary STL written in mm.
fn triangles(bytes: &[u8]) -> Vec<[V;3]> {
    let (vertices,triangles) = gcs_core::solid::agreement::stl_triangles(bytes,1.).unwrap();
    triangles.iter().map(|t| t.map(|i| vertices[i as usize])).collect()
}

use gcs_core::space::{sub,cross,norm};

/// Triangles under a square micrometre are counted apart, with their area: a cluster of them
/// is a defect in its own right, whatever a probe finds.
fn mesh_stats(triangles: &[[V;3]]) -> Mesh {
    let mut m = Mesh {triangles:triangles.len(),..Default::default()};
    for [a,b,c] in triangles {
        let area = 0.5*norm(cross(sub(*b,*a),sub(*c,*a)));
        m.area += area;
        if area < 1e-6 { m.tiny += 1; m.tiny_area += area; }
    }
    m
}

/// The field gate's rule against an independent truth: probes 0.1 mm off each side of a
/// thousand triangles chosen by area, a one-sided disagreement withdrawn when the centroid
/// lies within 0.025 of the truth's boundary. Returns probes, withdrawn, disagreeing.
fn truth_agreement(triangles: &[[V;3]],sd: &(dyn Fn(V) -> f64+Sync)) -> (usize,usize,usize) {
    let areas: Vec<f64> = triangles.iter().map(|[a,b,c]| 0.5*norm(cross(sub(*b,*a),sub(*c,*a)))).collect();
    let total: f64 = areas.iter().sum();
    let mut cumulative = Vec::with_capacity(areas.len());
    let mut sum = 0.; for a in &areas { sum += a; cumulative.push(sum); }
    let wanted = 1000;
    let mut chosen: Vec<usize> = (0..wanted).map(|k| cumulative.partition_point(|&c| c < total*(k as f64+0.5)/wanted as f64)
        .min(triangles.len()-1)).collect();
    chosen.dedup();
    let (mut probes,mut withdrawn,mut disagree) = (0,0,0);
    for i in chosen {
        let [a,b,c] = triangles[i];
        let n = cross(sub(b,a),sub(c,a)); let l = norm(n); if !(l > 0.) { continue; }
        let centroid: V = std::array::from_fn(|k| (a[k]+b[k]+c[k])/3.);
        let at = |d: f64| -> V { std::array::from_fn(|k| centroid[k]+d*n[k]/l) };
        let wrong = [sd(at(-0.1)) > 0.,sd(at(0.1)) < 0.];
        probes += 2;
        let count = wrong.iter().filter(|w| **w).count();
        if count == 1 && sd(centroid).abs() <= 0.025 { withdrawn += 1; continue; }
        disagree += count;
    }
    (probes,withdrawn,disagree)
}

/// The matrix, to stderr and `build/harness/<name>.md`.
/// Whose stage a refusal is at (`Stage::owner`).
fn owner(stage: &str) -> &'static str { Stage::from_key(stage).and_then(Stage::owner).unwrap_or("?") }

fn report(name: &str,cases: &[Case],records: &[Record]) {
    let mut table = String::from("| Case | Expect | Reached | Refused at | Owner | Time | Volume | Agreement | Truth (probes/withdrawn/disagree) | Mesh (triangles, tiny, tiny area) | Fault |\n| --- | --- | --- | --- | --- | ---: | ---: | --- | --- | --- | --- |\n");
    let mut owners: std::collections::BTreeMap<&str,usize> = Default::default();
    for (c,r) in cases.iter().zip(records) {
        let agreement = r.agreement.as_deref().and_then(|a| a.split(", ").last()).unwrap_or("—");
        let who = r.failed_at.as_deref().map_or("exported",owner);
        *owners.entry(who).or_default() += 1;
        table += &format!("| {} | {:?} | {} | {} | {} | {:.0} s | {} | {} | {} | {} | {} |\n",c.name,c.expect,
            r.completed.last().map_or("—",|(k,_)| k),r.failed_at.as_deref().unwrap_or("—"),who,r.seconds,
            r.volume.map_or("—".into(),|v| format!("{v:.4}")),agreement,
            r.truth.map_or("—".into(),|t| format!("{}/{}/{}",t.0,t.1,t.2)),
            r.mesh.as_ref().map_or("—".into(),|m| format!("{}, {}, {:.2e} mm²",m.triangles,m.tiny,m.tiny_area)),
            r.fault.as_deref().unwrap_or("ok"));
    }
    let messages: String = cases.iter().zip(records).filter(|(_,r)| r.exit != Some(0))
        .map(|(c,r)| format!("- **{}**: {}\n",c.name,r.message)).collect();
    let tally: Vec<String> = owners.iter().map(|(k,n)| format!("{k} {n}")).collect();
    let text = format!("# {name}\n\n{table}\nBy owner: {}.\n\n{messages}",tally.join(", "));
    eprintln!("{text}");
    let dir = workspace().join("build/harness");
    std::fs::create_dir_all(&dir).unwrap();
    let name = std::env::var("SOLVENT_HARNESS_BACKEND").map_or(name.to_string(),|b| format!("{name}-{b}"));
    std::fs::write(dir.join(format!("{name}.md")),text).unwrap();
}

/// Run every case the filter keeps, in order, reporting each as it finishes.
fn run_all(name: &str,cases: Vec<Case>) -> Vec<String> {
    let only = std::env::var("SOLVENT_HARNESS_ONLY").ok();
    let cases: Vec<Case> = cases.into_iter().filter(|c| only.as_deref().map_or(true,|o| c.name.contains(o))).collect();
    let mut records = Vec::new();
    for (i,case) in cases.iter().enumerate() {
        let record = run(case);
        eprintln!("harness: [{}/{}] {} — {} in {:.0} s{}",i+1,cases.len(),case.name,
            record.failed_at.as_deref().map_or("exported".into(),|a| format!("refused at {a}")),record.seconds,
            record.fault.as_ref().map_or(String::new(),|f| format!(" — FAULT: {f}")));
        records.push(record);
    }
    report(name,&cases,&records);
    cases.iter().zip(&records).filter_map(|(c,r)| r.fault.as_ref().map(|f| format!("{}: {f}",c.name))).collect()
}

// ---- Small fixtures: a post cut by a tool under a relative roll -------------------------

fn fixture(name: &str,tool: String,motion: String,roll_deg: f64,blank: String,expect: Expect,truth: Option<Truth>) -> Case {
    let dir = case_dir(name);
    std::fs::write(dir.join("case.sv"),format!("{tool}{motion}construction solid removal(tool, under: turn, from: -{roll_deg}deg, to: {roll_deg}deg)\n{blank}")).unwrap();
    Case {name:name.into(),dir,entry:"case.sv".into(),solid:"part".into(),arguments:vec![],expect,truth,budget:Duration::from_secs(300)}
}

fn rot_x(p: V,a: f64) -> V { let (s,c) = a.sin_cos(); [p[0],c*p[1]-s*p[2],s*p[1]+c*p[2]] }
/// Rotation by `a` about the line parallel to x through (0, 0.5, 0.5).
fn rot_skew(p: V,a: f64) -> V { let q = rot_x([p[0],p[1]-0.5,p[2]-0.5],a); [q[0],q[1]+0.5,q[2]+0.5] }
fn rot_z(p: V,a: f64) -> V { let (s,c) = a.sin_cos(); [c*p[0]-s*p[1],s*p[0]+c*p[1],p[2]] }
/// Rotation by `a` about the vertical cradle axis through (2, 0).
fn rot_cradle(p: V,a: f64) -> V { let q = rot_z([p[0]-2.,p[1],p[2]],a); [q[0]+2.,q[1],q[2]] }

/// The post's material, negative inside.
fn post_sd(p: V,cx: f64,radius: f64,low: f64,high: f64) -> f64 {
    ((p[0]-cx).hypot(p[1])-radius).max(low-p[2]).max(p[2]-high)
}

/// A tool's sweep, independently: its least distance `tool` over the roll, sampled every
/// 1e-4 rad (the travel there is under 5e-4 mm, far inside the probes' 0.1), the post less it.
fn swept_truth(tool: fn(V) -> f64,observer: fn(V,f64) -> V,cx: f64,radius: f64,low: f64,high: f64,ratio: f64,roll_deg: f64) -> Truth {
    let half = roll_deg.to_radians();
    let steps = (2.*half/1e-4).ceil() as usize;
    Truth {sd:Box::new(move |p| {
        let mut best = f64::INFINITY;
        for k in 0..=steps {
            let t = -half+2.*half*k as f64/steps as f64;
            best = best.min(tool(rot_cradle(observer(p,t),-ratio*t)));
        }
        post_sd(p,cx,radius,low,high).max(-best)
    }),volume:None}
}

/// The ring lens at height 2.
fn ring_lens_sd(q: V) -> f64 {
    let r = (q[0]-3.).hypot(q[1])-1.;
    (r.hypot(q[2]-1.8)-0.5).max(r.hypot(q[2]-2.2)-0.5)
}
/// The coaxial lens at height 2, 0.4 apart.
fn lens_sd(q: V) -> f64 { ((q[0]-3.).hypot(q[1]).hypot(q[2]-1.6)-1.).max((q[0]-3.).hypot(q[1]).hypot(q[2]-2.4)-1.) }
fn sphere_sd(q: V) -> f64 { (q[0]-3.).hypot(q[1]).hypot(q[2]-2.)-1. }
fn torus_sd(q: V) -> f64 { (((q[0]-3.).hypot(q[1])-1.).hypot(q[2]-2.))-0.5 }

#[test]
#[ignore]
fn fixtures() {
    let cases = vec![
        // Skew axes, as a generator's: the regular case and a tangent face.
        fixture("torus through a post",torus(2.),cradle_roll(0.25,Observer::Skew),75.,post(4.,0.4,0.5,2.),Expect::Export,
            Some(swept_truth(torus_sd,rot_skew,4.,0.4,0.5,2.,0.25,75.))),
        fixture("torus grazing a post's top (tangent)",torus(2.),cradle_roll(0.25,Observer::Skew),75.,post(4.,0.4,0.5,2.5),Expect::Either,
            Some(swept_truth(torus_sd,rot_skew,4.,0.4,0.5,2.5,0.25,75.))),
        // A post tall enough that the torus, turned on past the roll's limit, comes back into
        // it: the rectangular sheet extended past the roll does too, where the true boundary is
        // the clear cap. Refused at the sheet until the sheet is trimmed at the roll's limits.
        fixture("torus through a tall post (extension re-enters)",torus(2.),cradle_roll(0.25,Observer::Skew),75.,post(4.,0.4,-2.,2.),
            Expect::Refuse("sheet"),None),
        // A sphere's contact curve is a great circle, which crosses every meridian of its axis.
        fixture("sphere through a post (skew axes)",sphere(2.),cradle_roll(0.25,Observer::Skew),90.,post(3.5,0.4,1.,2.),Expect::Export,
            Some(swept_truth(sphere_sd,rot_skew,3.5,0.4,1.,2.,0.25,90.))),
        // The crease fan with poles: a lens about the tool's axis. Class B's reproducer in seconds:
        // it agrees with its truth but meshes with hundreds of sub-micron slivers along the crease
        // and an edge used three times, refused where the STL is checked. Moved 0.05 either way
        // it is refused at the fit instead, near a pole where the meridian stations meet.
        fixture("lens through a post (crease slivers)",lens(2.,0.4,false),cradle_roll(0.25,Observer::Skew),60.,post(3.5,0.4,1.,2.5),Expect::Either,
            Some(swept_truth(lens_sd,rot_skew,3.5,0.4,1.,2.5,0.25,60.))),
        // The crease without a pole: a sharp-rimmed ring, as a cutter blade's tip.
        fixture("ring lens through a post (crease fan)",ring_lens(2.),cradle_roll(0.25,Observer::Skew),75.,post(4.,0.4,0.5,2.),
            Expect::Export,Some(swept_truth(ring_lens_sd,rot_skew,4.,0.4,0.5,2.,0.25,75.))),
        // Two spheres about parallel axes, as the gear's cutter bounds its crown by an indexed
        // neighbour: the second face meets a meridian section of the first in two arcs, one each
        // side of the crease, which the profile walk cannot yet name apart. Refused at the reach.
        fixture("lens about two axes",lens(2.,0.,true),cradle_roll(0.25,Observer::Skew),60.,post(3.5,0.4,1.,2.5),Expect::Refuse("reach"),None),
        // Meeting axes: a surface of revolution's contact equation has no constant term, and a
        // ring where its amplitude vanishes is in contact at every time. Admission names it.
        fixture("torus, meeting axes (a stationary ring)",torus(2.),cradle_roll(0.25,Observer::Crossed),60.,post(4.,0.4,-2.,2.),
            Expect::Refuse("admission"),None),
        // Meeting axes again: a sphere point whose normal passes through the point where the axes
        // meet is in contact at every time. Two such points, isolated, pass 0.447 from the post's
        // axis; a post of radius 0.5 takes them in, and admission names them.
        fixture("sphere, meeting axes (stationary points in the blank)",sphere(2.),cradle_roll(0.25,Observer::Crossed),60.,post(3.,0.5,-2.,2.),
            Expect::Refuse("admission"),None),
        // Parallel axes: the sphere's poles are in contact at every time and pass through the
        // post, which admission must name rather than the construction meet.
        fixture("sphere, parallel axes (stationary poles)",sphere(0.),cradle_roll(0.25,Observer::Parallel),60.,post(3.,0.4,-2.,2.),Expect::Refuse("admission"),None),
    ];
    let faults = run_all("fixtures",cases);
    assert!(faults.is_empty(),"{faults:#?}");
}

// ---- The gear pair, as single tooth spaces -----------------------------------------------

/// The spiral-bevel project at a design, one tooth space per member.
fn gear(name: &str,member: &str,offset: f64,shift: f64,spiral: f64,one: bool,arguments: &[&str],expect: Expect) -> Case {
    let dir = case_dir(name);
    fixtures::gear::copy_design(&dir,offset,shift,spiral);
    if one {
        let pair = std::fs::read_to_string(dir.join("members.sv")).unwrap();
        std::fs::write(dir.join("members.sv"),fixtures::gear::one_space(&pair)).unwrap();
    }
    Case {name:name.into(),dir,entry:"gears.sv".into(),solid:format!("pair.{member}.body"),
        arguments:arguments.iter().map(|a| a.to_string()).collect(),expect,truth:None,budget:Duration::from_secs(420)}
}

#[test]
#[ignore]
fn controls() {
    let cases = vec![
        gear("bevel pinion space","pinion",0.,0.,35.,true,&[],Expect::Export),
        gear("bevel gear space","gear",0.,0.,35.,true,&[],Expect::Export),
        gear("hypoid 25 pinion space","pinion",25.,12.5,25.,true,&[],Expect::Export),
        gear("hypoid 25 gear space (crease sliver)","gear",25.,12.5,25.,true,&[],Expect::Either),
        gear("symmetric rack 17.5 mm off (undercut)","pinion",17.5,0.,35.,true,&[],Expect::Refuse("admission")),
        gear("symmetric rack 30 mm off (double contact)","pinion",30.,0.,35.,true,&[],Expect::Refuse("admission")),
    ];
    let faults = run_all("controls",cases);
    assert!(faults.is_empty(),"{faults:#?}");
}

/// The design space: every case refused at a named stage, or exported with agreement.
/// `SOLVENT_HARNESS_GRID=offsets;shifts;spirals` (comma lists; millimetres, then degrees)
/// replaces the default grid.
#[test]
#[ignore]
fn design_space() {
    let grid = std::env::var("SOLVENT_HARNESS_GRID").unwrap_or("0,15,20,25;0,5,10;35,25".into());
    let parts: Vec<Vec<f64>> = grid.split(';').map(|g| g.split(',').map(|v| v.trim().parse().unwrap()).collect()).collect();
    let mut cases = Vec::new();
    for &offset in &parts[0] { for &shift in &parts[1] { for &spiral in &parts[2] { for member in ["pinion","gear"] {
        cases.push(gear(&format!("{member} {offset}/{shift}/{spiral}"),member,offset,shift,spiral,true,&[],Expect::Either));
    }}}}
    let faults = run_all("design_space",cases);
    assert!(faults.is_empty(),"{faults:#?}");
}
