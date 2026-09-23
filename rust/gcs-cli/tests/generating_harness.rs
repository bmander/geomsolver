//! The generating-sweep harness (docs/generating-sweeps-robustness.md): every case runs the real
//! `solventc` in a child process with a wall-clock budget and a stage trace
//! (`SOLVENT_STAGE_TRACE`), so a failure is located at the stage that met it and a stuck stage
//! is a recorded result, not a hung suite. A case either exports with agreement, or is refused
//! at a named stage before the final gate; a failure at the final gate, a timeout, or an error
//! at no named stage is a fault of the construction's contracts, and the harness reports it.
//!
//! All three tests are ignored (minutes each): `cargo test -p gcs-cli --features occt,manifold
//! --test generating_harness -- --ignored --nocapture`. `SOLVENT_HARNESS_ONLY=<substring>`
//! runs the matching cases. The matrix is written to `build/harness/<test>.md`.
use std::{f64::consts::PI,io::Read,path::{Path,PathBuf},process::{Command,Stdio},time::{Duration,Instant}};

/// The stages of an STL export of a body with swept cuts, in the order they complete.
const ORDER: [&str;14] = ["admission","blank","clearance","reach","sheet","fit","withheld","split",
    "classify","fuse","stl","mesh","agreement","written"];

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
        .args(&case.arguments).env("SOLVENT_STAGE_TRACE",&trace).env("SOLVENT_KEEP_REJECTED",&rejected)
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
        record.completed = t.lines().filter_map(|l| l.split_once('\t'))
            .map(|(k,s)| (k.to_string(),s.parse().unwrap_or(f64::NAN))).collect();
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
    // Where it stopped: the admission refusal and the field gate say so; otherwise the first
    // stage in order that did not complete.
    if record.exit != Some(0) {
        record.failed_at = Some(if record.message.contains("outside the generating-sweep class") { "admission".into() }
            else if record.message.contains("disagrees with the material field") { "agreement".into() }
            else if record.message.starts_with("the mesh has") { "mesh".into() }
            else { ORDER.iter().find(|k| !record.completed.iter().any(|(c,_)| c == *k)).unwrap_or(&"?").to_string() });
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

fn triangles(bytes: &[u8]) -> Vec<[V;3]> {
    let count = u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize;
    (0..count).map(|i| std::array::from_fn(|k| std::array::from_fn(|j| {
        let at = 84+50*i+12+12*k+4*j;
        f32::from_le_bytes(bytes[at..at+4].try_into().unwrap()) as f64
    }))).collect()
}

fn sub(a: V,b: V) -> V { std::array::from_fn(|k| a[k]-b[k]) }
fn cross(a: V,b: V) -> V { [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]] }
fn norm(a: V) -> f64 { (a[0]*a[0]+a[1]*a[1]+a[2]*a[2]).sqrt() }

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
fn report(name: &str,cases: &[Case],records: &[Record]) {
    let mut table = String::from("| Case | Expect | Reached | Refused at | Time | Volume | Agreement | Truth (probes/withdrawn/disagree) | Mesh (triangles, tiny, tiny area) | Fault |\n| --- | --- | --- | --- | ---: | ---: | --- | --- | --- | --- |\n");
    for (c,r) in cases.iter().zip(records) {
        let agreement = r.agreement.as_deref().and_then(|a| a.split(", ").last()).unwrap_or("—");
        table += &format!("| {} | {:?} | {} | {} | {:.0} s | {} | {} | {} | {} | {} |\n",c.name,c.expect,
            r.completed.last().map_or("—",|(k,_)| k),r.failed_at.as_deref().unwrap_or("—"),r.seconds,
            r.volume.map_or("—".into(),|v| format!("{v:.4}")),agreement,
            r.truth.map_or("—".into(),|t| format!("{}/{}/{}",t.0,t.1,t.2)),
            r.mesh.as_ref().map_or("—".into(),|m| format!("{}, {}, {:.2e} mm²",m.triangles,m.tiny,m.tiny_area)),
            r.fault.as_deref().unwrap_or("ok"));
    }
    let messages: String = cases.iter().zip(records).filter(|(_,r)| r.exit != Some(0))
        .map(|(c,r)| format!("- **{}**: {}\n",c.name,r.message)).collect();
    let text = format!("# {name}\n\n{table}\n{messages}");
    eprintln!("{text}");
    let dir = workspace().join("build/harness");
    std::fs::create_dir_all(&dir).unwrap();
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

/// A unit sphere centred at (3, 0, `h`), its axis vertical.
fn sphere(h: f64) -> String {
    format!("unit mm
use std
construction centerline line spindle(std.origin, std.up.toward)
private point center
center distance(3mm, along: u) std.front
center distance({h}mm, along: v) std.front
private point bottom hint(x: 3, y: {b})
private point top hint(x: 3, y: {t})
private line diameter(bottom, top)
center midpoint diameter
diameter parallel spindle
distance(2mm) diameter
private arc meridian(center: center, start: bottom, end: top)
radius(1mm) meridian
construction solid tool(face(meridian, diameter), about: diameter)
",b=h-1.,t=h+1.)
}

/// A torus about the vertical line through (3, 0): a circle of radius 0.5 whose centre is 1
/// from that axis, at height `h`. Its profile never reaches its axis, as a cutter's does not.
fn torus(h: f64) -> String {
    format!("unit mm
use std
construction centerline line spindle(std.origin, std.up.toward)
private point ta hint(x: 3, y: {b})
private point tb hint(x: 3, y: {t})
ground ta
ground tb
private line taxis(ta, tb)
private point tc hint(x: 4, y: {h})
ground tc
private circle ring(center: tc) hint(r: 0.5)
radius(0.5mm) ring
construction solid tool(face(ring), about: taxis)
",b=h-1.,t=h+1.)
}

/// The unit sphere at (3, 0, `h`) bounded by the unit sphere at (3.8, 0, `h`): a lens whose
/// rim is a sharp crease circle of radius 0.917 in the plane x = 3.4.
fn lens(h: f64) -> String {
    let (b,t) = (h-1.,h+1.);
    format!("unit mm
use std
construction centerline line spindle(std.origin, std.up.toward)
private point center
center distance(3mm, along: u) std.front
center distance({h}mm, along: v) std.front
private point bottom hint(x: 3, y: {b})
private point top hint(x: 3, y: {t})
private line diameter(bottom, top)
center midpoint diameter
diameter parallel spindle
private arc meridian(center: center, start: bottom, end: top)
radius(1mm) meridian
construction solid stock(face(meridian, diameter), about: diameter)
construction solid tool(stock)
private point center2
center2 distance(3.8mm, along: u) std.front
center2 distance({h}mm, along: v) std.front
private point bottom2 hint(x: 3.8, y: {b})
private point top2 hint(x: 3.8, y: {t})
private line diameter2(bottom2, top2)
center2 midpoint diameter2
diameter2 parallel spindle
private arc meridian2(center: center2, start: bottom2, end: top2)
radius(1mm) meridian2
construction solid other(face(meridian2, diameter2), about: diameter2)
other bound tool
")
}

/// The tool spinning about its own vertical axis through (3, 0) at `ratio` turns a turn, seen
/// from an observer turning about `observer`: the spindle (world z, parallel to the tool's
/// axis) or world x (crossed, as a generator's axes are).
fn roll(ratio: f64,crossed: bool) -> String {
    let observer = if crossed { "private point xend hint(x: 5, y: 0)
xend distance(5mm, along: u) std.front
xend distance(0mm, along: v) std.front
construction centerline line xaxis(std.origin, xend)
private motion observer(about: xaxis)
" } else { "private motion observer(about: spindle)\n" };
    format!("private point hub hint(x: 3, y: 0)
hub distance(3mm, along: u) std.front
hub distance(0mm, along: v) std.front
private point hub_up hint(x: 3, y: 5)
hub_up distance(3mm, along: u) std.front
hub_up distance(5mm, along: v) std.front
construction centerline line own(hub, hub_up)
private motion spin(about: own, ratio: {ratio})
{observer}motion turn(spin, relative_to: observer)
")
}

/// A post of radius `radius` about the vertical line x = `cx`, z in [low, high], less the removal.
fn post(cx: f64,radius: f64,low: f64,high: f64) -> String {
    let r = cx+radius;
    format!("private point q0 hint(x: {cx}, y: {low})
private point q1 hint(x: {r}, y: {low})
private point q2 hint(x: {r}, y: {high})
private point q3 hint(x: {cx}, y: {high})
ground q0
ground q1
ground q2
ground q3
private line qb(q0, q1)
private line qw(q1, q2)
private line qt(q2, q3)
private line qa(q3, q0)
construction solid stock_post(face(qb, qw, qt, qa), about: qa)
solid part(stock_post)
removal cut part
")
}

fn fixture(name: &str,tool: String,motion: String,roll_deg: f64,blank: String,expect: Expect,truth: Option<Truth>) -> Case {
    let dir = case_dir(name);
    std::fs::write(dir.join("case.sv"),format!("{tool}{motion}construction solid removal(tool, under: turn, from: -{roll_deg}deg, to: {roll_deg}deg)\n{blank}")).unwrap();
    Case {name:name.into(),dir,entry:"case.sv".into(),solid:"part".into(),arguments:vec![],expect,truth,budget:Duration::from_secs(300)}
}

fn rot_x(p: V,a: f64) -> V { let (s,c) = a.sin_cos(); [p[0],c*p[1]-s*p[2],s*p[1]+c*p[2]] }
fn rot_z(p: V,a: f64) -> V { let (s,c) = a.sin_cos(); [c*p[0]-s*p[1],s*p[0]+c*p[1],p[2]] }
/// Rotation by `a` about the vertical line through (3, 0).
fn rot_own(p: V,a: f64) -> V { let q = rot_z([p[0]-3.,p[1],p[2]],a); [q[0]+3.,q[1],q[2]] }

/// The post's material, negative inside.
fn post_sd(p: V,cx: f64,radius: f64,low: f64,high: f64) -> f64 {
    ((p[0]-cx).hypot(p[1])-radius).max(low-p[2]).max(p[2]-high)
}

/// A unit sphere whose centre turns about world x on the circle of radius `h` in the plane
/// x = 3: near the post (the circle's top, z > 0) its sweep is exactly the tube about that
/// circle, whatever the sphere spins. The removal's volume inside the post by quadrature.
fn tube_truth(h: f64,radius: f64,low: f64,high: f64) -> Truth {
    let n = 2000;
    let mut removed = 0.;
    for i in 0..n { for j in 0..n {
        let (x,y) = (3.-radius+2.*radius*(i as f64+0.5)/n as f64,-radius+2.*radius*(j as f64+0.5)/n as f64);
        if (x-3.).hypot(y) > radius || (x-3.).abs() >= 1. { continue; }
        let w = (1.-(x-3.)*(x-3.)).sqrt();
        let (inner,outer) = ((h-w).max(0.),h+w);
        if outer <= y.abs() { continue; }
        let z0 = if inner > y.abs() { (inner*inner-y*y).sqrt() } else { 0. };
        let z1 = (outer*outer-y*y).sqrt();
        removed += (z1.min(high)-z0.max(low)).max(0.)*(2.*radius/n as f64).powi(2);
    }}
    let volume = PI*radius*radius*(high-low)-removed;
    Truth {sd:Box::new(move |p| {
        let tube = if p[2] > 0. { (p[1].hypot(p[2])-h).hypot(p[0]-3.)-1. } else { f64::INFINITY };
        post_sd(p,3.,radius,low,high).max(-tube)
    }),volume:Some(volume)}
}

/// The lens's sweep, independently: its least distance over the roll, sampled every 1e-4 rad
/// (the travel there is under 5e-4 mm, far inside the probes' 0.1).
fn lens_truth(h: f64,radius: f64,low: f64,high: f64,ratio: f64,roll_deg: f64) -> Truth {
    let half = roll_deg.to_radians();
    let steps = (2.*half/1e-4).ceil() as usize;
    Truth {sd:Box::new(move |p| {
        let mut best = f64::INFINITY;
        for k in 0..=steps {
            let t = -half+2.*half*k as f64/steps as f64;
            let q = rot_own(rot_x(p,t),-ratio*t);
            let lens = ((q[0]-3.).hypot(q[1]).hypot(q[2]-h)-1.).max((q[0]-3.8).hypot(q[1]).hypot(q[2]-h)-1.);
            best = best.min(lens);
        }
        post_sd(p,3.,radius,low,high).max(-best)
    }),volume:None}
}

/// A tool's sweep, independently: its least distance `tool` over the roll, sampled every
/// 1e-4 rad, the post less it.
fn swept_truth(tool: fn(V) -> f64,cx: f64,radius: f64,low: f64,high: f64,ratio: f64,roll_deg: f64) -> Truth {
    let half = roll_deg.to_radians();
    let steps = (2.*half/1e-4).ceil() as usize;
    Truth {sd:Box::new(move |p| {
        let mut best = f64::INFINITY;
        for k in 0..=steps {
            let t = -half+2.*half*k as f64/steps as f64;
            best = best.min(tool(rot_own(rot_x(p,t),-ratio*t)));
        }
        post_sd(p,cx,radius,low,high).max(-best)
    }),volume:None}
}

fn torus_sd(q: V) -> f64 { (((q[0]-3.).hypot(q[1])-1.).hypot(q[2]-2.))-0.5 }

#[test]
#[ignore]
fn fixtures() {
    let cases = vec![
        // Crossed axes, as a generator's: the regular case, the crease fan, a tangent face.
        fixture("torus through a post",torus(2.),roll(0.25,true),60.,post(4.,0.4,-2.,2.),Expect::Export,
            Some(swept_truth(torus_sd,4.,0.4,-2.,2.,0.25,60.))),
        // A sphere's poles lie on its spin axis with normals radial to the observer's turn: in
        // contact at every time, tracing the tube's outer and inner equators. The inner one
        // passes through the post, so admission names it.
        fixture("sphere through a post (inner pole in the blank)",sphere(2.),roll(0.25,true),60.,post(3.,0.4,-2.,2.),
            Expect::Refuse("admission"),None),
        fixture("lens through a post (crease fan)",lens(2.),roll(0.25,true),60.,post(3.,0.4,-2.,2.),Expect::Export,
            Some(lens_truth(2.,0.4,-2.,2.,0.25,60.))),
        fixture("torus grazing a post's top (tangent)",torus(2.),roll(0.25,true),60.,post(4.,0.4,-2.,2.5),Expect::Either,
            Some(swept_truth(torus_sd,4.,0.4,-2.,2.5,0.25,60.))),
        // Parallel axes: the sphere's poles are in contact at every time and pass through the
        // post, which admission must name rather than the construction meet.
        fixture("sphere, parallel axes (stationary poles)",sphere(0.),roll(0.25,false),60.,post(3.,0.4,-2.,2.),Expect::Refuse("admission"),None),
    ];
    let faults = run_all("fixtures",cases);
    assert!(faults.is_empty(),"{faults:#?}");
}

// ---- The gear pair, as single tooth spaces -----------------------------------------------

/// The spiral-bevel project at a design, one tooth space per member.
fn gear(name: &str,member: &str,offset: f64,shift: f64,spiral: f64,one: bool,arguments: &[&str],expect: Expect) -> Case {
    let dir = case_dir(name);
    let source = workspace().join("rust/examples/spiral_bevel");
    for entry in std::fs::read_dir(&source).unwrap() {
        let path = entry.unwrap().path();
        if path.extension().map_or(false,|e| e == "sv") { std::fs::copy(&path,dir.join(path.file_name().unwrap())).unwrap(); }
    }
    let set = ["param offset_angle","param pressure_shift","param spiral_angle"];
    let configuration = std::fs::read_to_string(dir.join("configuration.sv")).unwrap();
    let configuration: String = configuration.lines().filter(|l| !set.iter().any(|s| l.starts_with(s))).map(|l| format!("{l}\n")).collect::<String>()
        + &format!("param offset_angle = {offset}deg\nparam pressure_shift = {shift}deg\nparam spiral_angle = {spiral}deg\n");
    std::fs::write(dir.join("configuration.sv"),configuration).unwrap();
    if one {
        let pair = std::fs::read_to_string(dir.join("matched_pair.sv")).unwrap();
        std::fs::write(dir.join("matched_pair.sv"),pair.replace("repeat teeth as i {","repeat 1 as i {")).unwrap();
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
        gear("hypoid 25 pinion space","pinion",25.,10.,25.,true,&[],Expect::Export),
        gear("hypoid 25 gear space (crease sliver)","gear",25.,10.,25.,true,&[],Expect::Either),
        gear("symmetric rack at 17.5 (undercut)","pinion",17.5,0.,35.,true,&[],Expect::Refuse("admission")),
        gear("symmetric rack at 30 (double contact)","pinion",30.,0.,35.,true,&[],Expect::Refuse("admission")),
        gear("Manifold arrangement at 15 (negative control)","pinion",15.,0.,35.,true,&["--stl-backend","manifold"],Expect::Refuse("mesh")),
    ];
    let faults = run_all("controls",cases);
    assert!(faults.is_empty(),"{faults:#?}");
}

/// The design space: every case refused at a named stage, or exported with agreement.
/// `SOLVENT_HARNESS_GRID=offsets;shifts;spirals` (comma lists) replaces the default grid.
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
