//! Milestone 4: sheets that cross. Where one sheet turns inner across
//! another, the kept parts must meet along the crease between them, and the
//! shell must still close and certify.
use super::{closed::{closed,closed_shell_at,shell_at,volume},harness::{self,V3},motions,tools};
use gcs_core::{model::SolidDef,motion::Family};

const SAGITTA: f64 = 0.02;

// Each case's document: the tool, its motion and the sweep, one source
// shared by the case's test and the export below.
fn turning_prism() -> String { format!("{}{}{}",tools::TRIANGLE_PRISM,motions::TURN_OFFSET,motions::swept("turn",-50.,50.)) }
fn tumbling_cylinder() -> String { format!("{}{}{}",tools::CYLINDER,motions::TUMBLE,motions::swept("turn",-30.,30.)) }
fn turned_box() -> String { format!("{}{}{}",tools::BOX,motions::turn_about(4.,-1.5,5.,-1.5),motions::swept("turn",0.,30.)) }
fn turned_lens() -> String { format!("{}{}{}",tools::LENS,motions::TURN_SPINDLE,motions::swept("turn",-60.,60.)) }
fn sliding_dumbbell() -> String { format!("{}{}{}",tools::DUMBBELL,motions::slide_x(4.),motions::swept("feed",0.,360.)) }

/// The swept volume by sampled membership: a midpoint grid over the box,
/// each point material when the tool's closed-form membership holds at any
/// of `poses` parameters. Uses only `Family::at`, never the field.
fn sampled_volume(source: &str,member: &dyn Fn(V3) -> bool,lo: V3,hi: V3,grid: usize,poses: usize) -> f64 {
    let e = harness::read(source);
    let swept = harness::solid(&e,"swept");
    let SolidDef::Swept {motion,from,to,..} = &e.sketch.solids[swept].def else { panic!("not a sweep") };
    let family = Family::read(&e.sketch,*motion as usize).unwrap();
    let inverses: Vec<_> = (0..=poses).map(|k| family.at(from.value+(to.value-from.value)*k as f64/poses as f64).unwrap().inverse()).collect();
    let step: V3 = std::array::from_fn(|k| (hi[k]-lo[k])/grid as f64);
    let mut count = 0usize;
    for i in 0..grid { for j in 0..grid { for k in 0..grid {
        let p = [lo[0]+(i as f64+0.5)*step[0],lo[1]+(j as f64+0.5)*step[1],lo[2]+(k as f64+0.5)*step[2]];
        if inverses.iter().any(|m| member(m.point(p))) { count += 1; }
    } } }
    count as f64*step[0]*step[1]*step[2]
}

/// The area a plane region sweeps turned by `alpha` either way about the
/// origin, by ring quadrature: on each circle about the pivot (out to
/// `r_max`) the region's arcs, grown by the turn and their union measured,
/// at a resolution of `steps` around the circle.
fn turned_area(member: &dyn Fn(f64,f64) -> bool,r_max: f64,alpha: f64) -> f64 {
    let (rings,steps) = (1000,3600);
    let reach = (alpha/2./(std::f64::consts::TAU/steps as f64)).round() as usize;
    let mut area = 0.;
    for i in 0..rings {
        let r = r_max*(i as f64+0.5)/rings as f64;
        let on: Vec<bool> = (0..steps).map(|k| { let t = std::f64::consts::TAU*k as f64/steps as f64; member(r*t.cos(),r*t.sin()) }).collect();
        // grown by the turn: covered where some point of the region is
        // within reach, read off a running count round the circle
        let mut prefix = vec![0usize;3*steps+1];
        for k in 0..3*steps { prefix[k+1] = prefix[k]+on[k%steps] as usize; }
        let covered = (0..steps).filter(|&k| prefix[steps+k+reach+1] > prefix[steps+k-reach]).count();
        area += covered as f64/steps as f64*std::f64::consts::TAU*r*r_max/rings as f64;
    }
    area
}

#[test]
fn a_turning_prism_closes_along_its_creases() {
    // the prism's section (3, -0.8), (4.5, 0), (3, 0.8) (in x and z) extruded
    // 3 along y, turned 100° about the vertical through (2.5, 0): at each
    // height its section is a rectangle, swept by ring quadrature
    let source = turning_prism();
    let (mesh,certificate) = closed_shell_at(&source,SAGITTA);
    assert!(certificate.is_complete(),"{} triangles failed",certificate.failures.len());
    closed(&mesh).unwrap();
    let alpha = 100_f64.to_radians();
    let layers = 100;
    let expected: f64 = (0..layers).map(|j| {
        let z = -0.8+1.6*(j as f64+0.5)/layers as f64;
        let x_far = 4.5-1.875*z.abs();
        turned_area(&|x,y| x >= 0.5 && x <= x_far-2.5 && y.abs() <= 1.5,((x_far-2.5).powi(2)+2.25).sqrt(),alpha)*1.6/layers as f64
    }).sum();
    let v = volume(&mesh);
    eprintln!("turning prism: volume {v:.4}, expected {expected:.4}");
    assert!(v <= expected*(1.+1e-3) && v >= expected*(1.-3.*SAGITTA/0.5),"volume {v} against {expected}");
}

/// Not closed yet (milestone 5, the tracer): the sphere the stationary
/// ring sweeps is inner everywhere but at second order (within a quarter
/// sagitta of the boundary over a patch a fifth wide about each fixed
/// point of the axis, which the labels keep), the generators through the
/// fixed points sweep bowtie sectors whose halves face opposite ways, and
/// the rims' sweeps fold where a rim's tangent runs along its velocity.
/// Runs in four seconds and leaves nine loops and a few dozen refused
/// triangles; `SOLVENT_SHEETS=1` prints the stages.
#[test]
#[ignore]
fn a_tumbling_cylinder_closes_along_its_creases() {
    // radius 1, height 2 about the vertical through (3, 0), tumbled ±30°
    // about the horizontal line through its centre
    let source = tumbling_cylinder();
    let (mesh,certificate) = closed_shell_at(&source,SAGITTA);
    assert!(certificate.is_complete(),"{} triangles failed",certificate.failures.len());
    closed(&mesh).unwrap();
    let member = |p: V3| (p[0]-3.).powi(2)+p[1]*p[1] <= 1. && p[2].abs() <= 1.;
    let expected = sampled_volume(&source,&member,[1.5,-1.5,-1.5],[4.5,1.5,1.5],64,256);
    let v = volume(&mesh);
    eprintln!("tumbling cylinder: volume {v:.4}, sampled {expected:.4}");
    assert!((v-expected).abs() <= 0.03*expected,"volume {v} against sampled {expected}");
}

/// Milestone 5a: the end faces are square to the turn's axis, so the motion carries them within
/// their own planes and `n·v` is zero over the whole of them. Each is swept exactly in its plane
/// (`swept_boundary::grazing`) rather than traced: the tracer emitted the bands their edges sweep
/// in that plane, folded where a segment turning in its plane crosses its earlier positions, and
/// the slivers those left along the envelope arc pinched the rims the zip then closed wrongly.
#[test]
fn a_box_turned_about_its_face_centre_closes_along_its_creases() {
    // the 2 x 3 x 2 box turned 30° about the axis through the centre of its
    // x = 4 face along x: a 2 x 3 rectangle turned about its own centre
    // (the corners' arcs cross the faces and the faces turn inner), times 2
    let source = turned_box();
    let (mesh,certificate) = closed_shell_at(&source,SAGITTA);
    assert!(certificate.is_complete(),"{} triangles failed",certificate.failures.len());
    closed(&mesh).unwrap();
    let expected = 2.*turned_area(&|y,z| y.abs() <= 1. && z.abs() <= 1.5,(1_f64+2.25).sqrt(),30_f64.to_radians());
    let v = volume(&mesh);
    eprintln!("turned box: volume {v:.4}, expected {expected:.4}");
    assert!(v <= expected*(1.+1e-3) && v >= expected*(1.-3.*SAGITTA/1.),"volume {v} against {expected}");
}

#[test]
fn a_lens_turned_about_the_spindle_closes_along_its_crease() {
    // the lens of two unit spheres 0.8 apart, turned ±60° about the spindle:
    // its convex crease circle sweeps a fan
    let source = turned_lens();
    let (mesh,certificate) = closed_shell_at(&source,SAGITTA);
    assert!(certificate.is_complete(),"{} triangles failed",certificate.failures.len());
    closed(&mesh).unwrap();
    // every circle about the spindle meets the lens in one arc: at height z
    // it spans r from 3.8 - ρ to 3 + ρ (ρ the spheres' section radius), so
    // the sweep adds α (r_max² - r_min²) / 2 to the lens's own section
    let alpha = 120_f64.to_radians();
    let (h,layers) = (0.6_f64*(2.-0.6)/1_f64,2000);
    let h = h.sqrt(); // the crease's radius: sqrt(1 - 0.4²)
    let expected: f64 = (0..layers).map(|j| {
        let z = -h+2.*h*(j as f64+0.5)/layers as f64;
        let rho = (1.-z*z).sqrt();
        let (r_min,r_max) = (3.8-rho,3.+rho);
        // the section: two circular segments of the disks of radius ρ, cut 0.4 from their centres
        let segment = |d: f64| rho*rho*(d/rho).acos()-d*(rho*rho-d*d).sqrt();
        (2.*segment(0.4)+alpha*(r_max*r_max-r_min*r_min)/2.)*2.*h/layers as f64
    }).sum();
    let v = volume(&mesh);
    eprintln!("turned lens: volume {v:.4}, expected {expected:.4}");
    assert!(v <= expected*(1.+1e-3) && v >= expected*(1.-3.*SAGITTA/0.5),"volume {v} against {expected}");
}

/// Not closed yet (project 2, the Boolean meshes): the union's mesh is
/// forty-five thousand facets of the bar's wall shredded by the balls'
/// facet planes, so the caps take most of a minute and their slivers leave
/// four loops at the crease junctions and twenty-odd refused triangles.
#[test]
#[ignore]
fn a_dumbbell_translated_along_x_closes_along_its_creases() {
    // balls of radius 0.5 on a bar of radius 0.25, advanced 4 along x: the
    // two concave crease circles sweep, and every line along x meets the
    // tool in one segment
    let source = sliding_dumbbell();
    let (mesh,certificate) = closed_shell_at(&source,SAGITTA);
    assert!(certificate.is_complete(),"{} triangles failed",certificate.failures.len());
    closed(&mesh).unwrap();
    let member = |p: V3| ((p[0]-3.).powi(2)+p[1]*p[1] <= 0.0625 && p[2].abs() <= 1.) || (p[0]-3.).powi(2)+p[1]*p[1]+(p[2]-1.).powi(2) <= 0.25 || (p[0]-3.).powi(2)+p[1]*p[1]+(p[2]+1.).powi(2) <= 0.25;
    let expected = sampled_volume(&source,&member,[2.4,-0.6,-1.6],[7.6,0.6,1.6],64,256);
    let v = volume(&mesh);
    eprintln!("dumbbell: volume {v:.4}, sampled {expected:.4}");
    assert!((v-expected).abs() <= 0.03*expected,"volume {v} against sampled {expected}");
}

/// Every milestone-4 case written out for a person to look at: its Solvent
/// document and the swept boundary the pipeline makes of it, as binary STL,
/// into `SOLVENT_EXPORT` (default `rust/examples/swept_boundary`). The mesh is written whether or not it closed; a line per case says
/// which did. The dumbbell takes most of a minute, the rest seconds each.
#[test]
#[ignore]
fn export_milestone_4_cases() {
    let dir = std::path::PathBuf::from(std::env::var("SOLVENT_EXPORT").unwrap_or_else(|_| concat!(env!("CARGO_MANIFEST_DIR"),"/../examples/swept_boundary").into()));
    std::fs::create_dir_all(&dir).unwrap();
    let cases: [(&str,String); 5] = [
        ("turning_prism",turning_prism()),
        ("turned_lens",turned_lens()),
        ("tumbling_cylinder",tumbling_cylinder()),
        ("turned_box",turned_box()),
        ("sliding_dumbbell",sliding_dumbbell()),
    ];
    let mut summary = Vec::new();
    for (name,source) in &cases {
        eprintln!("== {name}");
        std::fs::write(dir.join(format!("{name}.sv")),source).unwrap();
        let (mesh,certificate) = shell_at(source,SAGITTA);
        let mesh = mesh.compact();
        let verdict = match &certificate { Ok(c) => format!("{} failed certificate",c.failures.len()),Err(e) => format!("refused {e:?}") };
        // written unchecked: an open case's slivers may collapse in float32,
        // and a person looking at it wants them there, counted
        let (bytes,collapsed) = stl(&mesh.vertices,&mesh.triangles,name);
        std::fs::write(dir.join(format!("{name}.stl")),bytes).unwrap();
        let shell = match closed(&mesh) { Ok(()) => "closed".to_string(),Err(e) => format!("open ({e})") };
        summary.push(format!("{name}: {} triangles ({collapsed} collapse in float32), {shell}, {verdict}, volume {:.4}",mesh.triangles.len(),volume(&mesh)));
    }
    eprintln!("written to {}",dir.display());
    for line in &summary { eprintln!("  {line}"); }
}

/// Binary STL of an indexed mesh as it stands, with how many of its
/// triangles float32 coordinates reduce to no area.
fn stl(vertices: &[V3],triangles: &[[u32;3]],name: &str) -> (Vec<u8>,usize) {
    let mut out = Vec::with_capacity(84+triangles.len()*50);
    let mut header = [0u8;80];
    for (i,b) in format!("solvent {name}").bytes().take(79).enumerate() { header[i] = b; }
    out.extend(header);
    out.extend((triangles.len() as u32).to_le_bytes());
    let mut collapsed = 0;
    for t in triangles {
        let [a,b,c] = t.map(|v| vertices[v as usize].map(|x| x as f32));
        let (u,w) = ([b[0]-a[0],b[1]-a[1],b[2]-a[2]],[c[0]-a[0],c[1]-a[1],c[2]-a[2]]);
        let n = [u[1]*w[2]-u[2]*w[1],u[2]*w[0]-u[0]*w[2],u[0]*w[1]-u[1]*w[0]];
        let len = (n[0]*n[0]+n[1]*n[1]+n[2]*n[2]).sqrt();
        if !(len > 0.) { collapsed += 1; }
        for x in if len > 0. { n.map(|x| x/len) } else { [0.;3] } { out.extend(x.to_le_bytes()); }
        for p in [a,b,c] { for x in p { out.extend(x.to_le_bytes()); } }
        out.extend(0u16.to_le_bytes());
    }
    (out,collapsed)
}

/// The certified cases do not stand on knife edges: every seed point moved by about 1e-12 (far
/// below every tolerance the construction uses) leaves the mesh as it was, triangle for
/// triangle.
#[test]
fn seeds_moved_below_every_tolerance_leave_the_mesh_as_it_was() {
    use gcs_core::solid::swept_boundary::{SweptBoundaryOptions,construct_from,seeds};
    for source in [turning_prism(),turned_lens(),turned_box()] {
        let e = harness::read(&source);
        let swept = harness::solid(&e,"swept");
        let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};
        let (_,sheets,_) = seeds(&e.sketch,swept,options.spacing,options.sagitta,&|_| {}).unwrap();
        let mut moved = sheets.clone();
        let mut k = 0_f64;
        for s in &mut moved { for p in &mut s.points { k += 1.; *p = [p[0]+1e-12*(1.7*k).sin(),p[1]+1e-12*(2.3*k).cos(),p[2]+1e-12*(3.1*k+1.).sin()]; } }
        let build = |sheets| construct_from(&e.sketch,swept,&options,sheets,&mut |_,_| {}).unwrap().mesh;
        let (a,b) = (build(sheets),build(moved));
        assert_eq!(a.triangles,b.triangles);
        assert_eq!(a.sheet,b.sheet);
        let worst = a.vertices.iter().zip(&b.vertices).map(|(p,q)| (0..3).map(|k| (p[k]-q[k]).abs()).fold(0.,f64::max)).fold(0.,f64::max);
        assert!(a.vertices.len() == b.vertices.len() && worst <= 1e-9,"{} vertices against {}, one moved {worst:e}",a.vertices.len(),b.vertices.len());
    }
}

/// The same of all five milestone-4 cases, the three deferred ones included: every stage's
/// labels, keep flags, triangles and vertices as they were. About a minute (the dumbbell); prints
/// a line per case.
#[test]
#[ignore]
fn seeds_moved_below_every_tolerance_leave_every_milestone_4_case_as_it_was() {
    use gcs_core::solid::swept_boundary::{KeptMesh,Stage,SweptBoundaryOptions,construct_from,seeds};
    #[derive(Clone)]
    enum Print { Codes(Vec<u64>),Mesh(Vec<[u32;3]>,Vec<u32>,Vec<V3>) }
    let print = |stage: &Stage<'_>| -> (&'static str,Print) {
        let mesh = |m: &KeptMesh| Print::Mesh(m.triangles.clone(),m.sheet.clone(),m.vertices.clone());
        match stage {
            Stage::Seeded {..} => ("seeded",Print::Codes(vec![])),
            Stage::Grazed {regions} => ("grazed",Print::Mesh(regions.iter().flat_map(|g| g.patch.triangles.clone()).collect(),vec![],regions.iter().flat_map(|g| g.patch.points.clone()).collect())),
            Stage::Capped {caps} => ("capped",Print::Mesh(caps.iter().flat_map(|c| c.patch.triangles.clone()).collect(),vec![],caps.iter().flat_map(|c| c.patch.points.clone()).collect())),
            Stage::Labelled {labelled,..} => ("labelled",Print::Codes(labelled.iter().flat_map(|l| l.labels.iter().map(|x| *x as u64)).collect())),
            Stage::Clipped {mesh:m,..} => ("clipped",mesh(m)),
            Stage::Merged {mesh:m,..} => ("merged",mesh(m)),
            Stage::Kept {keep,..} => ("kept",Print::Codes(keep.iter().map(|k| *k as u64).collect())),
            Stage::Unioned {mesh:m,..} => ("unioned",mesh(m)),
            Stage::Uncovered {mesh:m,..} => ("uncovered",mesh(m)),
            Stage::Welded {mesh:m,..} => ("welded",mesh(m)),
            Stage::Split {mesh:m} => ("split",mesh(m)),
            Stage::Zipped {mesh:m,..} => ("zipped",mesh(m)),
            Stage::Certified {mesh:m,..} => ("certified",mesh(m)),
        }
    };
    // None when the two agree: the same codes, or the same triangles on vertices within 1e-9
    let differ = |a: &Print,b: &Print| -> Option<String> {
        match (a,b) {
            (Print::Codes(x),Print::Codes(y)) => (x != y).then(|| format!("{} of {} decisions",x.iter().zip(y).filter(|(p,q)| p != q).count()+x.len().abs_diff(y.len()),x.len())),
            (Print::Mesh(ta,sa,va),Print::Mesh(tb,sb,vb)) => {
                // within the construction's coincidence (the weld's 1e-7): an intersection of two
                // nearly parallel lines turns a 1e-12 move into a larger, still negligible one
                let moved = va.len() != vb.len() || va.iter().zip(vb).any(|(p,q)| (0..3).any(|k| (p[k]-q[k]).abs() > 1e-7));
                if ta == tb && sa == sb && !moved { return None; }
                // the triangles by sheet and centroid, each matched within 1e-9 of one in the other
                let cs = |t: &Vec<[u32;3]>,s: &Vec<u32>,v: &Vec<V3>| -> Vec<(u32,V3)> { t.iter().enumerate().map(|(i,tri)| (s.get(i).copied().unwrap_or(0),std::array::from_fn(|k| tri.iter().map(|&x| v[x as usize][k]).sum::<f64>()/3.))).collect() };
                let (ca,cb) = (cs(ta,sa,va),cs(tb,sb,vb));
                let unmatched = |p: &Vec<(u32,V3)>,q: &Vec<(u32,V3)>| -> Vec<(u32,V3)> {
                    let mut grid: std::collections::BTreeMap<[i64;3],Vec<usize>> = Default::default();
                    for (j,(_,c)) in q.iter().enumerate() { grid.entry(c.map(|x| (x*1e4).floor() as i64)).or_default().push(j); }
                    p.iter().filter(|(s,c)| {
                        let k = c.map(|x| (x*1e4).floor() as i64);
                        !(-1..=1).any(|dx| (-1..=1).any(|dy| (-1..=1).any(|dz| grid.get(&[k[0]+dx,k[1]+dy,k[2]+dz]).is_some_and(|l| l.iter().any(|&j| q[j].0 == *s && (0..3).all(|d| (q[j].1[d]-c[d]).abs() <= 1e-7))))))
                    }).copied().collect()
                };
                let (oa,ob) = (unmatched(&ca,&cb),unmatched(&cb,&ca));
                Some(if oa.is_empty() && ob.is_empty() {
                    let at = (0..ta.len().min(tb.len())).find(|&i| ta[i] != tb[i] || sa.get(i) != sb.get(i));
                    let show = |t: [u32;3],v: &Vec<V3>| t.map(|x| (x,v[x as usize].map(|y| (y*1e4).round()/1e4)));
                    match at {
                        Some(i) => {
                            let kind = { let (mut x,mut y) = (ta[i],tb[i]); x.sort(); y.sort(); if x != y { "other corners" } else if (0..3).any(|r| (0..3).all(|k| ta[i][k] == tb[i][(k+r)%3])) { "rotated" } else { "reversed" } };
                            format!("the same {} triangles, ordered otherwise: first at {i} ({kind}) {:?} against {:?}",ta.len(),show(ta[i],va),show(tb[i],vb))
                        }
                        None => format!("the same {} triangles and indices, vertices moved",ta.len()),
                    }
                }
                    else { format!("{} against {} triangles; traced only {} e.g. {:?}; moved only {} e.g. {:?}",ta.len(),tb.len(),oa.len(),&oa[..oa.len().min(3)],ob.len(),&ob[..ob.len().min(3)]) })
            }
            _ => Some("different kinds".into()),
        }
    };
    let mut failed = Vec::new();
    for (name,source) in [("turning_prism",turning_prism()),("turned_lens",turned_lens()),("turned_box",turned_box()),("tumbling_cylinder",tumbling_cylinder()),("sliding_dumbbell",sliding_dumbbell())] {
        let e = harness::read(&source);
        let swept = harness::solid(&e,"swept");
        let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};
        let (_,sheets,_) = seeds(&e.sketch,swept,options.spacing,options.sagitta,&|_| {}).unwrap();
        // every stage's print up to the end or the refusal, and the refusal
        let run = |sheets| -> Result<(Vec<(&'static str,Print)>,Option<String>),String> {
            let mut prints = Vec::new();
            let refused = construct_from(&e.sketch,swept,&options,sheets,&mut |s,_| prints.push(print(&s))).err().map(|e| format!("{e:?}"));
            Ok((prints,refused))
        };
        let traced = run(sheets.clone());
        for phase in [0.,1.] {
            let mut moved = sheets.clone();
            let mut k = 0_f64;
            for s in &mut moved { for p in &mut s.points { k += 1.; *p = [p[0]+1e-12*(1.7*k+phase).sin(),p[1]+1e-12*(2.3*k+phase).cos(),p[2]+1e-12*(3.1*k+1.+phase).sin()]; } }
            let verdict = match (&traced,run(moved)) {
                (Ok((a,ra)),Ok((b,rb))) => a.iter().zip(&b).find_map(|((stage,x),(_,y))| differ(x,y).map(|d| format!("first differs at {stage}: {d}")))
                    .or_else(|| (a.len() != b.len() || *ra != rb).then(|| format!("refused otherwise: {ra:?} against {rb:?}"))),
                (a,b) => Some(format!("refused: traced {:?}, moved {:?}",a.as_ref().err(),b.err())),
            };
            eprintln!("{name} (phase {phase}): {}",verdict.as_deref().unwrap_or("as it was"));
            if let Some(v) = verdict { failed.push(format!("{name}: {v}")); }
        }
    }
    assert!(failed.is_empty(),"{failed:?}");
}
