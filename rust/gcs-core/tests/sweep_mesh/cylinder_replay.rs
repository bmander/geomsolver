//! Shared Phase 1/1a reporter for stage cuts, live zip proposals, oracle probes,
//! triangle distances and section overlays. This is diagnostic test support.
use super::{cylinder_oracle::{Cylinder,Side},creases,harness::{self,V3,distance}};
use gcs_core::solid::{MaterialField,swept_boundary::{self as sb,KeptMesh,Stage,SweptBoundaryOptions,FieldJudge}};
use sb::stitch::{ZipEvent,ZipPass,rim_zip_observed};
use std::{f64::consts::{PI,FRAC_PI_2},fmt::Write,path::Path};

fn sub(a: V3,b: V3) -> V3 { std::array::from_fn(|k| a[k]-b[k]) }
fn dot(a: V3,b: V3) -> f64 { (0..3).map(|k| a[k]*b[k]).sum() }
fn centre(p: [V3;3]) -> V3 { std::array::from_fn(|k| p.iter().map(|p| p[k]).sum::<f64>()/3.) }
fn area([a,b,c]: [V3;3]) -> f64 {
    let (u,v) = (sub(b,a),sub(c,a));
    let cr = [u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]];
    dot(cr,cr).sqrt()/2.
}
fn signed_volume(m: &KeptMesh) -> f64 {
    m.triangles.iter().map(|t| { let [a,b,c] = t.map(|v| m.vertices[v as usize]);
        (a[0]*(b[1]*c[2]-b[2]*c[1])+a[1]*(b[2]*c[0]-b[0]*c[2])+a[2]*(b[0]*c[1]-b[1]*c[0]))/6.
    }).sum()
}
fn sides(c: Cylinder,p: [V3;3],at: V3,h: f64) -> Option<[Side;2]> {
    let n = sb::triangle_normal(p[0],p[1],p[2])?;
    Some([-1.,1.].map(|s| c.side(std::array::from_fn(|k| at[k]+s*h*n[k]))))
}
fn reversed(c: Cylinder,p: [V3;3],h: f64) -> bool {
    sides(c,p,centre(p),h) == Some([Side::Exterior,Side::Material])
}

/// Piecewise analytic section rings split at the moving corner directions.
/// The two endpoint sector faces are essential; an x-uniform radial grid alone
/// misses their radial edges. Chords approximate each analytic patch; validation
/// below measures refinement, it does not claim a certified Hausdorff bound.
fn reference(c: Cylinder,n: usize) -> KeptMesh {
    assert!(c.half_roll > 0. && c.half_roll < PI/4.);
    let mut m = KeptMesh::default();
    let mut rings = Vec::new();
    for i in 0..=n {
        let x = 3.+(-FRAC_PI_2+PI*i as f64/n as f64).sin();
        let a = ((x-2.)*(4.-x)).max(0.).sqrt(); let beta = 1_f64.atan2(a);
        let (lo,hi) = ((beta-c.half_roll).max(0.),(beta+c.half_roll).min(FRAC_PI_2));
        let mut quarter = Vec::new();
        for branch in 0..3 { for j in 0..=n/4 {
            if branch > 0 && j == 0 { continue; }
            let t = j as f64/(n/4) as f64;
            let (phi,r) = match branch {
                0 => { let phi = lo*t; (phi,if j == n/4 { a.hypot(1.) } else { a/(phi+c.half_roll).cos() }) },
                1 => (lo+(hi-lo)*t,a.hypot(1.)),
                _ => { let phi = hi+(FRAC_PI_2-hi)*t; (phi,1./(phi-c.half_roll).sin()) },
            };
            // A collapsed upper branch still lies on the circumcircle, not the
            // continuation of an endpoint face hidden below it.
            let r = if branch == 2 && hi == FRAC_PI_2 { a.hypot(1.) } else { r };
            quarter.push((phi,r));
        } }
        let mut ring = Vec::new();
        for q in 0..4 { for k in 0..quarter.len()-1 {
            let (phi,r) = if q%2 == 0 { quarter[k] } else { quarter[quarter.len()-1-k] };
            let phi = if q%2 == 0 { q as f64*FRAC_PI_2+phi } else { (q+1) as f64*FRAC_PI_2-phi };
            ring.push(m.vertices.len() as u32); m.vertices.push([x,r*phi.cos(),r*phi.sin()]);
        } }
        rings.push(ring);
    }
    let mut add = |t: [u32;3],sheet| {
        if area(t.map(|v| m.vertices[v as usize])) > 1e-15 { m.triangles.push(t); m.sheet.push(sheet); }
    };
    for i in 0..n { for j in 0..rings[i].len() {
        let k = (j+1)%rings[i].len();
        let (a,b,d,e) = (rings[i][j],rings[i][k],rings[i+1][j],rings[i+1][k]);
        add([a,b,e],0); add([a,e,d],0);
    } }
    for i in [0,n] {
        // The axis is already a ring vertex at both end sections.
        let a = rings[i][0];
        for j in 1..rings[i].len()-1 {
            let t = [a,rings[i][j],rings[i][j+1]];
            add(if i == 0 { [t[0],t[2],t[1]] } else { t },1);
        }
    }
    m
}

// Test-only triangle BVH. Distances are to facets, never to mesh vertices.
struct Node { bounds: [V3;2], end: usize, first: usize, count: usize }
struct Index<'a> { mesh: &'a KeptMesh, nodes: Vec<Node>, order: Vec<usize> }
impl<'a> Index<'a> {
    fn new(mesh: &'a KeptMesh) -> Self {
        let mut s = Self {mesh,nodes:Vec::new(),order:(0..mesh.triangles.len()).collect()};
        if !s.order.is_empty() { s.build(0,s.order.len()); } s
    }
    fn bounds(&self,t: usize) -> [V3;2] {
        let p = self.mesh.triangles[t].map(|v| self.mesh.vertices[v as usize]);
        [std::array::from_fn(|k| p.iter().map(|p| p[k]).fold(f64::INFINITY,f64::min)),
         std::array::from_fn(|k| p.iter().map(|p| p[k]).fold(f64::NEG_INFINITY,f64::max))]
    }
    fn build(&mut self,first: usize,count: usize) {
        let mut bounds = [[f64::INFINITY;3],[f64::NEG_INFINITY;3]];
        for &t in &self.order[first..first+count] { let b = self.bounds(t);
            for k in 0..3 { bounds[0][k] = bounds[0][k].min(b[0][k]); bounds[1][k] = bounds[1][k].max(b[1][k]); }
        }
        let slot = self.nodes.len(); self.nodes.push(Node {bounds,end:slot+1,first,count});
        if count > 8 {
            let axis = (0..3).max_by(|&a,&b| (bounds[1][a]-bounds[0][a]).total_cmp(&(bounds[1][b]-bounds[0][b]))).unwrap();
            let mesh = self.mesh;
            self.order[first..first+count].select_nth_unstable_by(count/2,|&a,&b| {
                let coord = |t: usize| mesh.triangles[t].iter().map(|&v| mesh.vertices[v as usize][axis]).sum::<f64>();
                coord(a).total_cmp(&coord(b)).then(a.cmp(&b))
            });
            self.nodes[slot].count = 0; self.build(first,count/2); self.build(first+count/2,count-count/2);
            self.nodes[slot].end = self.nodes.len();
        }
    }
    fn nearest(&self,p: V3) -> (f64,usize) {
        let mut best = (f64::INFINITY,usize::MAX); let mut i = 0;
        while i < self.nodes.len() {
            let n = &self.nodes[i];
            let d2: f64 = (0..3).map(|k| (n.bounds[0][k]-p[k]).max(p[k]-n.bounds[1][k]).max(0.).powi(2)).sum();
            if d2 > best.0*best.0 { i = n.end; continue; }
            for &t in &self.order[n.first..n.first+n.count] {
                let [a,b,c] = self.mesh.triangles[t].map(|v| self.mesh.vertices[v as usize]);
                let d = distance(p,sb::closest_on_triangle(p,a,b,c).0);
                if d < best.0 { best = (d,t); }
            }
            i += 1;
        } best
    }
}

fn write_mesh(path: &Path,m: &KeptMesh) {
    let mut s = String::from("# v x y z; t a b c source_sheet; indices local to this snapshot\n");
    for p in &m.vertices { writeln!(s,"v {:.17e} {:.17e} {:.17e}",p[0],p[1],p[2]).unwrap(); }
    for (i,t) in m.triangles.iter().enumerate() { writeln!(s,"t {} {} {} {}",t[0],t[1],t[2],m.sheet[i]).unwrap(); }
    std::fs::write(path,s).unwrap();
}
fn read_mesh(s: &str) -> KeptMesh {
    let mut m = KeptMesh::default();
    for line in s.lines().filter(|l| !l.starts_with('#')) {
        let v: Vec<_> = line.split_whitespace().collect();
        match v.first().copied() {
            Some("v") => m.vertices.push(std::array::from_fn(|k| v[k+1].parse().unwrap())),
            Some("t") => { m.triangles.push(std::array::from_fn(|k| v[k+1].parse().unwrap())); m.sheet.push(v[4].parse().unwrap()); }
            _ => panic!("bad replay line: {line}"),
        }
    } m
}
fn rim_distance(edges: &[(V3,V3)],p: V3) -> f64 {
    edges.iter().map(|&(a,b)| {
        let ab = sub(b,a);
        let t = if dot(ab,ab) > 0. { (dot(sub(p,a),ab)/dot(ab,ab)).clamp(0.,1.) } else {0.};
        distance(p,std::array::from_fn(|k| a[k]+t*ab[k]))
    }).fold(f64::INFINITY,f64::min)
}

#[derive(Default,Debug)]
struct Measure { area: f64, correct_area: f64, reversed_area: f64, other_area: f64, away_reversed_area: f64 }
fn measure(c: Cylinder,m: &KeptMesh,h: f64,subdivide: usize) -> Measure {
    let mut out = Measure::default();
    let edges: Vec<_> = if subdivide == 1 {
        sb::boundary_loops(&m.triangles).iter().flat_map(|l| (0..l.len()).map(move |i| (m.vertices[l[i] as usize],m.vertices[l[(i+1)%l.len()] as usize]))).collect()
    } else { Vec::new() };
    for t in &m.triangles {
        let p = t.map(|v| m.vertices[v as usize]); let weight = area(p)/(subdivide*subdivide) as f64;
        let at = |i: f64,j: f64| std::array::from_fn(|k| p[0][k]+(p[1][k]-p[0][k])*i/subdivide as f64+(p[2][k]-p[0][k])*j/subdivide as f64);
        for i in 0..subdivide { for j in 0..subdivide-i { for upper in [false,true] {
            if upper && i+j+1 >= subdivide { continue; }
            let shift = if upper {2./3.} else {1./3.}; let q = at(i as f64+shift,j as f64+shift);
            out.area += weight;
            match sides(c,p,q,h) {
                Some([Side::Material,Side::Exterior]) => out.correct_area += weight,
                Some([Side::Exterior,Side::Material]) => { out.reversed_area += weight;
                    // Keep this expensive rim query out of the quadrature's inner refinements.
                    if subdivide == 1 && rim_distance(&edges,q) > 2.*h { out.away_reversed_area += weight; }
                }
                _ => out.other_area += weight,
            }
        } } }
    } out
}

/// Sample analytic boundary points, weighted by the reference facet area.
/// This reports estimated missing area; it is not an acceptance certificate.
fn coverage(c: Cylinder,reference: &KeptMesh,m: &KeptMesh,tol: f64) -> (f64,f64,f64) {
    let index = Index::new(m); let mut total = 0.; let mut missed = 0.; let mut max = 0_f64;
    for (i,t) in reference.triangles.iter().enumerate() {
        let p = t.map(|v| reference.vertices[v as usize]); let a = area(p); let mid = centre(p);
        let q = if reference.sheet[i] == 0 { c.point(mid[0],mid[2].atan2(mid[1])) } else { mid };
        let d = index.nearest(q).0; total += a; if d > tol { missed += a; } max = max.max(d);
    } (total,missed,max)
}

fn sections(c: Cylinder,m: &KeptMesh,xs: &[f64]) -> String {
    let mut s = format!("<svg xmlns='http://www.w3.org/2000/svg' width='{}' height='440' viewBox='0 0 {} 440'><rect width='100%' height='100%' fill='white'/><style>text{{font:14px sans-serif}} .mesh{{stroke:#b54e34;stroke-width:1;fill:none}} .oracle{{stroke:#176c9e;stroke-width:2;fill:none}} .bad{{stroke:#ce2645;stroke-width:4;fill:none}}</style><text x='15' y='22'>Tumbling cylinder sections: blue analytic boundary, rust candidate facets, red reversed facets</text>",xs.len()*370,xs.len()*370);
    for (panel,&x) in xs.iter().enumerate() {
        let xy = |p: V3| [185.+panel as f64*370.+p[1]*110.,230.-p[2]*110.];
        writeln!(s,"<text x='{}' y='50'>x={x:.6}</text>",panel*370+20).unwrap();
        // Plane/facet intersection, not a projected point cloud.
        for t in &m.triangles {
            let p = t.map(|v| m.vertices[v as usize]); let mut hits = Vec::new();
            for k in 0..3 { let (a,b) = (p[k],p[(k+1)%3]);
                if (a[0] <= x && x < b[0]) || (b[0] <= x && x < a[0]) {
                    let t = (x-a[0])/(b[0]-a[0]); hits.push(xy(std::array::from_fn(|k| a[k]+t*(b[k]-a[k]))));
                }
            }
            if hits.len() == 2 { writeln!(s,"<path class='{}' d='M {},{} L {},{}'/>",if reversed(c,p,0.02) {"bad"} else {"mesh"},hits[0][0],hits[0][1],hits[1][0],hits[1][1]).unwrap(); }
        }
        s.push_str("<path class='oracle' d='");
        for j in 0..=1440 { let p = xy(c.point(x,2.*PI*j as f64/1440.)); write!(s,"{} {},{} ",if j == 0 {"M"} else {"L"},p[0],p[1]).unwrap(); }
        s.push_str("Z'/>");
    }
    s.push_str("</svg>"); s
}

#[test]
fn analytic_reference_and_triangle_distance_controls() {
    let c = Cylinder::fixture();
    let coarse = reference(c,32); let fine = reference(c,64);
    let v = c.volume(256);
    assert!((signed_volume(&fine)-v).abs() < (signed_volume(&coarse)-v).abs());
    assert!((signed_volume(&fine)-v).abs() < 0.02);
    let good = measure(c,&fine,0.02,1);
    eprintln!("reference volume {} vs oracle {v}; sides {good:?}",signed_volume(&fine));
    assert_eq!(good.reversed_area,0.);
    assert!(good.correct_area/good.area > 0.99);
    let mut bad = fine.clone(); for t in &mut bad.triangles { t.swap(1,2); }
    assert!(measure(c,&bad,0.02,1).reversed_area/good.area > 0.99);
    // A point on a large facet may be far from every vertex.
    let facet = KeptMesh {vertices:vec![[0.,0.,0.],[10.,0.,0.],[0.,10.,0.]],triangles:vec![[0,1,2]],sheet:vec![0]};
    assert_eq!(Index::new(&facet).nearest([3.,3.,0.]),(0.,0));
    assert!((Index::new(&facet).nearest([3.,3.,2.]).0-2.).abs() < 1e-14);
    let index = Index::new(&coarse);
    for p in [[3.,0.,0.],[2.5,1.6,0.7],[4.2,0.,0.],[3.,0.,1.4]] {
        let brute = coarse.triangles.iter().map(|t| { let [a,b,c] = t.map(|v| coarse.vertices[v as usize]); distance(p,sb::closest_on_triangle(p,a,b,c).0) }).fold(f64::INFINITY,f64::min);
        assert!((index.nearest(p).0-brute).abs() < 1e-13);
    }
    let (total,missed,max) = coverage(c,&fine,&coarse,0.04);
    assert!(total > 20. && missed/total < 0.001 && max < 0.04);
    // A deliberately withheld analytic patch is a permanent negative control.
    let mut hole = fine.clone(); let keep: Vec<_> = hole.triangles.iter().map(|t| { let p = centre(t.map(|v| hole.vertices[v as usize])); !(p[0] > 2.8 && p[0] < 3.2 && p[1] > 1.) }).collect();
    hole = sb::retained(&hole,&keep);
    assert!(coverage(c,&fine,&hole,0.04).1 > 0.1);
}

#[test]
#[ignore = "exports continuous cylinder oracle and actual construction replay; set SOLVENT_EXPORT"]
fn phase_one_cylinder_replay() {
    let dir = std::path::PathBuf::from(std::env::var("SOLVENT_EXPORT").expect("set a separate artifact directory"));
    std::fs::create_dir_all(&dir).unwrap();
    let c = Cylinder::fixture(); let options = SweptBoundaryOptions::default();
    let source = creases::tumbling_cylinder();
    std::fs::write(dir.join("source.sv"),&source).unwrap();
    std::fs::write(dir.join("options.txt"),format!("{options:#?}\nDerived: vertex={} reach={} merge={:?} probe={} least={} coverage={} junction={}\n",options.vertex_tolerance(),options.reach(),options.crease_merge(),options.probe_distance(),options.least_probe(),options.coverage(),options.junction())).unwrap();
    let e = harness::read(&source); let swept = harness::solid(&e,"swept");
    let mut stages = Vec::<(String,KeptMesh)>::new();
    let mut source_seeds = Vec::new(); let mut source_labels = Vec::new(); let mut clipped_rims = Vec::new();
    let candidate = sb::candidate(&e.sketch,swept,&options,&|s| eprintln!("{s}"),&mut |stage,_| {
        let (name,m) = match stage {
            Stage::Labelled {seeds,labelled} => {
                source_seeds = seeds.to_vec(); source_labels = labelled.to_vec();
                std::fs::write(dir.join("seeds-and-label-witnesses.txt"),format!("{seeds:#?}\n{labelled:#?}")).unwrap();
                let mut raw = KeptMesh::default();
                for (i,s) in seeds.iter().enumerate() { let p = s.patch(); let base = raw.vertices.len() as u32;
                    raw.vertices.extend_from_slice(&p.points); raw.triangles.extend(p.triangles.iter().map(|t| t.map(|v| v+base))); raw.sheet.extend(vec![i as u32;p.triangles.len()]);
                }
                write_mesh(&dir.join("00-source.mesh"),&raw); stages.push(("source".into(),raw)); return;
            }
            Stage::Clipped {mesh,rims} => { clipped_rims = rims.to_vec(); std::fs::write(dir.join("clipped-rims.txt"),format!("{rims:#?}")).unwrap(); ("clipped",mesh) }
            Stage::Merged {mesh,..} => ("merged",mesh),
            Stage::Kept {keep,kept,..} => { std::fs::write(dir.join("centroid-keep.txt"),format!("{keep:?}")).unwrap(); ("kept",kept) }
            Stage::Unioned {mesh,..} => ("unioned",mesh), Stage::Uncovered {mesh,..} => ("uncovered",mesh),
            Stage::Welded {mesh,..} => ("welded",mesh), Stage::Split {mesh} => ("split",mesh),
            Stage::Zipped {mesh,..} => ("zipped",mesh), _ => return,
        };
        assert_eq!(m.sheet.len(),m.triangles.len(),"source alignment at {name}");
        assert!(m.sheet.iter().all(|&s| s == u32::MAX || (s as usize) < source_seeds.len()),"source range at {name}");
        write_mesh(&dir.join(format!("{:02}-{name}.mesh",stages.len())),m); stages.push((name.into(),m.clone()));
    }).unwrap();
    let mut report = String::from("# Binary64 diagnostic estimates, not whole-surface certificates\n# away_reversed_area is measured only when subdivision=1; finer rows omit that measurement.\n");
    writeln!(report,"candidate topology={:?} loops={:?} centroid certified={} failed={} unresolved={}",candidate.topology,candidate.unpaired.iter().map(Vec::len).collect::<Vec<_>>(),candidate.certificate.certified,candidate.certificate.failures.len(),candidate.certificate.unresolved.len()).unwrap();
    std::fs::write(dir.join("certificate-and-stats.txt"),format!("{:#?}\n{:#?}",candidate.certificate,candidate.stats)).unwrap();
    for (name,m) in &stages {
        let stats = measure(c,m,0.02,1);
        writeln!(report,"{name}: triangles={} signed_volume={} {stats:?}",m.triangles.len(),signed_volume(m)).unwrap();
    }
    let clipped = &stages.iter().find(|(name,_)| name == "clipped").unwrap().1;
    let mut merged = clipped.clone(); let mut merges = String::new();
    let mut first_merge_failure = None; let mut corrected = None; let mut actual_aliases = std::collections::BTreeMap::new();
    let (distance,snap) = options.crease_merge();
    sb::crease::merge_creases_observed(&mut merged,&clipped_rims,distance,snap,&mut |event| match event {
        sb::crease::MergeEvent::Aliases {mesh,aliases} => {
            actual_aliases = aliases.clone();
            writeln!(merges,"aliases {aliases:?}").unwrap();
            for (i,t) in mesh.triangles.iter().enumerate() {
                let after = t.map(|v| *aliases.get(&v).unwrap_or(&v));
                if after == *t { continue; }
                let p = after.map(|v| mesh.vertices[v as usize]);
                let before = t.map(|v| mesh.vertices[v as usize]);
                let before_sides = sides(c,before,centre(before),0.02);
                writeln!(merges,"source_triangle={i} true_source_sheet={} before={t:?} after={after:?} before_sides={before_sides:?} after_sides={:?}",mesh.sheet[i],sides(c,p,centre(p),0.02)).unwrap();
                if first_merge_failure.is_none() && area(p) > 1e-6 && before_sides == Some([Side::Material,Side::Exterior]) && reversed(c,p,0.01) && reversed(c,p,0.02) {
                    first_merge_failure = Some((i,before,p));
                    writeln!(report,"first reversed alias: clipped triangle={i}, true_source_sheet={}, before={before:?}, after={p:?}, aliases={:?}",mesh.sheet[i],t.map(|v| (v,*aliases.get(&v).unwrap_or(&v)))).unwrap();
                    for (name,vertices) in [("merge-before-proposal",before),("merge-after-proposal",p)] {
                        write_mesh(&dir.join(format!("{name}.mesh")),&KeptMesh {vertices:vertices.into(),triangles:vec![[0,1,2]],sheet:vec![mesh.sheet[i]]});
                    }
                }
            }
        }
        sb::crease::MergeEvent::Retained {mesh,source_triangles} => {
            let mut fixed = mesh.clone(); fixed.sheet = source_triangles.iter().map(|&i| clipped.sheet[i]).collect(); corrected = Some(fixed);
            let mismatches: Vec<_> = source_triangles.iter().enumerate().filter(|&(i,&source)| mesh.sheet[i] != clipped.sheet[source]).map(|(i,&source)| (i,source,mesh.sheet[i],clipped.sheet[source])).collect();
            assert_eq!(mesh.sheet.len(),mesh.triangles.len()); assert!(mismatches.is_empty());
            writeln!(report,"crease retain: triangles={}, sheet_labels={}, wrong_source_labels={}",mesh.triangles.len(),mesh.sheet.len(),mismatches.len()).unwrap();
            writeln!(merges,"retained origins={source_triangles:?}\nwrong_labels (output,source_triangle,actual,expected)={mismatches:?}").unwrap();
        }
    });
    let actual = &stages.iter().find(|(name,_)| name == "merged").unwrap().1;
    assert_eq!(merged.vertices,actual.vertices); assert_eq!(merged.triangles,actual.triangles); assert_eq!(merged.sheet,actual.sheet);
    // Independent parent-index reconstruction of ownership at the retention
    // boundary, followed by the same geometric splitter on the same admitted rims.
    // Compare both geometry and labels; equal array lengths alone are insufficient.
    let mut corrected = corrected.unwrap();
    let resolve = |v: u32| *actual_aliases.get(&v).unwrap_or(&v);
    let vertices: std::collections::BTreeSet<_> = clipped_rims.iter().flat_map(|r| r.vertices.iter().map(|&v| resolve(v))).collect();
    let mut edges = std::collections::BTreeSet::new();
    for r in &clipped_rims { for k in 0..if r.closed {r.vertices.len()} else {r.vertices.len().saturating_sub(1)} {
        let (a,b) = (resolve(r.vertices[k]),resolve(r.vertices[(k+1)%r.vertices.len()]));
        if a != b { edges.insert((a.min(b),a.max(b))); }
    } }
    sb::split_where(&mut corrected,distance,&|a,b| edges.contains(&(a.min(b),a.max(b))),&|v| vertices.contains(&v));
    assert_eq!(corrected.vertices,actual.vertices); assert_eq!(corrected.triangles,actual.triangles);
    let wrong: Vec<_> = actual.sheet.iter().zip(&corrected.sheet).enumerate().filter(|(_,(&a,&b))| a != b).map(|(i,(&a,&b))| (i,a,b)).collect();
    assert_eq!(corrected.sheet,actual.sheet);
    writeln!(report,"after crease splitting: geometry and ownership agree with independent parent mapping, incorrect live labels={}",wrong.len()).unwrap();
    writeln!(merges,"after split wrong_labels (triangle,actual,expected)={wrong:?}").unwrap();
    write_mesh(&dir.join("merged-lineage-check.mesh"),&corrected);
    std::fs::write(dir.join("crease-merge-decisions.txt"),merges).unwrap();
    // Replay the earlier overlap cuts with source identities and exact inputs.
    let unioned = &stages.iter().find(|(name,_)| name == "unioned").unwrap().1;
    let mut cuts = String::new(); let mut cut_id = 0;
    let (uncovered,_) = sb::trim::clip_overlaps_observed(unioned,options.coverage(),&mut |d| {
        if d.left.is_some() {
            writeln!(cuts,"cut {cut_id} source_triangle={} source_sheet={} covering_sheet={} piece={:?} covering_triangles={:?} outline={:?} left={:?}",
                d.source_triangle,unioned.sheet[d.source_triangle],d.covering_sheet,d.piece,d.covering_triangles,d.outline,d.left).unwrap();
        }
        cut_id += 1;
    });
    let actual = &stages.iter().find(|(name,_)| name == "uncovered").unwrap().1;
    assert_eq!(uncovered.vertices,actual.vertices); assert_eq!(uncovered.triangles,actual.triangles); assert_eq!(uncovered.sheet,actual.sheet);
    std::fs::write(dir.join("overlap-decisions.txt"),cuts).unwrap();
    let initial = stages.iter().find(|(name,_)| name == "split").unwrap().1.clone();
    // This is an execution of the production zip implementation with the exact
    // pre-zip mesh. Equality with the observed production output is required.
    let field = MaterialField::read(&e.sketch,swept,options.axis_tolerance()).unwrap();
    if let Some((i,_,after)) = first_merge_failure {
        let mut diagnostic = FieldJudge::new(field.clone(),options.judge_tolerance(),options.near_budget,options.far_budget,options.cached_poses);
        let at = centre(after); let n = sb::triangle_normal(after[0],after[1],after[2]).unwrap();
        for h in [0.01,0.02,0.04] {
            let probes = [-1.,1.].map(|s| std::array::from_fn(|k| at[k]+s*h*n[k]));
            writeln!(report,"alias triangle={i} h={h} points={probes:?} oracle={:?} field={:?}",probes.map(|p| c.side(p)),probes.map(|p| diagnostic.sign(p))).unwrap();
        }
    }
    let mut judge = FieldJudge::new(field.clone(),options.judge_tolerance(),options.near_budget,options.far_budget,options.cached_poses);
    let mut replay = initial.clone(); let mut trace = String::new(); let mut spans = String::new();
    let mut selected: Option<(usize,ZipPass,KeptMesh,[u32;3])> = None;
    let mut sequence = 0; let mut expected = None;
    let (pairs,unpaired) = rim_zip_observed(&mut replay,options.spacing,options.junction(),options.least_probe(),
        &mut |points,normals| {
            let id = spans.lines().filter(|l| l.starts_with("span ")).count();
            writeln!(spans,"span {id} points={points:?} owner_normals={normals:?}").unwrap();
            let result = sb::trim::loop_span_observed(&mut judge,points,normals,options.vertex_tolerance(),options.reach(),&mut |p,n,result| {
                writeln!(spans,"probe point={p:?} direction={n:?} projection={result:?}").unwrap();
            });
            writeln!(spans,"result {result:?}").unwrap(); matches!(result,Ok(sb::Span::Spanned))
        },&mut |event| match event {
            ZipEvent::Round {round,mesh} => { write_mesh(&dir.join(format!("zip-round-{round:02}.mesh")),mesh); writeln!(trace,"round {round}").unwrap(); }
            ZipEvent::Preflight {round,mesh,fan,accepted} => { writeln!(trace,"preflight round={round} accepted={accepted} fan={fan:?} vertices={}",mesh.vertices.len()).unwrap(); }
            ZipEvent::Triangle {round,pass,mesh,triangle,rejection} => {
                let p = triangle.map(|v| mesh.vertices[v as usize]);
                let incident: Vec<_> = mesh.triangles.iter().enumerate().filter(|(_,t)| t.iter().any(|v| triangle.contains(v))).map(|(i,_)| (i,mesh.sheet[i])).collect();
                writeln!(trace,"proposal {sequence} round={round} pass={pass:?} triangle={triangle:?} points={p:?} rejection={rejection:?} incident={incident:?} oracle_h002={:?}",sides(c,p,centre(p),0.02)).unwrap();
                if selected.is_none() && rejection.is_none() && area(p) > 1e-5 && reversed(c,p,0.01) && reversed(c,p,0.02) {
                    selected = Some((sequence,pass,mesh.clone(),triangle));
                }
                expected = rejection.is_none().then_some((sequence,triangle)); sequence += 1;
            }
            ZipEvent::Applied {round,pass,mesh,triangle} => {
                let (seq,t) = expected.take().expect("application without proposal"); assert_eq!(t,mesh.triangles[triangle]);
                writeln!(trace,"applied {seq} round={round} pass={pass:?} triangle_index={triangle}").unwrap();
            }
            ZipEvent::Finished {mesh} => { assert!(expected.is_none()); write_mesh(&dir.join("zip-finished.mesh"),mesh); }
        });
    assert_eq!(replay.vertices,candidate.mesh.vertices,"instrumented replay changed vertex geometry");
    assert_eq!(replay.triangles,candidate.mesh.triangles,"instrumented replay changed triangles");
    assert_eq!(replay.sheet,candidate.mesh.sheet);
    assert_eq!(unpaired,candidate.unpaired);
    writeln!(report,"zip replay identical: pairs={pairs}, proposals={sequence}").unwrap();
    std::fs::write(dir.join("zip-decisions.txt"),trace).unwrap();
    std::fs::write(dir.join("zip-field-witnesses.txt"),spans).unwrap();
    for n in [64,128,256,512,1024] { writeln!(report,"volume quadrature n={n}: {:.12}",c.volume(n)).unwrap(); }
    for n in [32,64,128] {
        let r = reference(c,n);
        writeln!(report,"reference n={n} volume={:.12} coverage_of_candidate(area,estimated_area_beyond_004,max_sample_distance)={:?} reference_self={:?}",signed_volume(&r),coverage(c,&r,&candidate.mesh,0.04),coverage(c,&r,&r,0.04)).unwrap();
        for tol in [0.01,0.02] { writeln!(report,"reference n={n} candidate coverage tol={tol}: {:?}",coverage(c,&r,&candidate.mesh,tol)).unwrap(); }
        if n == 128 { write_mesh(&dir.join("analytic-reference.mesh"),&r); }
    }
    let section_reference = reference(c,64);
    for (name,m) in &stages {
        writeln!(report,"stage coverage n=64 {name}: {:?}",coverage(c,&section_reference,m,0.04)).unwrap();
    }
    for n in [1,2,4,8] { writeln!(report,"candidate sides probe=.02 subdivision={n}: {:?}",measure(c,&candidate.mesh,0.02,n)).unwrap(); }
    std::fs::write(dir.join("sections.svg"),sections(c,&candidate.mesh,&[2.05,2.3,3.,3.7])).unwrap();
    if let Some((seq,pass,before,t)) = selected {
        write_mesh(&dir.join("selected-before.mesh"),&before);
        let p = t.map(|v| before.vertices[v as usize]); let at = centre(p);
        let n = sb::triangle_normal(p[0],p[1],p[2]).unwrap();
        let mut witness_judge = FieldJudge::new(field,options.judge_tolerance(),options.near_budget,options.far_budget,options.cached_poses);
        writeln!(report,"selected proposal={seq} pass={pass:?} triangle={t:?} points={p:?} centre={at:?} normal={n:?} area={}",area(p)).unwrap();
        for h in [0.01,0.02,0.04] {
            let probes = [-1.,1.].map(|s| std::array::from_fn(|k| at[k]+s*h*n[k]));
            let field = probes.map(|p| witness_judge.sign(p));
            writeln!(report,"selected h={h} probes={probes:?} oracle={:?} independent_field_diagnostic={field:?}",probes.map(|p| c.side(p))).unwrap();
        }
        let mut selected_triangle = KeptMesh::default(); selected_triangle.vertices = p.into(); selected_triangle.triangles.push([0,1,2]); selected_triangle.sheet.push(u32::MAX);
        write_mesh(&dir.join("selected-proposal.mesh"),&selected_triangle);
        let mut region = before.clone();
        let keep: Vec<_> = before.triangles.iter().map(|t| {
            let p = t.map(|v| before.vertices[v as usize]);
            (0..3).all(|k| p.iter().any(|p| p[k] >= at[k]-0.25) && p.iter().any(|p| p[k] <= at[k]+0.25))
        }).collect();
        region = sb::retained(&region,&keep).compact();
        write_mesh(&dir.join("selected-region.mesh"),&region);
        let sheets: std::collections::BTreeSet<_> = region.sheet.iter().copied().filter(|&s| s != u32::MAX).collect();
        writeln!(report,"selected region centre +/- .25: triangles={}, source sheet labels={sheets:?}; sides={:?}",region.triangles.len(),measure(c,&region,0.02,4)).unwrap();
        writeln!(report,"No validated outer interface has been established. Conservative reconstruction group: ALL {} source patches (IDs 0..{}), including both roll caps and all interacting bands; the local crop is diagnostic only.",source_seeds.len(),source_seeds.len()-1).unwrap();
        for (name,m) in &stages {
            let (d,i) = Index::new(m).nearest(at); let p = m.triangles[i].map(|v| m.vertices[v as usize]);
            writeln!(report,"selected region stage={name} nearest_triangle={i} sheet={} distance={d} nearest_centroid_sides={:?} (proximity, not ancestry)",m.sheet[i],sides(c,p,centre(p),0.02)).unwrap();
        }
        for s in sheets { std::fs::write(dir.join(format!("selected-source-{s}.txt")),format!("{:#?}\n{:#?}",source_seeds[s as usize],source_labels[s as usize])).unwrap(); }
        std::fs::write(dir.join("selected-section.svg"),sections(c,&candidate.mesh,&[at[0]-0.025,at[0],at[0]+0.025])).unwrap();
    } else { writeln!(report,"No robust reversed zip proposal found; inspect stage evidence before selecting a region.").unwrap(); }
    eprintln!("{report}"); std::fs::write(dir.join("report.txt"),report).unwrap();
}

#[test]
fn frozen_alias_and_zip_emissions_are_rejected_independently() {
    // Historical geometry is deliberately immutable. A future constructor may
    // succeed; these counterexamples must remain detectable by the oracle.
    let c = Cylinder::fixture();
    let before = read_mesh(include_str!("fixtures/cylinder-phase-one/merge-before.mesh"));
    let after = read_mesh(include_str!("fixtures/cylinder-phase-one/merge-after.mesh"));
    let zip = read_mesh(include_str!("fixtures/cylinder-phase-one/zip-proposal.mesh"));
    let p: [V3;3] = before.vertices.clone().try_into().unwrap();
    assert_eq!(sides(c,p,centre(p),0.02),Some([Side::Material,Side::Exterior]));
    for m in [&after,&zip] {
        let p = m.triangles[0].map(|v| m.vertices[v as usize]);
        assert!(area(p) > 1e-6);
        for h in [0.01,0.02] { assert!(reversed(c,p,h)); }
    }
    // The merge moves one vertex within its .02 snap tolerance yet flips the
    // material side. Nearness does not establish an admissible shared boundary.
    assert_eq!(before.vertices[0],after.vertices[0]); assert_eq!(before.vertices[2],after.vertices[2]);
    assert!(distance(before.vertices[1],after.vertices[1]) < 0.02);
}
