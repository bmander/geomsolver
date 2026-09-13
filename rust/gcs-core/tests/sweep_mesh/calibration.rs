//! Independent convex reference meshes: no sweep seeds, trimming or seam repair.
use super::{harness,motions,tools};
use gcs_core::solid::{MaterialField,PlanarField,RevolvedField,ExtrudedField,SpatialField};
use gcs_core::solid::swept_boundary::{KeptMesh,SweptBoundaryOptions,Check,inspect};
use std::f64::consts::PI;
use gcs_core::interval::Interval as I;

fn sphere(z: f64) -> SpatialField {
    RevolvedField::new(PlanarField::disk([0.;2],1.).unwrap(),[3.,0.,z],[0.,0.,1.]).unwrap().into()
}
fn static_field(length: f64) -> MaterialField {
    if length == 0. { return sphere(0.).into(); }
    let barrel: SpatialField = ExtrudedField::new(PlanarField::disk([0.;2],1.).unwrap(),
        [3.,0.,0.],[1.,0.,0.],[0.,1.,0.],[0.,length]).unwrap().into();
    barrel.union(sphere(0.)).unwrap().union(sphere(length)).unwrap().into()
}
fn swept_field(length: f64) -> MaterialField {
    let motion = if length == 0. { motions::TURN_OWN_AXIS.into() } else { motions::slide_z(length) };
    let sweep = if length == 0. { motions::swept("turn",-60.,60.) } else { motions::swept("feed",0.,360.) };
    let e = harness::read(&format!("{}{motion}{sweep}",tools::SPHERE));
    MaterialField::read(&e.sketch,harness::solid(&e,"swept"),1e-10).unwrap()
}

/// Latitude/longitude hemispheres and a polygonal barrel, sharing exact indices.
/// Each spherical cell spans at most h in each parameter. The norm of every
/// second partial of the unit sphere is <=1, so Taylor's remainder bounds the
/// parameter-interpolated triangle error by (2h)^2/2. Convexity and the same
/// parameter cover give both distance directions. Barrel error <= h^2/2.
/// Interval trig encloses ideal vertices and transfers their rounding error.
/// Pole vertices can use any longitude in their cell because their image is fixed.
fn reference(length: f64,n: usize) -> (KeptMesh,f64) {
    let m = 2*n;
    let pi = I::new(PI.next_down(),PI.next_up()).unwrap();
    let mut rounding = 0_f64;
    let mut vertices = vec![[3.,0.,-1.]];
    let mut rings = Vec::new();
    for k in 1..n {
        let theta = pi.mul(I::point(k as f64).unwrap().div(I::point(n as f64).unwrap()).unwrap().sub(I::point(0.5).unwrap()).unwrap()).unwrap();
        let (sz,radial) = theta.sin_cos().unwrap();
        let z = sz.add(I::point(if k > n/2 {length} else {0.}).unwrap()).unwrap();
        let mut ring = |z| {
            let start = vertices.len() as u32;
            for j in 0..m {
                let phi = pi.mul(I::point(2.*j as f64).unwrap()).unwrap().div(I::point(m as f64).unwrap()).unwrap();
                let (sn,cs) = phi.sin_cos().unwrap();
                let ideal = [I::point(3.).unwrap().add(radial.mul(cs).unwrap()).unwrap(),radial.mul(sn).unwrap(),z];
                let p = ideal.map(|v| { let [a,b] = v.bounds(); a*0.5+b*0.5 });
                let mut square = I::ZERO;
                for axis in 0..3 { square = square.add(ideal[axis].sub(I::point(p[axis]).unwrap()).unwrap().square().unwrap()).unwrap(); }
                rounding = rounding.max(I::new(0.,square.bounds()[1]).unwrap().sqrt().unwrap().bounds()[1]);
                vertices.push(p);
            }
            rings.push(start);
        };
        ring(z);
        if k == n/2 && length > 0. { ring(I::point(length).unwrap()); }
    }
    let top = vertices.len() as u32;
    vertices.push([3.,0.,length+1.]);
    let mut triangles = Vec::new();
    for j in 0..m as u32 {
        let next = (j+1)%m as u32;
        triangles.push([0,rings[0]+next,rings[0]+j]);
        for r in rings.windows(2) {
            triangles.push([r[0]+j,r[0]+next,r[1]+next]);
            triangles.push([r[0]+j,r[1]+next,r[1]+j]);
        }
        let last = *rings.last().unwrap();
        triangles.push([last+j,last+next,top]);
    }
    let sheet = vec![0;triangles.len()];
    let error = pi.div(I::point(n as f64).unwrap()).unwrap().square().unwrap().mul(I::point(2.).unwrap()).unwrap().add(I::point(rounding).unwrap()).unwrap().bounds()[1];
    (KeptMesh {vertices,triangles,sheet},error)
}

fn calibrate(tol: f64,n: usize,length: f64,swept: bool) {
    let (mesh,error) = reference(length,n);
    let count = mesh.triangles.len();
    let field = if swept { swept_field(length) } else { static_field(length) };
    let options = SweptBoundaryOptions::default();
    let mut audit = options.audit(); audit.tolerance = tol; audit.max_cells = if tol == 0.04 {800000} else {3200000}; audit.max_depth = 36;
    let start = std::time::Instant::now();
    assert!(error < tol/2.);
    let report = inspect(field,mesh,&options,audit);
    if let Check::Attempted(a) = report.spatial() {
        eprintln!("length={length} swept={swept} tol={tol} triangles={count} analytic={error:.9} seconds={:.3} visits={:?} witnesses={:?} unfinished={:?} queries={} rolls={} query_seconds={:?} query_limits={:?}",
            start.elapsed().as_secs_f64(),a.visited(),[a.surface().len(),a.coverage().len()],
            [a.surface_unfinished().len(),a.coverage_unfinished().len()],report.stats().near+report.stats().far,report.stats().roll_evaluations,
            [report.stats().near_time.as_secs_f64(),report.stats().far_time.as_secs_f64()],
            [report.stats().exhausted,report.stats().resolution_limited]);
        if let Some(w) = a.surface_unfinished().first() { eprintln!("first surface obligation {w:?}"); }
        if let Some(w) = a.coverage_unfinished().first() { eprintln!("first coverage obligation {w:?}"); }
    }
    let result = report.into_accepted();
    eprintln!("acceptance={:?}",result.as_ref().map(|s| s.spatial().distance_bound()));
    assert!(result.is_ok(),"length={length} swept={swept} tol={tol}: {result:?}");
}

#[test]
fn curved_reference_meshes_have_measured_acceptance() {
    for (tol,n) in [(0.04,32),(0.02,48)] { for length in [0.,2.] {
        calibrate(tol,n,length,false);
        if tol == 0.04 { calibrate(tol,n,length,true); }
    } }
}

#[test]
#[ignore = "fine continuous-field calibration takes minutes; run explicitly after audit or field changes"]
fn finer_continuous_reference_meshes_have_measured_acceptance() {
    for length in [0.,2.] { calibrate(0.02,48,length,true); }
}

#[test]
#[ignore = "diagnoses roll-query limits independently of spatial meshing"]
fn rotating_sphere_box_queries_at_known_exterior_points() {
    use gcs_core::interval::minimum::{Options,Stop};
    let field = swept_field(0.);
    for budget in [64,1000,4000] {
        let mut evaluator = field.evaluator(4096);
        let mut unresolved = 0;
        let mut evaluations = 0;
        for x in -1..=1 { for y in -1..=1 { for z in -1..=1 {
            if x == 0 && y == 0 && z == 0 { continue; }
            let len = ((x*x+y*y+z*z) as f64).sqrt();
            let direction = [x,y,z].map(|v| v as f64/len);
            let p: [f64;3] = std::array::from_fn(|k| if k == 0 {3.+1.03*direction[k]} else {1.03*direction[k]});
            let b = p.map(|v| I::new(v-0.0025,v+0.0025).unwrap());
            let mut squared = I::ZERO;
            for k in 0..3 { squared = squared.add(b[k].sub(I::point(if k == 0 {3.} else {0.}).unwrap()).unwrap().square().unwrap()).unwrap(); }
            assert!(squared.sqrt().unwrap().bounds()[0] > 1.02);
            let v = evaluator.bounds_stopping(b,Stop::Outside(I::ZERO),Options {value_tolerance:0.0025,max_evaluations:budget}).unwrap();
            evaluations += v.sweeps.iter().map(|s| s.minimum.evaluations).sum::<usize>();
            if v.value.contains(0.) {
                unresolved += 1;
                eprintln!("budget={budget} point={p:?} value={:?}",v.value);
            }
        } } }
        eprintln!("budget={budget}: {unresolved}/26 unresolved, {evaluations} roll evaluations");
    }
}
