//! Creases of a material field found from the field itself (`solid::crease`), checked against
//! shapes whose creases are closed forms.
use gcs_core::{delaunay::refine,solid::{crease,MaterialField},*};

fn field(source: &str,name: &str) -> MaterialField {
    let (mut p,errors) = syntax::parse(source);
    assert!(errors.is_empty(),"{errors:?}");
    assert!(modules::link(&mut p,&mut library::resolve).is_empty());
    let mut e = program::elaborate(&p);
    assert!(e.ok(),"{:?}",e.diags);
    assert!(solve::solve(&mut e.sketch,Default::default()).success);
    MaterialField::read(&e.sketch,e.map.ent_named(name).unwrap().i(),1e-10).unwrap()
}

/// The field meshed without features, coarsely, for its seeds.
fn unprotected(field: &MaterialField,centre: [f64;3],radius: f64,facet: f64) -> refine::Mesh {
    let criteria = refine::Criteria {facet_size:facet,facet_distance:facet/20.,facet_angle:25.,edge_size:facet,
        bisection:1e-7*radius,max_points:200_000,normal_angle:0.};
    let f = field.clone();
    let mut run = refine::Progressive::new(Box::new(move |p| f.side(p)),centre,radius,Vec::new(),criteria);
    while !run.step(usize::MAX).unwrap() {}
    // an unprotected sharp edge may leave the surface short of a manifold: its points still seed
    run.snapshot()
}

#[test]
fn a_pierced_spheres_two_rims_are_found_and_followed() {
    // A sphere of radius 20 with a hole of radius 6 through it square to the page, 5 off centre.
    // The page (`std.front`) is the world's x–z plane, so the hole runs along y, and the creases
    // are the two loops where |p| = 20 and (x − 5)² + z² = 36, one either side of the page.
    let f = field(include_str!("../../examples/pierced_sphere.sv"),"part");
    let mesh = unprotected(&f,[0.;3],21.,3.);
    let options = crease::CreaseOptions {step:1.,tolerance:1e-9,time_gap:0.1,centre:[0.;3],radius:21.,max_points:2000};
    let seeds = crease::seeds(&f,&mesh.vertices,&mesh.triangles,&options);
    assert!(!seeds.is_empty(),"no edge of {} triangles crosses a crease",mesh.triangles.len());
    let found = crease::creases(&f,&seeds,&options);
    eprintln!("{} seeds, {} creases: {:?}",seeds.len(),found.len(),found.iter().map(|c| (c.points.len(),c.closed)).collect::<Vec<_>>());
    assert_eq!(found.len(),2,"two rims");
    for c in &found {
        assert!(c.closed,"a rim is a closed loop");
        for p in &c.points {
            let r = (p[0]*p[0]+p[1]*p[1]+p[2]*p[2]).sqrt();
            let h = ((p[0]-5.).powi(2)+p[2]*p[2]).sqrt();
            assert!((r-20.).abs() < 1e-6 && (h-6.).abs() < 1e-6,"{p:?}: sphere {r}, hole {h}");
        }
        // one rim either side of the page
        assert!(c.points.iter().all(|p| p[1] > 0.) || c.points.iter().all(|p| p[1] < 0.));
    }
}

const BLOCK: &str = "unit mm\nuse std\n\
private point mid hint(x: 0, y: -6)\n\
std.origin vertical mid\n\
std.origin distance(6mm, along: down) mid\n\
private outline: CenteredRectangle(mid, w: 40mm, h: 12mm)\n\
solid block(outline.loop, from: -20mm, to: 20mm)\n";

#[test]
fn a_blocks_twelve_edges_are_found_and_end_at_its_corners() {
    // A 40 × 40 × 12 block: the page (`std.front`) is the world's x–z plane, so x across from -20
    // to 20, z up from -12 to 0 and y through the page from -20 to 20. One leaf, whose twelve
    // edges are creases between its pieces, each ending at two of its corners.
    let f = field(BLOCK,"block");
    let mesh = unprotected(&f,[0.,0.,-6.],30.,4.);
    let options = crease::CreaseOptions {step:1.,tolerance:1e-9,time_gap:0.1,centre:[0.,0.,-6.],radius:30.,max_points:2000};
    let seeds = crease::seeds(&f,&mesh.vertices,&mesh.triangles,&options);
    let found = crease::creases(&f,&seeds,&options);
    eprintln!("{} seeds, {} creases: {:?}",seeds.len(),found.len(),
        found.iter().map(|c| (c.points.len(),c.closed,c.points[0],*c.points.last().unwrap())).collect::<Vec<_>>());
    assert_eq!(found.len(),12,"twelve edges");
    let at = |v: f64,a: f64,b: f64| (v-a).abs() < 1e-6 || (v-b).abs() < 1e-6;
    let face = |p: &[f64;3]| [at(p[0],-20.,20.),at(p[1],-20.,20.),at(p[2],-12.,0.)].into_iter().filter(|&x| x).count();
    for c in &found {
        assert!(!c.closed);
        for p in &c.points { assert!(face(p) >= 2,"{p:?} is on no edge"); }
        // both ends are corners, found exactly
        for end in [c.points[0],*c.points.last().unwrap()] { assert_eq!(face(&end),3,"{end:?} is no corner"); }
    }
}

#[test]
fn a_swept_grooves_rim_is_one_loop_on_the_top_face() {
    // A ball of radius 4 whose centre swings on a circle of radius 14 in the top face (z = 0),
    // through 150° about the upright axis, cut into a block. The groove's rim is where the swept
    // ball meets the top: one loop, each point 4 from the nearest centre the swing reaches.
    let f = field(include_str!("../../examples/swept_groove.sv"),"part");
    let mesh = unprotected(&f,[0.,0.,-6.],30.,3.);
    let options = crease::CreaseOptions {step:0.75,tolerance:1e-9,time_gap:0.5,centre:[0.,0.,-6.],radius:30.,max_points:4000};
    let seeds = crease::seeds(&f,&mesh.vertices,&mesh.triangles,&options);
    let found = crease::creases(&f,&seeds,&options);
    let rim: Vec<_> = found.iter().filter(|c| c.closed).collect();
    eprintln!("{} seeds, {} creases, {} closed: {:?}",seeds.len(),found.len(),rim.len(),
        found.iter().map(|c| (c.points.len(),c.closed,c.points[0])).collect::<Vec<_>>());
    assert_eq!(found.len(),13,"the block's twelve edges and the rim");
    assert_eq!(rim.len(),1,"one closed rim");
    assert!(rim[0].points.iter().all(|p| p[2].abs() < 1e-6),"the rim is on the top face");
    let half = 75f64.to_radians();
    for p in &rim[0].points {
        let t = p[1].atan2(p[0]).clamp(-half,half);
        let d = ((p[0]-14.*t.cos()).powi(2)+(p[1]-14.*t.sin()).powi(2)).sqrt();
        assert!((d-4.).abs() < 1e-5,"{p:?} is {d} from the swing");
    }
}

#[test]
fn a_swept_groove_meshes_closed_with_its_rim_kept() {
    // The groove through the whole mesher: a first pass finds the creases, the second keeps them.
    // Its volume is the block's less the half of the swept ball below the top face: by Pappus a
    // torus segment through 150° and a ball, the two half balls at the ends.
    let (mut p,_) = syntax::parse(include_str!("../../examples/swept_groove.sv"));
    assert!(modules::link(&mut p,&mut library::resolve).is_empty());
    let mut e = program::elaborate(&p);
    assert!(solve::solve(&mut e.sketch,Default::default()).success);
    let clock = std::time::Instant::now();
    let surface = solid::FieldMesher::new(&e.sketch,e.map.ent_named("part").unwrap().i()).unwrap().finish().unwrap();
    let volume: f64 = surface.triangles.iter().map(|t| {
        let [a,b,c] = t.map(|i| surface.vertices[i as usize]);
        (a[0]*(b[1]*c[2]-b[2]*c[1])-a[1]*(b[0]*c[2]-b[2]*c[0])+a[2]*(b[0]*c[1]-b[1]*c[0]))/6.
    }).sum();
    let pi = std::f64::consts::PI;
    let exact = 40.*40.*12.-0.5*(pi*16.*14.*150f64.to_radians()+4./3.*pi*64.);
    eprintln!("groove: {} triangles in {:?}, volume {volume} against {exact}",surface.triangles.len(),clock.elapsed());
    assert!(!surface.provisional);
    assert!((volume-exact).abs() < 0.003*exact,"volume {volume} against {exact}");
}
