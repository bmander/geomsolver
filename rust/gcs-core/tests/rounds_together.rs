//! **Edges rounded together** (`brep::rounds::rounded`): a set of a solid's edges rounded in one
//! call, the kernel placing each round and each corner's ball. Each solid is held to its closed
//! forms: a convex polyhedron whose every edge is rounded to `r` is the polyhedron its faces moved
//! in by `r` bound, grown by the ball, so by Steiner's formula its volume is `V′ + S′ r + M r² +
//! 4πr³/3` and its area `S′ + 2 M r + 4πr²`, with `M = Σ l (π − θ) / 2` over the inner one's
//! edges (`θ` the angle between the faces there).
use gcs_core::brep::boolean::{combined,Op};
use gcs_core::brep::build::{prism,Profile,ProfileEdge};
use gcs_core::brep::geom::{tolerance,Rigid,V};
use gcs_core::brep::props::{area,volume};
use gcs_core::brep::rounds::rounded;
use gcs_core::brep::topo::Brep;
use std::collections::BTreeMap;
use std::f64::consts::PI;

fn polygon(pts: &[[f64;2]]) -> Profile {
    let p: Vec<V> = pts.iter().map(|q| [q[0],q[1],0.]).collect();
    Profile {origin:[0.;3],normal:[0.,0.,1.],names:vec![],
        loops:vec![(0..p.len()).map(|i| ProfileEdge::Line {a:p[i],b:p[(i+1)%p.len()]}).collect()]}
}

fn block(lo: V,hi: V) -> Brep {
    let mut p = polygon(&[[lo[0],lo[1]],[hi[0],lo[1]],[hi[0],hi[1]],[lo[0],hi[1]]]);
    p.origin = [0.;3];
    prism(&p,lo[2],hi[2]).unwrap()
}

/// The edges of `b` whose middles `pick` takes.
fn edges_where(b: &Brep,pick: impl Fn(V) -> bool) -> Vec<usize> {
    (0..b.edges.len()).filter(|&e| { let ed = &b.edges[e]; pick(ed.point(0.5*(ed.t[0]+ed.t[1]),&b.vertices)) }).collect()
}

fn all(b: &Brep) -> Vec<usize> { (0..b.edges.len()).collect() }

fn kinds(b: &Brep) -> BTreeMap<&'static str,usize> {
    let mut k = BTreeMap::new();
    for f in &b.faces { *k.entry(f.surface.kind()).or_insert(0) += 1; }
    k
}

fn total_area(b: &Brep) -> f64 { b.faces.iter().map(|f| area(b,f)).sum() }

fn close(what: &str,got: f64,want: f64) {
    assert!((got-want).abs() <= 1e-12*want.abs().max(1.),"{what}: {got} against {want} ({:e})",(got-want)/want);
}

/// A box `a × b × c` with every edge rounded to `r`: its volume and area by Steiner's formula.
fn rounded_box(a: f64,b: f64,c: f64,r: f64) -> (f64,f64) {
    let [x,y,z] = [a-2.*r,b-2.*r,c-2.*r];
    let s = 2.*(x*y+y*z+z*x);
    let m = PI*(x+y+z);
    (x*y*z+s*r+m*r*r+4.*PI*r*r*r/3.,s+2.*m*r+4.*PI*r*r)
}

/// **A box rounded all over** (brep-bench's fillet/box-all-edges): twelve cylinders, the six faces
/// shrunk to the contacts and eight spheres closing the corners, all exact — its volume and area
/// Steiner's to rounding, and so wherever the box stands, turned off the axes.
#[test]
fn a_box_rounded_on_every_edge_is_planes_cylinders_and_spheres() {
    let cube = block([0.;3],[10.;3]);
    let round = rounded(&cube,&all(&cube),1.,tolerance(cube.size())).unwrap();
    round.check(1e-9).unwrap();
    assert_eq!(kinds(&round),BTreeMap::from([("cylinder",12),("plane",6),("sphere",8)]));
    let (v,a) = rounded_box(10.,10.,10.,1.);
    close("the cube's volume",volume(&round),v);
    close("the cube's area",total_area(&round),a);
    // a box of three sides, turned and moved off the axes
    let turned = block([0.;3],[12.,7.,4.]).moved(&Rigid::turn([3.,-1.,2.],[0.3,-0.5,0.8],0.9));
    let round = rounded(&turned,&all(&turned),1.5,tolerance(turned.size())).unwrap();
    round.check(1e-9).unwrap();
    let (v,a) = rounded_box(12.,7.,4.,1.5);
    close("the turned box's volume",volume(&round),v);
    close("the turned box's area",total_area(&round),a);
}

/// **A triangular prism rounded all over**: faces meeting at the triangle's angles along its
/// vertical edges and square along the rest, each corner's ball touching three faces none square
/// to the others. The faces moved in by `r` bound the triangle shrunk about its incentre by `(ρ −
/// r) / ρ` (`ρ` its inradius) between planes `2r` nearer: `M = π (h − 2r) + π k P / 2`, the
/// triangle's angles turning a whole turn round its vertical edges.
#[test]
fn a_triangular_prism_rounded_on_every_edge_is_its_steiner_body() {
    let pts = [[0.,0.],[12.,0.],[3.,8.]];
    let (h,r) = (7.,1.);
    let p = prism(&polygon(&pts),0.,h).unwrap();
    let round = rounded(&p,&all(&p),r,tolerance(p.size())).unwrap();
    round.check(1e-9).unwrap();
    assert_eq!(kinds(&round),BTreeMap::from([("cylinder",9),("plane",5),("sphere",6)]));
    let side = |i: usize| { let (a,b) = (pts[i],pts[(i+1)%3]); ((a[0]-b[0]).powi(2)+(a[1]-b[1]).powi(2)).sqrt() };
    let perimeter = side(0)+side(1)+side(2);
    let t = 0.5*((pts[1][0]-pts[0][0])*(pts[2][1]-pts[0][1])-(pts[2][0]-pts[0][0])*(pts[1][1]-pts[0][1])).abs();
    let rho = 2.*t/perimeter;
    let k = (rho-r)/rho;
    let hh = h-2.*r;
    let (v0,s0) = (k*k*t*hh,2.*k*k*t+k*perimeter*hh);
    let m = PI*hh+PI*k*perimeter/2.;
    close("the prism's volume",volume(&round),v0+s0*r+m*r*r+4.*PI*r.powi(3)/3.);
    close("the prism's area",total_area(&round),s0+2.*m*r+4.*PI*r*r);
}

/// **A cavity's edges filled** (every round concave): a block of 20 with a cube of 10 hollowed out
/// of it, the cavity's twelve edges and eight corners filled to radius 2 — the cavity a box
/// rounded all over, the cylinders and spheres facing into it.
#[test]
fn a_cavitys_edges_rounded_fill_it_into_a_rounded_box() {
    let hollow = combined(&block([-5.;3],[15.;3]),&block([0.;3],[10.;3]),Op::Cut,0.).unwrap();
    let inside = |q: V| q.iter().all(|&x| (-1e-9..=10.+1e-9).contains(&x));
    let edges = edges_where(&hollow,inside);
    assert_eq!(edges.len(),12);
    let round = rounded(&hollow,&edges,2.,tolerance(hollow.size())).unwrap();
    round.check(1e-9).unwrap();
    assert_eq!(kinds(&round),BTreeMap::from([("cylinder",12),("plane",12),("sphere",8)]));
    assert!(round.faces.iter().filter(|f| f.surface.kind() != "plane").all(|f| f.reversed));
    let (v,a) = rounded_box(10.,10.,10.,2.);
    close("the hollow block's volume",volume(&round),8000.-v);
    close("the hollow block's area",total_area(&round),2400.+a);
}

/// **Rounds that meet no other** are rolled each in turn (`fillet::roll`): one edge of a cube, its
/// piece cut off — brep-bench's fillet/box-one-edge, `1000 − (4 − π) 10` — and two edges across
/// the cube from each other.
#[test]
fn rounds_that_meet_no_other_are_rolled_in_turn() {
    let cube = block([0.;3],[10.;3]);
    let tol = tolerance(cube.size());
    let one = rounded(&cube,&edges_where(&cube,|q| q[0] == 10. && q[1] == 10.),2.,tol).unwrap();
    assert!((volume(&one)-(1000.-(4.-PI)*10.)).abs() <= 1e-9*1000.,"{}",volume(&one));
    let two = rounded(&cube,&edges_where(&cube,|q| (q[0] == 10. && q[1] == 10.) || (q[0] == 0. && q[1] == 0.)),2.,tol).unwrap();
    assert!((volume(&two)-(1000.-2.*(4.-PI)*10.)).abs() <= 1e-9*1000.,"{}",volume(&two));
}

/// What the surgery cannot close is refused by name: rounds meeting at a corner whose third edge
/// stays sharp (a box's top rim alone), and a round wider than the faces beside it can hold.
#[test]
fn rounds_it_cannot_close_are_refused_by_name() {
    let cube = block([0.;3],[10.;3]);
    let tol = tolerance(cube.size());
    let why = rounded(&cube,&edges_where(&cube,|q| q[2] == 10.),1.,tol).unwrap_err();
    assert!(why.contains("not every one is rounded"),"{why}");
    let why = rounded(&cube,&all(&cube),5.5,tol).unwrap_err();
    assert!(why.contains("larger than the face"),"{why}");
}
