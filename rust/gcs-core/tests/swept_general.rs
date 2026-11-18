//! **The contact's road under any motion** (`brep::swept::general`, #880 T1d): a tool carried by a
//! motion no constant twist takes, its candidates edges whose numbers move with the roll — each
//! sharp edge's fan, each plane face's characteristic line — and the tool at either end, trimmed by
//! membership and judged. The contact itself is read exactly: its shape changes only where the
//! instants say, and each piece's ends stand where the faces' `n · v` say.
use gcs_core::brep::trajectory::{Screw,Trajectory};

/// brep-bench's `swept/tumbling-cube`'s motion: turned by `t/4` about the vertical line through
/// `(2, 0, 0)`, seen from a frame turning by `t` about the line through `(0, 0.5, 0.5)` along `x`.
fn tumble() -> Trajectory {
    let own = Trajectory::Twist(Screw {origin:[2.,0.,0.],axis:[0.,0.,1.],ratio:0.25,advance:0.});
    let frame = Trajectory::Twist(Screw {origin:[0.,0.5,0.5],axis:[1.,0.,0.],ratio:1.,advance:0.});
    Trajectory::Relative {source:Box::new(own),observer:Box::new(frame)}
}

/// The cube of edge 1.2 about `(3, 0, 2)`.
fn cube() -> gcs_core::brep::topo::Brep { crate::brep::block([2.4,-0.6,1.4],[3.6,0.6,2.6]) }

/// **A cube tumbling a short way is built closed** by the general road and judged. Not yet: a fan's
/// stretch is born at a point of its edge at 0.2003 of the roll, where the edge's carried line has a
/// pinch point — its pieces now close, the spur its sides' merge leaves dropped — and edge 1's strip
/// and edge 5's, both swept by edges of the cube's face x = 3.6, stand all but coplanar, and the trim
/// keeps edge 5's piece short of the rail they share (`general`'s module).
#[test]
#[ignore = "T1d: two all but coplanar strips meeting along a shared rail, one kept short of it"]
fn a_cube_tumbling_a_little_is_built_by_the_general_road() {
    let cube = cube();
    let roll = [-15f64.to_radians(),15f64.to_radians()];
    let tol = gcs_core::brep::geom::tolerance(cube.size());
    let path = tumble();
    let (candidates,given) = gcs_core::brep::swept::general::candidates(&cube,&path,roll,tol).unwrap();
    let size = cube.size();
    let trim = gcs_core::brep::swept::trim::Trim {resolution:size/32.,hair:(4.*tol).max(1e-6*size),reach:None};
    let (swept,report) = gcs_core::brep::swept::trim::trimmed(&cube,&path,roll,&candidates,&given,cube.size(),tol,trim).unwrap();
    eprintln!("{report:?}");
    swept.check(tol).unwrap();
    eprintln!("{} faces, volume {}",swept.faces.len(),gcs_core::brep::props::volume(&swept));
}

/// **The paths the general road gives lie on both their walls**: each given rail — a vertex's path
/// two fans share, the path where a face's chord reaches an edge of it, over each stretch holding
/// that end — within a millionth of the cube of both surfaces it is given for, read at seventeen
/// points along it. (Not yet held: a wall's meeting with the cube at an end of the roll, marched in
/// the wall's parameters, stands 1.1e-3 off the wall near a corner of the cube there.)
#[test]
fn the_paths_the_general_road_gives_lie_on_both_their_walls() {
    let cube = cube();
    let roll = [-15f64.to_radians(),15f64.to_radians()];
    let tol = gcs_core::brep::geom::tolerance(cube.size());
    let (candidates,given) = gcs_core::brep::swept::general::candidates(&cube,&tumble(),roll,tol).unwrap();
    assert!(candidates.len() > 2 && !given.is_empty(),"{} candidates, {} given",candidates.len(),given.len());
    let mut read = 0;
    for g in &given { for c in g.curves.iter().filter(|c| matches!(c,gcs_core::brep::geom::Curve::Rail(..))) {
        let [lo,hi] = gcs_core::brep::query::curve_domain(c).unwrap();
        for k in 0..=16 {
            let p = c.point(lo+(hi-lo)*k as f64/16.);
            for s in &g.surfaces {
                let q = s.point(s.inverse(p));
                let off = ((q[0]-p[0]).powi(2)+(q[1]-p[1]).powi(2)+(q[2]-p[2]).powi(2)).sqrt();
                assert!(off <= 1e-6*cube.size(),"a {} given {off:e} off a {}, at {p:?} ({k} of 16), its foot {q:?}",c.kind(),s.kind());
            }
        }
        read += 1;
    } }
    assert!(read > 0);
    eprintln!("{} candidates, {read} given rails",candidates.len());
}
