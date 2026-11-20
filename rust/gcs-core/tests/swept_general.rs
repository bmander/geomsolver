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

/// **A cube tumbling a short way is built closed** by the general road and judged. Not yet at 15°:
/// one of its fans passes back through itself and is cut into two runs, their crossing given
/// (#942 T1d-cube), and the trim then leaves the boundary open along a rail edge, face 4 of candidate
/// 8 parted by a meeting it was not given.
#[test]
#[ignore = "T1d-cube: the boundary open along a rail edge, a meeting the trim was not given"]
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

/// **A cube tumbling two degrees either way is built by the general road through `closed`** (#880
/// T1d): no convex solid of revolution, so K2 refused it (`Curved` at its corner) before reading
/// anything; `closed` now asks the general road, its candidates trimmed by membership and judged, and
/// the solid is held to membership at points through its box. (Turned farther, the bench's 75°, its
/// face x = 3.6 moving all but in its own plane, the trim's meetings near where three of its sheets
/// meet do not yet join.)
#[test]
fn a_cube_tumbling_two_degrees_is_built_by_the_general_road() {
    let cube = cube();
    let roll = [-2f64.to_radians(),2f64.to_radians()];
    let tol = gcs_core::brep::geom::tolerance(cube.size());
    let path = tumble();
    let built = gcs_core::brep::swept::closed::closed(&cube,&path,roll,cube.size(),tol).unwrap_or_else(|u| panic!("{u:?}"));
    built.check(tol).unwrap();
    crate::swept_closed::agrees_over(&built,&cube,&path,roll);
}

/// **A cube tumbling five degrees either way is built** (#880 T1d): its top front edge's fan and its
/// top side edge's, sharing the path of the corner between them, cross a second time beside that
/// path, which the road traces (`general::beside_rails`) — given the path alone, the arrangement
/// sought nothing more of the pair and left a hole between the two fans.
#[test]
fn a_cube_tumbling_five_degrees_is_built_by_the_general_road() {
    let cube = cube();
    let roll = [-5f64.to_radians(),5f64.to_radians()];
    let tol = gcs_core::brep::geom::tolerance(cube.size());
    let path = tumble();
    let built = gcs_core::brep::swept::closed::closed(&cube,&path,roll,cube.size(),tol).unwrap_or_else(|u| panic!("{u:?}"));
    built.check(tol).unwrap();
    crate::swept_closed::agrees_over(&built,&cube,&path,roll);
}

/// What the general road gives the trim for the cube tumbling `deg` either way, as `closed` gives
/// it: the tool, the motion, the roll, the candidates, their given meetings, and how the trim reads.
fn cube_trimming(deg: f64) -> gcs_core::brep::swept::dump::Trimming {
    let tool = cube();
    let roll = [-deg.to_radians(),deg.to_radians()];
    let tol = gcs_core::brep::geom::tolerance(tool.size());
    let path = tumble();
    let (candidates,given) = gcs_core::brep::swept::general::candidates(&tool,&path,roll,tol).unwrap();
    let size = tool.size();
    let trim = gcs_core::brep::swept::trim::Trim {resolution:size/32.,hair:(4.*tol).max(1e-6*size),reach:None};
    gcs_core::brep::swept::dump::Trimming {bar:tool.size(),tool,path,roll,candidates,given,tol,trim}
}

/// The refusal a trim replayed from its dump gives (#942 F5): what `trim.replay()` refused, then the
/// same trim written (`dump_trim`), read back (`load_trim`) and replayed refusing alike.
fn replayed_alike(trimming: &gcs_core::brep::swept::dump::Trimming,name: &str) -> String {
    use gcs_core::brep::swept::dump::{dump_trim,load_trim};
    let direct = format!("{:?}",trimming.replay().expect_err("the trim refuses"));
    let dir = std::env::temp_dir().join(format!("{name}-{}",std::process::id()));
    let t = trimming;
    dump_trim(&dir,&t.tool,&t.path,t.roll,&t.candidates,&t.given,t.bar,t.tol,t.trim,&direct).unwrap();
    let loaded = load_trim(&dir).unwrap();
    let _ = std::fs::remove_dir_all(&dir);
    let again = format!("{:?}",loaded.replay().expect_err("the replayed trim refuses"));
    assert_eq!(again,direct,"replayed from its dump, the trim refuses otherwise");
    direct
}

/// **The general road's candidates are written whole** (#942 F5): its walls (`Surface::Family`),
/// the rails and rims along them, and the meetings it gives — each written, read back and written
/// again to the same text, so a trim of them is run again bit for bit apart from the road.
#[test]
fn the_general_roads_candidates_are_written_and_read_back_whole() {
    use gcs_core::brep::swept::dump::{dump_trim,load_trim,read,write};
    let t = cube_trimming(2.);
    let kinds: std::collections::BTreeSet<&str> = t.candidates.iter().flat_map(|c| c.faces.iter().map(|f| f.surface.kind())).collect();
    assert!(kinds.contains("family"),"the road's walls are face families: {kinds:?}");
    for c in &t.candidates {
        let text = write(c).unwrap();
        let back = read(&text).unwrap();
        assert_eq!(write(&back).unwrap(),text);
    }
    let dir = std::env::temp_dir().join(format!("general-dump-{}",std::process::id()));
    dump_trim(&dir,&t.tool,&t.path,t.roll,&t.candidates,&t.given,t.bar,t.tol,t.trim,"a test").unwrap();
    let loaded = load_trim(&dir).unwrap();
    let _ = std::fs::remove_dir_all(&dir);
    assert_eq!(loaded.given.len(),t.given.len());
    for (g,h) in loaded.given.iter().zip(&t.given) { assert_eq!(g.curves,h.curves); assert_eq!(g.surfaces,h.surfaces) }
    for (k,c) in loaded.candidates.iter().enumerate() { assert_eq!(write(c).unwrap(),write(&t.candidates[k]).unwrap()) }
    assert_eq!(loaded.roll,t.roll);
    assert_eq!(format!("{:?}",loaded.path.pose(0.3).unwrap()),format!("{:?}",t.path.pose(0.3).unwrap()));
    // (and the trim replayed builds what the road built)
    let (built,_) = loaded.replay().unwrap();
    let (direct,_) = t.replay().unwrap();
    assert_eq!(gcs_core::brep::props::volume(&built),gcs_core::brep::props::volume(&direct));
    // (and its questions answer from the files: the boundary up through the top lies on a candidate
    // within the tolerance, and a given rail's own point is found on that rail)
    let crossings = loaded.boundary_along([3.,0.,2.6],[0.,0.,1.],0.3,200).unwrap();
    assert_eq!(crossings.len(),1,"{crossings:?}");
    assert_eq!((crossings[0].from,crossings[0].to),(Some(true),Some(false)));
    assert!(crossings[0].on.first().is_some_and(|n| n.off.abs() <= loaded.tol),"{crossings:?}");
    let rail = loaded.given.iter().flat_map(|g| &g.curves).find(|c| c.kind() == "rail").expect("a rail given");
    let d = gcs_core::brep::query::curve_domain(rail).unwrap();
    let at = rail.point(0.5*(d[0]+d[1]));
    let near = loaded.through(at,1e-9);
    assert!(near.given.iter().any(|g| g.2 == "rail" && g.3 <= 1e-9),"{:?}",near.given);
    assert!(near.faces.len() >= 2,"a rail stands on both its walls: {:?}",near.faces);
}

/// **A refused trim replays from its dump to the same refusal** (#942 F5's gate, fast): the cube
/// tumbling 15° either way, refused in its trim, gives the same refusal run again from the files —
/// its fan cut into runs, each a wall restricted to its stretches and the rails given it restricted
/// alike, and every wall written with the number it is shared by, so the walls read back are shared
/// as they were built (a wall's inverse starts from its last foot: read back apart, the refusal's
/// points moved in their last digits).
#[test]
fn a_refused_trim_replays_from_its_dump_to_the_same_refusal() {
    let why = replayed_alike(&cube_trimming(15.),"cube-15-dump");
    assert!(why.contains("the boundary is open along a rail edge"),"{why}");
}

/// **The bench's tumbling cube's refused trim replays from its dump alike** (#942 F5's gate): the
/// cube tumbling 75° either way, its candidates taken 17 s to make, refused in its trim, gives the
/// same refusal run again from the files in a fraction of that. Refused by name since #942 F2 where
/// its edge 5's fan passed back through itself, a meeting of the fan with itself that no given
/// meeting of two surfaces lays; since #942 T1d-cube that fan is cut into two runs, two candidates
/// whose crossing is given, and the trim gets past it. Two other fans' second crossing beside their
/// shared rail, once traced past the corner they touch at and left hanging in a face, is ended
/// there (`general::ended_on_rails`), and the trim is refused instead as the 15° cube is, the
/// boundary open along a rail edge.
#[test]
#[cfg_attr(not(feature = "slow"),ignore = "slow tier, 40 s: the bench's tumbling cube's candidates made, then its trim twice")]
fn the_bench_tumbling_cubes_refused_trim_replays_from_its_dump_alike() {
    let why = replayed_alike(&cube_trimming(75.),"cube-75-dump");
    // (no fan passing back through itself any more: its runs' crossing given; nor a branch beside a
    // rail left hanging, #942 T1d-cube)
    assert!(why.contains("the boundary is open along a rail edge") && !why.contains("passing back through itself"),"{why}");
}

/// **A meeting left out of what the trim is given is completed from its witness** (#942 F2's gate):
/// the cube tumbling 5° either way, which builds, its fans' second branches beside their rails taken
/// out of the meetings given (`general::beside_rails`'s, the hole the 5° cube once had). The trim
/// reads a piece the missing branch parts — the boundary at one point, off it at another — traces
/// the meeting from where it crosses between them, and builds the solid the whole given builds, to
/// its volume, where it once left the boundary open where no one said.
#[test]
fn a_meeting_left_out_of_the_trim_is_completed_from_its_witness() {
    let whole = cube_trimming(5.);
    let (built,report) = whole.replay().expect("the cube tumbling 5° trims");
    assert_eq!(report.completed,0,"the whole given needs nothing completed");
    let mut t = whole.clone();
    let mut taken = 0;
    for g in &mut t.given {
        if g.curves.iter().any(|c| c.kind() == "rail") {
            let before = g.curves.len();
            g.curves.retain(|c| c.kind() == "rail");
            taken += before-g.curves.len();
        }
    }
    assert!(taken > 0,"a branch beside a rail to take out");
    let (completed,report) = t.replay().expect("the trim completes the branch left out from its witness");
    assert!(report.completed > 0,"a meeting completed");
    let (v,w) = (gcs_core::brep::props::volume(&built),gcs_core::brep::props::volume(&completed));
    assert!((v-w).abs() <= 1e-9*v.abs(),"completed, the trim builds {w}, the whole given {v}");
}

/// A rise of `6s + 0.3s²` after `s` turns (brep-bench's `swept/square-thread-at-a-growing-lead`):
/// a lead growing from 6 by 0.6 a turn.
#[derive(Debug)]
struct Rise;

impl Rise { fn of(s: f64) -> f64 { 6.*s+0.3*s*s } }

impl gcs_core::brep::trajectory::Pace for Rise {
    fn jet(&self,theta: f64) -> Result<[f64;2],gcs_core::brep::KernelError> {
        let s = theta/std::f64::consts::TAU;
        Ok([Rise::of(s),(6.+0.6*s)/std::f64::consts::TAU])
    }
    fn range(&self,[a,b]: [f64;2]) -> Result<[f64;2],gcs_core::brep::KernelError> {
        let (s0,s1) = (a.min(b)/std::f64::consts::TAU,a.max(b)/std::f64::consts::TAU);
        let mut vs = vec![Rise::of(s0),Rise::of(s1)];
        if s0 < -10. && -10. < s1 { vs.push(Rise::of(-10.)) }
        Ok([vs.iter().cloned().fold(f64::INFINITY,f64::min),vs.iter().cloned().fold(f64::NEG_INFINITY,f64::max)])
    }
    fn enclose(&self,roll: gcs_core::brep::util::interval::Interval)
        -> Result<[gcs_core::brep::util::interval::Interval;2],gcs_core::brep::util::interval::Error> {
        use gcs_core::brep::util::interval::{Error,Interval};
        let [a,b] = roll.bounds();
        let [lo,hi] = self.range([a,b]).map_err(|_| Error::InvalidBounds)?;
        let rate = |t: f64| (6.+0.6*t/std::f64::consts::TAU)/std::f64::consts::TAU;
        let wide = |lo: f64,hi: f64| Interval::new(lo-lo.abs()*1e-15-1e-300,hi+hi.abs()*1e-15+1e-300);
        Ok([wide(lo,hi)?,wide(rate(a).min(rate(b)),rate(a).max(rate(b)))?])
    }
}

/// The bar `4 ≤ x ≤ 12, −8 ≤ y ≤ 8, −1 ≤ z ≤ 1` turned about `z` and risen at `Rise`'s pace, seen from
/// a spindle turning back under it (brep-bench's thread at a growing lead).
fn growing_lead() -> (gcs_core::brep::topo::Brep,Trajectory) {
    let up = [0.,0.,1.];
    let rise = Trajectory::Twist(Screw {origin:[0.;3],axis:up,ratio:0.,advance:std::f64::consts::TAU});
    let tool = Trajectory::Paced {path:Box::new(rise),pace:std::sync::Arc::new(Rise)};
    let spindle = Trajectory::Twist(Screw {origin:[0.;3],axis:up,ratio:-1.,advance:0.});
    (crate::brep::block([4.,-8.,-1.],[12.,8.,1.]),Trajectory::Relative {source:Box::new(tool),observer:Box::new(spindle)})
}

/// **A bar under a screw whose lead grows, a third of a turn, is built by the general road** (#880
/// T1d): its two vertical edges at one radius each sweep the one cylinder, taken as that cylinder
/// (`on_a_cylinder`), and its bottom edge's strip is cut beside the vertex where it leaves the bar
/// posed at the start (`pose_meeting`'s branch from the rim's corner, each end at its own nearest
/// corner). Held to membership.
#[test]
fn a_bar_under_a_growing_lead_a_third_of_a_turn_is_built_by_the_general_road() {
    let (bar,path) = growing_lead();
    let roll = [-90f64.to_radians(),0.5];
    let tol = gcs_core::brep::geom::tolerance(bar.size());
    let built = gcs_core::brep::swept::closed::closed(&bar,&path,roll,bar.size(),tol).unwrap_or_else(|u| panic!("{u:?}"));
    built.check(tol).unwrap();
    crate::swept_closed::agrees_over(&built,&bar,&path,roll);
}

/// **A rod threaded at a lead that grows along it** (brep-bench `swept/square-thread-at-a-growing-lead`,
/// #880 T1d): the bar carried three and three-quarter turns up the rod at the growing lead, its sweep
/// built by the general road and cut from the rod (`cut_from`), held to the volume and area OCCT and
/// cadrs agree on (to 8e-10 and 2e-10), at a billionth (7.5e-11 off). Once K2's `Curved` at the bar's
/// corner.
#[test]
#[cfg_attr(not(feature = "slow"),ignore = "slow tier, 24 s: the growing-lead thread cut from its rod")]
fn a_rod_threaded_at_a_growing_lead_is_its_bench_volume() {
    let (bar,path) = growing_lead();
    let up = [0.,0.,1.];
    let rod = gcs_core::brep::build::prism(&gcs_core::brep::build::disc([0.;3],up,5.),0.,20.).unwrap();
    let roll = [-90f64.to_radians(),1260f64.to_radians()];
    let tol = gcs_core::brep::geom::tolerance(rod.size().max(bar.size()));
    let cut = gcs_core::brep::swept::closed::cut_from(&rod,&bar,&path,roll,bar.size(),tol).unwrap_or_else(|u| panic!("{u:?}"));
    cut.check(tolerance_of(&cut)).unwrap();
    let v = gcs_core::brep::props::volume(&cut);
    let a: f64 = cut.faces.iter().map(|f| gcs_core::brep::props::area(&cut,f)).sum();
    assert!((v/1324.478271075-1.).abs() <= 1e-9,"the threaded rod's volume {v} against 1324.478271075");
    assert!((a/808.437626175-1.).abs() <= 1e-9,"its surface {a} against 808.437626175");
}

fn tolerance_of(b: &gcs_core::brep::topo::Brep) -> f64 { gcs_core::brep::geom::tolerance(b.size()) }

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
