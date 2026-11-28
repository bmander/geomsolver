//! **A thread hobbing a wheel** (`brep::swept::generated`): a worm's thread turned about its own
//! axis while the wheel turns, read as a rack slid along that axis, each point of its faces touching
//! the sweep's boundary at one instant in closed form; one tooth space's sheets trimmed within its
//! sector of the wheel by membership, judged, cut from the blank's sector and patterned round.
use crate::swept_rings::polygon;
use gcs_core::brep::build::{disc,kept,prism,screwed};
use gcs_core::brep::geom::Rigid;
use gcs_core::brep::topo::Brep;
use gcs_core::brep::trajectory::{Screw,Trajectory};
use gcs_core::space::{add,cross,scale,unit};
use std::f64::consts::{FRAC_PI_2,PI,TAU};

type V = [f64;3];

/// brep-bench's worm (`swept/worm-wheel-hobbed-by-its-worm`): its thread on the line through `at`
/// along x from x = `from` to `to` — a single start of lead 2π, its axial section the trapezoid π
/// wide at radius 10 from radius 7.5 to 12.5, its flanks 20° from square to the axis — built about
/// z, screwed a turn past each end and cut off there, then laid along x.
fn worm_thread(at: V,from: f64,to: f64) -> Brep {
    let w = |r: f64| FRAC_PI_2-(r-10.)*20f64.to_radians().tan();
    let z0 = -TAU*((-from/TAU).ceil()+1.)-FRAC_PI_2;
    let section = polygon(&[[7.5,0.,z0-w(7.5)],[12.5,0.,z0-w(12.5)],[12.5,0.,z0+w(12.5)],[7.5,0.,z0+w(7.5)]]);
    let turns = ((to-z0)/TAU).ceil()+1.;
    let thread = screwed(&section,[0.;3],[0.,0.,1.],TAU*turns,TAU*turns).unwrap();
    let thread = kept(&thread,[0.,0.,from],[0.,0.,1.],0.).unwrap();
    let thread = kept(&thread,[0.,0.,to],[0.,0.,-1.],0.).unwrap();
    thread.moved(&Rigid::turn([0.;3],[0.,1.,0.],FRAC_PI_2).then(&Rigid {t:at,..Rigid::identity()}))
}

/// The worm turning at 1 about its axis through `at` along x, seen from the wheel turning at 1/30
/// about z.
fn in_mesh(at: V) -> Trajectory {
    let worm = Trajectory::Twist(Screw {origin:at,axis:[1.,0.,0.],ratio:1.,advance:0.});
    let wheel = Trajectory::Twist(Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:1./30.,advance:0.});
    Trajectory::Relative {source:Box::new(worm),observer:Box::new(wheel)}
}

/// The worm wheel's blank: the disc of radius 32 about z from z = −6 to 6.
fn wheel() -> Brep { prism(&disc([0.,0.,-6.],[0.,0.,1.],32.),0.,12.).unwrap() }

/// The roll: thirty turns of the worm, one of the wheel.
const ROLL: [f64;2] = [0.,60.*PI];

/// The cylinder just past the wheel the road reads within.
fn reach() -> gcs_core::brep::swept::trim::Reach {
    gcs_core::brep::swept::trim::Reach::Turned {o:[0.;3],z:[0.,0.,1.],r:32.1,span:[-6.1,6.1]}
}

/// **Each sheet of a tooth space lies on the worm wheel's boundary**: one candidate — the worm's two
/// flanks', its two crest corners' and its crest's sheets, sharing their rails — read within one
/// sector a pitch wide, and at a point of each sheet within the wheel (its nearest the mid-plane),
/// the sweep's own membership (the thread turned by the two turns over the whole roll, by branch
/// and bound, meshed as the build meshes it) differs four of its sags to either side.
#[test]
fn each_sheet_of_a_tooth_space_lies_on_the_worm_wheels_boundary() {
    use gcs_core::brep::swept::member::{Solid,Swept};
    let at = [0.,40.,0.];
    let (thread,path) = (worm_thread(at,-20.,20.),in_mesh(at));
    let tol = gcs_core::brep::geom::tolerance(wheel().size());
    let found = gcs_core::brep::swept::generated::candidates(&thread,&path,ROLL,&reach(),tol).unwrap_or_else(|u| panic!("{u:?}"))
        .expect("the worm's thread is the rack's road's");
    let (sector,count) = found.sector.expect("one tooth space within a sector");
    assert_eq!(count,30);
    let gcs_core::brep::swept::trim::Reach::Sector {pitch,..} = sector else { panic!("{sector:?} no sector") };
    assert!((pitch-TAU/30.).abs() <= 1e-15);
    let [candidate] = &found.candidates[..] else { panic!("{} candidates for one copy",found.candidates.len()) };
    assert_eq!(candidate.faces.len(),5,"the flanks, the crest corners and the crest");
    let solid = Solid::new(&thread,0.01,1.).unwrap();
    let swept = Swept::new(&solid,&path,ROLL);
    let hair = 4.*solid.sag;
    // (one point a sheet, the nearest its middle within the wheel: each read is the whole roll's)
    for face in &candidate.faces {
        let gcs_core::brep::geom::Surface::BSpline(_,net) = &face.surface else { panic!("a sheet no net") };
        let [[u0,u1],[v0,v1]] = net.domain();
        let grid = (0..=8).flat_map(|i| (0..=32).map(move |j| (i,j)));
        let within = grid.map(|(i,j)| face.surface.d1([u0+(u1-u0)*(0.05+0.9*i as f64/8.),v0+(v1-v0)*j as f64/32.]))
            .filter(|(p,..)| p[0].hypot(p[1]) <= 31.5 && p[2].abs() <= 5.5)
            .min_by(|a,b| a.0[2].abs().total_cmp(&b.0[2].abs()));
        let Some((p,pu,pv)) = within else { panic!("{} has no point within the wheel",face.name) };
        let n = unit(cross(pu,pv));
        let (a,b) = (swept.member(add(p,scale(n,hair))),swept.member(add(p,scale(n,-hair))));
        assert!(matches!((&a,&b),(Ok(true),Ok(false)) | (Ok(false),Ok(true))),"{}'s point {p:?} reads {a:?} and {b:?} a hair either side",face.name);
    }
}

/// **A tooth space's sheets are charted by the wheel's slices, on sites halved only where they
/// miss** (`generated::Slices`, `refined`): over the screw's turn a flank bends hardest at the ends
/// of the window, and a grid doubled all over until those ends fit carried 65 × 257 poles a flank
/// and 9 × 257 a crest and its corners, 40 000 in all (108 MB of STEP for the wheel), and halved
/// where it misses 28 × 73 and 9 × 73, 6 000; over the slices of the wheel, a flank cut off where
/// each slice leaves the reach, the five hold within the same quarter of the tolerance on 23 × 45,
/// 8 × 45 and 6 × 45, some 3 000.
#[test]
fn a_tooth_spaces_sheets_are_charted_by_slices_on_sites_halved_where_they_miss() {
    let at = [0.,40.,0.];
    let tol = gcs_core::brep::geom::tolerance(wheel().size());
    let found = gcs_core::brep::swept::generated::candidates(&worm_thread(at,-20.,20.),&in_mesh(at),ROLL,&reach(),tol)
        .unwrap_or_else(|u| panic!("{u:?}")).expect("the worm's thread is the rack's road's");
    let [candidate] = &found.candidates[..] else { panic!("{} candidates for one copy",found.candidates.len()) };
    let poles: Vec<[usize;2]> = candidate.faces.iter().map(|f| match &f.surface {
        gcs_core::brep::geom::Surface::BSpline(_,net) => [net.poles.len(),net.poles[0].len()],
        s => panic!("a sheet no net: {}",s.kind()),
    }).collect();
    let all: usize = poles.iter().map(|p| p[0]*p[1]).sum();
    assert!(all <= 4000,"the tooth space's sheets on {poles:?}, {all} poles");
}

/// **A thread whose ends come within the wheel's reach is no rack's**: the worm cut off at x = ±10,
/// where the wheel reaches 16.6 along its axis, sweeps what its ends do too, which no slid endless
/// thread does; the road leaves it to the others.
#[test]
fn a_thread_whose_ends_reach_the_wheel_is_left_to_the_other_roads() {
    let at = [0.,40.,0.];
    let tol = gcs_core::brep::geom::tolerance(wheel().size());
    let found = gcs_core::brep::swept::generated::candidates(&worm_thread(at,-10.,10.),&in_mesh(at),ROLL,&reach(),tol).unwrap();
    assert!(found.is_none());
    // and a roll that is no whole turn of the wheel: its end poses would bound the sweep
    let found = gcs_core::brep::swept::generated::candidates(&worm_thread(at,-20.,20.),&in_mesh(at),[0.,59.*PI],&reach(),tol).unwrap();
    assert!(found.is_none());
}

/// **The worm wheel hobbed whole by its worm** (brep-bench `swept/worm-wheel-hobbed-by-its-worm`):
/// its thirty tooth spaces, one built and patterned, held to the volume and area cadrs builds,
/// within 1.8e-9 and 3.1e-9 (34429.34922122754 and 10356.511065918847), at a hundred-millionth; 182
/// faces, cadrs's count.
#[test]
fn the_worm_wheel_is_hobbed_whole_by_its_worm() {
    let at = [0.,40.,0.];
    let blank = wheel();
    let tol = gcs_core::brep::geom::tolerance(blank.size());
    let cut = gcs_core::brep::swept::closed::cut_from(&blank,&worm_thread(at,-20.,20.),&in_mesh(at),ROLL,0.01,tol)
        .unwrap_or_else(|u| panic!("{u:?}"));
    cut.check(tol).unwrap();
    assert_eq!(cut.faces.len(),182);
    let v = gcs_core::brep::props::volume(&cut);
    let a: f64 = cut.faces.iter().map(|f| gcs_core::brep::props::area(&cut,f)).sum();
    assert!((v/34429.34922122754-1.).abs() <= 1e-8,"the wheel's volume {v} against cadrs's 34429.34922122754");
    assert!((a/10356.511065918847-1.).abs() <= 1e-8,"its surface {a} against cadrs's 10356.511065918847");
}
