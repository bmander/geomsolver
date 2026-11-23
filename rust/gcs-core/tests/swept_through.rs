//! **A slide through a blank, by its shadow** (`swept::through`): a tool slid without turning from
//! wholly before a blank to wholly past it cuts from it the prism over its shadow along the slide,
//! built in the plane square to the slide — its walls extrusions of the tool's contact seen along
//! the slide — and nothing else.
use gcs_core::brep::build::{canal,ellipsoid,prism,square};
use gcs_core::brep::geom::Surface;
use gcs_core::brep::props::volume;
use gcs_core::brep::swept::closed::cut_from;
use gcs_core::brep::topo::Brep;
use gcs_core::brep::trajectory::{Screw,Trajectory};
use std::f64::consts::{PI,TAU};

/// The slide along `push` from where the tool is drawn, its whole length over a roll of a turn.
fn slide(from: [f64;3],push: [f64;3]) -> Trajectory {
    Trajectory::Twist(Screw {origin:from,axis:gcs_core::space::unit(push),ratio:0.,advance:gcs_core::space::norm(push)})
}

/// The cut's faces are its blank's planes and the shadow's walls, extrusions along the slide.
fn walled_by_extrusions(cut: &Brep) -> bool {
    cut.faces.iter().all(|f| matches!(f.surface,Surface::Plane(..) | Surface::Extrusion(..)))
        && cut.faces.iter().any(|f| matches!(f.surface,Surface::Extrusion(..)))
}

/// **A ball's tube has its half balls sewn on** (`build::canal`): each half ball's flat face is the
/// tube's end disc, so the three are sewn where the discs were rather than united by the Boolean.
/// The coil spring of `coil-spring-pushed-aslant-through-plate` — the balls of radius 0.4 about
/// three turns of the helix of radius 2 rising 1.5 a turn — holds a tube's measures (Weyl: its
/// volume `π r² L`, its area `2π r L`, `L` the helix's length) and a ball's at its two ends.
#[test]
fn a_canals_half_balls_are_sewn_onto_its_tube() {
    let (r,turns) = (0.4,3.);
    let spring = canal([2.,0.,-12.],r,[0.;3],[0.,0.,1.],turns*TAU,turns*1.5,"spring").unwrap();
    spring.check(1e-8).unwrap();
    let length = turns*(4.*PI).hypot(1.5);
    let v = PI*r*r*length+4./3.*PI*r*r*r;
    let a = TAU*r*length+4.*PI*r*r;
    let (got_v,got_a) = (volume(&spring),crate::brep::total_area(&spring));
    assert!((got_v-v).abs() <= 1e-9*v,"the spring's volume {got_v}, a tube's and a ball's {v}");
    assert!((got_a-a).abs() <= 1e-9*a,"the spring's area {got_a}, a tube's and a ball's {a}");
    // its faces the tube's wall and the two half balls, no disc left between
    assert_eq!(spring.faces.len(),3);
    assert!(spring.faces.iter().all(|f| !matches!(f.surface,Surface::Plane(..))));
}

/// **A coil spring pushed aslant through a plate is cut by its shadow** (brep-bench's
/// `swept/coil-spring-pushed-aslant-through-plate`, `swept_contacts::coil_spring_case`): the closed
/// class refuses the spring's coil (`Curved`), and the slide carries it from wholly below the plate
/// to wholly above, so the plate loses the prism over its shadow — the union of its balls' shadows,
/// whose loops overlap one another — at the kernel's tolerance, to the volume worked apart from the
/// build (3541.9379700, to 5e-7): the plate, two solids (the shadow is a ring, its core left
/// standing), walled by extrusions; and to a STEP file that reads back as the solid.
#[test]
fn a_coil_spring_pushed_through_a_plate_is_cut_by_its_shadow() {
    let case = crate::swept_contacts::coil_spring_case();
    let tol = case.tolerance();
    let cut = cut_from(case.blank.as_ref().unwrap(),&case.tool,&case.path,case.roll,case.bar,tol).unwrap();
    let v = volume(&cut);
    assert!((v-case.volume).abs() <= case.known,"the cut's volume {v}, apart from the build {}",case.volume);
    assert!(walled_by_extrusions(&cut),"the cut's faces: {:?}",cut.faces.iter().map(|f| f.surface.kind()).collect::<Vec<_>>());
    assert_eq!(gcs_core::brep::sew::parts(&cut).len(),2,"the plate and the core of the spring's shadow");
    let text = gcs_core::brep::step::write(&cut,"swept",tol,gcs_core::brep::step::Fit::Exact).unwrap();
    gcs_core::brep::step_check::verify(&text,&gcs_core::brep::step_check::Solid::of(&cut)).unwrap();
}

/// **A spheroid pushed aslant through a slab is cut by its shadow**: the ellipsoid of semi-axes 3,
/// 2 and 1 along x, y and z about (0, 0, −10) slid along (2, 1, 25) through the slab `|x|, |y| ≤ 10`,
/// `0 ≤ z ≤ 2`. Its contact under the slide is one closed curve crossing nothing, and the hole is
/// the slab's thickness times the area of its shadow on a level plane: the ellipsoid `xᵀAx ≤ 1`
/// seen square to the unit `d` has the area `π abc √(dᵀAd)` (a ball's `π r²`), and seen along `d`
/// on a level plane that over `|d_z|`.
#[test]
fn a_spheroid_pushed_aslant_through_a_slab_is_cut_by_its_shadow() {
    let ([a,b,c],push) = ([3.,2.,1.],[2.,1.,25.]);
    let spheroid = ellipsoid([0.,0.,-10.],[0.,0.,1.],[1.,0.,0.],[a,b,c]).unwrap();
    let slab = prism(&square([0.;3],[0.,0.,1.],10.),0.,2.).unwrap();
    let tol = gcs_core::brep::geom::tolerance(slab.size().max(spheroid.size()));
    let cut = cut_from(&slab,&spheroid,&slide([0.,0.,-10.],push),[0.,TAU],0.002,tol).unwrap();
    let d = gcs_core::space::unit(push);
    let seen = PI*a*b*c*(d[0]*d[0]/(a*a)+d[1]*d[1]/(b*b)+d[2]*d[2]/(c*c)).sqrt();
    let expected = 800.-2.*seen/d[2];
    let v = volume(&cut);
    assert!((v-expected).abs() <= 1e-6*expected,"the cut's volume {v}, its shadow's {expected}");
    assert!(walled_by_extrusions(&cut),"the cut's faces: {:?}",cut.faces.iter().map(|f| f.surface.kind()).collect::<Vec<_>>());
}

/// **A slide that stops inside its blank is left to the other roads** (`through::cut` is `None`):
/// the ring of `ring-pushed-in-past-its-fold` (`swept_contacts::ring_fold_case`) pushed down 8 into
/// a block 8 deep ends inside it, and its sweep there is no prism over its shadow.
#[test]
fn a_slide_stopping_in_its_blank_is_not_cut_by_its_shadow() {
    let case = crate::swept_contacts::ring_fold_case();
    let blank = case.blank.as_ref().unwrap();
    assert!(gcs_core::brep::swept::through::cut(blank,&case.tool,&case.path,case.roll,case.bar,case.tolerance()).is_none());
}
