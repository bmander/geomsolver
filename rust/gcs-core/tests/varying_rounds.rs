//! **A ball whose radius varies along the edge** (`brep::fillet::roll_varying`): at each point of
//! the edge, the ball of the radius there touching both faces with its centre in the plane square
//! to the edge, its arc between the contacts the round's section. Each round is held to its closed
//! forms, the area of its curved face integrated here from the section alone.
use crate::brep::{block,total_area};
use crate::common::rounds::{composite,gauss_legendre};
use gcs_core::brep::boolean::{combined,Op};
use gcs_core::brep::build::{disc,kept,prism};
use gcs_core::brep::fillet::roll_varying;
use gcs_core::brep::geom::tolerance;
use gcs_core::brep::props::volume;
use gcs_core::brep::topo::Brep;
use std::f64::consts::PI;

fn close(what: &str,got: f64,want: f64,within: f64) {
    assert!((got-want).abs() <= within*want.abs(),"{what}: {got} against {want} ({:e})",(got-want)/want);
}

fn rounded(b: &Brep,at: [f64;3],radius: &dyn Fn([f64;3]) -> f64) -> Brep {
    let round = roll_varying(b,at,radius,tolerance(b.size())).unwrap();
    let out = combined(b,&round.piece,if round.concave { Op::Union } else { Op::Cut },0.).unwrap();
    out.check(1e-7).unwrap();
    out
}

/// **A cube's edge rounded with a radius growing along it** (brep-bench's
/// fillet/variable-radius-edge): `r = 1 + z/10` between the faces x = 10 and y = 10, each section
/// at height z the quarter disc's corner `(1 − π/4) r²` taken off — `1000 − (70/3)(1 − π/4)` — and
/// the round's own area `∫ r dz · ∫ √(1 + r′²(1 − sin θ − cos θ)²) dθ`, its sections' arcs leaning
/// as the radius grows.
#[test]
fn a_cubes_edge_rounded_by_a_growing_radius_is_its_sections() {
    let cube = block([0.;3],[10.;3]);
    let out = rounded(&cube,[10.,10.,5.],&|p| 1.+p[2]/10.);
    let corner = 1.-PI/4.;
    close("the volume",volume(&out),1000.-70./3.*corner,1e-9);
    let rule = gauss_legendre(20);
    let lean = composite(&rule,0.,PI/2.,4,&|t: f64| (1.+(1.-t.sin()-t.cos()).powi(2)/100.).sqrt());
    close("the area",total_area(&out),570.-5.*corner+15.*lean,1e-9);
}

/// **A half cylinder's rim rounded with a radius growing round it** (brep-bench's
/// fillet/variable-radius-semicircular-rim): `r = 3/2 + φ/π` round the half circle where the side of
/// radius 5 meets the top at z = 6, the run cut off square where it meets the flat face x = 0. Each
/// section the quarter circle of radius `r` about the point `5 − r` from the axis and `r` under the
/// top: the volume by Pappus, section by section, and the round's area integrated from its
/// parameterisation.
#[test]
fn a_half_cylinders_rim_rounded_by_a_radius_growing_round_it_is_its_sections() {
    let half = kept(&prism(&disc([0.;3],[0.,0.,1.],5.),0.,6.).unwrap(),[0.;3],[1.,0.,0.],0.).unwrap();
    let out = rounded(&half,[5.*0.3f64.cos(),5.*0.3f64.sin(),6.],&|[x,y,_]| 1.5+y.atan2(x)/PI);
    let r = |phi: f64| 1.5+phi/PI;
    let corner = 1.-PI/4.;
    // each section's corner, `(1 − π/4) r²`, turned about the axis at its centroid's distance
    let moment = 5./6.-PI/4.;
    close("the volume",volume(&out),75.*PI-35.*PI/3.*corner+15.*PI/4.*moment,1e-9);
    let rule = gauss_legendre(20);
    let round = composite(&rule,-PI/2.,PI/2.,8,&|phi| composite(&rule,0.,PI/2.,4,&|th| {
        let (rr,dr) = (r(phi),1./PI);
        let rho = 5.-rr+rr*th.cos();
        // S(φ, θ) = (ρ cos φ, ρ sin φ, 6 − r + r sin θ)
        let drho = -dr+dr*th.cos();
        let sp = [drho*phi.cos()-rho*phi.sin(),drho*phi.sin()+rho*phi.cos(),-dr+dr*th.sin()];
        let st = [-rr*th.sin()*phi.cos(),-rr*th.sin()*phi.sin(),rr*th.cos()];
        let n = [sp[1]*st[2]-sp[2]*st[1],sp[2]*st[0]-sp[0]*st[2],sp[0]*st[1]-sp[1]*st[0]];
        (n[0]*n[0]+n[1]*n[1]+n[2]*n[2]).sqrt()
    }));
    let top = 0.5*composite(&rule,-PI/2.,PI/2.,8,&|phi| (5.-r(phi)).powi(2));
    let side = composite(&rule,-PI/2.,PI/2.,8,&|phi| 5.*(6.-r(phi)));
    close("the area",total_area(&out),60.-5.*corner+top+side+25.*PI/2.+round,1e-9);
}

/// **A radius of no size, and a run that closes**, are refused by name: no ball rolls where its
/// radius is not positive, and a closed rim has no end for the radius to start from.
#[test]
fn a_radius_of_no_size_and_a_closed_rim_are_refused() {
    let cube = block([0.;3],[10.;3]);
    let e = roll_varying(&cube,[10.,10.,5.],&|p| p[2]/10.,tolerance(cube.size())).unwrap_err();
    assert!(e.contains("radius of no size"),"{e}");
    let rod = prism(&disc([0.;3],[0.,0.,1.],5.),0.,6.).unwrap();
    let e = roll_varying(&rod,[0.,5.,6.],&|_| 1.,tolerance(rod.size())).unwrap_err();
    assert!(e.contains("closes on itself"),"{e}");
}
