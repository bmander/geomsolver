//! **K2's patches** (#780 L3, `brep::swept::patches`, docs/swept-solid-design.md step 7): a tool
//! carried by a rigid motion no constant twist takes — a frame along a spline (#398) — built closed
//! on one arrangement from the sheet each instant's contact sweeps, fitted, and the tool where it
//! starts and where it stops, their touches given. Held to what is known apart from the kernel: a
//! ball's sweep is the tube about its centre's path, which the canal class fits to it on its own
//! road (#684 D) and whose volume is `π r² L + 4/3 π r³`; a tool slid at a lean is the constant
//! twist's sweep, `V + |D| A`; and a sweep each of whose points the tool enters once and leaves
//! once grows at the rate its faces advance outward, `∫ (n · v)⁺ dA`. Each build agrees with
//! membership and with a Monte Carlo of the sweep's definition, read over the roll directly.
use gcs_core::brep::props::volume;
use gcs_core::brep::swept::closed::{Unbuilt,closed};
use gcs_core::brep::swept::patches::Revolved;
use gcs_core::brep::topo::Brep;
use gcs_core::brep::trajectory::{Screw,Trajectory,Twist};
use gcs_core::model::SolidDef;
use gcs_core::program::Elaborated;
use gcs_core::space::dot;
use std::f64::consts::{PI,TAU};

use crate::swept_closed::{agrees_over,bull_nose,monte_carlo,slide};

type V = [f64;3];

/// The example `name` of the solid sweeps series, elaborated and solved.
pub(crate) fn example(name: &str) -> Elaborated { fixtures::read(&crate::solid_sweeps_examples::text(name)) }

/// The sweep `name` of `e` as the kernel takes it: its tool's B-rep, its trajectory and its roll.
pub(crate) fn sweep(e: &Elaborated,name: &str) -> (Brep,Trajectory,[f64;2]) {
    let sk = &e.sketch;
    let swept = fixtures::solid(e,name);
    let SolidDef::Swept {source,motion,..} = &sk.solids[swept].def else { panic!("`{name}` is a sweep") };
    let path = gcs_core::motion::Family::read(sk,*motion as usize).unwrap().trajectory().unwrap();
    let roll = gcs_core::solid::roll::interval(sk,swept).unwrap();
    let (path,roll) = path.unpaced(roll).unwrap();
    let tool = gcs_core::brep::recipe::build(&gcs_core::solid::cad::recipe(sk,*source as usize).unwrap()).unwrap();
    (tool,path.clone(),roll)
}

/// The sweep of the tool `tool` under `path` over `roll`, read by the definition: whether a point is
/// in it, the tool's poses over `n` rolls each carrying it back onto the tool (`inside_of`) — a pose
/// carrying it farther than the tool reaches from its centre missing it at once.
fn defined<'a>(tool: &'a Revolved,path: &Trajectory,[a,b]: [f64;2],n: usize) -> impl Fn(V) -> bool + 'a {
    let back: Vec<gcs_core::brep::trajectory::Pose> = (0..=n).map(|i| path.pose(a+(b-a)*i as f64/n as f64).unwrap().inverse()).collect();
    let inside = inside_of(tool);
    move |p: V| back.iter().any(|m| {
        let q = m.point(p);
        gcs_core::space::norm(gcs_core::space::sub(q,tool.origin())) <= tool.size && inside(q)
    })
}

/// The inside of a solid of revolution read off its meridian (`Revolved`): a point's `(ρ, z)` about
/// its axis within the meridian's region, by the ray from its centre — where it leaves the tool is
/// beyond the point.
fn inside_of(tool: &Revolved) -> impl Fn(V) -> bool + '_ {
    move |p: V| {
        let d = gcs_core::space::sub(p,tool.origin());
        let r = gcs_core::space::norm(d);
        if r == 0. { return true }
        let (q,..) = tool.cast(d.map(|x| x/r));
        gcs_core::space::norm(gcs_core::space::sub(q,tool.origin())) > r
    }
}

/// **A ball along a spline frame is its fitted tube** (#780 L3's first gate): `path.sv`'s ball,
/// radius 3 riding `std.stations.Along` over its whole spline, built closed from the sheet its
/// contact — the great circle square to its centre's velocity, read round the pencil — sweeps, and
/// the ball where it starts and stops, touching it round those circles (given). It is held to the
/// tube the canal class fits on its own road (the disc's face family, `swept::canal::tube`), built
/// apart, to a millionth; to the tube's volume `π r² L + 4/3 π r³`, `L` the spline's length by
/// quadrature; to a Monte Carlo of the definition (the ball's centre's distance read over the
/// roll); and to membership.
#[test]
fn a_ball_along_a_spline_frame_is_its_fitted_tube() {
    let e = example("path.sv");
    let (tool,path,roll) = sweep(&e,"groove");
    let r = 3.;
    let built = closed(&tool,&path,roll,0.002,1e-6).unwrap_or_else(|u| panic!("{u:?}"));
    // the canal's tube, apart
    let centre = [0.;3];
    let tube = gcs_core::brep::swept::canal::tube(&path,centre,r,"wall",roll).unwrap().unwrap();
    assert!(matches!(tube.spine,gcs_core::brep::recipe::Spine::Tube(_)),"fitted to its path");
    let canal = gcs_core::brep::recipe::Node::Canal {centre:tube.centre,radius:tube.radius,spine:tube.spine,face:"wall".into()}
        .built(&Default::default(),&Default::default(),1e-6).unwrap();
    let (got,fitted) = (volume(&built),volume(&canal));
    assert!((got-fitted).abs() <= 1e-6*fitted,"{got} against the fitted tube's {fitted}");
    let length = crate::ball_paths::bezier_length([[0.,0.],[10.,0.],[20.,20.],[30.,20.]]);
    let closed_form = crate::canal::tube(r,length);
    assert!((got-closed_form).abs() <= 1e-6*closed_form,"{got} against π r² L + 4/3 π r³ = {closed_form}");
    // the definition: within the radius of the centre at some roll
    let centres: Vec<V> = (0..=4000).map(|i| path.pose(roll[0]+(roll[1]-roll[0])*i as f64/4000.).unwrap().point(centre)).collect();
    let (lo,hi) = built.bounds();
    let (mc,sigma) = monte_carlo(lo,hi,40_000,7,&|p| centres.iter().any(|&c| gcs_core::space::norm(gcs_core::space::sub(p,c)) <= r));
    assert!((mc-got).abs() <= 4.*sigma+1e-3*got,"a Monte Carlo of the definition {mc} ± {sigma} against {got}");
    agrees_over(&built,&tool,&path,roll);
}

/// `path` read as a motion no constant twist takes: seen from a frame standing still (`Relative`),
/// so the kernel reads it instant by instant.
fn as_any(path: Trajectory) -> Trajectory {
    let still = Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:0.,advance:0.};
    Trajectory::Relative {source:Box::new(path),observer:Box::new(Trajectory::Twist(still))}
}

/// **A bull-nose slid at a lean, read instant by instant, is the constant twist's sweep**: the mill
/// of radius 1, corner 0.3 and height 1 slid 3 down a line leaning 37° off its axis, its motion
/// seen from a frame standing still so no constant twist is read off it. Its contact runs down its
/// side, round its round and up again, and over its top rim's fan; each instant's is the same on
/// the tool, and the sheet the cylinder of the mill's outline along the slide. The build agrees
/// with #810's — the band's contour, its rulings' planes and its rims' oblique cylinders, in closed
/// form — to a part in a hundred million, and with the volume its faces advance through
/// (`advanced`; under a slide `V + |D| A`, `A` the shadow its faces' rate sums) to a part in a
/// million.
#[test]
fn a_bull_nose_slid_at_a_lean_instant_by_instant_is_the_slides_sweep() {
    let tool = bull_nose(0.,1.,0.3,1.);
    let d = [1.8,0.,-2.4];
    let built = closed(&tool,&as_any(slide(d)),[0.,1.],0.002,1e-6).unwrap_or_else(|u| panic!("{u:?}"));
    let twist = closed(&tool,&slide(d),[0.,1.],0.002,1e-6).unwrap();
    let (got,want) = (volume(&built),volume(&twist));
    assert!((got-want).abs() <= 1e-8*want,"{got} against the slide's {want}");
    let worked = advanced(&Revolved::read(&tool).unwrap(),&tool,&as_any(slide(d)),[0.,1.],false);
    assert!((got-worked).abs() <= 1e-6*worked,"{got} against the volume worked apart {worked}");
    agrees_over(&built,&tool,&as_any(slide(d)),[0.,1.]);
}

/// **A bull-nose screwed, read instant by instant**: the mill beside the axis, 4 out, screwed a
/// radian at 2 a turn — under a screw the constant-twist road refuses a torus's face (`Curved`),
/// and K2's builds it. Its contact runs down the side along the line in the plane through both
/// axes, round the round hugging its flat end, and over the top rim's fan. A helix meets the mill
/// once, and a radian is short of a return, so each point is entered once and left once: held to
/// the mill's own volume and `∫ ∫ (n · v)⁺ dA dt` (`advanced`) to a part in a million (its sheet
/// fitted within a tenth of the arrangement's tolerance), that held to a Monte Carlo of the
/// definition, and the build to membership.
#[test]
fn a_bull_nose_screwed_instant_by_instant_closes_into_its_sweep() {
    let (d,theta,advance) = (4.,1.,2.);
    let tool = bull_nose(d,1.,0.3,1.);
    let screw = Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:theta,advance};
    let path = as_any(Trajectory::Twist(screw));
    assert!(matches!(closed(&tool,&Trajectory::Twist(screw),[0.,1.],0.002,1e-6),Err(Unbuilt::Curved {..})),
        "the constant twist's road refuses a torus screwed");
    let built = closed(&tool,&path,[0.,1.],0.002,1e-6).unwrap_or_else(|u| panic!("{u:?}"));
    let revolved = Revolved::read(&tool).unwrap();
    let worked = advanced(&revolved,&tool,&path,[0.,1.],false);
    let (lo,hi) = built.bounds();
    let (mc,sigma) = monte_carlo(lo,hi,40_000,5,&defined(&revolved,&path,[0.,1.],600));
    assert!((mc-worked).abs() <= 4.*sigma,"a Monte Carlo of the definition {mc} ± {sigma} against the volume worked apart {worked}");
    let got = volume(&built);
    assert!((got-worked).abs() <= 1e-6*worked,"{got} against the volume worked apart {worked}");
    agrees_over(&built,&tool,&path,[0.,1.]);
}

/// Where `side` changes over `[x0, x1]`, read at `n` even steps and each change bisected to
/// rounding.
fn sign_cuts(n: usize,[x0,x1]: [f64;2],side: impl Fn(f64) -> bool) -> Vec<f64> {
    (0..n).filter_map(|i| {
        let (a,b) = (x0+(x1-x0)*i as f64/n as f64,x0+(x1-x0)*(i+1) as f64/n as f64);
        (side(a) != side(b)).then(|| {
            let (lo,hi) = gcs_core::roots::bisect(a,b,|l,h| 0.5*(l+h),|l,h| { let m = 0.5*(l+h); m > l && m < h },
                |m| side(m) == side(a));
            0.5*(lo+hi)
        })
    }).collect()
}

/// The integral over a convex tool of revolution's faces of `f(x, n).0` (`x` the point of the tool,
/// `n` its outward normal there, both in the tool's coordinates), smooth but where either of
/// `f(x, n).1` — each, round a ring, `α + β cos w + γ sin w` — changes sign. Each face is read ring
/// by ring: a ring's angle cut at those signs' changes (found by sampling, settled by bisection),
/// each stretch by Gauss–Legendre. Along the meridian, a ring's integral turns as a square root
/// where a sign first reaches the ring (`α² = β² + γ²`), so each piece of the meridian is cut there
/// too, and each stretch read in `s = (1 − cos πτ)/2`, which takes the root out, by Gauss–Legendre
/// in `τ`.
fn over_faces(tool: &Revolved,f: &dyn Fn(V,V) -> (f64,[f64;2])) -> f64 {
    use gcs_core::brep::swept::patches::Element;
    let rule = crate::common::rounds::gauss_legendre(8);
    let g = tool.frame;
    let at = |rho: f64,z: f64,nr: f64,nz: f64,w: f64| f(g.at([rho*w.cos(),rho*w.sin(),z]),g.dir([nr*w.cos(),nr*w.sin(),nz]));
    let ring = |rho: f64,z: f64,nr: f64,nz: f64| -> f64 {
        let h = |w: f64| at(rho,z,nr,nz,w);
        let mut cuts: Vec<f64> = (0..2).flat_map(|j| sign_cuts(64,[0.,TAU],|w| h(w).1[j] > 0.)).chain([0.,TAU]).collect();
        cuts.sort_by(f64::total_cmp);
        cuts.windows(2).map(|s| {
            rule.iter().map(|&(x,wt)| { let w = 0.5*(s[0]+s[1])+0.5*(s[1]-s[0])*x; wt*h(w).0 }).sum::<f64>()*0.5*(s[1]-s[0])
        }).sum::<f64>()*rho
    };
    // a piece of the meridian at `s` of it: its ring and the length of the piece
    let along = |e: &Element,s: f64| -> (f64,f64,f64,f64,f64) {
        match *e {
            Element::Line {a,b} => { let (dr,dz) = (b[0]-a[0],b[1]-a[1]); let l = dr.hypot(dz); (a[0]+s*dr,a[1]+s*dz,dz/l,-dr/l,l) }
            Element::Arc {c,r,from,sweep} => { let w = from+s*sweep; (c[0]+r*w.cos(),c[1]+r*w.sin(),w.cos(),w.sin(),r*sweep.abs()) }
        }
    };
    // where a sign first reaches a ring: α² − β² − γ² through nought
    let reach = |e: &Element,s: f64,j: usize| -> f64 {
        let (rho,z,nr,nz,_) = along(e,s);
        let q = |w: f64| at(rho,z,nr,nz,w).1[j];
        let (q0,q1,q2,q3) = (q(0.),q(PI/2.),q(PI),q(3.*PI/2.));
        let (alpha,beta,gamma) = (0.5*(q0+q2),0.5*(q0-q2),0.5*(q1-q3));
        alpha*alpha-beta*beta-gamma*gamma
    };
    tool.elements.iter().map(|e| {
        let mut cuts: Vec<f64> = (0..2).flat_map(|j| sign_cuts(48,[0.,1.],|s| reach(e,s,j) > 0.)).chain([0.,1.]).collect();
        cuts.sort_by(f64::total_cmp);
        cuts.windows(2).map(|c| {
            let (s0,s1) = (c[0],c[1]);
            let pieces = 8;
            (0..pieces).map(|p| rule.iter().map(|&(x,wt)| {
                let tau = (p as f64+0.5*(1.+x))/pieces as f64;
                let s = s0+(s1-s0)*0.5*(1.-(PI*tau).cos());
                let ds = (s1-s0)*0.5*PI*(PI*tau).sin()/pieces as f64;
                let (rho,z,nr,nz,l) = along(e,s);
                wt*0.5*ring(rho,z,nr,nz)*l*ds
            }).sum::<f64>()).sum::<f64>()
        }).sum::<f64>()
    }).sum()
}

/// **What a convex tool of revolution sweeps riding `path` over `roll`, where it enters each point
/// once and leaves it once**, worked apart from the kernel: what it holds where it starts, and over
/// the roll the rate at which its faces advance outward, `∫ ∫ (n · v)⁺ dA dt` — both within the
/// half-space below `z = 0` where `below` (a block's top), what it holds there `∫ z n_z dA` over
/// its faces below the top (the divergence of `(0, 0, z)`, nothing on the top itself). The roll by
/// Gauss–Legendre over 48 panels.
pub(crate) fn advanced(tool: &Revolved,brep: &Brep,path: &Trajectory,[a,b]: [f64;2],below: bool) -> f64 {
    let rule = crate::common::rounds::gauss_legendre(8);
    let start = path.pose(a).unwrap();
    let held = if below {
        over_faces(tool,&|x,n| { let (p,m) = (start.point(x),start.vector(n)); (if p[2] < 0. { p[2]*m[2] } else { 0. },[p[2],1.]) })
    } else { volume(brep) };
    let rate = |t: f64| -> f64 {
        let (tw,m) = (Twist::body(path.at(t).unwrap()),path.pose(t).unwrap());
        over_faces(tool,&|x,n| {
            let (rate,z) = (dot(n,tw.velocity(x)),if below { m.point(x)[2] } else { -1. });
            (if z < 0. { rate.max(0.) } else { 0. },[rate,z])
        })
    };
    let panels = 48;
    held+(0..panels).map(|i| {
        let (t0,t1) = (a+(b-a)*i as f64/panels as f64,a+(b-a)*(i+1) as f64/panels as f64);
        rule.iter().map(|&(x,wt)| wt*rate(0.5*(t0+t1)+0.5*(t1-t0)*x)).sum::<f64>()*0.5*(t1-t0)
    }).sum::<f64>()
}

/// **A bull-nose along a spline frame** (#780 L3's second gate, `solid_sweeps/spline_mill.sv`'s
/// mill): leaning 30° toward where it goes, riding `std.stations.Along` over `path.sv`'s spline. No
/// constant twist carries it, so its contact moves on it; the kernel builds its sweep closed on one
/// arrangement. Held to the volume worked apart (`advanced`: the mill's own and `∫ ∫ (n · v)⁺ dA
/// dt`) to a part in a million (its sheet fitted within a tenth of the arrangement's tolerance),
/// that held first to a Monte Carlo of the definition, and the build to membership.
#[test]
fn a_bull_nose_along_a_spline_frame_closes_into_the_volume_its_faces_advance_through() {
    let e = example("spline_mill.sv");
    let (tool,path,roll) = sweep(&e,"groove");
    let revolved = Revolved::read(&tool).unwrap();
    let built = closed(&tool,&path,roll,0.002,1e-6).unwrap_or_else(|u| panic!("{u:?}"));
    let worked = advanced(&revolved,&tool,&path,roll,false);
    let (lo,hi) = built.bounds();
    let (mc,sigma) = monte_carlo(lo,hi,40_000,3,&defined(&revolved,&path,roll,600));
    assert!((mc-worked).abs() <= 4.*sigma,"a Monte Carlo of the definition {mc} ± {sigma} against the volume worked apart {worked}");
    let got = volume(&built);
    assert!((got-worked).abs() <= 1e-6*worked,"{got} against the volume worked apart {worked}");
    agrees_over(&built,&tool,&path,roll);
}

/// **What K2's reading refuses, by name**: a tool no convex solid of revolution (a ring) under a
/// frame along a spline is `Curved` at a point of the face it cannot read, before anything is
/// built. A box under that frame was refused so too until #880 Q2: the frame turns about the
/// spline's plane's normal, so it keeps that plane, and the box standing along it is asked its
/// section's planar sweep (`swept::planar_solid`) — which refuses it, never building it wrong: the
/// frame's instant centre stands on the line through the box's corner square to its long sides, so
/// each side's contact is that corner at every instant (read with the rounding of the side's
/// parameter, a walk turning about: `Folded`), and through the spline's inflection the side is in
/// contact all along for an instant. Left open (#880).
#[test]
fn a_tool_no_convex_revolution_under_any_motion_is_refused_by_name() {
    let e = example("path.sv");
    let (_,path,roll) = sweep(&e,"groove");
    let tool = crate::brep::block([0.;3],[1.,2.,3.]);
    let boxed = closed(&tool,&path,roll,0.002,1e-6);
    assert!(matches!(boxed,Err(Unbuilt::Folded {..})),"{boxed:?}");
    let ring = gcs_core::brep::build::torus([0.;3],[0.,0.,1.],2.,0.5).unwrap();
    assert!(matches!(closed(&ring,&path,roll,0.002,1e-6),Err(Unbuilt::Curved {..})),"a ring is no convex solid");
}

/// **A sheet that folds or comes back onto itself under any motion is refused by name**, never left
/// to the judge: a ball turned beside the axis nearer it than its radius (its great circle sweeping
/// a spindle torus, its sheet still where the circle crosses the axis) is `Folded`; a ball screwed
/// a turn and a half at a lead short of its diameter (its tube coming back within reach of itself)
/// is `Returns`, at a point between its centre's passes.
#[test]
fn a_sheet_folding_or_coming_back_under_any_motion_is_refused_by_name() {
    let ball = |c: V,r: f64| gcs_core::brep::build::sphere(c,[0.,0.,1.],r).unwrap();
    let folded = closed(&ball([0.4,0.,0.],0.5),&as_any(Trajectory::Twist(Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:1.,advance:0.})),[0.,1.],0.002,1e-6);
    assert!(matches!(folded,Err(Unbuilt::Folded {..})),"{folded:?}");
    let back = closed(&ball([2.,0.,0.],0.5),&as_any(Trajectory::Twist(Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:1.,advance:0.6})),[0.,1.5*TAU],0.002,1e-6);
    let Err(Unbuilt::Returns {at}) = back else { panic!("{back:?}") };
    assert!((at[0].hypot(at[1])-2.).abs() <= 0.1,"between the centre's passes: {at:?}");
}
