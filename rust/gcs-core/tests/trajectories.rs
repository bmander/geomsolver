//! **The trajectory a sweep is carried along, the kernel's** (#721 F2): a rigid motion as
//! `brep::trajectory::Trajectory` keeps its structure — a constant twist exactly, one motion seen
//! from another, a path taken at a pace, a path of pieces, and any other motion evaluated — and the
//! solver's `motion::Family` lowers to it (`Family::trajectory`). Held here: every motion of the
//! sweeps', the generation's, the mechanisms' and the paths' examples lowers to a trajectory whose
//! pose and rate are the family's at every time sampled, and whose constant twist is the one the
//! family reads where it reads one; each kind of the structure is met; a screw's twist is its pose's
//! at every time, in the world and on what it carries; a paced path sweeps its path over its pace's
//! range; and a path of pieces is refused where its pieces do not meet.
use gcs_core::brep::trajectory::{Motion,Pace,Path,Piece,Screw,Trajectory,Twist};
use gcs_core::brep::KernelError;
use gcs_core::motion::Family;
use std::sync::Arc;

type V = [f64;3];

/// The points a pose is probed at: about the drawings' sizes, off every axis.
const PROBES: [V;3] = [[0.;3],[31.,-17.,8.],[-5.,44.,-23.]];

fn agree(a: V,b: V,what: &str) {
    let size = 1.+b.iter().map(|x| x.abs()).fold(0.,f64::max);
    for k in 0..3 { assert!((a[k]-b[k]).abs() <= 1e-9*size,"{what}: {a:?} against {b:?}") }
}

fn same_motion(a: Motion,b: Motion,what: &str) {
    for p in PROBES {
        agree(a.point(p),b.point(p),&format!("{what}, a point"));
        agree(a.vector(p),b.vector(p),&format!("{what}, a direction"));
        agree(a.velocity(p),b.velocity(p),&format!("{what}, a velocity"));
    }
}

/// Each kind of the trajectory's structure counted in `met`, as they nest.
fn kinds(t: &Trajectory,met: &mut std::collections::BTreeMap<&'static str,usize>) {
    let mut count = |k| *met.entry(k).or_default() += 1;
    match t {
        Trajectory::Twist(_) => count("twist"),
        Trajectory::Relative {source,observer} => { count("relative"); kinds(source,met); kinds(observer,met) }
        Trajectory::Paced {path,..} => { count("paced"); kinds(path,met) }
        Trajectory::Pieces(p) => { count("pieces"); for p in p.pieces() { kinds(&p.path,met) } }
        Trajectory::General(_) => count("general"),
        Trajectory::Placed {..} => count("placed"),
    }
}

#[test]
fn a_motion_lowers_to_the_kernels_trajectory() {
    let mut met = std::collections::BTreeMap::<&str,usize>::new();
    let (mut read,mut twists) = (0,0);
    for name in fixtures::example_names() {
        if !["solid_sweeps/","generation/","mechanisms/","paths/"].iter().any(|d| name.starts_with(d)) { continue }
        let Some(e) = fixtures::example(&name) else { continue };
        // a frame along a path is read where the solve draws it on its path
        let mut sk = e.sketch.clone();
        if (0..sk.motions.len()).any(|i| Family::read(&sk,i).is_err()) {
            gcs_core::solve::solve(&mut sk,gcs_core::solve::SolveOpts::default());
        }
        for i in 0..sk.motions.len() {
            let Ok(family) = Family::read(&sk,i) else { continue };
            let lowered = family.trajectory().unwrap_or_else(|m| panic!("{name}: `{}` does not lower: {m}",family.name));
            kinds(&lowered,&mut met);
            assert_eq!(lowered.screw(),family.twist(),"{name}: `{}`'s constant twist",family.name);
            for t in [-2.1,-0.4,0.,0.3,1.,2.7,5.9] {
                let (Ok(want),got) = (family.at(t),lowered.at(t)) else { continue };
                let got = got.unwrap_or_else(|m| panic!("{name}: `{}` at {t}: {m}",family.name));
                same_motion(got,want,&format!("{name}: `{}` at {t}",family.name));
                // a constant twist's pose is `pose_at`'s to the bit (`Screw::at` reads its unit axis
                // as it is): the walk's witnesses and reaches stood where they stood before the move
                if lowered.screw().is_some() {
                    let pose = family.pose_at(t).unwrap();
                    for p in PROBES { assert_eq!(got.point(p),pose.point(p),"{name}: `{}` at {t}, a point by `pose_at`",family.name) }
                    twists += 1;
                }
                read += 1;
            }
        }
    }
    eprintln!("{read} poses; kinds met {met:?}");
    for kind in ["twist","relative","paced","general"] { assert!(met.contains_key(kind),"no {kind} met: {met:?}") }
    assert!(read > 200 && twists > 50,"{read} poses, {twists} of a constant twist");
}

#[test]
fn a_screws_twist_is_its_poses_at_every_time() {
    let screws = [
        Screw {origin:[3.,-2.,7.],axis:[0.,0.6,0.8],ratio:1.5,advance:12.},
        Screw {origin:[0.,5.,0.],axis:[1.,0.,0.],ratio:-0.7,advance:0.},
        Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:0.,advance:-30.},
    ];
    for s in screws {
        let (path,w) = (Trajectory::Twist(s),s.twist());
        assert_eq!(path.screw(),Some(s));
        for t in [-3.,0.,0.8,4.4] {
            let m = path.at(t).unwrap();
            for (name,read) in [("in the world",Twist::spatial(m)),("on what it carries",Twist::body(m))] {
                agree(read.omega,w.omega,&format!("{} {name} at {t}",s.kind()));
                agree(read.v0,w.v0,&format!("{} {name} at {t}",s.kind()));
            }
            for p in PROBES {
                // the screw's own velocity per radian, its pose's carriage, and its twist's agree
                agree(w.velocity(p),s.velocity(p),&format!("{} at {p:?}",s.kind()));
                agree(m.point(p),s.carry(p,t),&format!("{} carrying {p:?} for {t}",s.kind()));
            }
        }
        if let Some((point,direction,pitch)) = w.axis() {
            agree(direction,s.axis.map(|a| a*s.ratio.signum()),"the twist's axis, the way it turns");
            agree(w.velocity(point),w.omega.map(|w| w*pitch),"along the axis by its pitch a radian");
        } else { assert!(s.is_slide()) }
    }
}

/// A pace that rises and comes back over `[0, 2]`: `3 t (2 − t)`, its greatest 3 at 1.
#[derive(Debug)]
pub(crate) struct Arch;

impl Pace for Arch {
    fn jet(&self,t: f64) -> Result<[f64;2],KernelError> { Ok([3.*t*(2.-t),6.-6.*t]) }
    fn range(&self,[a,b]: [f64;2]) -> Result<[f64;2],KernelError> {
        let f = |t: f64| 3.*t*(2.-t);
        let top = if a <= 1. && 1. <= b { 3. } else { f(a).max(f(b)) };
        Ok([f(a).min(f(b)),top])
    }
}

#[test]
fn a_paced_path_sweeps_its_path() {
    let s = Screw {origin:[1.,0.,0.],axis:[0.,0.,1.],ratio:1.,advance:4.};
    let paced = Trajectory::Paced {path:Box::new(Trajectory::Twist(s)),pace:Arc::new(Arch)};
    assert_eq!(paced.screw(),None);
    for t in [0.,0.5,1.,1.7] {
        let [at,rate] = Arch.jet(t).unwrap();
        let (m,base) = (paced.at(t).unwrap(),s.at(at).unwrap());
        for p in PROBES {
            agree(m.point(p),base.point(p),"a pace stands the path's pose");
            agree(m.velocity(p),base.velocity(p).map(|v| v*rate),"at the pace's rate");
        }
    }
    // over the whole arch the path is swept from 0 to its top and back: once, over [0, 3]
    let (path,roll) = paced.unpaced([0.,2.]).unwrap();
    assert_eq!(path.screw(),Some(s));
    assert_eq!(roll,[0.,3.]);
    assert_eq!(paced.unpaced([1.5,2.]).unwrap().1,[0.,2.25]);
}

/// A slide along `x` evaluated only, as a frame along a path is.
#[derive(Debug)]
pub(crate) struct Evaluated;

impl Path for Evaluated {
    fn at(&self,t: f64) -> Result<Motion,KernelError> { Ok(Motion::translation([2.*t,0.,0.],[2.,0.,0.]).unwrap()) }
}

#[test]
fn a_path_of_pieces_meets_or_is_refused() {
    let turn = Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:1.,advance:0.};
    let piece = |over,path| Piece {over,path};
    // a turn, then (where it stands at no turn) a slide read by evaluating it from there
    let shifted = Trajectory::General(Arc::new(Evaluated));
    let whole = Trajectory::pieces(vec![piece([-1.,0.],Trajectory::Twist(turn)),piece([0.,2.],shifted.clone())],1e-12,100.).unwrap();
    same_motion(whole.at(-0.5).unwrap(),turn.at(-0.5).unwrap(),"the first piece");
    same_motion(whole.at(1.5).unwrap(),Evaluated.at(1.5).unwrap(),"the second piece");
    assert!(whole.at(2.5).is_err(),"no piece stands at 2.5");
    assert!(whole.screw().is_none());
    // pieces that part where they meet, that leave a gap, or that span nothing are refused
    let parted = Trajectory::pieces(vec![piece([-1.,0.5],Trajectory::Twist(turn)),piece([0.5,2.],shifted.clone())],1e-9,100.);
    assert!(parted.unwrap_err().msg.contains("part by"));
    let gap = Trajectory::pieces(vec![piece([-1.,0.],Trajectory::Twist(turn)),piece([0.1,2.],shifted.clone())],1e-9,100.);
    assert!(gap.unwrap_err().msg.contains("followed by"));
    assert!(Trajectory::pieces(vec![piece([1.,1.],shifted)],1e-9,100.).is_err());
}

/// **A twist after a placement** (`Trajectory::Placed`, #721 K1: a turn taking a tool up where a
/// slide left it): its pose at `t` is the placement and then the twist's, its rate (of a point of
/// what it carries) the twist's at where the placement put it, and its speed bound holds every
/// point it carries back no slower than it moves, each read by differences.
#[test]
fn a_twist_after_a_placement_is_the_placement_then_the_twist() {
    use gcs_core::brep::trajectory::Screw;
    let twist = Screw {origin:[1.,2.,0.],axis:[0.,0.6,0.8],ratio:0.7,advance:1.3};
    let at = Screw {origin:[0.;3],axis:[1.,0.,0.],ratio:0.4,advance:2.}.pose(1.);
    let placed = Trajectory::Placed {twist,at};
    let domain = gcs_core::brep::util::interval::Interval::new(-1.,2.).unwrap();
    for &t in &[-0.5,0.,0.3,1.7] {
        let m = placed.at(t).unwrap();
        let want = at.then(twist.pose(t));
        for p in PROBES {
            agree(m.point(p),want.point(p),"a point");
            // the rate by differences
            let h = 1e-6;
            let (a,b) = (placed.pose(t-h).unwrap().point(p),placed.pose(t+h).unwrap().point(p));
            let rate: [f64;3] = std::array::from_fn(|k| (b[k]-a[k])/(2.*h));
            let v = m.velocity(p);
            assert!((0..3).all(|k| (v[k]-rate[k]).abs() <= 1e-6*(1.+rate[k].abs())),"{v:?} against {rate:?}");
            // carried back, no faster than the bound
            let back = |s: f64| placed.pose(s).unwrap().inverse().point(p);
            let (a,b) = (back(t-h),back(t+h));
            let speed = ((0..3).map(|k| (b[k]-a[k]).powi(2)).sum::<f64>()).sqrt()/(2.*h);
            let bound = placed.inverse_point_speed_bound(p,domain).unwrap();
            assert!(speed <= bound*(1.+1e-6),"{speed} past {bound}");
        }
    }
}

/// The cubic Bézier `poles` as the kernel holds it: a B-spline on clamped knots over [0, 1].
fn bezier(poles: &[V;4]) -> gcs_core::brep::nurbs::BSpline {
    gcs_core::brep::nurbs::BSpline::new(3,vec![0.,0.,0.,0.,1.,1.,1.,1.],poles.to_vec()).unwrap()
}

/// **A frame carried along a curve** (`trajectory::Along`, #721 K2): along a curve in a plane it is
/// the turn about the plane's normal by the tangent's angle and the curve's point, exactly, what is
/// drawn at the start carried rigidly; along a curve in space it turns about no tangent
/// (rotation-minimising); its rate is its pose's, read by differences; and every stretch of it
/// encloses its poses, so its speed bound holds a point carried back no slower than it moves. A
/// rational curve has no bound on its derivatives here, and its stretches fail closed.
#[test]
fn a_frame_along_a_curve_turns_only_as_its_tangent_does() {
    use gcs_core::brep::trajectory::Along;
    use gcs_core::interval::Interval;
    use gcs_core::space::{dot,norm,normalised};
    // a cubic in the plane z = 2, starting off the origin
    let poles = [[1.,-2.,2.],[11.,-2.,2.],[21.,18.,2.],[31.,18.,2.]];
    let along = Arc::new(Along::new(bezier(&poles)).unwrap());
    for k in 0..=40 {
        let t = k as f64/40.;
        let m = along.at(t).unwrap();
        let (x,d) = along.curve().d1(t);
        let (sn,cs) = (d[1]/d[0].hypot(d[1]),d[0]/d[0].hypot(d[1]));
        // the turn about z by the tangent's angle, the start carried to the curve's point
        let exact = |p: V| { let q = [p[0]-poles[0][0],p[1]-poles[0][1],p[2]-poles[0][2]]; [cs*q[0]-sn*q[1]+x[0],sn*q[0]+cs*q[1]+x[1],q[2]+x[2]] };
        for p in PROBES { agree(m.point(p),exact(p),&format!("the frame at {t}")) }
        // its rate, by a central difference of the pose
        let (a,b) = ((t-1e-6).max(0.),(t+1e-6).min(1.));
        let (pa,pb) = (along.pose(a).unwrap(),along.pose(b).unwrap());
        for p in PROBES {
            let fd: V = std::array::from_fn(|i| (pb.point(p)[i]-pa.point(p)[i])/(b-a));
            let v = m.velocity(p);
            let size = 1.+v.iter().map(|x| x.abs()).fold(0.,f64::max);
            for i in 0..3 { assert!((fd[i]-v[i]).abs() <= 1e-5*size,"the rate at {t}: {v:?} against {fd:?}") }
        }
    }
    // a curve in space: the frame turns about no tangent, ω·T = 0, ω read from R′Rᵀ by differences
    let spatial = Along::new(bezier(&[[0.,0.,0.],[10.,0.,5.],[20.,20.,-5.],[30.,20.,10.]])).unwrap();
    for k in 1..40 {
        let t = k as f64/40.;
        let h = 1e-5;
        let (a,b,m) = (spatial.pose(t-h).unwrap(),spatial.pose(t+h).unwrap(),spatial.pose(t).unwrap());
        // W = (R(t+h) − R(t−h)) Rᵀ / 2h, skew: ω = (W₂₁, W₀₂, W₁₀)
        let w = |i: usize,j: usize| (0..3).map(|q| (b.r[i][q]-a.r[i][q])*m.r[j][q]).sum::<f64>()/(2.*h);
        let omega = [w(2,1),w(0,2),w(1,0)];
        let twist = dot(omega,normalised(spatial.curve().d1(t).1).unwrap());
        assert!(twist.abs() <= 1e-6*(1.+norm(omega)),"the frame at {t} turns about its tangent at {twist} (of {})",norm(omega));
    }
    // every stretch encloses its poses, and a point carried back moves within the speed bound
    let path = Trajectory::General(along.clone());
    for [lo,hi] in [[0.,1.],[0.,0.1],[0.43,0.57],[0.9,1.]] {
        let g = along.stretch(lo,hi).expect("a polynomial curve's stretch");
        let bounds = PROBES.map(|p| path.inverse_point_speed_bound(p,Interval::new(lo,hi).unwrap()).unwrap());
        for j in 0..=200 {
            let t = lo+(hi-lo)*j as f64/200.;
            let pose = along.pose(t).unwrap();
            let at = along.curve().point(t);
            for k in 0..3 {
                assert!(g.point[k].contains(at[k]),"the stretch [{lo}, {hi}] holds the curve's point at {t}");
                for q in 0..3 { assert!(g.turn[k][q].contains(pose.r[k][q]),"the stretch [{lo}, {hi}] holds the turn at {t}") }
            }
            let back = along.at(t).unwrap().inverse();
            for (p,bound) in PROBES.iter().zip(bounds) {
                let speed = norm(back.velocity(*p));
                assert!(speed <= bound,"{p:?} carried back at {t} moves at {speed}, past the bound {bound}");
            }
        }
    }
    let rational = gcs_core::brep::nurbs::BSpline::rational(2,vec![0.,0.,0.,1.,1.,1.],vec![[1.,0.,0.],[1.,1.,0.],[0.,1.,0.]],
        vec![1.,std::f64::consts::FRAC_1_SQRT_2,1.]).unwrap();
    assert!(Along::new(rational).unwrap().stretch(0.,1.).is_none(),"a rational curve's stretch has no bound here");
}
