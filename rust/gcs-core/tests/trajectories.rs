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
struct Arch;

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
struct Evaluated;

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
