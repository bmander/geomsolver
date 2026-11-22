//! **A boundary read from a file is a solid's, or refused** (`brep::step_read::sound`; #993
//! stage 1): what closes edge by edge and still bounds no solid is told from what does.
//!
//! The kernel's own solids — primitives through their poles and round their seams, prisms with
//! holes, revolutions, Booleans along traced meetings, a spline block, a hollowed solid and
//! rounded ones — are each sound; each way of being no solid is made by mutating a sound one,
//! `Brep::check` still passing where the point is that it does not see it, and is refused with its
//! `Why` and its witness.
use gcs_core::brep::boolean::{combined,Op};
use gcs_core::brep::build::{cone,disc,prism,revolve,sphere,square,torus,Profile,ProfileEdge};
use gcs_core::brep::geom::{Frame,Rigid,Surface,V};
use gcs_core::brep::nurbs::Net;
use gcs_core::brep::offset::{shelled,Join,Side};
use gcs_core::brep::step_read::context::Context;
use gcs_core::brep::step_read::sound::{sound,Unsound,LOOSE};
use gcs_core::brep::step_read::Why;
use gcs_core::brep::topo::{reverse_loop,Brep,Coedge,EdgeCurve,Face,Pcurve};
use crate::brep::{block,poly};
use std::f64::consts::{PI,TAU};

const UP: V = [0.,0.,1.];

/// The context a file in millimetres stating 1e-7 mm reads in.
fn context() -> Context { Context {entity:1,length:1.,angle:1.,uncertainty:Some(1e-7)} }

/// `b` checked as the kernel holds it, then held sound.
fn is_sound(what: &str,b: &Brep) {
    b.check(1e-7).unwrap_or_else(|e| panic!("{what}: not checked: {e}"));
    let started = std::time::Instant::now();
    if let Err(u) = sound(b,&context()) { panic!("{what}: refused: {u:?}") }
    if std::env::var_os("SOUND_TIMES").is_some() { eprintln!("{what}: {} faces, sound in {:?}",b.faces.len(),started.elapsed()); }
}

/// `b` refused unsound, its witness the face `face` or the edge `edge`.
fn refused(b: &Brep,face: Option<usize>,edge: Option<usize>,says: &str) -> Unsound {
    let u = sound(b,&context()).expect_err("read sound");
    assert_eq!(u.why,Why::Unsound,"{u:?}");
    assert_eq!((u.face,u.edge),(face,edge),"{u:?}");
    assert!(u.msg.contains(says),"{u:?}");
    u
}

fn line(a: V,b: V) -> ProfileEdge { ProfileEdge::Line {a,b} }
fn circle(c: V,r: f64) -> ProfileEdge { ProfileEdge::Arc {frame:Frame::new(c,UP,[1.,0.,0.]),r,span:None} }
fn shift(d: V) -> Rigid { Rigid {t:d,..Rigid::identity()} }

/// A plate 20 by 20 by 4 with round holes of radius `r` at `at`.
fn plate(at: &[[f64;2]],r: f64) -> Brep {
    let mut loops = vec![poly(&[[-10.,-10.,0.],[10.,-10.,0.],[10.,10.,0.],[-10.,10.,0.]])];
    loops.extend(at.iter().map(|c| vec![circle([c[0],c[1],0.],r)]));
    prism(&Profile {names:vec![],origin:[0.;3],normal:UP,loops},0.,4.).unwrap()
}

/// The faces of `b` on a cylinder, in order.
fn walls(b: &Brep) -> Vec<usize> { (0..b.faces.len()).filter(|&f| matches!(b.faces[f].surface,Surface::Cylinder(..))).collect() }

/// The faces `faces` of `b`, with their edges and vertices (and so whatever other face shares
/// them), carried by `m`: a hole's wall moved with its rims, a shell moved whole.
fn carried(b: &Brep,faces: &[usize],m: &Rigid) -> Brep {
    let mut out = b.clone();
    let mut edges: Vec<usize> = faces.iter().flat_map(|&f| b.faces[f].loops.iter().flatten().map(|c| c.edge as usize)).collect();
    edges.sort_unstable();
    edges.dedup();
    let mut vertices: Vec<usize> = edges.iter().flat_map(|&e| b.edges[e].v.map(|v| v as usize)).collect();
    vertices.sort_unstable();
    vertices.dedup();
    for &v in &vertices { out.vertices[v].p = m.point(b.vertices[v].p); }
    for &e in &edges { if let EdgeCurve::Curve(c) = &b.edges[e].curve { out.edges[e].curve = EdgeCurve::Curve(c.moved(m)); } }
    for &f in faces { out.faces[f].surface = b.faces[f].surface.moved(m); }
    out
}

/// `b` turned inside out: every face's sense flipped and its loops walked the other way.
fn flipped(b: &Brep,faces: &[usize]) -> Brep {
    let mut out = b.clone();
    for &f in faces {
        let face = &mut out.faces[f];
        face.reversed = !face.reversed;
        face.loops = std::mem::take(&mut face.loops).into_iter().map(reverse_loop).collect();
    }
    out
}

/// The planar face of `b` holding `n` loops.
fn holed(b: &Brep,n: usize) -> usize { (0..b.faces.len()).find(|&f| b.faces[f].loops.len() == n).unwrap() }

/// **The kernel's own solids are sound**: primitives through their poles and round their seams,
/// prisms with holes, revolutions, Booleans along traced meetings, a spline block and a drum cut
/// from it, a hollowed solid (a void), rounded and filleted solids, and a solid pinched at an edge.
#[test]
fn the_kernels_own_solids_are_sound() {
    let ball = sphere([1.,2.,3.],[0.3,0.2,1.],5.).unwrap();
    is_sound("sphere",&ball);
    is_sound("cone",&cone([0.;3],UP,5.,2.,6.).unwrap());
    is_sound("cone to its apex",&cone([0.;3],UP,5.,0.,6.).unwrap());
    is_sound("torus",&torus([0.;3],UP,10.,2.).unwrap());
    let rod = prism(&disc([0.,0.,-1.],UP,3.),0.,12.).unwrap();
    is_sound("cylinder",&rod);
    let cube = prism(&square([0.;3],UP,5.),0.,10.).unwrap();
    is_sound("block",&cube);
    is_sound("plate with holes",&plate(&[[-5.,-5.],[5.,5.],[5.,-5.],[0.,0.]],2.));
    let drilled = combined(&cube,&rod,Op::Cut,0.).unwrap();
    is_sound("block drilled through",&drilled);
    // a revolution: a profile with an arc turned a whole turn and a quarter
    let section = Profile {names:vec![],origin:[0.;3],normal:[0.,1.,0.],loops:vec![vec![line([2.,0.,0.],[6.,0.,0.]),line([6.,0.,0.],[6.,0.,3.]),
        ProfileEdge::Arc {frame:Frame::new([4.,0.,3.],[0.,1.,0.],[1.,0.,0.]),r:2.,span:Some([0.,PI])},line([2.,0.,3.],[2.,0.,0.])]]};
    is_sound("revolved",&revolve(&section,[0.;3],UP,TAU).unwrap());
    is_sound("revolved a quarter",&revolve(&section,[0.;3],UP,PI/2.).unwrap());
    // Booleans along traced meetings: two crossing cylinders, a ball on a block, a ball cut
    let along_x = prism(&disc([-15.,0.,0.],[1.,0.,0.],5.),0.,30.).unwrap();
    let along_y = prism(&disc([0.,-15.,0.],[0.,1.,0.],5.),0.,30.).unwrap();
    is_sound("Steinmetz solid",&combined(&along_x,&along_y,Op::Common,0.).unwrap());
    is_sound("crossed pipes",&combined(&along_x,&along_y,Op::Union,0.).unwrap());
    let above = prism(&square([0.;3],UP,10.),0.,10.).unwrap();
    is_sound("hemisphere",&combined(&ball,&above,Op::Common,0.).unwrap());
    is_sound("ball bored",&combined(&ball,&prism(&disc([1.,2.,-5.],UP,2.),0.,20.).unwrap(),Op::Cut,0.).unwrap());
    is_sound("torus cut by a block",&combined(&torus([0.;3],[0.2,0.,1.],10.,2.).unwrap(),&block([0.,0.,-5.],[20.,20.,5.]),Op::Cut,0.).unwrap());
    // a spline block and a drum in common with it
    let spline = domed();
    is_sound("spline block",&spline);
    let drum = prism(&disc([10.,10.,-1.],UP,6.),0.,30.).unwrap();
    is_sound("drum under a spline",&combined(&spline,&drum,Op::Common,0.).unwrap());
    is_sound("spline block drilled",&combined(&spline,&prism(&disc([10.,10.,-1.],UP,3.),0.,30.).unwrap(),Op::Cut,0.).unwrap());
    // a hollowed solid: its wall's inside a void
    let hollow = shelled(&block([0.,0.,0.],[10.,8.,6.]),&[],1.,Side::In,Join::Round).unwrap();
    is_sound("hollowed",&hollow);
    // rounded: a cylinder's rim cut round (a torus's quarter), a bore's floor filled round, a box's
    // edges rounded in turn
    let mut section = disc([0.;3],UP,5.);
    section.names = vec![vec!["side".into()]];
    let can = prism(&section,0.,10.).unwrap();
    let rim = gcs_core::brep::fillet::roll(&can,[0.,5.,10.],2.,1e-9*can.size()).unwrap();
    is_sound("a cylinder's rim rounded",&combined(&can,&rim.piece,Op::Cut,0.).unwrap());
    let bored = combined(&block([-10.,-10.,0.],[10.,10.,12.]),&prism(&disc([0.,0.,4.],UP,5.),0.,8.).unwrap(),Op::Cut,0.).unwrap();
    is_sound("a block bored",&bored);
    let fill = gcs_core::brep::fillet::roll(&bored,[5.,0.,4.],2.,1e-9*bored.size()).unwrap();
    is_sound("a bore's floor filled",&combined(&bored,&fill.piece,if fill.concave { Op::Union } else { Op::Cut },0.).unwrap());
    let mut boxed = block([0.;3],[10.,4.,6.]);
    for (f,name) in boxed.faces.iter_mut().zip(["front","right","back","left"]) { f.name = name.into(); }
    let rounded = [[5.,0.,6.],[5.,4.,6.]].iter().fold(boxed,|b,&at| {
        let round = gcs_core::brep::fillet::roll(&b,at,1.,gcs_core::brep::geom::tolerance(b.size())).unwrap();
        combined(&b,&round.piece,if round.concave { Op::Union } else { Op::Cut },0.).unwrap()
    });
    is_sound("a box's two edges rounded",&rounded);
    let main = prism(&Profile {names:vec![],origin:[0.;3],normal:[1.,0.,0.],
        loops:vec![vec![ProfileEdge::Arc {frame:Frame::new([0.;3],[1.,0.,0.],[0.,0.,1.]),r:10.,span:None}]]},-30.,30.).unwrap();
    let tee = combined(&main,&prism(&disc([0.;3],UP,6.),0.,25.).unwrap(),Op::Union,0.).unwrap();
    is_sound("tee",&tee);
    let rolled = gcs_core::brep::fillet::roll(&tee,[6.,0.,10.],2.,1e-9).unwrap();
    is_sound("tee filleted",&combined(&tee,&rolled.piece,Op::Union,0.).unwrap());
    // two blocks touching along an edge: the kernel holds the line twice, its ends pinches
    let touching = combined(&block([0.;3],[1.,1.,1.]),&block([1.,1.,0.],[2.,2.,1.]),Op::Union,0.).unwrap();
    is_sound("blocks touching along an edge",&touching);
}

/// brep-bench's nurbs/bezier-dome block: lofted along x, its top a cubic net.
fn domed() -> Brep {
    const H: [[f64;4];4] = [[4.,6.,7.,5.],[6.,14.,12.,7.],[5.,13.,16.,8.],[3.,6.,8.,6.]];
    let clamped = |d: usize| [vec![0.;d+1],vec![1.;d+1]].concat();
    let x = |i: usize| 20.*i as f64/3.;
    let net = |du: usize,rows: &dyn Fn(usize) -> [V;4]| Net {du,dv:3,uknots:clamped(du),vknots:clamped(3),weights:None,
        poles:(0..=du).map(|k| rows(k).to_vec()).collect()};
    let floor = |y: f64| move |i: usize| [x(i),y,0.];
    let edge = |j: usize| move |i: usize| [x(i),20.*j as f64/3.,H[i][j]];
    let row = |f: &dyn Fn(usize) -> V| -> [V;4] { std::array::from_fn(f) };
    let nets = vec![
        net(1,&|k| if k == 0 { row(&floor(0.)) } else { row(&floor(20.)) }),
        net(1,&|k| if k == 0 { row(&floor(20.)) } else { row(&edge(3)) }),
        net(3,&|k| row(&edge(3-k))),
        net(1,&|k| if k == 0 { row(&edge(0)) } else { row(&floor(0.)) }),
    ];
    gcs_core::brep::build::lofted([0.;3],[1.,0.,0.],[0.,20.],vec![nets]).unwrap()
}

/// **A hole crossing its outer loop, a hole outside it and two holes overlapping are refused**,
/// the face that holds them the witness: each hole's wall moved with its rims, so every loop still
/// closes and turns its way and `check` passes.
#[test]
fn holes_crossing_outside_or_overlapping_are_refused() {
    let one = plate(&[[0.,0.]],3.);
    let wall = walls(&one)[0];
    let crossing = carried(&one,&[wall],&shift([8.,0.,0.]));
    crossing.check(1e-7).unwrap();
    let face = holed(&crossing,2);
    refused(&crossing,Some(face),None,"crosses");
    let outside = carried(&one,&[wall],&shift([25.,0.,0.]));
    outside.check(1e-7).unwrap();
    refused(&outside,Some(holed(&outside,2)),None,"outside its outer loop");
    // just inside, it is sound
    is_sound("hole moved within",&carried(&one,&[wall],&shift([6.5,0.,0.])));
    let two = plate(&[[-4.,0.],[4.,0.]],3.);
    is_sound("two holes",&two);
    let overlapping = carried(&two,&[walls(&two)[1]],&shift([-3.,0.,0.]));
    overlapping.check(1e-7).unwrap();
    refused(&overlapping,Some(holed(&overlapping,3)),None,"crosses loop");
    // a small hole moved whole into a large one: nested, never crossing
    let small = {
        let mut loops = vec![poly(&[[-10.,-10.,0.],[10.,-10.,0.],[10.,10.,0.],[-10.,10.,0.]])];
        loops.push(vec![circle([-4.,0.,0.],3.)]);
        loops.push(vec![circle([5.,0.,0.],1.)]);
        prism(&Profile {names:vec![],origin:[0.;3],normal:UP,loops},0.,4.).unwrap()
    };
    is_sound("a large and a small hole",&small);
    let w = walls(&small);
    let little = *w.iter().find(|&&f| matches!(small.faces[f].surface,Surface::Cylinder(_,r) if r < 2.)).unwrap();
    let inner = carried(&small,&[little],&shift([-9.,0.,0.]));
    inner.check(1e-7).unwrap();
    refused(&inner,Some(holed(&inner,3)),None,"inside another hole");
}

/// A sheet of one plane face bounded by the polygon `pts`, each edge's tolerance `tol`.
fn sheet(pts: &[V],tol: f64) -> Brep {
    let mut b = Brep::default();
    let vs: Vec<u32> = pts.iter().map(|&p| b.vertex(p)).collect();
    let n = vs.len();
    let uses = (0..n).map(|i| {
        let (a,z) = (pts[i],pts[(i+1)%n]);
        let d = gcs_core::space::sub(z,a);
        let l = gcs_core::space::norm(d);
        let e = b.edge(EdgeCurve::Curve(gcs_core::brep::geom::Curve::Line {p:a,d:gcs_core::space::scale(d,1./l)}),[0.,l],[vs[i],vs[(i+1)%n]]);
        b.edges[e as usize].tol = tol;
        Coedge {edge:e,reversed:false,pcurve:Pcurve::Line {a:[a[0],a[1]],b:[z[0],z[1]]}}
    }).collect();
    b.faces.push(Face {surface:Surface::Plane(Frame::new([0.;3],UP,[1.,0.,0.])),reversed:false,loops:vec![uses],name:String::new()});
    b
}

/// `b`'s faces held sound: whatever `sound` says, it is not of its faces' loops (a sheet bounds
/// no volume, which is all it may be refused for).
fn faces_sound(what: &str,b: &Brep) {
    if let Err(u) = sound(b,&context()) { assert!(u.msg.contains("bounds no volume") || u.msg.contains("inside out"),"{what}: {u:?}") }
}

/// **A loop crossing itself deeper than its tolerance is refused; one touching itself, or
/// crossing within its tolerance, is read**, the face the witness: a bow tie, a square whose
/// corner is pulled across its opposite side, and a loop whose corner is pushed through its
/// opposite edge by 0.5 are refused; that corner on the edge (a pinch away from any vertex of
/// it), or pushed through by a tenth of the edges' tolerance, is a tolerant writer's habit and is
/// read. Neither bow tie turns one way, so `check` refuses them too; `sound` is read alone.
#[test]
fn a_loop_crossing_itself_is_refused() {
    refused(&sheet(&[[0.,0.,0.],[4.,4.,0.],[4.,0.,0.],[0.,4.,0.]],0.),Some(0),None,"loop 0 crosses itself");
    refused(&sheet(&[[0.,0.,0.],[4.,0.,0.],[-1.,6.,0.],[0.,4.,0.],[2.,5.,0.]],0.),Some(0),None,"crosses itself");
    let pushed = |d: f64,tol: f64| sheet(&[[0.,0.,0.],[4.,0.,0.],[4.,4.,0.],[2.,-d,0.],[0.,4.,0.]],tol);
    refused(&pushed(0.5,1e-3),Some(0),None,"crosses itself");
    faces_sound("a corner on its opposite edge",&pushed(0.,0.));
    faces_sound("a corner touching its opposite edge within tolerance",&pushed(1e-4,1e-3));
    faces_sound("a corner through its opposite edge within tolerance",&pushed(-1e-4,1e-3));
    // a loop doubling back along itself within its tolerance (a pcurve wiggling at a tolerant
    // edge): the stretch it comes back over is its own neighbourhood
    faces_sound("a zigzag within tolerance",&sheet(&[[0.,0.,0.],[4.,0.,0.],[4.,4.,0.],[2.,4.,0.],[2.0005,4.,0.],[0.,4.,0.]],1e-3));
}

/// A band on a cylinder of radius 2 wound round it `turns` times as a helix rising `pitch` a
/// turn, `width` wide along the axis: its one loop in the cylinder's parameters, its edges lines
/// there.
fn band(turns: f64,pitch: f64,width: f64) -> Brep {
    let mut b = Brep::default();
    let (u,v) = (TAU*turns,pitch*turns);
    let at: [[f64;2];4] = [[0.,0.],[u,v],[u,v+width],[0.,width]];
    let vs: Vec<u32> = at.iter().map(|p| b.vertex([2.*p[0].cos(),2.*p[0].sin(),p[1]])).collect();
    let uses = (0..4).map(|i| {
        let (a,z) = (at[i],at[(i+1)%4]);
        let (p,q) = (b.vertices[vs[i] as usize].p,b.vertices[vs[(i+1)%4] as usize].p);
        let d = gcs_core::space::sub(q,p);
        let l = gcs_core::space::norm(d).max(1e-9);
        let e = b.edge(EdgeCurve::Curve(gcs_core::brep::geom::Curve::Line {p,d:gcs_core::space::scale(d,1./l)}),[0.,l],[vs[i],vs[(i+1)%4]]);
        Coedge {edge:e,reversed:false,pcurve:Pcurve::Line {a,b:z}}
    }).collect();
    b.faces.push(Face {surface:Surface::Cylinder(Frame::new([0.;3],UP,[1.,0.,0.]),2.),reversed:false,loops:vec![uses],name:String::new()});
    b
}

/// **A face reaching round its surface more than once is read where it never meets itself a
/// period along, refused where it does**: a thread's band wound two and a half times round its
/// cylinder, narrower than its pitch, is read; wider than its pitch, each turn crosses the next.
#[test]
fn a_band_wound_round_a_cylinder_overlaps_itself_only_where_its_turns_meet() {
    faces_sound("a band narrower than its pitch",&band(2.5,4.,1.));
    refused(&band(2.5,4.,5.),Some(0),None,"crosses");
}

/// A face on a cylinder of radius 2 bounded by the polygon `at` in its parameters, its edges
/// lines there; with `seam`, its second and fourth sides one edge used both ways (they must be a
/// period apart).
fn on_cylinder(at: &[[f64;2]],seam: bool) -> Brep {
    let mut b = Brep::default();
    let n = at.len();
    let vs: Vec<u32> = at.iter().map(|p| b.vertex([2.*p[0].cos(),2.*p[0].sin(),p[1]])).collect();
    let mut uses = Vec::new();
    for i in 0..n {
        let (a,z) = (at[i],at[(i+1)%n]);
        if seam && i == 3 {
            let e = uses.iter().map(|c: &Coedge| c.edge).nth(1).unwrap();
            uses.push(Coedge {edge:e,reversed:true,pcurve:Pcurve::Line {a:z,b:a}});
            continue
        }
        let (p,q) = (b.vertices[vs[i] as usize].p,b.vertices[vs[(i+1)%n] as usize].p);
        let d = gcs_core::space::sub(q,p);
        let l = gcs_core::space::norm(d).max(1e-9);
        let e = b.edge(EdgeCurve::Curve(gcs_core::brep::geom::Curve::Line {p,d:gcs_core::space::scale(d,1./l)}),[0.,l],[vs[i],vs[(i+1)%n]]);
        uses.push(Coedge {edge:e,reversed:false,pcurve:Pcurve::Line {a,b:z}});
    }
    b.faces.push(Face {surface:Surface::Cylinder(Frame::new([0.;3],UP,[1.,0.,0.]),2.),reversed:false,loops:vec![uses],name:String::new()});
    b
}

/// **A loop once round a periodic surface whose seam leans across it is read; one lapped over
/// itself is refused**: a band whose seam runs a radian round as it rises reaches more than a
/// period along `u` and meets its image a period along only on its seam; a loop reaching two
/// radians past a whole turn along its foot overlaps itself.
#[test]
fn a_band_whose_seam_leans_is_read() {
    faces_sound("a band with a leaning seam",&on_cylinder(&[[0.,0.],[TAU,0.],[TAU+1.,1.],[1.,1.]],true));
    refused(&on_cylinder(&[[0.,0.],[TAU+2.,0.],[TAU+3.,1.],[1.,1.]],false),Some(0),None,"overlap");
}

/// **A use whose curve in its face's parameters leaps a period is refused**, the face its
/// witness: an arc of three quarters of a turn on a cylinder, its pcurve the arc inverted onto
/// the cylinder between ends a quarter turn apart the short way, so that it changes branch half
/// way (what a reader that took an arc's two ends for the whole of it hands over).
#[test]
fn a_use_leaping_a_period_is_refused() {
    use gcs_core::brep::geom::Curve;
    let mut b = Brep::default();
    let frame = Frame::new([0.;3],UP,[1.,0.,0.]);
    let p = |a: f64,z: f64| [2.*a.cos(),2.*a.sin(),z];
    let vs = [b.vertex(p(0.,0.)),b.vertex(p(1.5*PI,0.)),b.vertex(p(1.5*PI,1.)),b.vertex(p(0.,1.))];
    let arc = |z: f64| EdgeCurve::Curve(Curve::Circle(Frame::new([0.,0.,z],UP,[1.,0.,0.]),2.));
    let line = |a: V,z: V| EdgeCurve::Curve(Curve::Line {p:a,d:gcs_core::space::scale(gcs_core::space::sub(z,a),1./gcs_core::space::distance(a,z))});
    let e0 = b.edge(arc(0.),[0.,1.5*PI],[vs[0],vs[1]]);
    let e1 = b.edge(line(p(1.5*PI,0.),p(1.5*PI,1.)),[0.,1.],[vs[1],vs[2]]);
    let e2 = b.edge(arc(1.),[0.,1.5*PI],[vs[3],vs[2]]);
    let e3 = b.edge(line(p(0.,1.),p(0.,0.)),[0.,1.],[vs[3],vs[0]]);
    let loops = vec![vec![
        Coedge {edge:e0,reversed:false,pcurve:Pcurve::Inverse {a:[0.,0.],b:[-0.5*PI,0.]}},
        Coedge {edge:e1,reversed:false,pcurve:Pcurve::Line {a:[1.5*PI,0.],b:[1.5*PI,1.]}},
        Coedge {edge:e2,reversed:true,pcurve:Pcurve::Line {a:[0.,1.],b:[1.5*PI,1.]}},
        Coedge {edge:e3,reversed:false,pcurve:Pcurve::Line {a:[0.,1.],b:[0.,0.]}},
    ]];
    b.faces.push(Face {surface:Surface::Cylinder(frame,2.),reversed:false,loops,name:String::new()});
    refused(&b,Some(0),None,"a period");
}

/// **Lumps and voids nested as their senses say are sound, however many; a shell inside out, a
/// void outside every lump, a lump inside a lump, a void inside a void, and two shells crossing
/// are refused**, a face of the offending shell the witness. Two blocks apart (a body a cut
/// parted, which `step::write` writes as one shell's faces) and an island in a cavity are sound.
#[test]
fn shells_inside_out_or_apart_are_refused() {
    let cube = block([0.;3],[10.,10.,10.]);
    let all: Vec<usize> = (0..cube.faces.len()).collect();
    let inverted = flipped(&cube,&all);
    inverted.check(1e-7).unwrap();
    refused(&inverted,Some(0),None,"inside out");
    // two blocks apart: two lumps of one body
    let pair = cube.joined(&block([20.,0.,0.],[25.,5.,5.]));
    pair.check(1e-7).unwrap();
    is_sound("two blocks apart",&pair);
    let second: Vec<usize> = (cube.faces.len()..pair.faces.len()).collect();
    // the small one turned inside out beside the big one: a void outside it
    let beside = flipped(&pair,&second);
    beside.check(1e-7).unwrap();
    refused(&beside,Some(second[0]),None,"outside the solid's outer shell");
    // inside the big one, the same void is sound
    let within = flipped(&cube.joined(&block([2.,2.,2.],[5.,5.,5.])),&second);
    is_sound("block with a cubic void",&within);
    // and a block inside that void, an island in a cavity: sound; not turned, a lump in a lump
    let island = within.joined(&block([3.,3.,3.],[4.,4.,4.]));
    is_sound("an island in a cavity",&island);
    refused(&cube.joined(&block([2.,2.,2.],[5.,5.,5.])),Some(6),None,"inside the solid of face 0");
    // two blocks overlapping: their shells cross
    let overlapping = cube.joined(&block([5.,5.,5.],[15.,15.,15.]));
    overlapping.check(1e-7).unwrap();
    let u = sound(&overlapping,&context()).unwrap_err();
    assert!(u.msg.contains("crosses the shell"),"{u:?}");
    // the hollowed block's void carried out through its wall
    let hollow = shelled(&block([0.,0.,0.],[10.,8.,6.]),&[],1.,Side::In,Join::Round).unwrap();
    let shells = shells_of(&hollow);
    assert_eq!(shells.len(),2);
    let flux = gcs_core::brep::props::fluxes(&hollow);
    let void = shells.iter().find(|s| s.iter().map(|&f| flux[f]).sum::<f64>() < 0.).unwrap().clone();
    let out = carried(&hollow,&void,&shift([30.,0.,0.]));
    out.check(1e-7).unwrap();
    refused(&out,Some(void[0]),None,"outside the solid's outer shell");
    // and the outer shell flipped with it: two shells inside out
    let both = flipped(&hollow,&(0..hollow.faces.len()).collect::<Vec<_>>());
    both.check(1e-7).unwrap();
    let u = sound(&both,&context()).unwrap_err();
    assert!(u.msg.contains("inside out") || u.msg.contains("outside"),"{u:?}");
    // a void inside a void
    let nest = flipped(&cube.joined(&block([1.,1.,1.],[9.,9.,9.])).joined(&block([3.,3.,3.],[6.,6.,6.])),&(6..18).collect::<Vec<_>>());
    nest.check(1e-7).unwrap();
    refused(&nest,Some(12),None,"inside the void of face 6");
    // no faces at all: nothing bounded
    let u = sound(&Brep::default(),&context()).unwrap_err();
    assert_eq!(u.why,Why::Open,"{u:?}");
}

/// The faces of `b` by shell, each in order.
fn shells_of(b: &Brep) -> Vec<Vec<usize>> {
    let mut shell = vec![usize::MAX;b.faces.len()];
    let mut out: Vec<Vec<usize>> = Vec::new();
    for start in 0..b.faces.len() {
        if shell[start] != usize::MAX { continue }
        let k = out.len();
        out.push(Vec::new());
        let mut stack = vec![start];
        shell[start] = k;
        while let Some(f) = stack.pop() {
            out[k].push(f);
            for c in b.faces[f].loops.iter().flatten() {
                for g in 0..b.faces.len() {
                    if shell[g] == usize::MAX && b.faces[g].loops.iter().flatten().any(|d| d.edge == c.edge) { shell[g] = k; stack.push(g); }
                }
            }
        }
        out[k].sort_unstable();
    }
    out
}

/// **A vertex or an edge moved off its faces by more than `LOOSE` of the solid is refused**, an
/// edge through it the witness, its tolerance said against the file's uncertainty; moved by less,
/// its tolerance measured, it is sound. An edge whose tolerance is not small against its length,
/// and an edge whose vertices stand within their tolerances, are a tolerant writer's habits, and
/// sound.
#[test]
fn tolerances_past_the_geometry_are_refused() {
    let cube = block([0.;3],[10.,10.,10.]);
    let bar = LOOSE*cube.size();
    // a corner moved off: its edges' ends measured off it
    let moved = |d: f64| {
        let mut b = cube.clone();
        b.vertices[0].p[2] += d;
        b.measure_tolerances();
        b
    };
    is_sound("a corner a hair off",&moved(1e-4*bar));
    let off = moved(3.*bar);
    off.check(1e-7).unwrap();
    let u = sound(&off,&context()).unwrap_err();
    assert_eq!(u.why,Why::Unsound);
    let e = u.edge.unwrap();
    assert!(cube.edges[e].v.contains(&0),"{u:?}");
    assert!(u.msg.contains("times the uncertainty the file states"),"{u:?}");
    // an edge's line moved off its two faces
    let mut lifted = cube.clone();
    let e = 3;
    if let EdgeCurve::Curve(c) = &cube.edges[e].curve { lifted.edges[e].curve = EdgeCurve::Curve(c.moved(&shift([0.,0.,2.*bar]))); }
    lifted.measure_tolerances();
    lifted.check(1e-7).unwrap();
    let u = sound(&lifted,&context()).unwrap_err();
    assert!(u.edge == Some(e) || u.edge.is_some_and(|x| cube.edges[x].v.iter().any(|v| cube.edges[e].v.contains(v))),"{u:?}");
    // a prism with a short edge: its tolerance past half its length, then its ends' tolerances
    // reaching each other — read, as CADGenBench 246 and 217 are
    let short = prism(&Profile {names:vec![],origin:[0.;3],normal:UP,loops:vec![poly(&[[0.,0.,0.],[50.,0.,0.],[50.,30.,0.],[0.01,30.,0.],[0.,30.,0.]])]},0.,20.).unwrap();
    is_sound("a prism with a short edge",&short);
    let e = (0..short.edges.len()).find(|&e| {
        let [a,z] = short.edges[e].v.map(|v| short.vertices[v as usize].p);
        (gcs_core::space::distance(a,z)-0.01).abs() < 1e-9
    }).unwrap();
    let mut thin = short.clone();
    thin.edges[e].tol = 0.6*0.01;
    sound(&thin,&context()).unwrap();
    let mut near = short.clone();
    for v in short.edges[e].v { near.vertices[v as usize].tol = 0.006; }
    sound(&near,&context()).unwrap();
}

/// **A boundary of hundreds of holes is held sound in well under a second**: a plate of 400
/// holes (each wall a cylinder, each rim a circle read in its plane's parameters), the plane faces
/// four hundred loops each — the pairs of pieces read are found by sweeping their boxes, never
/// each against each.
#[test]
fn a_plate_of_four_hundred_holes_is_read_quickly() {
    let at: Vec<[f64;2]> = (0..20).flat_map(|i| (0..20).map(move |j| [-9.5+i as f64,-9.5+j as f64])).collect();
    let b = plate(&at,0.3);
    assert_eq!(b.faces.len(),406);
    // and two thousand
    let many: Vec<[f64;2]> = (0..45).flat_map(|i| (0..45).map(move |j| [-9.68+0.44*i as f64,-9.68+0.44*j as f64])).collect();
    let large = plate(&many,0.15);
    let started = std::time::Instant::now();
    sound(&large,&context()).unwrap();
    let took = started.elapsed();
    eprintln!("a plate of {} faces, {} edges: sound in {took:?}",large.faces.len(),large.edges.len());
    assert!(took.as_secs_f64() < 4.,"{took:?}");
    // its rims given as circles in the caps' parameters, as a native kernel hands them over
    let drawn = in_parameters(&large);
    drawn.check(1e-7).unwrap();
    let started = std::time::Instant::now();
    sound(&drawn,&context()).unwrap();
    eprintln!("its rims as circles in its caps' parameters: sound in {:?}",started.elapsed());
    let started = std::time::Instant::now();
    sound(&b,&context()).unwrap();
    let took = started.elapsed();
    eprintln!("a plate of {} faces, {} edges: sound in {took:?}",b.faces.len(),b.edges.len());
    assert!(took.as_secs_f64() < 2.,"{took:?}");
    // one hole moved onto its neighbour: found among them
    let w = walls(&b);
    let moved = carried(&b,&[w[210]],&shift([0.,0.5,0.]));
    let u = sound(&moved,&context()).unwrap_err();
    assert!(u.msg.contains("crosses loop") && u.face.is_some_and(|f| b.faces[f].loops.len() == 401),"{u:?}");
}


/// `b` with every circle on a plane read in the plane's parameters as a circle of its own
/// (`Pcurve::Curve`), at the edge's own parameter: what a native kernel's pcurve is.
fn in_parameters(b: &Brep) -> Brep {
    use gcs_core::brep::geom::Curve;
    let mut out = b.clone();
    for f in &mut out.faces {
        let Surface::Plane(pf) = f.surface else { continue };
        let local = |d: V| [gcs_core::space::dot(d,pf.x),gcs_core::space::dot(d,pf.y),0.];
        for c in f.loops.iter_mut().flatten() {
            let EdgeCurve::Curve(Curve::Circle(cf,r)) = b.edges[c.edge as usize].curve else { continue };
            let o = local(gcs_core::space::sub(cf.o,pf.o));
            let frame = Frame {o,x:local(cf.x),y:local(cf.y),z:[0.,0.,1.]};
            c.pcurve = Pcurve::Curve(std::sync::Arc::new(Curve::Circle(frame,r)));
        }
    }
    out
}

/// **A native kernel's boundary read through `brep::json` is sound**: a cylinder whose side is a
/// rational spline sheet, its rims rational splines, each cap's rim its own rational spline in the
/// cap's parameters (`Pcurve::Curve`), as OCCT writes them; turned inside out, refused.
#[test]
fn a_native_kernels_cylinder_is_sound() {
    use gcs_core::brep::nurbs::arc;
    let (r,h) = (1.5,4.);
    let c = arc(&Frame::new([0.;3],UP,[1.,0.,0.]),r,[0.,TAU]);
    let w = c.weights.as_ref().unwrap();
    let list = |x: &[f64]| format!("[{}]",x.iter().map(|v| format!("{v:?}")).collect::<Vec<_>>().join(","));
    let rim = |z: f64| format!("{{\"kind\":\"bspline\",\"degree\":2,\"knots\":{},\"poles\":[{}],\"weights\":{}}}",
        list(&c.knots),c.poles.iter().map(|p| list(&[p[0],p[1],z])).collect::<Vec<_>>().join(","),list(w));
    let side = format!("{{\"kind\":\"bspline\",\"du\":2,\"dv\":1,\"uknots\":{},\"vknots\":[0,0,{h:?},{h:?}],\"poles\":[{}],\"weights\":[{}]}}",
        list(&c.knots),c.poles.iter().map(|p| format!("[{},{}]",list(&[p[0],p[1],0.]),list(&[p[0],p[1],h]))).collect::<Vec<_>>().join(","),
        w.iter().map(|&x| list(&[x,x])).collect::<Vec<_>>().join(","));
    let line = |p: [f64;2],d: [f64;2]| format!("{{\"kind\":\"line\",\"p\":[{:?},{:?},0],\"d\":[{:?},{:?},0]}}",p[0],p[1],d[0],d[1]);
    let plane = |z: f64| format!("{{\"kind\":\"plane\",\"frame\":{{\"o\":[0,0,{z:?}],\"x\":[1,0,0],\"y\":[0,1,0],\"z\":[0,0,1]}}}}");
    let used = |e: usize,reversed: bool,pcurve: String| format!("{{\"edge\":{e},\"reversed\":{reversed},\"pcurve\":{pcurve}}}");
    let json = format!("{{\"vertices\":[{{\"p\":{},\"tol\":0}},{{\"p\":{},\"tol\":0}}],\"edges\":[\
        {{\"v\":[0,0],\"t\":[0,{TAU:?}],\"tol\":0,\"curve\":{}}},\
        {{\"v\":[1,1],\"t\":[0,{TAU:?}],\"tol\":0,\"curve\":{}}},\
        {{\"v\":[0,1],\"t\":[0,{h:?}],\"tol\":0,\"curve\":{{\"kind\":\"line\",\"p\":{},\"d\":[0,0,1]}}}}],\"faces\":[\
        {{\"reversed\":false,\"surface\":{side},\"loops\":[{{\"outer\":true,\"uses\":[{},{},{},{}]}}]}},\
        {{\"reversed\":true,\"surface\":{},\"loops\":[{{\"outer\":true,\"uses\":[{}]}}]}},\
        {{\"reversed\":false,\"surface\":{},\"loops\":[{{\"outer\":true,\"uses\":[{}]}}]}}]}}",
        list(&[r,0.,0.]),list(&[r,0.,h]),rim(0.),rim(h),list(&[r,0.,0.]),
        used(0,false,line([0.,0.],[1.,0.])),used(2,false,line([TAU,0.],[0.,1.])),
        used(1,true,line([0.,h],[1.,0.])),used(2,true,line([0.,0.],[0.,1.])),
        plane(0.),used(0,false,rim(0.)),plane(h),used(1,false,rim(0.)));
    let b = gcs_core::brep::json::read(&json).unwrap();
    assert!(b.faces.iter().flat_map(|f| f.loops.iter().flatten()).any(|c| matches!(&c.pcurve,Pcurve::Curve(x) if matches!(**x,gcs_core::brep::geom::Curve::BSpline(_)))));
    is_sound("a native kernel's cylinder",&b);
    let all: Vec<usize> = (0..b.faces.len()).collect();
    refused(&flipped(&b,&all),Some(0),None,"inside out");
}

/// **A spline face trimmed by many traced holes is held sound quickly**: brep-bench's bezier dome
/// in common with a plate of 64 bores, its dome face trimmed by 64 curves traced on the spline,
/// each inverted onto it.
#[test]
fn a_spline_face_of_many_traced_holes_is_read_quickly() {
    let mut loops = vec![poly(&[[1.,1.,-1.],[19.,1.,-1.],[19.,19.,-1.],[1.,19.,-1.]])];
    loops.extend((0..8).flat_map(|i| (0..8).map(move |j| vec![circle([2.5+2.*i as f64,2.5+2.*j as f64,-1.],0.6)])));
    let bores = prism(&Profile {names:vec![],origin:[0.,0.,-1.],normal:UP,loops},0.,30.).unwrap();
    let started = std::time::Instant::now();
    let b = combined(&domed(),&bores,Op::Common,0.).unwrap();
    eprintln!("built in {:?}",started.elapsed());
    is_sound("a dome of 64 traced holes",&b);
    let started = std::time::Instant::now();
    sound(&b,&context()).unwrap();
    let took = started.elapsed();
    eprintln!("{} faces, {} edges: sound in {took:?}",b.faces.len(),b.edges.len());
    // (most of it the volume, `props::fluxes` read along the traced curves)
    assert!(took.as_secs_f64() < 5.,"{took:?}");
}

/// **Every corpus solid the kernel builds exactly, held sound**: each static object of every
/// example whose exact B-rep is built (`Sketch::exact_solid`), checked and read by `sound`; those
/// refused, and the slowest read, said. A body of several lumps (a ball set, a row of caps) is no
/// manifold solid, which a file writes one lump to: those are refused as a second solid, and
/// counted apart. On 2026-11-21, 22 of 406 were refused, every one a body of several lumps. A
/// report, ignored (minutes):
/// `cargo test sound_census -- --ignored --nocapture`.
#[test]
#[ignore = "a tool, minutes: every corpus solid's exact B-rep held sound"]
fn sound_census() {
    let (mut read,mut lumps,mut refused,mut slowest) = (0,0,Vec::new(),(std::time::Duration::ZERO,String::new()));
    for (name,e) in fixtures::examples() {
        let Some(e) = e else { continue };
        if !e.ok() { continue }
        let mut solved = e.sketch.clone();
        gcs_core::solve::solve(&mut solved,gcs_core::solve::SolveOpts::default());
        for i in gcs_core::overview::objects(&solved) {
            if solved.is_swept(i) { continue }
            let Ok(x) = solved.exact_solid(i) else { continue };
            if x.brep.check(1e-7).is_err() { continue }
            read += 1;
            let started = std::time::Instant::now();
            let said = sound(&x.brep,&context());
            let took = started.elapsed();
            let label = format!("{name} `{}` ({} faces)",solved.solid_name(i),x.brep.faces.len());
            if took > slowest.0 { slowest = (took,label.clone()); }
            match said {
                Err(u) if u.msg.contains("a second solid") => lumps += 1,
                Err(u) => { eprintln!("REFUSED {label}: {u:?}"); refused.push(label); }
                Ok(()) => {}
            }
        }
    }
    eprintln!("{} of {read} refused, besides {lumps} bodies of several lumps; slowest {:?}: {}",refused.len(),slowest.0,slowest.1);
    assert!(refused.is_empty());
}

