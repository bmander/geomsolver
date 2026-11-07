//! **An apex on the tool's axis** (#721 G5): a face of revolution meeting its own axis at an angle,
//! a cone to a point (an engraving bit, a countersink) or an arc to one (an ogive), is walked down
//! one root of its rings to the apex and back up the other (`Junction::Apex`). Every normal of the
//! cone round the apex is the tool's there, and the walk turns at it from one root's normal to the
//! other's: the sweep has a sharp edge along the apex's path, the sheet creased there
//! (`Node::apex`, `Characteristic::apexes`). Held here in the kernel alone, under every constant
//! twist and for either end of a meridian: each node of the walk a point of the tool, in contact
//! (`n·v = 0`), its normal the tool's outward there, and the apex among them, once, the walk passing
//! through the blank on both of its sides.
use gcs_core::brep::build::{revolve,Profile,ProfileEdge};
use gcs_core::brep::geom::Frame;
use gcs_core::brep::swept::admission::{check_screw,Generating,Sampling};
use gcs_core::brep::swept::constant_twist::Characteristic;
use gcs_core::brep::swept::tool::Tool;
use gcs_core::brep::topo::Brep;
use gcs_core::brep::trajectory::{Screw,Trajectory};
use gcs_core::space::{add,distance,dot,norm,scale};

type V = [f64;3];

use std::f64::consts::{PI,TAU};

/// How a pointed tool's point is drawn: a cone of this half-angle (radians), or an arc meeting the
/// shank smoothly at this radius, its tip where it meets the axis.
#[derive(Clone,Copy,Debug)]
enum Point { Cone(f64),Ogive }

/// A pointed tool along `axis` from its apex at `tip`, the shank of radius 3 reaching 6 past the
/// point: its meridian laid in the half-plane through the axis toward `out` (square to it), drawn
/// from the apex (`first`) or toward it.
fn pointed(tip: V,axis: V,out: V,point: Point,apex_first: bool) -> Brep {
    let at = |a: f64,r: f64| -> V { add(add(tip,scale(axis,a)),scale(out,r)) };
    let normal = gcs_core::space::cross(axis,out);
    let shank = 3.;
    let (lip,edge) = match point {
        Point::Cone(half) => { let lip = shank/half.tan(); (lip,ProfileEdge::Line {a:at(0.,0.),b:at(lip,shank)}) }
        // an arc tangent to the shank at the lip, `lip` along: its centre level with the lip, the
        // radius `(shank² + lip²) / 2 shank` reaching the axis at the tip
        Point::Ogive => {
            let lip = 5.;
            let r = (shank*shank+lip*lip)/(2.*shank);
            let centre = at(lip,shank-r);
            let reach = (lip/r).asin();
            (lip,ProfileEdge::Arc {frame:Frame::new(centre,normal,out),r,span:Some([0.,reach])})
        }
    };
    let back = lip+6.;
    let mut edges = vec![edge,ProfileEdge::Line {a:at(lip,shank),b:at(back,shank)},ProfileEdge::Line {a:at(back,shank),b:at(back,0.)},
        ProfileEdge::Line {a:at(back,0.),b:at(0.,0.)}];
    if !apex_first { edges.reverse(); }
    let profile = Profile {origin:at(lip,1.),normal,loops:vec![edges],names:Vec::new()};
    revolve(&profile,tip,axis,std::f64::consts::TAU).unwrap()
}

/// The walk of `tool` along `path` over `roll` through the blank `inside`, which the solver's
/// admission asks the same way.
fn walked(tool: &Brep,path: &Trajectory,roll: [f64;2],inside: &(dyn Fn(V) -> bool+Sync)) -> Characteristic {
    let tool = Tool::read(tool,1.).unwrap();
    let sweep = Generating {tool:&tool,path,roll,whole:false,mm:1.};
    check_screw(&sweep,inside,None,&Sampling::default(),None).unwrap_or_else(|e| panic!("the walk: {e:?}")).0
}

/// Every node of `walked` a point of `tool`'s boundary in contact under `screw`, its normal the
/// tool's outward one, and the apex `tip` among them where the walk turns there: how far the worst
/// node is off contact (as `n·v` over `|v|`), and how many apexes it turns at.
fn held(walked: &Characteristic,tool: &Brep,screw: &Screw,tip: V) -> (f64,usize) {
    let material = Tool::read(tool,1.).unwrap();
    let mut worst = 0_f64;
    for n in &walked.nodes {
        let v = screw.velocity(n.position);
        worst = worst.max(dot(n.normal,v).abs()/norm(v));
        assert!(material.on(n.position),"a node off the tool: {:?}",n.position);
        // the material lies behind the normal
        let probe = |k: f64| material.material().value(add(n.position,scale(n.normal,k*1e-4)));
        assert!(probe(1.) > probe(-1.),"a node's normal not outward at {:?}: {:?} {} {}",n.position,n.normal,probe(1.),probe(-1.));
    }
    let apexes = walked.nodes.iter().filter(|n| distance(n.position,tip) <= 1e-9).count();
    assert_eq!(walked.apexes().len(),apexes.min(1),"the apex the walk turns at, named: {:?}",walked.apexes());
    (worst,apexes)
}


/// Each pointed tool walked under `screw` over `roll` through `inside`: every node in contact, the
/// walk turning at the apex once and reaching the blank on both sides of it.
fn through_its_apex(tip: V,axis: V,out: V,screw: Screw,roll: [f64;2],inside: &(dyn Fn(V) -> bool+Sync)) {
    let path = Trajectory::Twist(screw);
    for point in [Point::Cone(30_f64.to_radians()),Point::Cone(60_f64.to_radians()),Point::Ogive] { for first in [true,false] {
        let tool = pointed(tip,axis,out,point,first);
        let found = walked(&tool,&path,roll,inside);
        let (worst,apexes) = held(&found,&tool,&screw,tip);
        eprintln!("{point:?} (apex first {first}): {} nodes, {apexes} at the apex, {worst:.2e} off contact",found.nodes.len());
        assert_eq!(apexes,1,"the walk turns at the apex once");
        assert!(worst <= 1e-9,"a node {worst:.2e} off contact");
        let w = found.nodes.iter().position(|n| distance(n.position,tip) <= 1e-9).unwrap();
        assert!(found.nodes[..w].iter().any(|n| n.reach.is_some()) && found.nodes[w+1..].iter().any(|n| n.reach.is_some()),
            "the walk reaches the blank on both sides of the apex");
        // the sheet is creased at the apex, and nowhere else
        assert_eq!(found.creases(),[w as f64]);
    } }
}

/// The rod of radius 5 about z from 0 to 20.
fn rod(p: V) -> bool { p[0].hypot(p[1]) < 5. && p[2] > 0. && p[2] < 20. }

/// **A point screwed along a rod**: the tool lying along +x, its apex 1 into the rod, screwed about
/// the rod's axis at a lead of 6 from below the rod to above, as a thread is chased with a pointed
/// tool. Each ring of the point near its apex has two roots, either side of the plane through the
/// rod's axis and the tool's, which meet at the apex; the walk runs in from the rod's surface along
/// one, turns at the apex and runs back out along the other. A cone or an ogive (a spindle torus's
/// lemon), its meridian drawn from the apex or toward it.
#[test]
fn a_point_screwed_along_a_rod_is_walked_through_its_apex() {
    let screw = Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:1.,advance:6.};
    through_its_apex([4.,0.,0.],[1.,0.,0.],[0.,0.,1.],screw,[-TAU*4./6.,TAU*24./6.],&rod);
}

/// **A point turned across a ring**: the tool pointing down, leaning 25° across a turn about z, its
/// apex half a millimetre into the top of a ring sector 10 out, turned from clear of the sector to
/// clear of it: the walk turns at the apex, where the characteristic laid in a half-plane through
/// the axis has a corner.
#[test]
fn a_point_turned_across_a_ring_is_walked_through_its_apex() {
    let lean = 25_f64.to_radians();
    let screw = Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:1.,advance:0.};
    let sector = |p: V| { let (r,a) = (p[0].hypot(p[1]),p[1].atan2(p[0])); r > 8. && r < 12. && p[2].abs() < 1. && a > 0. && a < PI/3. };
    through_its_apex([10.,0.,0.5],[0.,lean.sin(),lean.cos()],[1.,0.,0.],screw,[-140_f64.to_radians(),140_f64.to_radians()],&sector);
}

/// **A point slid leaning along its slide**: the tool pointing down, leaning 20° toward where it
/// goes, its apex a millimetre into a block's top, slid along x through it: on a cone the
/// characteristic is two rulings, the walk up one, round the apex and down the other.
#[test]
fn a_point_slid_raked_along_its_slide_is_walked_through_its_apex() {
    let lean = 20_f64.to_radians();
    let screw = Screw {origin:[0.;3],axis:[1.,0.,0.],ratio:0.,advance:36.};
    let block = |p: V| p[0] > 0. && p[0] < 20. && p[1].abs() < 5. && p[2] > 0. && p[2] < 10.;
    through_its_apex([-4.,0.,9.],[lean.sin(),0.,lean.cos()],[0.,1.,0.],screw,[0.,TAU],&block);
}

/// **A point of the surface nearest one behind a cone's apex, or by a spindle torus's lemon, is read
/// across the axis** (`Surface::inverse`): a cone's generator runs on past its apex into the other
/// nappe, and a spindle torus's meridian circle crosses its axis, so the point nearest is on the
/// meridian in the opposite half-plane where that is the nearer. Read in its own half-plane alone, a
/// point behind a pointed tool's apex fell on the generator past the apex, which the face's
/// parameters then took for the face: the tool's material read it inside, out to a millimetre behind
/// an ogive's point (#721 G5). And near a degenerate edge, an apex, every edge of the face meets in
/// space, so the side of the nearest is no reading of the face: a point of the other nappe just past
/// the apex was read beside the cone's seam and in the face (`Located::face_place`). Now every point
/// of each surface inverts to itself, every point past the apex is off the face, and the tool's
/// material reads behind its point as outside, the distance to the apex.
#[test]
fn behind_a_point_is_outside_the_tool() {
    use gcs_core::brep::geom::Surface;
    for point in [Point::Cone(30_f64.to_radians()),Point::Cone(60_f64.to_radians()),Point::Ogive] {
        let tool = pointed([4.,0.,0.],[1.,0.,0.],[0.,0.,1.],point,true);
        for f in &tool.faces {
            if !matches!(f.surface,Surface::Cone(..) | Surface::Torus(..)) { continue }
            // its points on both sides of the axis, and past the apex, each its own foot
            for (u,v) in [(0.3,0.5),(3.5,0.2),(1.,-0.4),(5.,-2.),(2.,1.5)] {
                let p = f.surface.point([u,v]);
                let q = f.surface.point(f.surface.inverse(p));
                assert!(distance(p,q) <= 1e-12*(1.+norm(p)),"{point:?}: {p:?} inverts to {q:?}");
            }
        }
        // a point of the other nappe past the apex, however near it, is off the face: near a
        // degenerate edge its parameters decide, not the side of whichever edge is nearest in space
        let located = gcs_core::brep::query::Located::new(&tool,1e-9);
        for (fi,f) in tool.faces.iter().enumerate() {
            let Surface::Cone(..) = f.surface else { continue };
            for v in [-1e-5,-1e-3,-0.1] { for u in [0.5,2.,4.,6.] {
                assert_eq!(located.face_place(fi,f.surface.point([u,v])),gcs_core::brep::query::Place::Out,"{point:?}: ({u}, {v})");
            } }
        }
        let material = Tool::read(&tool,1.).unwrap();
        // (past the material's tolerance of the apex, within which it is the apex)
        for k in [1e-4,1e-3,1e-2,0.1,0.5,1.,2.,3.5] {
            let behind = add([4.,0.,0.],scale([-1.,0.,0.],k));
            let value = material.material().value(behind);
            assert!((value-k).abs() <= 1e-9*(1.+k),"{point:?}: {k} behind the apex reads {value}");
        }
    }
}

/// **The drawing's field reads behind an ogive's point as outside** (`profile::convex`): a convex
/// meridian's arc bounds only its angular sector, and where it reaches the axis about a centre off it
/// nothing bounded the wedge between its radius at the tip and the axis, which is no wall: the field
/// read material behind the tip, which a sweep carried through a rod's core. The arc's tangent there
/// bounds it, a supporting line of the convex loop that moves no boundary (#721 G5).
#[test]
fn the_drawings_field_reads_behind_an_ogives_point_as_outside() {
    use gcs_core::solid::MaterialField;
    let text = crate::solid_sweeps_examples::text("chased_point.sv").replace("  private cone := line(tip, lip)",
        "  private c := point\n  c coords(du: shank - 34mm / 6, dv: 5mm) bench\n  private cone := arc(center: c, start: tip, end: lip) hint(r: 5.6667)")
        .replace("lip coords(du: shank, dv: shank / tan(half)) bench","lip coords(du: shank, dv: 5mm) bench")
        .replace("reach + shank / tan(half)","reach + 5mm");
    let e = fixtures::read(&text);
    let mut m = MaterialField::read(&e.sketch,fixtures::solid(&e,"vbit"),1e-10).unwrap().evaluator(4096);
    let read = |m: &mut gcs_core::solid::MaterialEvaluator,p: V| m.bounds(p.map(|x| gcs_core::interval::Interval::point(x).unwrap()),
        gcs_core::interval::minimum::Options {value_tolerance:1e-9,max_evaluations:50000}).unwrap().value.bounds();
    // within, a tenth in front of the tip: as far from the arc as the tangent there leans
    let [lo,hi] = read(&mut m,[4.1,0.,0.]);
    assert!(hi < 0.,"in front of the tip, within: [{lo}, {hi}]");
    for k in [0.1,0.5,1.,3.] {
        let [lo,_] = read(&mut m,[4.-k,0.,0.]);
        assert!(lo > 0.,"{k} behind the tip, outside: {lo}");
        let [lo,_] = read(&mut m,[4.-k,0.3*k,0.]);
        assert!(lo > 0.,"{k} behind the tip and aside, outside: {lo}");
    }
}

