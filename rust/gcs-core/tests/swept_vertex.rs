//! **A vertex of a prism, where three or more faces meet** (#721 G1): a prism's corner, rim or
//! crease is walked along its fan, its root running along its ruling as the fan's normal turns,
//! until it reaches an end of the ruling, a vertex of the prism. Every normal of the cone between
//! the faces meeting there is the tool's, and those square to the motion lie on one arc of that
//! cone, from the fan the walk came along to the other fan bounding the cone that the arc reaches:
//! all stand at the vertex, so the walk goes on along that fan from the station whose root is the
//! vertex, the sweep with a sharp edge along the vertex's path and the sheet creased there
//! (`Along::vertices`, `Node::vertex`, `Characteristic::vertices`). Held here in the kernel alone,
//! under a turn and a screw, for a bar whose corner runs to its end and a bar drafted both ways
//! whose corner runs to its crease: each node of the walk a point of the tool, in contact
//! (`n·v = 0`), its normal the tool's outward there, and every crease at a vertex of the prism, the
//! walk reaching the blank on both sides of one. A vertex one of whose edges is no fan the walk
//! reads, a round side's rim, still ends the walk (`VERTEX`). And drawn, each bar's cut is held to
//! what its half-spaces sweep (`prism_twists::Bar`), its STL to the field.
use gcs_core::brep::build::{drafted,Profile,ProfileEdge};
use gcs_core::brep::geom::Frame;
use gcs_core::brep::swept::admission::{check_screw,Generating,Lone,Sampling};
use gcs_core::brep::swept::constant_twist::{Characteristic,Row};
use gcs_core::brep::swept::tool::Tool;
use gcs_core::brep::topo::Brep;
use gcs_core::brep::trajectory::{Screw,Trajectory};
use gcs_core::solid::admission::Class;
use gcs_core::space::{add,cross,distance,dot,norm,scale};

use crate::prism_twists::Bar;
use crate::solid_sweeps_examples::class;
use crate::turns_and_slides::{exported,near};

type V = [f64;3];

use std::f64::consts::PI;

/// A bar drawn in the plane through `origin` along `across` and `along`, its section `corners` (in
/// those coordinates, counter-clockwise), extruded along `across × along` from `from` to `to`,
/// drafted `draft` degrees; its side from the last corner to the first bulging out to an arc about
/// `bulge` (in the same coordinates) where one is given.
fn bar(origin: V,across: V,along: V,corners: &[[f64;2]],from: f64,to: f64,draft: f64,bulge: Option<[f64;2]>) -> Brep {
    let lift = |[x,y]: [f64;2]| -> V { add(origin,add(scale(across,x),scale(along,y))) };
    let n = corners.len();
    let mut edges: Vec<ProfileEdge> = (0..n-1).map(|k| ProfileEdge::Line {a:lift(corners[k]),b:lift(corners[k+1])}).collect();
    let normal = cross(across,along);
    edges.push(match bulge {
        None => ProfileEdge::Line {a:lift(corners[n-1]),b:lift(corners[0])},
        Some(c) => {
            let (a,b) = (corners[n-1],corners[0]);
            let angle = |p: [f64;2]| (p[1]-c[1]).atan2(p[0]-c[0]);
            let (a0,mut a1) = (angle(a),angle(b));
            if a1 < a0 { a1 += 2.*PI }
            let r = (a[0]-c[0]).hypot(a[1]-c[1]);
            ProfileEdge::Arc {frame:Frame::new(lift(c),normal,across),r,span:Some([a0,a1])}
        }
    });
    let profile = Profile {origin,normal,loops:vec![edges],names:Vec::new()};
    drafted(&profile,from,to,draft.to_radians()).unwrap()
}

/// The point `[u, v]` turned `beta` degrees about the origin of its plane.
fn turned(beta: f64,[u,v]: [f64;2]) -> [f64;2] { let (s,c) = beta.to_radians().sin_cos(); [c*u-s*v,s*u+c*v] }

/// `prism_twists::flat_end`'s bar: standing up and outward from its flat end, 20° from upright,
/// the end facing down toward the axis, 10 from it, its section the 3 × 4 rectangle turned `beta`
/// degrees in its plane, 3 long. Turned so far that a corner of its section stands nearest the
/// axis, that corner's line is the characteristic within the block, and its root runs along it
/// to the bar's far end. Where `bulging`, its inner side bulges out to an arc about the point 6
/// within it.
fn flat_end(beta: f64,bulging: bool) -> Brep {
    let (s,c) = 20_f64.to_radians().sin_cos();
    let corners = [[0.,-2.],[3.,-2.],[3.,2.],[0.,2.]].map(|p| turned(beta,p));
    bar([10.,0.,0.],[c,0.,-s],[0.,1.,0.],&corners,0.,3.,0.,bulging.then(|| turned(beta,[6.,0.])))
}

/// `prism_twists`'s bar stood on its inner corner, 10 from the axis, its section a square of side
/// `1.5 √2` leaning 20° across the turn, standing ±4 either side of the plane through x its section
/// is drawn in and drafted 2° both ways from it: its corner's line reaches that plane at the vertex
/// where the corner meets the crease.
fn creased() -> Brep {
    let (s,c) = 20_f64.to_radians().sin_cos();
    bar([0.;3],[1.,0.,0.],[0.,c,-s],&[[10.,0.],[11.5,1.5],[13.,0.],[11.5,-1.5]],-4.,4.,2.,None)
}

/// The ring sector of `turn.sv`: radii 8 and 12, heights ±1, 60° round from x.
fn sector(p: V) -> bool { let (r,a) = (p[0].hypot(p[1]),p[1].atan2(p[0])); r > 8. && r < 12. && p[2].abs() < 1. && a > 0. && a < PI/3. }

/// A turn about z, and a screw about z climbing 8 a turn, over the rolls `Bar` turns and screws
/// them: from clear of the sector to clear of it.
fn motions() -> [(Screw,[f64;2]);2] {
    let (turn,screw) = (140_f64.to_radians(),280_f64.to_radians());
    [(Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:1.,advance:0.},[-turn,turn]),
     (Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:1.,advance:8.},[-screw,screw])]
}

/// The walk of the prism `b` under `screw` over `roll` through the sector, read as the solver's
/// admission reads it: the tool from its B-rep and, standing alone, as a lone prism, its caps walked.
fn walk(b: &Brep,screw: Screw,roll: [f64;2]) -> (Tool,Result<Characteristic,(Row,String)>) {
    let tool = Tool::read(b,1.).unwrap();
    let lone = Lone::read(b,&tool,1.);
    assert!(lone.is_some(),"a lone prism");
    let path = Trajectory::Twist(screw);
    let sweep = Generating {tool:&tool,path:&path,roll,whole:false,mm:1.};
    let found = check_screw(&sweep,&sector,None,&Sampling::default(),lone.as_ref()).map(|f| f.0).map_err(|(row,m,_)| (row,m));
    (tool,found)
}

/// Every node of `found` a point of `tool`'s boundary in contact under `screw`, its normal the
/// tool's outward one, and each crease at a vertex of the prism `b`: how far the worst node is off
/// contact (as `n·v` over `|v|`), and the nodes at a vertex.
fn held(found: &Characteristic,tool: &Tool,b: &Brep,screw: &Screw) -> (f64,Vec<usize>) {
    let mut worst = 0_f64;
    for n in &found.nodes {
        let v = screw.velocity(n.position);
        worst = worst.max(dot(n.normal,v).abs()/norm(v));
        assert!(tool.on(n.position),"a node off the tool: {:?}",n.position);
        let probe = |k: f64| tool.material().value(add(n.position,scale(n.normal,k*1e-4)));
        assert!(probe(1.) > probe(-1.),"a node's normal not outward at {:?}: {:?}",n.position,n.normal);
    }
    let corner = |p: V| b.vertices.iter().any(|q| distance(p,q.p) <= 1e-9);
    let at: Vec<usize> = (0..found.nodes.len()).filter(|&w| corner(found.nodes[w].position)).collect();
    let creases: Vec<usize> = found.creases().into_iter().map(|s| s as usize).collect();
    assert_eq!(creases,at,"the sheet creased at the vertices the walk goes round, and nowhere else");
    (worst,at)
}

/// Each bar under each motion: walked round a vertex, every node in contact, the sheet creased at
/// the vertex and nowhere else, and the walk reaching the blank on both sides of one.
fn round_its_vertex(b: &Brep,named: &str) {
    for (screw,roll) in motions() {
        let (tool,found) = walk(b,screw,roll);
        let found = found.unwrap_or_else(|e| panic!("{named} under {}: {e:?}",screw.kind()));
        let (worst,at) = held(&found,&tool,b,&screw);
        eprintln!("{named} under {}: {} nodes, at a vertex {at:?}, reach {:?}, fans {:?}, {worst:.2e} off contact",screw.kind(),
            found.nodes.len(),found.reach,found.fans());
        assert!(!found.vertices().is_empty() && !at.is_empty(),"the walk goes round a vertex");
        assert!(worst <= 1e-9,"a node {worst:.2e} off contact");
        let [first,last] = found.reach;
        assert!(at.iter().any(|&w| first < w && w < last),"the walk reaches the blank on both sides of a vertex: {at:?} in {first}..{last}");
    }
}

/// **A bar's corner running to its far end**: `flat_end`'s bar turned so a corner stands nearest the
/// axis (its section turned 30°, 45° or 60° one way, or 30° the other), turned across the sector
/// and screwed across it. The corner's root runs out along its ruling to the far end, a vertex
/// where the corner meets the end's two rims, and the walk goes on along the rim whose root leaves
/// it there.
#[test]
fn a_corner_running_to_a_bars_end_is_walked_round_the_vertex() {
    for beta in [30.,45.,60.,-30.] { round_its_vertex(&flat_end(beta,false),&format!("the bar turned {beta}°")); }
}

/// **A corner running to a crease**: the bar drafted both ways from its plane, its inner corner's
/// line reaching the plane at the vertex where the corner meets the crease, four faces meeting
/// there: the walk goes on from the corner's half below onto the fan beyond.
#[test]
fn a_corner_running_to_its_crease_is_walked_round_the_vertex() { round_its_vertex(&creased(),"the bar drafted both ways") }

/// **A vertex one of whose edges is no fan the walk reads still ends it** (`VERTEX`): the bar's
/// inner side bulging out to an arc, its rim on the far end no line, so the vertex where the corner
/// meets the end has a cone of normals the walk does not bound by its fans alone. Turned, the
/// corner's root runs into it. (Screwed, the round side's own root runs over the end first, where
/// it has no rim to cross: `OVER_CAP`.)
#[test]
fn a_vertex_with_a_round_sides_rim_ends_the_walk() {
    let [(turn,roll),(screw,roll_screwed)] = motions();
    let refused = |screw: Screw,roll: [f64;2]| -> String {
        let (_,found) = walk(&flat_end(30.,true),screw,roll);
        let Err((row,message)) = found else { panic!("refused under {}",screw.kind()) };
        eprintln!("under {}: {row:?} {message}",screw.kind());
        assert_eq!(row,Row::Regular);
        message
    };
    let message = refused(turn,roll);
    assert!(message.contains("runs into a vertex of the prism"),"{message}");
    let message = refused(screw,roll_screwed);
    assert!(message.contains("runs over a cap of the prism"),"{message}");
}

/// **Drawn, a bar whose corner runs to its end cuts what its half-spaces sweep**: `prism_twists`'s
/// flat end, its section turned 30°, turned across the ring sector and screwed across it at 8 a
/// turn, and turned −30° and screwed (where the walk goes round two vertices): the constant twist's,
/// the block losing what `Bar::removed` and `Bar::screwed_removed` read off the bar apart from the
/// code under test, each STL agreeing with the field. (The fitted sheets come out a few millionths
/// low, as every turned or screwed bar's do: #721 F4.)
#[test]
fn a_bars_corner_running_to_its_end_cuts_as_the_bar() {
    for (beta,lead) in [(30.,0.),(30.,8.),(-30.,8.)] {
        let bar = Bar {lead,..crate::prism_twists::flat_end(20.,beta)};
        let e = fixtures::read(&bar.text());
        let Class::ConstantTwist(found) = class(&e,"part") else { panic!("the constant-twist class") };
        assert!(!found.vertices().is_empty(),"round a vertex");
        let removed = if lead == 0. { bar.removed() } else { bar.screwed_removed() };
        near(exported(&e,"part"),PI/3.*80.-removed,2e-5);
    }
}
