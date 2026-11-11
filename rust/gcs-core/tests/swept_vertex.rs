//! **A vertex of a prism, where three or more faces meet** (#721 G1): a prism's corner, rim or
//! crease is walked along its fan, its root running along its ruling as the fan's normal turns,
//! until it reaches an end of the ruling, a vertex of the prism. Every normal of the cone between
//! the faces meeting there is the tool's, and those square to the motion lie on one arc of that
//! cone, from the fan the walk came along to the other fan bounding the cone that the arc reaches:
//! all stand at the vertex, so the walk goes on along that fan from the station whose root is the
//! vertex, the sweep with a sharp edge along the vertex's path and the sheet creased there
//! (`Junction::Vertex`, `Crease::Vertex`, `Characteristic::vertices`). Held here in the kernel alone,
//! under turns, screws and slides, for a bar whose corner runs to its end and a bar drafted both ways
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
use gcs_core::space::{add,cross,distance,scale};

use crate::prism_twists::{Bar,cornered,flat_end};
use crate::solid_sweeps_examples::class;
use crate::swept_apex::{off_contact,sector};
use crate::turns_and_slides::{exported,near};

type V = [f64;3];

use std::f64::consts::PI;

/// `bar`'s B-rep, built in the kernel from its section and depth, its side from its last corner to
/// its first bulging out to an arc about `bulge` (in the section's coordinates) where one is given.
fn brep(bar: &Bar,bulge: Option<[f64;2]>) -> Brep {
    let lift = |[x,y]: [f64;2]| -> V { add(bar.origin,add(scale(bar.across,x),scale(bar.along,y))) };
    let (corners,n) = (&bar.corners,bar.corners.len());
    let mut edges: Vec<ProfileEdge> = (0..n-1).map(|k| ProfileEdge::Line {a:lift(corners[k]),b:lift(corners[k+1])}).collect();
    let normal = cross(bar.across,bar.along);
    edges.push(match bulge {
        None => ProfileEdge::Line {a:lift(corners[n-1]),b:lift(corners[0])},
        Some(c) => {
            let (a,b) = (corners[n-1],corners[0]);
            let angle = |p: [f64;2]| (p[1]-c[1]).atan2(p[0]-c[0]);
            let (a0,mut a1) = (angle(a),angle(b));
            if a1 < a0 { a1 += 2.*PI }
            ProfileEdge::Arc {frame:Frame::new(lift(c),normal,bar.across),r:(a[0]-c[0]).hypot(a[1]-c[1]),span:Some([a0,a1])}
        }
    });
    let profile = Profile {origin:bar.origin,normal,loops:vec![edges],names:Vec::new()};
    drafted(&profile,bar.from,bar.to,bar.draft.to_radians()).unwrap()
}

/// `prism_twists::flat_end`'s bar, 20° from upright, its section the 3 × 4 rectangle turned `beta`
/// degrees in its plane: turned so far that a corner of its section stands nearest the axis, that
/// corner's line is the characteristic within the block, and its root runs along it to the bar's
/// far end. Where `bulging`, its inner side, from its last corner to its first, bulges out to an arc
/// about the point 6 within it.
fn turned_end(beta: f64,bulging: bool) -> Brep {
    let bar = flat_end(20.,beta);
    let [c0,c1,_,c3] = [0,1,2,3].map(|k| bar.corners[k]);
    // (the inner side's middle, 6 in along the lower side's way)
    let centre = std::array::from_fn(|i| 0.5*(c0[i]+c3[i])+2.*(c1[i]-c0[i]));
    brep(&bar,bulging.then_some(centre))
}

/// A turn about z, and a screw about z climbing 8 a turn, over the rolls `Bar` turns and screws
/// them: from clear of the sector to clear of it.
fn motions() -> [(Screw,[f64;2]);2] {
    let (turn,screw) = (140_f64.to_radians(),280_f64.to_radians());
    [(Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:1.,advance:0.},[-turn,turn]),
     (Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:1.,advance:8.},[-screw,screw])]
}

/// The walk of the prism `b` under `screw` over `roll` through the blank `inside`, read as the
/// solver's admission reads it: the tool from its B-rep and, standing alone, as a lone prism, its
/// caps walked.
fn walk(b: &Brep,screw: Screw,roll: [f64;2],inside: &(dyn Fn(V) -> bool+Sync)) -> (Tool,Result<Characteristic,(Row,String)>) {
    let tool = Tool::read(b,1.).unwrap();
    let lone = Lone::read(b,&tool,1.);
    assert!(lone.is_some(),"a lone prism");
    let path = Trajectory::Twist(screw);
    let sweep = Generating {tool:&tool,path:&path,roll,whole:false,mm:1.};
    let found = check_screw(&sweep,inside,None,&Sampling::default(),lone.as_ref()).map(|f| f.0).map_err(|(row,m,_)| (row,m));
    (tool,found)
}

/// Every node of `found` in contact with its normal outward (`swept_apex::off_contact`), and each
/// crease at a vertex of the prism `b`: how far the worst node is off contact (as `n·v` over `|v|`),
/// and the nodes at a vertex.
fn held(found: &Characteristic,tool: &Tool,b: &Brep,screw: &Screw) -> (f64,Vec<usize>) {
    let worst = off_contact(found,tool,screw);
    let corner = |p: V| b.vertices.iter().any(|q| distance(p,q.p) <= 1e-9);
    let at: Vec<usize> = (0..found.nodes.len()).filter(|&w| corner(found.nodes[w].position)).collect();
    let creases: Vec<usize> = found.creases().into_iter().map(|s| s as usize).collect();
    assert_eq!(creases,at,"the sheet creased at the vertices the walk goes round, and nowhere else");
    (worst,at)
}

/// The bar `b` under `screw` over `roll` through the blank `inside`: walked round a vertex, every
/// node in contact, the sheet creased at the vertex and nowhere else, and (where `both`) the walk
/// reaching the blank on both sides of one.
fn walked_round(b: &Brep,named: &str,(screw,roll): (Screw,[f64;2]),inside: &(dyn Fn(V) -> bool+Sync),both: bool) {
    let (tool,found) = walk(b,screw,roll,inside);
    let found = found.unwrap_or_else(|e| panic!("{named} under {}: {e:?}",screw.kind()));
    let (worst,at) = held(&found,&tool,b,&screw);
    eprintln!("{named} under {}: {} nodes, at a vertex {at:?}, reach {:?}, fans {:?}, {worst:.2e} off contact",screw.kind(),
        found.nodes.len(),found.reach,found.fans());
    assert!(!found.vertices().is_empty() && !at.is_empty(),"the walk goes round a vertex");
    assert!(worst <= 1e-9,"a node {worst:.2e} off contact");
    let [first,last] = found.reach;
    assert!(!both || at.iter().any(|&w| first < w && w < last),
        "the walk reaches the blank on both sides of a vertex: {at:?} in {first}..{last}");
}

/// Each bar under each motion through the sector, walked round a vertex (`walked_round`).
fn round_its_vertex(b: &Brep,named: &str) { for motion in motions() { walked_round(b,named,motion,&sector,true) } }

/// **A bar's corner running to its far end**: `flat_end`'s bar turned so a corner stands nearest the
/// axis (its section turned 30°, 45° or 60° one way, or 30° the other), turned across the sector
/// and screwed across it. The corner's root runs out along its ruling to the far end, a vertex
/// where the corner meets the end's two rims, and the walk goes on along the rim whose root leaves
/// it there.
#[test]
fn a_corner_running_to_a_bars_end_is_walked_round_the_vertex() {
    for beta in [30.,45.,60.,-30.] { round_its_vertex(&turned_end(beta,false),&format!("the bar turned {beta}°")); }
}

/// **A corner running to a crease**: the bar drafted both ways from its plane, its inner corner's
/// line reaching the plane at the vertex where the corner meets the crease, four faces meeting
/// there: the walk goes on from the corner's half below onto the fan beyond.
#[test]
fn a_corner_running_to_its_crease_is_walked_round_the_vertex() { round_its_vertex(&brep(&cornered(0.),None),"the bar drafted both ways") }

/// **A vertex one of whose edges is no fan the walk reads still ends it** (`VERTEX`): the bar's
/// inner side bulging out to an arc, its rim on the far end no line, so the vertex where the corner
/// meets the end has a cone of normals the walk does not bound by its fans alone. Turned, the
/// corner's root runs into it. (Screwed, the round side's own root runs over the end first, where
/// it has no rim to cross: `OVER_CAP`.)
#[test]
fn a_vertex_with_a_round_sides_rim_ends_the_walk() {
    let [(turn,roll),(screw,roll_screwed)] = motions();
    let refused = |screw: Screw,roll: [f64;2]| -> String {
        let (_,found) = walk(&turned_end(30.,true),screw,roll,&sector);
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
/// flat end, its section turned 30° and −30° (where the walk goes round two vertices), screwed across
/// the ring sector at 8 a turn: the constant twist's, the block losing what `Bar::lost` reads off the
/// bar apart from the code under test, each STL agreeing with the field. (Turned, it is
/// `solid_sweeps/square_end.sv`. The fitted sheets come out a few millionths low, as every turned or
/// screwed bar's do: #721 F4.)
#[test]
fn a_bars_corner_running_to_its_end_screwed_cuts_as_the_bar() {
    for beta in [30.,-30.] {
        let bar = Bar {lead:8.,..flat_end(20.,beta)};
        let e = fixtures::read(&bar.text());
        let Class::ConstantTwist(found) = class(&e,"part") else { panic!("the constant-twist class") };
        assert!(!found.vertices().is_empty(),"round a vertex");
        near(exported(&e,"part"),PI/3.*80.-bar.lost(),2e-5);
    }
}

/// A finite diamond bar slid obliquely across a cube: the contact on its bottom rib
/// reaches the cap at a vertex and continues along a rim. Both are whole edges in
/// contact, and each transition is a crease of the sheet, independently of its pace.
#[test]
fn a_slide_walks_from_a_silhouette_rib_onto_a_cap_rim() {
    let bar = Bar {origin:[0.,-4.,9.8],across:[0.,1.,0.],along:[0.,0.,1.],
        corners:vec![[0.,0.],[2.,2.],[0.,4.],[-2.,2.]],names:vec!["a","b","c","d"],
        from:2.,to:8.,draft:0.,lead:0.};
    let b = brep(&bar,None);
    let inside = |p: V| p.iter().all(|&x| x > 0. && x < 10.);
    for drop in [-0.1_f64,-0.2,-0.3] { for pace in [1.,2.] {
        let length = (0.09+1.+drop*drop).sqrt();
        let screw = Screw {origin:[0.;3],axis:[0.3/length,1./length,drop/length],ratio:0.,advance:18.*length*pace};
        walked_round(&b,&format!("drop {drop}, pace {pace}"),(screw,[0.,2.*PI/pace]),&inside,true);
    } }
}

/// The cap's contact line passes exactly through an inner profile corner. Its end
/// names that corner in the side chart, so it crosses the same vertex junction as a
/// rib rather than choosing a rim solely by the coincidence of its cap normal.
#[test]
fn a_caps_contact_line_leaves_at_a_profile_vertex() {
    let mut bar = flat_end(20.,30.);
    bar.origin[1] = -bar.corners[3][1];
    let b = brep(&bar,None);
    walked_round(&b,"cap line at a vertex",motions()[0],&sector,false);
    let e = fixtures::read(&bar.text());
    assert!(matches!(class(&e,"part"),Class::ConstantTwist(_)));
    near(exported(&e,"part"),PI/3.*80.-bar.lost(),2e-5);
}

/// A screw along the prism's extrusion carries whole silhouette rulings, like a
/// slide, but its cap rims have a root that moves along the edge as the normal turns.
#[test]
fn a_screw_along_the_extrusion_connects_its_silhouette_to_the_cap() {
    let bar = Bar {lead:8.,..flat_end(0.,30.)};
    let b = brep(&bar,None);
    walked_round(&b,"axial screw",motions()[1],&sector,false);
}

/// A horizontal bar's upper side stays in its plane. Its contact line ends at
/// profile vertices, where the boundary continues along a cap rim. The lower
/// side is below the blank, so the reaching characteristic is an open stretch.
#[test]
fn a_flats_line_leaves_through_a_vertex() {
    let bar = Bar {origin:[9.,1.,0.],across:[0.,1.,0.],along:[0.,0.,1.],
        corners:vec![[-1.,-3.],[2.,-3.],[1.,0.5],[0.,0.5]],names:vec!["a","b","c","d"],
        from:0.,to:2.,draft:0.,lead:0.};
    for reverse in [false,true] { for pace in [1.,2.,-1.] {
        let mut bar = Bar {corners:bar.corners.clone(),names:bar.names.clone(),..bar};
        if reverse { bar.corners.reverse(); }
        let (mut screw,roll) = motions()[0];
        screw.ratio *= pace;
        let roll = if pace > 0. { roll.map(|t| t/pace) } else { roll };
        walked_round(&brep(&bar,None),&format!("flat side, reverse {reverse}, pace {pace}"),(screw,roll),&sector,true);
    } }
    let e = fixtures::read(&bar.text());
    let Class::ConstantTwist(found) = class(&e,"part") else { panic!("the constant-twist class") };
    assert_eq!(found.vertices().len(),2);
    assert_eq!(found.flats().len(),1);
    // At height z, the bar runs from y = a = (z+3)/3.5 to 3-a.
    // Its turn removes r² from 9²+a² to 11²+(3-a)², over z=-1..0.5
    // and 60°: π/6 ∫ (49-6(z+3)/3.5) dz = 155π/14.
    // The sloping sides keep this out of the planar subtraction class.
    // `exported` also asks
    // solid::agreement of the STL, with zero field disagreements.
    near(exported(&e,"part"),80.*PI/3.-155.*PI/14.,1e-5);
}

/// The flat's omitted fans do not complete a concave vertex's normal cone.
#[test]
fn a_flat_at_a_concave_vertex_still_ends_the_walk() {
    let bar = Bar {origin:[9.,1.,0.],across:[0.,1.,0.],along:[0.,0.,1.],
        corners:vec![[0.,-3.],[2.,-3.],[2.,0.5],[1.,0.5],[1.,0.],[0.,0.]],
        names:vec!["a","b","c","d","e","f"],from:0.,to:2.,draft:0.,lead:0.};
    let (screw,roll) = motions()[0];
    let (_,found) = walk(&brep(&bar,None),screw,roll,&sector);
    let Err((row,message)) = found else { panic!("a concave vertex is still refused") };
    assert_eq!(row,Row::Regular);
    assert!(message.contains("runs off the flat face's line") || message.contains("runs into a vertex of the prism"),"{message}");
}

/// A shorter lead really crosses its section: changing the finite sheet's chart must not
/// bypass S4's admission gate (#721 F4).
#[test]
fn a_short_screw_whose_section_crosses_still_fails_s4() {
    let bar = Bar {lead:0.5,..flat_end(40.,5.)};
    let screw = Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:1.,advance:bar.lead};
    let (_,found) = walk(&brep(&bar,None),screw,[-140_f64.to_radians(),140_f64.to_radians()],&sector);
    let (row,why) = match found { Err(r) => r,Ok(_) => panic!("a crossing section was admitted") };
    assert_eq!(row,Row::Lead,"{why}");
    assert!(why.contains("crosses itself"),"{why}");
}
