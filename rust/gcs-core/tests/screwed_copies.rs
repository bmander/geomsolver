//! **Copies of one screwed closed sweep, each cut at the cost of one** (#847): a 1 mm slab of a
//! 2 mm square, 10 mm off the axis, carried up a 50 mm-lead screw through a tube 12/9.5 × 10 mm
//! (`some t: p in slab at(under: screw) t`, the roll left to the body it cuts), cut once and turned
//! about the axis and cut again. Each cut had been rebuilt against the whole blank so far, its
//! earlier sheets met and split by every candidate, so each took about twice the one before (23,
//! 42 and 76 s); the tube kept 2 446 edges on 26 faces and 30 shorter than 10 µm. Now a cut splits
//! the blank only within the box holding its sweep (`boolean::arrangement_reached`), two meetings
//! lying along one helix are no crossing of each other (`boolean::along_one_another`), a sheet's
//! piece is boxed by what it reaches of the sheet, and the edges a healed meeting cut are one again
//! (`boolean::rejoined`): the copies remove exactly what the first does each, and the tube's edges
//! are its slots' corners alone.
use gcs_core::brep::build::{screwed,Profile,ProfileEdge};
use gcs_core::brep::geom::{Curve,Surface};
use gcs_core::brep::props::volume;
use gcs_core::brep::topo::{Brep,EdgeCurve};
use gcs_core::solid::admission::Class;
use gcs_core::space::distance;

use crate::solid_sweeps_examples::class;
use crate::turns_and_slides::exported_within;

type V = [f64;3];

/// The issue's tube and slab, the slab's sweep cut where it stands and at each of `angles` (degrees)
/// turned about the tube's axis.
fn tube(angles: &[f64]) -> String {
    let copies: String = angles.iter().enumerate().map(|(k,a)| format!("turned{k} := space at(under: index) {a}deg\nturned{k} cut part\n")).collect();
    format!("unit mm
use std
use std.shapes
use std.solids (cut)
use std.place (vertical, coords)
use std.motion (screws, turns)

in std.front {{
  construction spindle := vertical line(std.origin, hint((0, 10)))
  length(10mm) spindle
}}
in std.top {{
  a := point hint((10, 0))
  a coords(du: 10mm, dv: 0mm) std.top
  sq := std.shapes.CenteredRectangle(a, w: 2mm, h: 2mm)
  rim := radius(12mm) circle(center: std.top.origin)
  bore := radius(9.5mm) circle(center: std.top.origin)
}}
part := solid(std.solids.Prism(face(rim, holes: bore), from: 0mm, to: 10mm))
slab := solid(std.solids.Prism(face(sq.loop), from: -0.5mm, to: 0.5mm))
screw := frame
screw screws(50mm) spindle
index := frame
index turns(1) spindle
space := solid({{ p | some t: p in slab at(under: screw) t }})
space cut part
{copies}")
}

/// The tube with its slots: the exact build, its volume, and its edges' lengths.
fn slotted(angles: &[f64]) -> (Brep,f64) {
    let e = fixtures::read(&tube(angles));
    let Class::Closed(found) = class(&e,"part") else { panic!("the closed class") };
    assert!(found.screw().is_some_and(|s| !s.is_slide() && !s.is_turn()),"a screw");
    let solid = exported_within(&e,"part",None).solid;
    let v = volume(&solid);
    (solid,v)
}

/// An edge's length, along its curve.
fn length(b: &Brep,e: usize) -> f64 {
    let e = &b.edges[e];
    let EdgeCurve::Curve(c) = &e.curve else { return 0. };
    (0..32).map(|k| distance(c.point(e.t[0]+(e.t[1]-e.t[0])*k as f64/32.),c.point(e.t[0]+(e.t[1]-e.t[0])*(k+1) as f64/32.))).sum()
}

/// Every vertex where two edges alone meet: two pieces of one curve end to end there, which
/// `rejoined` makes one.
fn split_curves(b: &Brep) -> Vec<V> {
    (0..b.vertices.len()).filter_map(|v| {
        let at: Vec<&gcs_core::brep::topo::Edge> = b.edges.iter().filter(|e| e.v.contains(&(v as u32))).collect();
        let [x,y] = at[..] else { return None };
        let (EdgeCurve::Curve(cx),EdgeCurve::Curve(cy)) = (&x.curve,&y.curve) else { return None };
        (cx == cy && x.v[0] != x.v[1]).then_some(b.vertices[v].p)
    }).collect()
}

/// **One placement's slot leaves the tube's edges whole** (#847): the slot cut once, its faces those
/// of the slab's sweep and the tube's four — the edges the slot's corners alone, no piece of one
/// curve beside another where a healed meeting cut it, and none under the export's 10 µm (two
/// rulings had been cut 7 µm from their corners, the sheets' touch helix into 400 pieces).
#[test]
fn one_screwed_slot_leaves_the_tubes_edges_whole() {
    let (solid,_) = slotted(&[]);
    assert_eq!(solid.faces.len(),12,"the tube's four faces and the slot's eight");
    let short: Vec<f64> = (0..solid.edges.len()).map(|e| length(&solid,e)).filter(|&l| l < 0.01).collect();
    assert!(short.is_empty(),"edges under 10 µm: {short:?}");
    assert_eq!(split_curves(&solid),Vec::<V>::new(),"pieces of one curve left end to end");
    assert!(solid.edges.len() <= 40,"{} edges",solid.edges.len());
}

/// **A turned copy is cut as the first was, whatever the first left** (#847): the slot cut where it
/// stands and turned half a turn, the tube symmetric about its axis, so the two remove exactly twice
/// what one does, and the second adds its own slot's edges and no more — it splits the tube only
/// within the box holding its sweep, the first slot's faces and edges left whole.
#[test]
fn a_turned_copy_of_a_screwed_slot_is_cut_as_the_first() {
    let whole = std::f64::consts::PI*(144.-90.25)*10.;
    let (one,v1) = slotted(&[]);
    let (two,v2) = slotted(&[180.]);
    eprintln!("one slot: {} faces, {} edges; two: {} faces, {} edges",one.faces.len(),one.edges.len(),two.faces.len(),two.edges.len());
    assert!(((whole-v2)-2.*(whole-v1)).abs() <= 1e-9*whole,"two slots remove {} against twice {}",whole-v2,whole-v1);
    assert!(two.faces.len() <= 2*one.faces.len()-4,"{} faces against {} for one",two.faces.len(),one.faces.len());
    assert!(two.edges.len() <= 2*one.edges.len(),"{} edges against {} for one",two.edges.len(),one.edges.len());
    assert_eq!(split_curves(&two),Vec::<V>::new(),"pieces of one curve left end to end");
}

/// The sheet the level line through `p` along `e` screws into about the `z` axis over `angle` (a
/// rise of `advance`), from the line screwed back by `from` of that: a level triangle on the line
/// screwed (`build::screwed`), the sheet its face, the triangle's other faces left off.
fn sheet(p: V,e: V,reach: f64,angle: f64,advance: f64,from: f64) -> Brep {
    let away = [-e[1],e[0],0.];
    let corners = [[p[0]-reach*e[0],p[1]-reach*e[1],p[2]],[p[0]+reach*e[0],p[1]+reach*e[1],p[2]],
        [p[0]-reach*away[0],p[1]-reach*away[1],p[2]]];
    let triangle = Profile {origin:corners[0],normal:[0.,0.,1.],names:vec![vec!["sheet".into(),String::new(),String::new()]],
        loops:vec![(0..3).map(|k| ProfileEdge::Line {a:corners[k],b:corners[(k+1)%3]}).collect()]};
    let back = gcs_core::brep::geom::Rigid::turn([0.;3],[0.,0.,1.],-from*angle)
        .then(&gcs_core::brep::geom::Rigid {t:[0.,0.,-from*advance],..gcs_core::brep::geom::Rigid::identity()});
    let mut b = screwed(&triangle.moved(&back),[0.;3],[0.,0.,1.],angle,advance).unwrap();
    b.faces.retain(|f| f.name == "sheet");
    b.compacted()
}

/// **Two meetings lying along one helix are read as one curve** (#847, `boolean::along_one_another`,
/// which `boolean`'s triple points do not search for crossings): a level line's sheet touches the
/// cylinder its foot runs on along the foot's helix, and meets the sheet of the radial line through
/// a point of that helix higher up along the same helix — on the first sheet, two meetings that are
/// one curve, each of whose chords stood within reach of the other's, so that the search cut both at
/// every one (hundreds of vertices along one helix of the issue's slot). A parallel helix on the
/// sheet, and a ruling crossing the helix, are not that curve.
#[test]
fn two_meetings_lying_along_one_helix_are_read_as_one_curve() {
    use gcs_core::brep::boolean::along_one_another;
    use gcs_core::brep::geom::Frame;
    use gcs_core::brep::ssi::{intersect,Ssi};
    let (angle,advance,tol) = (1.5,1.2,1e-6);
    let reach = 128.*tol;
    // the line x = 2 at z = 0, its foot (2, 0, 0); and the radial line through that foot screwed
    // three tenths of the way on
    let touching = sheet([2.,0.,0.],[0.,1.,0.],6.,angle,advance,0.5).faces[0].surface.clone();
    let (s,c) = (0.3*angle).sin_cos();
    let radial = sheet([2.*c,2.*s,0.3*advance],[c,s,0.],6.,angle,advance,0.8).faces[0].surface.clone();
    let cylinder = |r: f64| Surface::Cylinder(Frame::about([0.;3],[0.,0.,1.]),r);
    let curves = |a: &Surface,b: &Surface| match intersect(a,b,tol) { Ssi::Curves(c) => c,other => panic!("{other:?}") };
    let [foot] = &curves(&touching,&cylinder(2.))[..] else { panic!("the foot's helix, once") };
    let [same] = &curves(&touching,&radial)[..] else { panic!("the two sheets' helix, once") };
    let farther = curves(&touching,&cylinder(3.));
    let level = Surface::Plane(Frame::about([0.,0.,0.1],[0.,0.,1.]));
    let [ruling] = &curves(&touching,&level)[..] else { panic!("a level plane's ruling") };
    // where the two sheets' helix lies on the foot's, over the stretch of each within both patches
    let within = |c: &Curve| -> [f64;2] {
        let ts: Vec<f64> = (0..=64).map(|k| k as f64/64.).filter(|&t| { let p = c.point(t); p[2].abs() <= 0.35 }).collect();
        [ts[0],ts[ts.len()-1]]
    };
    let (tf,ts) = (within(foot),within(same));
    assert!(along_one_another(same,ts,foot,tf,reach) && along_one_another(foot,tf,same,ts,reach),"one helix");
    for other in &farther { assert!(!along_one_another(other,within(other),foot,tf,reach),"a parallel helix") }
    assert!(!along_one_another(ruling,[-4.,4.],foot,tf,reach),"a ruling crossing the helix");
}

/// **A piece of a screwed sheet is boxed by what it reaches** (#847, `Surface::bounds_over` of a
/// blend over a face's parameters): a sliver of a cell beside a screwed slot had been boxed as the
/// whole sheet turned a full revolution, a hundred millimetres across, and its interior sought over
/// all of that. Every point of the piece lies in its box, and the box is the piece's own size,
/// not the ring's; so too a helix on the sheet, and a plane's cut of it, over a stretch.
#[test]
fn a_piece_of_a_screwed_sheet_is_boxed_by_what_it_reaches() {
    let b = sheet([10.,0.,0.],[0.,1.,0.],20.,0.6,5.,0.);
    let s = &b.faces[0].surface;
    let Surface::Blend(..) = s else { panic!("a blend") };
    let size = |(lo,hi): ([f64;3],[f64;3])| distance(lo,hi);
    let ring = s.bounds_over(|| [[0.,1.],[0.,1.]]);
    let range = [[0.45,0.55],[0.3,0.4]];
    let piece = s.bounds_over(|| range);
    let mut hull = ([f64::INFINITY;3],[f64::NEG_INFINITY;3]);
    for i in 0..=16 { for j in 0..=16 {
        let p = s.point([range[0][0]+(range[0][1]-range[0][0])*i as f64/16.,range[1][0]+(range[1][1]-range[1][0])*j as f64/16.]);
        for k in 0..3 {
            assert!(p[k] >= piece.0[k]-1e-12 && p[k] <= piece.1[k]+1e-12,"{p:?} outside {piece:?}");
            hull.0[k] = hull.0[k].min(p[k]); hull.1[k] = hull.1[k].max(p[k]);
        }
    } }
    eprintln!("the ring {:.3}, the piece {:.3}, its points {:.3}",size(ring),size(piece),size(hull));
    assert!(size(piece) <= 1.5*size(hull),"the piece's box {:.3} against its points' {:.3}",size(piece),size(hull));
    assert!(size(piece) < 0.2*size(ring),"the piece's box {:.3} against the ring's {:.3}",size(piece),size(ring));
    // its curves: the helix of the line's foot, and a level plane's ruling, over a stretch
    let helix = Curve::Iso(match s { Surface::Blend(_,bl) => bl.clone(),_ => unreachable!() },0.5);
    for c in [helix] {
        let t = [0.3,0.4];
        let bx = c.bounds(t);
        for k in 0..=32 {
            let p = c.point(t[0]+(t[1]-t[0])*k as f64/32.);
            for i in 0..3 { assert!(p[i] >= bx.0[i]-1e-12 && p[i] <= bx.1[i]+1e-12,"{p:?} outside {bx:?}") }
        }
        assert!(size(bx) < 0.2*size(ring),"a curve's box {:.3} against the ring's {:.3}",size(bx),size(ring));
    }
}
