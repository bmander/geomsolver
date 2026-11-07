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
use gcs_core::brep::boolean::along_one_another;
use gcs_core::brep::geom::{include,Curve,Frame,Surface,EMPTY};
use gcs_core::brep::props::volume;
use gcs_core::brep::sliver::slivers;
use gcs_core::brep::ssi::{intersect,Ssi};
use gcs_core::brep::topo::{Brep,Edge,EdgeCurve};
use gcs_core::solid::admission::Class;
use gcs_core::space::distance;

use crate::meeting::screwed_sheet;
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

/// The tube with its slots, built exactly.
fn slotted(angles: &[f64]) -> Brep {
    let e = fixtures::read(&tube(angles));
    let Class::Closed(found) = class(&e,"part") else { panic!("the closed class") };
    assert!(found.screw().is_some_and(|s| !s.is_slide() && !s.is_turn()),"a screw");
    exported_within(&e,"part",None).solid
}

/// Every vertex where two edges alone meet: two pieces of one curve end to end there, which
/// `rejoined` makes one.
fn split_curves(b: &Brep) -> Vec<V> {
    (0..b.vertices.len()).filter_map(|v| {
        let at: Vec<&Edge> = b.edges.iter().filter(|e| e.v.contains(&(v as u32))).collect();
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
    let solid = slotted(&[]);
    assert_eq!(solid.faces.len(),12,"the tube's four faces and the slot's eight");
    assert!(slivers(&solid,0.01).is_empty(),"slivers under 10 µm: {:?}",slivers(&solid,0.01));
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
    let (one,two) = (slotted(&[]),slotted(&[180.]));
    let (v1,v2) = (volume(&one),volume(&two));
    eprintln!("one slot: {} faces, {} edges; two: {} faces, {} edges",one.faces.len(),one.edges.len(),two.faces.len(),two.edges.len());
    assert!(((whole-v2)-2.*(whole-v1)).abs() <= 1e-9*whole,"two slots remove {} against twice {}",whole-v2,whole-v1);
    assert!(two.faces.len() <= 2*one.faces.len()-4,"{} faces against {} for one",two.faces.len(),one.faces.len());
    assert!(two.edges.len() <= 2*one.edges.len(),"{} edges against {} for one",two.edges.len(),one.edges.len());
    assert_eq!(split_curves(&two),Vec::<V>::new(),"pieces of one curve left end to end");
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
    let (angle,advance,tol) = (1.5,1.2,1e-6);
    let reach = 128.*tol;
    // the line x = 2 at z = 0, its foot (2, 0, 0); and the radial line through that foot screwed
    // three tenths of the way on
    let touching = screwed_sheet([2.,0.,0.],[0.,1.,0.],6.,angle,advance,0.5);
    let (s,c) = (0.3*angle).sin_cos();
    let radial = screwed_sheet([2.*c,2.*s,0.3*advance],[c,s,0.],6.,angle,advance,0.8);
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
/// all of that. Every point of the piece lies in its box, and the box is the piece's own size, not
/// the ring's; so too a helix on the sheet, and a plane's cut of it, over a stretch.
#[test]
fn a_piece_of_a_screwed_sheet_is_boxed_by_what_it_reaches() {
    let s = screwed_sheet([10.,0.,0.],[0.,1.,0.],20.,6.,5.,0.5);
    let Surface::Blend(_,blend) = &s else { panic!("a blend") };
    let size = |(lo,hi): ([f64;3],[f64;3])| distance(lo,hi);
    let holds = |b: ([f64;3],[f64;3]),p: [f64;3]| (0..3).all(|k| p[k] >= b.0[k]-1e-12 && p[k] <= b.1[k]+1e-12);
    let ring = s.bounds_over(|| [[0.,1.],[0.,1.]]);
    let range = [[0.45,0.55],[0.3,0.4]];
    let piece = s.bounds_over(|| range);
    let mut hull = EMPTY;
    for i in 0..=16 { for j in 0..=16 {
        let p = s.point([range[0][0]+(range[0][1]-range[0][0])*i as f64/16.,range[1][0]+(range[1][1]-range[1][0])*j as f64/16.]);
        assert!(holds(piece,p),"{p:?} outside {piece:?}");
        include(&mut hull,p);
    } }
    eprintln!("the ring {:.3}, the piece {:.3}, its points {:.3}",size(ring),size(piece),size(hull));
    assert!(size(piece) <= 1.5*size(hull),"the piece's box {:.3} against its points' {:.3}",size(piece),size(hull));
    assert!(size(piece) < 0.2*size(ring),"the piece's box {:.3} against the ring's {:.3}",size(piece),size(ring));
    // the helix of the line's foot, and the cut of the plane along the axis holding its ruling there
    let Ssi::Curves(cut) = intersect(&Surface::Plane(Frame::about([10.,0.,0.],[1.,0.,0.])),&s,1e-9) else { panic!("a closed form") };
    let Some(cut) = cut.into_iter().find(|c| matches!(c,Curve::SheetCut(_))) else { panic!("a sheet cut") };
    for c in [Curve::Iso(blend.clone(),0.5),cut] {
        let t = [0.3,0.4];
        let bx = c.bounds(t);
        for k in 0..=32 { let p = c.point(t[0]+(t[1]-t[0])*k as f64/32.); assert!(holds(bx,p),"{}: {p:?} outside {bx:?}",c.kind()) }
        assert!(size(bx) < 0.2*size(ring),"{}'s box {:.3} against the ring's {:.3}",c.kind(),size(bx),size(ring));
    }
}
