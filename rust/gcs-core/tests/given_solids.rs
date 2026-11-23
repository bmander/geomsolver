//! **A body a file gives** (#993 stage 2, `SolidDef::Given`): a B-rep `brep::step_read` read,
//! made a solid of the model and evaluated as any other — exact and untouched, an operand of a
//! body's Booleans, meshed, measured, written again.
//!
//! The fixtures are Open CASCADE's (`step_file/`): the drilled block is 40 × 30 × 20 mm with a
//! bore of radius 6 through it at (20, 15), its volume `40·30·20 − π·6²·20`; the void is a 20 mm
//! cube about the origin less a ball of radius 4.  A property is stated as its closed form; the
//! one comparison against the file is `step_roundtrip::same`, the clauses of the reader's own
//! round trip: the same faces in the same order, each on its own surface with the same numbers.

use crate::step_roundtrip::{measured,same,stages,Found};
use gcs_core::brep::export;
use gcs_core::brep::sweep::Say;
use gcs_core::brep::topo::Brep;
use gcs_core::model::{EntRef,Extent,Given,Sketch,SolidDef};
use std::f64::consts::PI;
use std::sync::Arc;

const DRILLED: &str = include_str!("step_file/drilled_ap214.step");
const DRILLED_INCH: &str = include_str!("step_file/drilled_inch.step");
const VOID: &str = include_str!("step_file/void_ap214.step");

/// The drilled block's volume (mm³) and area (mm²), in closed form.
const DRILLED_VOLUME: f64 = 40.*30.*20.-PI*36.*20.;
const DRILLED_AREA: f64 = 2.*(40.*30.+40.*20.+30.*20.)-2.*PI*36.+2.*PI*6.*20.;

const QUIET: Say = Say {stage:&|_| {},mark:&|_| {}};

/// A digest of a file's text (FNV-1a): any number the same on every run will do here.
fn digest(text: &str) -> u64 {
    text.bytes().fold(0xcbf2_9ce4_8422_2325,|h,b| (h^u64::from(b)).wrapping_mul(0x100_0000_01b3))
}

/// The one solid `text` gives, as the reader gives it.
fn read(text: &str) -> Brep {
    let mut solids = gcs_core::brep::step_read::read(text).unwrap_or_else(|r| panic!("{r}"));
    assert_eq!(solids.len(),1);
    solids.remove(0).brep
}

/// `text`'s solid made solid `name` of `sk`.
fn given(sk: &mut Sketch,text: &str,name: &str) -> usize {
    sk.solid(SolidDef::Given(Arc::new(Given::new(read(text),digest(text)))),name)
}

fn near(a: f64,b: f64,rel: f64) -> bool { (a-b).abs() <= rel*b.abs().max(1.) }

/// Solid `i`'s export (`brep::export::exact`, then `step`, which runs `step_check`) read back
/// the same solid as `b`, every face on its original surface.
fn exported_reads_back(sk: &Sketch,i: usize,b: &Brep) -> Brep {
    let built = export::exact(sk,i,None,None,&QUIET).unwrap_or_else(|r| panic!("not built: {}",r.message));
    let text = export::step(&built,"again",None,&QUIET).unwrap_or_else(|r| panic!("not written: {}",r.message));
    let again = stages(&text).unwrap_or_else(|(by,r,_)| panic!("read again, refused by {by:?}: {r}"));
    assert_eq!(again.len(),1);
    let tol = again[0].context.uncertainty.unwrap_or(1e-7);
    same(b,&measured(b),&text,&again[0],gcs_core::brep::step::Fit::Exact,tol,&mut Found::default())
        .unwrap_or_else(|d| panic!("written again, not the same solid: {d}"));
    built.solid
}

/// Whether `a` and `b` are one B-rep: the same faces in the same order, each on the same
/// surface the same way round with the same loops, and the same edges and vertices.
fn untouched(a: &Brep,b: &Brep) -> bool {
    a.faces.len() == b.faces.len() && a.edges.len() == b.edges.len() && a.vertices.len() == b.vertices.len()
        && a.faces.iter().zip(&b.faces).all(|(f,g)| format!("{:?}{:?}{}",f.surface,f.loops,f.reversed)
            == format!("{:?}{:?}{}",g.surface,g.loops,g.reversed))
        && format!("{:?}",a.edges) == format!("{:?}",b.edges)
        && format!("{:?}",a.vertices) == format!("{:?}",b.vertices)
}

/// **The given body is the B-rep read, untouched**: its exact build has the file's faces in the
/// file's order, each on its own surface, its tolerances as measured; each face named apart,
/// by a name no written path reaches; and its volume and area are the block's closed forms.
#[test]
fn a_given_body_is_its_b_rep_untouched() {
    let mut sk = Sketch::new();
    let i = given(&mut sk,DRILLED,"bracket");
    let b = read(DRILLED);
    let x = sk.exact_solid(i).unwrap_or_else(|e| panic!("{e}"));
    assert_eq!((x.mm,x.origin),(1.,[0.;3]));
    assert!(untouched(&x.brep,&b),"the exact build is not the B-rep read");
    let names: Vec<&str> = x.brep.faces.iter().map(|f| f.name.as_str()).collect();
    let want: Vec<String> = (0..b.faces.len()).map(|k| format!("bracket.{}",Given::face_name(k))).collect();
    assert_eq!(names,want);
    assert!(near(gcs_core::brep::props::volume(&x.brep),DRILLED_VOLUME,1e-9));
    let solid = sk.evaluated_solid(i,gcs_core::solid::ApproximationPolicy::Report).unwrap_or_else(|e| panic!("{e}"));
    assert!(near(solid.volume(),DRILLED_VOLUME,1e-9),"{}",solid.volume());
    assert!(near(solid.area(),DRILLED_AREA,1e-9),"{}",solid.area());
}

/// **Its export is the file's solid again**: written by `brep::export` (checked by `step_check`)
/// and read back, the same solid as the file gave, every face on its original surface — the bare
/// given body and a body whose stock it is (`solid(bracket)`), whose build hands it through too.
#[test]
fn its_export_reads_back_as_the_file_gave_it() {
    let b = read(DRILLED);
    let mut sk = Sketch::new();
    let i = given(&mut sk,DRILLED,"bracket");
    let out = exported_reads_back(&sk,i,&b);
    assert!(untouched(&out,&b));
    // `whole := solid(bracket)`: a body whose stock is the given body, nothing on, through or bound
    let whole = sk.solid(SolidDef::Body {stock:i as u32,on:vec![],through:vec![],bound:vec![]},"whole");
    let x = sk.exact_solid(whole).unwrap_or_else(|e| panic!("{e}"));
    assert!(untouched(&x.brep,&b),"the body's exact build is not the B-rep read");
    let out = exported_reads_back(&sk,whole,&b);
    assert!(untouched(&out,&b));
}

/// A rectangle of lines on the page (the front plane: `u = x`, `v = z`, its normal `−y`), as a face.
fn rect_face(sk: &mut Sketch,x0: f64,y0: f64,x1: f64,y1: f64,name: &str) -> usize {
    let ids: Vec<usize> = [(x0,y0),(x1,y0),(x1,y1),(x0,y1)].iter().map(|&(x,y)| sk.point(x,y,false,"")).collect();
    let edges = (0..4).map(|k| EntRef::line(sk.line(ids[k],ids[(k+1)%4]))).collect();
    sk.face(edges,(0..4).map(|k| format!("e{k}")).collect(),name)
}

/// **A given body is a Boolean operand like any other**: a slot 5 × 30 × 10 cut from the top of
/// the block (a prism over its whole width), built exactly to the block's volume less the slot's,
/// written and checked by the export, and read back to the same volume.
#[test]
fn a_slot_is_cut_from_a_given_body() {
    let mut sk = Sketch::new();
    let block = given(&mut sk,DRILLED,"block");
    // x in (−1, 5), z in (10, 25), and along the page's normal −y from y = 31 to y = −1
    let f = rect_face(&mut sk,-1.,10.,5.,25.,"slot");
    let slot = sk.solid(SolidDef::Prism {face:f as u32,from:Extent::at(-31.),to:Extent::at(1.),draft:None},"slot");
    let part = sk.solid(SolidDef::Body {stock:block as u32,on:vec![],through:vec![slot as u32],bound:vec![]},"part");
    let want = DRILLED_VOLUME-5.*30.*10.;
    let x = sk.exact_solid(part).unwrap_or_else(|e| panic!("{e}"));
    assert!(near(gcs_core::brep::props::volume(&x.brep)/(x.mm*x.mm*x.mm),want,1e-9));
    // the block's faces keep their names through the cut, the slot's are its own
    let names: std::collections::BTreeSet<&str> = x.brep.faces.iter().map(|f| f.name.as_str()).collect();
    assert!(names.contains(format!("block.{}",Given::face_name(0)).as_str()),"{names:?}");
    assert!(names.iter().any(|n| n.starts_with("slot.")),"{names:?}");
    let solid = sk.evaluated_solid(part,gcs_core::solid::ApproximationPolicy::Report).unwrap_or_else(|e| panic!("{e}"));
    assert!(near(solid.volume(),want,1e-9),"{}",solid.volume());
    let built = export::exact(&sk,part,None,None,&QUIET).unwrap_or_else(|r| panic!("not built: {}",r.message));
    let text = export::step(&built,"part",None,&QUIET).unwrap_or_else(|r| panic!("not written: {}",r.message));
    let again = read(&text);
    assert!(near(gcs_core::brep::props::volume(&again),want,1e-9));
}

/// **What reads a solid through its B-rep reads a given body**: its display mesh closed, its
/// feature edges the block's twelve and the bore's two circles (each face named apart, so no
/// edge between two of them is lost), a section through it, its facet term (what `mesh_unit` and a
/// refused build read) the block's box, its glTF written, and its faces read by the rounds' reader.
#[test]
fn a_given_body_is_meshed_edged_and_cut() {
    let mut sk = Sketch::new();
    let i = given(&mut sk,DRILLED,"bracket");
    let solid = sk.evaluated_solid(i,gcs_core::solid::ApproximationPolicy::Mesh).unwrap_or_else(|e| panic!("{e}"));
    let b = solid.bounds();
    assert!((0..3).all(|k| (b.lo[k]-[0.,0.,0.][k]).abs() < 1e-9 && (b.hi[k]-[40.,30.,20.][k]).abs() < 1e-9),"{b:?}");
    assert!(near(solid.volume(),DRILLED_VOLUME,1e-9));
    // the edges where two faces turn from one another: 4·(40 + 30 + 20) of the block's and the
    // bore's two circles, their chords a little short of 2·2π·6
    let edges = solid.face_edges();
    assert!(edges.iter().all(|e| e.faces.0 != e.faces.1));
    let length: f64 = edges.iter().flat_map(|e| &e.segs).map(|[a,b]| gcs_core::space::distance(*a,*b)).sum();
    let want = 4.*(40.+30.+20.)+2.*2.*PI*6.;
    assert!(length <= want+1e-6 && length >= want*(1.-1e-3),"{length} against {want}");
    let pairs: std::collections::BTreeSet<_> = edges.iter().map(|e| e.faces.clone()).collect();
    assert!(pairs.len() >= 14,"{pairs:?}");
    // cut at mid-height: the section's area is the block's less the bore's, 40·30 − π·36
    let cut = gcs_core::plane::Basis {u:[1.,0.,0.],v:[0.,1.,0.],o:[0.,0.,10.]};
    let section = solid.section(&cut);
    let area: f64 = section.triangles.chunks(9).map(|t| {
        let [a,b,c] = [0,3,6].map(|k| [t[k],t[k+1],t[k+2]]);
        gcs_core::space::norm(gcs_core::space::cross(gcs_core::space::sub(b,a),gcs_core::space::sub(c,a)))/2.
    }).sum();
    assert!(near(area,40.*30.-PI*36.,1e-3),"{area}");
    // the facet term stands for it as its exact mesh does: the block's box, never an empty solid
    let csg = gcs_core::solid::resolve(&sk,i,gcs_core::solid::REPORT_UNIT);
    let fb = csg.bbox();
    assert!((0..3).all(|k| fb.lo[k].abs() < 1e-9 && (fb.hi[k]-[40.,30.,20.][k]).abs() < 1e-9),"{fb:?}");
    assert!(near(gcs_core::solid::mesh_unit(&sk,i),(50f64.hypot(20.)*gcs_core::solid::MESH_SAGITTA
        /gcs_core::curve::FLATNESS_PX).max(gcs_core::solid::REPORT_UNIT),1e-9));
    gcs_core::gltf::checked_glb(&sk,&[i],gcs_core::solid::ApproximationPolicy::Mesh)
        .unwrap_or_else(|e| panic!("no glTF: {e}"));
    let faces = gcs_core::solid::faces::cached(&sk,i).unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(faces.paths().len(),read(DRILLED).faces.len());
}

/// **A model in inches holds a millimetre B-rep**: the given body's volume is in cubic inches,
/// its mesh in inches, and its export in millimetres the file's solid again.
#[test]
fn a_model_in_inches_reads_the_millimetre_b_rep() {
    let mut sk = Sketch::new();
    sk.units = gcs_core::units::Units::with_length("in").unwrap();
    let i = given(&mut sk,DRILLED,"bracket");
    let solid = sk.evaluated_solid(i,gcs_core::solid::ApproximationPolicy::Report).unwrap_or_else(|e| panic!("{e}"));
    assert!(near(solid.volume(),DRILLED_VOLUME/25.4f64.powi(3),1e-9),"{}",solid.volume());
    let b = solid.bounds();
    assert!((0..3).all(|k| (b.hi[k]-[40.,30.,20.][k]/25.4).abs() < 1e-9 && b.lo[k].abs() < 1e-9),"{b:?}");
    let out = exported_reads_back(&sk,i,&read(DRILLED));
    assert!(untouched(&out,&read(DRILLED)));
    // the file in inches gives the same solid in millimetres
    let mut sk = Sketch::new();
    let i = given(&mut sk,DRILLED_INCH,"bracket");
    let solid = sk.evaluated_solid(i,gcs_core::solid::ApproximationPolicy::Report).unwrap_or_else(|e| panic!("{e}"));
    assert!(near(solid.volume(),DRILLED_VOLUME,1e-9),"{}",solid.volume());
}

/// **A given body is read by its material field** where every face's nearest point is exact (the
/// block's planes and its bore's cylinder): the signed distance, negative inside, is 5 from the
/// corner faces at (5, 5, 10) and 6 out of the bore's axis.
#[test]
fn a_given_body_has_its_material_field() {
    let mut sk = Sketch::new();
    let i = given(&mut sk,DRILLED,"bracket");
    let field = gcs_core::solid::SpatialField::read(&sk,i,1e-10).unwrap_or_else(|e| panic!("{e}"));
    assert!((field.value([5.,5.,10.])+5.).abs() < 1e-9,"{}",field.value([5.,5.,10.]));
    assert!((field.value([20.,15.,10.])-6.).abs() < 1e-9,"{}",field.value([20.,15.,10.]));
}

/// **A void, and a solid far from the origin**: the cube about the origin less the ball (volume
/// `20³ − 4/3·π·4³`, the ball's centre outside its material); and the drilled block a reader
/// placed at (1000, −2000, 500) mm, which stands there, its exact build untouched, in a model in
/// centimetres too.
#[test]
fn a_void_and_a_placed_solid_stand_where_the_file_puts_them() {
    let mut sk = Sketch::new();
    let v = given(&mut sk,VOID,"hollow");
    let hollow = sk.evaluated_solid(v,gcs_core::solid::ApproximationPolicy::Report).unwrap_or_else(|e| panic!("{e}"));
    assert!(near(hollow.volume(),8000.-4./3.*PI*64.,1e-9),"{}",hollow.volume());
    assert!(!hollow.contains_world(gcs_core::solid::WorldPoint([0.;3])));
    assert!(hollow.contains_world(gcs_core::solid::WorldPoint([6.,0.,0.])));
    let by = [1000.,-2000.,500.];
    let moved = read(DRILLED).moved(&gcs_core::brep::geom::Rigid {t:by,..gcs_core::brep::geom::Rigid::identity()});
    for units in ["mm","cm"] {
        let mut sk = Sketch::new();
        sk.units = gcs_core::units::Units::with_length(units).unwrap();
        let p = sk.solid(SolidDef::Given(Arc::new(Given::new(moved.clone(),7))),"block");
        let mm = sk.units.mm_per();
        let block = sk.evaluated_solid(p,gcs_core::solid::ApproximationPolicy::Report).unwrap_or_else(|e| panic!("{e}"));
        let b = block.world_bounds();
        let (lo,hi) = (by.map(|x| x/mm),[40.,30.,20.].map(|x| x/mm));
        assert!((0..3).all(|k| (b.lo[k]-lo[k]).abs() < 1e-9 && (b.hi[k]-lo[k]-hi[k]).abs() < 1e-9),"{units}: {b:?}");
        assert!(near(block.volume(),DRILLED_VOLUME/mm.powi(3),1e-9),"{units}: {}",block.volume());
        let x = sk.exact_solid(p).unwrap_or_else(|e| panic!("{e}"));
        assert!(untouched(&x.brep,&moved));
    }
}

/// **What a given body is, it is by its digest**: the memo keys a given body by its file's
/// digest, so another file's body in its place is evaluated again — the block after the void.
#[test]
fn another_file_in_its_place_is_evaluated_again() {
    let mut sk = Sketch::new();
    let i = given(&mut sk,VOID,"part");
    let report = gcs_core::solid::ApproximationPolicy::Report;
    let was = sk.evaluated_solid(i,report).unwrap_or_else(|e| panic!("{e}")).volume();
    assert!(near(was,8000.-4./3.*PI*64.,1e-9));
    sk.solids[i].def = SolidDef::Given(Arc::new(Given::new(read(DRILLED),digest(DRILLED))));
    let now = sk.evaluated_solid(i,report).unwrap_or_else(|e| panic!("{e}")).volume();
    assert!(near(now,DRILLED_VOLUME,1e-9),"{now}");
}

/// **What a given body cannot be read as is refused in its own words, never panicked over**: a
/// face on an extruded spline, whose nearest point no closed form gives, has no material field;
/// a B-rep of no face is an empty solid, its export refused as one.
#[test]
fn what_a_given_body_cannot_be_is_refused() {
    use gcs_core::brep::build::{prism,Profile,ProfileEdge};
    use gcs_core::brep::nurbs::BSpline;
    let bump = BSpline::new(3,vec![0.,0.,0.,0.,1.,1.,1.,1.],vec![[20.,0.,0.],[20.,15.,0.],[0.,15.,0.],[0.,0.,0.]]).unwrap();
    let lobe = Profile {origin:[0.;3],normal:[0.,0.,1.],names:vec![],
        loops:vec![vec![ProfileEdge::Line {a:[0.,0.,0.],b:[20.,0.,0.]},ProfileEdge::Spline(Arc::new(bump))]]};
    let mut sk = Sketch::new();
    let i = sk.solid(SolidDef::Given(Arc::new(Given::new(prism(&lobe,0.,10.).unwrap(),1))),"lobe");
    let why = gcs_core::solid::SpatialField::read(&sk,i,1e-10).err().expect("no field of an extruded spline");
    assert!(why.starts_with("`lobe`: ") && why.contains("extrusion surface"),"{why}");
    // it is meshed and measured all the same
    let v = sk.evaluated_solid(i,gcs_core::solid::ApproximationPolicy::Report).unwrap_or_else(|e| panic!("{e}")).volume();
    assert!(v > 0.,"{v}");
    let e = sk.solid(SolidDef::Given(Arc::new(Given::new(Brep::default(),2))),"nothing");
    let refused = export::exact(&sk,e,None,None,&QUIET).err().expect("an empty body is not written");
    assert!(refused.message.contains("`nothing` is empty"),"{}",refused.message);
}

/// **The export holds a given body to its measured tolerances, no tighter**: the drilled block
/// with a corner read 2e-4 mm off where its three edges meet it, the vertex and those edges
/// carrying the 5e-4 mm a reader measured there, is built, checked and written by the export, and
/// reads back the same solid.
#[test]
fn a_given_body_is_held_to_its_measured_tolerances() {
    let mut b = read(DRILLED);
    let v = b.vertices.iter().position(|v| gcs_core::space::norm(v.p) < 1e-9).expect("a corner at the origin");
    b.vertices[v].p = [2e-4,0.,0.];
    b.vertices[v].tol = 5e-4;
    for e in b.edges.iter_mut().filter(|e| e.v.contains(&(v as u32))) { e.tol = 5e-4; }
    b.check(1e-7).expect("the B-rep is sound to its tolerances");
    let mut sk = Sketch::new();
    let i = sk.solid(SolidDef::Given(Arc::new(Given::new(b.clone(),3))),"bracket");
    let out = exported_reads_back(&sk,i,&b);
    assert!(untouched(&out,&b));
}

/// **A sliver a file's body holds is said as the file's** (#993 stage 2): a slab 5 µm thick given
/// as a file gives a body — its four walls thinner than the export's tolerance — is said by the
/// export with each face as a face of the body, never by a key no document can write, and with
/// no advice to move a cut's end, since the document made no cut: the file gives it so.  A body
/// the document cuts from it keeps the advice, for there an end is the document's to move.
#[test]
fn a_sliver_a_files_body_holds_is_said_as_the_files() {
    use gcs_core::brep::build::{prism,Profile,ProfileEdge};
    let corners = [[0.,0.,0.],[20.,0.,0.],[20.,10.,0.],[0.,10.,0.]];
    let slab = Profile {origin:[0.;3],normal:[0.,0.,1.],names:vec![],
        loops:vec![(0..4).map(|k| ProfileEdge::Line {a:corners[k],b:corners[(k+1)%4]}).collect()]};
    let mut sk = Sketch::new();
    let given = sk.solid(SolidDef::Given(Arc::new(Given::new(prism(&slab,0.,5e-3).unwrap(),4))),"bracket");
    let part = sk.solid(SolidDef::Body {stock:given as u32,on:vec![],through:vec![],bound:vec![]},"part");
    for i in [given,part] {
        let exact = export::exact(&sk,i,None,None,&QUIET).unwrap_or_else(|r| panic!("not built: {}",r.message));
        let s = export::slivers(&sk,i,&exact,None).expect("walls 5 µm tall are slivers");
        for said in ["holds 4 faces thinner than the export's 10.000 µm: the first, a face of `bracket`, 5.000 µm across",
            "The file `bracket` gives it so, and the document states nothing of it: it is written as it is given"] {
            assert!(s.message.contains(said),"{said}: {}",s.message);
        }
        for unsaid in ["#given","move that end","a face of `bracket` and a face of `bracket`"] {
            assert!(!s.message.contains(unsaid),"{unsaid}: {}",s.message);
        }
    }
}

/// **A given body read on its sheet's rows is exported and read back the same** (#993 stage 2):
/// the slab under benchy's crumpled face (`step_kinds/crumpled_corner.step`, `topology::rows`),
/// exported as the document exports it (`export::step`, its file read by the kernel's reader) —
/// whose export of #3DBenchy the reader refused — and read back the file's solid.
#[test]
fn a_body_read_on_its_sheets_rows_exports_and_reads_back() {
    const CRUMPLED: &str = include_str!("step_kinds/crumpled_corner.step");
    let b = read(CRUMPLED);
    let mut sk = Sketch::new();
    let i = given(&mut sk,CRUMPLED,"crumpled");
    let out = exported_reads_back(&sk,i,&b);
    assert!(untouched(&out,&b));
}
