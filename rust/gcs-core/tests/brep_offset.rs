//! The kernel's offsets (`brep::offset`, docs/contract/brep.md "Offsets"): a solid grown or
//! shrunk by a distance, hollowed to a wall, a sheet thickened — each built exactly as the union of
//! the pieces its points are nearest (each face's normal sweep, each crease's tube or kite, each
//! corner's ball or parallelepiped), held to its closed form to 1e-9.  And the two Boolean and
//! mesher faults the brep-bench run found (#elliptic-hole, #horn-torus): a closed rational spline
//! cut square to its sweep, a torus whose chart pinches at the middle of its box.
use gcs_core::brep::boolean::{combined,Op};
use gcs_core::brep::build::{cone,disc,kept,prism,sphere,torus,Profile,ProfileEdge};
use gcs_core::brep::geom::{Frame,V};
use gcs_core::brep::nurbs::BSpline;
use gcs_core::brep::offset::{grown,sheet_of,shelled,shrunk,thickened,Join,Side};
use gcs_core::brep::props::{area,volume};
use gcs_core::brep::query::{Located,Place};
use gcs_core::brep::topo::Brep;
use std::f64::consts::{FRAC_1_SQRT_2,PI};
use std::sync::Arc;

fn close(what: &str,a: f64,b: f64) { assert!((a-b).abs() <= 1e-9*b.abs().max(1.),"{what}: {a} against {b} ({:e})",(a-b)/b); }

/// The closed polygon through `pts` as a profile, its normal by Newell's rule.
fn polygon(pts: &[V]) -> Profile {
    let n = pts.len();
    let mut normal = [0.;3];
    for k in 0..n {
        let (a,b) = (pts[k],pts[(k+1)%n]);
        normal = [normal[0]+a[1]*b[2]-a[2]*b[1],normal[1]+a[2]*b[0]-a[0]*b[2],normal[2]+a[0]*b[1]-a[1]*b[0]];
    }
    let l = (normal[0]*normal[0]+normal[1]*normal[1]+normal[2]*normal[2]).sqrt();
    Profile {origin:pts[0],normal:normal.map(|x| x/l),loops:vec![(0..n).map(|k| ProfileEdge::Line {a:pts[k],b:pts[(k+1)%n]}).collect()],
        names:Vec::new()}
}

fn block(lo: V,hi: V) -> Brep {
    prism(&polygon(&[lo,[hi[0],lo[1],lo[2]],[hi[0],hi[1],lo[2]],[lo[0],hi[1],lo[2]]]),0.,hi[2]-lo[2]).unwrap()
}

fn l_block() -> Brep {
    prism(&polygon(&[[0.,0.,0.],[20.,0.,0.],[20.,5.,0.],[5.,5.,0.],[5.,15.,0.],[0.,15.,0.]]),0.,8.).unwrap()
}

fn total_area(b: &Brep) -> f64 { b.faces.iter().map(|f| area(b,f)).sum() }

/// The faces of `b` the points `on` lie inside, one each.
fn faces_through(b: &Brep,on: &[V]) -> Vec<usize> {
    let located = Located::new(b,1e-9*b.size());
    on.iter().map(|&p| (0..b.faces.len()).find(|&fi| {
        let q = b.faces[fi].surface.point(located.uv(fi,p));
        (0..3).all(|k| (q[k]-p[k]).abs() <= 1e-9) && located.face_place(fi,p) == Place::In
    }).unwrap_or_else(|| panic!("no face through {p:?}"))).collect()
}

/// Steiner's formula for the box `a × b × c` grown by `d`: its faces' slabs, its edges' quarter
/// cylinders, its corners' eighths of a ball.
fn steiner_box(a: f64,b: f64,c: f64,d: f64) -> f64 {
    a*b*c+2.*(a*b+b*c+c*a)*d+PI*(a+b+c)*d*d+4.*PI*d*d*d/3.
}

#[test]
fn a_box_grown_is_steiners_volume_and_sharp_its_moved_faces_box() {
    let b = block([0.,0.,0.],[10.,8.,6.]);
    let g = grown(&b,1.,Join::Round).unwrap();
    g.check(1e-7).unwrap();
    close("grown",volume(&g),steiner_box(10.,8.,6.,1.));
    close("its area",total_area(&g),2.*(80.+48.+60.)+2.*PI*24.+4.*PI);
    let s = grown(&b,1.,Join::Sharp).unwrap();
    close("grown sharp",volume(&s),12.*10.*8.);
    let k = shrunk(&b,1.,Join::Round).unwrap();
    close("shrunk",volume(&k),8.*6.*4.);
}

#[test]
fn an_l_grown_rounds_its_convex_edges_and_shrunk_its_concave_one() {
    let (a,p,t,d) = (150.,70.,8.,1.);
    let k = 5.*PI/4.-1.;
    close("grown",volume(&grown(&l_block(),d,Join::Round).unwrap()),
        t*(a+p*d+k*d*d)+2.*(a*d+PI*p*d*d/4.+2.*k*d*d*d/3.));
    let k = 5.-PI/4.;
    close("shrunk",volume(&shrunk(&l_block(),d,Join::Round).unwrap()),(t-2.*d)*(a-p*d+k*d*d));
    close("grown sharp",volume(&grown(&l_block(),d,Join::Sharp).unwrap()),(22.*7.+7.*10.)*10.);
}

/// The tetrahedron's corners' balls make one whole ball, its edges' cylinders turn through
/// π − acos(1/3) each.
#[test]
fn a_tetrahedron_grown_rounds_its_oblique_edges_and_corners() {
    let corners: [V;4] = [[3.,3.,3.],[3.,-3.,-3.],[-3.,3.,-3.],[-3.,-3.,3.]];
    let mut t = block([-4.,-4.,-4.],[4.,4.,4.]);
    for k in 0..4 {
        let [a,b,c] = [corners[(k+1)%4],corners[(k+2)%4],corners[(k+3)%4]];
        let (u,v) = ([b[0]-a[0],b[1]-a[1],b[2]-a[2]],[c[0]-a[0],c[1]-a[1],c[2]-a[2]]);
        let mut n = [u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]];
        let w = [corners[k][0]-a[0],corners[k][1]-a[1],corners[k][2]-a[2]];
        if n[0]*w[0]+n[1]*w[1]+n[2]*w[2] < 0. { n = n.map(|x| -x); }
        let l = (n[0]*n[0]+n[1]*n[1]+n[2]*n[2]).sqrt();
        t = kept(&t,a,n.map(|x| x/l),0.).unwrap();
    }
    let e = 6.*2f64.sqrt();
    let want = 72.+72.*3f64.sqrt()+3.*e*(PI-(1f64/3.).acos())+4.*PI/3.;
    close("grown",volume(&grown(&t,1.,Join::Round).unwrap()),want);
}

/// Grown past its hole a torus is an apple: the tube's disc cut at the axis, turned.  Shrunk, the
/// tube narrows about the same middle circle.
#[test]
fn a_torus_grown_past_its_hole_is_an_apple_and_shrunk_a_thinner_torus() {
    let (big,s): (f64,f64) = (4.,5.);
    let c = (s*s-big*big).sqrt();
    let phi = (big/s).acos();
    let apple = grown(&torus([0.,0.,0.],[0.,0.,1.],big,2.).unwrap(),3.,Join::Round).unwrap();
    close("apple",volume(&apple),2.*PI*(PI*big*s*s+2.*c*c*c/3.-big*(s*s*phi-big*c)));
    let thin = shrunk(&torus([0.,0.,0.],[0.,0.,1.],10.,3.).unwrap(),1.5,Join::Round).unwrap();
    close("shrunk",volume(&thin),2.*PI*PI*10.*1.5*1.5);
}

/// A groove narrower than twice the growth is filled; a flange thinner than twice the shrinking
/// is taken away — the moved faces cross, and the union decides.
#[test]
fn a_groove_grown_shut_and_a_flange_shrunk_away() {
    let a: f64 = 0.8;
    let m = |s: f64| 20.*a-10.*(a*(1.-a*a).sqrt()+a.asin())+s*a*a*a/3.;
    let drum = prism(&disc([0.,0.,0.],[0.,0.,1.],10.),0.,10.).unwrap();
    let o = [0.,0.,4.2];
    let ring = Profile {origin:o,normal:[0.,0.,1.],names:Vec::new(),loops:vec![
        vec![ProfileEdge::Arc {frame:Frame::about(o,[0.,0.,1.]),r:12.,span:None}],
        vec![ProfileEdge::Arc {frame:Frame::about(o,[0.,0.,1.]),r:7.,span:None}]]};
    let grooved = combined(&drum,&prism(&ring,0.,1.6).unwrap(),Op::Cut,0.).unwrap();
    close("grooved, grown",volume(&grown(&grooved,1.,Join::Round).unwrap()),2.*PI*(705.+5.*PI+2./3.-m(1.)));
    let drum = prism(&disc([0.,0.,0.],[0.,0.,1.],8.),0.,6.).unwrap();
    let flange = prism(&disc([0.,0.,2.2],[0.,0.,1.],11.),0.,1.6).unwrap();
    let flanged = combined(&drum,&flange,Op::Union,0.).unwrap();
    let m = 16.*a-8.*(a*(1.-a*a).sqrt()+a.asin())-a*a*a/3.;
    close("flanged, shrunk",volume(&shrunk(&flanged,1.,Join::Round).unwrap()),PI*49.*4.+2.*PI*m);
}

/// A cone's apex grown is a cap of a ball: a corner of no crease.
#[test]
fn a_cone_grown_caps_its_apex_with_a_ball() {
    let (r,h,l,d) = (3.,4.,5.,1.);
    let beta = (4f64/3.).atan();
    let want = PI*r*r*h/3.+PI*(r*r+r*l)*d+PI*(h+r*(PI-beta))*d*d+4.*PI*d*d*d/3.;
    close("grown",volume(&grown(&cone([0.,0.,0.],[0.,0.,1.],r,0.,h).unwrap(),d,Join::Round).unwrap()),want);
}

#[test]
fn a_box_shelled_open_at_its_top_inward_outward_and_sharp() {
    let b = block([0.,0.,0.],[10.,8.,6.]);
    let top = faces_through(&b,&[[5.,4.,6.]]);
    close("inward",volume(&shelled(&b,&top,1.,Side::In,Join::Round).unwrap()),480.-8.*6.*5.);
    close("closed",volume(&shelled(&b,&[],1.,Side::In,Join::Round).unwrap()),480.-8.*6.*4.);
    close("walls meeting",volume(&shelled(&b,&top,4.,Side::In,Join::Round).unwrap()),480.);
    close("outward, sharp",volume(&shelled(&b,&top,1.,Side::Out,Join::Sharp).unwrap()),12.*10.*7.-480.);
    // outward: the box grown, cut off at the top's plane, less the box
    close("outward",volume(&shelled(&b,&top,1.,Side::Out,Join::Round).unwrap()),296.+47.*PI/3.);
}

/// A rounded plate's corners swept in past their axis: what lies past it is its neighbours'.
#[test]
fn a_rounded_tray_shelled_past_its_rounds() {
    use std::f64::consts::FRAC_PI_2;
    let (lo,hi,r) = ([0.,0.],[20.,14.],2.);
    let corners = [([hi[0]-r,lo[1]+r],-FRAC_PI_2),([hi[0]-r,hi[1]-r],0.),([lo[0]+r,hi[1]-r],FRAC_PI_2),([lo[0]+r,lo[1]+r],PI)];
    let mut edges = Vec::new();
    for k in 0..4 {
        let (c,a) = corners[k];
        edges.push(ProfileEdge::Arc {frame:Frame::new([c[0],c[1],0.],[0.,0.,1.],[1.,0.,0.]),r,span:Some([a,a+FRAC_PI_2])});
        let (n,b) = corners[(k+1)%4];
        edges.push(ProfileEdge::Line {a:[c[0]+r*(a+FRAC_PI_2).cos(),c[1]+r*(a+FRAC_PI_2).sin(),0.],b:[n[0]+r*b.cos(),n[1]+r*b.sin(),0.]});
    }
    let plate = prism(&Profile {origin:[0.;3],normal:[0.,0.,1.],loops:vec![edges],names:Vec::new()},0.,10.).unwrap();
    let top = faces_through(&plate,&[[10.,7.,10.]]);
    close("rounds consumed",volume(&shelled(&plate,&top,3.,Side::In,Join::Round).unwrap()),1856.+40.*PI);
}

/// A moved wall runs on across a face left open until it meets it: inside a ball's cap the cavity
/// is the inner ball, which the opening never reaches.
#[test]
fn a_wall_runs_on_to_its_opening() {
    let cap = kept(&sphere([0.,0.,0.],[0.,0.,1.],5.).unwrap(),[0.,0.,4.5],[0.,0.,-1.],0.).unwrap();
    let flat = faces_through(&cap,&[[0.5,0.5,4.5]]);
    close("sealed",volume(&shelled(&cap,&flat,1.,Side::In,Join::Round).unwrap()),641.*PI/8.);
}

/// Sheets lifted off solids, thickened: an open box out of its cube with rounded and sharp
/// edges, and a bend thickened past its own radius, whose moved bend turns inside out.
#[test]
fn sheets_thickened() {
    let cube = block([0.,0.,0.],[10.,10.,10.]);
    let walls = faces_through(&cube,&[[5.,5.,0.],[0.,5.,5.],[10.,5.,5.],[5.,0.,5.],[5.,10.,5.]]);
    let tray = sheet_of(&cube,&walls).unwrap();
    close("rounded",volume(&thickened(&tray,[0.,1.],Join::Round).unwrap()),5.*100.+8.*PI*10./4.+4.*PI/6.);
    close("sharp",volume(&thickened(&tray,[0.,1.],Join::Sharp).unwrap()),12.*12.*11.-1000.);
    close("inward",volume(&thickened(&tray,[-1.,0.],Join::Round).unwrap()),1000.-8.*8.*9.);
    let bend = Frame::new([0.,0.,2.],[0.,1.,0.],[1.,0.,0.]);
    let section = Profile {origin:[0.;3],normal:[0.,1.,0.],names:Vec::new(),loops:vec![vec![
        ProfileEdge::Line {a:[-10.,0.,0.],b:[0.,0.,0.]},
        ProfileEdge::Arc {frame:bend,r:2.,span:Some([0.,std::f64::consts::FRAC_PI_2])},
        ProfileEdge::Line {a:[2.,0.,2.],b:[2.,0.,12.]},ProfileEdge::Line {a:[2.,0.,12.],b:[-10.,0.,12.]},
        ProfileEdge::Line {a:[-10.,0.,12.],b:[-10.,0.,0.]}]]};
    let solid = prism(&section,0.,8.).unwrap();
    let flange = sheet_of(&solid,&faces_through(&solid,&[[-5.,4.,0.],[2f64.sqrt(),4.,2.-2f64.sqrt()],[2.,4.,7.]])).unwrap();
    let (l,r,t,w) = (10.,2.,3.,8.);
    close("past its bend",volume(&thickened(&flange,[-t,0.],Join::Round).unwrap()),w*(2.*l*t-(t-r)*(t-r)+PI*r*r/4.));
    let hemi = kept(&sphere([0.,0.,0.],[0.,0.,1.],5.).unwrap(),[0.,0.,0.],[0.,0.,1.],0.).unwrap();
    let dome = sheet_of(&hemi,&faces_through(&hemi,&[[3.,0.,4.]])).unwrap();
    close("to its centre",volume(&thickened(&dome,[-5.,0.],Join::Round).unwrap()),2.*PI*125./3.);
}

/// A sheet with no offset in closed form is refused, not approximated.
#[test]
fn a_spline_sheet_is_not_thickened() {
    use gcs_core::brep::nurbs::Net;
    let net = Net {du:2,dv:1,uknots:vec![0.,0.,0.,1.,1.,1.],vknots:vec![0.,0.,1.,1.],weights:None,
        poles:vec![vec![[-4.,0.,2.],[-4.,10.,2.]],vec![[0.,0.,-2.],[0.,10.,-2.]],vec![[4.,0.,2.],[4.,10.,2.]]]};
    let e = thickened(&gcs_core::brep::build::sheet(net).unwrap(),[0.,1.],Join::Round).unwrap_err();
    assert!(e.contains("no offset in closed form"),"{e}");
}

/// A plane cutting a rational spline's sweep square to it meets it in the spline carried along
/// the sweep — its weights kept — and that meeting, closed on one point with no other on it, is
/// one edge round: an elliptic bore cut through a plate, which was the plate untouched.
#[test]
fn an_elliptic_bore_cuts_through_its_plate() {
    let plate = block([-12.,-8.,0.],[12.,8.,8.]);
    let at = |x: f64,y: f64| [6.*x,3.*y,-1.];
    let poles = vec![at(1.,0.),at(1.,1.),at(0.,1.),at(-1.,1.),at(-1.,0.),at(-1.,-1.),at(0.,-1.),at(1.,-1.),at(1.,0.)];
    let h = FRAC_1_SQRT_2;
    let knots = vec![0.,0.,0.,0.25,0.25,0.5,0.5,0.75,0.75,1.,1.,1.];
    let rim = BSpline::rational(2,knots,poles,vec![1.,h,1.,h,1.,h,1.,h,1.]).unwrap();
    let section = Profile {origin:[0.,0.,-1.],normal:[0.,0.,1.],loops:vec![vec![ProfileEdge::Spline(Arc::new(rim))]],names:Vec::new()};
    let bored = combined(&plate,&prism(&section,0.,10.).unwrap(),Op::Cut,0.).unwrap();
    bored.check(1e-7).unwrap();
    close("bored",volume(&bored),3072.-144.*PI);
}

/// A horn torus's chart pinches at the middle of its box, where every `u` is one point: its
/// parameters scaled by their speed there would shrink `u` to nothing everywhere, and its mesh was
/// slivers across the whole turn (an area of 6.3e6 against 987).  Meshed, its facets bound its
/// volume and cover its area within the sag's share.
#[test]
fn a_horn_torus_meshes_within_its_sag() {
    let centre = [5.,0.,0.];
    let section = Profile {origin:centre,normal:[0.,-1.,0.],names:Vec::new(),
        loops:vec![vec![ProfileEdge::Arc {frame:Frame::new(centre,[0.,-1.,0.],[1.,0.,0.]),r:5.,span:None}]]};
    let horn = gcs_core::brep::build::revolve(&section,[0.;3],[0.,0.,1.],std::f64::consts::TAU).unwrap();
    let m = gcs_core::brep::mesh::mesh(&horn,1e-2,0.1).unwrap();
    let (mut a,mut v) = (0.,0.);
    for t in &m.tris {
        let [p,q,r] = t.map(|i| m.pts[i as usize]);
        let (u,w) = ([q[0]-p[0],q[1]-p[1],q[2]-p[2]],[r[0]-p[0],r[1]-p[1],r[2]-p[2]]);
        let n = [u[1]*w[2]-u[2]*w[1],u[2]*w[0]-u[0]*w[2],u[0]*w[1]-u[1]*w[0]];
        a += 0.5*(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]).sqrt();
        v += (p[0]*(q[1]*r[2]-q[2]*r[1])-p[1]*(q[0]*r[2]-q[2]*r[0])+p[2]*(q[0]*r[1]-q[1]*r[0]))/6.;
    }
    assert!((a-100.*PI*PI).abs() < 1e-2*100.*PI*PI,"meshed area {a} against {}",100.*PI*PI);
    assert!((v-250.*PI*PI).abs() < 1e-2*250.*PI*PI,"meshed volume {v} against {}",250.*PI*PI);
}
