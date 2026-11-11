//! **A face's contact under a constant twist, in pieces** (#880 Q1b S1, `brep::swept::contacts`,
//! docs/swept-solid-design.md §13): the zero set of `g = n · v` over a face's parameters, for any
//! surface. Held to what is known apart from the reading: every point of a piece on `g = 0` and in its
//! face; no change of sign of `g` on a grid four times finer without a piece beside it (the set read
//! whole); on a face turned about its own axis, each ring's two closed-form roots (`ring_contact`)
//! on a piece, within the tolerance; and a face the motion keeps in its own surface read whole.
use gcs_core::brep::build::{disc,prism,revolve,torus};
use gcs_core::brep::geom::Surface;
use gcs_core::brep::query::Located;
use gcs_core::brep::swept::contacts::{Contact,Piece,face_contact,CELLS};
use gcs_core::brep::topo::Brep;
use gcs_core::brep::trajectory::{Screw,Twist};
use gcs_core::space::distance;

type V = [f64;3];

/// The tolerance pieces are read within, and the model's size it is a part of.
const TOL: f64 = 1e-5;

/// The screw of brep-bench's `swept/torus-screwed-through-rod` and `v-wheel-screwed-through-rod`:
/// about z, rising 30 a turn.
fn rod_screw() -> Screw { Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:1.,advance:30.} }

/// The torus of `torus-screwed-through-rod`: radii 6 and 2 about (11.5, 0, 0), leaning by the helix
/// angle of the point (5.5, 0, 0).
fn leaning_torus() -> Brep {
    let h = (std::f64::consts::TAU*5.5/30.).atan();
    torus([11.5,0.,0.],[0.,h.cos(),-h.sin()],6.,2.).unwrap()
}

/// The V wheel of `v-wheel-screwed-through-rod`: a triangle revolved about the axis through (14, 0, 0)
/// leaning by the helix angle of (4.5, 0, 0): two cones meeting in a sharp rim, and a bore.
fn v_wheel() -> Brep {
    let h = (std::f64::consts::TAU*4.5/30.).atan();
    let axis = [0.,h.cos(),-h.sin()];
    let half = gcs_core::space::scale(axis,2.5*30f64.to_radians().tan());
    let back = [6.5,0.,0.];
    let tri = crate::swept_rings::polygon(&[[4.,0.,0.],gcs_core::space::add(back,half),gcs_core::space::add(back,gcs_core::space::scale(half,-1.))]);
    revolve(&tri,[14.,0.,0.],axis,std::f64::consts::TAU).unwrap()
}

/// Every face's contact under `twist`, its pieces' faces read.
fn contacts(tool: &Brep,twist: &Twist) -> Vec<(usize,Contact)> {
    let located = Located::new(tool,TOL);
    (0..tool.faces.len()).map(|fi| (fi,face_contact(tool,&located,fi,twist,TOL,CELLS))).collect()
}

/// **Each piece is on the contact and in its face**: `g` within rounding of nought at every point
/// (beside its greatest over the face), the point on the surface at its parameters, and in the face.
fn on_contact(tool: &Brep,twist: &Twist,fi: usize,pieces: &[Piece]) {
    let located = Located::new(tool,TOL);
    let s = &tool.faces[fi].surface;
    let [lo,hi] = located.parameter_box(fi);
    let most = (0..=40).flat_map(|i| (0..=40).map(move |j| [lo[0]+(hi[0]-lo[0])*i as f64/40.,lo[1]+(hi[1]-lo[1])*j as f64/40.]))
        .filter_map(|uv| s.normal_velocity(uv,twist)).fold(0f64,|m,x| m.max(x.abs()));
    for p in pieces {
        assert!(p.uv.len() >= 2,"face {fi}: a piece of one point");
        for (k,&uv) in p.uv.iter().enumerate() {
            let g = s.normal_velocity(uv,twist).unwrap();
            assert!(g.abs() <= 1e-9*most,"face {fi}: g = {g:e} at {uv:?} (its greatest {most:e})");
            assert!(distance(s.point(uv),p.points[k]) <= 1e-12*(1.+tool.size()));
            // (an end on the boundary lies on it, read either way)
            let inner = k > 0 && k+1 < p.uv.len();
            if inner { assert!(located.holds_parameters(fi,uv),"face {fi}: a piece's point {uv:?} outside it") }
        }
    }
}

/// **The set read whole**: on a grid four times finer than the reading's, every cell edge inside the
/// face where `g` changes sign has a piece's point within a cell of it.
fn complete(tool: &Brep,twist: &Twist,fi: usize,pieces: &[Piece]) {
    let located = Located::new(tool,TOL);
    let s = &tool.faces[fi].surface;
    let [lo,hi] = located.parameter_box(fi);
    let n = 4*CELLS;
    let cell = [(hi[0]-lo[0])/n as f64,(hi[1]-lo[1])/n as f64];
    let at = |i: usize,j: usize| [lo[0]+cell[0]*i as f64,lo[1]+cell[1]*j as f64];
    let points: Vec<[f64;2]> = pieces.iter().flat_map(|p| p.uv.iter().copied()).collect();
    let mut missed = 0;
    for i in 0..n { for j in 0..=n {
        let (a,b) = (at(i,j),at(i+1,j));
        let (Some(ga),Some(gb)) = (s.normal_velocity(a,twist),s.normal_velocity(b,twist)) else { continue };
        if (ga < 0.) == (gb < 0.) || !located.holds_parameters(fi,a) || !located.holds_parameters(fi,b) { continue }
        let m = [(a[0]+b[0])/2.,(a[1]+b[1])/2.];
        let near = points.iter().any(|q| (q[0]-m[0]).abs() <= 1.5*cell[0]*4. && (q[1]-m[1]).abs() <= 1.5*cell[1]*4.);
        if !near { missed += 1 }
    } }
    assert_eq!(missed,0,"face {fi}: {missed} sign changes of g with no piece beside them");
}

/// **A face turned about its own axis agrees with its rings' closed form**: on rings across the face,
/// each root of `A cos u + B sin u + C` (`Surface::ring_contact`) inside the face within the tolerance
/// of a piece's polyline.
fn rings_agree(tool: &Brep,twist: &Twist,fi: usize,pieces: &[Piece]) -> usize {
    let located = Located::new(tool,TOL);
    let s = &tool.faces[fi].surface;
    let [lo,hi] = located.parameter_box(fi);
    let near_piece = |x: V| pieces.iter().any(|p| p.points.windows(2).any(|w| segment_distance(x,w[0],w[1]) <= TOL));
    let mut read = 0;
    for k in 1..60 {
        let v = lo[1]+(hi[1]-lo[1])*k as f64/60.;
        let Some(ring) = s.ring_contact(v,twist) else { continue };
        let (a,b,c) = (ring.a,ring.b,ring.c);
        let r = (a*a+b*b).sqrt();
        if !(r > c.abs()) { continue }
        let phase = b.atan2(a);
        for sign in [1.,-1.] {
            let u = phase+sign*(-c/r).acos();
            // on the face's branch of the turn
            let mut u = lo[0]+(u-lo[0]).rem_euclid(std::f64::consts::TAU);
            if u > hi[0] { u -= std::f64::consts::TAU }
            if u < lo[0] || !located.holds_parameters(fi,[u,v]) { continue }
            let x = s.point([u,v]);
            assert!(near_piece(x),"face {fi}: the ring at v = {v}'s root u = {u} is on no piece");
            read += 1;
        }
    }
    read
}

fn segment_distance(x: V,a: V,b: V) -> f64 {
    let d = gcs_core::space::sub(b,a);
    let dd = gcs_core::space::dot(d,d);
    let t = if dd > 0. { (gcs_core::space::dot(gcs_core::space::sub(x,a),d)/dd).clamp(0.,1.) } else { 0. };
    distance(x,gcs_core::space::add(a,gcs_core::space::scale(d,t)))
}

/// **The leaning torus of `torus-screwed-through-rod` under its screw**: its one face's contact on
/// the contact, read whole, and on each ring at the closed form's two roots.
#[test]
fn a_leaning_torus_screwed_is_in_contact_where_its_rings_say() {
    let tool = leaning_torus();
    let twist = rod_screw().twist();
    let read = contacts(&tool,&twist);
    let mut agreed = 0;
    for (fi,c) in &read {
        let Contact::Pieces(pieces) = c else { panic!("the torus is in contact all over") };
        assert!(!pieces.is_empty(),"the torus is in contact somewhere");
        on_contact(&tool,&twist,*fi,pieces);
        complete(&tool,&twist,*fi,pieces);
        agreed += rings_agree(&tool,&twist,*fi,pieces);
    }
    assert!(agreed > 100,"{agreed} ring roots held");
}

/// **The V wheel of `v-wheel-screwed-through-rod` under its screw**: each of its cones' and its
/// bore's contact on the contact, read whole, and at its rings' closed-form roots.
#[test]
fn a_v_wheel_screwed_is_in_contact_where_its_rings_say() {
    let tool = v_wheel();
    let twist = rod_screw().twist();
    let mut faces_in_contact = 0;
    for (fi,c) in contacts(&tool,&twist) {
        let Contact::Pieces(pieces) = c else { panic!("face {fi} is in contact all over") };
        if pieces.is_empty() { continue }
        faces_in_contact += 1;
        on_contact(&tool,&twist,fi,&pieces);
        complete(&tool,&twist,fi,&pieces);
        if matches!(tool.faces[fi].surface,Surface::Cone(..) | Surface::Cylinder(..)) { rings_agree(&tool,&twist,fi,&pieces); }
    }
    assert!(faces_in_contact >= 2,"{faces_in_contact} faces in contact");
}

/// **A box under a screw leaning to it**: each face a plane, its contact a line across it
/// (`Surface::plane_contact`), read on the contact and whole, each piece ending on the face's boundary.
#[test]
fn a_box_screwed_is_in_contact_along_lines_across_its_faces() {
    let tool = prism(&gcs_core::brep::build::square([8.,0.,-1.],[0.3,0.2,1.],1.5),0.,2.).unwrap();
    let twist = rod_screw().twist();
    for (fi,c) in contacts(&tool,&twist) {
        let Contact::Pieces(pieces) = c else { panic!("face {fi} is in contact all over") };
        on_contact(&tool,&twist,fi,&pieces);
        complete(&tool,&twist,fi,&pieces);
        for p in &pieces {
            assert_eq!(p.ends,[gcs_core::brep::swept::contacts::End::Boundary;2],"face {fi}: a line across a plane ends on its boundary");
            // straight: every point on the chord between its ends
            let (a,b) = (p.points[0],*p.points.last().unwrap());
            assert!(p.points.iter().all(|&x| segment_distance(x,a,b) <= 1e-9),"face {fi}: a plane's contact is a line");
        }
    }
}

/// **A face the motion keeps in its own surface is read whole**: a drum's side turned about its own
/// axis, `n · v ≡ 0`.
#[test]
fn a_drum_turned_about_its_own_axis_is_in_contact_all_over_its_side() {
    let tool = prism(&disc([0.,0.,0.],[0.,0.,1.],3.),0.,4.).unwrap();
    let twist = Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:1.,advance:0.}.twist();
    let read = contacts(&tool,&twist);
    let side = read.iter().find(|(fi,_)| matches!(tool.faces[*fi].surface,Surface::Cylinder(..))).unwrap();
    assert!(matches!(side.1,Contact::Whole),"the side is in contact all over: {:?}",side.1);
}

