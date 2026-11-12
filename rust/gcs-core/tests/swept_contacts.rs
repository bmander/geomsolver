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
use gcs_core::space::{distance,segment_distance};

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
pub(crate) fn v_wheel() -> Brep {
    let h = (std::f64::consts::TAU*4.5/30.).atan();
    let axis = [0.,h.cos(),-h.sin()];
    let half = gcs_core::space::scale(axis,2.5*30f64.to_radians().tan());
    let back = [6.5,0.,0.];
    let tri = crate::swept_rings::polygon(&[[4.,0.,0.],gcs_core::space::add(back,half),gcs_core::space::add(back,gcs_core::space::scale(half,-1.))]);
    revolve(&tri,[14.,0.,0.],axis,std::f64::consts::TAU).unwrap()
}

/// Every face's contact under `twist`, its pieces' faces read.
fn contacts(tool: &Brep,located: &Located,twist: &Twist) -> Vec<(usize,Contact)> {
    (0..tool.faces.len()).map(|fi| (fi,face_contact(tool,located,fi,twist,TOL,CELLS))).collect()
}

/// **Each piece is on the contact and in its face**: `g` within rounding of nought at every point
/// (beside its greatest over the face), the point on the surface at its parameters, and in the face.
fn on_contact(tool: &Brep,located: &Located,twist: &Twist,fi: usize,pieces: &[Piece]) {
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
fn complete(tool: &Brep,located: &Located,twist: &Twist,fi: usize,pieces: &[Piece]) {
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
fn rings_agree(tool: &Brep,located: &Located,twist: &Twist,fi: usize,pieces: &[Piece]) -> usize {
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
            let mut u = gcs_core::brep::geom::around(u,lo[0],std::f64::consts::TAU);
            if u > hi[0] { u -= std::f64::consts::TAU }
            if u < lo[0] || !located.holds_parameters(fi,[u,v]) { continue }
            let x = s.point([u,v]);
            assert!(near_piece(x),"face {fi}: the ring at v = {v}'s root u = {u} is on no piece");
            read += 1;
        }
    }
    read
}

/// **The leaning torus of `torus-screwed-through-rod` under its screw**: its one face's contact on
/// the contact, read whole, and on each ring at the closed form's two roots.
#[test]
fn a_leaning_torus_screwed_is_in_contact_where_its_rings_say() {
    let tool = leaning_torus();
    let twist = rod_screw().twist();
    let located = Located::new(&tool,TOL);
    let read = contacts(&tool,&located,&twist);
    let mut agreed = 0;
    for (fi,c) in &read {
        let Contact::Pieces(pieces) = c else { panic!("the torus is in contact all over") };
        assert!(!pieces.is_empty(),"the torus is in contact somewhere");
        on_contact(&tool,&located,&twist,*fi,pieces);
        complete(&tool,&located,&twist,*fi,pieces);
        agreed += rings_agree(&tool,&located,&twist,*fi,pieces);
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
    let located = Located::new(&tool,TOL);
    for (fi,c) in contacts(&tool,&located,&twist) {
        let Contact::Pieces(pieces) = c else { panic!("face {fi} is in contact all over") };
        if pieces.is_empty() { continue }
        faces_in_contact += 1;
        on_contact(&tool,&located,&twist,fi,&pieces);
        complete(&tool,&located,&twist,fi,&pieces);
        if matches!(tool.faces[fi].surface,Surface::Cone(..) | Surface::Cylinder(..)) { rings_agree(&tool,&located,&twist,fi,&pieces); }
    }
    assert!(faces_in_contact >= 2,"{faces_in_contact} faces in contact");
}

/// **A box under a screw leaning to it**: each face a plane, its contact a line across it
/// (`Surface::plane_contact`), read on the contact and whole, each piece ending on the face's boundary.
#[test]
fn a_box_screwed_is_in_contact_along_lines_across_its_faces() {
    let tool = prism(&gcs_core::brep::build::square([8.,0.,-1.],[0.3,0.2,1.],1.5),0.,2.).unwrap();
    let twist = rod_screw().twist();
    let located = Located::new(&tool,TOL);
    for (fi,c) in contacts(&tool,&located,&twist) {
        let Contact::Pieces(pieces) = c else { panic!("face {fi} is in contact all over") };
        on_contact(&tool,&located,&twist,fi,&pieces);
        complete(&tool,&located,&twist,fi,&pieces);
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
    let located = Located::new(&tool,TOL);
    let read = contacts(&tool,&located,&twist);
    let side = read.iter().find(|(fi,_)| matches!(tool.faces[*fi].surface,Surface::Cylinder(..))).unwrap();
    assert!(matches!(side.1,Contact::Whole),"the side is in contact all over: {:?}",side.1);
}


/// **A piece carried over the roll is a sheet read back exactly** (#880 Q1b S3, `contacts::carried`,
/// `Blend::carried_uv`): the leaning torus's contact under its screw over `torus-screwed-through-rod`'s
/// roll, carried 456° — a closed curve more than a turn — cut into stretches (`stretches`), each a
/// sheet inverted at its own points to rounding (from the screw's invariants: radius and height along
/// the helix), the two end poses after them, and each two neighbouring stretches given to meet along
/// the rail they share, lying on both sheets (every two given their meeting:
/// `every_two_sheets_of_one_carry_are_given_the_paths_they_share`).
#[test]
fn a_contact_carried_over_the_roll_is_inverted_exactly_at_its_own_points() {
    use gcs_core::brep::swept::contacts::candidates;
    use gcs_core::brep::trajectory::Trajectory;
    let tool = leaning_torus();
    let path = Trajectory::Twist(rod_screw());
    let roll = [(-108f64).to_radians(),348f64.to_radians()];
    let (cands,given) = candidates(&tool,&path,roll,TOL).unwrap();
    let sheets = cands.len()-2;
    assert!(sheets >= 4,"the closed contact in four stretches at least: {sheets}");
    assert!(cands[..sheets].iter().all(|c| matches!(c.faces[..],[ref f] if matches!(f.surface,Surface::Blend(..)))),"each stretch one sheet");
    assert!(cands[sheets..].iter().all(|c| matches!(c.faces[0].surface,Surface::Torus(..))),"the end poses last");
    assert_eq!(given.len(),sheets*(sheets-1)/2+2*sheets,"every two sheets given their meeting, and each sheet its meeting with each end pose");
    for c in &cands[..sheets] {
        let sheet = &c.faces[0].surface;
        for i in 0..=10 { for j in 0..=10 {
            let uv = [i as f64/10.,j as f64/10.];
            let p = sheet.point(uv);
            let back = sheet.inverse(p);
            assert!(distance(sheet.point(back),p) <= 1e-9,"a sheet at {uv:?} read back at {back:?}");
        } }
    }
    // each neighbour's rail, its given's first curve
    let rails = given.iter().filter(|g| g.curves.first().is_some_and(|c| {
        let gcs_core::brep::geom::Curve::Iso(_,u) = c else { return false };
        *u == 1. || *u == 0.
    }));
    assert!(rails.clone().count() >= sheets,"each two neighbours of a closed piece given their rail");
    for (k,g) in rails.enumerate() {
        for c in &g.curves[..1] {
            for s in &g.surfaces {
                let off = (0..=100).map(|i| s.implicit(c.point(i as f64/100.)).abs()).fold(0f64,f64::max);
                assert!(off <= 1e-7,"rail {k} off a sheet it is given on by {off:e}");
            }
        }
    }
}


/// **A branch arriving at a touch along an edge joins it** (#880 Q1b, `boolean::landing`): the
/// stretch of the leaning torus's carried contact that runs back through the start pose touches
/// that pose along its rim, and its re-entry into it — a transversal branch of the two's meeting —
/// runs to the rim, where the two part too slowly for a march to land. Followed down to the rim, it
/// ends there, one curve; before, it stopped short, the stop read as a touch of its own and its
/// other half-branches traced along the rim, and the arrangement's pieces did not close.
#[test]
fn a_contact_carried_back_through_its_start_pose_is_arranged_with_it() {
    use gcs_core::brep::boolean::arrangement;
    use gcs_core::brep::swept::contacts::candidates;
    use gcs_core::brep::trajectory::Trajectory;
    let tool = leaning_torus();
    let path = Trajectory::Twist(rod_screw());
    let roll = [(-108f64).to_radians(),348f64.to_radians()];
    let (cands,_) = candidates(&tool,&path,roll,TOL).unwrap();
    let start = &cands[cands.len()-2];
    if let Err(e) = arrangement(&[&cands[8],start],TOL) { panic!("the stretch through the start pose arranged with it: {e}") }
}


/// **Every two sheets of one carry are given the paths they share** (#880 Q1b, `contacts::candidates`'s
/// `shared_paths`): two sheets a screw sweeps from two curves meet only along whole helices of it,
/// where the curves cross — or touch — in its invariants, so each pair's meeting is given and not
/// sought. Of the leaning torus's stretches, every path given lies in both sheets (in each sheet's own
/// invariants, `Blend::carried_gap`: a crossing to rounding, a touch within the sections' fit, a
/// tenth of the tolerance). Its contact crosses itself at a double point (a saddle of the contact,
/// on the crossing of the torus's seams, where the grid's node hid it): cut there
/// (`face_contact`), four stretches end on that point's path, every two of them tangent there in
/// the invariants as every two branches through one point of the tool are, and every two are given
/// that path once — where Newton alone read a touch as five paths a hair apart, and the sections'
/// own fit, parting so slowly from it, crossed again 7e-3 along — and the four arrange with them.
#[test]
fn every_two_sheets_of_one_carry_are_given_the_paths_they_share() {
    use gcs_core::brep::boolean::arrangement_given;
    use gcs_core::brep::build::{prism,square};
    use gcs_core::brep::geom::Curve;
    use gcs_core::brep::swept::contacts::candidates;
    use gcs_core::brep::trajectory::Trajectory;
    let tool = leaning_torus();
    let path = Trajectory::Twist(rod_screw());
    let roll = [(-108f64).to_radians(),348f64.to_radians()];
    let (cands,given) = candidates(&tool,&path,roll,TOL).unwrap();
    let blend = |s: &Surface| match s { Surface::Blend(_,b) => b.clone(),_ => panic!("a sheet") };
    let mut paths = 0;
    for g in given.iter().filter(|g| g.surfaces.iter().all(|s| matches!(s,Surface::Blend(..)))) {
        for c in &g.curves {
            let Curve::Iso(..) = c else { panic!("a path of the carry") };
            paths += 1;
            for s in &g.surfaces {
                for k in 0..=8 {
                    let (gap,_) = blend(s).carried_gap(c.point(k as f64/8.)).unwrap();
                    assert!(gap <= 0.1*TOL,"a given path {gap:e} off a sheet's paths");
                }
            }
        }
    }
    let sheets = cands.len()-2;
    assert!(paths > sheets,"the sheets cross back over one another, beyond their rails: {paths} paths");
    // the double point: where four sections end
    let ends: Vec<(usize,[f64;3])> = (0..sheets).flat_map(|k| { let a = blend(&cands[k].faces[0].surface).a.clone(); [(k,a.point(0.)),(k,a.point(1.))] }).collect();
    let at = ends.iter().map(|&(_,p)| p).find(|&p| ends.iter().filter(|&&(_,q)| distance(p,q) <= 1e-9).count() == 4)
        .expect("the contact cut at its double point, four stretches ending there");
    let four: Vec<usize> = ends.iter().filter(|&&(_,q)| distance(at,q) <= 1e-9).map(|&(k,_)| k).collect();
    let key = |s: &Surface| s.point([0.37,0.61]);
    let of = |i: usize,j: usize| given.iter().find(|g| {
        let (a,b) = (key(&g.surfaces[0]),key(&g.surfaces[1]));
        let (x,y) = (key(&cands[i].faces[0].surface),key(&cands[j].faces[0].surface));
        (a == x && b == y) || (a == y && b == x)
    }).expect("every two sheets given their meeting").clone();
    let mut among = Vec::new();
    for (n,&i) in four.iter().enumerate() { for &j in &four[n+1..] {
        let g = of(i,j);
        let on = g.curves.iter().filter(|c| matches!(c,Curve::Iso(_,u) if *u == 0. || *u == 1.)).count();
        assert_eq!(on,1,"stretches {i} and {j} given the double point's path once: {:?}",g.curves.iter().map(|c| match c { Curve::Iso(_,u) => *u,_ => -1. }).collect::<Vec<_>>());
        among.push(g);
    } }
    let domain = prism(&square([0.,0.,-30.],[0.,0.,1.],40.),0.,90.).unwrap();
    let operands: Vec<_> = four.iter().map(|&k| &cands[k]).collect();
    if let Err(e) = arrangement_given(&domain,&operands,&among,TOL) { panic!("the double point's four stretches arranged with their paths: {e}") }
}


/// **Each sheet is given its whole meeting with the tool at each end of the roll** (#880 Q1b,
/// `contacts::pose_meeting`): the sheet touches the pose its own end is carried to along that end's
/// rim, its distance from it `(v − e)² h` across, so what of it passes back into the tool there is
/// the zero set of `h`, read in the sheet's own parameters — a curve where a march reached the rim
/// only as a step it could not land. Every curve given lies on both surfaces within the
/// tolerance; and at the contact's double point, where every sheet and the pose are tangent, a
/// branch leaves the corner of a stretch ending there from the corner, to rounding, not from a hair
/// beside it on the rim or the rail.
#[test]
fn each_sheet_is_given_its_meeting_with_the_tool_at_each_end() {
    use gcs_core::brep::swept::contacts::candidates;
    use gcs_core::brep::trajectory::Trajectory;
    let tool = leaning_torus();
    let path = Trajectory::Twist(rod_screw());
    let roll = [(-108f64).to_radians(),348f64.to_radians()];
    let (cands,given) = candidates(&tool,&path,roll,TOL).unwrap();
    let sheets = cands.len()-2;
    let poses = [&cands[sheets].faces[0].surface,&cands[sheets+1].faces[0].surface];
    let posed: Vec<_> = given.iter().filter(|g| matches!(g.surfaces[1],Surface::Torus(..))).collect();
    assert_eq!(posed.len(),2*sheets,"each sheet given its meeting with each end pose");
    let mut worst = 0f64;
    for g in &posed {
        for c in &g.curves {
            for k in 0..=32 {
                let p = c.point(k as f64/32.);
                for s in &g.surfaces { worst = worst.max(s.implicit(p).abs()); }
            }
        }
    }
    assert!(worst <= TOL,"the meetings given lie on both surfaces: {worst:e} off");
    assert!(posed.iter().any(|g| !g.curves.is_empty()),"the sheets pass back into the tool at its ends");
    // the four stretches ending at the double point: a branch leaving each pose's corner from it
    let blend = |s: &Surface| match s { Surface::Blend(_,b) => b.clone(),_ => panic!("a sheet") };
    let ends: Vec<[f64;3]> = (0..sheets).flat_map(|k| { let a = blend(&cands[k].faces[0].surface).a.clone(); [a.point(0.),a.point(1.)] }).collect();
    let at = *ends.iter().find(|&&p| ends.iter().filter(|&&q| distance(p,q) <= 1e-9).count() == 4).expect("the double point");
    let mut from_corner = 0;
    for g in &posed {
        let b = blend(&g.surfaces[0]);
        let e = if g.surfaces[1] == *poses[0] { 0. } else { 1. };
        let corner = b.carry.at(at,e);
        from_corner += g.curves.iter().filter(|c| [c.point(0.),c.point(1.)].iter().any(|&p| distance(p,corner) <= 1e-9)).count();
    }
    assert!(from_corner >= 4,"branches leave the double point's corners from them exactly: {from_corner}");
}


/// **The rod grooved by the leaning torus screwed along it** (#880 Q1b S4's gate; brep-bench's
/// `swept/torus-screwed-through-rod`): the candidates trimmed by membership into the closed sweep,
/// judged by the sweep's definition, and cut from the rod, held to the groove's volume computed
/// apart from the build — the groove's section square to the axis the same at every height, turned,
/// so the cut is 20 A, A the rod's disc covered at radius ρ by the helical heights `z − c φ` the
/// torus's exact signed distance reaches, over `c` (`scratch: groove.py`, Gauss–Legendre in ρ over
/// [3.5, 5], each height's depth by golden section over a turn, converged at 12 and 24 points to
/// 1460.6300031715 left of the rod). Built to 1e-5 and to 1e-6, each within ten tolerances of it.
#[test]
fn a_rod_grooved_by_a_leaning_torus_screwed_along_it_is_its_volume() {
    use gcs_core::brep::boolean::{combined,Op};
    use gcs_core::brep::swept::contacts::candidates;
    use gcs_core::brep::swept::trim::{trimmed,Trim};
    let Case {blank,tool,path,roll,bar,volume:known,..} = torus_case();
    let rod = blank.unwrap();
    let size = tool.size();
    for tol in [TOL,0.1*TOL] {
        let (cands,given) = candidates(&tool,&path,roll,tol).unwrap();
        let (swept,_) = trimmed(&tool,&path,roll,&cands,&given,bar,tol,Trim {resolution:size/32.,hair:4.*tol})
            .unwrap_or_else(|e| panic!("the sweep trimmed to {tol:e}: {e:?}"));
        let grooved = combined(&rod,&swept,Op::Cut,tol).unwrap();
        let volume = gcs_core::brep::props::volume(&grooved);
        assert!((volume-known).abs() <= 10.*tol,"to {tol:e}, the grooved rod's volume {volume}, apart from the build {known}");
    }
}


/// **A sharp edge's fan in contact, in stretches** (#880 Q1b S2's gate, `contacts::fan_runs`): on
/// each sharp rim of the V wheel and of the leaning drum under their screws, each stretch ends where
/// `n · v` of one of the edge's two faces is nought (within the tolerance, against the speed), the
/// two faces' `n · v` differ in sign inside every stretch and agree between them, on a grid a
/// hundred times finer than the stretches; and each tool has a stretch.
#[test]
fn a_sharp_edges_fan_is_in_contact_between_its_faces_contacts() {
    use gcs_core::brep::swept::contacts::{fan_runs,sharp_edges};
    use gcs_core::brep::topo::EdgeCurve;
    for Case {what,tool,path,..} in [v_wheel_case(),drum_case()] {
        let twist = path.screw().unwrap().twist();
        let mut stretches = 0;
        for (e,[f1,f2]) in sharp_edges(&tool).unwrap() {
            let edge = &tool.edges[e];
            let EdgeCurve::Curve(curve) = &edge.curve else { continue };
            let [t0,t1] = edge.t;
            let at = |t: f64| curve.point(if t > t1 { t-(t1-t0) } else { t });
            let nv = |f: usize,t: f64| { let p = at(t); gcs_core::space::dot(tool.faces[f].outward(p).unwrap(),twist.velocity(p)) };
            let speed = |t: f64| gcs_core::space::norm(twist.velocity(at(t)));
            let runs = fan_runs(&tool,e,[f1,f2],&twist);
            let closed = edge.closed();
            for run in &runs {
                for t in *run {
                    if !closed && (t == t0 || t == t1) { continue }
                    let least = nv(f1,t).abs().min(nv(f2,t).abs());
                    assert!(least <= TOL*speed(t),"{what}: a stretch of edge {e} ends at {t} where n · v is {least}");
                }
            }
            stretches += runs.len();
            let inside = |t: f64| runs.iter().any(|r| (r[0]..=r[1]).contains(&t) || (r[0]..=r[1]).contains(&(t+(t1-t0))));
            let n = 51_200;
            for k in 0..=n {
                let t = t0+(t1-t0)*k as f64/n as f64;
                let fan = nv(f1,t)*nv(f2,t);
                let near = runs.iter().flatten().any(|&r| (r-t).abs() <= 1e-3*(t1-t0) || (r-(t1-t0)-t).abs() <= 1e-3*(t1-t0));
                if near { continue }
                assert_eq!(fan < 0.,inside(t),"{what}: edge {e} at {t}, the faces' n · v {} and {}",nv(f1,t),nv(f2,t));
            }
        }
        assert!(stretches > 0,"{what}: a fan in contact");
    }
}

/// One of S4's three bench cases (#880 Q1b): the blank, the tool, its screw and roll, the mesh's bar,
/// and the volume of the blank less the sweep computed apart from the build, with how closely that
/// reading is known.
struct Case { what: &'static str,blank: Option<Brep>,tool: Brep,path: gcs_core::brep::trajectory::Trajectory,roll: [f64;2],bar: f64,
    volume: f64,known: f64 }

/// The rod of `torus-screwed-through-rod` and `v-wheel-screwed-through-rod`.
fn rod() -> Brep { prism(&disc([0.;3],[0.,0.,1.],5.),0.,20.).unwrap() }

/// `torus-screwed-through-rod`: the groove's volume by the helical heights the torus reaches at
/// each radius (`a_rod_grooved_by_a_leaning_torus_screwed_along_it_is_its_volume`).
fn torus_case() -> Case {
    Case {what:"the torus",blank:Some(rod()),tool:leaning_torus(),path:gcs_core::brep::trajectory::Trajectory::Twist(rod_screw()),
        roll:[(-108f64).to_radians(),348f64.to_radians()],bar:0.01,volume:1460.6300031715,known:1e-9}
}

/// `v-wheel-screwed-through-rod`: each point of the rod cut where its helical invariants
/// `(ρ, z − c φ)` lie in the wheel's shadow, the radii integrated by Gauss–Legendre — 1554.3809694 at
/// forty points, moving by 1e-5 a doubling, so known to a few millionths.
fn v_wheel_case() -> Case {
    Case {what:"the V wheel",blank:Some(rod()),tool:v_wheel(),path:gcs_core::brep::trajectory::Trajectory::Twist(rod_screw()),
        roll:[(-144f64).to_radians(),384f64.to_radians()],bar:0.01,volume:1554.3809694,known:1e-5}
}

/// `drum-leaning-to-a-screw`: a cylinder of radius 0.8 and length 3 about (10, 0, 0), its axis
/// leaning 25° toward +y, screwed about z rising 1 a turn from −140° to 140° through a ring sector
/// (radii 8 to 12, z from −1 to 1, 0° to 60°). The sector less the sweep, apart from the build: the
/// strip of helical heights the drum's shadow covers at each radius integrated by eight-point
/// Gauss–Legendre over sixteen pieces of each panel between the strip's kinks (where its shadow
/// splits and where its ends cross the window's), each panel's ends tamed by a cosine (over eight
/// pieces, 2e-10 less).
fn drum_case() -> Case {
    let section = crate::swept_rings::polygon(&[[8.,0.,-1.],[12.,0.,-1.],[12.,0.,1.],[8.,0.,1.]]);
    let sector = revolve(&section,[0.;3],[0.,0.,1.],std::f64::consts::FRAC_PI_3).unwrap();
    let (s,c) = 25f64.to_radians().sin_cos();
    let along = [0.,s,c];
    let drum = prism(&disc(gcs_core::space::sub([10.,0.,0.],gcs_core::space::scale(along,1.5)),along,0.8),0.,3.).unwrap();
    let turn = 140f64.to_radians();
    Case {what:"the drum",blank:Some(sector),tool:drum,
        path:gcs_core::brep::trajectory::Trajectory::Twist(Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:1.,advance:1.}),
        roll:[-turn,turn],bar:0.002,volume:50.2500186377,known:1e-9}
}

/// The case cut through the front door (`swept::closed::cut_from`, or `closed` where it has no
/// blank) at `tol`, held to its volume within ten tolerances or what it is known to, and written as a
/// STEP file that reads back as the solid (the S4 gate's closed file).
fn cut_through_the_front_door(case: &Case,tol: f64) {
    let cut = match &case.blank {
        Some(blank) => gcs_core::brep::swept::closed::cut_from(blank,&case.tool,&case.path,case.roll,case.bar,tol),
        None => gcs_core::brep::swept::closed::closed(&case.tool,&case.path,case.roll,case.bar,tol),
    }.unwrap_or_else(|e| panic!("{}: {e:?}",case.what));
    let volume = gcs_core::brep::props::volume(&cut);
    let within = (10.*tol).max(case.known);
    assert!((volume-case.volume).abs() <= within,"{}: the cut's volume {volume}, apart from the build {}",case.what,case.volume);
    let text = gcs_core::brep::step::write(&cut,"swept",tol).unwrap();
    gcs_core::brep::step_check::verify(&text,&gcs_core::brep::step_check::Solid::of(&cut))
        .unwrap_or_else(|e| panic!("{}: its STEP: {e}",case.what));
}

/// **The front door takes a torus screwed along a rod** (#880 Q1b S4, `swept::closed::cut_from`):
/// the closed class refuses a torus by what it is (`Curved`), and the contact's road — candidates,
/// trimmed by membership, judged, and cut from the blank — builds it, to the groove's volume computed
/// apart from the build, and writes it as a STEP file that reads back as the solid.
#[test]
fn a_torus_screwed_through_a_rod_is_cut_through_the_front_door() { cut_through_the_front_door(&torus_case(),TOL) }

/// **The front door takes a V wheel screwed through a rod** (#880 Q1b S2 and S4, brep-bench's
/// `swept/v-wheel-screwed-through-rod`): the wheel's two cones meet in a sharp rim, whose stretch
/// where the cones' `n·v` differ in sign is the fan, carried as a sheet of its own.
#[test]
fn a_v_wheel_screwed_through_a_rod_is_cut_through_the_front_door() { cut_through_the_front_door(&v_wheel_case(),TOL) }

/// **The front door takes a drum leaning to a screw** (#880 Q1b S2 and S4, brep-bench's
/// `swept/drum-leaning-to-a-screw`): both rims' fans are carried as sheets.
#[test]
fn a_drum_leaning_to_a_screw_is_cut_through_the_front_door() { cut_through_the_front_door(&drum_case(),TOL) }

/// **S4's three cases at the kernel's own tolerance** (`geom::tolerance`, a billionth of the size,
/// what the bench builds them at): each cut through the front door, to its volume and a STEP file.
/// At it the leaning torus's sheet beside its rim read its rate across the rim by five-point
/// differences of `F` a thousandth of the roll apart, 2.7e-5 for its own 1e-8, and its branch there
/// stood 4e-8 off the tool (`pose_meeting`).
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about a minute: three screwed sweeps at the kernel's tolerance")]
fn the_three_cases_are_cut_at_the_kernels_own_tolerance() {
    for case in [torus_case(),v_wheel_case(),drum_case()] {
        let tol = gcs_core::brep::geom::tolerance(case.blank.as_ref().map_or(0.,|b| b.size()).max(case.tool.size()));
        cut_through_the_front_door(&case,tol);
    }
}

/// The turn of brep-bench's spun and swung cases: about z, a radian a radian of the roll.
fn turn_about_z() -> gcs_core::brep::trajectory::Trajectory {
    gcs_core::brep::trajectory::Trajectory::Twist(Screw {origin:[0.;3],axis:[0.,0.,1.],ratio:1.,advance:0.})
}

/// `rod-spun-about-a-line-through-it`: the rod `x ∈ [−3, 5]`, `y² + z² ≤ 1/4`, turned 120° about z.
/// Apart from the build: at height `z` (`w = √(r² − z²)`) and radius `ρ`, with `α = asin(w/ρ)`, the turn
/// sweeps the whole circle out to `ρ = 2w` (the two arcs' sweeps meeting there), then `2(2α + T)`
/// while the arcs about 0 and π both reach, out to `√(9 + w²)`, then `2α + T` out to `√(25 + w²)`;
/// with `∫ρ asin(w/ρ) dρ = ρ²/2 asin(w/ρ) + w/2 √(ρ² − w²)` each slice's area is closed, and the
/// height is integrated by Gauss–Legendre in `θ`, `z = r sin θ`: 41.685706292608 at sixteen points
/// and at a hundred and twenty-eight.
fn rod_spun_case() -> Case {
    Case {what:"the spun rod",blank:None,tool:prism(&disc([-3.,0.,0.],[1.,0.,0.],0.5),0.,8.).unwrap(),path:turn_about_z(),
        roll:[0.,120f64.to_radians()],bar:0.005,volume:41.685706292608,known:1e-9}
}

/// `ball-swung-across-its-own-center-line`: the ball of radius 2 about (1, 0, 0) turned 120° about z.
/// Apart from the build: at `(ρ, z)` its points on the circle are `|φ| ≤ acos k`, `k = (ρ² + z² − 3)/(2ρ)`,
/// and the turn sweeps `min(2π, 2 acos k + T)` of it; the volume is the integral of `ρ` times that,
/// split where the sweep closes the circle and where the slice leaves the axis, each panel's ends
/// tamed by a cosine (square roots at the ball's rim): 59.333194348871 from 32 points to 128.
fn ball_swung_case() -> Case {
    Case {what:"the swung ball",blank:None,tool:gcs_core::brep::build::sphere([1.,0.,0.],[0.,0.,1.],2.).unwrap(),path:turn_about_z(),
        roll:[0.,120f64.to_radians()],bar:0.01,volume:59.333194348871,known:1e-9}
}

/// **A rod spun about a line through it** (#880 Q1b S5, brep-bench's
/// `swept/rod-spun-about-a-line-through-it`): under the turn each sheet lies on the surface of
/// revolution of its section's meridian (`meridian_surface`) — the top and bottom lines' sectors on
/// planes, the end rims' fans on balls, both halves of each rim on one — the end discs' lines and the
/// middle ring, whose paths bend inward, buried and left out; the lines' sheets close at a pole on
/// the axis, a degenerate edge.
#[test]
fn a_rod_spun_about_a_line_through_it_is_swept_through_the_front_door() { cut_through_the_front_door(&rod_spun_case(),TOL) }

/// **A ball swung across its own centre line** (#880 Q1b S5, brep-bench's
/// `swept/ball-swung-across-its-own-center-line`): its contact, the great circle in the plane through
/// the axis and its centre, is cut where it crosses the axis; the arc beyond it buried, the rest on
/// one spindle torus closing at the axis at both ends — the ball drawn about the plane's normal, its
/// poles off its contact.
#[test]
fn a_ball_swung_across_its_own_centre_line_is_swept_through_the_front_door() { cut_through_the_front_door(&ball_swung_case(),TOL) }
