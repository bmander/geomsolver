//! **The arrangement of any number of solids at once** (#735 A4, `brep::boolean::arrangement`):
//! every face of every operand split by all the others in one pass, the cells of the complex they
//! part space into classified by whether each lies in each operand (`members`), and a Boolean the
//! boundary between the cells it keeps and the rest (`selected`). Held to what is known apart from
//! it: the same solid as the pairwise Booleans run one after another (`combined`), by volume and
//! by validity; every cell's membership against the operands read directly; and the arrangement
//! unchanged, cell for cell, whatever the operands' order (the determinism gate).
use crate::brep::block;
use crate::complex::volumes;
use gcs_core::brep::boolean::{arrangement,combined,members,selected,Op};
use gcs_core::brep::build::{disc,prism,sphere};
use gcs_core::brep::props::volume;
use gcs_core::brep::query::{Located,Place};
use gcs_core::brep::topo::Brep;

fn close(a: f64,b: f64,rel: f64) { assert!((a-b).abs() <= rel*b.abs().max(1.),"{a} against {b}"); }

/// Three blocks, each crossing the other two, where three faces of three operands meet at points
/// no edge of any passes; and a ball and a rod crossing them.
fn operands() -> Vec<Brep> {
    vec![block([0.;3],[2.,2.,2.]),block([1.,1.,1.],[3.,3.,3.]),block([0.5,1.5,-1.],[1.5,2.5,1.5]),
        sphere([1.7,0.4,1.2],[0.,0.,1.],0.8).unwrap(),prism(&disc([0.3,0.3,-0.5],[0.,0.,1.],0.4),0.,4.).unwrap()]
}

/// **Three blocks arranged at once are the pairwise Booleans run in turn**: each Boolean a
/// selection of the one arrangement's cells — the union, the first less the others, what all three
/// share, and what lies in exactly one — the same volume as `combined` gives, and a valid boundary.
#[test]
fn three_blocks_arranged_at_once_are_their_booleans_in_turn() {
    let ops = operands();
    let (a,b,c) = (&ops[0],&ops[1],&ops[2]);
    let all = [a,b,c];
    let complex = arrangement(&all,1e-9).unwrap();
    complex.check(1e-9).unwrap();
    let pair = |x: &Brep,y: &Brep,op| combined(x,y,op,0.).unwrap();
    let cases: Vec<(&dyn Fn(&[bool]) -> bool,f64)> = vec![
        (&|m| m.iter().any(|&x| x),volume(&pair(&pair(a,b,Op::Union),c,Op::Union))),
        (&|m| m[0] && !m[1] && !m[2],volume(&pair(&pair(a,b,Op::Cut),c,Op::Cut))),
        (&|m| m.iter().all(|&x| x),volume(&pair(&pair(a,b,Op::Common),c,Op::Common))),
    ];
    for (keep,expected) in cases {
        let s = selected(&all,keep,1e-9).unwrap();
        s.brep.check(1e-9).unwrap();
        close(volume(&s.brep),expected,1e-9);
    }
    // what lies in exactly one: the cells' volumes, read three ways
    let inside = members(&complex,&all,1e-9).unwrap();
    let cells = complex.cells().unwrap();
    let one: f64 = (1..cells.len()).filter(|&k| inside[k].iter().filter(|&&x| x).count() == 1).map(|k| cells[k].volume).sum();
    let s = selected(&all,&|m| m.iter().filter(|&&x| x).count() == 1,1e-9).unwrap();
    s.brep.check(1e-9).unwrap();
    close(volume(&s.brep),one,1e-9);
}

/// **Every cell's membership is what its points read against each operand**: at a point deep
/// inside each cell, each operand's own classification of the point agrees with `members`.
#[test]
fn every_cells_membership_is_its_points_against_each_operand() {
    let ops = operands();
    let all: Vec<&Brep> = ops.iter().collect();
    let complex = arrangement(&all,1e-9).unwrap();
    complex.check(1e-9).unwrap();
    let inside = members(&complex,&all,1e-9).unwrap();
    let located: Vec<Located> = all.iter().map(|b| Located::new(b,1e-9)).collect();
    for c in 1..complex.cells().unwrap().len() {
        let p = complex.inside(c).unwrap().unwrap();
        for (k,l) in located.iter().enumerate() {
            assert_eq!(l.solid_place(p) == Place::In,inside[c][k],"cell {c} at {p:?} against operand {k}");
        }
    }
    // the union of all five is the pairwise union
    let union = all[1..].iter().fold(all[0].clone(),|u,b| combined(&u,b,Op::Union,0.).unwrap());
    let s = selected(&all,&|m| m.iter().any(|&x| x),1e-9).unwrap();
    s.brep.check(1e-9).unwrap();
    close(volume(&s.brep),volume(&union),1e-8);
}

/// **The arrangement is the same whatever the operands' order** (#735's determinism gate): the
/// cells' volumes, and every selection's, the same for the operands in any of several orders.
#[test]
fn the_arrangement_is_the_same_in_any_order_of_its_operands() {
    let ops = operands();
    let base: Vec<&Brep> = ops.iter().collect();
    let first = volumes(&arrangement(&base,1e-9).unwrap());
    for order in [[4,3,2,1,0],[2,0,4,1,3],[1,4,0,3,2]] {
        let ordered: Vec<&Brep> = order.iter().map(|&k| &ops[k]).collect();
        let v = volumes(&arrangement(&ordered,1e-9).unwrap());
        assert_eq!(v.len(),first.len(),"{order:?}: {} cells against {}",v.len(),first.len());
        for (x,y) in v.iter().zip(&first) { close(*x,*y,1e-9); }
    }
}

/// **Coincident faces are one face of the complex, naming each operand**: two blocks stacked share
/// the plane between them over a square, one face with both blocks among its origins, facing
/// opposite ways; their union is one block, its middle gone.
#[test]
fn coincident_faces_are_one_face_naming_both() {
    let (a,b) = (block([0.;3],[1.,1.,1.]),block([0.,0.,1.],[1.,1.,2.]));
    let c = arrangement(&[&a,&b],1e-9).unwrap();
    c.check(1e-9).unwrap();
    let shared: Vec<_> = c.origins.iter().filter(|o| o.len() > 1).collect();
    assert_eq!(shared.len(),1);
    assert!(shared[0].iter().any(|o| o.operand == 1 && !o.alike));
    let u = selected(&[&a,&b],&|m| m[0] || m[1],1e-9).unwrap();
    u.brep.check(1e-9).unwrap();
    close(volume(&u.brep),2.,1e-12);
    assert_eq!(u.brep.faces.len(),10,"each side face whole on each block, the middle gone");
}

/// A block split by a wavy B-spline sheet (traced meetings, the sheet's foot found by search at
/// every step) and a ball beside it: the split's cells, as bits — each cell's volume, faces and
/// vertices — folded into one number.
fn split_fingerprint() -> u64 {
    use gcs_core::brep::nurbs::{interpolate_net,Parametrization};
    let (rows,cols) = (9,9);
    let pts: Vec<[f64;3]> = (0..rows).flat_map(|i| (0..cols).map(move |j| {
        let (x,y) = (-5.+10.*i as f64/(rows-1) as f64,-5.+10.*j as f64/(cols-1) as f64);
        [x,y,0.6*(0.7*x).sin()*(0.5*y+0.3).cos()+0.2]
    })).collect();
    let net = interpolate_net(&pts,rows,cols,Parametrization::ChordLength).unwrap();
    let [[a,z],[c,d]] = net.domain();
    let sheet = gcs_core::brep::build::grid_sheet(&net,&[a,z],&[c,(c+d)/2.,d],&["wave".into()],[false,false]).unwrap();
    let block = block([-3.,-3.,-2.],[3.,3.,2.]);
    let cells = gcs_core::brep::boolean::split(&block,&sheet,1e-6).unwrap();
    let mut h: u64 = 0xcbf29ce484222325;
    let mut fold = |x: u64| { h ^= x; h = h.wrapping_mul(0x100000001b3); };
    for cell in &cells {
        fold(volume(cell).to_bits());
        fold(cell.faces.len() as u64);
        for v in &cell.vertices { for k in 0..3 { fold(v.p[k].to_bits()); } }
    }
    h
}

/// **A split is the same with `SOLVENT_BREP_DEBUG` set and unset, run again and on another
/// thread** (#735's determinism gate): the same bits. A search's warm start is forgotten at every
/// arrangement and every parallel item, and narration keeps it as it was (`util::warm`), so what a
/// thread asked before — or what the narration asked — reaches no result. The debug run is this
/// binary again, in a process of its own (the switch is read once a process).
#[test]
fn a_split_is_the_same_narrated_or_not_on_any_thread() {
    let here = split_fingerprint();
    assert_eq!(split_fingerprint(),here,"run again");
    assert_eq!(std::thread::spawn(split_fingerprint).join().unwrap(),here,"on another thread");
    let out = std::process::Command::new(std::env::current_exe().unwrap())
        .args(["--ignored","--exact","arrangement::the_split_narrated","--nocapture","--test-threads","1"])
        .env("SOLVENT_BREP_DEBUG","1").output().unwrap();
    let said = String::from_utf8_lossy(&out.stdout);
    // (the harness prints the test's name before what it says, on the same line)
    let narrated: u64 = said.lines().find_map(|l| l.split("fingerprint: ").nth(1))
        .and_then(|x| x.split_whitespace().next()).and_then(|x| x.parse().ok())
        .unwrap_or_else(|| panic!("the narrated run said no fingerprint: {said}"));
    assert_eq!(narrated,here,"narrated");
}

/// The split of `a_split_is_the_same_narrated_or_not_on_any_thread`, its fingerprint printed: what
/// that test runs in a process of its own with `SOLVENT_BREP_DEBUG` set.
#[test]
#[ignore = "a tool: the narrated run of a_split_is_the_same_narrated_or_not_on_any_thread"]
fn the_split_narrated() { println!("fingerprint: {}",split_fingerprint()); }

/// A solid of revolution about z whose meridian is the rational quadratic through `poles` (middle
/// weight `w`), closed down the axis at both ends: the hyperboloids a leaning edge turns into
/// (`swept::closed`), as built there.
fn turned_conic(poles: [[f64;3];3],w: f64) -> Brep {
    use gcs_core::brep::build::{revolve,Profile,ProfileEdge};
    let s = gcs_core::brep::nurbs::BSpline::rational(2,vec![0.,0.,0.,1.,1.,1.],poles.to_vec(),vec![1.,w,1.]).unwrap();
    let (a,b) = (poles[0],poles[2]);
    let profile = Profile {origin:[0.;3],normal:[0.,1.,0.],names:vec![],loops:vec![vec![
        ProfileEdge::Line {a:[0.,0.,a[2]],b:a},ProfileEdge::Spline(std::sync::Arc::new(s)),
        ProfileEdge::Line {a:b,b:[0.,0.,b[2]]},ProfileEdge::Line {a:[0.,0.,b[2]],b:[0.,0.,a[2]]}]]};
    revolve(&profile,[0.;3],[0.,0.,1.],std::f64::consts::TAU).unwrap()
}

/// A block standing on the plane through `o` with unit normal `n`, far wider than `domain`.
fn half_space(o: [f64;3],n: [f64;3]) -> Brep { prism(&gcs_core::brep::build::square(o,n,20.),0.,20.).unwrap() }

/// The box the leaning block's part turn is arranged in, and its waisted hyperboloid: the throat
/// of radius 3 at the height where the cylinder of that radius touches it round a circle.
fn waisted() -> (Brep,Brep) {
    (block([-6.,-6.,-5.],[6.,6.,5.]),turned_conic([[3.0867985099372675,0.,0.5330806309078084],
        [2.974760969788629,0.,0.7067599136694217],[3.0075175436766224,0.,0.8759784383632598]],1.012095400943987))
}

/// `operands` arranged within `domain` at `tol`, each operand's cells (`members`) its common with
/// the domain (`combined`), by volume: no cell joined across a face it should part.
fn arranged_exactly(domain: &Brep,operands: &[&Brep],tol: f64) {
    let complex = gcs_core::brep::boolean::arrangement_within(domain,operands,tol).unwrap();
    complex.check(tol).unwrap();
    let cells = complex.cells().unwrap();
    let all: Vec<&Brep> = std::iter::once(domain).chain(operands.iter().copied()).collect();
    let inside = members(&complex,&all,tol).unwrap();
    for k in 1..all.len() {
        let mine: f64 = (1..cells.len()).filter(|&c| inside[c][k]).map(|c| cells[c].volume).sum();
        close(mine,volume(&combined(domain,all[k],Op::Common,tol).unwrap()),1e-6);
    }
}

/// **A plane crossing where two surfaces touch crosses there once** (#735 A3): a cylinder of radius
/// 3 and the hyperboloid whose throat it touches round a circle, cut by a leaning plane through
/// the circle. On the plane the two meetings it lays (an ellipse, a conic) are tangent where it
/// crosses the circle, so their crossing is placed where the hyperboloid crosses the cylinder's
/// meeting with the plane's — or, where the three touch, at the vertex already there — not a
/// hair along the stretch the two tangent curves stay within reach of each other. Through the
/// seams (the plane through them) and off them.
#[test]
fn a_plane_crossing_where_two_surfaces_touch_crosses_there_once() {
    let (domain,waist) = waisted();
    let cylinder = prism(&disc([0.,0.,-0.8759784383632598],[0.,0.,1.],3.),0.,2.*0.8759784383632598).unwrap();
    let through = half_space([0.,-0.2571733555915885,0.704529534635534],[0.,-0.34289780745545134,0.9393727128473789]);
    let off = half_space([-0.4270845953111909,-0.19545849139507201,0.1714489037277257],[-0.8541691906223818,-0.39091698279014403,0.3428978074554514]);
    for tol in [1e-6,1e-7] {
        arranged_exactly(&domain,&[&cylinder,&through,&waist],tol);
        arranged_exactly(&domain,&[&off,&cylinder,&waist],tol);
    }
}

/// **A plane tangent to a hyperboloid at its throat crosses it in two rulings, cut where they
/// cross** (a singular point of a meeting, #735 A2): their crossing, the point the plane touches,
/// a vertex of both. At a tenth of the tolerance the hyperboloid's own rounding shows: the meeting
/// is a hyperbola's two branches passing close by the throat, and the faces round each branch are
/// read where they stand apart, not at its middle beside the throat, where all are nearly tangent.
#[test]
fn a_plane_tangent_to_a_hyperboloid_at_its_throat_is_cut_along_its_rulings() {
    let (domain,waist) = waisted();
    let o = [-1.2484405096414273,2.727892280477045,0.];
    for n in [[0.4161468365471424,-0.9092974268256817,0.],[-0.4161468365471424,0.9092974268256817,0.]] {
        arranged_exactly(&domain,&[&half_space(o,n),&waist],1e-6);
    }
    let wide = turned_conic([[4.065504278801209,0.,0.5330806309078084],[3.9809075908084663,0.,0.705801157314606],
        [4.005641243986119,0.,0.8759784383632598]],1.0068425080938888);
    let o = [-1.6645873461885696,3.637189707302727,0.];
    for n in [[-0.4161468365471424,0.9092974268256817,0.],[0.4161468365471424,-0.9092974268256817,0.]] {
        for tol in [1e-6,1e-7] { arranged_exactly(&domain,&[&half_space(o,n),&wide],tol); }
    }
}

/// **Two hyperboloids meeting at a shallow angle, crossed by a plane, are cut where all three meet**
/// (#735 A3): mirror images of each other, they meet round a circle at a few degrees, a meeting
/// traced only to the tolerance over that angle's sine — the reach a crossing with it is read to
/// (`on_meeting`), then placed on all three surfaces. At a tenth of the tolerance too.
#[test]
fn a_plane_crossing_a_shallow_meeting_is_cut_where_the_three_meet() { crossing_a_shallow_meeting(1e-6) }

/// The same at a tenth of the tolerance, where the traced meeting stands farther than a vertex's
/// reach off the exact one.
#[test]
fn a_plane_crossing_a_shallow_meeting_is_cut_where_the_three_meet_at_a_tenth_of_the_tolerance() { crossing_a_shallow_meeting(1e-7) }

/// `a_plane_crossing_a_shallow_meeting_…` at `tol`.
fn crossing_a_shallow_meeting(tol: f64) {
    let domain = block([-6.,-6.,-5.],[6.,6.,5.]);
    let plane = half_space([0.,-0.2571733555915885,0.704529534635534],[0.,-0.34289780745545134,0.9393727128473789]);
    let lower = turned_conic([[4.005641243986119,0.,-0.8759784383632598],[4.019183976272764,0.,-0.1766743424143582],
        [4.065504278801209,0.,0.5330806309078084]],1.0020367458049737);
    let upper = turned_conic([[4.065504278801209,0.,-0.5330806309078084],[4.019183976272764,0.,0.17667434241435764],
        [4.005641243986119,0.,0.8759784383632598]],1.0020367458049662);
    arranged_exactly(&domain,&[&plane,&lower,&upper],tol);
}

/// **A Boolean lays a touch as an edge and takes it back out where a face is kept on both sides of
/// it** (#735, `boolean::combined`): a rod lying on a block's top touches it along a line, which
/// the Boolean's complex is split along as any arrangement is; the block's top and the rod's side
/// are each kept on both sides of it, so each is whole again in the result, the two faces touching
/// there as they do. United, cut either way, each is its volume and its faces, a valid boundary.
#[test]
fn a_boolean_lays_a_touch_and_heals_it_where_a_face_is_kept_on_both_sides() {
    use std::f64::consts::PI;
    let top = block([-2.,-2.,-1.],[2.,2.,0.]);
    let rod = prism(&disc([-1.,0.,1.],[1.,0.,0.],1.),0.,2.).unwrap();
    for (a,b,op,v,faces) in [(&top,&rod,Op::Union,16.+2.*PI,9),(&rod,&top,Op::Union,16.+2.*PI,9),
        (&top,&rod,Op::Cut,16.,6),(&rod,&top,Op::Cut,2.*PI,3)] {
        let out = combined(a,b,op,0.).unwrap();
        out.check(1e-9).unwrap();
        close(volume(&out),v,1e-9);
        assert_eq!(out.faces.len(),faces,"{op:?}");
    }
}

/// **A hole whose end caps touch the bore and the rim** (the V-twin's crank disc, its set screw's
/// hole along a radius from the bore to the rim): each flat cap is tangent to a cylinder of the
/// disc along a line, laid in the Boolean's complex as an edge and inside the bore's face a slit —
/// both its sides that face's — closed again in the result, at any angle round the disc. The
/// volume the hole takes is its section's run from the bore's tangent plane to the rim,
/// `∫∫ (√(R² − y²) − r) dy dz` over its disc, apart from the build.
#[test]
fn a_hole_whose_caps_touch_the_bore_and_the_rim_is_cut_whole() {
    use std::f64::consts::{FRAC_PI_2,PI};
    let (rim,bore,hole,depth) = (20.,4.,1.5,6.);
    let plate = prism(&disc([0.,0.,-depth/2.],[0.,0.,1.],rim),0.,depth).unwrap();
    let hub = prism(&disc([0.,0.,-depth/2.],[0.,0.,1.],bore),0.,depth).unwrap();
    let body = combined(&plate,&hub,Op::Cut,0.).unwrap();
    // y = hole sin θ across the hole's disc: its chord 2 hole cos θ
    let taken = crate::brep::simpson(-FRAC_PI_2,FRAC_PI_2,400,|th: f64| {
        let y = hole*th.sin();
        2.*hole*hole*th.cos()*th.cos()*((rim*rim-y*y).sqrt()-bore)
    });
    let expected = PI*(rim*rim-bore*bore)*depth-taken;
    for k in 0..12 {
        let a = 0.3+0.47*k as f64;
        let (c,s) = (a.cos(),a.sin());
        let bored = prism(&disc([bore*c,bore*s,0.],[c,s,0.],hole),0.,rim-bore).unwrap();
        let out = combined(&body,&bored,Op::Cut,0.).unwrap();
        out.check(1e-9).unwrap();
        close(volume(&out),expected,1e-9);
        assert_eq!(out.faces.len(),6,"at {a}: the plate's two, the rim, the bore, the hole and its cap");
    }
}

/// **A split lays a touch as a Boolean does, and heals it out of each cell** (`boolean::split`): a
/// block split by a plane across it and by a tube lying on its top, touching it along a line — the
/// line laid as an edge in the complex, and in each cell the block's top whole again. Two cells of
/// half the block each, six faces each, valid.
#[test]
fn a_split_heals_a_touch_out_of_each_cell() {
    use gcs_core::brep::build::{face,square,tube};
    let block = block([-2.,-2.,-1.],[2.,2.,0.]);
    // the tube along y on the top, touching it along x = 0.5; the plane x = 0 across the block
    let across = face(&square([0.,0.,-0.5],[1.,0.,0.],5.)).unwrap();
    let rod = tube([0.5,-3.,1.],[0.,1.,0.],1.,6.,2).unwrap();
    let cells = gcs_core::brep::boolean::split(&block,&across.joined(&rod),1e-9).unwrap();
    assert_eq!(cells.len(),2);
    for c in &cells {
        c.check(1e-9).unwrap();
        close(volume(c),8.,1e-9);
        assert_eq!(c.faces.len(),6,"the block's half, its top whole");
    }
}

/// **A sheet's piece that parts nothing is no cell** (#812, `boolean::split`): a block split by a
/// plane across it and by a square standing wholly inside its upper half, touching nothing — the
/// square's two sides a shell of their own, which was returned as a cell of volume nought with no
/// point inside it, and a 64-tooth gear built whole was refused for it. Two cells of half the block
/// each, six faces each. A square slit into the block from its top is of its cell: one cell of
/// eight faces, the slit's two sides among them.
#[test]
fn a_sheets_piece_parting_nothing_is_no_cell() {
    use gcs_core::brep::build::{face,square};
    let block = block([-2.,-2.,-2.],[2.,2.,2.]);
    let across = face(&square([0.,0.,0.],[0.,0.,1.],3.)).unwrap();
    let fin = face(&square([0.,0.,1.],[1.,0.,0.],0.5)).unwrap();
    let cells = gcs_core::brep::boolean::split(&block,&across.joined(&fin),1e-9).unwrap();
    assert_eq!(cells.len(),2,"{:?}",cells.iter().map(|c| (volume(c),c.faces.len())).collect::<Vec<_>>());
    for c in &cells {
        c.check(1e-9).unwrap();
        close(volume(c),32.,1e-12);
        assert_eq!(c.faces.len(),6,"the block's half, the fin no face of it");
    }
    let slit = face(&square([0.5,0.,2.],[1.,0.,0.],1.)).unwrap();
    let cells = gcs_core::brep::boolean::split(&block,&slit,1e-9).unwrap();
    assert_eq!(cells.len(),1);
    close(volume(&cells[0]),64.,1e-12);
    assert_eq!(cells[0].faces.len(),8,"the block's six, the slit's two sides");
}

/// **A bilinear sheet's block in common with a drum is the drum under the sheet** (#796): the block
/// under the saddle `z = 10 + xy/8` over the square |x|, |y| ≤ 10, floored at z = −5 (four bilinear
/// nets round it, lofted along x), and a drum of radius 8 about z standing on z = 0 share the drum
/// cut by the saddle — its floor, its wall and the saddle's piece over it: 640π by volume, and by
/// area the floor's 64π, the wall's 160π and the saddle's (128π/3)(2√2 − 1). A ray from inside the
/// block met its side at the middle of the stretch the side's box clipped it to, on the face the
/// search for the crossing halved its first box along, and neither half found it: the drum's floor
/// and the wall below the saddle read outside the block, and the selection refused a shell kept on
/// one side and dropped on the other (brep-bench's `geometry/saddle-drum`, refused by 0.4.0). Every
/// point of a grid through the block reads where it stands.
#[test]
fn a_bilinear_sheets_block_in_common_with_a_drum_is_the_drum_under_the_sheet() {
    use gcs_core::brep::build::lofted;
    use gcs_core::brep::nurbs::Net;
    use gcs_core::brep::props::area;
    use std::f64::consts::PI;
    const HALF: f64 = 10.;
    const BELOW: f64 = -5.;
    let saddle = |x: f64,y: f64| 10.+x*y/8.;
    let net = |a: [[f64;3];2],b: [[f64;3];2]| {
        let l = vec![0.,0.,1.,1.];
        Net {du:1,dv:1,uknots:l.clone(),vknots:l,poles:vec![a.to_vec(),b.to_vec()],weights:None}
    };
    let floor = |y: f64| [[-HALF,y,BELOW],[HALF,y,BELOW]];
    let top = |y: f64| [[-HALF,y,saddle(-HALF,y)],[HALF,y,saddle(HALF,y)]];
    let nets = vec![net(floor(-HALF),floor(HALF)),net(floor(HALF),top(HALF)),net(top(HALF),top(-HALF)),net(top(-HALF),floor(-HALF))];
    let under = lofted([0.,0.,0.],[1.,0.,0.],[-HALF,HALF],vec![nets]).unwrap();
    // its inside read as it is, a step clear of its boundary
    let located = Located::new(&under,gcs_core::brep::geom::tolerance_over(under.size(),0.));
    let mut misread = Vec::new();
    for i in 0..13 { for j in 0..13 { for k in 0..13 {
        let p = [-9.+1.5*i as f64,-9.+1.5*j as f64,-6.+2.5*k as f64];
        let depth = (p[2]-BELOW).min(saddle(p[0],p[1])-p[2]);
        if depth.abs() < 0.1 { continue }
        let place = located.solid_place(p);
        if place != if depth > 0. { Place::In } else { Place::Out } { misread.push(format!("{p:?} {place:?}")) }
    } } }
    assert!(misread.is_empty(),"{} points misread: {}",misread.len(),misread.join(", "));
    let drum = prism(&disc([0.,0.,0.],[0.,0.,1.],8.),0.,30.).unwrap();
    let out = combined(&under,&drum,Op::Common,0.).unwrap();
    out.check(1e-9).unwrap();
    assert_eq!(out.faces.len(),3,"the drum's floor, its wall and the saddle over it");
    close(volume(&out),640.*PI,1e-9);
    let a: f64 = out.faces.iter().map(|f| area(&out,f)).sum();
    close(a,224.*PI+128.*PI/3.*(2.*2f64.sqrt()-1.),1e-9);
}
