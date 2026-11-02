//! **The arrangement of any number of solids at once** (#735 A4, `brep::boolean::arrangement`):
//! every face of every operand split by all the others in one pass, the cells of the complex they
//! part space into classified by whether each lies in each operand (`members`), and a Boolean the
//! boundary between the cells it keeps and the rest (`selected`). Held to what is known apart from
//! it: the same solid as the pairwise Booleans run one after another (`combined`), by volume and
//! by validity; every cell's membership against the operands read directly; and the arrangement
//! unchanged, cell for cell, whatever the operands' order (the determinism gate).
use crate::brep::block;
use gcs_core::brep::boolean::{arrangement,combined,members,selected,Op};
use gcs_core::brep::build::{disc,prism,sphere};
use gcs_core::brep::props::volume;
use gcs_core::brep::query::{Located,Place};
use gcs_core::brep::topo::Brep;

fn close(a: f64,b: f64,rel: f64) { assert!((a-b).abs() <= rel*b.abs().max(1.),"{a} against {b}"); }

/// The cells' volumes, smallest first.
fn volumes(c: &gcs_core::brep::complex::Complex) -> Vec<f64> {
    let mut v: Vec<f64> = c.cells().unwrap()[1..].iter().map(|k| k.volume).collect();
    v.sort_by(f64::total_cmp);
    v
}

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
