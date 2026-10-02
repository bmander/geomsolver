//! Material fields of ordinary prisms: convex and concave sections, arcs, placed
//! copies and through cuts, checked by sign against closed forms and the faceted
//! kernel, and for the one-Lipschitz bound every field promises.
use gcs_core::{interval::Interval as I,program,solid::{MaterialField,SpatialField,REPORT_UNIT},
    solve,syntax};

const BLOCK: &str = "unit mm
use std
a := point hint(x: 0, y: 0)
b := point hint(x: 60, y: 0)
c := point hint(x: 60, y: 40)
d := point hint(x: 0, y: 40)
horizontal (ab := line(a, b)) -> vertical (bc := line(b, c)) -> horizontal (cd := line(c, d)) -> vertical (da := line(d, a)) -> close
a distance(60) b
b distance(40) c
ground a
sec := face(ab, bc, cd, da)
block := solid(sec, depth: 30mm)
";

const TURN: &str = "o := point hint(x: 0, y: 0)
z := point hint(x: 0, y: 10)
ground o
ground z
axis := line(o, z)
turn := motion(about: axis)
moved := solid(block, under: turn, at: 40deg)
";

const BORE: &str = "h := point hint(x: 30, y: 20)
ground h
hole := circle(center: h) hint(r: 8)
radius(8) hole
hole_f := face(hole)
body := solid(block)
bore := solid(hole_f, through: body)
bore cut body
";

const ELL: &str = "unit mm
use std
a := point hint(x: 0, y: 0)
b := point hint(x: 50, y: 0)
c := point hint(x: 50, y: 20)
d := point hint(x: 20, y: 20)
e := point hint(x: 20, y: 50)
f := point hint(x: 0, y: 50)
horizontal (ab := line(a, b)) -> vertical (bc := line(b, c)) -> horizontal (cd := line(c, d)) -> vertical (de := line(d, e)) -> horizontal (ef := line(e, f)) -> vertical (fa := line(f, a)) -> close
a distance(50) b
b distance(20) c
c distance(30) d
d distance(30) e
ground a
sec := face(ab, bc, cd, de, ef, fa)
ell := solid(sec, depth: 12mm)
";

// A block with a semicircular notch bitten out of its top edge: the arc turns
// against the loop, and the face enters it by its end.
const NOTCH: &str = "unit mm
use std
a := point hint(x: 0, y: 0)
b := point hint(x: 60, y: 0)
c := point hint(x: 60, y: 30)
d := point hint(x: 40, y: 30)
e := point hint(x: 20, y: 30)
f := point hint(x: 0, y: 30)
m := point hint(x: 30, y: 30)
ground a
ground m
a distance(60) b
b distance(30) c
a distance(30) f
ab := line(a, b)
bc := line(b, c)
cd := line(c, d)
notch := arc(center: m, start: e, end: d) hint(r: 10)
ef := line(e, f)
fa := line(f, a)
horizontal ab
vertical bc
horizontal cd
horizontal ef
vertical fa
radius(10) notch
sec := face(ab, bc, cd, notch, ef, fa)
notched := solid(sec, depth: 8mm)
";

fn read(source: &str) -> program::Elaborated {
    let (mut p,errors) = syntax::parse(source);
    assert!(errors.is_empty(),"{errors:?}");
    assert!(gcs_core::modules::link(&mut p,&mut gcs_core::library::resolve).is_empty());
    let mut e = program::elaborate(&p);
    assert!(e.ok(),"{:?}",e.diags);
    assert!(solve::solve(&mut e.sketch,Default::default()).success);
    e
}
fn solid(e: &program::Elaborated,name: &str) -> usize { e.map.ent_named(name).unwrap().i() }
fn field(e: &program::Elaborated,name: &str) -> SpatialField {
    SpatialField::read(&e.sketch,solid(e,name),1e-10).unwrap()
}
fn point(p: [f64;3]) -> [I;3] { p.map(|v| I::point(v).unwrap()) }
fn cube(lo: [f64;3],hi: [f64;3]) -> [I;3] { std::array::from_fn(|k| I::new(lo[k],hi[k]).unwrap()) }
fn value(f: &SpatialField,p: [f64;3]) -> [f64;2] { f.bounds(point(p)).unwrap().bounds() }

// A deterministic scatter over a box grown past the material on every side.
fn scatter(lo: [f64;3],hi: [f64;3],n: usize) -> Vec<[f64;3]> {
    (0..n).map(|i| std::array::from_fn(|k| {
        let fraction = ((i+1) as f64*[0.618034,0.414214,0.732051][k]).fract();
        lo[k]+(hi[k]-lo[k])*(1.3*fraction-0.15)
    })).collect()
}

fn lipschitz(f: &SpatialField,points: &[[f64;3]]) {
    for pair in points.windows(2) {
        let (p,q) = (pair[0],pair[1]);
        let (a,b) = (value(f,p),value(f,q));
        let apart = (0..3).map(|k| (p[k]-q[k]).powi(2)).sum::<f64>().sqrt();
        // Point enclosures are a few ulps wide; the bound holds between their far ends.
        assert!((a[1]-b[0]).max(b[1]-a[0]) <= apart+1e-9,"{p:?} {a:?} against {q:?} {b:?}");
    }
}

fn sign_agrees(f: &SpatialField,expected: impl Fn([f64;3]) -> f64,points: &[[f64;3]]) {
    let mut decided = 0;
    for &p in points {
        let want = expected(p);
        if want.abs() < 1e-6 { continue; }
        let got = value(f,p);
        assert!(if want < 0. { got[1] < 0. } else { got[0] > 0. },"{p:?}: {want} but {got:?}");
        decided += 1;
    }
    assert!(decided > points.len()/2);
}

// The page is the front view: `u = x`, `v = z`, normal `-y`, so a face drawn at
// `(a, b)` sweeps behind the page along `+y`, and `depth: d` fills `y` in [0, d].
fn slab(p: [f64;3],depth: f64) -> f64 { (p[1]-depth/2.).abs()-depth/2. }
fn boxed(p: [f64;3],a: f64,b: f64,depth: f64) -> f64 {
    ((p[0]-a/2.).abs()-a/2.).max((p[2]-b/2.).abs()-b/2.).max(slab(p,depth))
}

#[test]
fn block_field_has_the_box_sign_and_a_unit_lipschitz_bound() {
    let e = read(BLOCK);
    let block = field(&e,"block");
    let points = scatter([0.,0.,0.],[60.,30.,40.],200);
    sign_agrees(&block,|p| boxed(p,60.,40.,30.),&points);
    lipschitz(&block,&points);
    // A prism's field is the exact signed distance on the face nearest each point.
    let [lo,hi] = value(&block,[30.,15.,20.]);
    assert!(lo <= -15. && hi >= -15. && hi-lo < 1e-9,"{lo} {hi}");
    let [lo,hi] = value(&block,[70.,15.,20.]);
    assert!(lo <= 10. && hi >= 10. && hi-lo < 1e-9,"{lo} {hi}");
    let support = block.support_bounds().unwrap().unwrap();
    assert!(support[0].contains(0.) && support[0].contains(60.));
    assert!(support[1].contains(0.) && support[1].contains(30.));
    // Whole boxes enclose every point value: one straddling the far cap is mixed.
    let whole = block.bounds(cube([10.,25.,10.],[50.,35.,30.])).unwrap();
    assert!(whole.bounds()[0] < 0. && whole.bounds()[1] > 0.,"{whole:?}");
    let same = MaterialField::read(&e.sketch,solid(&e,"block"),1e-10).unwrap();
    assert!(same.support_bounds().unwrap().is_some());
}

#[test]
fn placed_block_field_turns_with_its_motion() {
    let e = read(&format!("{BLOCK}{TURN}"));
    let moved = field(&e,"moved");
    let pose = gcs_core::motion::Family::read(&e.sketch,e.map.ent_named("turn").unwrap().i())
        .unwrap().at(40_f64.to_radians()).unwrap();
    // A world point is in the placed copy exactly when the pose's inverse puts it in the source.
    let closed_form = |p: [f64;3]| boxed(pose.inverse().point(p),60.,40.,30.);
    let points: Vec<_> = scatter([0.,0.,0.],[60.,30.,40.],200).into_iter()
        .map(|q| pose.point(q)).collect();
    sign_agrees(&moved,closed_form,&points);
    lipschitz(&moved,&points);
    let support = moved.support_bounds().unwrap().unwrap();
    for q in [[0.,0.,0.],[60.,30.,40.]] {
        let p = pose.point(q);
        assert!((0..3).all(|k| support[k].contains(p[k])),"{support:?} excludes {p:?}");
    }
    let turned = pose.point([60.,0.,0.]);
    assert!(turned[0] < 60. && turned[1].abs() > 30.,"the copy really turned: {turned:?}");
}

fn agrees_with_facets(e: &program::Elaborated,name: &str,points: &[[f64;3]],clearance: f64) {
    let f = field(e,name);
    let csg = gcs_core::solid::resolve(&e.sketch,solid(e,name),REPORT_UNIT);
    let mut decided = 0;
    for &p in points {
        let got = value(&f,p);
        // Sign is only asked away from the boundary, where faceting cannot differ.
        if got[0].abs().min(got[1].abs()) < clearance || got[0] < 0. && got[1] > 0. { continue; }
        assert_eq!(got[1] < 0.,csg.inside(p),"{name} at {p:?}: {got:?}");
        decided += 1;
    }
    assert!(decided > points.len()/2,"{decided} of {} decided",points.len());
    lipschitz(&f,points);
}

#[test]
fn through_bore_cut_agrees_with_the_faceted_kernel() {
    let e = read(&format!("{BLOCK}{BORE}"));
    let points = scatter([0.,0.,0.],[60.,30.,40.],300);
    agrees_with_facets(&e,"body",&points,0.05);
    let body = field(&e,"body");
    let [lo,hi] = value(&body,[30.,15.,20.]);
    assert!(lo <= 8. && hi >= 8. && hi-lo < 1e-9,"the bore is void to its wall: {lo} {hi}");
    assert!(value(&body,[45.,15.,20.])[1] < 0.,"beside the bore is material");
    sign_agrees(&body,|p| boxed(p,60.,40.,30.).max(8.-(p[0]-30.).hypot(p[2]-20.)),&points);
    let explicit = read(&format!("{BLOCK}{}",BORE.replace("through: body","from: -31mm, to: 1mm")));
    agrees_with_facets(&explicit,"body",&points,0.05);
}

#[test]
fn concave_l_section_reads_and_agrees_with_the_faceted_kernel() {
    let e = read(ELL);
    let points = scatter([0.,0.,0.],[50.,12.,50.],300);
    agrees_with_facets(&e,"ell",&points,0.05);
    let ell = field(&e,"ell");
    let closed_form = |p: [f64;3]| boxed(p,50.,20.,12.).min(boxed(p,20.,50.,12.));
    sign_agrees(&ell,closed_form,&points);
    // In the notch both legs' edges are 3 away; inside, the reflex corner is the nearest boundary.
    let [lo,hi] = value(&ell,[23.,6.,23.]);
    assert!(lo <= 3. && hi >= 3. && hi-lo < 1e-9,"{lo} {hi}");
    let [lo,hi] = value(&ell,[18.,6.,18.]);
    assert!(lo <= -8_f64.sqrt() && hi >= -8_f64.sqrt() && hi-lo < 1e-9,"{lo} {hi}");
    let [lo,hi] = value(&ell,[10.,6.,10.]);
    assert!(lo <= -6. && hi >= -6. && hi-lo < 1e-9,"{lo} {hi}");
    let whole = ell.bounds(cube([5.,2.,5.],[15.,10.,15.])).unwrap();
    assert!(whole.bounds()[1] < 0.,"a box inside the leg is material: {whole:?}");
    let whole = ell.bounds(cube([30.,2.,30.],[45.,10.,45.])).unwrap();
    assert!(whole.bounds()[0] > 0.,"a box in the notch is void: {whole:?}");
}

#[test]
fn concave_notch_with_an_arc_reads_and_agrees_with_the_faceted_kernel() {
    let e = read(NOTCH);
    let points = scatter([0.,0.,0.],[60.,8.,30.],300);
    agrees_with_facets(&e,"notched",&points,0.1);
    let notched = field(&e,"notched");
    // The block less the disk about the notch's centre.
    let closed_form = |p: [f64;3]| boxed(p,60.,30.,8.).max(10.-(p[0]-30.).hypot(p[2]-30.));
    sign_agrees(&notched,closed_form,&points);
    // Under the notch the arc is the nearest wall, measured as a ring distance.
    let [lo,hi] = value(&notched,[30.,4.,15.]);
    assert!(lo <= -4. && hi >= -4. && hi-lo < 1e-8,"{lo} {hi}");
    let [lo,hi] = value(&notched,[30.,4.,25.]);
    assert!(lo <= 5. && hi >= 5. && hi-lo < 1e-8,"{lo} {hi}");
    let whole = notched.bounds(cube([25.,2.,24.],[35.,6.,29.])).unwrap();
    assert!(whole.bounds()[0] > 0.,"a box inside the notch is void: {whole:?}");
    let whole = notched.bounds(cube([2.,2.,2.],[18.,6.,28.])).unwrap();
    assert!(whole.bounds()[1] < 0.,"a box beside the notch is material: {whole:?}");
}
