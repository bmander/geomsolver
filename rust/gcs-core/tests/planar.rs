//! Planar regions of lines and arcs and their Booleans (`brep::planar`), against closed forms.
use gcs_core::brep::planar::{boolean,Op,Region,Seg};
use std::f64::consts::PI;

fn rect(x0: f64,y0: f64,x1: f64,y1: f64) -> Region {
    let p = [[x0,y0],[x1,y0],[x1,y1],[x0,y1]];
    Region::plain(vec![(0..4).map(|i| Seg::Line {a:p[i],b:p[(i+1)%4]}).collect()])
}
fn disc(c: [f64;2],r: f64) -> Region { Region::plain(vec![vec![Seg::Arc {c,r,a0:0.,sweep:2.*PI}]]) }
fn close(a: f64,b: f64,tol: f64) { assert!((a-b).abs() <= tol,"{a} against {b}"); }

#[test]
fn squares_overlapping_and_sharing_edges() {
    let (a,b) = (rect(0.,0.,2.,2.),rect(1.,1.,3.,3.));
    // areas by the region's own polygon: exact for lines
    close(boolean(&a,&b,Op::Union,1e-9).unwrap().area(),7.,1e-12);
    close(boolean(&a,&b,Op::Common,1e-9).unwrap().area(),1.,1e-12);
    close(boolean(&a,&b,Op::Cut,1e-9).unwrap().area(),3.,1e-12);
    // edges shared along a stretch: side by side, and one inside the other along its bottom
    let c = rect(2.,0.,4.,1.);
    let u = boolean(&a,&c,Op::Union,1e-9).unwrap();
    close(u.area(),6.,1e-12);
    assert_eq!(u.loops.len(),1);
    let d = rect(0.5,0.,1.5,1.);
    close(boolean(&a,&d,Op::Cut,1e-9).unwrap().area(),3.,1e-12);
    close(boolean(&a,&d,Op::Common,1e-9).unwrap().area(),1.,1e-12);
    // a hole: the square less a square inside it
    let h = boolean(&a,&rect(0.5,0.5,1.5,1.5),Op::Cut,1e-9).unwrap();
    assert_eq!(h.loops.len(),2);
    close(h.area(),3.,1e-12);
}

#[test]
fn discs_and_squares_cut_exactly() {
    // a disc's area by its polygon of 64ths of a turn: compared at that polygon's precision
    let poly = |r: f64| 32.*r*r*(2.*PI/64.).sin();
    let half = boolean(&disc([0.,0.],1.),&rect(0.,-2.,2.,2.),Op::Common,1e-9).unwrap();
    close(half.area(),poly(1.)/2.,1e-12);
    // the region's pieces are exact: every arc of the half-disc is on the circle, every line on x = 0
    for s in half.loops.iter().flatten() {
        match s.seg {
            Seg::Arc {c,r,..} => { close(c[0].hypot(c[1]),0.,1e-15); close(r,1.,1e-15); }
            Seg::Line {a,b} => { close(a[0],0.,1e-15); close(b[0],0.,1e-15); }
            Seg::Conic {..} => unreachable!("lines and arcs make no conic"),
        }
    }
    // two unit discs a unit apart: the lens is 2π/3 − √3/2, its arcs' ends where the circles cross
    let lens = boolean(&disc([0.,0.],1.),&disc([1.,0.],1.),Op::Common,1e-9).unwrap();
    assert_eq!(lens.loops.len(),1);
    // (a whole circle is also split where it starts: three arcs, each on its own circle)
    for s in &lens.loops[0] {
        let Seg::Arc {c,r,..} = s.seg else { panic!("a lens of lines") };
        close(r,1.,1e-15);
        for p in [s.start(),s.end()] { close((p[0]-c[0]).hypot(p[1]-c[1]),1.,1e-12); }
    }
    let corners = lens.loops[0].iter().filter(|s| (s.start()[0]-0.5).abs() < 1e-12).count();
    assert_eq!(corners,2);
    let sweep: f64 = lens.loops[0].iter().map(|s| match s.seg { Seg::Arc {sweep,..} => sweep.abs(),_ => 0. }).sum();
    close(sweep,4.*PI/3.,1e-12);
}

#[test]
fn a_profile_listed_out_of_order_is_walked() {
    // the same square, its edges listed out of order and two of them backwards
    let p = [[0.,0.],[2.,0.],[2.,2.],[0.,2.]];
    let r = Region::plain(vec![vec![Seg::Line {a:p[2],b:p[1]},Seg::Line {a:p[0],b:p[1]},Seg::Line {a:p[3],b:p[0]},Seg::Line {a:p[2],b:p[3]}]]);
    close(r.area(),4.,1e-12);
    close(boolean(&r,&rect(1.,1.,3.,3.),Op::Common,1e-9).unwrap().area(),1.,1e-12);
}

/// A recipe: a ring about z (r 10–20, z 0–5) bounded by a ring about the parallel line through
/// (3, 0) (r 8–15, z −1–6), that one drawn about z and placed there.
fn rings() -> gcs_core::json::Json { rings_tilted(0.) }

/// `rings`, the second ring's line also turned by `tilt` (radians) about x.
fn rings_tilted(tilt: f64) -> gcs_core::json::Json {
    let (c,s) = (tilt.cos(),tilt.sin());
    let rect = |r0: f64,r1: f64,z0: f64,z1: f64| format!(r#"{{"origin":[0,0,0],"normal":[0,1,0],"loops":[[
        {{"kind":"line","start":[{r0},0,{z0}],"end":[{r1},0,{z0}]}},{{"kind":"line","start":[{r1},0,{z0}],"end":[{r1},0,{z1}]}},
        {{"kind":"line","start":[{r1},0,{z1}],"end":[{r0},0,{z1}]}},{{"kind":"line","start":[{r0},0,{z1}],"end":[{r0},0,{z0}]}}]]}}"#);
    let text = format!(r#"{{"root":4,"nodes":[
        {{"id":1,"kind":"revolve","name":"a","origin":[0,0,0],"axis":[0,0,1],"angle":{tau},"profile":{a}}},
        {{"id":2,"kind":"revolve","name":"b","origin":[0,0,0],"axis":[0,0,1],"angle":{tau},"profile":{b}}},
        {{"id":3,"kind":"placed","name":"b2","source":2,"matrix":[1,0,0,3, 0,{c},{n},0, 0,{s},{c},0]}},
        {{"id":4,"kind":"body","name":"body","stock":1,"on":[],"cut":[],"bound":[3]}}]}}"#,
        tau=std::f64::consts::TAU,a=rect(10.,20.,0.,5.),b=rect(8.,15.,-1.,6.),n=-s);
    gcs_core::json::parse(&text).unwrap()
}

/// A line parallel only to its rounding, as a solved one is (the cutter of an 18-tooth hypoid
/// pair's gear, 2.7e-12 off), is read as parallel: the section moves by the tilt times the reach,
/// within the recipe's tolerance. A tilt that moves it farther is still refused.
#[test]
fn a_line_parallel_within_the_tolerance_sections_as_parallel() {
    use gcs_core::brep::section::Sectioned;
    let s = Sectioned::read(&rings_tilted(1e-11)).unwrap();
    close(s.section([1.,0.,0.]).unwrap().area(),5.*(18.-11.),1e-9);
    let refused = Sectioned::read(&rings_tilted(1e-6)).err().unwrap();
    assert!(refused.to_string().contains("not parallel to the first's"),"{refused}");
}

#[test]
fn a_ring_bounded_by_an_offset_ring_sections_exactly() {
    use gcs_core::brep::section::Sectioned;
    let s = Sectioned::read(&rings()).unwrap();
    // towards the other line's offset: its ring from s = 3 + 8 to 3 + 15, cut by 10–20
    let x = s.section([1.,0.,0.]).unwrap();
    close(x.area(),5.*(18.-11.),1e-9);
    // square to it: r'² = s² + 9 from 8² to 15², so s from √55 to √216, cut at 10
    let y = s.section([0.,1.,0.]).unwrap();
    close(y.area(),5.*(216f64.sqrt()-10.),1e-6);
    // the offset ring's inner wall in the first: at s = 11, its normal back towards its own line
    let wall = x.loops.iter().flatten().find(|st| st.chart.is_some() && (st.start()[0]-11.).abs() < 1e-9 && (st.end()[0]-11.).abs() < 1e-9).unwrap();
    let (p,n) = s.at([1.,0.,0.],wall,0.5);
    close(p[0],11.,1e-9);
    close(n[0],-1.,1e-12); close(n[1].abs()+n[2].abs(),0.,1e-12);
    // and its outer wall in the second, square: its normal away from (3, 0) along the radius there
    let outer = y.loops.iter().flatten().find(|st| st.chart.is_some() && (st.start()[0]-216f64.sqrt()).abs() < 1e-9).unwrap();
    let (p,n) = s.at([0.,1.,0.],outer,0.5);
    let radial = [p[0]-3.,p[1],0.];
    let l = radial[0].hypot(radial[1]);
    close(n[0],radial[0]/l,1e-12); close(n[1],radial[1]/l,1e-12);
}

/// A conic's arc (#196): the rational quadratic Bézier over its ends and where their tangents meet.
/// The parabola `y = x²` from `(1, 1)` to `(−1, 1)` (its tangents meet at `(0, −1)`, the middle
/// weighed one) and the quarter of the unit circle (weighed `cos 45°`): every point on its curve,
/// a point's parameter read back, a stretch of it the same curve, its length the closed form's.
#[test]
fn a_conics_arc_is_exact() {
    let parabola = Seg::Conic {a:[1.,1.],m:[0.,-1.],b:[-1.,1.],w:1.};
    let quarter = Seg::Conic {a:[1.,0.],m:[1.,1.],b:[0.,1.],w:0.5f64.sqrt()};
    for k in 0..=50 {
        let t = k as f64/50.;
        let (p,q) = (parabola.point(t),quarter.point(t));
        close(p[1],p[0]*p[0],1e-15);
        close(q[0].hypot(q[1]),1.,1e-15);
        close(parabola.param(p),t,1e-12);
        close(quarter.param(q),t,1e-12);
    }
    // a stretch: the same curve between the same points, and the curve reversed
    let piece = parabola.part(0.2,0.7);
    for k in 0..=10 {
        let p = piece.point(k as f64/10.);
        close(p[1],p[0]*p[0],1e-14);
    }
    for (x,y) in [(piece.start(),parabola.point(0.2)),(piece.end(),parabola.point(0.7))] {
        close(x[0],y[0],1e-14); close(x[1],y[1],1e-14);
    }
    assert_eq!(parabola.reversed().point(0.),[-1.,1.]);
    close(parabola.length(),5f64.sqrt()+2f64.asinh()/2.,1e-12);
    close(quarter.length(),PI/2.,1e-12);
    // the cap above the parabola: the conic and the chord back
    let cap = Region::plain(vec![vec![parabola.reversed(),Seg::Line {a:[1.,1.],b:[-1.,1.]}]]);
    for i in 0..40 { for j in 0..40 {
        let p = [-1.2+2.4*(i as f64+0.31)/40.,-0.2+1.4*(j as f64+0.17)/40.];
        assert_eq!(cap.contains(p),p[1] > p[0]*p[0] && p[1] < 1.,"{p:?}");
    } }
    // a Boolean across it: the cap's right half, its area the polygon's of the conic's chords
    let half = boolean(&cap,&rect(0.,-1.,2.,2.),Op::Common,1e-9).unwrap();
    close(half.area(),2./3.,1e-3);
    assert!(half.loops.iter().flatten().any(|s| matches!(s.seg,Seg::Conic {..})));
}

/// `corners::sum`, `std.rounds.Chamfered`'s flat: between two lines the chamfer from the other
/// face; between a line and a circle the conic its sum of distances bounds, through the same ends.
#[test]
fn a_chamfers_sum_of_distances_is_a_line_or_a_conic() {
    use gcs_core::brep::corners::{chamfer,summed,Leg};
    // each step tagged by its place, from 1
    let tagged = |r: Region| r.tagged(&|_,k| k as u32+1);
    let square = tagged(rect(0.,0.,4.,4.));
    let flat = chamfer(&square,&[2],&[3],[Leg::FromOther(1.),Leg::FromOther(0.5)],9).unwrap();
    let cut = |r: &Region| r.loops.iter().flatten().find(|s| s.tag == 9).unwrap().seg;
    assert_eq!(cut(&flat),cut(&summed(&square,&[2],&[3],[1.,0.5],9).unwrap()));
    // a disc's quarter: the right side a line, the rim the circle of radius 4 about the origin
    let sector = tagged(Region::plain(vec![vec![Seg::Line {a:[0.,0.],b:[4.,0.]},
        Seg::Arc {c:[0.,0.],r:4.,a0:0.,sweep:PI/2.},Seg::Line {a:[0.,4.],b:[0.,0.]}]]));
    let (la,lb) = (0.5,0.8);
    let relieved = summed(&sector,&[1],&[2],[la,lb],9).unwrap();
    let conic = cut(&relieved);
    assert!(matches!(conic,Seg::Conic {..}));
    for k in 0..=40 {
        let q = conic.point(k as f64/40.);
        // from the line (y = 0) and from the circle, each into the material, over the other's leg
        close(q[1]/lb+(4.-q[0].hypot(q[1]))/la,1.,1e-12);
    }
    // its ends: on the line `la` inside the circle, on the circle `lb` above the line
    let ends = [conic.start(),conic.end()];
    assert!(ends.iter().any(|p| p[1].abs() < 1e-12 && (p[0]-(4.-la)).abs() < 1e-12));
    assert!(ends.iter().any(|p| (p[0].hypot(p[1])-4.).abs() < 1e-12 && (p[1]-lb).abs() < 1e-12));
}

/// Two rounded corners tangent to one line a hair apart, as a gear cutter's crown and its
/// neighbour meet at an 18-tooth hypoid pair's tip: a stretch of one arc lies within the tolerance
/// of the other at its middle but not at its end, so it is no stretch the two share, and the
/// Common closes. `a` is [0, 10] × [0, 1] rounded at its top left, `b` [−10, 1 − δ] × [0, 1] at its
/// top right, the arcs' tangent points δ apart on the top: the stretch between where they cross and
/// `a`'s tangent point is δ²/2 off `b` at its middle and δ² at its end, either side of `tol`.
#[test]
fn near_tangent_corners_combine_and_close() {
    let (r,d,tol) = (0.5,1e-3,7e-7);
    let h = std::f64::consts::FRAC_PI_2;
    let a = Region::plain(vec![vec![Seg::Line {a:[10.,0.],b:[10.,1.]},Seg::Line {a:[10.,1.],b:[r,1.]},
        Seg::Arc {c:[r,1.-r],r,a0:h,sweep:h},Seg::Line {a:[0.,1.-r],b:[0.,0.]},Seg::Line {a:[0.,0.],b:[10.,0.]}]]);
    let xb = r-d;
    let b = Region::plain(vec![vec![Seg::Line {a:[-10.,0.],b:[xb+r,0.]},Seg::Line {a:[xb+r,0.],b:[xb+r,1.-r]},
        Seg::Arc {c:[xb,1.-r],r,a0:0.,sweep:h},Seg::Line {a:[xb,1.],b:[-10.,1.]},Seg::Line {a:[-10.,1.],b:[-10.,0.]}]]);
    // [0, 1 − δ] × [0, 1] rounded by both arcs, which cross at x = r − δ/2: nothing of the top
    let common = boolean(&a,&b,Op::Common,tol).unwrap();
    assert_eq!(common.loops.len(),1);
    let mut sweep = 0.;
    for s in &common.loops[0] {
        match s.seg {
            Seg::Arc {c,r: q,sweep: w,..} => {
                close(q,r,1e-15); close(c[1],1.-r,1e-15);
                assert!(c[0] == r || c[0] == xb,"an arc about {c:?}");
                sweep += w.abs();
            }
            Seg::Line {a: p,b: q} => assert!(p[1].max(q[1]) < 1.-r+1e-12 || (p[0]-q[0]).abs() < 1e-15,"{p:?} {q:?}"),
            Seg::Conic {..} => unreachable!("lines and arcs make no conic"),
        }
    }
    let crossing = common.loops[0].iter().filter(|s| matches!(s.seg,Seg::Arc {..}))
        .flat_map(|s| [s.start(),s.end()]).filter(|p| p[1] > 1.-r+0.1).collect::<Vec<_>>();
    assert_eq!(crossing.len(),2);
    for p in crossing { close(p[0],r-d/2.,1e-9); }
    close(sweep,PI-2.*(d/2./r).asin(),1e-9);
    // and their union closes too, one loop
    assert_eq!(boolean(&a,&b,Op::Union,tol).unwrap().loops.len(),1);
}
