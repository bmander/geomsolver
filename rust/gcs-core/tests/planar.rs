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
fn rings() -> gcs_core::json::Json {
    let rect = |r0: f64,r1: f64,z0: f64,z1: f64| format!(r#"{{"origin":[0,0,0],"normal":[0,1,0],"loops":[[
        {{"kind":"line","start":[{r0},0,{z0}],"end":[{r1},0,{z0}]}},{{"kind":"line","start":[{r1},0,{z0}],"end":[{r1},0,{z1}]}},
        {{"kind":"line","start":[{r1},0,{z1}],"end":[{r0},0,{z1}]}},{{"kind":"line","start":[{r0},0,{z1}],"end":[{r0},0,{z0}]}}]]}}"#);
    let text = format!(r#"{{"root":4,"nodes":[
        {{"id":1,"kind":"revolve","name":"a","origin":[0,0,0],"axis":[0,0,1],"angle":{tau},"profile":{a}}},
        {{"id":2,"kind":"revolve","name":"b","origin":[0,0,0],"axis":[0,0,1],"angle":{tau},"profile":{b}}},
        {{"id":3,"kind":"placed","name":"b2","source":2,"matrix":[1,0,0,3, 0,1,0,0, 0,0,1,0]}},
        {{"id":4,"kind":"body","name":"body","stock":1,"on":[],"cut":[],"bound":[3]}}]}}"#,
        tau=std::f64::consts::TAU,a=rect(10.,20.,0.,5.),b=rect(8.,15.,-1.,6.));
    gcs_core::json::parse(&text).unwrap()
}

#[test]
fn a_ring_bounded_by_an_offset_ring_sections_exactly() {
    use gcs_core::brep::section::Sectioned;
    let s = Sectioned::read(&rings()).unwrap().unwrap();
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
