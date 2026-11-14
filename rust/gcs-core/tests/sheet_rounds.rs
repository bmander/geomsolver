//! **A ball rolled on a sheet** (`brep::fillet::roll` where a face is a B-spline sheet): the ball's
//! centre stands off the sheet by its radius, and a sheet's offset has no closed form, so it is
//! read by the sheet's signed distance (`Surface::parallel`, `Surface::Offset`) — on the sheet
//! carried on past its domain by its own polynomials, where the ball rolls on past a face's edge.
//! Each property is held to what it says, and the round of brep-bench's
//! fillet/bezier-dome-one-edge to its volume integrated here from the patch alone, section by
//! section, sharing nothing with the kernel but the patch's heights.
use gcs_core::brep::boolean::{combined,Op};
use gcs_core::brep::build::lofted;
use gcs_core::brep::fillet::roll;
use gcs_core::brep::geom::{tolerance,Curve,Frame,Surface,V};
use gcs_core::brep::nurbs::{BSpline,Net};
use gcs_core::brep::props::volume;
use gcs_core::brep::topo::Brep;
use gcs_core::brep::util::quadrature::GAUSS8;
use std::sync::Arc;

/// Row i at x = 20i/3, column j at y = 20j/3: the heights of brep-bench's nurbs/bezier-dome.
const HEIGHTS: [[f64;4];4] = [[4.,6.,7.,5.],[6.,14.,12.,7.],[5.,13.,16.,8.],[3.,6.,8.,6.]];
const SIDE: f64 = 20.;

fn clamped(d: usize) -> Vec<f64> { [vec![0.;d+1],vec![1.;d+1]].concat() }

/// The dome itself: `x = 20u`, `y = 20v`.
fn dome() -> Net {
    Net {du:3,dv:3,uknots:clamped(3),vknots:clamped(3),weights:None,
        poles:(0..4).map(|i| (0..4).map(|j| [SIDE*i as f64/3.,SIDE*j as f64/3.,HEIGHTS[i][j]]).collect()).collect()}
}

/// The block on the square under the dome, as brep-bench builds it: lofted along x, its section
/// walked round counter-clockwise about +x (floor, the side y = 20, the dome back to y = 0, the
/// side y = 0), each face a net cubic along x, capped by the planes x = 0 and x = 20.
fn domed_block() -> Brep {
    let x = |i: usize| SIDE*i as f64/3.;
    let net = |du: usize,rows: &dyn Fn(usize) -> [V;4]| Net {du,dv:3,uknots:clamped(du),vknots:clamped(3),weights:None,
        poles:(0..=du).map(|k| rows(k).to_vec()).collect()};
    let floor = |y: f64| move |i: usize| [x(i),y,0.];
    let edge = |j: usize| move |i: usize| [x(i),SIDE*j as f64/3.,HEIGHTS[i][j]];
    let row = |f: &dyn Fn(usize) -> V| -> [V;4] { std::array::from_fn(f) };
    let nets = vec![
        net(1,&|k| if k == 0 { row(&floor(0.)) } else { row(&floor(SIDE)) }),
        net(1,&|k| if k == 0 { row(&floor(SIDE)) } else { row(&edge(3)) }),
        net(3,&|k| row(&edge(3-k))),
        net(1,&|k| if k == 0 { row(&edge(0)) } else { row(&floor(0.)) }),
    ];
    lofted([0.;3],[1.,0.,0.],[0.,SIDE],vec![nets]).unwrap()
}

fn sub(a: V,b: V) -> V { [a[0]-b[0],a[1]-b[1],a[2]-b[2]] }
fn add(a: V,b: V) -> V { [a[0]+b[0],a[1]+b[1],a[2]+b[2]] }
fn scale(a: V,s: f64) -> V { [a[0]*s,a[1]*s,a[2]*s] }
fn dot(a: V,b: V) -> f64 { a[0]*b[0]+a[1]*b[1]+a[2]*b[2] }
fn cross(a: V,b: V) -> V { [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]] }
fn norm(a: V) -> f64 { dot(a,a).sqrt() }
fn unit(a: V) -> V { scale(a,1./norm(a)) }

/// **A sheet's offset stands its distance off the sheet along its normal, past its domain too**:
/// its points are the sheet's continued points moved `d` along the unit normal there, its signed
/// distance is the sheet's less `d` (nothing at its own points, `t − d` at `t` along the normal),
/// its gradient the normal, and its partials by Weingarten's equations are its points' rates.
#[test]
fn a_sheets_offset_stands_its_distance_off_along_its_normal() {
    let net = dome();
    let sheet = Surface::BSpline(Frame::new([0.;3],[0.,0.,1.],[1.,0.,0.]),Arc::new(net.clone()));
    assert!(!sheet.has_offset() && sheet.has_parallel());
    let d = -1.;
    let offset = sheet.parallel(d,1e-9).unwrap();
    assert_eq!(offset.kind(),"offset");
    // an offset's offset is the sheet's, by the sum
    let twice = offset.parallel(-0.5,1e-9).unwrap();
    // inside the domain and past each of its edges, as far as a ball rolls on past a face
    for (u,v) in [(0.5,0.5),(0.1,0.9),(0.9,0.2),(-0.04,0.3),(1.03,0.7),(0.4,-0.05),(0.6,1.04)] {
        let [x,xu,xv,..] = net.d2_continued(u,v);
        let n = unit(cross(xu,xv));
        let p = offset.point([u,v]);
        assert!(norm(sub(p,add(x,scale(n,d)))) <= 1e-12,"({u}, {v}): the offset's point off the normal");
        assert!(offset.implicit(p).abs() <= 1e-10,"({u}, {v}): the offset's own point {:e} off it",offset.implicit(p));
        assert!(norm(sub(offset.gradient(p),n)) <= 1e-9,"({u}, {v}): the gradient is not the normal");
        for t in [-0.3,0.2] {
            let q = add(x,scale(n,t));
            assert!((offset.implicit(q)-(t-d)).abs() <= 1e-10,"({u}, {v}) at {t}: {}",offset.implicit(q));
            assert!((twice.implicit(q)-(t-d+0.5)).abs() <= 1e-10,"({u}, {v}) at {t}: the offset's offset");
        }
        // its partials, against its points' central differences
        let (_,su,sv) = offset.d1([u,v]);
        let h = 1e-5;
        let fd = |du: f64,dv: f64| scale(sub(offset.point([u+du,v+dv]),offset.point([u-du,v-dv])),0.5/h);
        for (got,want) in [(su,fd(h,0.)),(sv,fd(0.,h))] {
            assert!(norm(sub(got,want)) <= 1e-6*norm(want),"({u}, {v}): {got:?} against {want:?}");
        }
        // a point's foot on the offset is its foot on the continued sheet
        let uv = offset.inverse(p);
        assert!((uv[0]-u).abs() <= 1e-9 && (uv[1]-v).abs() <= 1e-9,"({u}, {v}): its foot read at {uv:?}");
    }
    // no other kind without a closed form is read yet
    let turned = Surface::Revolution(Frame::new([0.;3],[0.,0.,1.],[1.,0.,0.]),
        Arc::new(Curve::BSpline(Arc::new(BSpline::new(2,clamped(2),vec![[1.,0.,0.],[2.,0.,1.],[1.,0.,2.]]).unwrap()))));
    assert!(!turned.has_parallel() && turned.parallel(0.1,1e-9).is_none());
}

/// **A sheet carried on by its own polynomials is the same sheet**: `Net::extended`'s points past
/// the old domain are the continuation `Net::d2_continued` reads, and `ssi::same` takes a net and
/// itself carried on for one surface (as a round's strip run on past a face's edge lies on the
/// face's sheet), but not a net with a pole moved.
#[test]
fn a_sheet_carried_on_is_the_same_sheet() {
    use gcs_core::brep::ssi::same;
    let net = dome();
    let longer = net.extended(1,false,0.1).unwrap().extended(0,true,0.2).unwrap();
    assert_eq!(longer.domain(),[[0.,1.2],[-0.1,1.]]);
    for (u,v) in [(1.1,-0.05),(1.19,0.5),(0.3,-0.09),(0.5,0.5)] {
        let want = net.d2_continued(u,v)[0];
        assert!(norm(sub(longer.point(u,v),want)) <= 1e-11,"({u}, {v}): {:?} against {want:?}",longer.point(u,v));
    }
    let on = |n: Net| Surface::BSpline(Frame::new([0.;3],[0.,0.,1.],[1.,0.,0.]),Arc::new(n));
    assert!(same(&on(net.clone()),&on(longer.clone()),1e-9) && same(&on(longer.clone()),&on(net.clone()),1e-9));
    assert!(same(&on(net.clone()),&on(net.clone()),1e-9));
    let mut moved = net.clone();
    moved.poles[1][2][2] += 1e-3;
    assert!(!same(&on(net.clone()),&on(moved.clone()),1e-9));
    assert!(!same(&on(longer),&on(moved),1e-9));
}

/// **Two splines lying in a face cross where they cross** (`query::crossings_in`): neither a line
/// nor a conic, their crossing found by their enclosures and settled by Gauss–Newton — a parabola
/// `y = x²` and a cubic `y = 1 − x³` in the plane z = 2, which cross where `x² + x³ = 1`.
#[test]
fn splines_lying_in_a_face_cross_where_they_cross() {
    use gcs_core::brep::query::crossings_in;
    let plane = Surface::Plane(Frame::new([0.,0.,2.],[0.,0.,1.],[1.,0.,0.]));
    // y = x² over x in [−1, 2] as a quadratic Bézier; y = 1 − x³ over x in [0, 1.5] as a cubic one
    let parabola = Curve::BSpline(Arc::new(BSpline::new(2,clamped(2),vec![[-1.,1.,2.],[0.5,-2.,2.],[2.,4.,2.]]).unwrap()));
    let a = 1.5;
    let cubic = Curve::BSpline(Arc::new(BSpline::new(3,clamped(3),
        vec![[0.,1.,2.],[a/3.,1.,2.],[2.*a/3.,1.,2.],[a,1.-a*a*a,2.]]).unwrap()));
    // the root of x² + x³ = 1 in (0, 1), by bisection
    let (mut lo,mut hi) = (0.,1.);
    for _ in 0..200 { let m = 0.5*(lo+hi); if m*m+m*m*m < 1. { lo = m } else { hi = m } }
    let x = 0.5*(lo+hi);
    for (p,q) in [(&parabola,&cubic),(&cubic,&parabola)] {
        let found = crossings_in(&plane,p,[0.,1.],q,[0.,1.],1e-9).unwrap().expect("the crossing is read");
        assert_eq!(found.len(),1,"{found:?}");
        let (t,at) = found[0];
        assert!(norm(sub(at,[x,x*x,2.])) <= 1e-9,"{at:?} against x = {x}");
        assert!(norm(sub(p.point(t),at)) <= 1e-9);
    }
    // apart, they do not cross
    let high = Curve::BSpline(Arc::new(BSpline::new(3,clamped(3),
        vec![[0.,9.,2.],[a/3.,9.,2.],[2.*a/3.,9.,2.],[a,9.5,2.]]).unwrap()));
    assert_eq!(crossings_in(&plane,&parabola,[0.,1.],&high,[0.,1.],1e-9).unwrap(),Some(vec![]));
}

/// The volume the round takes off the block, integrated from the patch alone: the patch is the
/// height field `z = h(x, y)`, the bicubic Bernstein sum (its continuation past the square the
/// same polynomial); the ball's centre stands 1 under it along its normal, `C = F − N(F)` from its
/// foot `F`, and 1 off the side y = 0, so on the plane y = 1; in the plane x = c the round's
/// section is bounded by the side, the dome and the ball's arc from its touch on the side to its
/// touch on the dome, each point of the arc `C + slerp(e_side, N, s)` at the ball's centre whose arc
/// crosses x = c there. Its area by Green's theorem, Gauss–Legendre along each piece, and the
/// volume by Gauss–Legendre along x.
fn removed_by_sections() -> f64 {
    let bern = |t: f64| [(1.-t).powi(3),3.*t*(1.-t).powi(2),3.*t*t*(1.-t),t.powi(3)];
    let dbern = |t: f64| [-3.*(1.-t).powi(2),3.*(1.-t).powi(2)-6.*t*(1.-t),6.*t*(1.-t)-3.*t*t,3.*t*t];
    let sum = |bu: [f64;4],bv: [f64;4]| (0..4).map(|i| (0..4).map(|j| bu[i]*bv[j]*HEIGHTS[i][j]).sum::<f64>()).sum::<f64>();
    let h = |x: f64,y: f64| sum(bern(x/SIDE),bern(y/SIDE));
    let hx = |x: f64,y: f64| sum(dbern(x/SIDE),bern(y/SIDE))/SIDE;
    let hy = |x: f64,y: f64| sum(bern(x/SIDE),dbern(y/SIDE))/SIDE;
    let normal = |x: f64,y: f64| unit([-hx(x,y),-hy(x,y),1.]);
    let centre = |x: f64,y: f64| sub([x,y,h(x,y)],normal(x,y));
    // the foot's y whose centre stands on y = 1, by the secant method
    let foot_y = |x: f64| {
        let f = |y: f64| centre(x,y)[1]-1.;
        let (mut a,mut b) = (0.2,0.4);
        for _ in 0..100 { let (fa,fb) = (f(a),f(b)); if fb == fa { break } let c = b-fb*(b-a)/(fb-fa); a = b; b = c; if (b-a).abs() < 1e-15 { break } }
        b
    };
    let arc = |xf: f64,s: f64| {
        let yf = foot_y(xf);
        let (c,es,ed) = (centre(xf,yf),[0.,-1.,0.],normal(xf,yf));
        let phi = dot(es,ed).clamp(-1.,1.).acos();
        add(c,scale(add(scale(es,((1.-s)*phi).sin()),scale(ed,(s*phi).sin())),1./phi.sin()))
    };
    // the foot's x whose arc crosses x = c at `s`
    let foot_x = |c: f64,s: f64,start: f64| {
        let f = |x: f64| arc(x,s)[0]-c;
        let (mut a,mut b) = (start,start+0.01);
        for _ in 0..100 { let (fa,fb) = (f(a),f(b)); if fb == fa { break } let n = b-fb*(b-a)/(fb-fa); a = b; b = n; if (b-a).abs() < 1e-15 { break } }
        b
    };
    // Gauss–Legendre over [a, b] in `panels` panels of eight points
    let gauss = |a: f64,b: f64,panels: usize,f: &mut dyn FnMut(f64) -> f64| -> f64 {
        let w = (b-a)/panels as f64;
        (0..panels).map(|k| { let m = a+w*(k as f64+0.5);
            GAUSS8.iter().map(|&(t,g)| 0.5*w*g*f(m+0.5*w*t)).sum::<f64>() }).sum()
    };
    let area = |c: f64| {
        // the arc, from the side's touch to the dome's: ½ (y z' − z y') along s, x held at c
        let mut start = c;
        let along = gauss(0.,1.,4,&mut |s| {
            let x = foot_x(c,s,start);
            start = x;
            let e = 1e-6;
            let p = arc(x,s);
            let ps = scale(sub(arc(x,s+e),arc(x,s-e)),0.5/e);
            let px = scale(sub(arc(x+e,s),arc(x-e,s)),0.5/e);
            let dp = add(ps,scale(px,-ps[0]/px[0]));
            0.5*(p[1]*dp[2]-p[2]*dp[1])
        });
        // the dome, from its touch back to the corner over the side: z = h(c, y), y from its touch to 0
        let ya = arc(foot_x(c,1.,c),1.)[1];
        let over = gauss(0.,ya,4,&mut |y| 0.5*(y*hy(c,y)-h(c,y)));
        // (the side, y = 0, adds nothing)
        (along-over).abs()
    };
    gauss(0.,SIDE,40,&mut |c| area(c))
}

/// **The Bézier dome block with one top edge rounded** (brep-bench's fillet/bezier-dome-one-edge):
/// both faces at the edge are sheets — the bicubic dome and the side y = 0, a net flat in its
/// plane — and the ball rolls on past the caps x = 0 and x = 20 on the sheets carried on, the
/// round cut off by the caps. Valid, and its volume the block's less what the sections say the
/// round takes (`removed_by_sections`); the block's own volume is the mean of the patch's heights
/// over the square, each Bernstein polynomial's integral a quarter.
#[test]
fn the_bezier_dome_block_rounded_along_one_top_edge() {
    let block = domed_block();
    let mean: f64 = HEIGHTS.iter().flatten().sum::<f64>()/16.;
    assert!((volume(&block)-mean*SIDE*SIDE).abs() <= 1e-9*mean*SIDE*SIDE,"{}",volume(&block));
    // the edge picked over x = 10, where the dome's edge stands at the Bernstein mean of its heights
    let round = roll(&block,[SIDE/2.,0.,5.],1.,tolerance(block.size())).unwrap();
    assert!(!round.concave);
    let rounded = combined(&block,&round.piece,Op::Cut,0.).unwrap();
    rounded.check(tolerance(rounded.size())).unwrap();
    assert_eq!(rounded.faces.len(),7);
    let want = mean*SIDE*SIDE-removed_by_sections();
    let got = volume(&rounded);
    assert!((got-want).abs() <= 1e-7,"the rounded block's volume {got} against {want} ({:e})",got-want);
}
