//! **The zero set of a scalar function traced** (`brep::zero_set`, #942): each gate a function on a
//! plane whose zero set has a closed form, none needing a sweep — closed loops inside a rectangle
//! (a circle and an ellipse, a loop far under any natural grid's cell, two loops close together),
//! the outline of two discs' union as the least of their signed distances (two arcs meeting at
//! two corners, returned exactly at the circles' crossings), a curve touching a given edge
//! tangentially and a cusp ending on one, a curve run rim to rim and one touching the rim, and the
//! depth of a plane's points under a ball moved along a line and round a circle (a stadium's and an
//! annulus's outlines), `f = min over t of |P − c(t)| − r`, the nearest thing to the real callers.
//! Each asserts every returned point's and chord middle's distance in space from the true zero set,
//! the count and the ends of the pieces, and the corners' places.
use gcs_core::brep::zero_set::{through,trace,Bound,Edge,End,Field,Piece,Question,Rect,Traced,Value};
use gcs_core::fmath::Det;
use gcs_core::interval::{Iv,Num};

type Uv = [f64;2];
type At = (usize,f64);
/// A branch read at a point of the plane in space: its value, its gradient in space (none where
/// it has none), and its minimiser.
type Branch = Box<dyn Fn([f64;2]) -> (f64,Option<[f64;2]>,f64)>;

/// A function on the plane `P(u, v) = (s₀ u, s₁ v, 0)`: the least of its branches, with a bound
/// over cells where one is given.
struct Plane { s: [f64;2],branches: Vec<Branch>,bound: Option<Box<dyn Fn(Rect) -> Bound>> }

impl Plane {
    fn new(s: [f64;2],branches: Vec<Branch>) -> Plane { Plane {s,branches,bound:None} }
    fn bounded(mut self,b: impl Fn(Rect) -> Bound+'static) -> Plane { self.bound = Some(Box::new(b)); self }
    fn read(&self,i: usize,uv: Uv) -> Value<At> {
        let (f,g,t) = self.branches[i]([self.s[0]*uv[0],self.s[1]*uv[1]]);
        Value {f,grad:g.map(|g| [self.s[0]*g[0],self.s[1]*g[1]]),at:(i,t)}
    }
}

impl Field for Plane {
    type At = At;
    fn least(&self,uv: Uv) -> Option<Value<At>> {
        (0..self.branches.len()).map(|i| self.read(i,uv)).min_by(|a,b| a.f.total_cmp(&b.f))
    }
    fn along(&self,uv: Uv,from: &At) -> Option<Value<At>> { Some(self.read(from.0,uv)) }
    fn surface(&self,uv: Uv) -> ([f64;3],[f64;3],[f64;3]) {
        ([self.s[0]*uv[0],self.s[1]*uv[1],0.],[self.s[0],0.,0.],[0.,self.s[1],0.])
    }
    fn bound(&self,cell: Rect) -> Option<Bound> { self.bound.as_ref().map(|b| b(cell)) }
}

/// A cell's two coordinates in space as intervals.
fn xy(c: Rect,s: [f64;2]) -> (Iv,Iv) { (Iv::new(s[0]*c[0][0],s[0]*c[0][1]),Iv::new(s[1]*c[1][0],s[1]*c[1][1])) }
fn sq(x: Iv) -> Iv { (x*x).meet(Iv::new(0.,f64::MAX)) }
fn k(x: f64) -> Iv { Iv::point(x) }

/// A disc's signed distance, `|P − c| − r`.
fn disc(c: [f64;2],r: f64) -> Branch {
    Box::new(move |p| {
        let d = (p[0]-c[0]).dhypot(p[1]-c[1]);
        (d-r,(d > 0.).then(|| [(p[0]-c[0])/d,(p[1]-c[1])/d]),0.)
    })
}
/// The disc's bound over a cell: its value and gradient in the parameters, in intervals.
fn disc_bound(cell: Rect,s: [f64;2],c: [f64;2],r: f64) -> Bound {
    let (x,y) = xy(cell,s);
    let (dx,dy) = (x-k(c[0]),y-k(c[1]));
    // (the sum rounded outward may reach below nought where the cell holds the centre)
    let d = (sq(dx)+sq(dy)).meet(Iv::new(0.,f64::MAX)).sqrt();
    Bound {value:d-k(r),grad:Some([(dx/d).times(s[0]),(dy/d).times(s[1])])}
}
/// The least of several branches' bounds: each branch's gradient joins where it may be the least.
fn least_bound(bs: &[Bound]) -> Bound {
    let lo = bs.iter().map(|b| b.value.lo()).fold(f64::INFINITY,f64::min);
    let hi = bs.iter().map(|b| b.value.hi()).fold(f64::INFINITY,f64::min);
    let mut grad: Option<[Iv;2]> = None;
    for b in bs {
        if b.value.lo() <= hi {
            let bg = b.grad.unwrap();
            grad = Some(match grad { None => bg,Some(g) => [g[0].hull(bg[0]),g[1].hull(bg[1])] });
        }
    }
    Bound {value:Iv::new(lo,hi),grad}
}

fn question(rect: Rect,tol: f64,feature: f64) -> Question<'static> { Question {rect,edges:&[],seeds:&[],tol,feature} }

const TOL: f64 = 1e-6;

/// Every point of every piece, and every chord's middle, in space within `tol` of the zero set
/// whose distance in space is `distance`.
fn assert_on(t: &Traced<At>,s: [f64;2],distance: impl Fn([f64;2]) -> f64,tol: f64) {
    let mut worst: f64 = 0.;
    let mut at = [0.;2];
    for p in &t.pieces {
        assert!(p.uv.len() >= 2,"a piece of one point: {p:?}");
        for w in p.uv.windows(2) {
            for x in [w[0],[(w[0][0]+w[1][0])/2.,(w[0][1]+w[1][1])/2.]] {
                let d = distance([s[0]*x[0],s[1]*x[1]]);
                if d > worst { worst = d; at = x; }
            }
        }
    }
    assert!(worst <= tol,"a traced point or chord stands {worst:e} from the zero set at {at:?}");
}

/// The turn of a closed piece about `c` in space, in whole turns.
fn turns(p: &Piece<At>,s: [f64;2],c: [f64;2]) -> f64 {
    let a: Vec<f64> = p.uv.iter().map(|uv| (s[1]*uv[1]-c[1]).datan2(s[0]*uv[0]-c[0])).collect();
    let mut sum = 0.;
    for w in a.windows(2) {
        let mut d = w[1]-w[0];
        while d > std::f64::consts::PI { d -= 2.*std::f64::consts::PI }
        while d < -std::f64::consts::PI { d += 2.*std::f64::consts::PI }
        sum += d;
    }
    sum/(2.*std::f64::consts::PI)
}

/// The distance in space from `p` to the ellipse of semi-axes `a`, `b` about `c`: the foot's angle
/// by Newton from the nearest of a fine ring of samples.
fn ellipse_distance(p: [f64;2],c: [f64;2],a: f64,b: f64) -> f64 {
    let (x,y) = (p[0]-c[0],p[1]-c[1]);
    let d = |t: f64| { let (s,co) = t.dsin_cos(); (a*co-x).dhypot(b*s-y) };
    let mut t = (0..256).map(|i| i as f64*std::f64::consts::TAU/256.).min_by(|u,v| d(*u).total_cmp(&d(*v))).unwrap();
    for _ in 0..50 {
        let (s,co) = t.dsin_cos();
        let (ex,ey) = (a*co-x,b*s-y);
        let (dx,dy) = (-a*s,b*co);
        let g = ex*dx+ey*dy;
        let h = dx*dx+dy*dy+ex*(-a*co)+ey*(-b*s);
        if h <= 0. { break }
        t -= g/h;
    }
    d(t)
}

/// The distance in space from `p` to the graph `y = h(x)` over `[x0, x1]`: the nearest of a fine
/// set of samples refined by Brent's method.
fn graph_distance(p: [f64;2],h: &dyn Fn(f64) -> f64,[x0,x1]: [f64;2]) -> f64 {
    let d = |x: f64| (x-p[0]).dhypot(h(x)-p[1]);
    let n = 400;
    let step = (x1-x0)/n as f64;
    let i = (0..=n).min_by(|a,b| d(x0+*a as f64*step).total_cmp(&d(x0+*b as f64*step))).unwrap();
    let (lo,hi) = ((x0+(i as f64-1.)*step).max(x0),(x0+(i as f64+1.)*step).min(x1));
    gcs_core::roots::brent(&d,lo,hi,1e-15,200,|_,_| false).value.min(d(x0+i as f64*step))
}

fn ellipse_branch(c: [f64;2],a: f64,b: f64) -> Branch {
    Box::new(move |p| {
        let (x,y) = (p[0]-c[0],p[1]-c[1]);
        (x*x/(a*a)+y*y/(b*b)-1.,Some([2.*x/(a*a),2.*y/(b*b)]),0.)
    })
}

#[test]
fn a_circle_and_an_ellipse_inside_a_rectangle_are_traced_closed() {
    let s = [1.,1.];
    let (cc,r) = ([-0.5,0.1],0.3);
    let (ce,a,b) = ([0.45,-0.2],0.35,0.2);
    let plane = Plane::new(s,vec![disc(cc,r),ellipse_branch(ce,a,b)]).bounded(move |cell| {
        let (x,y) = xy(cell,s);
        let (ex,ey) = (x-k(ce[0]),y-k(ce[1]));
        let e = Bound {value:sq(ex).times(1./(a*a))+sq(ey).times(1./(b*b))-k(1.),grad:Some([ex.times(2./(a*a)),ey.times(2./(b*b))])};
        least_bound(&[disc_bound(cell,s,cc,r),e])
    });
    let t = trace(&plane,&question([[-1.,1.],[-1.,1.]],TOL,0.1)).unwrap();
    assert_eq!(t.pieces.len(),2,"{:?}",t.pieces.iter().map(|p| p.ends).collect::<Vec<_>>());
    assert!(t.pieces.iter().all(|p| p.closed()));
    assert!(t.search.unresolved.is_empty() && t.search.sampled == 0,"{:?}",t.search);
    assert_on(&t,s,|p| ((p[0]-cc[0]).dhypot(p[1]-cc[1])-r).abs().min(ellipse_distance(p,ce,a,b)),TOL);
    for (c,i) in [(cc,0),(ce,1)] {
        let p = t.pieces.iter().find(|p| p.at[0].0 == i).expect("each loop traced");
        assert!(p.at.iter().all(|a| a.0 == i));
        assert!((turns(p,s,c).abs()-1.).abs() < 1e-9,"the loop about {c:?} turns {} times",turns(p,s,c));
    }
}

#[test]
fn a_loop_far_under_a_grid_cell_is_found() {
    // a loop a fiftieth of the feature size across, nowhere near a cell's middle, found by the
    // bound's subdivision: no grid of the feature's size would see it
    let s = [1.,1.];
    let (c,r) = ([0.1234,-0.2718],0.002);
    let plane = Plane::new(s,vec![disc(c,r)]).bounded(move |cell| disc_bound(cell,s,c,r));
    let t = trace(&plane,&question([[-1.,1.],[-1.,1.]],TOL,0.2)).unwrap();
    assert_eq!(t.pieces.len(),1);
    assert!(t.pieces[0].closed());
    assert!(t.search.unresolved.is_empty() && t.search.sampled == 0,"{:?}",t.search);
    assert_on(&t,s,|p| ((p[0]-c[0]).dhypot(p[1]-c[1])-r).abs(),TOL);
    assert!((turns(&t.pieces[0],s,c).abs()-1.).abs() < 1e-9);
}

#[test]
fn two_loops_close_together_are_both_found() {
    // two circles a two-hundredth apart, each a tenth of the feature size across
    let s = [1.,1.];
    let (c1,c2,r) = ([0.31,0.27],[0.335,0.27],0.01);
    let plane = Plane::new(s,vec![disc(c1,r),disc(c2,r)])
        .bounded(move |cell| least_bound(&[disc_bound(cell,s,c1,r),disc_bound(cell,s,c2,r)]));
    let t = trace(&plane,&question([[-1.,1.],[-1.,1.]],TOL,0.2)).unwrap();
    assert_eq!(t.pieces.len(),2,"{:?} {:?}",t.search,t.pieces.iter().map(|p| (p.ends,p.uv.len(),p.uv[0])).collect::<Vec<_>>());
    assert!(t.pieces.iter().all(|p| p.closed()));
    assert!(t.search.unresolved.is_empty() && t.search.sampled == 0,"{:?}",t.search);
    let dist = |p: [f64;2]| ((p[0]-c1[0]).dhypot(p[1]-c1[1])-r).abs().min(((p[0]-c2[0]).dhypot(p[1]-c2[1])-r).abs());
    assert_on(&t,s,dist,TOL);
    for (c,i) in [(c1,0),(c2,1)] {
        let p = t.pieces.iter().find(|p| p.at[0].0 == i).unwrap();
        assert!((turns(p,s,c).abs()-1.).abs() < 1e-9);
    }
}

#[test]
fn loops_are_found_without_a_bound_by_sampling() {
    // three loops, one a fiftieth of the feature across, no bound given: found by pulling cells'
    // middles onto the zero set, and said to be sampled (two loops within the feature of each
    // other are not told apart without a bound: the hole the module states)
    let s = [1.,1.];
    let (c1,c2,c3) = ([0.31,0.27],[-0.2,0.55],[-0.6,-0.45]);
    let (r,small) = (0.01,0.004);
    let plane = Plane::new(s,vec![disc(c1,r),disc(c2,r),disc(c3,small)]);
    let t = trace(&plane,&question([[-1.,1.],[-1.,1.]],TOL,0.2)).unwrap();
    assert_eq!(t.pieces.len(),3);
    assert!(t.pieces.iter().all(|p| p.closed()));
    assert!(t.search.sampled > 0);
    let dist = |p: [f64;2]| ((p[0]-c1[0]).dhypot(p[1]-c1[1])-r).abs().min(((p[0]-c2[0]).dhypot(p[1]-c2[1])-r).abs())
        .min(((p[0]-c3[0]).dhypot(p[1]-c3[1])-small).abs());
    assert_on(&t,s,dist,TOL);
}

#[test]
fn the_union_of_two_discs_is_two_arcs_meeting_at_the_circles_crossings() {
    let s = [1.,1.];
    let (c1,c2,r) = ([-0.3,0.],[0.3,0.1],0.5);
    let plane = Plane::new(s,vec![disc(c1,r),disc(c2,r)])
        .bounded(move |cell| least_bound(&[disc_bound(cell,s,c1,r),disc_bound(cell,s,c2,r)]));
    let t = trace(&plane,&question([[-1.,1.],[-1.,1.]],TOL,0.1)).unwrap();
    assert_eq!(t.pieces.len(),2,"{:?}",t.pieces.iter().map(|p| p.ends).collect::<Vec<_>>());
    assert!(t.search.unresolved.is_empty(),"{:?}",t.search);
    // the circles' crossings, in closed form
    let (dx,dy) = (c2[0]-c1[0],c2[1]-c1[1]);
    let d = dx.dhypot(dy);
    let h = (r*r-d*d/4.).sqrt();
    let m = [(c1[0]+c2[0])/2.,(c1[1]+c2[1])/2.];
    let crossings = [[m[0]-h*dy/d,m[1]+h*dx/d],[m[0]+h*dy/d,m[1]-h*dx/d]];
    let mut met = [0;2];
    for p in &t.pieces {
        assert_eq!(p.ends,[End::Switch,End::Switch]);
        // one branch along each piece, its minimiser carried
        let i = p.at[1].0;
        assert!(p.at[1..p.at.len()-1].iter().all(|a| a.0 == i));
        for (k,x) in [p.uv[0],p.uv[p.uv.len()-1]].into_iter().enumerate() {
            let near = (0..2).min_by(|a,b| (crossings[*a][0]-x[0]).dhypot(crossings[*a][1]-x[1])
                .total_cmp(&(crossings[*b][0]-x[0]).dhypot(crossings[*b][1]-x[1]))).unwrap();
            let off = (crossings[near][0]-x[0]).dhypot(crossings[near][1]-x[1]);
            assert!(off <= 1e-12,"a corner {off:e} from the crossing");
            met[near] += 1;
            // the corner names the branch on its own side
            let a = if k == 0 { p.at[0] } else { p.at[p.at.len()-1] };
            assert_eq!(a.0,i);
        }
    }
    assert_eq!(met,[2,2]);
    assert_ne!(t.pieces[0].at[1].0,t.pieces[1].at[1].0);
    // the union's outline: each point on one circle and outside the other
    assert_on(&t,s,|p| {
        let (a,b) = ((p[0]-c1[0]).dhypot(p[1]-c1[1])-r,(p[0]-c2[0]).dhypot(p[1]-c2[1])-r);
        a.min(b).abs()
    },TOL);
}

#[test]
fn a_curve_touching_an_inner_edge_is_cut_there() {
    // `v = 2u²` touches the edge `v = 0` (the domain `v ≥ 0`) at the origin: two pieces, each from
    // the rim to the touch, ended on the edge
    let s = [1.,1.];
    let plane = Plane::new(s,vec![Box::new(|p: [f64;2]| (p[1]-2.*p[0]*p[0],Some([-4.*p[0],1.]),0.))]).bounded(move |cell| {
        let (x,y) = xy(cell,s);
        Bound {value:y-sq(x).times(2.),grad:Some([x.times(-4.),k(1.)])}
    });
    let at = |t: f64| [-1.+2.*t,0.];
    let side = |uv: Uv| uv[1];
    let edges = [Edge {at:&at,side:&side}];
    let q = Question {rect:[[-1.,1.],[-0.5,1.]],edges:&edges,seeds:&[],tol:TOL,feature:0.1};
    let t = trace(&plane,&q).unwrap();
    assert_eq!(t.pieces.len(),2,"{:?}",t.pieces.iter().map(|p| p.ends).collect::<Vec<_>>());
    for p in &t.pieces {
        let mut ends = p.ends;
        ends.sort_by_key(|e| matches!(e,End::Edge(_)));
        assert_eq!(ends,[End::Rim,End::Edge(0)]);
        let touch = if p.ends[0] == End::Edge(0) { p.uv[0] } else { p.uv[p.uv.len()-1] };
        assert!(touch[0].abs() <= TOL && touch[1] == 0.,"the touch at {touch:?}");
        assert!(p.uv.iter().all(|uv| uv[1] >= 0.));
    }
    assert_on(&t,s,|p| graph_distance(p,&|x| 2.*x*x,[-1.,1.]),TOL);
}

#[test]
fn a_cusp_ending_on_an_inner_edge_is_reached_from_both_sides() {
    // `v² = u³` over `u ≥ 0`: two branches from the rim into the cusp at the origin, on the edge
    // `u = 0`, where the gradient vanishes — reached, not stalled at
    let s = [1.,1.];
    let plane = Plane::new(s,vec![Box::new(|p: [f64;2]| (p[1]*p[1]-p[0]*p[0]*p[0],Some([-3.*p[0]*p[0],2.*p[1]]),0.))])
        .bounded(move |cell| {
            let (x,y) = xy(cell,s);
            Bound {value:sq(y)-x*x*x,grad:Some([sq(x).times(-3.),y.times(2.)])}
        });
    let at = |t: f64| [0.,-1.2+2.4*t];
    let side = |uv: Uv| uv[0];
    let edges = [Edge {at:&at,side:&side}];
    let q = Question {rect:[[-0.5,1.],[-1.2,1.2]],edges:&edges,seeds:&[],tol:TOL,feature:0.1};
    let t = trace(&plane,&q).unwrap();
    assert_eq!(t.pieces.len(),2,"{:?}",t.pieces.iter().map(|p| p.ends).collect::<Vec<_>>());
    for p in &t.pieces {
        let mut ends = p.ends;
        ends.sort_by_key(|e| matches!(e,End::Edge(_)));
        assert_eq!(ends,[End::Rim,End::Edge(0)]);
        let cusp = if p.ends[0] == End::Edge(0) { p.uv[0] } else { p.uv[p.uv.len()-1] };
        assert!(cusp[0].dhypot(cusp[1]) <= TOL,"the cusp at {cusp:?}");
    }
    let dist = |p: [f64;2]| graph_distance(p,&|x: f64| x.max(0.).powf(1.5),[0.,1.]).min(graph_distance(p,&|x: f64| -x.max(0.).powf(1.5),[0.,1.]));
    assert_on(&t,s,dist,TOL);
}

#[test]
fn a_curve_run_rim_to_rim_is_one_piece() {
    let s = [1.,1.];
    let h = |x: f64| 0.3*(3.*x).dsin()+0.1;
    let plane = Plane::new(s,vec![Box::new(move |p: [f64;2]| (p[1]-h(p[0]),Some([-0.9*(3.*p[0]).dcos(),1.]),0.))])
        .bounded(move |cell| {
            let (x,y) = xy(cell,s);
            let (sn,cs) = x.times(3.).sin_cos();
            Bound {value:y-sn.times(0.3)-k(0.1),grad:Some([cs.times(-0.9),k(1.)])}
        });
    let t = trace(&plane,&question([[-1.,1.],[-1.,1.]],TOL,0.1)).unwrap();
    assert_eq!(t.pieces.len(),1);
    assert_eq!(t.pieces[0].ends,[End::Rim,End::Rim]);
    let p = &t.pieces[0];
    let mut xs = [p.uv[0][0],p.uv[p.uv.len()-1][0]];
    xs.sort_by(f64::total_cmp);
    assert_eq!(xs,[-1.,1.]);
    assert_on(&t,s,|p| graph_distance(p,&h,[-1.,1.]),TOL);
}

#[test]
fn a_curve_touching_the_rim_without_crossing_is_cut_at_the_touch() {
    // `v = −1 + 0.8 (u − 0.2)²` touches the bottom of the rim at `u = 0.2`
    let s = [1.,1.];
    let h = |x: f64| -1.+0.8*(x-0.2)*(x-0.2);
    let plane = Plane::new(s,vec![Box::new(move |p: [f64;2]| (p[1]-h(p[0]),Some([-1.6*(p[0]-0.2),1.]),0.))])
        .bounded(move |cell| {
            let (x,y) = xy(cell,s);
            Bound {value:y+k(1.)-sq(x-k(0.2)).times(0.8),grad:Some([(x-k(0.2)).times(-1.6),k(1.)])}
        });
    let t = trace(&plane,&question([[-1.,1.],[-1.,1.]],TOL,0.1)).unwrap();
    assert_eq!(t.pieces.len(),2,"{:?}",t.pieces.iter().map(|p| (p.ends,p.uv[0],p.uv[p.uv.len()-1])).collect::<Vec<_>>());
    for p in &t.pieces {
        assert_eq!(p.ends,[End::Rim,End::Rim]);
        let touch = [p.uv[0],p.uv[p.uv.len()-1]].into_iter().find(|uv| uv[1] == -1.).expect("an end at the touch");
        assert!((touch[0]-0.2).abs() <= 1e-3*TOL.sqrt(),"the touch at {touch:?}");
    }
    assert_on(&t,s,|p| graph_distance(p,&h,[-1.,1.]),TOL);
}

/// The depth of a plane's points under a ball of radius `r` moved along `c(t)`, `t ∈ [0, 1]`,
/// read with its minimiser and its gradient (the envelope theorem's: `(P − c) / |P − c|`).
fn depth(c: impl Fn([f64;2]) -> [f64;3]+'static,r: f64) -> Branch {
    Box::new(move |p| {
        let [cx,cy,t] = c(p);
        let d = (p[0]-cx).dhypot(p[1]-cy);
        (d-r,(d > 0.).then(|| [(p[0]-cx)/d,(p[1]-cy)/d]),t)
    })
}

#[test]
fn a_ball_moved_along_a_line_leaves_a_stadium() {
    // a plane stretched twofold along `u`, so the march's lengths are in space and not in the
    // parameters; the bound Lipschitz alone (a depth moves no faster than the point does)
    let s = [2.,1.];
    let (a,b,r) = ([-0.4,0.],[0.4,0.],0.2);
    let foot = move |p: [f64;2]| {
        let t = (((p[0]-a[0])*(b[0]-a[0])+(p[1]-a[1])*(b[1]-a[1]))/((b[0]-a[0]).powi(2)+(b[1]-a[1]).powi(2))).clamp(0.,1.);
        [a[0]+(b[0]-a[0])*t,a[1]+(b[1]-a[1])*t,t]
    };
    let plane = Plane::new(s,vec![depth(foot,r)]);
    let read = move |uv: Uv| { let x = [s[0]*uv[0],s[1]*uv[1]]; let c = foot(x); (x[0]-c[0]).dhypot(x[1]-c[1])-r };
    let plane = plane.bounded(move |cell| {
        let mid = [(cell[0][0]+cell[0][1])/2.,(cell[1][0]+cell[1][1])/2.];
        let reach = (s[0]*(cell[0][1]-cell[0][0])).dhypot(s[1]*(cell[1][1]-cell[1][0]))/2.;
        Bound::lipschitz(read(mid),1.,reach)
    });
    let t = trace(&plane,&question([[-0.5,0.5],[-0.5,0.5]],TOL,0.1)).unwrap();
    assert_eq!(t.pieces.len(),1);
    assert!(t.pieces[0].closed());
    assert!(t.search.unresolved.is_empty(),"{:?}",t.search);
    assert_on(&t,s,|p| { let c = foot(p); ((p[0]-c[0]).dhypot(p[1]-c[1])-r).abs() },TOL);
    // each point carries its instant: the ball there touches it
    for (uv,at) in t.pieces[0].uv.iter().zip(&t.pieces[0].at) {
        let c = [a[0]+(b[0]-a[0])*at.1,a[1]+(b[1]-a[1])*at.1];
        assert!(((s[0]*uv[0]-c[0]).dhypot(s[1]*uv[1]-c[1])-r).abs() <= TOL);
    }
    assert!((turns(&t.pieces[0],s,[0.,0.]).abs()-1.).abs() < 1e-9);
}

#[test]
fn a_ball_moved_round_a_circle_leaves_an_annulus() {
    let s = [1.,1.];
    let (big,r) = (0.5,0.15);
    let foot = move |p: [f64;2]| { let t = p[1].datan2(p[0]); let (sn,cs) = t.dsin_cos(); [big*cs,big*sn,t] };
    let read = move |uv: Uv| { let c = foot(uv); (uv[0]-c[0]).dhypot(uv[1]-c[1])-r };
    let plane = Plane::new(s,vec![depth(foot,r)]).bounded(move |cell| {
        let mid = [(cell[0][0]+cell[0][1])/2.,(cell[1][0]+cell[1][1])/2.];
        let reach = (cell[0][1]-cell[0][0]).dhypot(cell[1][1]-cell[1][0])/2.;
        Bound::lipschitz(read(mid),1.,reach)
    });
    let t = trace(&plane,&question([[-1.,1.],[-1.,1.]],TOL,0.1)).unwrap();
    assert_eq!(t.pieces.len(),2);
    assert!(t.pieces.iter().all(|p| p.closed()));
    assert!(t.search.unresolved.is_empty(),"{:?}",t.search);
    assert_on(&t,s,|p| ((p[0].dhypot(p[1])-big).abs()-r).abs(),TOL);
    let mut radii: Vec<f64> = t.pieces.iter().map(|p| p.uv[0][0].dhypot(p.uv[0][1])).collect();
    radii.sort_by(f64::total_cmp);
    assert!((radii[0]-(big-r)).abs() <= TOL && (radii[1]-(big+r)).abs() <= TOL,"{radii:?}");
    for p in &t.pieces { assert!((turns(p,s,[0.,0.]).abs()-1.).abs() < 1e-9) }
}

/// A function of the parameters on a surface given by its map: the callers' shapes, where the
/// parameters and space part ways.
struct Mapped {
    f: Box<dyn Fn(Uv) -> (f64,[f64;2])>,
    map: Box<dyn Fn(Uv) -> ([f64;3],[f64;3],[f64;3])>,
    bound: Option<Box<dyn Fn(Rect) -> Bound>>,
}

impl Field for Mapped {
    type At = At;
    fn least(&self,uv: Uv) -> Option<Value<At>> { let (f,g) = (self.f)(uv); Some(Value {f,grad:Some(g),at:(0,0.)}) }
    fn along(&self,uv: Uv,_: &At) -> Option<Value<At>> { self.least(uv) }
    fn surface(&self,uv: Uv) -> ([f64;3],[f64;3],[f64;3]) { (self.map)(uv) }
    fn bound(&self,cell: Rect) -> Option<Bound> { self.bound.as_ref().map(|b| b(cell)) }
}

fn flat(uv: Uv) -> ([f64;3],[f64;3],[f64;3]) { ([uv[0],uv[1],0.],[1.,0.,0.],[0.,1.,0.]) }

#[test]
fn a_zero_ill_placed_in_the_parameter_is_placed_in_space() {
    // the plane `x = u³`, `y = v`: across `u = 0` the surface barely moves with `u`, so the line
    // `x = 0.2 y + 0.01` crossing there has a depth cubic in `u` — a part in a million of `f` over
    // a hundredth of the parameter — and is judged in space, not chased in the parameter
    let m = Mapped {
        f: Box::new(|uv| (uv[0].powi(3)-0.2*uv[1]-0.01,[3.*uv[0]*uv[0],-0.2])),
        map: Box::new(|uv| ([uv[0].powi(3),uv[1],0.],[3.*uv[0]*uv[0],0.,0.],[0.,1.,0.])),
        bound: Some(Box::new(|c: Rect| {
            let (u,v) = (Iv::new(c[0][0],c[0][1]),Iv::new(c[1][0],c[1][1]));
            Bound {value:u*u*u-v.times(0.2)-k(0.01),grad:Some([sq(u).times(3.),k(-0.2)])}
        })),
    };
    let t = trace(&m,&question([[-0.5,0.5],[-0.5,0.5]],TOL,0.1)).unwrap();
    assert_eq!(t.pieces.len(),1,"{:?}",t.pieces.iter().map(|p| p.ends).collect::<Vec<_>>());
    assert_eq!(t.pieces[0].ends,[End::Rim,End::Rim]);
    // in space every point, and every chord's middle mapped, is on the line
    let off = |uv: Uv| (uv[0].powi(3)-0.2*uv[1]-0.01).abs()/(1.04f64).sqrt();
    for w in t.pieces[0].uv.windows(2) {
        assert!(off(w[0]) <= TOL);
        // the chord in space between the two points' images
        let (a,b) = ([w[0][0].powi(3),w[0][1]],[w[1][0].powi(3),w[1][1]]);
        let mid = [(a[0]+b[0])/2.,(a[1]+b[1])/2.];
        assert!((mid[0]-0.2*mid[1]-0.01).abs()/(1.04f64).sqrt() <= TOL);
    }
    // it crosses the parameter's flat place
    assert!(t.pieces[0].uv.windows(2).any(|w| (w[0][0] < 0.) != (w[1][0] < 0.) || w[0][0] == 0.));
}

#[test]
fn two_branches_closing_on_a_fold_apex_end_there_and_do_not_wander() {
    // `f = (v − u²)(v − 4u²)` over `v ≥ 0` (the fold, an inner edge): two curves tangent to the fold
    // at the apex, the field between them `3u⁴` deep — under the tolerance long before the apex.
    // Four pieces, each from the rim to the apex, ended on the edge
    let m = Mapped {
        f: Box::new(|uv| {
            let (a,b) = (uv[1]-uv[0]*uv[0],uv[1]-4.*uv[0]*uv[0]);
            (a*b,[-2.*uv[0]*b-8.*uv[0]*a,a+b])
        }),
        map: Box::new(flat),
        bound: Some(Box::new(|c: Rect| {
            let (u,v) = (Iv::new(c[0][0],c[0][1]),Iv::new(c[1][0],c[1][1]));
            let (a,b) = (v-sq(u),v-sq(u).times(4.));
            Bound {value:a*b,grad:Some([u.times(-2.)*b-u.times(8.)*a,a+b])}
        })),
    };
    let at = |t: f64| [-0.5+t,0.];
    let side = |uv: Uv| uv[1];
    let edges = [Edge {at:&at,side:&side}];
    let q = Question {rect:[[-0.5,0.5],[-0.2,0.5]],edges:&edges,seeds:&[],tol:TOL,feature:0.1};
    let t = trace(&m,&q).unwrap();
    assert_eq!(t.pieces.len(),4,"{:?}",t.pieces.iter().map(|p| (p.ends,p.uv[0],p.uv[p.uv.len()-1])).collect::<Vec<_>>());
    for p in &t.pieces {
        let mut ends = p.ends;
        ends.sort_by_key(|e| matches!(e,End::Edge(_)));
        assert_eq!(ends,[End::Rim,End::Edge(0)]);
        let apex = if p.ends[0] == End::Edge(0) { p.uv[0] } else { p.uv[p.uv.len()-1] };
        assert!(apex[0].dhypot(apex[1]) <= TOL,"the apex at {apex:?}");
        // one curve along each piece: the rim end tells which, and no point strays to the other
        // farther than the tolerance
        let rim = if p.ends[0] == End::Rim { p.uv[0] } else { p.uv[p.uv.len()-1] };
        let c = if (rim[1]-rim[0]*rim[0]).abs() < (rim[1]-4.*rim[0]*rim[0]).abs() { 1. } else { 4. };
        for uv in &p.uv { assert!(graph_distance(*uv,&|x| c*x*x,[-0.5,0.5]) <= TOL,"{uv:?} off y = {c} x²"); }
    }
}

#[test]
fn a_wedge_pinching_to_a_cusp_at_the_rectangles_corner() {
    // `f = (v − u²)(v − 2u²)` over `[0, 1]²`: a wedge between two curves leaving the corner tangent
    // to `v = 0`, its far curve ending exactly at the corner. Two pieces, rim to the corner
    let m = Mapped {
        f: Box::new(|uv| {
            let (a,b) = (uv[1]-uv[0]*uv[0],uv[1]-2.*uv[0]*uv[0]);
            (a*b,[-2.*uv[0]*b-4.*uv[0]*a,a+b])
        }),
        map: Box::new(flat),
        bound: Some(Box::new(|c: Rect| {
            let (u,v) = (Iv::new(c[0][0],c[0][1]),Iv::new(c[1][0],c[1][1]));
            let (a,b) = (v-sq(u),v-sq(u).times(2.));
            Bound {value:a*b,grad:Some([u.times(-2.)*b-u.times(4.)*a,a+b])}
        })),
    };
    let t = trace(&m,&question([[0.,1.],[0.,1.]],TOL,0.1)).unwrap();
    assert_eq!(t.pieces.len(),2,"{:?}",t.pieces.iter().map(|p| (p.ends,p.uv[0],p.uv[p.uv.len()-1])).collect::<Vec<_>>());
    for p in &t.pieces {
        assert_eq!(p.ends,[End::Rim,End::Rim]);
        let ends = [p.uv[0],p.uv[p.uv.len()-1]];
        assert!(ends.iter().any(|e| e[0].dhypot(e[1]) <= TOL),"no end at the corner: {ends:?}");
        let dist = |uv: Uv| graph_distance(uv,&|x| x*x,[0.,1.]).min(graph_distance(uv,&|x| 2.*x*x,[0.,1.]));
        for uv in &p.uv { assert!(dist(*uv) <= TOL) }
    }
}

/// Two curves `δ` either side of `v = 0.2 sin 2u`: `f = (v − h)² − δ²`, a long meeting all but
/// tangent, `f` under `δ²` across a band far wider than the curves are apart.
fn near_tangent(delta: f64) -> Mapped {
    let h = |u: f64| 0.2*(2.*u).dsin();
    let f = move |uv: Uv| { let d = uv[1]-h(uv[0]); (d*d-delta*delta,[-2.*d*0.4*(2.*uv[0]).dcos(),2.*d]) };
    Mapped {
        f: Box::new(f),
        map: Box::new(flat),
        // Lipschitz alone: `|∇f| = 2|d| |(−h′, 1)| ≤ 2 · 1.2 · 1.08` over the rectangle
        bound: Some(Box::new(move |c: Rect| {
            let mid = [(c[0][0]+c[0][1])/2.,(c[1][0]+c[1][1])/2.];
            let reach = (c[0][1]-c[0][0]).dhypot(c[1][1]-c[1][0])/2.;
            Bound::lipschitz(f(mid).0,2.6,reach)
        })),
    }
}

#[test]
fn a_long_meeting_all_but_tangent_is_answered_beside_under_a_lipschitz_bound() {
    // what a bound of `f` alone cannot clear within the feature of a traced piece is answered
    // beside it at once, not refined toward the curves cell by tiny cell (which took this band
    // twenty times as long, and a full-turn torus's longer one past the search's cap)
    let delta = 1e-4;
    let m = near_tangent(delta);
    let t = trace(&m,&question([[-1.,1.],[-1.,1.]],TOL,0.1)).unwrap();
    assert_eq!(t.pieces.len(),2,"{:?}",t.pieces.iter().map(|p| p.ends).collect::<Vec<_>>());
    assert!(t.pieces.iter().all(|p| p.ends == [End::Rim,End::Rim]));
    assert!(t.search.sought && t.search.beside > 0 && t.search.unresolved.is_empty(),"{:?}",t.search);
    let h = |u: f64| 0.2*(2.*u).dsin();
    for p in &t.pieces {
        let sign = if p.uv[1][1] > h(p.uv[1][0]) { 1. } else { -1. };
        for uv in &p.uv {
            assert!(graph_distance(*uv,&|x| h(x)+sign*delta,[-1.,1.]) <= TOL);
        }
    }
}

#[test]
fn the_pieces_through_a_callers_seeds_alone() {
    // `through`: one seed beside the upper curve, the upper curve alone, completeness not sought
    let delta = 1e-4;
    let m = near_tangent(delta);
    let seeds = [[0.3,0.2*(0.6f64).dsin()+delta]];
    let q = Question {rect:[[-1.,1.],[-1.,1.]],edges:&[],seeds:&seeds,tol:TOL,feature:0.1};
    let t = through(&m,&q).unwrap();
    assert!(!t.search.sought);
    assert_eq!(t.pieces.len(),1);
    assert_eq!(t.pieces[0].ends,[End::Rim,End::Rim]);
    let p = &t.pieces[0];
    assert!(p.uv.iter().all(|uv| uv[1] > 0.2*(2.*uv[0]).dsin()),"the seed's curve alone");
    let mut xs = [p.uv[0][0],p.uv[p.uv.len()-1][0]];
    xs.sort_by(f64::total_cmp);
    assert_eq!(xs,[-1.,1.]);
}

/// **A cell spanning a whole turn is halved across it** (#880 T1f): on a cylinder over `θ` in a
/// whole turn and its height, the distance from a ball off its axis less its radius closes a loop
/// on the far side of the turn; the search halves the whole rectangle across `θ` too (measured
/// between its sides' ends, which over a whole turn are one point, it was halved along the height
/// alone and never settled), and the loop is traced once, closed, on the ball's sphere.
#[test]
fn a_cell_spanning_a_whole_turn_is_halved_across_it() {
    use std::f64::consts::{PI,TAU};
    let (c,r) = ([-1.,0.,0.5],0.3);
    let at = |uv: Uv| { let (s,co) = uv[0].dsin_cos(); [co,s,uv[1]] };
    let map = move |uv: Uv| { let (s,co) = uv[0].dsin_cos(); ([co,s,uv[1]],[-s,co,0.],[0.,0.,1.]) };
    let f = move |uv: Uv| {
        let (x,xu,xv) = map(uv);
        let d = [x[0]-c[0],x[1]-c[1],x[2]-c[2]];
        let l = (d[0]*d[0]+d[1]*d[1]+d[2]*d[2]).sqrt();
        let g = |w: [f64;3]| (d[0]*w[0]+d[1]*w[1]+d[2]*w[2])/l;
        (l-r,[g(xu),g(xv)])
    };
    let bound = move |cell: Rect| {
        let mid = [0.5*(cell[0][0]+cell[0][1]),0.5*(cell[1][0]+cell[1][1])];
        let m = at(mid);
        let mut reach: f64 = 0.;
        for i in 0..3 { for j in 0..3 {
            let p = at([cell[0][0]+(cell[0][1]-cell[0][0])*i as f64/2.,cell[1][0]+(cell[1][1]-cell[1][0])*j as f64/2.]);
            reach = reach.max(((p[0]-m[0]).powi(2)+(p[1]-m[1]).powi(2)+(p[2]-m[2]).powi(2)).sqrt());
        } }
        // (a chord's sag over a cell's arc of the unit circle widens the reach: its square over two)
        Bound::lipschitz(f(mid).0,1.,reach*(1.+reach))
    };
    let m = Mapped {f:Box::new(f),map:Box::new(map),bound:Some(Box::new(bound))};
    let tol = 1e-6;
    let t = trace(&m,&Question {rect:[[0.,TAU],[0.,1.]],edges:&[],seeds:&[],tol,feature:0.1}).unwrap();
    let [p] = &t.pieces[..] else { panic!("one loop: {} pieces",t.pieces.len()) };
    assert!(p.closed(),"the loop closed: {:?}",p.ends);
    for uv in &p.uv {
        assert!(f(*uv).0.abs() <= tol,"a point {:e} off the sphere",f(*uv).0);
        assert!((uv[0]-PI).abs() < 0.5,"a point at {uv:?}, off the far side");
    }
    assert!(t.search.unresolved.is_empty(),"unresolved cells: {:?}",t.search.unresolved);
}

/// **A curve whose image lies on another's is still traced** (#880 T1f): a surface wound twice round
/// a cylinder brings `u` and `u + 2π` to one point, as a sheet meeting itself along its double
/// curve does. A line rim to rim at `u = 2` and a loop about `(2 + 2π, ½)` cross in space, never in
/// the parameters; the loop, found only by the search, stood in space beside the traced line, and
/// its cells were answered beside it — the test whether a point is traced is read in the chart now
/// (the line's chord within the tolerance in space and, by the metric there, in the parameters).
#[test]
fn a_loop_whose_image_crosses_a_traced_curve_is_found() {
    use std::f64::consts::TAU;
    let (c,r) = ([2.+TAU,0.5],0.05);
    let map = |uv: Uv| { let (s,co) = uv[0].dsin_cos(); ([co,s,uv[1]],[-s,co,0.],[0.,0.,1.]) };
    // (the least of the line's and the loop's signed distances in the parameters, the cylinder's
    // own metric: their zero sets the line and the circle)
    let f = move |uv: Uv| {
        let line = (uv[0]-2.,[1.,0.]);
        let d = (uv[0]-c[0]).dhypot(uv[1]-c[1]);
        let disc = (d-r,[(uv[0]-c[0])/d,(uv[1]-c[1])/d]);
        if line.0 <= disc.0 { line } else { disc }
    };
    // (1-Lipschitz in the parameters: the cell's half diagonal bounds it there)
    let bound = move |cell: Rect| {
        let mid = [0.5*(cell[0][0]+cell[0][1]),0.5*(cell[1][0]+cell[1][1])];
        Bound::lipschitz(f(mid).0,1.,0.5*(cell[0][1]-cell[0][0]).dhypot(cell[1][1]-cell[1][0]))
    };
    let m = Mapped {f:Box::new(f),map:Box::new(map),bound:Some(Box::new(bound))};
    let t = trace(&m,&question([[1.,2.*TAU-1.],[0.,1.]],TOL,0.1)).unwrap();
    assert_eq!(t.pieces.len(),2,"the line and the loop: {:?}",t.pieces.iter().map(|p| (p.uv[0],p.ends)).collect::<Vec<_>>());
    let loops: Vec<_> = t.pieces.iter().filter(|p| p.closed()).collect();
    let [lp] = loops[..] else { panic!("one loop") };
    for uv in &lp.uv { assert!(((uv[0]-c[0]).dhypot(uv[1]-c[1])-r).abs() <= TOL,"a point {uv:?} off the loop") }
}

/// **A rim crossed at two readings of one place is no piece** (#880 T1f): `f = v + g(u)` over
/// `v ≥ 0`, `g` a parabola a hundredth of the tolerance deep between roots 2.5 tolerances apart —
/// the curve within the tolerance of the rim from one root to the other, as a shallow crossing read
/// through a field known to its tolerance changes sign twice about the one root (the thin's sheet
/// `−` at its rim). The march from one root arrived at the other, a piece of two points along the
/// rim, longer than the tolerance: kept, and the run it joined doubled back. Now no piece.
#[test]
fn a_rim_crossed_at_two_readings_of_one_place_is_no_piece() {
    let (u0,d,h) = (0.5,1.25*TOL,0.01*TOL);
    let g = move |u: f64| (h*((u-u0)*(u-u0)/(d*d)-1.),2.*h*(u-u0)/(d*d));
    let m = Mapped {
        f: Box::new(move |uv| { let (gu,gd) = g(uv[0]); (uv[1]+gu,[gd,1.]) }),
        map: Box::new(flat),
        bound: Some(Box::new(move |c: Rect| {
            let (u,v) = (Iv::new(c[0][0],c[0][1]),Iv::new(c[1][0],c[1][1]));
            let w = u-k(u0);
            Bound {value:v+(sq(w).times(1./(d*d))-k(1.)).times(h),grad:Some([w.times(2.*h/(d*d)),k(1.)])}
        })),
    };
    let t = trace(&m,&question([[0.,1.],[0.,0.5]],TOL,0.1)).unwrap();
    assert!(t.pieces.is_empty(),"pieces: {:?}",t.pieces.iter().map(|p| (p.uv.clone(),p.ends)).collect::<Vec<_>>());
}

/// A function of the parameters defined only where `inside` holds: the depth's far cover, which
/// is no cover where the point's own contact holds the instant.
struct Partial { f: Mapped,inside: Box<dyn Fn(Uv) -> bool> }

impl Field for Partial {
    type At = At;
    fn least(&self,uv: Uv) -> Option<Value<At>> { (self.inside)(uv).then(|| self.f.least(uv)).flatten() }
    fn along(&self,uv: Uv,a: &At) -> Option<Value<At>> { self.f.along(uv,a) }
    fn surface(&self,uv: Uv) -> ([f64;3],[f64;3],[f64;3]) { self.f.surface(uv) }
    fn bound(&self,_: Rect) -> Option<Bound> { None }
}

/// **A curve running to where its field is not defined ends there** (#880 T1f, `End::Undefined`):
/// the line `v = ½` from the rim at `u = 0`, its field defined only left of the oblique line
/// `u = ½ + (v − ½)/5` (as the far cover is none past where its basin merges into the point's own
/// contact: the bench's tumbling torus at its corner). The march halved its step under the rounding
/// there and refused; now it ends there, within the tolerance of that edge.
#[test]
fn a_curve_running_to_where_its_field_is_undefined_ends_there() {
    let edge = |uv: Uv| 0.5+0.2*(uv[1]-0.5)-uv[0];
    let p = Partial {
        f: Mapped {f:Box::new(|uv| (uv[1]-0.5,[0.,1.])),map:Box::new(flat),bound:None},
        inside: Box::new(move |uv| edge(uv) >= 0.),
    };
    let t = trace(&p,&question([[0.,1.],[0.,1.]],TOL,0.1)).unwrap();
    let [piece] = &t.pieces[..] else { panic!("one piece: {:?}",t.pieces.iter().map(|p| p.ends).collect::<Vec<_>>()) };
    let ends = [piece.uv[0],piece.uv[piece.uv.len()-1]];
    let undefined = (0..2).find(|&e| piece.ends[e] == End::Undefined).expect("an end where the field stops");
    assert_eq!(piece.ends[1-undefined],End::Rim);
    let q = ends[undefined];
    assert!(edge(q) >= 0. && edge(q) <= 2.*TOL,"the end {q:?} {:e} from the field's edge",edge(q));
    for uv in &piece.uv { assert!((uv[1]-0.5).abs() <= TOL) }
}
