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
use gcs_core::brep::zero_set::{trace,Bound,Edge,End,Field,Piece,Question,Rect,Traced,Value};
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
    let d = (sq(dx)+sq(dy)).sqrt();
    Bound {value:d-k(r),grad:[(dx/d).times(s[0]),(dy/d).times(s[1])]}
}
/// The least of several branches' bounds: each branch's gradient joins where it may be the least.
fn least_bound(bs: &[Bound]) -> Bound {
    let lo = bs.iter().map(|b| b.value.lo()).fold(f64::INFINITY,f64::min);
    let hi = bs.iter().map(|b| b.value.hi()).fold(f64::INFINITY,f64::min);
    let mut grad: Option<[Iv;2]> = None;
    for b in bs {
        if b.value.lo() <= hi {
            grad = Some(match grad { None => b.grad,Some(g) => [g[0].hull(b.grad[0]),g[1].hull(b.grad[1])] });
        }
    }
    Bound {value:Iv::new(lo,hi),grad:grad.unwrap()}
}

fn question(rect: Rect,tol: f64,feature: f64) -> Question<'static> { Question {rect,edges:&[],seeds:&[],tol,feature} }

const TOL: f64 = 1e-6;

/// Every point of every piece, and every chord's middle, in space within `tol` of the zero set
/// whose distance in space is `distance`.
fn assert_on(t: &Traced<At>,s: [f64;2],distance: impl Fn([f64;2]) -> f64,tol: f64) {
    let mut worst: f64 = 0.;
    for p in &t.pieces {
        assert!(p.uv.len() >= 2,"a piece of one point: {p:?}");
        for w in p.uv.windows(2) {
            for x in [w[0],[(w[0][0]+w[1][0])/2.,(w[0][1]+w[1][1])/2.]] {
                worst = worst.max(distance([s[0]*x[0],s[1]*x[1]]));
            }
        }
    }
    assert!(worst <= tol,"a traced point or chord stands {worst:e} from the zero set");
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
    let mut t = (0..4096).map(|i| i as f64*std::f64::consts::TAU/4096.).min_by(|u,v| d(*u).total_cmp(&d(*v))).unwrap();
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
    let n = 20000;
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
        let e = Bound {value:sq(ex).times(1./(a*a))+sq(ey).times(1./(b*b))-k(1.),grad:[ex.times(2./(a*a)),ey.times(2./(b*b))]};
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
    assert_eq!(t.pieces.len(),2);
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
    // the same two loops and a third far under the feature, no bound given: found by pulling cells'
    // middles onto the zero set and casting rays from critical points, and said to be sampled
    let s = [1.,1.];
    let (c1,c2,c3,r) = ([0.31,0.27],[0.335,0.27],[-0.6,-0.45],0.01);
    let plane = Plane::new(s,vec![disc(c1,r),disc(c2,r),ellipse_branch(c3,0.004,0.003)]);
    let t = trace(&plane,&question([[-1.,1.],[-1.,1.]],TOL,0.2)).unwrap();
    assert_eq!(t.pieces.len(),3);
    assert!(t.pieces.iter().all(|p| p.closed()));
    assert!(t.search.sampled > 0);
    let dist = |p: [f64;2]| ((p[0]-c1[0]).dhypot(p[1]-c1[1])-r).abs().min(((p[0]-c2[0]).dhypot(p[1]-c2[1])-r).abs())
        .min(ellipse_distance(p,c3,0.004,0.003));
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
        Bound {value:y-sq(x).times(2.),grad:[x.times(-4.),k(1.)]}
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
            Bound {value:sq(y)-x*x*x,grad:[sq(x).times(-3.),y.times(2.)]}
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
            Bound {value:y-sn.times(0.3)-k(0.1),grad:[cs.times(-0.9),k(1.)]}
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
            Bound {value:y+k(1.)-sq(x-k(0.2)).times(0.8),grad:[(x-k(0.2)).times(-1.6),k(1.)]}
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
