//! A B-rep written as STEP (ISO 10303-21, AP214's advanced B-rep): the product and units a
//! reader expects (millimetres, radians), a manifold solid on one closed shell, its faces on
//! their exact surfaces and its edges on their exact curves; a traced curve written as the
//! B-spline of degree one through points within `tol` of it; every edge's curve in each face that
//! uses it (its pcurve, a seam's two) written beside it, so a reader need not project its own; and
//! a pole's degenerate edge left out of its loop, as a reader rebuilds it.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use super::geom::{Curve,Frame,Surface,V};
use super::topo::{Brep,EdgeCurve};
use crate::util::space::{distance,scale};
use std::fmt::Write;

/// A real as STEP writes one: shortest round-trip digits, a decimal point, `E` for the exponent.
fn real(x: f64) -> String {
    let x = if x == 0. { 0. } else { x };
    let s = format!("{x:?}");
    match s.split_once('e') {
        Some((m,e)) => format!("{}{}E{e}",m,if m.contains('.') { "" } else { "." }),
        None if s.contains('.') => s,
        None => format!("{s}."),
    }
}

struct Out { text: String,next: usize }

impl Out {
    fn add(&mut self,entity: String) -> usize {
        let id = self.next;
        self.next += 1;
        let _ = writeln!(self.text,"#{id} = {entity};");
        id
    }
    fn point(&mut self,p: V) -> usize { self.add(format!("CARTESIAN_POINT('',({},{},{}))",real(p[0]),real(p[1]),real(p[2]))) }
    fn direction(&mut self,d: V) -> usize { self.add(format!("DIRECTION('',({},{},{}))",real(d[0]),real(d[1]),real(d[2]))) }
    fn placement(&mut self,f: &Frame) -> usize {
        let (o,z,x) = (self.point(f.o),self.direction(f.z),self.direction(f.x));
        self.add(format!("AXIS2_PLACEMENT_3D('',#{o},#{z},#{x})"))
    }
    fn surface(&mut self,s: &Surface) -> Result<usize,String> {
        match s {
            Surface::Extrusion(f,c) => {
                let c = self.swept(c);
                let d = self.direction(f.z);
                let v = self.add(format!("VECTOR('',#{d},1.)"));
                return Ok(self.add(format!("SURFACE_OF_LINEAR_EXTRUSION('',#{c},#{v})")))
            }
            Surface::Blend(..) | Surface::BSpline(..) | Surface::Family(..) => {
                let net = match s { Surface::Blend(_,b) => blend_net(b)?,Surface::BSpline(_,n) => (**n).clone(),
                    Surface::Family(_,w) => family_net(w)?,_ => unreachable!() };
                let rows: Vec<String> = net.poles.iter().map(|col| {
                    let ids: Vec<String> = col.iter().map(|&p| format!("#{}",self.point(p))).collect();
                    format!("({})",ids.join(","))
                }).collect();
                let (uk,vk) = (super::nurbs::distinct(&net.uknots),super::nurbs::distinct(&net.vknots));
                let list = |k: &[(f64,usize)],f: &dyn Fn(&(f64,usize)) -> String| k.iter().map(f).collect::<Vec<_>>().join(",");
                if let Some(w) = &net.weights {
                    let w: Vec<String> = w.iter().map(|r| format!("({})",r.iter().map(|&w| real(w)).collect::<Vec<_>>().join(","))).collect();
                    return Ok(self.add(format!("( BOUNDED_SURFACE() B_SPLINE_SURFACE({},{},({}),.UNSPECIFIED.,.F.,.F.,.F.) \
                        B_SPLINE_SURFACE_WITH_KNOTS(({}),({}),({}),({}),.UNSPECIFIED.) GEOMETRIC_REPRESENTATION_ITEM() \
                        RATIONAL_B_SPLINE_SURFACE(({})) REPRESENTATION_ITEM('') SURFACE() )",
                        net.du,net.dv,rows.join(","),list(&uk,&|k| k.1.to_string()),list(&vk,&|k| k.1.to_string()),
                        list(&uk,&|k| real(k.0)),list(&vk,&|k| real(k.0)),w.join(","))))
                }
                return Ok(self.add(format!("B_SPLINE_SURFACE_WITH_KNOTS('',{},{},({}),.UNSPECIFIED.,.F.,.F.,.F.,({}),({}),({}),({}),.UNSPECIFIED.)",
                    net.du,net.dv,rows.join(","),list(&uk,&|k| k.1.to_string()),list(&vk,&|k| k.1.to_string()),
                    list(&uk,&|k| real(k.0)),list(&vk,&|k| real(k.0)))))
            }
            Surface::Revolution(f,c) => {
                let c = self.swept(c);
                let (o,z) = (self.point(f.o),self.direction(f.z));
                let a = self.add(format!("AXIS1_PLACEMENT('',#{o},#{z})"));
                return Ok(self.add(format!("SURFACE_OF_REVOLUTION('',#{c},#{a})")))
            }
            _ => {}
        }
        Ok(match *s {
            Surface::Plane(f) => { let a = self.placement(&f); self.add(format!("PLANE('',#{a})")) }
            Surface::Cylinder(f,r) => { let a = self.placement(&f); self.add(format!("CYLINDRICAL_SURFACE('',#{a},{})",real(r))) }
            Surface::Cone(f,r,a) => {
                // STEP's semi-angle is positive: a cone opening against its z is the same cone
                // about the reversed axis (x kept, y reversed), its normal unchanged
                let f = if a < 0. { Frame {o:f.o,x:f.x,y:scale(f.y,-1.),z:scale(f.z,-1.)} } else { f };
                let p = self.placement(&f);
                self.add(format!("CONICAL_SURFACE('',#{p},{},{})",real(r),real(a.abs())))
            }
            Surface::Sphere(f,r) => { let a = self.placement(&f); self.add(format!("SPHERICAL_SURFACE('',#{a},{})",real(r))) }
            Surface::Torus(f,big,r) => {
                let a = self.placement(&f);
                self.add(format!("TOROIDAL_SURFACE('',#{a},{},{})",real(big),real(r)))
            }
            Surface::Extrusion(..) | Surface::Revolution(..) | Surface::Blend(..) | Surface::BSpline(..) | Surface::Family(..) => unreachable!("written above"),
        })
    }
    /// The curve a swept surface sweeps (a B-spline, as a profile gives one).
    fn swept(&mut self,c: &Curve) -> usize {
        let Curve::BSpline(b) = c else { unreachable!("a swept surface's curve is a profile's B-spline") };
        let ids: Vec<String> = b.poles.iter().map(|&p| format!("#{}",self.point(p))).collect();
        spline(self,b.degree,&ids,&b.knots,b.weights.as_deref())
    }
}

/// A B-spline of `degree` through the poles `ids` over the full knot vector `knots` (written as
/// its distinct knots and their multiplicities), rational where it has `weights`: the complex
/// entity, its parts in the alphabetical order the standard asks.
fn spline(o: &mut Out,degree: usize,ids: &[String],knots: &[f64],weights: Option<&[f64]>) -> usize {
    let distinct = super::nurbs::distinct(knots);
    let mults: Vec<String> = distinct.iter().map(|d| d.1.to_string()).collect();
    let ks: Vec<String> = distinct.iter().map(|d| real(d.0)).collect();
    if let Some(w) = weights {
        let w: Vec<String> = w.iter().map(|&w| real(w)).collect();
        return o.add(format!("( BOUNDED_CURVE() B_SPLINE_CURVE({degree},({}),.UNSPECIFIED.,.F.,.F.) B_SPLINE_CURVE_WITH_KNOTS(({}),({}),.UNSPECIFIED.) \
            CURVE() GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE(({})) REPRESENTATION_ITEM('') )",ids.join(","),mults.join(","),ks.join(","),w.join(",")))
    }
    o.add(format!("B_SPLINE_CURVE_WITH_KNOTS('',{degree},({}),.UNSPECIFIED.,.F.,.F.,({}),({}),.UNSPECIFIED.)",
        ids.join(","),mults.join(","),ks.join(",")))
}

/// The parameters of a stretch of a curve at which `at` (a point, in space or in a face's
/// parameters) may be joined by chords whose middles map within `tol` of the curve (`off`
/// measures that): a traced curve's own points between the ends, and more wherever a chord strays.
fn samples(c: &Curve,t: [f64;2],tol: f64,off: &dyn Fn(f64,f64) -> f64) -> Vec<f64> {
    let mut ts: Vec<f64> = vec![t[0]];
    if let Curve::Traced(_) = c { ts.extend((t[0].floor() as i64+1..=t[1].ceil() as i64-1).map(|k| k as f64)); }
    else { ts.extend((1..8).map(|k| t[0]+(t[1]-t[0])*k as f64/8.)); }
    ts.push(t[1]);
    let mut out = vec![t[0]];
    for w in ts.windows(2) {
        let mut stack = vec![(w[0],w[1],0)];
        while let Some((a,z,depth)) = stack.pop() {
            if depth < 24 && off(a,z) > tol { let m = (a+z)/2.; stack.push((m,z,depth+1)); stack.push((a,m,depth+1)); }
            else { out.push(z); }
        }
    }
    out
}

/// How near its exact surface or curve a loft's face or rail is written (millimetres): the file
/// carries a B-spline fitted within this, its parameters the exact geometry's.
pub const FIT: f64 = 1e-6;

/// A loft face's B-spline, fitted within `FIT`: cubic across the section, and cubic along the guide
/// where it turns or linear where it runs straight (a blend carried along a line is linear in `v`).
/// A screw a cubic net cannot follow within `FIT` at the fit's cap (a section carried round many
/// turns, #243 L6) is fitted quintic along it, as a face family's wall is. Refused where no net
/// the fit may grow to comes within `FIT`, rather than written further off.
pub fn blend_net(b: &super::geom::Blend) -> Result<super::nurbs::Net,String> {
    let (dv,nv) = match b.carry {
        super::geom::Carry::Line {..} => (1,1),
        // a quarter of a turn a span to start with, as a helix wants
        super::geom::Carry::Screw {angle,..} => (3,((angle.abs()/std::f64::consts::FRAC_PI_2).ceil() as usize).max(4)),
    };
    let f = |u,v| b.d1([u,v]).0;
    let fitted = match super::nurbs::fit_net(&f,3,dv,8,nv,FIT) {
        Some((_,err)) if err > FIT && dv == 3 => super::nurbs::fit_net(&f,3,5,8,nv,FIT),
        other => other,
    };
    match fitted {
        Some((net,err)) if err <= FIT => Ok(net),
        Some((_,err)) => Err(format!("a loft face is fitted no nearer than {err:.3e} mm to its surface, past the file's {FIT:e}")),
        None => Err("a loft face could not be fitted (its surface is degenerate)".into()),
    }
}

/// A face family's wall as the file writes it (#243 L2): the net through it fitted within `FIT` as
/// a loft face is — cubic along its edge, quintic along the family, whose rails may wind many
/// times (a cubic through a twisted wall's 512 rows of a three-and-a-half-turn auger misses by
/// 1e-6) — its `v` the family's running number over its interval.
pub fn family_net(w: &super::family::Sheet) -> Result<super::nurbs::Net,String> {
    let [a,b] = w.span();
    let pieces = w.stations.spans.len().clamp(4,64);
    match super::nurbs::fit_net(&|u,v| w.d1([u,a+(b-a)*v]).0,3,5,8,pieces,FIT) {
        Some((mut net,err)) if err <= FIT => {
            for k in &mut net.vknots { *k = a+(b-a)**k; }
            Ok(net)
        }
        Some((_,err)) => Err(format!("a face family's wall is fitted no nearer than {err:.3e} mm to its surface, past the file's {FIT:e}")),
        None => Err("a face family's wall could not be fitted (its surface is degenerate)".into()),
    }
}

/// A B-spline surface's numbers as a reader of the file compares them: its degrees, the size of
/// its net, its distinct knots' counts, whether rational, the knots and their multiplicities along
/// `u` then `v`, then its poles by `u` then `v`, and a rational one's weights in the same order.
pub fn net_numbers(net: &super::nurbs::Net) -> Vec<f64> {
    let (uk,vk) = (super::nurbs::distinct(&net.uknots),super::nurbs::distinct(&net.vknots));
    let rational = if net.is_rational() { 1. } else { 0. };
    let mut out = vec![net.du as f64,net.dv as f64,net.poles.len() as f64,net.poles[0].len() as f64,uk.len() as f64,vk.len() as f64,rational];
    out.extend(uk.iter().map(|k| k.0)); out.extend(uk.iter().map(|k| k.1 as f64));
    out.extend(vk.iter().map(|k| k.0)); out.extend(vk.iter().map(|k| k.1 as f64));
    for col in &net.poles { for p in col { out.extend(p); } }
    if let Some(w) = &net.weights { for col in w { out.extend(col); } }
    out
}

/// A degree-one B-spline through `pts` with the knots `ts` (its parameter the curve's own).
fn bspline(o: &mut Out,pts: &[String],ts: &[f64]) -> usize {
    let n = pts.len();
    let knots: Vec<String> = ts.iter().map(|&t| real(t)).collect();
    let mults: Vec<String> = (0..n).map(|k| if k == 0 || k == n-1 { "2".into() } else { "1".into() }).collect();
    o.add(format!("B_SPLINE_CURVE_WITH_KNOTS('',1,({}),.UNSPECIFIED.,.F.,.F.,({}),({}),.UNSPECIFIED.)",
        pts.join(","),mults.join(","),knots.join(",")))
}

/// A surface as the file writes it: its placement (origin, axis, reference direction) and its
/// numbers — a cone opening against its axis written about the reversed one.
pub fn written(s: &Surface) -> (Frame,Vec<f64>) {
    match s.clone() {
        Surface::Plane(f) => (f,vec![]),
        Surface::Cylinder(f,r) => (f,vec![r]),
        Surface::Cone(f,r,a) => (if a < 0. { Frame {o:f.o,x:f.x,y:scale(f.y,-1.),z:scale(f.z,-1.)} } else { f },vec![r,a.abs()]),
        Surface::Sphere(f,r) => (f,vec![r]),
        Surface::Torus(f,big,r) => (f,vec![big,r]),
        Surface::Extrusion(f,_) | Surface::Revolution(f,_) | Surface::Blend(f,_) | Surface::BSpline(f,_) | Surface::Family(f,_) => (f,vec![]),
    }
}

/// A face's parameters as STEP's surface reads them: the same, but on a cone written about its
/// reversed axis, where both run the other way.
fn step_uv(s: &Surface,uv: super::geom::Uv) -> [f64;2] {
    match s { Surface::Cone(_,_,a) if *a < 0. => [-uv[0],-uv[1]],_ => uv }
}

/// The curve `uv(t)` over `[t0, t1]` as cubic Bézier pieces (a B-spline of degree 3 whose inner
/// knots are triple): cut at `breaks` (the edge's own knots, where its derivatives may jump) and
/// halved wherever a piece's quarter points come onto `surface` farther than a tenth of `FIT`
/// from `uv(t)`'s own; each piece is the Hermite cubic of its ends' values and derivatives.
/// Measured against the edge's foot on the surface, not the edge: a fitted intersection curve
/// stands off each of its surfaces by its own fit, which no pcurve can close. None where a piece
/// halved thirty times still misses, or past 4096 pieces.
fn bezier_pcurve(uv: &dyn Fn(f64) -> super::geom::Uv,duv: &dyn Fn(f64) -> super::geom::Uv,surface: &Surface,[t0,t1]: [f64;2],
    breaks: &[f64]) -> Option<(Vec<[f64;2]>,Vec<f64>)> {
    let hermite = |a: f64,b: f64| -> [[f64;2];4] {
        let (p,q,dp,dq) = (uv(a),uv(b),duv(a),duv(b));
        let h = (b-a)/3.;
        [p,[p[0]+h*dp[0],p[1]+h*dp[1]],[q[0]-h*dq[0],q[1]-h*dq[1]],q]
    };
    let at = |c: &[[f64;2];4],s: f64| -> [f64;2] {
        let w = [(1.-s).dpowi(3),3.*s*(1.-s)*(1.-s),3.*s*s*(1.-s),s.dpowi(3)];
        [(0..4).map(|k| w[k]*c[k][0]).sum(),(0..4).map(|k| w[k]*c[k][1]).sum()]
    };
    let mut cuts = vec![t0];
    cuts.extend(breaks.iter().copied().filter(|&k| k > t0 && k < t1));
    cuts.push(t1);
    let mut pieces: Vec<(f64,f64,[[f64;2];4])> = Vec::new();
    for w in cuts.windows(2) {
        let mut stack = vec![(w[0],w[1],0)];
        while let Some((a,b,depth)) = stack.pop() {
            let c = hermite(a,b);
            let off = [0.25,0.5,0.75].iter().map(|&f| distance(surface.point(at(&c,f)),surface.point(uv(a+(b-a)*f)))).fold(0.,f64::max);
            if off > FIT/10. {
                if depth >= 30 || pieces.len()+stack.len() > 4096 { return None }
                let m = (a+b)/2.;
                stack.push((m,b,depth+1));
                stack.push((a,m,depth+1));
            } else { pieces.push((a,b,c)); }
        }
    }
    let mut poles = vec![pieces[0].2[0]];
    let mut knots = vec![t0;4];
    for (k,(_,b,c)) in pieces.iter().enumerate() {
        poles.extend_from_slice(&c[1..]);
        knots.extend(std::iter::repeat_n(*b,if k+1 == pieces.len() { 4 } else { 3 }));
    }
    Some((poles,knots))
}

/// A use's pcurve, as a STEP `PCURVE` on the face's surface: a line where it is one, a circle
/// where a plane's edge is one turning the plane's way, otherwise the degree-one B-spline through
/// its parameters at points chosen so each chord's middle maps within `tol` of the edge.
fn pcurve(o: &mut Out,b: &Brep,fi: usize,u: &super::topo::Coedge,surface: usize,context2: usize,tol: f64) -> usize {
    use super::topo::Pcurve;
    let f = &b.faces[fi];
    let e = &b.edges[u.edge as usize];
    let EdgeCurve::Curve(c) = &e.curve else { unreachable!("a degenerate edge has no pcurve written") };
    let at = |t: f64| step_uv(&f.surface,u.pcurve.at(t,e,&f.surface,&b.vertices));
    let p2 = |o: &mut Out,p: [f64;2]| o.add(format!("CARTESIAN_POINT('',({},{}))",real(p[0]),real(p[1])));
    let line = |o: &mut Out,origin: [f64;2],d: [f64;2]| {
        let l = d[0].dhypot(d[1]);
        let p = p2(o,origin);
        let dir = o.add(format!("DIRECTION('',({},{}))",real(d[0]/l),real(d[1]/l)));
        let v = o.add(format!("VECTOR('',#{dir},{})",real(l)));
        o.add(format!("LINE('',#{p},#{v})"))
    };
    let (a,z) = (at(e.t[0]),at(e.t[1]));
    // a reader places a closed curve's vertices within its first period: the pcurve's parameter is
    // the edge's moved there too (a traced curve is written over its own parameters, unmoved)
    let shift = match c { Curve::Circle(..) | Curve::Ellipse(..) => (e.t[0]/std::f64::consts::TAU).floor()*std::f64::consts::TAU,_ => 0. };
    // affine in the edge's parameter (a coaxial circle on a cylinder, a line along its axis)
    let affine = (1..8).all(|k| {
        let s = k as f64/8.;
        let q = at(e.t[0]+(e.t[1]-e.t[0])*s);
        (q[0]-(a[0]+s*(z[0]-a[0]))).abs().max((q[1]-(a[1]+s*(z[1]-a[1]))).abs()) <= 1e-12*(1.+a[0].abs().max(a[1].abs()))
    });
    let curve = match (&u.pcurve,&f.surface,c) {
        // linear in the edge's parameter: a line through where it is at t = 0
        (Pcurve::Line {..},_,_) | (_,_,_) if affine && (z[0]-a[0]).dhypot(z[1]-a[1]) > 0. => {
            let d = [(z[0]-a[0])/(e.t[1]-e.t[0]),(z[1]-a[1])/(e.t[1]-e.t[0])];
            line(o,[a[0]-d[0]*(e.t[0]-shift),a[1]-d[1]*(e.t[0]-shift)],d)
        }
        // a kernel's own curve in the face's parameters (`brep::json`): its poles, exactly
        (Pcurve::Curve(pc),_,_) if matches!(**pc,Curve::BSpline(_)) => {
            let Curve::BSpline(bs) = &**pc else { unreachable!() };
            let ids: Vec<String> = bs.poles.iter().map(|q| format!("#{}",p2(o,step_uv(&f.surface,[q[0],q[1]])))).collect();
            let knots: Vec<f64> = bs.knots.iter().map(|k| k-shift).collect();
            spline(o,bs.degree,&ids,&knots,bs.weights.as_deref())
        }
        (_,Surface::Plane(pl),Curve::Line {p,d}) => {
            let (l,dl) = (pl.local(*p),pl.dir_local(*d));
            line(o,[l[0],l[1]],[dl[0],dl[1]])
        }
        (_,Surface::Plane(pl),Curve::Circle(cf,r)) if crate::util::space::dot(cf.z,pl.z) > 0. => {
            let (l,x) = (pl.local(cf.o),pl.dir_local(cf.x));
            let p = p2(o,[l[0],l[1]]);
            let dir = o.add(format!("DIRECTION('',({},{}))",real(x[0]),real(x[1])));
            let ax = o.add(format!("AXIS2_PLACEMENT_2D('',#{p},#{dir})"));
            o.add(format!("CIRCLE('',#{ax},{})",real(*r)))
        }
        // a plane's parameters are an affine image of space: a B-spline's poles carried over
        (_,Surface::Plane(pl),Curve::BSpline(bs)) => {
            let ids: Vec<String> = bs.poles.iter().map(|&q| { let l = pl.local(q); format!("#{}",p2(o,[l[0],l[1]])) }).collect();
            spline(o,bs.degree,&ids,&bs.knots,bs.weights.as_deref())
        }
        // a cubic in the edge's parameter through its parameters at evenly spaced points, doubled
        // until it maps within a tenth of `FIT` of the edge between them (a chordal pcurve within
        // the file's tolerance trims a spline face that far off its edge, and a reader measures
        // the face it trims: 1e-5 of a pinion read back)
        _ if let Some((poles,knots)) = bezier_pcurve(&|t| u.pcurve.at(t,e,&f.surface,&b.vertices),
            &|t| u.pcurve.derivative(t,e,&f.surface,&b.vertices),&f.surface,e.t,&c.breaks(e.t)) => {
            let ids: Vec<String> = poles.iter().map(|&q| format!("#{}",p2(o,step_uv(&f.surface,q)))).collect();
            let knots: Vec<f64> = knots.iter().map(|k| k-shift).collect();
            spline(o,3,&ids,&knots,None)
        }
        _ => {
            let off = |ta: f64,tz: f64| {
                let (ua,uz) = (u.pcurve.at(ta,e,&f.surface,&b.vertices),u.pcurve.at(tz,e,&f.surface,&b.vertices));
                distance(f.surface.point([(ua[0]+uz[0])/2.,(ua[1]+uz[1])/2.]),c.point((ta+tz)/2.))
            };
            let ts = samples(c,e.t,tol,&off);
            if super::debug() {
                let middle = crate::util::warm::kept(|| at((e.t[0]+e.t[1])/2.));
                eprintln!("step: a {} on a {} as {} points, ends {a:?} {z:?}, middle {middle:?}",c.kind(),f.surface.kind(),ts.len());
            }
            let ids: Vec<String> = ts.iter().map(|&t| format!("#{}",p2(o,at(t)))).collect();
            let knots: Vec<f64> = ts.iter().map(|t| t-shift).collect();
            bspline(o,&ids,&knots)
        }
    };
    let rep = o.add(format!("DEFINITIONAL_REPRESENTATION('',(#{curve}),#{context2})"));
    o.add(format!("PCURVE('',#{surface},#{rep})"))
}

/// The solid as a STEP file, `name` its product's.
/// Refused where a loft's face or rail cannot be fitted within `FIT`.
pub fn write(b: &Brep,name: &str,tol: f64) -> Result<String,String> {
    let mut o = Out {text:String::new(),next:1};
    let context = application(&mut o);
    let pctx = o.add(format!("PRODUCT_CONTEXT('',#{context},'mechanical')"));
    let name = name.replace('\'',"''");
    let (_,shape) = product(&mut o,&name,context,pctx,&mut None);
    let gctx = geometric(&mut o,tol);
    let solid = solid(&mut o,b,tol)?;
    let origin = o.placement(&Frame::WORLD);
    let rep = o.add(format!("ADVANCED_BREP_SHAPE_REPRESENTATION('',(#{origin},#{solid}),#{gctx})"));
    o.add(format!("SHAPE_DEFINITION_REPRESENTATION(#{shape},#{rep})"));
    Ok(file(&name,&o.text))
}

/// **An assembly as STEP's product structure** (AP214): products, each a part with its solid or an
/// assembly of others, and each use of a product in an assembly placed where it stands there. The
/// first product is the root; a product is used only by products before it, so the structure is
/// a tree laid out top down, and a part one definition however many times it is used.
pub struct Assembly { pub products: Vec<Product>,pub uses: Vec<Usage> }

/// A product: a part, its solid in its own coordinates (millimetres), or an assembly (`None`); a
/// part's colour, if it has one, its sRGB components in 0..=1 as `COLOUR_RGB` takes them (an
/// assembly has none).
pub struct Product { pub name: String,pub solid: Option<Brep>,pub colour: Option<[f64;3]> }

/// A use of product `child` in the assembly `parent`, named, its own axes standing at `placement`
/// in the parent's (right-handed, orthonormal, millimetres).
pub struct Usage { pub parent: usize,pub child: usize,pub name: String,pub placement: Frame }

impl Assembly {
    /// Refused where it is not a tree laid out top down: a use of a product not after its
    /// parent, a part used as an assembly, a product other than the root no assembly uses, or an
    /// assembly with nothing in it; or where a colour is not a part's three numbers in 0..=1.
    pub fn check(&self) -> Result<(),String> {
        let n = self.products.len();
        if n == 0 { return Err("an assembly has no product".into()) }
        for p in &self.products {
            match p.colour {
                Some(_) if p.solid.is_none() => return Err(format!("the assembly `{}` is coloured, and only a part is",p.name)),
                Some(c) if !c.iter().all(|x| (0. ..=1.).contains(x)) => {
                    return Err(format!("the part `{}`'s colour {c:?} is not three numbers in 0..=1",p.name))
                }
                _ => {}
            }
        }
        let mut used = vec![false;n];
        let mut parent = vec![false;n];
        for u in &self.uses {
            if u.parent >= u.child || u.child >= n {
                return Err(format!("the use `{}` places product {} in product {}, which does not come before it",u.name,u.child,u.parent))
            }
            if self.products[u.parent].solid.is_some() {
                return Err(format!("the part `{}` is used as an assembly, of `{}`",self.products[u.parent].name,self.products[u.child].name))
            }
            used[u.child] = true;
            parent[u.parent] = true;
        }
        if let Some(k) = (1..n).find(|&k| !used[k]) { return Err(format!("`{}` is in no assembly",self.products[k].name)) }
        if let Some(k) = (0..n).find(|&k| self.products[k].solid.is_none() && !parent[k]) {
            return Err(format!("the assembly `{}` has nothing in it",self.products[k].name))
        }
        Ok(())
    }
}

/// The assembly as a STEP file, `name` the file's own: every part's solid written once, as in
/// `write`, and every use as the next assembly usage occurrence of its product in its parent's,
/// its shape the transformation carrying the child's axes onto its placement among the parent's
/// (a part's representation and an assembly's each list its own axes, and an assembly's the
/// placement of each product it uses).  The products are written the last first, so each
/// child's representation is written before its parent's.  A coloured part's solid is the item
/// of a styled item, its surfaces filled with the colour on both sides (AP214's presentation, the
/// chain CAD readers take a part's colour from), the styles of one colour written once and
/// shared; the styled items are listed by one presentation representation in the shapes' context.
pub fn write_assembly(a: &Assembly,name: &str,tol: f64) -> Result<String,String> {
    a.check()?;
    let mut o = Out {text:String::new(),next:1};
    let context = application(&mut o);
    let pctx = o.add(format!("PRODUCT_CONTEXT('',#{context},'mechanical')"));
    let gctx = geometric(&mut o,tol);
    let mut dctx = None;
    // each product's definition, representation and its own axes there, once written
    let mut written: Vec<(usize,usize,usize)> = vec![(0,0,0);a.products.len()];
    // each coloured part's solid, and its colour
    let mut coloured: Vec<(usize,[f64;3])> = Vec::new();
    for (k,p) in a.products.iter().enumerate().rev() {
        let (definition,shape) = product(&mut o,&p.name.replace('\'',"''"),context,pctx,&mut dctx);
        let origin = o.placement(&Frame::WORLD);
        let rep = match &p.solid {
            Some(b) => {
                let solid = solid(&mut o,b,tol)?;
                if let Some(c) = p.colour { coloured.push((solid,c)); }
                o.add(format!("ADVANCED_BREP_SHAPE_REPRESENTATION('',(#{origin},#{solid}),#{gctx})"))
            }
            None => {
                let uses: Vec<(&Usage,usize)> = a.uses.iter().filter(|u| u.parent == k).map(|u| (u,o.placement(&u.placement))).collect();
                let items: Vec<String> = std::iter::once(origin).chain(uses.iter().map(|&(_,at)| at)).map(|i| format!("#{i}")).collect();
                let rep = o.add(format!("SHAPE_REPRESENTATION('',({}),#{gctx})",items.join(",")));
                for (u,at) in uses {
                    let (child,child_rep,child_origin) = written[u.child];
                    let used = u.name.replace('\'',"''");
                    let moved = o.add(format!("ITEM_DEFINED_TRANSFORMATION('','',#{child_origin},#{at})"));
                    let relation = o.add(format!("( REPRESENTATION_RELATIONSHIP('','',#{child_rep},#{rep}) \
                        REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#{moved}) SHAPE_REPRESENTATION_RELATIONSHIP() )"));
                    let occurrence = o.add(format!("NEXT_ASSEMBLY_USAGE_OCCURRENCE('{used}','{used}','',#{definition},#{child},$)"));
                    let occurrence_shape = o.add(format!("PRODUCT_DEFINITION_SHAPE('','',#{occurrence})"));
                    o.add(format!("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#{relation},#{occurrence_shape})"));
                }
                rep
            }
        };
        o.add(format!("SHAPE_DEFINITION_REPRESENTATION(#{shape},#{rep})"));
        written[k] = (definition,rep,origin);
    }
    // the colours, each one's style assignment written once (by its components' bits)
    let mut styles: std::collections::BTreeMap<[u64;3],usize> = Default::default();
    let mut styled: Vec<String> = Vec::new();
    for (solid,c) in coloured {
        let style = *styles.entry(c.map(f64::to_bits)).or_insert_with(|| {
            let rgb = o.add(format!("COLOUR_RGB('',{},{},{})",real(c[0]),real(c[1]),real(c[2])));
            let fill = o.add(format!("FILL_AREA_STYLE_COLOUR('',#{rgb})"));
            let area = o.add(format!("FILL_AREA_STYLE('',(#{fill}))"));
            let surface = o.add(format!("SURFACE_STYLE_FILL_AREA(#{area})"));
            let side = o.add(format!("SURFACE_SIDE_STYLE('',(#{surface}))"));
            let usage = o.add(format!("SURFACE_STYLE_USAGE(.BOTH.,#{side})"));
            o.add(format!("PRESENTATION_STYLE_ASSIGNMENT((#{usage}))"))
        });
        styled.push(format!("#{}",o.add(format!("STYLED_ITEM('color',(#{style}),#{solid})"))));
    }
    if !styled.is_empty() {
        o.add(format!("MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION('',({}),#{gctx})",styled.join(",")));
    }
    Ok(file(&name.replace('\'',"''"),&o.text))
}

/// The application a file's products are defined in (AP214's), its context's id.
fn application(o: &mut Out) -> usize {
    let context = o.add("APPLICATION_CONTEXT('core data for automotive mechanical design processes')".into());
    o.add(format!("APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#{context})"));
    context
}

/// A product named `name` (escaped already), its definition and that definition's shape: their ids.
/// The definition's context is written beside the first product, and shared by every other.
fn product(o: &mut Out,name: &str,context: usize,pctx: usize,dctx: &mut Option<usize>) -> (usize,usize) {
    let product = o.add(format!("PRODUCT('{name}','{name}','',(#{pctx}))"));
    o.add(format!("PRODUCT_RELATED_PRODUCT_CATEGORY('part',$,(#{product}))"));
    let formation = o.add(format!("PRODUCT_DEFINITION_FORMATION('','',#{product})"));
    let dctx = *dctx.get_or_insert_with(|| o.add(format!("PRODUCT_DEFINITION_CONTEXT('part definition',#{context},'design')")));
    let definition = o.add(format!("PRODUCT_DEFINITION('design','',#{formation},#{dctx})"));
    let shape = o.add(format!("PRODUCT_DEFINITION_SHAPE('','',#{definition})"));
    (definition,shape)
}

/// The units (millimetres, radians), and the context the geometry is written in: its id.
fn geometric(o: &mut Out,tol: f64) -> usize {
    let mm = o.add("( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) )".into());
    let rad = o.add("( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) )".into());
    let sr = o.add("( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() )".into());
    let uncertainty = o.add(format!("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE({}),#{mm},'distance_accuracy_value','confusion accuracy')",real(tol.max(1e-7))));
    o.add(format!("( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{uncertainty})) \
        GLOBAL_UNIT_ASSIGNED_CONTEXT((#{mm},#{rad},#{sr})) REPRESENTATION_CONTEXT('Context #1','3D Context with UNIT and UNCERTAINTY') )"))
}

/// The file around its data section, `name` (escaped already) its own.
fn file(name: &str,data: &str) -> String {
    format!("ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('Solvent model'),'2;1');\nFILE_NAME('{name}','',(''),(''),'Solvent','Solvent','');\n\
        FILE_SCHEMA(('AUTOMOTIVE_DESIGN {{ 1 0 10303 214 1 1 1 1 }}'));\nENDSEC;\nDATA;\n{data}ENDSEC;\nEND-ISO-10303-21;\n")
}

/// The solid's topology and geometry, bottom up, to its manifold solid: its id.
fn solid(o: &mut Out,b: &Brep,tol: f64) -> Result<usize,String> {
    // the topology, bottom up
    // only the vertices a written edge ends at (a pole's degenerate edge is not written)
    let mut wanted = vec![false;b.vertices.len()];
    for e in &b.edges { if let EdgeCurve::Curve(_) = e.curve { for v in e.v { wanted[v as usize] = true; } } }
    let vertices: Vec<usize> = b.vertices.iter().zip(&wanted).map(|(v,&w)| if w {
        let p = o.point(v.p); o.add(format!("VERTEX_POINT('',#{p})"))
    } else { 0 }).collect();
    let surfaces: Vec<usize> = b.faces.iter().map(|f| o.surface(&f.surface)).collect::<Result<_,_>>()?;
    let context2 = o.add("( GEOMETRIC_REPRESENTATION_CONTEXT(2) PARAMETRIC_REPRESENTATION_CONTEXT() REPRESENTATION_CONTEXT('2D SPACE','') )".into());
    let edges: Vec<Option<usize>> = b.edges.iter().enumerate().map(|(ei,e)| -> Result<Option<usize>,String> {
        let EdgeCurve::Curve(c) = &e.curve else { return Ok(None) };
        let curve = match c {
            Curve::Line {p,d} => {
                let (p,d) = (o.point(*p),o.direction(*d));
                let v = o.add(format!("VECTOR('',#{d},1.)"));
                o.add(format!("LINE('',#{p},#{v})"))
            }
            Curve::Circle(f,r) => { let a = o.placement(f); o.add(format!("CIRCLE('',#{a},{})",real(*r))) }
            Curve::Ellipse(f,a,bb) => { let p = o.placement(f); o.add(format!("ELLIPSE('',#{p},{},{})",real(*a),real(*bb))) }
            Curve::BSpline(bs) => {
                let ids: Vec<String> = bs.poles.iter().map(|&p| format!("#{}",o.point(p))).collect();
                spline(o,bs.degree,&ids,&bs.knots,bs.weights.as_deref())
            }
            // a loft's rail: the cubic through it, fitted within `FIT`, its parameter the rail's
            Curve::Iso(bl,u) => {
                let fit = match super::nurbs::fit_curve(&|v| bl.d1([*u,v]).0,3,8,FIT) {
                    Some((fit,err)) if err <= FIT => fit,
                    Some((_,err)) => return Err(format!("a loft's rail is fitted no nearer than {err:.3e} mm, past the file's {FIT:e}")),
                    None => return Err("a loft's rail could not be fitted (it is degenerate)".into()),
                };
                let ids: Vec<String> = fit.poles.iter().map(|&p| format!("#{}",o.point(p))).collect();
                spline(o,fit.degree,&ids,&fit.knots,None)
            }
            // a family's rail or rim likewise, its parameter the rail's or the rim's
            Curve::Rail(..) | Curve::Rim(..) => {
                let [t0,t1] = e.t;
                let fit = match super::nurbs::fit_curve(&|f| c.point(t0+(t1-t0)*f),3,8,FIT) {
                    Some((fit,err)) if err <= FIT => fit,
                    Some((_,err)) => return Err(format!("a face family's {} is fitted no nearer than {err:.3e} mm, past the file's {FIT:e}",c.kind())),
                    None => return Err(format!("a face family's {} could not be fitted (it is degenerate)",c.kind())),
                };
                // its knots the edge's own parameters
                let knots: Vec<f64> = fit.knots.iter().map(|k| t0+(t1-t0)*k).collect();
                let ids: Vec<String> = fit.poles.iter().map(|&p| format!("#{}",o.point(p))).collect();
                spline(o,fit.degree,&ids,&knots,None)
            }
            Curve::Traced(_) => {
                let ts = samples(c,e.t,tol,&|a,z| distance(c.point((a+z)/2.),crate::util::space::lerp(c.point(a),c.point(z),0.5)));
                let ids: Vec<String> = ts.iter().map(|&t| format!("#{}",o.point(c.point(t)))).collect();
                bspline(o,&ids,&ts)
            }
        };
        // the edge's curve in each face that uses it: a seam's two in one face, or one in each
        let uses: Vec<(usize,&super::topo::Coedge)> = b.faces.iter().enumerate()
            .flat_map(|(fi,f)| f.loops.iter().flatten().filter(|c| c.edge as usize == ei).map(move |c| (fi,c))).collect();
        let pcurves: Vec<String> = uses.iter().map(|&(fi,u)| format!("#{}",pcurve(o,b,fi,u,surfaces[fi],context2,tol))).collect();
        let seam = uses.len() == 2 && uses[0].0 == uses[1].0;
        let geometry = o.add(format!("{}('',#{curve},({}),.PCURVE_S1.)",if seam { "SEAM_CURVE" } else { "SURFACE_CURVE" },pcurves.join(",")));
        Ok(Some(o.add(format!("EDGE_CURVE('',#{},#{},#{geometry},.T.)",vertices[e.v[0] as usize],vertices[e.v[1] as usize]))))
    }).collect::<Result<_,_>>()?;
    let mut faces = Vec::new();
    for (fi,f) in b.faces.iter().enumerate() {
        let surface = surfaces[fi];
        let mut bounds = Vec::new();
        for (li,l) in f.loops.iter().enumerate() {
            let mut uses: Vec<String> = Vec::new();
            for c in l {
                let Some(e) = edges[c.edge as usize] else { continue };
                uses.push(format!("#{}",o.add(format!("ORIENTED_EDGE('',*,*,#{e},{})",if c.reversed { ".F." } else { ".T." }))));
            }
            if uses.is_empty() { continue }
            let lp = o.add(format!("EDGE_LOOP('',({}))",uses.join(",")));
            bounds.push(format!("#{}",o.add(format!("{}('',#{lp},.T.)",if li == 0 { "FACE_OUTER_BOUND" } else { "FACE_BOUND" }))));
        }
        faces.push(format!("#{}",o.add(format!("ADVANCED_FACE('',({}),#{surface},{})",bounds.join(","),if f.reversed { ".F." } else { ".T." }))));
    }
    let shell = o.add(format!("CLOSED_SHELL('',({}))",faces.join(",")));
    Ok(o.add(format!("MANIFOLD_SOLID_BREP('',#{shell})")))
}
