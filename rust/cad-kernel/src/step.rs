//! A B-rep written as STEP (ISO 10303-21, AP214's advanced B-rep): the product and units a
//! reader expects (millimetres, radians), a manifold solid on one closed shell for each lump of
//! the body (`lumps`), its voids a `BREP_WITH_VOIDS`'s, its faces on their exact surfaces (a
//! surface swept or turned from a line, a conic or a spline as STEP's own) and its edges on their
//! exact curves; a traced curve written as the cubic
//! within `Fit::bar` of it, in as few pieces as that allows; every edge's curve in each face
//! that uses it (its pcurve, a seam's two) written beside it, so a reader need not project its
//! own, where it can be in at most `3 MOST + 1` points; a pole's degenerate edge left out of its
//! loop, as a reader rebuilds it; and an uncertainty stated no less than what it all holds.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use super::geom::{Curve,Frame,Surface,V};
use std::sync::Arc;
use super::topo::{Brep,EdgeCurve};
use crate::util::space::{add,distance,dot,norm,scale};
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
    /// An id kept for an entity written later (`put`): one others refer to before what it says
    /// is known (the context, whose uncertainty is the most the solids written in it hold).
    fn reserve(&mut self) -> usize { let id = self.next; self.next += 1; id }
    fn put(&mut self,id: usize,entity: String) { let _ = writeln!(self.text,"#{id} = {entity};"); }
    fn point(&mut self,p: V) -> usize { self.add(format!("CARTESIAN_POINT('',({},{},{}))",real(p[0]),real(p[1]),real(p[2]))) }
    fn direction(&mut self,d: V) -> usize { self.add(format!("DIRECTION('',({},{},{}))",real(d[0]),real(d[1]),real(d[2]))) }
    fn placement(&mut self,f: &Frame) -> usize {
        let (o,z,x) = (self.point(f.o),self.direction(f.z),self.direction(f.x));
        self.add(format!("AXIS2_PLACEMENT_3D('',#{o},#{z},#{x})"))
    }
    /// A surface written as itself, and how far the file's stands off it (a swept surface's fitted
    /// curve's miss; else nought).
    fn surface(&mut self,s: &Surface) -> Result<(usize,f64),String> {
        match s {
            Surface::Extrusion(f,c) => {
                let (c,off) = self.swept(c)?;
                let d = self.direction(f.z);
                let v = self.add(format!("VECTOR('',#{d},1.)"));
                return Ok((self.add(format!("SURFACE_OF_LINEAR_EXTRUSION('',#{c},#{v})")),off))
            }
            Surface::Revolution(f,c) => {
                let (c,off) = self.swept(c)?;
                let (o,z) = (self.point(f.o),self.direction(f.z));
                let a = self.add(format!("AXIS1_PLACEMENT('',#{o},#{z})"));
                return Ok((self.add(format!("SURFACE_OF_REVOLUTION('',#{c},#{a})")),off))
            }
            _ => {}
        }
        Ok((match *s {
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
            // a toroidal surface's radii are positive lengths: the kernel's torus of negative major
            // radius, a spindle torus's lemon, is the standard's degenerate torus of major radius
            // `−big` whose inner part it is (`select_outer` false), its parameters `(u + π, π − v)`
            // and its normal turned (`step_uv`, `turned`)
            Surface::Torus(f,big,r) if big < 0. => {
                let a = self.placement(&f);
                self.add(format!("DEGENERATE_TOROIDAL_SURFACE('',#{a},{},{},.F.)",real(-big),real(r)))
            }
            Surface::Torus(f,big,r) => {
                let a = self.placement(&f);
                self.add(format!("TOROIDAL_SURFACE('',#{a},{},{})",real(big),real(r)))
            }
            // (written above, or as its face's net: `solid` writes a face here only where
            // `face_net` gives none, and it gives one for each of these)
            Surface::Extrusion(..) | Surface::Revolution(..) | Surface::Blend(..) | Surface::BSpline(..) | Surface::Family(..)
                | Surface::Affine(..) => return Err(format!("a {} written as no net",s.kind())),
            // a sheet's offset read from a file (`OFFSET_SURFACE` of a net), written so again:
            // `S + d N`, `N` the unit `S_u × S_v`, as the standard's and the reader's
            Surface::Offset(_,ref n,d) => {
                let basis = self.net(n);
                self.add(format!("OFFSET_SURFACE('',#{basis},{},.F.)",real(d)))
            }
        },0.))
    }
    /// A B-spline surface, rational where it has weights: the complex entity, its parts in the
    /// alphabetical order the standard asks.
    fn net(&mut self,net: &super::nurbs::Net) -> usize {
        let rows: Vec<String> = net.poles.iter().map(|col| {
            let ids: Vec<String> = col.iter().map(|&p| format!("#{}",self.point(p))).collect();
            format!("({})",ids.join(","))
        }).collect();
        let (uk,vk) = (super::nurbs::distinct(&net.uknots),super::nurbs::distinct(&net.vknots));
        let list = |k: &[(f64,usize)],f: &dyn Fn(&(f64,usize)) -> String| k.iter().map(f).collect::<Vec<_>>().join(",");
        if let Some(w) = &net.weights {
            let w: Vec<String> = w.iter().map(|r| format!("({})",r.iter().map(|&w| real(w)).collect::<Vec<_>>().join(","))).collect();
            return self.add(format!("( BOUNDED_SURFACE() B_SPLINE_SURFACE({},{},({}),.UNSPECIFIED.,.F.,.F.,.F.) \
                B_SPLINE_SURFACE_WITH_KNOTS(({}),({}),({}),({}),.UNSPECIFIED.) GEOMETRIC_REPRESENTATION_ITEM() \
                RATIONAL_B_SPLINE_SURFACE(({})) REPRESENTATION_ITEM('') SURFACE() )",
                net.du,net.dv,rows.join(","),list(&uk,&|k| k.1.to_string()),list(&vk,&|k| k.1.to_string()),
                list(&uk,&|k| real(k.0)),list(&vk,&|k| real(k.0)),w.join(",")))
        }
        self.add(format!("B_SPLINE_SURFACE_WITH_KNOTS('',{},{},({}),.UNSPECIFIED.,.F.,.F.,.F.,({}),({}),({}),({}),.UNSPECIFIED.)",
            net.du,net.dv,rows.join(","),list(&uk,&|k| k.1.to_string()),list(&vk,&|k| k.1.to_string()),
            list(&uk,&|k| real(k.0)),list(&vk,&|k| real(k.0))))
    }
    /// **The curve a swept surface sweeps, written as itself** (#993 stage 2), and how far the
    /// file's stands off it: a B-spline (as a profile gives one), and a line, a circle or an
    /// ellipse (as a file read gives one: `SURFACE_OF_LINEAR_EXTRUSION` and
    /// `SURFACE_OF_REVOLUTION` take any curve, and sweep it over its own parameter as the kernel's
    /// `Extrusion` and `Revolution` do, so the face's parameters are the file's and its pcurves are
    /// written in them unchanged); refused for any other curve, which no reading or construction
    /// sweeps.
    fn swept(&mut self,c: &Curve) -> Result<(usize,f64),String> {
        // a curve read exactly in a frame (a turned contact's meridian, #780 C3; a slid torus's
        // contour, #805), which no entity of the standard writes: the cubic through it over its
        // span, fitted within `FIT`, its knots its own parameters
        if let Some(span) = c.exact_span() { return fitted(self,c,span,whose(c)) }
        Ok((match c {
            Curve::BSpline(b) => {
                let ids: Vec<String> = b.poles.iter().map(|&p| format!("#{}",self.point(p))).collect();
                spline(self,b.degree,&ids,&b.knots,b.weights.as_deref())
            }
            // (its vector the line's own `d`, a unit direction and its length, so its parameter is
            // the line's: `p + t d`)
            Curve::Line {p,d} => {
                let l = norm(*d);
                if !(l > 0. && l.is_finite()) { return Err("a swept surface's line has no direction".into()) }
                let (p,d) = (self.point(*p),self.direction(scale(*d,1./l)));
                let v = self.add(format!("VECTOR('',#{d},{})",real(l)));
                self.add(format!("LINE('',#{p},#{v})"))
            }
            Curve::Circle(f,r) => { let a = self.placement(f); self.add(format!("CIRCLE('',#{a},{})",real(*r))) }
            Curve::Ellipse(f,a,b) => { let p = self.placement(f); self.add(format!("ELLIPSE('',#{p},{},{})",real(*a),real(*b))) }
            other => return Err(format!("a surface swept or turned from a {} curve has no entity of the standard to write it",other.kind())),
        },0.))
    }
}

/// Whose curve `c` is, as a fitted curve's refusal names it.
fn whose(c: &Curve) -> &'static str {
    match c { Curve::SheetCut(..) => "a screwed sheet's",Curve::Turned(..) => "a turned contact's",Curve::Slid(..) => "a slid torus's",
        Curve::Trochoid(..) => "a point's path's",
        _ => "a face family's" }
}

/// **`c` over `[t0, t1]` as the cubic through it**, fitted within `FIT` (a curve no entity of the
/// standard writes: a family's rail or rim, an affine image, a screwed sheet's cut, a turned
/// contact's meridian), its knots the curve's own parameters, and how far it was measured off;
/// refused naming `whose` it is.
fn fitted(o: &mut Out,c: &Curve,[t0,t1]: [f64;2],whose: &str) -> Result<(usize,f64),String> {
    let (fit,miss) = match super::nurbs::fit_curve(&|f| c.point(t0+(t1-t0)*f),3,8,FIT) {
        Some((mut fit,err)) if err <= FIT => { for k in &mut fit.knots { *k = t0+(t1-t0)**k; } (fit,err) }
        // (a curve that turns at its breaks — a family's rail at its stations' joints — fitted
        // stretch by stretch between them, the stretches joined in value)
        Some((_,err)) => match fitted_stretches(c,[t0,t1]) {
            Some((fit,miss)) if miss <= FIT => (fit,miss),
            _ => return Err(format!("{whose} {} is fitted no nearer than {err:.3e} mm, past the file's {FIT:e}",c.kind())),
        },
        None => return Err(format!("{whose} {} could not be fitted (it is degenerate)",c.kind())),
    };
    let ids: Vec<String> = fit.poles.iter().map(|&p| format!("#{}",o.point(p))).collect();
    Ok((spline(o,fit.degree,&ids,&fit.knots,None),miss))
}

/// `c` over `[t0, t1]` as cubics fitted between its breaks (`Curve::breaks`), joined where they
/// meet with a triple knot (each stretch's end its point there), its knots the curve's own
/// parameters; and the most any stretch misses by. `None` where it has no break inside.
fn fitted_stretches(c: &Curve,[t0,t1]: [f64;2]) -> Option<(super::nurbs::BSpline,f64)> {
    let mut stops: Vec<f64> = c.breaks([t0,t1]).into_iter().filter(|&k| k > t0 && k < t1).collect();
    if stops.is_empty() { return None }
    stops.sort_by(f64::total_cmp);
    stops.dedup();
    stops.insert(0,t0);
    stops.push(t1);
    let mut joined: Option<super::nurbs::BSpline> = None;
    let mut most = 0f64;
    for w in stops.windows(2) {
        let (a,z) = (w[0],w[1]);
        let (mut fit,err) = super::nurbs::fit_curve(&|f| c.point(a+(z-a)*f),3,4,FIT)?;
        most = most.max(err);
        for k in &mut fit.knots { *k = a+(z-a)**k; }
        joined = Some(match joined {
            None => fit,
            Some(mut all) => {
                if all.degree != fit.degree { return None }
                let p = all.degree;
                all.knots.pop();
                all.knots.extend_from_slice(&fit.knots[p+1..]);
                all.poles.extend_from_slice(&fit.poles[1..]);
                all
            }
        });
    }
    joined.map(|j| (j,most))
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
/// `None` past `MOST` points: a polyline the file would carry in millions of points (a pcurve on
/// a toolpath's net within 1e-7 mm, 2.5 million) is not written.
fn samples(c: &Curve,t: [f64;2],tol: f64,off: &dyn Fn(f64,f64) -> f64) -> Option<Vec<f64>> {
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
            if out.len()+stack.len() > MOST+1 { return None }
        }
    }
    Some(out)
}

/// **The most pieces a curve the file fits is written in** (#993 stage 1): a cubic's (`hermite_fit`,
/// three poles a piece) or a polyline's (`samples`, a point a piece). So no edge's curve, in space
/// or in a face's parameters, is written in more than `3 MOST + 1` points: past it a pcurve is
/// left out (a reader puts the edge on the face itself, as it does where a file writes none) and a
/// traced curve's polyline is written within the bar its `MOST` pieces hold, which the file then
/// states (`write_stating`).
pub const MOST: usize = 4096;

/// How near its exact surface or curve a loft's face or rail is written (millimetres): the file
/// carries a B-spline fitted within this, its parameters the exact geometry's.
pub const FIT: f64 = 1e-6;

/// A loft face's B-spline, fitted within `FIT`: cubic across the section, and cubic along the guide
/// where it turns or linear where it runs straight (a blend carried along a line is linear in `v`).
/// A screw a cubic net cannot follow within `FIT` at the fit's cap (a section carried round many
/// turns, #243 L6) is fitted quintic along it, as a face family's wall is. Refused where no net
/// the fit may grow to comes within `FIT`, rather than written further off. With how far the net
/// was measured off the face.
pub fn blend_net(b: &super::geom::Blend) -> Result<(super::nurbs::Net,f64),String> {
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
        Some((net,err)) if err <= FIT => Ok((net,err)),
        Some((_,err)) => Err(format!("a loft face is fitted no nearer than {err:.3e} mm to its surface, past the file's {FIT:e}")),
        None => Err("a loft face could not be fitted (its surface is degenerate)".into()),
    }
}

/// A face family's wall as the file writes it (#243 L2): the net through it fitted within `FIT` as
/// a loft face is — cubic along its edge, quintic along the family, whose rails may wind many
/// times (a cubic through a twisted wall's 512 rows of a three-and-a-half-turn auger misses by
/// 1e-6) — its `v` the family's running number over its interval. With how far the net was
/// measured off the wall.
pub fn family_net(w: &super::family::Sheet) -> Result<(super::nurbs::Net,f64),String> {
    let [a,b] = w.span();
    let pieces = w.stations.spans.len().clamp(4,64);
    let err = match super::nurbs::fit_net(&|u,v| w.d1([u,a+(b-a)*v]).0,3,5,8,pieces,FIT) {
        Some((mut net,err)) if err <= FIT => {
            for k in &mut net.vknots { *k = a+(b-a)**k; }
            return Ok((net,err))
        }
        Some((_,err)) => err,
        None => return Err("a face family's wall could not be fitted (its surface is degenerate)".into()),
    };
    // (a wall whose stations' polynomials meet at an angle, its piece by piece)
    if let Some(fit) = family_pieces(w).filter(|f| f.1 <= FIT) { return Ok(fit) }
    Err(format!("a face family's wall is fitted no nearer than {err:.3e} mm to its surface, past the file's {FIT:e}"))
}

/// **A face family's wall fitted piece by piece** of its stations (#993 stage 1): each piece's
/// numbers are a polynomial of the running number through its six stations, and two pieces meet
/// only in value, so where the wall turns at a joint no net smooth across it comes within `FIT`
/// (`lofts/pipe.sv`'s tube along a path of straight runs and arcs, 1.7e-2 mm at the fit's cap of
/// 512 rows). Each piece is fitted over its own stretch on one grid along the edge, and the nets
/// joined at its ends with a knot of the degree's multiplicity (the joint's row, the same points
/// interpolated the same way, is each net's end), the grid doubled each way it misses by more than
/// `FIT` (along the edge, or along the family: a tube's round wall needs its columns, never its
/// rows, which doubled alike wrote 7233 rows of 129 for a ball's ring along a spline ellipse, 381
/// MB) until every piece is within it or past 1024 columns and 64 rows a piece. With the most any
/// piece misses by.
fn family_pieces(w: &super::family::Sheet) -> Option<(super::nurbs::Net,f64)> {
    use super::nurbs::{fit_net_each,Net};
    let spans = &w.stations.spans;
    let (mut nu,mut nv) = (8,5);
    loop {
        let mut joined: Option<Net> = None;
        let (mut most_u,mut most_v) = (0f64,0f64);
        for (j,sp) in spans.iter().enumerate() {
            // (each piece from where the last one ended, so the joint is one number)
            let a = if j == 0 { sp[0] } else { spans[j-1][1] };
            let (mut net,eu,ev) = fit_net_each(&|u,v| w.d1([u,a+(sp[1]-a)*v]).0,3,5,nu,nv,f64::INFINITY)?;
            (most_u,most_v) = (most_u.max(eu),most_v.max(ev));
            for k in &mut net.vknots { *k = a+(sp[1]-a)**k; }
            joined = Some(match joined {
                None => net,
                Some(mut all) => {
                    if all.du != net.du || all.dv != net.dv || all.uknots != net.uknots { return None }
                    let p = all.dv;
                    all.vknots.pop();
                    all.vknots.extend_from_slice(&net.vknots[p+1..]);
                    for (col,more) in all.poles.iter_mut().zip(&net.poles) { col.extend_from_slice(&more[1..]); }
                    all
                }
            });
        }
        let most = most_u.max(most_v);
        let (grow_u,grow_v) = (most_u > FIT && nu < 1024,most_v > FIT && nv < 64);
        if most <= FIT || !grow_u && !grow_v { return joined.map(|n| (n,most)) }
        if grow_u { nu = (2*nu).min(1024); }
        if grow_v { nv = (2*nv).min(64); }
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
/// numbers — a cone opening against its axis written about the reversed one, an affine image by
/// its preimage's and its map's (what two faces are one surface by, `pattern`).
pub fn written(s: &Surface) -> (Frame,Vec<f64>) {
    match s.clone() {
        Surface::Plane(f) => (f,vec![]),
        Surface::Cylinder(f,r) => (f,vec![r]),
        Surface::Cone(f,r,a) => (if a < 0. { Frame {o:f.o,x:f.x,y:scale(f.y,-1.),z:scale(f.z,-1.)} } else { f },vec![r,a.abs()]),
        Surface::Sphere(f,r) => (f,vec![r]),
        Surface::Torus(f,big,r) => (f,vec![big,r]),
        Surface::Extrusion(f,_) | Surface::Revolution(f,_) | Surface::Blend(f,_) | Surface::BSpline(f,_) | Surface::Family(f,_) => (f,vec![]),
        // its sheet's frame, and how far off it
        Surface::Offset(f,_,d) => (f,vec![d]),
        // its preimage's, and the map's numbers: one image is another only under one map
        Surface::Affine(m) => {
            let (g,mut numbers) = written(&m.of);
            numbers.extend(g.o.iter().chain(&g.z).chain(&g.x).chain(m.map.m.iter().flatten()).chain(&m.map.t));
            (m.frame,numbers)
        }
    }
}

/// **A face's parameters as STEP's surface reads them** (ISO 10303-42): the same, but on a cone.
/// The kernel's cone runs `v` along its generator (OCCT's `Geom_ConicalSurface`); the standard's
/// `conical_surface` runs it along the axis, `C + (R + v tan α)(cos u x + sin u y) + v z`, so the
/// file's `v` is the kernel's times `cos α` — and on a cone written about its reversed axis both
/// run the other way. Each is a scaling, so a line, a spline's poles and a fitted cubic's are
/// carried exactly. (An affine image's parameters are its net's, `OnNet::uv`.)
/// A lemon (a torus of negative major radius `−B`) is written as the degenerate torus of major
/// radius `B`, whose point at `(u + π, π − v)` is the kernel's at `(u, v)`.
fn step_uv(s: &Surface,uv: super::geom::Uv) -> [f64;2] {
    use std::f64::consts::PI;
    match *s {
        Surface::Cone(_,_,a) => { let c = a.dcos(); if a < 0. { [-uv[0],-uv[1]*c] } else { [uv[0],uv[1]*c] } }
        Surface::Torus(_,big,_) if big < 0. => [uv[0]+PI,PI-uv[1]],
        _ => uv,
    }
}

/// **Whether the surface the file writes for `s` runs against it**: its normal `S_u × S_v` the
/// kernel's turned, so a face on it is written with its sense turned — a lemon's degenerate torus,
/// whose `v` runs the other way (`step_uv`).
pub fn turned(s: &Surface) -> bool { matches!(*s,Surface::Torus(_,big,_) if big < 0.) }

/// **A face STEP writes as a B-spline net** (`face_net`): a sheet as its own, a loft's or a family
/// wall's as the net fitted to it, and an affine image (STEP has no image of a surface) as its
/// preimage's exact net over the parameters the face reaches, its poles mapped. How the face's
/// parameters land on the net's: an angle at `nurbs::arc_parameter` over the turn it was built on,
/// anything else as it is (`plain` where both are).
pub struct OnNet { pub net: Arc<super::nurbs::Net>,arcs: [Option<[f64;2]>;2],pub surface: Surface,
    /// How far the net stands off the face's own surface (millimetres): its fit's measured miss
    /// for a loft's or a family wall's, nought for an exact one.
    pub fit: f64 }

impl OnNet {
    /// The net's parameters of the face's `uv`, and their rates along the face's own.
    pub fn uv(&self,uv: super::geom::Uv) -> ([f64;2],[f64;2]) {
        let at = |k: usize| match self.arcs[k] { Some(span) => super::nurbs::arc_parameter(span,uv[k]),None => (uv[k],1.) };
        let ((s,ds),(t,dt)) = (at(0),at(1));
        ([s,t],[ds,dt])
    }
    /// Whether the net's parameters are the face's own.
    pub fn plain(&self) -> bool { self.arcs == [None,None] }
}

/// Face `fi`'s net where STEP writes it as one (`OnNet`), else none (an analytic surface or a swept
/// curve, written as itself); refused where a loft's or a family's net cannot be fitted, or an
/// affine image's preimage has no net (a revolution or an extrusion of anything but a spline).
pub fn face_net(b: &Brep,fi: usize) -> Option<Result<OnNet,String>> {
    use super::nurbs::{arc,Net};
    use std::f64::consts::FRAC_PI_2;
    let f = &b.faces[fi];
    let net = |n: Result<(Net,f64),String>,arcs: [Option<[f64;2]>;2],frame: Frame| n.map(|(n,fit)| {
        let net = Arc::new(n);
        OnNet {surface:Surface::BSpline(frame,net.clone()),net,arcs,fit}
    });
    let m = match &f.surface {
        // **a sheet's face written on its own stretch of the net** (#1013): the net cut to the face's
        // parameter box grown by a sixteenth (`Net::segment`, which keeps the parameters, so its
        // pcurves are the face's still): a surface of its own, a fraction of the net. Every face on
        // one sheet wrote the whole net: a swept sheet's is some 55 MB, and the crossed tori's
        // union was a 2.4 GB file no reader could measure. Not one surface shared by the faces on
        // a net: an edge between two of them on two layers of it (a sheet's double curve) has a
        // pcurve on each, and only their surfaces tell a reader which is whose
        Surface::BSpline(g,n) => {
            let [u,v] = super::boolean::parameters(b,fi,true);
            let [du,dv] = n.domain();
            let within = |r: [f64;2],d: [f64;2]| r[0] > d[0] || r[1] < d[1];
            if !(u[1] > u[0] && v[1] > v[0]) || !(within(u,du) || within(v,dv)) {
                return Some(Ok(OnNet {net:n.clone(),arcs:[None,None],surface:f.surface.clone(),fit:0.}))
            }
            let net = Arc::new(n.segment(u,v));
            return Some(Ok(OnNet {surface:Surface::BSpline(*g,net.clone()),net,arcs:[None,None],fit:0.}))
        }
        Surface::Blend(g,bl) => return Some(net(blend_net(bl),[None,None],*g)),
        Surface::Family(g,w) => return Some(net(family_net(w),[None,None],*g)),
        Surface::Affine(m) => m,
        _ => return None,
    };
    let [ur,vr] = super::boolean::parameters(b,fi,true);
    let mapped = |n: Net| super::affine::net_mapped(&n,&|p| m.map.point(p));
    let stretch = m.map.stretches().into_iter().fold(0.,f64::max);
    // each of a whole turn's poles `c` (`nurbs::arc` over the face's `u`) by each of a meridian's
    // `(ρ, z)` in the preimage's frame `g`, weights the products, poles carried by the map
    let turned = |g: &Frame,meridian: &[[f64;2]],mw: &[f64],vknots: Vec<f64>,dv: usize| -> Net {
        let round = arc(&Frame::WORLD,1.,ur);
        let wu = round.weights.as_ref().expect("an arc is rational");
        let poles = round.poles.iter().map(|c| meridian.iter().map(|&[rho,z]| m.map.point(g.at([rho*c[0],rho*c[1],z]))).collect()).collect();
        let weights = wu.iter().map(|&a| mw.iter().map(|&b| a*b).collect()).collect();
        Net {du:2,dv,uknots:round.knots.clone(),vknots,poles,weights:Some(weights)}
    };
    let built = match m.of {
        // a tube's circle of radius `r` about `big` out from the axis, over the face's `v` (a
        // sphere's, about none, held to its poles)
        Surface::Sphere(..) | Surface::Torus(..) => {
            let (g,big,r,span) = match m.of {
                Surface::Sphere(g,r) => (g,0.,r,[vr[0].max(-FRAC_PI_2),vr[1].min(FRAC_PI_2)]),
                Surface::Torus(g,big,r) => (g,big,r,vr),
                _ => unreachable!(),
            };
            let a = arc(&Frame::WORLD,1.,span);
            let meridian: Vec<[f64;2]> = a.poles.iter().map(|p| [big+r*p[0],r*p[1]]).collect();
            Ok((turned(&g,&meridian,a.weights.as_ref().expect("an arc is rational"),a.knots.clone(),2),[Some(ur),Some(span)],0.))
        }
        // a cone's generator over the face's `v` (a cylinder's, of no half-angle)
        Surface::Cylinder(..) | Surface::Cone(..) => {
            let (g,r,h) = match m.of { Surface::Cylinder(g,r) => (g,r,0.),Surface::Cone(g,r,h) => (g,r,h),_ => unreachable!() };
            let (s,c) = h.dsin_cos();
            let ends = vr.map(|v| [r+v*s,v*c]);
            Ok((turned(&g,&ends,&[1.,1.],vec![vr[0],vr[0],vr[1],vr[1]],1),[Some(ur),None],0.))
        }
        Surface::Revolution(g,ref c) => match &**c {
            Curve::BSpline(bs) => {
                let q: Vec<[f64;2]> = bs.poles.iter().map(|&p| { let l = g.local(p); [l[0],l[2]] }).collect();
                let w = bs.weights.clone().unwrap_or_else(|| vec![1.;q.len()]);
                Ok((turned(&g,&q,&w,bs.knots.clone(),bs.degree),[Some(ur),None],0.))
            }
            ref other => Err(format!("an affine image of a revolution of a {} has no net to write",other.kind())),
        },
        Surface::Extrusion(g,ref c) => match &**c {
            Curve::BSpline(bs) => {
                let poles = bs.poles.iter().map(|&p| vr.iter().map(|&v| m.map.point(add(p,scale(g.z,v)))).collect()).collect();
                let weights = bs.weights.as_ref().map(|w| w.iter().map(|&w| vec![w,w]).collect());
                Ok((Net {du:bs.degree,dv:1,uknots:bs.knots.clone(),vknots:vec![vr[0],vr[0],vr[1],vr[1]],poles,weights},[None,None],0.))
            }
            ref other => Err(format!("an affine image of an extrusion of a {} has no net to write",other.kind())),
        },
        // (a fitted net's miss carried by the map's greatest stretch)
        Surface::Blend(_,ref bl) => blend_net(bl).map(|(n,e)| (mapped(n),[None,None],e*stretch)),
        Surface::Family(_,ref w) => family_net(w).map(|(n,e)| (mapped(n),[None,None],e*stretch)),
        ref other => Err(format!("an affine image of a {} has no net to write",other.kind())),
    };
    Some(built.and_then(|(n,arcs,fit)| net(Ok((n,fit)),arcs,m.frame)))
}

/// The least uncertainty a file states (millimetres): a tolerance asked finer is written as this.
const LEAST_UNCERTAINTY: f64 = 1e-7;

/// **How near the exact geometry a curve the file fits is held** (#961): a traced edge's cubic and
/// every cubic pcurve. Two things the file says are kept apart here: the uncertainty it states
/// (`write`'s `tol`, what a reader may take two points to be one within, and the bar of a polyline
/// written where no cubic closes in), and how near its fitted curves stand to the exact ones.
/// Where no tolerance was asked, the second is no licence of the first's size: the curves are held
/// to `EXACT`, so a reader measures the solid the kernel built (only a polyline written where no
/// cubic closes in, the last resort, stands as far off as the stated uncertainty).
#[derive(Clone,Copy,Debug,PartialEq)]
pub enum Fit {
    /// No tolerance stated: every fitted curve within `EXACT` of the exact one, on each face it
    /// trims, so a reader measuring the faces reads the solid's measures to the kernel's own
    /// (brep-bench's half torus, a ball bored off its centre, the saddle drums, the grazing
    /// cylinders: within some 1e-9 of their closed forms, as before #961).
    Exact,
    /// A tolerance the author stated, as the bar its fitted curves are held to (millimetres, no
    /// finer than `EXACT`): each face the file trims is trimmed within it of the exact meeting,
    /// measured on that face, and the file is the smaller for it (`Fit::stated`).
    Within(f64),
}

/// The bar a curve is fitted to where no tolerance is stated (millimetres): a thousandth of `FIT`,
/// the bar a loft's face or rail is written within. A Hermite cubic's error keeps its sign along a
/// piece, so a reader's measures move by about the bar over the solid's size — over its thickness,
/// for a thin one: held to a tenth of `FIT`, the half torus read 1.5e-8 off by the kernel's own
/// measure; to a hundredth, brep-bench's grazing cylinders (a cushion 0.01 thick) 2.6e-8, where
/// cutting a piece at every marched point read them 6e-9.
pub const EXACT: f64 = FIT/1000.;

impl Fit {
    /// The bar the fitted curves are held to.
    pub fn bar(self) -> f64 { match self { Fit::Exact => EXACT,Fit::Within(b) => b.max(EXACT) } }
    /// The fit an export at a stated tolerance licenses, its file stating `uncertainty`: half of
    /// it, so the curve and each of its pcurves stand within the uncertainty of one another.
    pub fn stated(uncertainty: f64) -> Fit { Fit::Within(uncertainty.max(LEAST_UNCERTAINTY)/2.) }
    /// The bar of a polyline written where no cubic closes in: the stated uncertainty where no
    /// tolerance was asked (the polyline is the last resort), else the stated fit's.
    fn polyline(self,tol: f64) -> f64 { match self { Fit::Exact => tol.max(LEAST_UNCERTAINTY),Fit::Within(_) => self.bar() } }
}

/// The interior points a piece of a fitted cubic is measured at, as sixteenths of it: the middle
/// among them, where a Hermite cubic's error `s²(1 − s)²` is greatest, and enough beside it that
/// an error of another shape (a rate that steps inside the piece) is not read between them.
const MEASURED: usize = 16;

/// **A curve as a cubic B-spline within `bar` of it** over `[t0, t1]` of its parameter: piecewise
/// Hermite through its points and rates, each piece kept where its `MEASURED` points come within
/// `bar` of the curve's at the same parameter (`off`, a distance in space). The pieces are
/// marched from `t0`, stopping at every break (`breaks`: where the curve's derivatives may jump,
/// a spline's knots), each as long as the bar allows: a cubic's miss goes as the fourth power of
/// its length, so a piece that misses is shortened, and the next after one that holds lengthened,
/// by the fourth root of the bar over its miss (a tenth short, at most halved or doubled), the way
/// to the next stop shared evenly among the pieces it needs. A rate is the curve's own (`rate`)
/// except at a break or where it is not defined (a cone's apex in its parameters), where it is
/// the one-sided difference toward the piece; so two pieces meeting where the curve is smooth
/// share their rate there, and the joint is a double knot (the cubic `C¹`, its joint's pole
/// dropped), a triple one at a break. The pieces follow the bar, not the curve's own sampling: a
/// trace marched in a thousand points is written in the few pieces the bar needs. The ends are the
/// curve's own, exactly. `None` where a piece shortened to `2⁻³⁰` of the stretch still misses, past
/// `MOST` pieces, or where a rate cannot be read. With the poles and knots, the most any piece was
/// measured to miss by.
fn hermite_fit<const N: usize>(at: &dyn Fn(f64) -> [f64;N],rate: &dyn Fn(f64) -> [f64;N],off: &dyn Fn([f64;N],f64) -> f64,
    [t0,t1]: [f64;2],breaks: &[f64],bar: f64) -> Option<(Vec<[f64;N]>,Vec<f64>,f64)> {
    let finite = |d: &[f64;N]| d.iter().all(|x| x.is_finite()) && d.iter().any(|&x| x != 0.);
    // the rate at `x` as the piece running toward `y` sees it
    let rate = |x: f64,y: f64| -> Option<[f64;N]> {
        if !breaks.contains(&x) { let d = rate(x); if finite(&d) { return Some(d) } }
        // one-sided, second order: (−3 C(x) + 4 C(x + h) − C(x + 2h)) / 2h, `h` a ten-thousandth of
        // the piece
        let h = 1e-4*(y-x);
        let (p,q,r) = (at(x),at(x+h),at(x+2.*h));
        let d = std::array::from_fn(|k| (-3.*p[k]+4.*q[k]-r[k])/(2.*h));
        finite(&d).then_some(d)
    };
    let hermite = |a: f64,b: f64| -> Option<([[f64;N];4],[[f64;N];2])> {
        let (p,q) = (at(a),at(b));
        let (dp,dq) = (rate(a,b)?,rate(b,a)?);
        let h = (b-a)/3.;
        Some(([p,std::array::from_fn(|k| p[k]+h*dp[k]),std::array::from_fn(|k| q[k]-h*dq[k]),q],[dp,dq]))
    };
    let bezier = |c: &[[f64;N];4],s: f64| -> [f64;N] {
        let w = [(1.-s).dpowi(3),3.*s*(1.-s)*(1.-s),3.*s*s*(1.-s),s.dpowi(3)];
        std::array::from_fn(|k| (0..4).map(|j| w[j]*c[j][k]).sum())
    };
    // each piece: its end, its poles, and its rates at its ends — marched from `t0`, each piece
    // as long as the last one's miss says the bar allows (a cubic's misses as the fourth power
    // of its length), shortened the same way where it misses, and the stretch to the next stop
    // shared evenly among the pieces it needs
    let mut stops: Vec<f64> = breaks.iter().copied().filter(|&k| k > t0 && k < t1).collect();
    stops.sort_by(f64::total_cmp);
    stops.push(t1);
    let least = (t1-t0)*0.5f64.powi(30);
    let mut pieces: Vec<(f64,[[f64;N];4],[[f64;N];2])> = Vec::new();
    let mut most = 0f64;
    let (mut a,mut length,mut stop) = (t0,t1-t0,0);
    while a < t1 {
        while stops[stop] <= a { stop += 1; }
        let left = stops[stop]-a;
        let b = if length >= left { stops[stop] } else { a+left/(left/length).ceil() };
        let (c,d) = hermite(a,b)?;
        let miss = (1..MEASURED).map(|k| { let s = k as f64/MEASURED as f64; off(bezier(&c,s),a+(b-a)*s) }).fold(0.,f64::max);
        // the length the bar allows, by the fourth power, a tenth short of it
        let allowed = |most: f64| if miss > 0. { (0.9*(bar/miss).powf(0.25)).min(most) } else { most };
        if miss <= bar {
            pieces.push((b,c,d));
            most = most.max(miss);
            if pieces.len() > MOST { return None }
            length = (b-a)*allowed(2.);
            a = b;
        } else {
            length = (b-a)*allowed(0.5).max(0.1);
            if !(length >= least) { return None }
        }
    }
    let mut poles = vec![pieces.first()?.1[0]];
    let mut knots = vec![t0;4];
    for (i,(b,c,_)) in pieces.iter().enumerate() {
        // smooth where this piece starts at the rate the last one ended at: its joint pole dropped,
        // the joint's knot made double
        if i > 0 && pieces[i-1].2[1] == pieces[i].2[0] { poles.pop(); knots.pop(); }
        poles.extend_from_slice(&c[1..]);
        knots.extend(std::iter::repeat_n(*b,if i+1 == pieces.len() { 4 } else { 3 }));
    }
    Some((poles,knots,most))
}

/// **The parameter the file writes an edge's curve over**: the curve's own, except a traced
/// curve's. That counts its points, and its speed steps at each: the stretch between two points
/// is the parabola through them and a neighbour pulled onto both surfaces, the neighbour behind
/// on one side of a point and ahead on the other, so the rate at a point differs either side by
/// some `(κh)²` of the speed (the march's step `h` against the curvature) — the curve is smooth,
/// its parameter is not, and no cubic over several points follows that parameter within a bar
/// finer than that step times a chord: every piece would end at a point. The file writes a
/// traced curve over `s` instead, which runs over each stretch `[i, i + 1]` as the cubic in `t`
/// rising by the stretch's length at the curve's own speed at each end, read on the stretch's
/// side (`speeds`): so `dC/ds` is the unit tangent at every point from either side, and the
/// curve in `s` is `C¹` across its points, its speed one but for the speed's departure from a
/// quadratic within a stretch. (Rising by the chord instead, the speed was off by the chord's
/// shortfall, and each piece drifted along the curve by that times its length.) A closed trace's
/// `s` runs on round its period. Every curve of the edge, in space and in each face, is written
/// over the same `s`.
struct Along { chords: Option<Chords> }

/// A traced curve's `s` (`Along`): where it stands at each point, and each stretch's speed in `t`
/// at its start and its end.
struct Chords { at: Vec<f64>,speeds: Vec<[f64;2]>,closed: bool }

impl Along {
    fn of(c: &Curve) -> Along {
        let Curve::Traced(t) = c else { return Along {chords:None} };
        let m = t.pts.len()-1;
        // the speed at a point, from the side `toward` (one-sided, second order, a ten-thousandth
        // of a stretch)
        let speed = |x: f64,toward: f64| {
            let h = 1e-4*(toward-x);
            let (p,q,r) = (c.point(x),c.point(x+h),c.point(x+2.*h));
            norm(std::array::from_fn(|k| (-3.*p[k]+4.*q[k]-r[k])/(2.*h)))
        };
        let mut at = vec![0.];
        let mut speeds = Vec::with_capacity(m);
        for i in 0..m {
            // the stretch's length, by ten-point Gauss–Legendre (its nodes inside it, where the
            // rate is the stretch's own)
            let (a,z) = (i as f64,i as f64+1.);
            let length = super::props::gauss(a,z,1,&mut |x| norm(c.d1(x).1));
            let v = [speed(a,z),speed(z,a)];
            // (a stretch of no length, or a speed not read, or one the cubic would turn back
            // under: nothing to write it over)
            if !(length > 0. && v.iter().all(|&v| v.is_finite() && v > 0. && v < 3.*length)) { return Along {chords:None} }
            at.push(at[i]+length);
            speeds.push(v);
        }
        Along {chords:Some(Chords {at,speeds,closed:t.closed})}
    }
    /// `s` at the curve's parameter `t`.
    fn s(&self,t: f64) -> f64 {
        let Some(c) = &self.chords else { return t };
        let m = c.speeds.len();
        let turns = if c.closed { (t/m as f64).floor() } else { 0. };
        let i = ((t-turns*m as f64).floor().max(0.) as usize).min(m-1);
        let f = t-turns*m as f64-i as f64;
        turns*c.at[m]+c.at[i]+c.rise(i,f).0
    }
    /// The curve's parameter at `s`, and its rate `dt/ds` there.
    fn t(&self,x: f64) -> (f64,f64) {
        let Some(c) = &self.chords else { return (x,1.) };
        let m = c.speeds.len();
        let turns = if c.closed { (x/c.at[m]).floor() } else { 0. };
        let r = x-turns*c.at[m];
        let i = c.at.partition_point(|&k| k <= r).clamp(1,m)-1;
        let goal = r-c.at[i];
        // Newton on the stretch's cubic, which rises throughout, kept within the stretch's ends
        // by halving the bracket where a step leaves it
        let (mut lo,mut hi) = (0f64,1f64);
        let mut f = (goal/(c.at[i+1]-c.at[i])).clamp(0.,1.);
        for _ in 0..60 {
            let (h,dh) = c.rise(i,f);
            if h < goal { lo = f } else { hi = f }
            let next = f-(h-goal)/dh;
            let next = if next > lo && next < hi { next } else { 0.5*(lo+hi) };
            if (next-f).abs() <= 4.*f64::EPSILON { f = next; break }
            f = next;
        }
        (turns*m as f64+i as f64+f,1./c.rise(i,f).1)
    }
    /// The edge's stretch `[t0, t1]` in `s`.
    fn span(&self,[t0,t1]: [f64;2]) -> [f64;2] { [self.s(t0),self.s(t1)] }
    /// Where, over `[t0, t1]`, the curve may stop being smooth in `s`, its ends among them where
    /// they are such points, as `hermite_fit` reads them: a curve's own `breaks`, but none of a
    /// traced curve's points, across which it is smooth in `s`.
    fn breaks(&self,c: &Curve,[t0,t1]: [f64;2]) -> Vec<f64> {
        if self.chords.is_some() { return vec![] }
        let w = 1e-9*(t1-t0).abs();
        let mut out = c.breaks([t0-w,t1+w]);
        for end in [t0,t1] { if out.iter().any(|&k| (k-end).abs() <= w) && !out.contains(&end) { out.push(end); } }
        out
    }
}

impl Chords {
    /// How far `s` has risen from point `i` at the fraction `f` of its stretch, and its rate:
    /// the cubic from nought to the stretch's length, its slopes the speeds at the ends.
    fn rise(&self,i: usize,f: f64) -> (f64,f64) {
        let (l,[v0,v1]) = (self.at[i+1]-self.at[i],self.speeds[i]);
        let (f2,f3) = (f*f,f*f*f);
        (l*(3.*f2-2.*f3)+v0*(f-2.*f2+f3)+v1*(f3-f2),l*(6.*f-6.*f2)+v0*(1.-4.*f+3.*f2)+v1*(3.*f2-2.*f))
    }
}

/// A use's pcurve `uv(t)` as a cubic over `along`'s `s` within `bar` of the edge (`hermite_fit`),
/// measured on `surface` against the edge's foot there, not the edge: a fitted intersection curve
/// stands off each of its surfaces by its own fit, which no pcurve can close. With the poles,
/// knots and miss, the most the edge was found standing off its foot.
fn bezier_pcurve(uv: &dyn Fn(f64) -> super::geom::Uv,duv: &dyn Fn(f64) -> super::geom::Uv,surface: &Surface,along: &Along,
    c: &Curve,t: [f64;2],bar: f64) -> Option<(Vec<[f64;2]>,Vec<f64>,f64,f64)> {
    let at = |s: f64| uv(along.t(s).0);
    let rate = |s: f64| { let (t,r) = along.t(s); let d = duv(t); [d[0]*r,d[1]*r] };
    // (and the most the edge stands off the surface at every point the fit reads: what a reader
    // measures it standing off the face by, at the pieces' joints among others)
    let gap = std::cell::Cell::new(0f64);
    let off = |q: [f64;2],s: f64| {
        let foot = surface.point(at(s));
        gap.set(gap.get().max(distance(foot,c.point(along.t(s).0))));
        distance(surface.point(q),foot)
    };
    let (poles,knots,miss) = hermite_fit(&at,&rate,&off,along.span(t),&along.breaks(c,t),bar)?;
    Some((poles,knots,miss,gap.get()))
}

/// **A traced intersection as a cubic** over `along`'s `s` (`hermite_fit`), which no entity of the
/// standard writes: within `bar` of the curve at the same `s`, and — what a reader measures, which
/// puts each loop on its face by the feet of the edge's curve there — the foot of each of its
/// points on either surface within `bar` of the meeting, along that surface. Where the two
/// surfaces meet at an angle `θ`, a point `δ` off the meeting in space has a foot as much as
/// `δ / sin θ` along the surface from it: two cylinders grazing at a few degrees, their cubic
/// held to `5e-5` in space, trimmed a face 1e-3 off its meeting (brep-bench's grazing cylinders,
/// their cushion's volume 1.2e-4 off). `None` for any other curve.
fn bezier_curve(c: &Curve,along: &Along,t: [f64;2],bar: f64) -> Option<(Vec<V>,Vec<f64>,f64)> {
    let Curve::Traced(tr) = c else { return None };
    let at = |s: f64| c.point(along.t(s).0);
    let rate = |s: f64| { let (t,r) = along.t(s); scale(c.d1(t).1,r) };
    // the foot of `q` on `surface`, by Newton along its gradient (the surfaces' sides are signed
    // distances, so a step or two)
    let foot = |surface: &Surface,q: V| {
        let mut p = q;
        for _ in 0..8 {
            let (f,g) = surface.side(p);
            let gg = dot(g,g);
            if !(gg > 0.) || f.abs() <= 1e-15*(1.+norm(p)) { break }
            p = add(p,scale(g,-f/gg));
        }
        p
    };
    let off = |q: V,s: f64| {
        let mut d = distance(q,at(s));
        for surface in [&tr.a,&tr.b] { let p = foot(surface,q); d = d.max(distance(p,tr.project(p))); }
        d
    };
    hermite_fit(&at,&rate,&off,along.span(t),&along.breaks(c,t),bar)
}

/// A use's pcurve, as a STEP `PCURVE` on the face's surface: a line where it is one, a circle
/// where a plane's edge is one turning the plane's way, otherwise the cubic within `fit`'s bar
/// of the edge (`bezier_pcurve`), over the parameter the edge's curve is written over (`Along`);
/// and where that does not close in, the degree-one B-spline through its parameters at points
/// chosen so each chord's middle maps within `Fit::polyline` of the edge. **A fitted pcurve is
/// held no nearer the edge's foot than the edge stands off the face** (its tolerance, where that
/// is past the bar): the foot of an edge standing off is the kernel's search, which a cubic held
/// to a thousandth of the edge's own distance chased into pieces of `2⁻³⁰` of the edge (the
/// drill's flutes, the toolpaths' nets: a rate read just past a joint some six times the feet's
/// own). With its id, how far its image was measured off the edge's foot (nought where it is
/// written exactly), and whether it is the kernel's own curve in the face's parameters written as
/// it is (`Pcurve::Curve`, a spline built there); `None`, no pcurve written, where neither the
/// cubic nor a polyline of `MOST` points holds it.
fn pcurve(o: &mut Out,b: &Brep,fi: usize,u: &super::topo::Coedge,surface: usize,context2: usize,tol: f64,fit: Fit,net: Option<&OnNet>,
    gap: f64) -> Option<(usize,f64,bool)> {
    use super::topo::Pcurve;
    let f = &b.faces[fi];
    let e = &b.edges[u.edge as usize];
    let EdgeCurve::Curve(c) = &e.curve else { unreachable!("a degenerate edge has no pcurve written") };
    let at = |t: f64| {
        let uv = u.pcurve.at(t,e,&f.surface,&b.vertices);
        match net { Some(n) => n.uv(uv).0,None => step_uv(&f.surface,uv) }
    };
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
    // the edge's moved there too (a traced curve is written over its chords' length, unmoved)
    let shift = match c { Curve::Circle(..) | Curve::Ellipse(..) => (e.t[0]/std::f64::consts::TAU).floor()*std::f64::consts::TAU,_ => 0. };
    // the parameter the edge's curve is written over (a traced curve's chords, `Along`)
    let along = Along::of(c);
    // affine in the edge's parameter (a coaxial circle on a cylinder, a line along its axis),
    // read at sixteen fractions spread unevenly: at the eighths of a turn a net's arc parameter
    // equals the angle by symmetry, and a line read only there was written for a sheared torus's
    // circles 0.075 mm off between them
    let affine = along.chords.is_none() && (0..16).all(|k| {
        let s = (k as f64+0.381966)/16.;
        let q = at(e.t[0]+(e.t[1]-e.t[0])*s);
        (q[0]-(a[0]+s*(z[0]-a[0]))).abs().max((q[1]-(a[1]+s*(z[1]-a[1]))).abs()) <= 1e-12*(1.+a[0].abs().max(a[1].abs()))
    });
    // (held to the edge's own distance off the face where that is past the fit's bar)
    let bar = fit.bar().max(gap);
    let written = |o: &mut Out,curve: usize| {
        let rep = o.add(format!("DEFINITIONAL_REPRESENTATION('',(#{curve}),#{context2})"));
        o.add(format!("PCURVE('',#{surface},#{rep})"))
    };
    let (curve,miss) = match (&u.pcurve,&f.surface,c) {
        // linear in the edge's parameter: a line through where it is at t = 0
        (Pcurve::Line {..},_,_) | (_,_,_) if affine && (z[0]-a[0]).dhypot(z[1]-a[1]) > 0. => {
            let d = [(z[0]-a[0])/(e.t[1]-e.t[0]),(z[1]-a[1])/(e.t[1]-e.t[0])];
            (line(o,[a[0]-d[0]*(e.t[0]-shift),a[1]-d[1]*(e.t[0]-shift)],d),0.)
        }
        // a kernel's own curve in the face's parameters (`brep::json`): its poles, exactly
        (Pcurve::Curve(pc),_,_) if matches!(**pc,Curve::BSpline(_)) && net.is_none_or(OnNet::plain) => {
            let Curve::BSpline(bs) = &**pc else { unreachable!() };
            let ids: Vec<String> = bs.poles.iter().map(|q| format!("#{}",p2(o,step_uv(&f.surface,[q[0],q[1]])))).collect();
            let knots: Vec<f64> = bs.knots.iter().map(|k| k-shift).collect();
            let own = spline(o,bs.degree,&ids,&knots,bs.weights.as_deref());
            return Some((written(o,own),0.,true))
        }
        (_,Surface::Plane(pl),Curve::Line {p,d}) => {
            let (l,dl) = (pl.local(*p),pl.dir_local(*d));
            (line(o,[l[0],l[1]],[dl[0],dl[1]]),0.)
        }
        (_,Surface::Plane(pl),Curve::Circle(cf,r)) if crate::util::space::dot(cf.z,pl.z) > 0. => {
            let (l,x) = (pl.local(cf.o),pl.dir_local(cf.x));
            let p = p2(o,[l[0],l[1]]);
            let dir = o.add(format!("DIRECTION('',({},{}))",real(x[0]),real(x[1])));
            let ax = o.add(format!("AXIS2_PLACEMENT_2D('',#{p},#{dir})"));
            (o.add(format!("CIRCLE('',#{ax},{})",real(*r))),0.)
        }
        // a plane's parameters are an affine image of space: a B-spline's poles carried over
        (_,Surface::Plane(pl),Curve::BSpline(bs)) => {
            let ids: Vec<String> = bs.poles.iter().map(|&q| { let l = pl.local(q); format!("#{}",p2(o,[l[0],l[1]])) }).collect();
            (spline(o,bs.degree,&ids,&bs.knots,bs.weights.as_deref()),0.)
        }
        // a cubic in the edge's parameter through its parameters and rates, its pieces as few as
        // map within `fit`'s bar of the edge (`hermite_fit`; a cubic's error changes sign about
        // the edge less than a chord's, which always cuts inside, so a reader measuring the face it
        // trims reads it nearer): in the parameters the file writes, measured on the surface it
        // writes — a net's, where they need be no affine image of the face's own
        _ if let Some((poles,knots,miss,off)) = match net {
            Some(n) => bezier_pcurve(&at,&|t| {
                let (uv,d) = u.pcurve.at_and_derivative(t,e,&f.surface,&b.vertices);
                let rate = n.uv(uv).1;
                [rate[0]*d[0],rate[1]*d[1]]
            },&n.surface,&along,c,e.t,bar),
            None => bezier_pcurve(&|t| u.pcurve.at(t,e,&f.surface,&b.vertices),&|t| u.pcurve.derivative(t,e,&f.surface,&b.vertices),
                &f.surface,&along,c,e.t,bar).map(|(poles,knots,miss,off)| (poles.iter().map(|&q| step_uv(&f.surface,q)).collect(),knots,miss,off)),
        } => {
            let ids: Vec<String> = poles.iter().map(|&q| format!("#{}",p2(o,q))).collect();
            let knots: Vec<f64> = knots.iter().map(|k| k-shift).collect();
            // (the edge's distance off the face where the fit found it farther than `standing`)
            (spline(o,3,&ids,&knots,None),miss+(off-gap).max(0.))
        }
        _ => {
            let off = |ta: f64,tz: f64| {
                let (ua,uz) = (u.pcurve.at(ta,e,&f.surface,&b.vertices),u.pcurve.at(tz,e,&f.surface,&b.vertices));
                distance(f.surface.point([(ua[0]+uz[0])/2.,(ua[1]+uz[1])/2.]),c.point((ta+tz)/2.))
            };
            let within = fit.polyline(tol).max(gap);
            let ts = samples(c,e.t,within,&off)?;
            if super::debug() {
                let middle = crate::util::warm::kept(|| at((e.t[0]+e.t[1])/2.));
                eprintln!("step: a {} on a {} as {} points, ends {a:?} {z:?}, middle {middle:?}",c.kind(),f.surface.kind(),ts.len());
            }
            let ids: Vec<String> = ts.iter().map(|&t| format!("#{}",p2(o,at(t)))).collect();
            let knots: Vec<f64> = ts.iter().map(|&t| along.s(t)-shift).collect();
            (bspline(o,&ids,&knots),within)
        }
    };
    Some((written(o,curve),miss,false))
}

/// The solid as a STEP file, `name` its product's: stating the uncertainty `tol` (millimetres, no
/// finer than 1e-7) or, where its geometry holds no nearer, what it holds (`write_stating`), its
/// fitted curves held as `fit` says — `Fit::Exact` where no tolerance was asked. Refused where a
/// loft's face or rail cannot be fitted within `FIT`.
pub fn write(b: &Brep,name: &str,tol: f64,fit: Fit) -> Result<String,String> { write_stating(b,name,tol,fit).map(|w| w.0) }

/// **`write`, and the uncertainty the file states** (millimetres). **The file never states less
/// than it holds** (#993 stage 1): it states `tol` (no finer than 1e-7) where that covers the most
/// a reader can measure anything written standing off what it stands for, and else what it holds
/// (`statement`) — each edge's own distance off its faces (its tolerance), and on the way from
/// its curve as written to the image of each of its curves in a face's parameters on the face's
/// surface as written, every fit's measured miss: the curve's, the surface's and the pcurve's; and
/// each vertex's own tolerance. A loft's net and rails are fitted within `FIT`, a micron, and a
/// solid's own edges may stand off their faces farther than any fit (`solid_sweeps/creased.sv`'s,
/// 2e-5 off its net): a file of either asked at 1e-7 states what it holds, and the caller is told
/// it here; and the lumps written (`lumps`), which a check of the file reads its faces by
/// (`step_check::Solid::of_lumps`).
pub fn write_stating(b: &Brep,name: &str,tol: f64,fit: Fit) -> Result<(String,f64,Vec<Lump>),String> {
    let mut o = Out {text:String::new(),next:1};
    let context = application(&mut o);
    let pctx = o.add(format!("PRODUCT_CONTEXT('',#{context},'mechanical')"));
    let name = name.replace('\'',"''");
    let (_,shape) = product(&mut o,&name,context,pctx,&mut None);
    let gctx = geometric(&mut o);
    let (solids,held,lumps) = solid(&mut o,b,tol,fit)?;
    let origin = o.placement(&Frame::WORLD);
    let rep = o.add(format!("ADVANCED_BREP_SHAPE_REPRESENTATION('',({}),#{gctx})",items(origin,&solids)));
    o.add(format!("SHAPE_DEFINITION_REPRESENTATION(#{shape},#{rep})"));
    if !held.is_finite() { return Err("the solid's geometry is not finite: no uncertainty covers it".into()) }
    let stated = statement(tol,held);
    gctx.state(&mut o,stated);
    Ok((file(&name,&o.text),stated,lumps))
}

/// **The uncertainty a file states** for a tolerance asked, `tol`, over solids whose written
/// geometry holds within `held` (`write_stating`): `tol` (no finer than 1e-7) where it covers a
/// quarter more than `held` (a fit's miss is measured at sixteen points a piece, the reader's at
/// others), else that, up to the next of 1, 2 and 5 times a power of ten.
pub fn statement(tol: f64,held: f64) -> f64 {
    let (asked,held) = (tol.max(LEAST_UNCERTAINTY),1.25*held);
    if !(held > asked) { return asked }
    // (a power of ten below one as a quotient, so 5e-6 is written as itself)
    let k = held.dlog10().floor() as i32;
    let at = |m: f64| if k < 0 { m/10f64.powi(-k) } else { m*10f64.powi(k) };
    [1.,2.,5.,10.,20.].into_iter().map(at).find(|&x| x >= held).unwrap_or(at(20.))
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
pub fn write_assembly(a: &Assembly,name: &str,tol: f64,fit: Fit) -> Result<String,String> {
    a.check()?;
    let mut o = Out {text:String::new(),next:1};
    let context = application(&mut o);
    let pctx = o.add(format!("PRODUCT_CONTEXT('',#{context},'mechanical')"));
    let gctx = geometric(&mut o);
    let mut held = 0f64;
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
                let (solids,most,_) = solid(&mut o,b,tol,fit)?;
                held = held.max(most);
                if let Some(c) = p.colour { coloured.extend(solids.iter().map(|&s| (s,c))); }
                o.add(format!("ADVANCED_BREP_SHAPE_REPRESENTATION('',({}),#{gctx})",items(origin,&solids)))
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
    if !held.is_finite() { return Err("a part's geometry is not finite: no uncertainty covers it".into()) }
    gctx.state(&mut o,statement(tol,held));
    Ok(file(&name.replace('\'',"''"),&o.text))
}

/// A shape representation's items: its axes, and its solids.
fn items(origin: usize,solids: &[usize]) -> String {
    std::iter::once(origin).chain(solids.iter().copied()).map(|i| format!("#{i}")).collect::<Vec<_>>().join(",")
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

/// The context the geometry is written in: its units (millimetres, radians), written first, and
/// its id, kept for the context itself, written once its uncertainty is known (`Geometric::state`).
struct Geometric { id: usize,units: [usize;3] }

impl std::fmt::Display for Geometric {
    fn fmt(&self,f: &mut std::fmt::Formatter) -> std::fmt::Result { self.id.fmt(f) }
}

impl Geometric {
    /// The context, stating `uncertainty` (millimetres).
    fn state(&self,o: &mut Out,uncertainty: f64) {
        let [mm,rad,sr] = self.units;
        let u = o.add(format!("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE({}),#{mm},'distance_accuracy_value','confusion accuracy')",real(uncertainty)));
        o.put(self.id,format!("( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{u})) \
            GLOBAL_UNIT_ASSIGNED_CONTEXT((#{mm},#{rad},#{sr})) REPRESENTATION_CONTEXT('Context #1','3D Context with UNIT and UNCERTAINTY') )"));
    }
}

fn geometric(o: &mut Out) -> Geometric {
    let mm = o.add("( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) )".into());
    let rad = o.add("( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) )".into());
    let sr = o.add("( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() )".into());
    Geometric {id:o.reserve(),units:[mm,rad,sr]}
}

/// The file around its data section, `name` (escaped already) its own.
fn file(name: &str,data: &str) -> String {
    format!("ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('Solvent model'),'2;1');\nFILE_NAME('{name}','',(''),(''),'Solvent','Solvent','');\n\
        FILE_SCHEMA(('AUTOMOTIVE_DESIGN {{ 1 0 10303 214 1 1 1 1 }}'));\nENDSEC;\nDATA;\n{data}ENDSEC;\nEND-ISO-10303-21;\n")
}

/// **A lump of a solid as the file writes it** (`lumps`): the faces of its outer shell, and of each
/// of its voids' shells, by their indices in the `Brep`, each in its order there.
#[derive(Clone,Debug,PartialEq)]
pub struct Lump { pub outer: Vec<usize>,pub voids: Vec<Vec<usize>> }

/// **The lumps a `Brep` is written as** (#993 stage 2). ISO 10303-42 holds a `closed_shell` to be
/// a connected face set (its faces, edges and vertices together connected), a
/// `manifold_solid_brep` to be bounded by one outer closed shell, and a void to be an inner
/// shell of a `brep_with_voids`, disjoint from the others and inside its outer shell: so a
/// `Brep` of several shells (its faces' components by the edges they share) is not one closed
/// shell. Its shells are nested as a reader judges them (`step_read::sound::nesting`: each facing
/// out, a lump, or in, a void; a void inside a lump), each lump written as one solid with its voids
/// — the voids each the void nearest inside it — and the lumps in the order of their first faces.
/// A `Brep` of one shell is one lump whatever it faces (nothing is decided that a reader does not
/// decide itself). Refused, never written as one shell, where its shells nest as no solid's do.
pub fn lumps(b: &Brep) -> Result<Vec<Lump>,String> {
    let ne = b.edges.len();
    if let Some(fi) = b.faces.iter().position(|f| f.loops.iter().flatten().any(|c| c.edge as usize >= ne)) {
        return Err(format!("face {fi} uses no edge of the solid"))
    }
    let shells = super::step_read::sound::shells_of(b);
    if shells.len() <= 1 { return Ok(vec![Lump {outer:(0..b.faces.len()).collect(),voids:Vec::new()}]) }
    let n = super::step_read::sound::nesting(b,shells).map_err(|u| format!("its shells nest as no solid's do: {}",u.msg))?;
    Ok((0..n.shells.len()).filter(|&x| n.outward[x]).map(|x| Lump {outer:n.shells[x].clone(),
        voids:(0..n.shells.len()).filter(|&y| !n.outward[y] && n.about[y] == Some(x)).map(|y| n.shells[y].clone()).collect()}).collect())
}

impl Lump {
    /// Its faces in the order the file writes them: its outer shell's, then each void's.
    pub fn faces(&self) -> impl Iterator<Item = usize> + '_ { self.outer.iter().chain(self.voids.iter().flatten()).copied() }
}

/// **The lumps a check reads a file of `b` by** (`lumps`): or, for a solid the writer refuses,
/// whose file is never read, one lump of every face in its own order.
pub fn written_lumps(b: &Brep) -> Vec<Lump> {
    lumps(b).unwrap_or_else(|_| vec![Lump {outer:(0..b.faces.len()).collect(),voids:Vec::new()}])
}

/// **`b`'s faces in the order the writer writes them** (`written_lumps`): each lump's outer
/// shell's, then its voids'.
pub fn written_order(b: &Brep) -> Vec<usize> {
    written_lumps(b).iter().flat_map(Lump::faces).collect()
}

/// The solid's topology and geometry, bottom up, to its manifold solids (one a lump, `lumps`, its
/// voids a `BREP_WITH_VOIDS`'s): their ids, the most anything written stands off what it stands
/// for (`held`), and the lumps.
fn solid(o: &mut Out,b: &Brep,tol: f64,fit: Fit) -> Result<(Vec<usize>,f64,Vec<Lump>),String> {
    // (every index in range: a `Brep` handed in from outside the kernel's own construction — read
    // from a file, from JSON — is refused, not followed past its arrays)
    let nv = b.vertices.len();
    if let Some(ei) = b.edges.iter().position(|e| e.v.iter().any(|&v| v as usize >= nv)) { return Err(format!("edge {ei} ends on no vertex of the solid")) }
    // (and a face's edges, by `lumps`)
    let lumps = lumps(b)?;
    // the topology, bottom up
    // only the vertices a written edge ends at (a pole's degenerate edge is not written)
    let mut wanted = vec![false;b.vertices.len()];
    for e in &b.edges { if let EdgeCurve::Curve(_) = e.curve { for v in e.v { wanted[v as usize] = true; } } }
    let vertices: Vec<usize> = b.vertices.iter().zip(&wanted).map(|(v,&w)| if w {
        let p = o.point(v.p); o.add(format!("VERTEX_POINT('',#{p})"))
    } else { 0 }).collect();
    // (a vertex stands off its edges' ends by its own tolerance: every curve is written ending
    // where the edge's does)
    let mut held = b.vertices.iter().zip(&wanted).filter(|(_,&w)| w).map(|(v,_)| v.tol).fold(0.,f64::max);
    let nets: Vec<Option<OnNet>> = (0..b.faces.len()).map(|fi| face_net(b,fi).transpose()).collect::<Result<_,_>>()?;
    let (surfaces,sheets): (Vec<usize>,Vec<f64>) = b.faces.iter().zip(&nets).map(|(f,n)| match n {
        Some(n) => Ok((o.net(&n.net),n.fit)),
        None => o.surface(&f.surface),
    }).collect::<Result<Vec<_>,String>>()?.into_iter().unzip();
    let context2 = o.add("( GEOMETRIC_REPRESENTATION_CONTEXT(2) PARAMETRIC_REPRESENTATION_CONTEXT() REPRESENTATION_CONTEXT('2D SPACE','') )".into());
    let edges: Vec<Option<usize>> = b.edges.iter().enumerate().map(|(ei,e)| -> Result<Option<usize>,String> {
        let EdgeCurve::Curve(c) = &e.curve else { return Ok(None) };
        // the curve, and how far it was measured off the edge's
        let (curve,miss) = match c {
            Curve::Line {p,d} => {
                let (p,d) = (o.point(*p),o.direction(*d));
                let v = o.add(format!("VECTOR('',#{d},1.)"));
                (o.add(format!("LINE('',#{p},#{v})")),0.)
            }
            Curve::Circle(f,r) => { let a = o.placement(f); (o.add(format!("CIRCLE('',#{a},{})",real(*r))),0.) }
            Curve::Ellipse(f,a,bb) => { let p = o.placement(f); (o.add(format!("ELLIPSE('',#{p},{},{})",real(*a),real(*bb))),0.) }
            Curve::BSpline(bs) => {
                let ids: Vec<String> = bs.poles.iter().map(|&p| format!("#{}",o.point(p))).collect();
                (spline(o,bs.degree,&ids,&bs.knots,bs.weights.as_deref()),0.)
            }
            // a loft's rail: the cubic through it, fitted within `FIT`, its parameter the rail's
            Curve::Iso(bl,u) => {
                let (fit,err) = match super::nurbs::fit_curve(&|v| bl.d1([*u,v]).0,3,8,FIT) {
                    Some((fit,err)) if err <= FIT => (fit,err),
                    Some((_,err)) => return Err(format!("a loft's rail is fitted no nearer than {err:.3e} mm, past the file's {FIT:e}")),
                    None => return Err("a loft's rail could not be fitted (it is degenerate)".into()),
                };
                let ids: Vec<String> = fit.poles.iter().map(|&p| format!("#{}",o.point(p))).collect();
                (spline(o,fit.degree,&ids,&fit.knots,None),err)
            }
            // a family's rail or rim likewise, its parameter the rail's or the rim's; an affine image
            // of a curve (a rail under a shear); and a plane's cut of a screwed sheet
            Curve::Rail(..) | Curve::Rim(..) | Curve::Affine(..) | Curve::SheetCut(..) | Curve::Turned(..) | Curve::Slid(..)
            | Curve::Trochoid(..) => {
                fitted(o,c,e.t,whose(c))?
            }
            // a traced curve: the cubic through its points and tangents within `fit`'s bar, as
            // its pcurves are (`bezier_pcurve`); where that does not close in, the polyline
            // through it within `Fit::polyline`, or the bar `MOST` points hold where that is
            // past them (over the length of its chords, `Along`, as its pcurves are)
            Curve::Traced(_) => match bezier_curve(c,&Along::of(c),e.t,fit.bar()) {
                Some((poles,knots,miss)) => {
                    let ids: Vec<String> = poles.iter().map(|&p| format!("#{}",o.point(p))).collect();
                    (spline(o,3,&ids,&knots,None),miss)
                }
                None => {
                    let sag = |a: f64,z: f64| distance(c.point((a+z)/2.),crate::util::space::lerp(c.point(a),c.point(z),0.5));
                    let mut within = fit.polyline(tol);
                    let ts = loop {
                        if let Some(ts) = samples(c,e.t,within,&sag) { break ts }
                        within *= 2.;
                    };
                    let ids: Vec<String> = ts.iter().map(|&t| format!("#{}",o.point(c.point(t)))).collect();
                    let along = Along::of(c);
                    (bspline(o,&ids,&ts.iter().map(|&t| along.s(t)).collect::<Vec<_>>()),within)
                }
            },
        };
        // the edge's curve in each face that uses it: a seam's two in one face, or one in each
        let uses: Vec<(usize,&super::topo::Coedge)> = b.faces.iter().enumerate()
            .flat_map(|(fi,f)| f.loops.iter().flatten().filter(|c| c.edge as usize == ei).map(move |c| (fi,c))).collect();
        let seam = uses.len() == 2 && uses[0].0 == uses[1].0;
        let before = (o.text.len(),o.next);
        let gaps: Vec<f64> = uses.iter().map(|&(fi,u)| b.standing(fi,u).max(e.tol)).collect();
        let mut pcurves: Vec<Option<(usize,f64,bool)>> = uses.iter().zip(&gaps)
            .map(|(&(fi,u),&gap)| pcurve(o,b,fi,u,surfaces[fi],context2,tol,fit,nets[fi].as_ref(),gap)).collect();
        // (a seam's two curves are written both or neither: one alone does not say which side of
        // the seam it is)
        if seam && pcurves.iter().any(Option::is_none) {
            o.text.truncate(before.0);
            o.next = before.1;
            pcurves = vec![None,None];
        }
        // what a reader measures the edge off its faces by, at most: its own distance off them,
        // its curve's miss, its faces' surfaces', and its pcurves' (each a distance along the
        // way from the file's curve to the image of the file's pcurve on the file's surface)
        let sheet = uses.iter().map(|&(fi,_)| sheets[fi]).fold(0.,f64::max);
        let drawn = pcurves.iter().flatten().map(|p| p.1).fold(0.,f64::max);
        held = held.max(gaps.iter().fold(0.,|a: f64,&g| a.max(g))+miss+sheet+drawn);
        // (a curve no pcurve is written for is the edge's geometry on its own, a reader putting it
        // on its faces as it does where a file writes none)
        let written: Vec<String> = pcurves.iter().flatten().map(|p| format!("#{}",p.0)).collect();
        // **the representation the file prefers is the exact one** (ISO 10303-42's
        // `master_representation`, which a reader takes where the three differ): the curve in
        // space wherever it is written as itself, and every pcurve but the kernel's own spline in
        // a face's parameters is fitted to it or read off it; a pcurve only where it is such a
        // spline, on a surface written exactly, and the curve in space was fitted (a curve built
        // in a surface's parameters, carried into space and traced or fitted there)
        let fitted = miss > 0. || !matches!(c,Curve::Line {..} | Curve::Circle(..) | Curve::Ellipse(..) | Curve::BSpline(_));
        let master = match uses.iter().zip(&pcurves).filter_map(|(&(fi,_),p)| p.map(|p| (fi,p))).position(|(fi,p)| p.2 && sheets[fi] == 0.) {
            Some(k) if fitted => format!(".PCURVE_S{}.",k+1),
            _ => ".CURVE_3D.".into(),
        };
        let geometry = if written.is_empty() { curve } else {
            o.add(format!("{}('',#{curve},({}),{master})",if seam { "SEAM_CURVE" } else { "SURFACE_CURVE" },written.join(",")))
        };
        Ok(Some(o.add(format!("EDGE_CURVE('',#{},#{},#{geometry},.T.)",vertices[e.v[0] as usize],vertices[e.v[1] as usize]))))
    }).collect::<Result<_,_>>()?;
    // each face, in its lump's order (its outer shell's faces, then each void's), turned where it
    // bounds a void — its sense and its loops: a void's closed shell is written facing out of the
    // void, a closed shell of its own as a solid of it would be, and its oriented shell (`.F.`)
    // turns it back, so each face's sense in the solid is the kernel's
    let mut faces = vec![String::new();b.faces.len()];
    let voided: Vec<bool> = { let mut v = vec![false;b.faces.len()]; for l in &lumps { for &fi in l.voids.iter().flatten() { v[fi] = true; } } v };
    for fi in lumps.iter().flat_map(Lump::faces) {
        let f = &b.faces[fi];
        let surface = surfaces[fi];
        let mut bounds = Vec::new();
        for (li,l) in f.loops.iter().enumerate() {
            let mut uses: Vec<String> = Vec::new();
            // (a void's face, its sense turned in its shell, has its loops run the other way with
            // it: each its uses in the reverse order, each turned)
            let turn = voided[fi];
            let ordered: Box<dyn Iterator<Item = &super::topo::Coedge>> = if turn { Box::new(l.iter().rev()) } else { Box::new(l.iter()) };
            for c in ordered {
                let Some(e) = edges[c.edge as usize] else { continue };
                uses.push(format!("#{}",o.add(format!("ORIENTED_EDGE('',*,*,#{e},{})",if c.reversed != turn { ".F." } else { ".T." }))));
            }
            if uses.is_empty() { continue }
            let lp = o.add(format!("EDGE_LOOP('',({}))",uses.join(",")));
            bounds.push(format!("#{}",o.add(format!("{}('',#{lp},.T.)",if li == 0 { "FACE_OUTER_BOUND" } else { "FACE_BOUND" }))));
        }
        // against its surface where the solid reverses it, or where the file's surface runs against
        // the kernel's (`turned`), not both
        let against = (f.reversed != turned(&f.surface)) != voided[fi];
        faces[fi] = format!("#{}",o.add(format!("ADVANCED_FACE('',({}),#{surface},{})",bounds.join(","),if against { ".F." } else { ".T." })));
    }
    let shell = |o: &mut Out,s: &[usize]| o.add(format!("CLOSED_SHELL('',({}))",s.iter().map(|&fi| faces[fi].as_str()).collect::<Vec<_>>().join(",")));
    let solids = lumps.iter().map(|l| {
        let outer = shell(o,&l.outer);
        if l.voids.is_empty() { return o.add(format!("MANIFOLD_SOLID_BREP('',#{outer})")) }
        let voids: Vec<String> = l.voids.iter().map(|v| { let s = shell(o,v); format!("#{}",o.add(format!("ORIENTED_CLOSED_SHELL('',*,#{s},.F.)"))) }).collect();
        o.add(format!("BREP_WITH_VOIDS('',#{outer},({}))",voids.join(",")))
    }).collect();
    Ok((solids,held,lumps))
}
