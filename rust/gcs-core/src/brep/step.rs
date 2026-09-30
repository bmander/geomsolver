//! A B-rep written as STEP (ISO 10303-21, AP214's advanced B-rep): the product and units a
//! reader expects (millimetres, radians), a manifold solid on one closed shell, its faces on
//! their exact surfaces and its edges on their exact curves; a traced curve written as the
//! B-spline of degree one through points within `tol` of it; every edge's curve in each face that
//! uses it (its pcurve, a seam's two) written beside it, so a reader need not project its own; and
//! a pole's degenerate edge left out of its loop, as a reader rebuilds it.
use super::geom::{Curve,Frame,Surface,V};
use super::topo::{Brep,EdgeCurve};
use crate::space::{distance,scale};
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
    fn surface(&mut self,s: &Surface) -> usize {
        match s {
            Surface::Extrusion(f,c) => {
                let c = self.swept(c);
                let d = self.direction(f.z);
                let v = self.add(format!("VECTOR('',#{d},1.)"));
                return self.add(format!("SURFACE_OF_LINEAR_EXTRUSION('',#{c},#{v})"))
            }
            Surface::Blend(_,b) => {
                let net = blend_net(b);
                let rows: Vec<String> = net.poles.iter().map(|col| {
                    let ids: Vec<String> = col.iter().map(|&p| format!("#{}",self.point(p))).collect();
                    format!("({})",ids.join(","))
                }).collect();
                let (uk,vk) = (super::nurbs::distinct(&net.uknots),super::nurbs::distinct(&net.vknots));
                let list = |k: &[(f64,usize)],f: &dyn Fn(&(f64,usize)) -> String| k.iter().map(f).collect::<Vec<_>>().join(",");
                return self.add(format!("B_SPLINE_SURFACE_WITH_KNOTS('',{},{},({}),.UNSPECIFIED.,.F.,.F.,.F.,({}),({}),({}),({}),.UNSPECIFIED.)",
                    net.du,net.dv,rows.join(","),list(&uk,&|k| k.1.to_string()),list(&vk,&|k| k.1.to_string()),
                    list(&uk,&|k| real(k.0)),list(&vk,&|k| real(k.0))))
            }
            Surface::Revolution(f,c) => {
                let c = self.swept(c);
                let (o,z) = (self.point(f.o),self.direction(f.z));
                let a = self.add(format!("AXIS1_PLACEMENT('',#{o},#{z})"));
                return self.add(format!("SURFACE_OF_REVOLUTION('',#{c},#{a})"))
            }
            _ => {}
        }
        match *s {
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
            Surface::Extrusion(..) | Surface::Revolution(..) | Surface::Blend(..) => unreachable!(),
        }
    }
    /// The curve a swept surface sweeps (a B-spline, as a profile gives one).
    fn swept(&mut self,c: &Curve) -> usize {
        let Curve::BSpline(b) = c else { unreachable!("a swept surface's curve is a profile's B-spline") };
        let ids: Vec<String> = b.poles.iter().map(|&p| format!("#{}",self.point(p))).collect();
        spline(self,b.degree,&ids,&b.knots)
    }
}

/// A B-spline of `degree` through the poles `ids` over the full knot vector `knots` (written as
/// its distinct knots and their multiplicities).
fn spline(o: &mut Out,degree: usize,ids: &[String],knots: &[f64]) -> usize {
    let mut distinct: Vec<(f64,usize)> = Vec::new();
    for &k in knots { match distinct.last_mut() { Some((x,n)) if *x == k => *n += 1,_ => distinct.push((k,1)) } }
    let mults: Vec<String> = distinct.iter().map(|d| d.1.to_string()).collect();
    let ks: Vec<String> = distinct.iter().map(|d| real(d.0)).collect();
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
pub fn blend_net(b: &super::geom::Blend) -> super::nurbs::Net {
    let (dv,nv) = match b.carry { super::geom::Carry::Line {..} => (1,1),super::geom::Carry::Arc {..} => (3,4) };
    super::nurbs::fit_net(&|u,v| b.d1([u,v]).0,3,dv,8,nv,FIT).expect("a loft face's fit").0
}

/// A B-spline surface's numbers as a reader of the file compares them: its degrees, the size of
/// its net, its distinct knots' counts, whether rational, the knots and their multiplicities along
/// `u` then `v`, then its poles by `u` then `v`.
pub fn net_numbers(net: &super::nurbs::Net) -> Vec<f64> {
    let (uk,vk) = (super::nurbs::distinct(&net.uknots),super::nurbs::distinct(&net.vknots));
    let mut out = vec![net.du as f64,net.dv as f64,net.poles.len() as f64,net.poles[0].len() as f64,uk.len() as f64,vk.len() as f64,0.];
    out.extend(uk.iter().map(|k| k.0)); out.extend(uk.iter().map(|k| k.1 as f64));
    out.extend(vk.iter().map(|k| k.0)); out.extend(vk.iter().map(|k| k.1 as f64));
    for col in &net.poles { for p in col { out.extend(p); } }
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
        Surface::Extrusion(f,_) | Surface::Revolution(f,_) | Surface::Blend(f,_) => (f,vec![]),
    }
}

/// A face's parameters as STEP's surface reads them: the same, but on a cone written about its
/// reversed axis, where both run the other way.
fn step_uv(s: &Surface,uv: super::geom::Uv) -> [f64;2] {
    match s { Surface::Cone(_,_,a) if *a < 0. => [-uv[0],-uv[1]],_ => uv }
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
        let l = d[0].hypot(d[1]);
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
        (Pcurve::Line {..},_,_) | (_,_,_) if affine && (z[0]-a[0]).hypot(z[1]-a[1]) > 0. => {
            let d = [(z[0]-a[0])/(e.t[1]-e.t[0]),(z[1]-a[1])/(e.t[1]-e.t[0])];
            line(o,[a[0]-d[0]*(e.t[0]-shift),a[1]-d[1]*(e.t[0]-shift)],d)
        }
        (_,Surface::Plane(pl),Curve::Line {p,d}) => {
            let (l,dl) = (pl.local(*p),pl.dir_local(*d));
            line(o,[l[0],l[1]],[dl[0],dl[1]])
        }
        (_,Surface::Plane(pl),Curve::Circle(cf,r)) if crate::space::dot(cf.z,pl.z) > 0. => {
            let (l,x) = (pl.local(cf.o),pl.dir_local(cf.x));
            let p = p2(o,[l[0],l[1]]);
            let dir = o.add(format!("DIRECTION('',({},{}))",real(x[0]),real(x[1])));
            let ax = o.add(format!("AXIS2_PLACEMENT_2D('',#{p},#{dir})"));
            o.add(format!("CIRCLE('',#{ax},{})",real(*r)))
        }
        // a plane's parameters are an affine image of space: a B-spline's poles carried over
        (_,Surface::Plane(pl),Curve::BSpline(bs)) => {
            let ids: Vec<String> = bs.poles.iter().map(|&q| { let l = pl.local(q); format!("#{}",p2(o,[l[0],l[1]])) }).collect();
            spline(o,bs.degree,&ids,&bs.knots)
        }
        _ => {
            let off = |ta: f64,tz: f64| {
                let (ua,uz) = (u.pcurve.at(ta,e,&f.surface,&b.vertices),u.pcurve.at(tz,e,&f.surface,&b.vertices));
                distance(f.surface.point([(ua[0]+uz[0])/2.,(ua[1]+uz[1])/2.]),c.point((ta+tz)/2.))
            };
            let ts = samples(c,e.t,tol,&off);
            if std::env::var_os("SOLVENT_BREP_DEBUG").is_some() {
                eprintln!("step: a {} on a {} as {} points, ends {a:?} {z:?}, middle {:?}",c.kind(),f.surface.kind(),ts.len(),at((e.t[0]+e.t[1])/2.));
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
pub fn write(b: &Brep,name: &str,tol: f64) -> String {
    let mut o = Out {text:String::new(),next:1};
    let context = o.add("APPLICATION_CONTEXT('core data for automotive mechanical design processes')".into());
    o.add(format!("APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#{context})"));
    let pctx = o.add(format!("PRODUCT_CONTEXT('',#{context},'mechanical')"));
    let name = name.replace('\'',"''");
    let product = o.add(format!("PRODUCT('{name}','{name}','',(#{pctx}))"));
    o.add(format!("PRODUCT_RELATED_PRODUCT_CATEGORY('part',$,(#{product}))"));
    let formation = o.add(format!("PRODUCT_DEFINITION_FORMATION('','',#{product})"));
    let dctx = o.add(format!("PRODUCT_DEFINITION_CONTEXT('part definition',#{context},'design')"));
    let definition = o.add(format!("PRODUCT_DEFINITION('design','',#{formation},#{dctx})"));
    let shape = o.add(format!("PRODUCT_DEFINITION_SHAPE('','',#{definition})"));
    // the units, and the geometry's context
    let mm = o.add("( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) )".into());
    let rad = o.add("( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) )".into());
    let sr = o.add("( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() )".into());
    let uncertainty = o.add(format!("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE({}),#{mm},'distance_accuracy_value','confusion accuracy')",real(tol.max(1e-7))));
    let gctx = o.add(format!("( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{uncertainty})) \
        GLOBAL_UNIT_ASSIGNED_CONTEXT((#{mm},#{rad},#{sr})) REPRESENTATION_CONTEXT('Context #1','3D Context with UNIT and UNCERTAINTY') )"));
    // the topology, bottom up
    // only the vertices a written edge ends at (a pole's degenerate edge is not written)
    let mut wanted = vec![false;b.vertices.len()];
    for e in &b.edges { if let EdgeCurve::Curve(_) = e.curve { for v in e.v { wanted[v as usize] = true; } } }
    let vertices: Vec<usize> = b.vertices.iter().zip(&wanted).map(|(v,&w)| if w {
        let p = o.point(v.p); o.add(format!("VERTEX_POINT('',#{p})"))
    } else { 0 }).collect();
    let surfaces: Vec<usize> = b.faces.iter().map(|f| o.surface(&f.surface)).collect();
    let context2 = o.add("( GEOMETRIC_REPRESENTATION_CONTEXT(2) PARAMETRIC_REPRESENTATION_CONTEXT() REPRESENTATION_CONTEXT('2D SPACE','') )".into());
    let edges: Vec<Option<usize>> = b.edges.iter().enumerate().map(|(ei,e)| {
        let EdgeCurve::Curve(c) = &e.curve else { return None };
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
                spline(&mut o,bs.degree,&ids,&bs.knots)
            }
            // a loft's rail: the cubic through it, fitted within `FIT`, its parameter the rail's
            Curve::Iso(bl,u) => {
                let (fit,_) = super::nurbs::fit_curve(&|v| bl.d1([*u,v]).0,3,8,FIT).expect("a rail's fit");
                let ids: Vec<String> = fit.poles.iter().map(|&p| format!("#{}",o.point(p))).collect();
                spline(&mut o,fit.degree,&ids,&fit.knots)
            }
            Curve::Traced(_) => {
                let ts = samples(c,e.t,tol,&|a,z| distance(c.point((a+z)/2.),crate::space::lerp(c.point(a),c.point(z),0.5)));
                let ids: Vec<String> = ts.iter().map(|&t| format!("#{}",o.point(c.point(t)))).collect();
                bspline(&mut o,&ids,&ts)
            }
        };
        // the edge's curve in each face that uses it: a seam's two in one face, or one in each
        let uses: Vec<(usize,&super::topo::Coedge)> = b.faces.iter().enumerate()
            .flat_map(|(fi,f)| f.loops.iter().flatten().filter(|c| c.edge as usize == ei).map(move |c| (fi,c))).collect();
        let pcurves: Vec<String> = uses.iter().map(|&(fi,u)| format!("#{}",pcurve(&mut o,b,fi,u,surfaces[fi],context2,tol))).collect();
        let seam = uses.len() == 2 && uses[0].0 == uses[1].0;
        let geometry = o.add(format!("{}('',#{curve},({}),.PCURVE_S1.)",if seam { "SEAM_CURVE" } else { "SURFACE_CURVE" },pcurves.join(",")));
        Some(o.add(format!("EDGE_CURVE('',#{},#{},#{geometry},.T.)",vertices[e.v[0] as usize],vertices[e.v[1] as usize])))
    }).collect();
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
    let solid = o.add(format!("MANIFOLD_SOLID_BREP('',#{shell})"));
    let origin = o.placement(&Frame {o:[0.;3],x:[1.,0.,0.],y:[0.,1.,0.],z:[0.,0.,1.]});
    let rep = o.add(format!("ADVANCED_BREP_SHAPE_REPRESENTATION('',(#{origin},#{solid}),#{gctx})"));
    o.add(format!("SHAPE_DEFINITION_REPRESENTATION(#{shape},#{rep})"));
    format!("ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('Solvent model'),'2;1');\nFILE_NAME('{name}','',(''),(''),'Solvent','Solvent','');\n\
        FILE_SCHEMA(('AUTOMOTIVE_DESIGN {{ 1 0 10303 214 1 1 1 1 }}'));\nENDSEC;\nDATA;\n{}ENDSEC;\nEND-ISO-10303-21;\n",o.text)
}
