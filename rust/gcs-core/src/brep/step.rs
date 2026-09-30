//! A B-rep written as STEP (ISO 10303-21, AP214's advanced B-rep): the product and units a
//! reader expects (millimetres, radians), a manifold solid on one closed shell, its faces on
//! their exact surfaces and its edges on their exact curves; a traced curve written as the
//! B-spline of degree one through points within `tol` of it. No pcurves are written — a reader
//! projects them, as it must for any file — and a pole's degenerate edge is left out of its loop.
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
        }
    }
}

/// The points of a stretch of a traced curve within `tol` of it: its own points between the ends,
/// and more wherever a chord strays farther.
fn polyline(c: &Curve,t: [f64;2],tol: f64) -> Vec<V> {
    let mut ts: Vec<f64> = vec![t[0]];
    ts.extend((t[0].floor() as i64+1..=t[1].ceil() as i64-1).map(|k| k as f64));
    ts.push(t[1]);
    let mut out = vec![c.point(t[0])];
    for w in ts.windows(2) {
        let mut stack = vec![(w[0],w[1],0)];
        while let Some((a,z,depth)) = stack.pop() {
            let (pa,pz) = (c.point(a),c.point(z));
            let m = (a+z)/2.;
            let chord = crate::space::lerp(pa,pz,0.5);
            if depth < 24 && distance(c.point(m),chord) > tol { stack.push((m,z,depth+1)); stack.push((a,m,depth+1)); }
            else { out.push(pz); }
        }
    }
    out
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
    let vertices: Vec<usize> = b.vertices.iter().map(|v| { let p = o.point(v.p); o.add(format!("VERTEX_POINT('',#{p})")) }).collect();
    let edges: Vec<Option<usize>> = b.edges.iter().map(|e| {
        let EdgeCurve::Curve(c) = &e.curve else { return None };
        let curve = match c {
            Curve::Line {p,d} => {
                let (p,d) = (o.point(*p),o.direction(*d));
                let v = o.add(format!("VECTOR('',#{d},1.)"));
                o.add(format!("LINE('',#{p},#{v})"))
            }
            Curve::Circle(f,r) => { let a = o.placement(f); o.add(format!("CIRCLE('',#{a},{})",real(*r))) }
            Curve::Ellipse(f,a,bb) => { let p = o.placement(f); o.add(format!("ELLIPSE('',#{p},{},{})",real(*a),real(*bb))) }
            Curve::Traced(_) => {
                let pts = polyline(c,e.t,tol);
                let ids: Vec<String> = pts.iter().map(|&p| format!("#{}",o.point(p))).collect();
                let n = pts.len();
                let knots: Vec<String> = (0..n).map(|k| real(k as f64)).collect();
                let mults: Vec<String> = (0..n).map(|k| if k == 0 || k == n-1 { "2".into() } else { "1".into() }).collect();
                o.add(format!("B_SPLINE_CURVE_WITH_KNOTS('',1,({}),.UNSPECIFIED.,.F.,.F.,({}),({}),.UNSPECIFIED.)",
                    ids.join(","),mults.join(","),knots.join(",")))
            }
        };
        Some(o.add(format!("EDGE_CURVE('',#{},#{},#{curve},.T.)",vertices[e.v[0] as usize],vertices[e.v[1] as usize])))
    }).collect();
    let mut faces = Vec::new();
    for f in &b.faces {
        let surface = o.surface(&f.surface);
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
