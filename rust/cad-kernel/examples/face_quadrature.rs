//! **A referee for one face's area and flux**, sharing nothing with `props` or the reader's
//! topology: the face's surface evaluated from its entity by formulas of its own (each spline by
//! de Boor here), its region the set of points its file's own curves in its parameters (`PCURVE`)
//! wind round, and `∬ |S_u × S_v|` (and the flux `⅓ ∬ S · (S_u × S_v)`, the face's share of the
//! solid's volume) taken by columns: across `u` in strips, each column's crossings with the loops'
//! polygons found, the stretches inside (nonzero winding) integrated along `v` by Gauss–Legendre.
//! Each edge's stretch of its pcurve is bounded by the points of the pcurve nearest its vertices
//! in space. The error estimate is the change from half as many strips and half as many polygon
//! points; derivatives are central differences.
//!
//!     face_quadrature FILE ENTITY…
//!
//! Only the file's parsing and its units (`context`) and the placements' and points' reading
//! (`geometry::Geometry::point`, `direction`, `placement`) are the reader's. Supported: a plane, a
//! cylinder, a cone, a sphere, a torus, a linear extrusion of a line, a circle or a spline, and a
//! spline surface with its knots written (rational or not); curves in the parameters: a line, a
//! circle and a spline with its knots written. Anything else is said and not measured.
use cad_kernel::step_file::{File,Value};
use cad_kernel::step_read::{context,geometry::Geometry};

type V = [f64;3];

/// A B-spline of any dimension (up to 3) by its own de Boor, rational in homogeneous coordinates.
#[derive(Clone)]
struct Spline { p: usize,knots: Vec<f64>,poles: Vec<[f64;4]> }

impl Spline {
    fn new(p: usize,mults: &[usize],ks: &[f64],pts: Vec<V>,w: Option<Vec<f64>>) -> Spline {
        let mut knots = Vec::new();
        for (m,k) in mults.iter().zip(ks) { for _ in 0..*m { knots.push(*k); } }
        let poles = pts.iter().enumerate().map(|(i,q)| { let w = w.as_ref().map_or(1.,|w| w[i]); [q[0]*w,q[1]*w,q[2]*w,w] }).collect();
        Spline {p,knots,poles}
    }
    fn domain(&self) -> [f64;2] { [self.knots[self.p],self.knots[self.poles.len()]] }
    /// The homogeneous point at `t` (clamped into the domain).
    fn h(&self,t: f64) -> [f64;4] {
        let [a,b] = self.domain();
        let t = t.clamp(a,b);
        let n = self.poles.len();
        let k = (self.p..n).rev().find(|&k| self.knots[k] <= t && (self.knots[k+1] > t || k == n-1)).unwrap_or(self.p);
        let mut d: Vec<[f64;4]> = (0..=self.p).map(|j| self.poles[k-self.p+j]).collect();
        for r in 1..=self.p {
            for j in (r..=self.p).rev() {
                let i = k-self.p+j;
                let den = self.knots[i+self.p+1-r]-self.knots[i];
                let a = if den > 0. { (t-self.knots[i])/den } else { 0. };
                for c in 0..4 { d[j][c] = (1.-a)*d[j-1][c]+a*d[j][c]; }
            }
        }
        d[self.p]
    }
    fn at(&self,t: f64) -> V { let h = self.h(t); [h[0]/h[3],h[1]/h[3],h[2]/h[3]] }
}

/// A net: rows along `u` of splines along `v` (each row's poles a spline in `v`, its points then a
/// spline in `u`).
struct Net { du: usize,uk: Vec<f64>,rows: Vec<Spline> }

impl Net {
    fn at(&self,u: f64,v: f64) -> V {
        let col = Spline {p:self.du,knots:self.uk.clone(),poles:self.rows.iter().map(|r| r.h(v)).collect()};
        col.at(u)
    }
}

enum Surf { Plane(Fr),Cyl(Fr,f64),Cone(Fr,f64,f64),Sphere(Fr,f64),Torus(Fr,f64,f64),Extr(Box<Crv>,V),Net(Net) }
#[derive(Clone,Copy)]
struct Fr { o: V,x: V,y: V,z: V }
enum Crv { Line(V,V),Circle(Fr,f64),Ellipse(Fr,f64,f64),Spline(Spline) }

fn add(a: V,b: V) -> V { [a[0]+b[0],a[1]+b[1],a[2]+b[2]] }
fn sc(a: V,s: f64) -> V { [a[0]*s,a[1]*s,a[2]*s] }
fn cross(a: V,b: V) -> V { [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]] }
fn norm(a: V) -> f64 { (a[0]*a[0]+a[1]*a[1]+a[2]*a[2]).sqrt() }
fn dist(a: V,b: V) -> f64 { norm([a[0]-b[0],a[1]-b[1],a[2]-b[2]]) }

impl Crv {
    fn at(&self,t: f64) -> V {
        match self {
            Crv::Line(p,d) => add(*p,sc(*d,t)),
            Crv::Circle(f,r) => add(f.o,add(sc(f.x,r*t.cos()),sc(f.y,r*t.sin()))),
            Crv::Ellipse(f,a,b) => add(f.o,add(sc(f.x,a*t.cos()),sc(f.y,b*t.sin()))),
            Crv::Spline(s) => s.at(t),
        }
    }
}

impl Surf {
    /// The point at the file's parameters (lengths in millimetres, angles in radians).
    fn at(&self,u: f64,v: f64) -> V {
        let ring = |f: &Fr,r: f64,u: f64| add(sc(f.x,r*u.cos()),sc(f.y,r*u.sin()));
        match self {
            Surf::Plane(f) => add(f.o,add(sc(f.x,u),sc(f.y,v))),
            Surf::Cyl(f,r) => add(add(f.o,ring(f,*r,u)),sc(f.z,v)),
            Surf::Cone(f,r,a) => add(add(f.o,ring(f,r+v*a.tan(),u)),sc(f.z,v)),
            Surf::Sphere(f,r) => add(add(f.o,ring(f,r*v.cos(),u)),sc(f.z,r*v.sin())),
            Surf::Torus(f,big,r) => add(add(f.o,ring(f,big+r*v.cos(),u)),sc(f.z,r*v.sin())),
            Surf::Extr(c,d) => add(c.at(u),sc(*d,v)),
            Surf::Net(n) => n.at(u,v),
        }
    }
    /// The point and `S_u × S_v` by central differences `h` apart.
    fn d(&self,u: f64,v: f64,h: [f64;2]) -> (V,V) {
        let su = sc(sub(self.at(u+h[0],v),self.at(u-h[0],v)),0.5/h[0]);
        let sv = sc(sub(self.at(u,v+h[1]),self.at(u,v-h[1])),0.5/h[1]);
        (self.at(u,v),cross(su,sv))
    }
}
fn sub(a: V,b: V) -> V { [a[0]-b[0],a[1]-b[1],a[2]-b[2]] }

struct Reader<'f,'a> { g: Geometry<'f,'a>,file: &'f File<'a> }

impl<'f,'a> Reader<'f,'a> {
    fn ent(&self,r: u32) -> (&'a str,&'f [Value<'a>]) {
        let e = self.file.get(r);
        match e.simple() { Some(x) => x,None => ("COMPLEX",&[]) }
    }
    fn nums(v: &Value) -> Vec<f64> { v.list().map_or(vec![],|l| l.iter().filter_map(Value::number).collect()) }
    fn ints(v: &Value) -> Vec<usize> { v.list().map_or(vec![],|l| l.iter().filter_map(|x| x.number().map(|n| n as usize)).collect()) }
    fn len(&self) -> f64 { self.g.context.length }
    /// A point in space, mm.
    fn pt(&self,r: Option<u32>) -> V { self.g.point(0,r).expect("a point") }
    /// A point of `n` coordinates as written.
    fn raw(&self,r: Option<u32>) -> V {
        let (_,v) = self.ent(r.expect("a point"));
        let c = Self::nums(&v[1]);
        [c[0],c.get(1).copied().unwrap_or(0.),c.get(2).copied().unwrap_or(0.)]
    }
    fn frame(&self,r: Option<u32>) -> Fr {
        let f = self.g.placement(0,r).expect("a placement");
        Fr {o:f.o,x:f.x,y:f.y,z:f.z}
    }
    /// A spline curve (in space scaled to mm where `space`, else as written).
    fn spline(&self,r: u32,space: bool) -> Option<Spline> {
        let e = self.file.get(r);
        let (p,pts,mults,ks,w) = if let Some(("B_SPLINE_CURVE_WITH_KNOTS",v)) = e.simple() {
            (v[1].number()? as usize,v[2].list()?.to_vec(),Self::ints(&v[6]),Self::nums(&v[7]),None)
        } else {
            let b = e.part("B_SPLINE_CURVE")?;
            let k = e.part("B_SPLINE_CURVE_WITH_KNOTS")?;
            (b[0].number()? as usize,b[1].list()?.to_vec(),Self::ints(&k[0]),Self::nums(&k[1]),e.part("RATIONAL_B_SPLINE_CURVE").map(|w| Self::nums(&w[0])))
        };
        let s = if space { self.len() } else { 1. };
        let pts: Vec<V> = pts.iter().map(|q| sc(self.raw(q.reference()),s)).collect();
        Some(Spline::new(p,&mults,&ks,pts,w))
    }
    fn surface(&self,r: u32) -> Result<Surf,String> {
        let e = self.file.get(r);
        let l = self.len();
        let ang = self.g.context.angle;
        if let Some((kind,v)) = e.simple() {
            return Ok(match kind {
                "PLANE" => Surf::Plane(self.frame(v[1].reference())),
                "CYLINDRICAL_SURFACE" => Surf::Cyl(self.frame(v[1].reference()),v[2].number().unwrap()*l),
                "CONICAL_SURFACE" => Surf::Cone(self.frame(v[1].reference()),v[2].number().unwrap()*l,v[3].number().unwrap()*ang),
                "SPHERICAL_SURFACE" => Surf::Sphere(self.frame(v[1].reference()),v[2].number().unwrap()*l),
                "TOROIDAL_SURFACE" => Surf::Torus(self.frame(v[1].reference()),v[2].number().unwrap()*l,v[3].number().unwrap()*l),
                "SURFACE_OF_LINEAR_EXTRUSION" => {
                    let c = self.curve(v[1].reference().unwrap())?;
                    let (_,vec) = self.ent(v[2].reference().unwrap());
                    let d = self.g.direction(0,vec[1].reference()).unwrap();
                    Surf::Extr(Box::new(c),sc(d,vec[2].number().unwrap()*l))
                }
                "B_SPLINE_SURFACE_WITH_KNOTS" => self.net(v[1].number().unwrap() as usize,v[2].number().unwrap() as usize,&v[3],
                    (Self::ints(&v[8]),Self::ints(&v[9]),Self::nums(&v[10]),Self::nums(&v[11])),None),
                k => return Err(format!("a surface of the kind {k}")),
            })
        }
        let (Some(b),Some(k)) = (e.part("B_SPLINE_SURFACE"),e.part("B_SPLINE_SURFACE_WITH_KNOTS")) else { return Err(format!("a surface of the kinds {}",e.kinds())) };
        let w = e.part("RATIONAL_B_SPLINE_SURFACE").map(|w| w[0].list().unwrap().iter().map(Self::nums).collect::<Vec<_>>());
        Ok(self.net(b[0].number().unwrap() as usize,b[1].number().unwrap() as usize,&b[2],(Self::ints(&k[0]),Self::ints(&k[1]),Self::nums(&k[2]),Self::nums(&k[3])),w))
    }
    fn net(&self,du: usize,dv: usize,pts: &Value,(um,vm,uk,vk): (Vec<usize>,Vec<usize>,Vec<f64>,Vec<f64>),w: Option<Vec<Vec<f64>>>) -> Surf {
        let l = self.len();
        let rows: Vec<Spline> = pts.list().unwrap().iter().enumerate().map(|(i,row)| {
            let p: Vec<V> = row.list().unwrap().iter().map(|q| sc(self.raw(q.reference()),l)).collect();
            Spline::new(dv,&vm,&vk,p,w.as_ref().map(|w| w[i].clone()))
        }).collect();
        let mut uknots = Vec::new();
        for (m,k) in um.iter().zip(&uk) { for _ in 0..*m { uknots.push(*k); } }
        Surf::Net(Net {du,uk:uknots,rows})
    }
    fn curve(&self,r: u32) -> Result<Crv,String> {
        let e = self.file.get(r);
        let l = self.len();
        match e.simple() {
            Some(("LINE",v)) => {
                let (_,vec) = self.ent(v[2].reference().unwrap());
                let d = self.g.direction(0,vec[1].reference()).unwrap();
                Ok(Crv::Line(self.pt(v[1].reference()),sc(d,vec[2].number().unwrap()*l)))
            }
            Some(("CIRCLE",v)) => Ok(Crv::Circle(self.frame(v[1].reference()),v[2].number().unwrap()*l)),
            Some(("ELLIPSE",v)) => Ok(Crv::Ellipse(self.frame(v[1].reference()),v[2].number().unwrap()*l,v[3].number().unwrap()*l)),
            _ => self.spline(r,true).map(Crv::Spline).ok_or_else(|| format!("a curve of the kinds {}",e.kinds())),
        }
    }
    /// A curve in a surface's parameters, as written (angles in the file's unit).
    fn flat(&self,r: u32) -> Result<Crv,String> {
        let e = self.file.get(r);
        match e.simple() {
            Some(("LINE",v)) => {
                let p = self.raw(v[1].reference());
                let (_,vec) = self.ent(v[2].reference().unwrap());
                let d = self.raw(vec[1].reference());
                let n = (d[0]*d[0]+d[1]*d[1]).sqrt();
                Ok(Crv::Line(p,sc(d,vec[2].number().unwrap()/n)))
            }
            Some(("CIRCLE",v)) => {
                let (_,a) = self.ent(v[1].reference().unwrap());
                let o = self.raw(a[1].reference());
                let x = a.get(2).and_then(Value::reference).map_or([1.,0.,0.],|d| { let d = self.raw(Some(d)); let n = (d[0]*d[0]+d[1]*d[1]).sqrt(); [d[0]/n,d[1]/n,0.] });
                Ok(Crv::Circle(Fr {o,x,y:[-x[1],x[0],0.],z:[0.,0.,1.]},v[2].number().unwrap()))
            }
            _ => self.spline(r,false).map(Crv::Spline).ok_or_else(|| format!("a curve in parameters of the kinds {}",e.kinds())),
        }
    }
}

/// The face's loops as polygons in its parameters (radians and millimetres), `n` points an edge.
fn loops(rd: &Reader,face: u32,surface: u32,s: &Surf,n: usize,worst: &mut (f64,f64)) -> Result<Vec<Vec<[f64;2]>>,String> {
    let (_,fv) = rd.ent(face);
    let ang = rd.g.context.angle;
    let l = rd.len();
    // the file's parameters of this surface to the formulas': angles in radians, lengths in mm
    let unit: [f64;2] = match rd.ent(surface).0 {
        "PLANE" => [l,l],
        "CYLINDRICAL_SURFACE" | "CONICAL_SURFACE" => [ang,l],
        "SPHERICAL_SURFACE" | "TOROIDAL_SURFACE" => [ang,ang],
        "SURFACE_OF_LINEAR_EXTRUSION" => [1.,1.],
        _ => [1.,1.],
    };
    let periods: [Option<f64>;2] = match s {
        Surf::Cyl(..) | Surf::Cone(..) => [Some(std::f64::consts::TAU),None],
        Surf::Sphere(..) => [Some(std::f64::consts::TAU),None],
        Surf::Torus(..) => [Some(std::f64::consts::TAU);2],
        _ => [None,None],
    };
    let mut out = Vec::new();
    for b in fv[1].list().unwrap() {
        let (_,bv) = rd.ent(b.reference().unwrap());
        let along_bound = bv[2].logical().unwrap();
        let (lk,lv) = rd.ent(bv[1].reference().unwrap());
        if lk != "EDGE_LOOP" { return Err(format!("a bound of the kind {lk}")) }
        // each use: its candidate polygons (one per pcurve on this surface)
        let mut uses: Vec<Vec<Vec<[f64;2]>>> = Vec::new();
        for o in lv[1].list().unwrap() {
            let (_,ov) = rd.ent(o.reference().unwrap());
            let along = ov[4].logical().unwrap();
            let ec = ov[3].reference().unwrap();
            let (_,ev) = rd.ent(ec);
            let (a,z) = (rd.pt(rd.ent(ev[1].reference().unwrap()).1[1].reference()),rd.pt(rd.ent(ev[2].reference().unwrap()).1[1].reference()));
            let same = ev[4].logical().unwrap();
            let gc = ev[3].reference().unwrap();
            let (gk,gv) = rd.ent(gc);
            if !matches!(gk,"SURFACE_CURVE" | "SEAM_CURVE") { return Err(format!("edge #{ec}'s curve is {gk}: no curve in the face's parameters")) }
            let mut cands = Vec::new();
            for pc in gv[2].list().unwrap() {
                let (pk,pv) = rd.ent(pc.reference().unwrap());
                if pk != "PCURVE" || pv[1].reference() != Some(surface) { continue }
                let (_,dv) = rd.ent(pv[2].reference().unwrap());
                let item = dv[1].list().unwrap()[0].reference().unwrap();
                let c = rd.flat(item)?;
                let on = |t: f64| { let q = c.at(t); s.at(q[0]*unit[0],q[1]*unit[1]) };
                // the stretch: the pcurve's parameters nearest the edge's vertices in space
                let range: [f64;2] = match &c { Crv::Spline(sp) => sp.domain(),Crv::Circle(..) | Crv::Ellipse(..) => [0.,std::f64::consts::TAU],
                    Crv::Line(..) => { let t = 4.*(1.+(0..3).map(|k| a[k].abs().max(z[k].abs())).fold(0.,f64::max)/l); [-t,t] } };
                // (of the points of the pcurve nearest, the one at its least parameter in size: a
                // line in a periodic surface's parameters passes each point once a period)
                let near = |p: V,lo: f64,hi: f64| -> f64 {
                    let m = 20000;
                    let ts: Vec<f64> = (0..=m).map(|j| lo+(hi-lo)*j as f64/m as f64).collect();
                    let ds: Vec<f64> = ts.iter().map(|&t| dist(on(t),p)).collect();
                    let refine = |x0: f64| { let (mut x,mut h) = (x0,(hi-lo)/m as f64);
                        for _ in 0..300 {
                            let (dl,dr,d0) = (dist(on(x-h),p),dist(on(x+h),p),dist(on(x),p));
                            if dl < d0 { x -= h } else if dr < d0 { x += h } else { h *= 0.5 }
                            if h < 1e-15*(1.+x.abs()) { break }
                        }
                        (dist(on(x),p),x) };
                    let found: Vec<(f64,f64)> = (0..=m).filter(|&j| (j == 0 || ds[j] <= ds[j-1]) && (j == m || ds[j] <= ds[j+1])).map(|j| refine(ts[j])).collect();
                    let best = found.iter().map(|x| x.0).fold(f64::INFINITY,f64::min);
                    found.iter().filter(|x| x.0 <= best+1e-9*(1.+norm(p))).map(|x| x.1).min_by(|a,b| a.abs().total_cmp(&b.abs())).unwrap_or(lo)
                };
                // the edge from its start `a` to its end `z`: along its curve where `same`, a
                // circle's way round and a closed edge's whole turn read so
                let tau = std::f64::consts::TAU;
                let closed = dist(a,z) < 1e-9*(1.+norm(a));
                let (mut ta,mut tz) = (near(a,range[0],range[1]),near(z,range[0],range[1]));
                match (&c,closed) {
                    (Crv::Circle(..) | Crv::Ellipse(..),_) => if same { while tz <= ta+1e-12 { tz += tau; } } else { while tz >= ta-1e-12 { tz -= tau; } },
                    // (a line closing round its surface's period: a whole period along it)
                    (Crv::Line(_,d),true) => {
                        let k = if d[0].abs() >= d[1].abs() { 0 } else { 1 };
                        let Some(p) = periods[k] else { return Err(format!("edge #{ec}: a closed edge along a line in parameters that do not close")) };
                        let dt = p/unit[k]/d[k].abs();
                        tz = if same { ta+dt } else { ta-dt };
                    }
                    (_,true) => if same { (ta,tz) = (range[0],range[1]) } else { (ta,tz) = (range[1],range[0]) },
                    // (a line across a period: its end the first after its start the way its edge runs)
                    (Crv::Line(_,d),false) => {
                        let k = if d[0].abs() >= d[1].abs() { 0 } else { 1 };
                        if let Some(p) = periods[k].filter(|_| d[k].abs() > 1e-9) {
                            let dt = p/unit[k]/d[k].abs();
                            if same { while tz <= ta { tz += dt; } while tz-dt > ta { tz -= dt; } }
                            else { while tz >= ta { tz -= dt; } while tz+dt < ta { tz += dt; } }
                        }
                    }
                    _ => {}
                }
                let (ta,tb) = if along { (ta,tz) } else { (tz,ta) };
                let pts: Vec<[f64;2]> = (0..=n).map(|j| { let q = c.at(ta+(tb-ta)*j as f64/n as f64); [q[0]*unit[0],q[1]*unit[1]] }).collect();
                // how far the pcurve's image stands from the edge's curve in space (a polyline of
                // it between its vertices), and its ends from the vertices
                let off = match rd.curve(gv[1].reference().unwrap()) {
                    Ok(k) => {
                        let at3 = |t: f64| k.at(t);
                        let range3: [f64;2] = match &k { Crv::Spline(sp) => sp.domain(),Crv::Circle(..) | Crv::Ellipse(..) => [0.,tau],
                            Crv::Line(..) => { let t = 4.*(1.+(0..3).map(|i| a[i].abs().max(z[i].abs())).fold(0.,f64::max)); [-t,t] } };
                        let near3 = |p: V| -> f64 { let m = 20000; (0..=m).map(|j| range3[0]+(range3[1]-range3[0])*j as f64/m as f64)
                            .min_by(|x,y| dist(at3(*x),p).total_cmp(&dist(at3(*y),p))).unwrap() };
                        let (mut sa,mut sz) = (near3(a),near3(z));
                        match (&k,closed) {
                            (Crv::Circle(..) | Crv::Ellipse(..),_) => if same { while sz <= sa+1e-12 { sz += tau; } } else { while sz >= sa-1e-12 { sz -= tau; } },
                            (_,true) => if same { (sa,sz) = (range3[0],range3[1]) } else { (sa,sz) = (range3[1],range3[0]) },
                            _ => {}
                        }
                        let poly: Vec<V> = (0..=4000).map(|j| at3(sa+(sz-sa)*j as f64/4000.)).collect();
                        let seg = |p: V,a: V,b: V| { let d = sub(b,a); let l = d[0]*d[0]+d[1]*d[1]+d[2]*d[2];
                            let t = if l > 0. { ((p[0]-a[0])*d[0]+(p[1]-a[1])*d[1]+(p[2]-a[2])*d[2])/l } else { 0. }.clamp(0.,1.); dist(p,add(a,sc(d,t))) };
                        (0..=64).map(|j| { let q = pts[j*n/64]; let p = s.at(q[0],q[1]); poly.windows(2).map(|w| seg(p,w[0],w[1])).fold(f64::INFINITY,f64::min) }).fold(0.,f64::max)
                    }
                    Err(_) => f64::NAN,
                };
                let ends = { let (p,q) = (pts[0],pts[n]); let (x,y) = if along { (a,z) } else { (z,a) }; dist(s.at(p[0],p[1]),x).max(dist(s.at(q[0],q[1]),y)) };
                if std::env::var("FQ_DEBUG").is_ok() { eprintln!("  edge #{ec} along {along} same {same}: {:?} -> {:?}, image off its curve {off:.2e}, ends off their vertices {ends:.2e}",pts[0],pts[n]); }
                worst.0 = worst.0.max(off);
                worst.1 = worst.1.max(ends);
                cands.push(pts);
            }
            if cands.is_empty() { return Err(format!("edge #{ec} has no curve in the face's parameters")) }
            uses.push(cands);
        }
        if !along_bound { uses.reverse(); for u in &mut uses { for c in u.iter_mut() { c.reverse(); } } }
        // walk: each use moved by whole periods to start where the last ended, the candidate
        // nearest; tried from each candidate of the first, the loop closing best kept
        let mut best: Option<(f64,Vec<[f64;2]>)> = None;
        for first in 0..uses[0].len() {
            let mut poly: Vec<[f64;2]> = uses[0][first].clone();
            for u in &uses[1..] {
                let last = *poly.last().unwrap();
                let pick = u.iter().map(|c| {
                    let d: [f64;2] = std::array::from_fn(|k| periods[k].map_or(0.,|p| ((last[k]-c[0][k])/p).round()*p));
                    let c: Vec<[f64;2]> = c.iter().map(|q| [q[0]+d[0],q[1]+d[1]]).collect();
                    ((c[0][0]-last[0]).hypot(c[0][1]-last[1]),c)
                }).min_by(|x,y| x.0.total_cmp(&y.0)).unwrap();
                poly.extend(pick.1);
            }
            let gap = (poly[0][0]-poly.last().unwrap()[0]).hypot(poly[0][1]-poly.last().unwrap()[1]);
            if best.as_ref().is_none_or(|b| gap < b.0) { best = Some((gap,poly)); }
        }
        let (gap,poly) = best.unwrap();
        if gap > 1e-3 { eprintln!("  (a loop of face #{face} closes {gap:e} apart in its parameters)"); }
        out.push(poly);
    }
    Ok(out)
}

/// `∬` of the area's and the flux's integrands over the region the polygons wind round, by
/// `strips` columns across `u` of `GAUSS` nodes each.
fn columns(s: &Surf,polys: &[Vec<[f64;2]>],strips: usize) -> (f64,f64) {
    const G: [(f64,f64);8] = [(-0.9602898564975363,0.1012285362903763),(-0.7966664774136267,0.2223810344533745),
        (-0.5255324099163290,0.3137066458778873),(-0.1834346424956498,0.3626837833783620),(0.1834346424956498,0.3626837833783620),
        (0.5255324099163290,0.3137066458778873),(0.7966664774136267,0.2223810344533745),(0.9602898564975363,0.1012285362903763)];
    let (mut lo,mut hi) = ([f64::INFINITY;2],[f64::NEG_INFINITY;2]);
    for q in polys.iter().flatten() { for k in 0..2 { lo[k] = lo[k].min(q[k]); hi[k] = hi[k].max(q[k]); } }
    let h = [1e-6*(hi[0]-lo[0]).max(1e-9),1e-6*(hi[1]-lo[1]).max(1e-9)];
    // the strips cut where a polygon turns in `u` or runs along a column (there each column's
    // stretches start or stop, and the integrand across `u` is no smooth function), each stretch
    // between cuts in strips as many as its share of the span
    let mut cuts = vec![lo[0],hi[0]];
    for p in polys {
        let n = p.len();
        for i in 0..n {
            let (a,b,c) = (p[(i+n-1)%n],p[i],p[(i+1)%n]);
            if (b[0]-a[0])*(c[0]-b[0]) <= 0. { cuts.push(b[0]); }
        }
    }
    cuts.sort_by(f64::total_cmp);
    cuts.dedup_by(|x,y| (*x-*y).abs() <= 1e-12*(hi[0]-lo[0]));
    let mut pieces: Vec<(f64,f64)> = Vec::new();
    for c in cuts.windows(2) {
        let k = (((c[1]-c[0])/(hi[0]-lo[0])*strips as f64).ceil() as usize).max(1);
        for j in 0..k { pieces.push((c[0]+(c[1]-c[0])*j as f64/k as f64,(c[1]-c[0])/k as f64)); }
    }
    let (mut area,mut flux) = (0.,0.);
    for (start,w) in pieces {
        for &(x,wx) in &G {
            let u = start+w*(0.5+0.5*x);
            // crossings with their winding: +1 where the polygon crosses `u` rightward
            let mut cs: Vec<(f64,i32)> = Vec::new();
            for p in polys {
                for e in p.windows(2).chain(std::iter::once(&[*p.last().unwrap(),p[0]][..])) {
                    let (a,b) = (e[0],e[1]);
                    if (a[0] <= u) != (b[0] <= u) {
                        let t = (u-a[0])/(b[0]-a[0]);
                        cs.push((a[1]+t*(b[1]-a[1]),if b[0] > a[0] { 1 } else { -1 }));
                    }
                }
            }
            cs.sort_by(|a,b| a.0.total_cmp(&b.0));
            let mut wind = 0;
            for k in 0..cs.len() {
                wind += cs[k].1;
                if wind == 0 || k+1 == cs.len() { continue }
                let (v0,v1) = (cs[k].0,cs[k+1].0);
                // along v: eight nodes on each of sixteen pieces
                let m = 16;
                for j in 0..m {
                    let (a,b) = (v0+(v1-v0)*j as f64/m as f64,v0+(v1-v0)*(j+1) as f64/m as f64);
                    for &(y,wy) in &G {
                        let v = 0.5*(a+b)+0.5*(b-a)*y;
                        let (p,n) = s.d(u,v,h);
                        let ww = wx*0.5*w*wy*0.5*(b-a);
                        area += ww*norm(n);
                        flux += ww*(p[0]*n[0]+p[1]*n[1]+p[2]*n[2])/3.*(wind.signum() as f64);
                    }
                }
            }
        }
    }
    (area,flux)
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let text = std::fs::read_to_string(&args[0]).expect("a file");
    let file = context::parse(&text).expect("a STEP file");
    let found = context::solids(&file).expect("a solid");
    let rd = Reader {g:Geometry {file:&file,context:found[0].context},file:&file};
    for a in &args[1..] {
        let face: u32 = a.trim_start_matches('#').parse().expect("an entity number");
        let (_,fv) = rd.ent(face);
        let surface = fv[2].reference().unwrap();
        let s = match rd.surface(surface) { Ok(s) => s,Err(e) => { println!("#{face}\tnot measured: {e}"); continue } };
        let mut worst = (0f64,0f64);
        let mut read = |n: usize,strips: usize| -> Result<(f64,f64),String> { Ok(columns(&s,&loops(&rd,face,surface,&s,n,&mut worst)?,strips)) };
        // (`FQ_STRIPS` columns across `u`, the polygons `FQ_POINTS` an edge: the estimate is the
        // change from half of each)
        let strips: usize = std::env::var("FQ_STRIPS").ok().and_then(|x| x.parse().ok()).unwrap_or(400);
        let points: usize = std::env::var("FQ_POINTS").ok().and_then(|x| x.parse().ok()).unwrap_or(2000);
        match (read(points,strips),read(points/2,strips/2)) {
            (Ok((a,f)),Ok((a2,f2))) => {
                // (the sign of the flux by the loops' winding as written: the face's own side)
                println!("#{face}\t{}\tarea {a:.12e}\t± {:.1e}\tflux/3 {f:.12e}\t± {:.1e}\tpcurves off their edges {:.1e} mm, ends off vertices {:.1e} mm",
                    rd.ent(surface).0,(a-a2).abs(),(f-f2).abs(),worst.0,worst.1);
            }
            (Err(e),_) | (_,Err(e)) => println!("#{face}\tnot measured: {e}"),
        }
    }
}
