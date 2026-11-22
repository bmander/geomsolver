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
//!     face_quadrature [--write CORPUS] [--master] FILE ENTITY…
//!
//! With `--master`, each edge bounds the face by its master representation (ISO 10303-42
//! `surface_curve.master_representation`) rather than by this face's pcurve: where the master is
//! the 3D curve, or a pcurve on another surface, each point of the use's polygon is moved to the
//! foot on this face's surface of the point of the master's image nearest it (sought from the
//! pcurve's own point; the master's image in space sampled at 20,000 points and refined); where it
//! is this face's pcurve, the pcurve as it is. A `--master` reading is the region the file says the
//! face is, and is never written (`--write` takes the pcurves' region alone).
//!
//! With `--write`, each face measured to a part in 1e5 of its area, its file's curves in its
//! parameters within a thousandth of its size (the square root of its area) of its edges, is put
//! in `CORPUS/oracle/<FILE's path under CORPUS>.quad.faces.tsv` — the measurer's columns, its first
//! line `#` and the SHA-256 of the file from its `DATA;` on — beside the faces already there (the
//! corpus gate's third reader).
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

enum Surf { Plane(Fr),Cyl(Fr,f64),Cone(Fr,f64,f64),Sphere(Fr,f64),Torus(Fr,f64,f64),Extr(Box<Crv>,V),Rev(Box<Crv>,V,V),Net(Net) }
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
            // (the curve's point at `v` turned by `u` about the axis through `o` along `z`: Rodrigues)
            Surf::Rev(c,o,z) => {
                let p = sub(c.at(v),*o);
                let along = sc(*z,p[0]*z[0]+p[1]*z[1]+p[2]*z[2]);
                let r = sub(p,along);
                add(add(*o,along),add(sc(r,u.cos()),sc(cross(*z,r),u.sin())))
            }
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
                "SURFACE_OF_REVOLUTION" => {
                    let c = self.curve(v[1].reference().unwrap())?;
                    let (_,ax) = self.ent(v[2].reference().unwrap());
                    let o = self.pt(ax[1].reference());
                    let z = ax.get(2).and_then(Value::reference).map_or([0.,0.,1.],|d| self.g.direction(0,Some(d)).unwrap());
                    Surf::Rev(Box::new(c),o,z)
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
fn loops(rd: &Reader,face: u32,surface: u32,s: &Surf,n: usize,worst: &mut (f64,f64),master: bool) -> Result<Vec<Vec<[f64;2]>>,String> {
    let (_,fv) = rd.ent(face);
    let ang = rd.g.context.angle;
    let l = rd.len();
    // the file's parameters of this surface to the formulas': angles in radians, lengths in mm
    let unit: [f64;2] = match rd.ent(surface).0 {
        "PLANE" => [l,l],
        "CYLINDRICAL_SURFACE" | "CONICAL_SURFACE" => [ang,l],
        "SPHERICAL_SURFACE" | "TOROIDAL_SURFACE" => [ang,ang],
        "SURFACE_OF_REVOLUTION" => [ang,1.],
        "SURFACE_OF_LINEAR_EXTRUSION" => [1.,1.],
        _ => [1.,1.],
    };
    let periods: [Option<f64>;2] = match s {
        Surf::Cyl(..) | Surf::Cone(..) => [Some(std::f64::consts::TAU),None],
        Surf::Sphere(..) => [Some(std::f64::consts::TAU),None],
        Surf::Torus(..) => [Some(std::f64::consts::TAU);2],
        Surf::Rev(..) => [Some(std::f64::consts::TAU),None],
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
            // (the master: none for the 3D curve, else the place of its pcurve)
            let named = match gv.get(3).and_then(Value::word) { Some("PCURVE_S1") => Some(0),Some("PCURVE_S2") => Some(1),_ => None };
            for (at,pc) in gv[2].list().unwrap().iter().enumerate() {
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
                        // (the nearest of 20,000 samples, refined: a sample's own spacing, a
                        // part in 20,000 of a long curve's domain, cut the polyline short of its
                        // vertex and read a pcurve exact to 1e-13 as 5e-4 off its edge)
                        let near3 = |p: V| -> f64 {
                            let (m,h0) = (20000,(range3[1]-range3[0])/20000.);
                            let mut x = (0..=m).map(|j| range3[0]+h0*j as f64).min_by(|x,y| dist(at3(*x),p).total_cmp(&dist(at3(*y),p))).unwrap();
                            let mut h = h0;
                            for _ in 0..300 {
                                let (dl,dr,d0) = (dist(at3(x-h),p),dist(at3(x+h),p),dist(at3(x),p));
                                if dl < d0 && x-h >= range3[0] { x -= h } else if dr < d0 && x+h <= range3[1] { x += h } else { h *= 0.5 }
                                if h < 1e-15*(1.+x.abs()) { break }
                            }
                            x
                        };
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
                let pts = if master && named != Some(at) { mastered(rd,gv,named,s,pts,a,z)? } else { pts };
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

/// **A use's polygon moved onto its master** (`--master`): each point `q` of `pts` (on `s`) to
/// the foot on `s`, sought from `q` by Gauss–Newton, of the point of the master's image nearest
/// `s(q)` — the 3D curve where `named` is none, else the image on its own surface of the pcurve
/// in the place `named` of the surface curve `gv` (`a`, `z` the edge's vertices, bounding a line).
fn mastered(rd: &Reader,gv: &[Value],named: Option<usize>,s: &Surf,pts: Vec<[f64;2]>,a: V,z: V) -> Result<Vec<[f64;2]>,String> {
    let tau = std::f64::consts::TAU;
    let span = 4.*(1.+(0..3).map(|k| a[k].abs().max(z[k].abs())).fold(0.,f64::max));
    // the master's image in space and its parameter's range
    let (image,range): (Box<dyn Fn(f64) -> V>,[f64;2]) = match named {
        None => {
            let k = rd.curve(gv[1].reference().unwrap())?;
            let r = match &k { Crv::Spline(sp) => sp.domain(),Crv::Circle(..) | Crv::Ellipse(..) => [0.,tau],Crv::Line(..) => [-span,span] };
            (Box::new(move |t| k.at(t)),r)
        }
        Some(i) => {
            let (_,pv) = rd.ent(gv[2].list().unwrap()[i].reference().unwrap());
            let other = pv[1].reference().unwrap();
            let o = rd.surface(other)?;
            let (_,dv) = rd.ent(pv[2].reference().unwrap());
            let c = rd.flat(dv[1].list().unwrap()[0].reference().unwrap())?;
            let (ang,l) = (rd.g.context.angle,rd.len());
            let unit: [f64;2] = match rd.ent(other).0 {
                "PLANE" => [l,l],
                "CYLINDRICAL_SURFACE" | "CONICAL_SURFACE" => [ang,l],
                "SPHERICAL_SURFACE" | "TOROIDAL_SURFACE" => [ang,ang],
                "SURFACE_OF_REVOLUTION" => [ang,1.],
                _ => [1.,1.],
            };
            let r = match &c { Crv::Spline(sp) => sp.domain(),Crv::Circle(..) | Crv::Ellipse(..) => [0.,tau],Crv::Line(..) => [-span/l,span/l] };
            (Box::new(move |t| { let q = c.at(t); o.at(q[0]*unit[0],q[1]*unit[1]) }),r)
        }
    };
    let m = 20000;
    let samples: Vec<(f64,V)> = (0..=m).map(|j| { let t = range[0]+(range[1]-range[0])*j as f64/m as f64; (t,image(t)) }).collect();
    let nearest = |p: V| -> V {
        let (mut x,_) = samples.iter().map(|&(t,q)| (t,dist(q,p))).min_by(|a,b| a.1.total_cmp(&b.1)).unwrap();
        let mut h = (range[1]-range[0])/m as f64;
        for _ in 0..300 {
            let (dl,dr,d0) = (dist(image(x-h),p),dist(image(x+h),p),dist(image(x),p));
            if dl < d0 { x -= h } else if dr < d0 { x += h } else { h *= 0.5 }
            if h < 1e-15*(1.+x.abs()) { break }
        }
        image(x)
    };
    Ok(pts.into_iter().map(|q| {
        let target = nearest(s.at(q[0],q[1]));
        // (the foot: Gauss–Newton on |S(u, v) − target|², derivatives by central differences)
        let mut uv = q;
        for _ in 0..30 {
            let h = [1e-7*(1.+uv[0].abs()),1e-7*(1.+uv[1].abs())];
            let p = s.at(uv[0],uv[1]);
            let su = sc(sub(s.at(uv[0]+h[0],uv[1]),s.at(uv[0]-h[0],uv[1])),0.5/h[0]);
            let sv = sc(sub(s.at(uv[0],uv[1]+h[1]),s.at(uv[0],uv[1]-h[1])),0.5/h[1]);
            let r = sub(target,p);
            let dot = |a: V,b: V| a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
            let (a11,a12,a22,b1,b2) = (dot(su,su),dot(su,sv),dot(sv,sv),dot(su,r),dot(sv,r));
            let det = a11*a22-a12*a12;
            if !(det.abs() > 0.) { break }
            let d = [(b1*a22-b2*a12)/det,(a11*b2-a12*b1)/det];
            uv = [uv[0]+d[0],uv[1]+d[1]];
            if d[0].abs() <= 1e-14*(1.+uv[0].abs()) && d[1].abs() <= 1e-14*(1.+uv[1].abs()) { break }
        }
        uv
    }).collect())
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

/// SHA-256 of `data` (FIPS 180-4), as hex.
fn sha256(data: &[u8]) -> String {
    const K: [u32;64] = [0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
        0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,0xe49b69c1,0xefbe4786,
        0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,
        0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,
        0x81c2c92e,0x92722c85,0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
        0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,0x748f82ee,0x78a5636f,
        0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2];
    let mut h: [u32;8] = [0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19];
    let mut m = data.to_vec();
    m.push(0x80);
    while m.len()%64 != 56 { m.push(0); }
    m.extend_from_slice(&((data.len() as u64)*8).to_be_bytes());
    for c in m.chunks(64) {
        let mut w = [0u32;64];
        for i in 0..16 { w[i] = u32::from_be_bytes([c[4*i],c[4*i+1],c[4*i+2],c[4*i+3]]); }
        for i in 16..64 {
            let s0 = w[i-15].rotate_right(7)^w[i-15].rotate_right(18)^(w[i-15]>>3);
            let s1 = w[i-2].rotate_right(17)^w[i-2].rotate_right(19)^(w[i-2]>>10);
            w[i] = w[i-16].wrapping_add(s0).wrapping_add(w[i-7]).wrapping_add(s1);
        }
        let mut v = h;
        for i in 0..64 {
            let s1 = v[4].rotate_right(6)^v[4].rotate_right(11)^v[4].rotate_right(25);
            let ch = (v[4]&v[5])^(!v[4]&v[6]);
            let t1 = v[7].wrapping_add(s1).wrapping_add(ch).wrapping_add(K[i]).wrapping_add(w[i]);
            let s0 = v[0].rotate_right(2)^v[0].rotate_right(13)^v[0].rotate_right(22);
            let mj = (v[0]&v[1])^(v[0]&v[2])^(v[1]&v[2]);
            let t2 = s0.wrapping_add(mj);
            v = [t1.wrapping_add(t2),v[0],v[1],v[2],v[3].wrapping_add(t1),v[4],v[5],v[6]];
        }
        for i in 0..8 { h[i] = h[i].wrapping_add(v[i]); }
    }
    h.iter().map(|x| format!("{x:08x}")).collect()
}

fn main() {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    let write = if args.first().map(String::as_str) == Some("--write") { args.remove(0); Some(args.remove(0)) } else { None };
    let master = args.first().map(String::as_str) == Some("--master");
    if master { args.remove(0); }
    let write = write.filter(|_| !master);
    let mut written: Vec<(u32,f64)> = Vec::new();
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
        let mut read = |n: usize,strips: usize| -> Result<(f64,f64),String> { Ok(columns(&s,&loops(&rd,face,surface,&s,n,&mut worst,master)?,strips)) };
        // (`FQ_STRIPS` columns across `u`, the polygons `FQ_POINTS` an edge: the estimate is the
        // change from half of each)
        let strips: usize = std::env::var("FQ_STRIPS").ok().and_then(|x| x.parse().ok()).unwrap_or(400);
        let points: usize = std::env::var("FQ_POINTS").ok().and_then(|x| x.parse().ok()).unwrap_or(2000);
        match (read(points,strips),read(points/2,strips/2)) {
            (Ok((a,f)),Ok((a2,f2))) => {
                // (the sign of the flux by the loops' winding as written: the face's own side)
                println!("#{face}\t{}\tarea {a:.12e}\t± {:.1e}\tflux/3 {f:.12e}\t± {:.1e}\tpcurves off their edges {:.1e} mm, ends off vertices {:.1e} mm",
                    rd.ent(surface).0,(a-a2).abs(),(f-f2).abs(),worst.0,worst.1);
                if (a-a2).abs() <= 1e-5*a && worst.0 <= 1e-3*a.sqrt() && worst.1 <= 1e-3*a.sqrt() { written.push((face,a)); }
            }
            (Err(e),_) | (_,Err(e)) => println!("#{face}\tnot measured: {e}"),
        }
    }
    let Some(root) = write else { return };
    let path = std::fs::canonicalize(&args[0]).expect("the file");
    let root = std::fs::canonicalize(&root).expect("the corpus");
    let rel = path.strip_prefix(&root).expect("a file under the corpus");
    let out = root.join("oracle").join(format!("{}.quad.faces.tsv",rel.display()));
    let raw = std::fs::read(&path).expect("the file");
    let at = raw.windows(6).position(|w| w == b"\nDATA;").unwrap_or(0);
    let sha = sha256(&raw[at..]);
    // (the faces already there, of these same bytes, kept beside the new)
    let mut faces: std::collections::BTreeMap<u32,f64> = std::collections::BTreeMap::new();
    if let Ok(old) = std::fs::read_to_string(&out) {
        if old.lines().next() == Some(&format!("#\t{sha}")) {
            for l in old.lines().skip(2) { let mut x = l.split('\t'); if let (Some(e),Some(a)) = (x.next(),x.next()) { if let (Ok(e),Ok(a)) = (e.parse(),a.parse()) { faces.insert(e,a); } } }
        }
    }
    for (e,a) in written { faces.insert(e,a); }
    let body: String = faces.iter().map(|(e,a)| format!("{e}\t{a:?}\n")).collect();
    std::fs::write(&out,format!("#\t{sha}\nentity\tarea\n{body}")).expect("written");
    eprintln!("{}: {} faces",out.display(),faces.len());
}
