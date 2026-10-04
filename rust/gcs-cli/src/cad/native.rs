//! The native kernel (OCCT, through `backend/`): a session constructing the core's recipes,
//! and the swept construction over it. What is written, and whether, is `cad::output`'s.
use gcs_core::json::Json;
use std::{collections::BTreeMap,ffi::{c_char,c_int,c_void,CStr,CString}};
pub(crate) use super::progress::{beside,holding,mark,side_by_side,stage,under};

#[path="native/kernel.rs"]
pub(crate) mod kernel;
#[path="native/sweep_boundary.rs"]
pub(crate) mod sweep_boundary;
#[path="native/features.rs"]
pub(crate) mod features;
#[path="native/step_check.rs"]
pub(crate) mod step_check;

extern "C" {
    fn solvent_cad_new() -> *mut c_void;
    fn solvent_cad_free(cad: *mut c_void);
    fn solvent_cad_error(cad: *mut c_void) -> *const c_char;
    fn solvent_cad_line(cad: *mut c_void,a: *const f64,b: *const f64) -> c_int;
    fn solvent_cad_bspline(cad: *mut c_void,degree: c_int,count: c_int,poles: *const f64,knots: *const f64,weights: *const f64) -> c_int;
    fn solvent_cad_circle(cad: *mut c_void,c: *const f64,n: *const f64,x: *const f64,
        radius: f64,start: f64,end: f64) -> c_int;
    fn solvent_cad_face(cad: *mut c_void,edges: *const c_int,count: c_int) -> c_int;
    fn solvent_cad_prism(cad: *mut c_void,face: c_int,at: *const f64,by: *const f64) -> c_int;
    fn solvent_cad_revolve(cad: *mut c_void,face: c_int,at: *const f64,axis: *const f64,
        angle: f64) -> c_int;
    fn solvent_cad_boolean(cad: *mut c_void,a: c_int,b: c_int,operation: c_int) -> c_int;
    fn solvent_cad_transform(cad: *mut c_void,source: c_int,matrix: *const f64) -> c_int;
    fn solvent_cad_fillet(cad: *mut c_void,id: c_int,radius: f64,points: *const f64,count: c_int) -> c_int;
    fn solvent_cad_bounds(cad: *mut c_void,ids: *const c_int,count: c_int,out: *mut f64) -> c_int;
    fn solvent_cad_validate(cad: *mut c_void,id: c_int) -> c_int;
    fn solvent_cad_step(cad: *mut c_void,id: c_int,path: *const c_char,full: c_int,unchecked: c_int) -> c_int;
    fn solvent_cad_brep_summary(cad: *mut c_void,id: c_int,output: *mut f64,capacity: c_int) -> c_int;
    fn solvent_cad_brep_json(cad: *mut c_void,id: c_int) -> *const c_char;
    fn solvent_cad_stl(cad: *mut c_void,id: c_int,path: *const c_char,deflection: f64,angular: f64) -> c_int;
    fn solvent_cad_remesh(cad: *mut c_void,id: c_int,deflection: f64,angular: f64) -> c_int;
    fn solvent_cad_mesh_sag(cad: *mut c_void,id: c_int,output: *mut f64) -> c_int;
}

// Recipes are built by the core in this process, not parsed from external JSON.
fn field<'a>(r: &'a Json,key: &str) -> &'a Json { r.get(key).expect("core CAD schema") }
fn v(r: &Json,key: &str) -> [f64;3] {
    let a = field(r,key).arr();
    std::array::from_fn(|i| a[i].as_f64())
}

/// The turn by `angle` (radians) about the line through `origin` along the unit `axis`.
pub(crate) fn turned_about(origin: [f64;3],axis: [f64;3],angle: f64) -> Result<gcs_core::envelope::Motion,String> {
    use gcs_core::envelope::Motion;
    let error = |e| format!("{e:?}");
    Ok(Motion::translation(origin.map(|x| -x),[0.;3]).map_err(error)?.then(Motion::rotation(axis,angle,0.).map_err(error)?)
        .then(Motion::translation(origin,[0.;3]).map_err(error)?))
}

/// A blank built as its meridian region turned about its line (`Session::construct_meridian`): the
/// region, in the half-plane of the line towards `seam`, which the same solid may be turned from
/// again with its faces' parameters starting elsewhere.
#[derive(Clone,Copy,Debug)]
pub(crate) struct Meridian { pub region: c_int,pub origin: [f64;3],pub axis: [f64;3],pub seam: [f64;3] }

pub(crate) struct Session(*mut c_void);
// A session is called from several threads at once, each building its own shapes and reading
// shared ones: the native side guards its tables and keeps each thread's last error
// (`backend/occt.hpp`).
unsafe impl Sync for Session {}
impl Drop for Session { fn drop(&mut self) { unsafe { solvent_cad_free(self.0); } } }
impl Session {
    pub(crate) fn new() -> Result<Self,String> {
        let context = unsafe { solvent_cad_new() };
        if context.is_null() { Err("cannot allocate native CAD session".into()) }
        else { Ok(Self(context)) }
    }
    /// The session's handle, for a test crate calling the backend's C ABI directly
    /// (`tests/native_boundary.rs`); the binary's own test build has no use for it.
    #[cfg(test)]
    #[allow(dead_code)]
    pub(crate) fn as_ptr(&self) -> *mut c_void { self.0 }
    pub(crate) fn result(&self,id: c_int) -> Result<c_int,String> {
        if id < 0 {
            Err(unsafe { CStr::from_ptr(solvent_cad_error(self.0)) }.to_string_lossy().into_owned())
        } else { Ok(id) }
    }
    /// `operation`: the body rule's word, `on` fusing, `cut` subtracting and
    /// `bound` keeping what the two share.
    pub(crate) fn boolean(&self,a: c_int,b: c_int,operation: &str) -> Result<c_int,String> {
        let kind = match operation { "on" => 0,"cut" => 1,"bound" => 2,_ => return Err(format!("unknown body operation `{operation}`")) };
        self.result(unsafe { solvent_cad_boolean(self.0,a,b,kind) })
    }
    fn face(&self,edges: &Json) -> Result<c_int,String> {
        let mut ids = Vec::new();
        for edge in edges.arr() {
            let id = match field(edge,"kind").as_str() {
                "line" => unsafe { solvent_cad_line(self.0,v(edge,"start").as_ptr(),v(edge,"end").as_ptr()) },
                "circle" => {
                    let (start,end) = edge.get("angles").map(|a| (a.arr()[0].as_f64(),a.arr()[1].as_f64()))
                        .unwrap_or((0.,std::f64::consts::TAU));
                    unsafe { solvent_cad_circle(self.0,v(edge,"center").as_ptr(),
                        v(edge,"normal").as_ptr(),v(edge,"x_dir").as_ptr(),
                        field(edge,"radius").as_f64(),start,end) }
                }
                "bspline" => {
                    let poles: Vec<f64> = field(edge,"poles").arr().iter().flat_map(|p| p.arr().iter().map(Json::as_f64).collect::<Vec<_>>()).collect();
                    let knots: Vec<f64> = field(edge,"knots").arr().iter().map(Json::as_f64).collect();
                    // a rational edge's weights, one per pole; null for a polynomial one
                    let weights: Option<Vec<f64>> = edge.get("weights").map(|w| w.arr().iter().map(Json::as_f64).collect());
                    let w = weights.as_ref().map_or(std::ptr::null(),|w| w.as_ptr());
                    unsafe { solvent_cad_bspline(self.0,field(edge,"degree").as_i64() as c_int,(poles.len()/3) as c_int,poles.as_ptr(),knots.as_ptr(),w) }
                }
                _ => return Err("unsupported CAD profile edge".into()),
            };
            ids.push(self.result(id)?);
        }
        self.result(unsafe { solvent_cad_face(self.0,ids.as_ptr(),ids.len() as c_int) })
    }
    fn primitive(&self,node: &Json,shapes: &BTreeMap<i64,c_int>) -> Result<c_int,String> {
        let profile = field(node,"profile");
        let kind = field(node,"kind").as_str();
        let n = v(profile,"normal");
        let (start,end) = match kind {
            "prism" => (field(node,"from").as_f64(),field(node,"to").as_f64()),
            "through" => {
                let ids: Vec<_> = field(node,"sources").arr().iter().map(|i| shapes[&i.as_i64()]).collect();
                let mut bounds = [0.;6];
                self.result(unsafe { solvent_cad_bounds(self.0,ids.as_ptr(),ids.len() as c_int,bounds.as_mut_ptr()) })?;
                let origin = v(profile,"origin");
                let (mut lo,mut hi,mut diagonal) = (0.,0.,0.);
                for k in 0..3 {
                    let (a,b) = (n[k]*(bounds[k]-origin[k]),n[k]*(bounds[k+3]-origin[k]));
                    lo += a.min(b); hi += a.max(b);
                    diagonal += (bounds[k+3]-bounds[k]).powi(2);
                }
                let pad = diagonal.sqrt()*4e-5;
                (lo-pad,hi+pad)
            }
            "revolve" => (0.,0.),
            "loft" => return Err("native CAD export does not yet support along-guide lofts; the core's kernel builds them \
                (`--kernel rust`)".into()),
            _ => return Err("unsupported CAD primitive".into()),
        };
        let mut solid = None;
        for edges in field(profile,"loops").arr() {
            let face = self.face(edges)?;
            let id = if kind == "revolve" {
                unsafe { solvent_cad_revolve(self.0,face,v(node,"origin").as_ptr(),
                    v(node,"axis").as_ptr(),field(node,"angle").as_f64()) }
            } else {
                unsafe { solvent_cad_prism(self.0,face,n.map(|x| x*start).as_ptr(),
                    n.map(|x| x*(end-start)).as_ptr()) }
            };
            let id = self.result(id)?;
            solid = Some(match solid { None => id,Some(outer) => self.boolean(outer,id,"cut")? });
        }
        solid.ok_or("empty CAD profile".into())
    }
    /// A static recipe whose every operand is a full revolution about one line (or a ball centred on
    /// it) built as its meridian region — each operand's section in one half-plane of the line, the
    /// Booleans of the recipe taken of the sections — turned once about the line, where its Booleans
    /// in space took a Boolean of revolutions each (docs/native-speed-plan.md). None, with the reason,
    /// where the recipe is not such (a prism, a placement, a partial turn, a revolution about another
    /// line, a section crossing its axis). `SOLVENT_BLANK=booleans` builds every blank by its Booleans.
    pub(crate) fn construct_meridian(&self,recipe: &Json) -> Result<Result<(c_int,Meridian),String>,String> {
        if std::env::var("SOLVENT_BLANK").is_ok_and(|v| v == "booleans") { return Ok(Err("its Booleans asked for".into())); }
        let nodes = field(recipe,"nodes").arr();
        let unit = |a: [f64;3]| { let n = (a[0]*a[0]+a[1]*a[1]+a[2]*a[2]).sqrt(); a.map(|x| x/n) };
        let sub = |a: [f64;3],b: [f64;3]| -> [f64;3] { std::array::from_fn(|k| a[k]-b[k]) };
        let dot = |a: [f64;3],b: [f64;3]| a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
        let cross = |a: [f64;3],b: [f64;3]| -> [f64;3] { [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]] };
        let size = nodes.iter().filter(|n| field(n,"kind").as_str() == "revolve").map(|n| {
            let o = v(n,"origin"); dot(o,o).sqrt()+dot(v(n,"axis"),v(n,"axis")).sqrt()
        }).fold(1_f64,f64::max);
        let tolerance = 1e-9*size;
        // the line: the first revolution's that is not a ball
        let is_ball = |n: &Json| -> Option<[f64;3]> {
            let (o,a) = (v(n,"origin"),unit(v(n,"axis")));
            let on = |p: [f64;3]| { let d = sub(p,o); let c = cross(d,a); dot(c,c).sqrt() <= tolerance };
            let mut ball: Option<([f64;3],f64)> = None;
            for edges in field(field(n,"profile"),"loops").arr() { for e in edges.arr() {
                match field(e,"kind").as_str() {
                    "line" => if !on(v(e,"start")) || !on(v(e,"end")) { return None },
                    "circle" => {
                        let (c,r) = (v(e,"center"),field(e,"radius").as_f64());
                        if !on(c) { return None; }
                        match ball { None => ball = Some((c,r)),
                            Some((k,q)) => if dot(sub(k,c),sub(k,c)).sqrt() > tolerance || (q-r).abs() > tolerance { return None } }
                    }
                    _ => return None,
                }
            } }
            ball.map(|(c,_)| c)
        };
        let Some(line) = nodes.iter().find(|n| field(n,"kind").as_str() == "revolve" && is_ball(n).is_none()) else {
            return Ok(Err("no revolution of it is about a line of its own".into()));
        };
        let (origin,axis) = (v(line,"origin"),unit(v(line,"axis")));
        let on_line = |p: [f64;3]| { let c = cross(sub(p,origin),axis); dot(c,c).sqrt() <= tolerance };
        // the half-plane the sections are taken in: towards `seam`, square to the line
        let seed = if axis[0].abs() < 0.9 { [1.,0.,0.] } else { [0.,1.,0.] };
        let seam = unit(cross(cross(axis,seed),axis));
        let turn = |angle: f64| turned_about(origin,axis,angle);
        let mut regions: BTreeMap<i64,c_int> = BTreeMap::new();
        for node in nodes {
            let name = field(node,"name").as_str();
            let region = match field(node,"kind").as_str() {
                "revolve" => {
                    if (field(node,"angle").as_f64().abs()-std::f64::consts::TAU).abs() > 1e-9 {
                        return Ok(Err(format!("`{name}` is a partial revolution")));
                    }
                    if let Some(c) = is_ball(node) {
                        if !on_line(c) { return Ok(Err(format!("`{name}` is a ball off the line")))}
                        // its section: the half-disc about its centre in the half-plane
                        let r = field(field(field(node,"profile"),"loops").arr()[0].arr().iter()
                            .find(|e| field(e,"kind").as_str() == "circle").unwrap(),"radius").as_f64();
                        let (a,b) = (std::array::from_fn::<f64,3,_>(|k| c[k]-r*axis[k]),std::array::from_fn::<f64,3,_>(|k| c[k]+r*axis[k]));
                        let chord = self.result(unsafe { solvent_cad_line(self.0,a.as_ptr(),b.as_ptr()) })?;
                        let normal = cross(seam,axis);
                        let arc = self.result(unsafe { solvent_cad_circle(self.0,c.as_ptr(),normal.as_ptr(),seam.as_ptr(),r,
                            -std::f64::consts::FRAC_PI_2,std::f64::consts::FRAC_PI_2) })?;
                        self.result(unsafe { solvent_cad_face(self.0,[arc,chord].as_ptr(),2) })?
                    } else {
                        let (o,a) = (v(node,"origin"),unit(v(node,"axis")));
                        if dot(cross(a,axis),cross(a,axis)).sqrt() > 1e-12 || !on_line(o) {
                            return Ok(Err(format!("`{name}` is a revolution about another line")));
                        }
                        // its profile on one side of the line: the side every point of it is on
                        let profile = field(node,"profile");
                        let mut side: Option<[f64;3]> = None;
                        let mut straddles = false;
                        for edges in field(profile,"loops").arr() { for e in edges.arr() {
                            let points: Vec<[f64;3]> = match field(e,"kind").as_str() {
                                "line" => vec![v(e,"start"),v(e,"end")],
                                _ => {
                                    // a circle's extent across the line: its centre's offset, less and more its radius
                                    let (c,r) = (v(e,"center"),field(e,"radius").as_f64());
                                    let d = sub(c,origin); let d = sub(d,axis.map(|x| x*dot(d,axis)));
                                    if dot(d,d).sqrt() <= r+tolerance { straddles = true; }
                                    vec![c]
                                }
                            };
                            for p in points {
                                let d = sub(p,origin); let d = sub(d,axis.map(|x| x*dot(d,axis)));
                                let n = dot(d,d).sqrt();
                                if n <= tolerance { continue }
                                let d = d.map(|x| x/n);
                                match side { None => side = Some(d), Some(s) => if dot(s,d) < 1.-1e-9 { straddles = true } }
                            }
                        } }
                        let Some(side) = side.filter(|_| !straddles) else {
                            return Ok(Err(format!("`{name}`'s section crosses its axis")));
                        };
                        let mut face = None;
                        for edges in field(profile,"loops").arr() {
                            let loop_face = self.face(edges)?;
                            face = Some(match face { None => loop_face,Some(outer) => self.boolean(outer,loop_face,"cut")? });
                        }
                        let face = face.ok_or("empty CAD profile")?;
                        // turned about the line from its side to the half-plane's
                        let angle = dot(axis,cross(side,seam)).atan2(dot(side,seam));
                        self.place(face,turn(angle)?,1.)?
                    }
                }
                "body" => {
                    let mut id = regions[&field(node,"stock").as_i64()];
                    for operation in ["on","cut","bound"] {
                        for operand in field(node,operation).arr() { id = self.boolean(id,regions[&operand.as_i64()],operation)?; }
                    }
                    id
                }
                kind => return Ok(Err(format!("`{name}` is a {kind}, not a revolution"))),
            };
            regions.insert(field(node,"id").as_i64(),region);
        }
        let region = regions[&field(recipe,"root").as_i64()];
        let solid = self.revolve_region(region,origin,axis)?;
        Ok(Ok((solid,Meridian {region,origin,axis,seam})))
    }

    /// OCCT's rolling-ball fillet of `radius` (mm) on the edges of `id` nearest `points` (mm): the
    /// oracle a fillet's pieces are held to (`tests/fillet_oracle.rs`).
    pub(crate) fn fillet(&self,id: c_int,radius: f64,points: &[[f64;3]]) -> Result<c_int,String> {
        let flat: Vec<f64> = points.iter().flatten().copied().collect();
        self.result(unsafe { solvent_cad_fillet(self.0,id,radius,flat.as_ptr(),points.len() as c_int) })
    }

    pub(crate) fn construct(&self,recipe: &Json) -> Result<c_int,String> {
        let mut shapes = BTreeMap::new();
        for node in field(recipe,"nodes").arr() {
            let make = || -> Result<c_int,String> {
                let id = if field(node,"kind").as_str() == "body" {
                    let mut id = shapes[&field(node,"stock").as_i64()];
                    for operation in ["on","cut","bound"] {
                        for operand in field(node,operation).arr() {
                            id = self.boolean(id,shapes[&operand.as_i64()],operation)?;
                        }
                    }
                    id
                } else if field(node,"kind").as_str() == "fillet" {
                    // each edge's piece built as any primitive is, and fused
                    let mut id: Option<c_int> = None;
                    for piece in field(node,"pieces").arr() {
                        let p = self.primitive(piece,&shapes)?;
                        id = Some(match id { None => p,Some(sum) => self.boolean(sum,p,"on")? });
                    }
                    id.ok_or("a fillet with no pieces")?
                } else if field(node,"kind").as_str() == "placed" {
                    let matrix: Vec<_> = field(node,"matrix").arr().iter().map(Json::as_f64).collect();
                    self.result(unsafe { solvent_cad_transform(self.0,
                        shapes[&field(node,"source").as_i64()],matrix.as_ptr()) })?
                } else { self.primitive(node,&shapes)? };
                self.result(unsafe { solvent_cad_validate(self.0,id) })?;
                Ok(id)
            };
            let clock = std::time::Instant::now();
            let id = make().map_err(|e| format!("{}: {e}",field(node,"name").as_str()))?;
            if std::env::var_os("SOLVENT_CONSTRUCT_DEBUG").is_some() {
                eprintln!("construct: `{}` ({}) in {:?}",field(node,"name").as_str(),field(node,"kind").as_str(),clock.elapsed());
            }
            shapes.insert(field(node,"id").as_i64(),id);
        }
        Ok(shapes[&field(recipe,"root").as_i64()])
    }
}
