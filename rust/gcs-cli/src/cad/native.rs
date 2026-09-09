use gcs_core::json::Json;
use std::{collections::BTreeMap,ffi::{c_char,c_int,c_void,CStr,CString},path::Path,
    sync::atomic::{AtomicU64,Ordering}};

// Candidate construction is exercised independently until closed sweep assembly
// connects it to export; candidates must never pass as completed sweep solids.
#[allow(dead_code)]
#[path="native/sweep.rs"]
mod sweep;
#[allow(dead_code)]
#[path="native/trims.rs"]
mod trims;
#[allow(dead_code)]
#[path="native/traces.rs"]
mod traces;
#[allow(dead_code)]
#[path="native/cells.rs"]
pub(crate) mod cells;
#[path="native/sweep_boundary.rs"]
pub(crate) mod sweep_boundary;

extern "C" {
    fn solvent_cad_new() -> *mut c_void;
    fn solvent_cad_free(cad: *mut c_void);
    fn solvent_cad_error(cad: *mut c_void) -> *const c_char;
    fn solvent_cad_line(cad: *mut c_void,a: *const f64,b: *const f64) -> c_int;
    fn solvent_cad_circle(cad: *mut c_void,c: *const f64,n: *const f64,x: *const f64,
        radius: f64,start: f64,end: f64) -> c_int;
    fn solvent_cad_face(cad: *mut c_void,edges: *const c_int,count: c_int) -> c_int;
    fn solvent_cad_prism(cad: *mut c_void,face: c_int,at: *const f64,by: *const f64) -> c_int;
    fn solvent_cad_revolve(cad: *mut c_void,face: c_int,at: *const f64,axis: *const f64,
        angle: f64) -> c_int;
    fn solvent_cad_boolean(cad: *mut c_void,a: c_int,b: c_int,cut: c_int) -> c_int;
    fn solvent_cad_transform(cad: *mut c_void,source: c_int,matrix: *const f64) -> c_int;
    fn solvent_cad_bounds(cad: *mut c_void,ids: *const c_int,count: c_int,out: *mut f64) -> c_int;
    fn solvent_cad_validate(cad: *mut c_void,id: c_int) -> c_int;
    fn solvent_cad_step(cad: *mut c_void,id: c_int,path: *const c_char) -> c_int;
    fn solvent_cad_stl(cad: *mut c_void,id: c_int,path: *const c_char) -> c_int;
}

// Recipes are built by the core in this process, not parsed from external JSON.
fn field<'a>(r: &'a Json,key: &str) -> &'a Json { r.get(key).expect("core CAD schema") }
fn v(r: &Json,key: &str) -> [f64;3] {
    let a = field(r,key).arr();
    std::array::from_fn(|i| a[i].as_f64())
}

pub(crate) struct Session(*mut c_void);
impl Drop for Session { fn drop(&mut self) { unsafe { solvent_cad_free(self.0); } } }
impl Session {
    pub(crate) fn new() -> Result<Self,String> {
        let context = unsafe { solvent_cad_new() };
        if context.is_null() { Err("cannot allocate native CAD session".into()) }
        else { Ok(Self(context)) }
    }
    #[cfg(test)]
    pub(crate) fn as_ptr(&self) -> *mut c_void { self.0 }
    fn result(&self,id: c_int) -> Result<c_int,String> {
        if id < 0 {
            Err(unsafe { CStr::from_ptr(solvent_cad_error(self.0)) }.to_string_lossy().into_owned())
        } else { Ok(id) }
    }
    fn boolean(&self,a: c_int,b: c_int,cut: bool) -> Result<c_int,String> {
        self.result(unsafe { solvent_cad_boolean(self.0,a,b,i32::from(cut)) })
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
            solid = Some(match solid { None => id,Some(outer) => self.boolean(outer,id,true)? });
        }
        solid.ok_or("empty CAD profile".into())
    }
    pub(crate) fn construct(&self,recipe: &Json) -> Result<c_int,String> {
        let mut shapes = BTreeMap::new();
        for node in field(recipe,"nodes").arr() {
            let make = || -> Result<c_int,String> {
                let id = if field(node,"kind").as_str() == "body" {
                    let mut id = shapes[&field(node,"stock").as_i64()];
                    for (operation,cut) in [("on",false),("cut",true)] {
                        for operand in field(node,operation).arr() {
                            id = self.boolean(id,shapes[&operand.as_i64()],cut)?;
                        }
                    }
                    id
                } else if field(node,"kind").as_str() == "placed" {
                    let matrix: Vec<_> = field(node,"matrix").arr().iter().map(Json::as_f64).collect();
                    self.result(unsafe { solvent_cad_transform(self.0,
                        shapes[&field(node,"source").as_i64()],matrix.as_ptr()) })?
                } else { self.primitive(node,&shapes)? };
                self.result(unsafe { solvent_cad_validate(self.0,id) })?;
                Ok(id)
            };
            let id = make().map_err(|e| format!("{}: {e}",field(node,"name").as_str()))?;
            shapes.insert(field(node,"id").as_i64(),id);
        }
        Ok(shapes[&field(recipe,"root").as_i64()])
    }
}

/// Build once, stage and validate every requested format, then replace outputs.
/// A geometry or encoding failure cannot leave only half the requested pair updated.
pub fn export(sk: &gcs_core::model::Sketch,solid: usize,step: Option<&str>,stl: Option<&str>) -> Result<(),String> {
    let session = Session::new()?;
    let solid = sweep_boundary::construct_solid(&session,sk,solid)?;
    let mut staged = Vec::new();
    let mut directories = Vec::new();
    let result = (|| {
        for (kind,path) in [("step",step),("stl",stl)] {
            let Some(path) = path else { continue; };
            let output = Path::new(path);
            let parent = output.parent().filter(|p| !p.as_os_str().is_empty()).unwrap_or(Path::new("."));
            let destination = parent.canonicalize().map_err(|e| e.to_string())?
                .join(output.file_name().ok_or("CAD output needs a filename")?);
            if staged.iter().any(|(_,p)| p == &destination) {
                return Err("STEP and STL need distinct output paths".into());
            }
            static NEXT: AtomicU64 = AtomicU64::new(0);
            let directory = loop {
                let d = parent.join(format!(".solvent-cad-{}-{}",std::process::id(),NEXT.fetch_add(1,Ordering::Relaxed)));
                match std::fs::create_dir(&d) {
                    Ok(()) => break d,
                    Err(e) if e.kind() == std::io::ErrorKind::AlreadyExists => continue,
                    Err(e) => return Err(format!("cannot create CAD output directory: {e}")),
                }
            };
            let temporary = directory.join(format!("solid.{kind}"));
            directories.push(directory);
            let name = CString::new(temporary.to_str().ok_or("CAD path must be UTF-8")?)
                .map_err(|e| e.to_string())?;
            session.result(unsafe {
                if kind == "step" { solvent_cad_step(session.0,solid,name.as_ptr()) }
                else { solvent_cad_stl(session.0,solid,name.as_ptr()) }
            })?;
            if kind == "stl" {
                let bytes = std::fs::read(&temporary).map_err(|e| e.to_string())?;
                gcs_core::mesh::stl_shells(&bytes)
                    .map_err(|e| format!("native float32 STL validation failed: {e}"))?;
            }
            staged.push((temporary,destination));
        }
        for (temporary,output) in &staged {
            std::fs::rename(temporary,output).map_err(|e| format!("cannot replace CAD output: {e}"))?;
        }
        Ok(())
    })();
    for directory in directories { let _ = std::fs::remove_dir_all(directory); }
    result
}
