//! The native kernel (OCCT, through `backend/`): a session constructing the core's recipes,
//! and the swept construction over it. What is written, and whether, is `cad::output`'s.
use gcs_core::json::Json;
use std::{collections::BTreeMap,ffi::{c_char,c_int,c_void,CStr,CString}};
pub(crate) use super::progress::{mark,stage};

#[path="native/kernel.rs"]
pub(crate) mod kernel;
#[path="native/sweep_boundary.rs"]
pub(crate) mod sweep_boundary;
#[path="native/features.rs"]
pub(crate) mod features;

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
    fn solvent_cad_boolean(cad: *mut c_void,a: c_int,b: c_int,operation: c_int) -> c_int;
    fn solvent_cad_transform(cad: *mut c_void,source: c_int,matrix: *const f64) -> c_int;
    fn solvent_cad_bounds(cad: *mut c_void,ids: *const c_int,count: c_int,out: *mut f64) -> c_int;
    fn solvent_cad_validate(cad: *mut c_void,id: c_int) -> c_int;
    fn solvent_cad_step(cad: *mut c_void,id: c_int,path: *const c_char) -> c_int;
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
    pub(crate) fn result(&self,id: c_int) -> Result<c_int,String> {
        if id < 0 {
            Err(unsafe { CStr::from_ptr(solvent_cad_error(self.0)) }.to_string_lossy().into_owned())
        } else { Ok(id) }
    }
    /// `operation`: the body rule's word, `on` fusing, `cut` subtracting and
    /// `bound` keeping what the two share.
    fn boolean(&self,a: c_int,b: c_int,operation: &str) -> Result<c_int,String> {
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
            solid = Some(match solid { None => id,Some(outer) => self.boolean(outer,id,"cut")? });
        }
        solid.ok_or("empty CAD profile".into())
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
