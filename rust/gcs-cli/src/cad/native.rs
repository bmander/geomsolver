//! The native kernel (OCCT, through `backend/`): a session constructing the core's recipes,
//! and the swept construction over it. What is written, and whether, is `cad::output`'s.
use gcs_core::brep::recipe::{Edge,Named,Node,Recipe,Stroke};
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
    fn solvent_cad_bounds(cad: *mut c_void,ids: *const c_int,count: c_int,out: *mut f64) -> c_int;
    fn solvent_cad_validate(cad: *mut c_void,id: c_int) -> c_int;
    fn solvent_cad_step(cad: *mut c_void,id: c_int,path: *const c_char,full: c_int,unchecked: c_int) -> c_int;
    fn solvent_cad_brep_summary(cad: *mut c_void,id: c_int,output: *mut f64,capacity: c_int) -> c_int;
    fn solvent_cad_brep_json(cad: *mut c_void,id: c_int) -> *const c_char;
    fn solvent_cad_stl(cad: *mut c_void,id: c_int,path: *const c_char,deflection: f64,angular: f64) -> c_int;
    fn solvent_cad_remesh(cad: *mut c_void,id: c_int,deflection: f64,angular: f64) -> c_int;
    fn solvent_cad_mesh_sag(cad: *mut c_void,id: c_int,output: *mut f64) -> c_int;
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
    fn boolean(&self,a: c_int,b: c_int,operation: &str) -> Result<c_int,String> {
        let kind = match operation { "on" => 0,"cut" => 1,"bound" => 2,_ => return Err(format!("unknown body operation `{operation}`")) };
        self.result(unsafe { solvent_cad_boolean(self.0,a,b,kind) })
    }
    fn face(&self,edges: &[Edge]) -> Result<c_int,String> {
        let mut ids = Vec::new();
        for edge in edges {
            let id = match &edge.shape {
                Stroke::Line {start,end} => unsafe { solvent_cad_line(self.0,start.as_ptr(),end.as_ptr()) },
                Stroke::Circle {center,normal,x_dir,radius,angles} => {
                    let [start,end] = angles.unwrap_or([0.,std::f64::consts::TAU]);
                    unsafe { solvent_cad_circle(self.0,center.as_ptr(),normal.as_ptr(),x_dir.as_ptr(),*radius,start,end) }
                }
                Stroke::BSpline {degree,knots,poles,weights} => {
                    let poles: Vec<f64> = poles.iter().flatten().copied().collect();
                    // a rational edge's weights, one per pole; null for a polynomial one
                    let w = weights.as_ref().map_or(std::ptr::null(),|w| w.as_ptr());
                    unsafe { solvent_cad_bspline(self.0,*degree as c_int,(poles.len()/3) as c_int,poles.as_ptr(),knots.as_ptr(),w) }
                }
            };
            ids.push(self.result(id)?);
        }
        self.result(unsafe { solvent_cad_face(self.0,ids.as_ptr(),ids.len() as c_int) })
    }
    fn primitive(&self,node: &Node,shapes: &BTreeMap<i64,c_int>) -> Result<c_int,String> {
        let (profile,start,end) = match node {
            Node::Canal {..} => return Err("native CAD export does not build a ball's tube (a ball swept by a turn or a screw); \
                the core's kernel builds it (`--kernel rust`), held to the sweep's field".into()),
            Node::Lowered(_) => return Err("native CAD export does not build a lowered round or chamfer; the core's kernel builds \
                them (`--kernel rust`), and `tests/fillet_oracle.rs` holds them to OCCT's own fillet".into()),
            Node::Family(_) => return Err("native CAD export does not build a face family; the core's kernel builds its \
                walls exactly (`--kernel rust`), and OCCT's lofts are its oracle only later (#243 L10)".into()),
            Node::Prism {draft:Some(_),..} => return Err("native CAD export does not build a drafted prism; the core's kernel \
                builds it (`--kernel rust`)".into()),
            Node::Prism {profile,from,to,..} => (profile,*from,*to),
            Node::Through {profile,sources} => {
                let ids: Vec<_> = sources.iter().map(|i| shapes[i]).collect();
                let mut bounds = [0.;6];
                self.result(unsafe { solvent_cad_bounds(self.0,ids.as_ptr(),ids.len() as c_int,bounds.as_mut_ptr()) })?;
                let (n,origin) = (profile.normal,profile.origin);
                let (mut lo,mut hi,mut diagonal) = (0.,0.,0.);
                for k in 0..3 {
                    let (a,b) = (n[k]*(bounds[k]-origin[k]),n[k]*(bounds[k+3]-origin[k]));
                    lo += a.min(b); hi += a.max(b);
                    diagonal += (bounds[k+3]-bounds[k]).powi(2);
                }
                let pad = diagonal.sqrt()*4e-5;
                (profile,lo-pad,hi+pad)
            }
            Node::Revolve {profile,..} => (profile,0.,0.),
            Node::Loft {..} => return Err("native CAD export does not yet support along-guide lofts; the core's kernel builds them \
                (`--kernel rust`)".into()),
            _ => return Err("unsupported CAD primitive".into()),
        };
        let n = profile.normal;
        let mut solid = None;
        for edges in &profile.loops {
            let face = self.face(edges)?;
            let id = if let Node::Revolve {origin,axis,angle,..} = node {
                unsafe { solvent_cad_revolve(self.0,face,origin.as_ptr(),axis.as_ptr(),*angle) }
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
        self.built_meridian(&Recipe::read(recipe)?)
    }
    /// `construct_meridian` of a recipe read.
    pub(crate) fn built_meridian(&self,recipe: &Recipe) -> Result<Result<(c_int,Meridian),String>,String> {
        if std::env::var("SOLVENT_BLANK").is_ok_and(|v| v == "booleans") { return Ok(Err("its Booleans asked for".into())); }
        let nodes = &recipe.nodes;
        let unit = |a: [f64;3]| { let n = (a[0]*a[0]+a[1]*a[1]+a[2]*a[2]).sqrt(); a.map(|x| x/n) };
        let sub = |a: [f64;3],b: [f64;3]| -> [f64;3] { std::array::from_fn(|k| a[k]-b[k]) };
        let dot = |a: [f64;3],b: [f64;3]| a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
        let cross = |a: [f64;3],b: [f64;3]| -> [f64;3] { [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]] };
        let size = nodes.iter().filter_map(|n| match n.node { Node::Revolve {origin,axis,..} => Some((origin,axis)),_ => None })
            .map(|(o,a)| dot(o,o).sqrt()+dot(a,a).sqrt()).fold(1_f64,f64::max);
        let tolerance = 1e-9*size;
        // the line: the first revolution's that is not a ball
        let is_ball = |n: &Node| -> Option<[f64;3]> {
            let Node::Revolve {profile,origin,axis,..} = n else { return None };
            let (o,a) = (*origin,unit(*axis));
            let on = |p: [f64;3]| { let d = sub(p,o); let c = cross(d,a); dot(c,c).sqrt() <= tolerance };
            let mut ball: Option<([f64;3],f64)> = None;
            for e in profile.loops.iter().flatten() {
                match e.shape {
                    Stroke::Line {start,end} => if !on(start) || !on(end) { return None },
                    Stroke::Circle {center: c,radius: r,..} => {
                        if !on(c) { return None; }
                        match ball { None => ball = Some((c,r)),
                            Some((k,q)) => if dot(sub(k,c),sub(k,c)).sqrt() > tolerance || (q-r).abs() > tolerance { return None } }
                    }
                    _ => return None,
                }
            }
            ball.map(|(c,_)| c)
        };
        let Some((origin,axis)) = nodes.iter().find_map(|n| match n.node {
            Node::Revolve {origin,axis,..} if is_ball(&n.node).is_none() => Some((origin,unit(axis))),
            _ => None,
        }) else {
            return Ok(Err("no revolution of it is about a line of its own".into()));
        };
        let on_line = |p: [f64;3]| { let c = cross(sub(p,origin),axis); dot(c,c).sqrt() <= tolerance };
        // the half-plane the sections are taken in: towards `seam`, square to the line
        let seed = if axis[0].abs() < 0.9 { [1.,0.,0.] } else { [0.,1.,0.] };
        let seam = unit(cross(cross(axis,seed),axis));
        let turn = |angle: f64| turned_about(origin,axis,angle);
        let mut regions: BTreeMap<i64,c_int> = BTreeMap::new();
        for Named {id,name,node} in nodes {
            let region = match node {
                Node::Revolve {profile,origin: o,axis: a,angle} => {
                    if (angle.abs()-std::f64::consts::TAU).abs() > 1e-9 {
                        return Ok(Err(format!("`{name}` is a partial revolution")));
                    }
                    if let Some(c) = is_ball(node) {
                        if !on_line(c) { return Ok(Err(format!("`{name}` is a ball off the line")))}
                        // its section: the half-disc about its centre in the half-plane
                        let r = profile.loops[0].iter().find_map(|e| match e.shape { Stroke::Circle {radius,..} => Some(radius),_ => None })
                            .unwrap();
                        let (a,b) = (std::array::from_fn::<f64,3,_>(|k| c[k]-r*axis[k]),std::array::from_fn::<f64,3,_>(|k| c[k]+r*axis[k]));
                        let chord = self.result(unsafe { solvent_cad_line(self.0,a.as_ptr(),b.as_ptr()) })?;
                        let normal = cross(seam,axis);
                        let arc = self.result(unsafe { solvent_cad_circle(self.0,c.as_ptr(),normal.as_ptr(),seam.as_ptr(),r,
                            -std::f64::consts::FRAC_PI_2,std::f64::consts::FRAC_PI_2) })?;
                        self.result(unsafe { solvent_cad_face(self.0,[arc,chord].as_ptr(),2) })?
                    } else {
                        let (o,a) = (*o,unit(*a));
                        if dot(cross(a,axis),cross(a,axis)).sqrt() > 1e-12 || !on_line(o) {
                            return Ok(Err(format!("`{name}` is a revolution about another line")));
                        }
                        // its profile on one side of the line: the side every point of it is on
                        let mut side: Option<[f64;3]> = None;
                        let mut straddles = false;
                        for e in profile.loops.iter().flatten() {
                            let points: Vec<[f64;3]> = match e.shape {
                                Stroke::Line {start,end} => vec![start,end],
                                Stroke::Circle {center: c,radius: r,..} => {
                                    // a circle's extent across the line: its centre's offset, less and more its radius
                                    let d = sub(c,origin); let d = sub(d,axis.map(|x| x*dot(d,axis)));
                                    if dot(d,d).sqrt() <= r+tolerance { straddles = true; }
                                    vec![c]
                                }
                                Stroke::BSpline {..} => return Ok(Err(format!("`{name}`'s profile has a spline"))),
                            };
                            for p in points {
                                let d = sub(p,origin); let d = sub(d,axis.map(|x| x*dot(d,axis)));
                                let n = dot(d,d).sqrt();
                                if n <= tolerance { continue }
                                let d = d.map(|x| x/n);
                                match side { None => side = Some(d), Some(s) => if dot(s,d) < 1.-1e-9 { straddles = true } }
                            }
                        }
                        let Some(side) = side.filter(|_| !straddles) else {
                            return Ok(Err(format!("`{name}`'s section crosses its axis")));
                        };
                        let mut face = None;
                        for edges in &profile.loops {
                            let loop_face = self.face(edges)?;
                            face = Some(match face { None => loop_face,Some(outer) => self.boolean(outer,loop_face,"cut")? });
                        }
                        let face = face.ok_or("empty CAD profile")?;
                        // turned about the line from its side to the half-plane's
                        let angle = dot(axis,cross(side,seam)).atan2(dot(side,seam));
                        self.place(face,turn(angle)?,1.)?
                    }
                }
                Node::Body(body) => {
                    let mut id = regions[&body.stock];
                    for (operation,operands) in body.sets().into_iter().take(3) {
                        for operand in operands { id = self.boolean(id,regions[operand],operation)?; }
                    }
                    id
                }
                node => return Ok(Err(format!("`{name}` is a {}, not a revolution",node.kind()))),
            };
            regions.insert(*id,region);
        }
        let region = regions[&recipe.root];
        let solid = self.revolve_region(region,origin,axis)?;
        Ok(Ok((solid,Meridian {region,origin,axis,seam})))
    }

    /// The recipe built by the native kernel (`built`), read first.
    pub(crate) fn construct(&self,recipe: &Json) -> Result<c_int,String> { self.built(&Recipe::read(recipe)?) }

    /// Every node of the recipe built in turn, each checked, and the root returned.
    pub(crate) fn built(&self,recipe: &Recipe) -> Result<c_int,String> {
        let mut shapes = BTreeMap::new();
        for Named {id,name,node} in &recipe.nodes {
            let make = || -> Result<c_int,String> {
                let id = match node {
                    Node::Body(body) => {
                        let mut id = shapes[&body.stock];
                        // then what reads the faces the rest make, fills then cuts (the stratum)
                        for (key,operands) in body.sets() {
                            let operation = match key { "on_late" => "on","cut_late" => "cut",key => key };
                            for operand in operands {
                                id = self.boolean(id,shapes[operand],operation)?;
                            }
                        }
                        id
                    }
                    Node::Region(pieces) => {
                        // a region in general position (`solid::bounded`, #238): the common of the
                        // primitives it keeps within, less those it keeps outside
                        let mut id = None;
                        for piece in pieces {
                            let p = self.primitive(&piece.node,&shapes)?;
                            id = Some(match id { None => p,Some(s) => self.boolean(s,p,if piece.keep { "bound" } else { "cut" })? });
                        }
                        id.ok_or("a region of no pieces")?
                    }
                    Node::Placed {source,matrix} => {
                        self.result(unsafe { solvent_cad_transform(self.0,shapes[source],matrix.as_ptr()) })?
                    }
                    node => self.primitive(node,&shapes)?,
                };
                self.result(unsafe { solvent_cad_validate(self.0,id) })?;
                Ok(id)
            };
            let clock = std::time::Instant::now();
            let made = make().map_err(|e| format!("{name}: {e}"))?;
            if std::env::var_os("SOLVENT_CONSTRUCT_DEBUG").is_some() {
                eprintln!("construct: `{name}` ({}) in {:?}",node.kind(),clock.elapsed());
            }
            shapes.insert(*id,made);
        }
        Ok(shapes[&recipe.root])
    }
}
