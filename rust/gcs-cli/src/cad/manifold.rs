//! Closed-mesh Booleans through the Manifold library's C API. Every value is a
//! closed, oriented manifold mesh or the construction fails; nothing here
//! decides material, it only arranges.
use std::ffi::{c_int,c_void};

#[repr(C)] pub struct ManifoldManifold { _private: [u8;0] }
#[repr(C)] pub struct ManifoldMeshGL64 { _private: [u8;0] }
#[repr(C)] pub struct ManifoldManifoldVec { _private: [u8;0] }
#[repr(C)] struct ManifoldManifoldPair { first: *mut ManifoldManifold,second: *mut ManifoldManifold }

extern "C" {
    fn manifold_alloc_manifold() -> *mut ManifoldManifold;
    fn manifold_alloc_meshgl64() -> *mut ManifoldMeshGL64;
    fn manifold_alloc_manifold_vec() -> *mut ManifoldManifoldVec;
    fn manifold_delete_manifold(m: *mut ManifoldManifold);
    fn manifold_delete_meshgl64(m: *mut ManifoldMeshGL64);
    fn manifold_delete_manifold_vec(m: *mut ManifoldManifoldVec);
    fn manifold_meshgl64(mem: *mut c_void,vert_props: *mut f64,n_verts: usize,n_props: usize,
        tri_verts: *mut u64,n_tris: usize) -> *mut ManifoldMeshGL64;
    fn manifold_meshgl64_merge(mem: *mut c_void,m: *mut ManifoldMeshGL64) -> *mut ManifoldMeshGL64;
    fn manifold_of_meshgl64(mem: *mut c_void,mesh: *mut ManifoldMeshGL64) -> *mut ManifoldManifold;
    fn manifold_get_meshgl64(mem: *mut c_void,m: *mut ManifoldManifold) -> *mut ManifoldMeshGL64;
    fn manifold_meshgl64_num_vert(m: *mut ManifoldMeshGL64) -> usize;
    fn manifold_meshgl64_num_tri(m: *mut ManifoldMeshGL64) -> usize;
    fn manifold_meshgl64_num_prop(m: *mut ManifoldMeshGL64) -> usize;
    fn manifold_meshgl64_vert_properties_length(m: *mut ManifoldMeshGL64) -> usize;
    fn manifold_meshgl64_tri_length(m: *mut ManifoldMeshGL64) -> usize;
    fn manifold_meshgl64_vert_properties(mem: *mut c_void,m: *mut ManifoldMeshGL64) -> *mut f64;
    fn manifold_meshgl64_tri_verts(mem: *mut c_void,m: *mut ManifoldMeshGL64) -> *mut u64;
    fn manifold_boolean(mem: *mut c_void,a: *mut ManifoldManifold,b: *mut ManifoldManifold,op: c_int) -> *mut ManifoldManifold;
    fn manifold_batch_boolean(mem: *mut c_void,ms: *mut ManifoldManifoldVec,op: c_int) -> *mut ManifoldManifold;
    fn manifold_split(mem_first: *mut c_void,mem_second: *mut c_void,a: *mut ManifoldManifold,b: *mut ManifoldManifold) -> ManifoldManifoldPair;
    fn manifold_decompose(mem: *mut c_void,m: *mut ManifoldManifold) -> *mut ManifoldManifoldVec;
    fn manifold_manifold_empty_vec(mem: *mut c_void) -> *mut ManifoldManifoldVec;
    fn manifold_manifold_vec_length(ms: *mut ManifoldManifoldVec) -> usize;
    fn manifold_manifold_vec_get(mem: *mut c_void,ms: *mut ManifoldManifoldVec,idx: usize) -> *mut ManifoldManifold;
    fn manifold_manifold_vec_push_back(ms: *mut ManifoldManifoldVec,m: *mut ManifoldManifold);
    fn manifold_transform(mem: *mut c_void,m: *mut ManifoldManifold,x1: f64,y1: f64,z1: f64,x2: f64,y2: f64,z2: f64,
        x3: f64,y3: f64,z3: f64,x4: f64,y4: f64,z4: f64) -> *mut ManifoldManifold;
    fn manifold_volume(m: *mut ManifoldManifold) -> f64;
    fn manifold_surface_area(m: *mut ManifoldManifold) -> f64;
    fn manifold_simplify(mem: *mut c_void,m: *mut ManifoldManifold,tolerance: f64) -> *mut ManifoldManifold;
    fn manifold_status(m: *mut ManifoldManifold) -> c_int;
    fn manifold_is_empty(m: *mut ManifoldManifold) -> c_int;
    fn manifold_num_tri(m: *mut ManifoldManifold) -> usize;
}

const ADD: c_int = 0;
const SUBTRACT: c_int = 1;
const INTERSECT: c_int = 2;

fn status_text(code: c_int) -> &'static str {
    match code {
        0 => "no error",1 => "non-finite vertex",2 => "not manifold",3 => "vertex index out of bounds",
        4 => "properties wrong length",5 => "missing position properties",6 => "merge vectors different lengths",
        7 => "merge index out of bounds",8 => "transform wrong length",9 => "run index wrong length",
        10 => "face id wrong length",11 => "invalid construction",12 => "result too large",
        13 => "invalid tangents",14 => "cancelled",_ => "unknown Manifold error",
    }
}

/// One closed manifold mesh, owned.
pub struct Solid(*mut ManifoldManifold);
impl Drop for Solid { fn drop(&mut self) { unsafe { manifold_delete_manifold(self.0); } } }

struct MeshGl(*mut ManifoldMeshGL64);
impl Drop for MeshGl { fn drop(&mut self) { unsafe { manifold_delete_meshgl64(self.0); } } }

struct Vec_(*mut ManifoldManifoldVec);
impl Drop for Vec_ { fn drop(&mut self) { unsafe { manifold_delete_manifold_vec(self.0); } } }

impl Solid {
    fn checked(raw: *mut ManifoldManifold,what: &str) -> Result<Self,String> {
        if raw.is_null() { return Err(format!("Manifold {what} returned nothing")); }
        let solid = Self(raw);
        let status = unsafe { manifold_status(solid.0) };
        if status != 0 { return Err(format!("Manifold {what}: {}",status_text(status))); }
        Ok(solid)
    }

    /// From indexed triangles with outward winding. Nearly coincident vertices
    /// are merged by the library's own tolerance before the manifold check.
    pub fn from_triangles(vertices: &[[f64;3]],triangles: &[[u32;3]]) -> Result<Self,String> {
        if vertices.is_empty() || triangles.is_empty() { return Err("an empty mesh is not a solid".into()); }
        let mut props: Vec<f64> = vertices.iter().flatten().copied().collect();
        let mut tris: Vec<u64> = triangles.iter().flatten().map(|&i| i as u64).collect();
        unsafe {
            let mesh = MeshGl(manifold_meshgl64(manifold_alloc_meshgl64().cast(),props.as_mut_ptr(),vertices.len(),3,
                tris.as_mut_ptr(),triangles.len()));
            if mesh.0.is_null() { return Err("Manifold mesh construction failed".into()); }
            let merged = MeshGl(manifold_meshgl64_merge(manifold_alloc_meshgl64().cast(),mesh.0));
            let source = if merged.0.is_null() { mesh.0 } else { merged.0 };
            Self::checked(manifold_of_meshgl64(manifold_alloc_manifold().cast(),source),"mesh")
        }
    }

    /// Indexed triangles back, outward winding.
    pub fn triangles(&self) -> Result<(Vec<[f64;3]>,Vec<[u32;3]>),String> {
        unsafe {
            let mesh = MeshGl(manifold_get_meshgl64(manifold_alloc_meshgl64().cast(),self.0));
            if mesh.0.is_null() { return Err("Manifold mesh extraction failed".into()); }
            let (nv,nt,np) = (manifold_meshgl64_num_vert(mesh.0),manifold_meshgl64_num_tri(mesh.0),manifold_meshgl64_num_prop(mesh.0));
            if np < 3 { return Err("Manifold mesh has no positions".into()); }
            let mut props = vec![0f64;manifold_meshgl64_vert_properties_length(mesh.0)];
            manifold_meshgl64_vert_properties(props.as_mut_ptr().cast(),mesh.0);
            let mut tris = vec![0u64;manifold_meshgl64_tri_length(mesh.0)];
            manifold_meshgl64_tri_verts(tris.as_mut_ptr().cast(),mesh.0);
            let vertices = (0..nv).map(|i| [props[i*np],props[i*np+1],props[i*np+2]]).collect();
            let triangles = (0..nt).map(|i| [tris[3*i] as u32,tris[3*i+1] as u32,tris[3*i+2] as u32]).collect();
            Ok((vertices,triangles))
        }
    }

    fn boolean(&self,other: &Self,op: c_int,what: &str) -> Result<Self,String> {
        unsafe { Self::checked(manifold_boolean(manifold_alloc_manifold().cast(),self.0,other.0,op),what) }
    }
    pub fn union(&self,other: &Self) -> Result<Self,String> { self.boolean(other,ADD,"union") }
    pub fn difference(&self,other: &Self) -> Result<Self,String> { self.boolean(other,SUBTRACT,"difference") }
    pub fn intersection(&self,other: &Self) -> Result<Self,String> { self.boolean(other,INTERSECT,"intersection") }

    /// Union or intersection of many solids in one operation.
    pub fn batch(solids: &[&Self],union: bool) -> Result<Self,String> {
        if solids.is_empty() { return Err("a batch Boolean needs operands".into()); }
        unsafe {
            let list = Vec_(manifold_manifold_empty_vec(manifold_alloc_manifold_vec().cast()));
            for s in solids { manifold_manifold_vec_push_back(list.0,s.0); }
            Self::checked(manifold_batch_boolean(manifold_alloc_manifold().cast(),list.0,if union { ADD } else { INTERSECT }),"batch")
        }
    }

    /// (inside `cutter`, outside `cutter`).
    pub fn split(&self,cutter: &Self) -> Result<(Self,Self),String> {
        unsafe {
            let pair = manifold_split(manifold_alloc_manifold().cast(),manifold_alloc_manifold().cast(),self.0,cutter.0);
            Ok((Self::checked(pair.first,"split")?,Self::checked(pair.second,"split")?))
        }
    }

    /// Connected components as separate solids.
    pub fn components(&self) -> Result<Vec<Self>,String> {
        unsafe {
            let list = Vec_(manifold_decompose(manifold_alloc_manifold_vec().cast(),self.0));
            if list.0.is_null() { return Err("Manifold decomposition failed".into()); }
            (0..manifold_manifold_vec_length(list.0)).map(|i|
                Self::checked(manifold_manifold_vec_get(manifold_alloc_manifold().cast(),list.0,i),"component")).collect()
        }
    }

    /// Place by a row-major 3x4 matrix, the layout `cad::placement_matrix` gives.
    pub fn placed(&self,matrix: &[f64;12]) -> Result<Self,String> {
        let m = matrix;
        unsafe { Self::checked(manifold_transform(manifold_alloc_manifold().cast(),self.0,
            m[0],m[4],m[8],m[1],m[5],m[9],m[2],m[6],m[10],m[3],m[7],m[11]),"transform") }
    }

    pub fn volume(&self) -> f64 { unsafe { manifold_volume(self.0) } }
    pub fn area(&self) -> f64 { unsafe { manifold_surface_area(self.0) } }
    /// Remove vertices within `tolerance` of the surface they lie on, keeping
    /// the mesh manifold: the way to drop slivers below an output's resolution.
    pub fn simplified(&self,tolerance: f64) -> Result<Self,String> {
        unsafe { Self::checked(manifold_simplify(manifold_alloc_manifold().cast(),self.0,tolerance),"simplify") }
    }
    pub fn is_empty(&self) -> bool { unsafe { manifold_is_empty(self.0) != 0 } }
    pub fn triangle_count(&self) -> usize { unsafe { manifold_num_tri(self.0) } }
}
