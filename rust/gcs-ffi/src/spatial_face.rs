//! Spatial-face boundary metadata and checked samples; all geometry stays in core.
use super::{guard,set_error,sk,Sketch};
use gcs_core::{edge::EdgeTolerance,seam::{BoundarySeamTolerance,SeamTolerance},
    spatial_face::SpatialFaceBoundary,topology::Direction};

/// Returns the number of directed edge uses, or -1 on error. Capacity is in uses;
/// zero capacity queries size. Each output use has two i32s: edge ID and +/-1 sense.
#[no_mangle]
pub unsafe extern "C" fn gcs_spatial_face_loop(h: *mut Sketch,idx: i32,out: *mut i32,capacity: u32) -> i32 {
    guard(-1,move || {
        match SpatialFaceBoundary::declared_boundary(sk(h),idx as usize) {
            Ok(uses) => {
                if capacity != 0 {
                    if (capacity as usize) < uses.len() { set_error("face boundary buffer is too small"); return -1; }
                    for (i,u) in uses.iter().enumerate() {
                        out.add(2*i).write(u.edge as i32);
                        out.add(2*i+1).write(if u.direction == Direction::Forward { 1 } else { -1 });
                    }
                }
                uses.len() as i32
            }
            Err(message) => { set_error(message); -1 }
        }
    })
}

/// Vertex parameters are triples indexed by spatial vertex ID. Output is seven
/// doubles: shared edge xyz, this support's [u,v,roll] and finite incidence error.
#[no_mangle]
pub unsafe extern "C" fn gcs_face_boundary_sample(h: *mut Sketch,idx: i32,edge: u32,fraction: f64,
    vertices: *const f64,vertex_count: u32,position_tolerance: f64,normal_tolerance: f64,
    axis_tolerance: f64,normal_velocity_tolerance: f64,incidence_tolerance: f64,trim_tolerance: f64,
    max_iterations: u32,out: *mut f64) -> i32 {
    guard(0,move || {
        let sketch = sk(h);
        if vertex_count as usize > sketch.vertices.len() {
            set_error("face witness count exceeds sketch vertex count"); return 0;
        }
        let witnesses = (0..vertex_count as usize).map(|i|
            std::array::from_fn(|j| vertices.add(i*3+j).read())).collect::<Vec<_>>();
        let tolerance = EdgeTolerance {
            junction:SeamTolerance {position:position_tolerance,normal:normal_tolerance,axis:axis_tolerance},
            point:BoundarySeamTolerance {normal_velocity:normal_velocity_tolerance,
                incidence:incidence_tolerance,trim:trim_tolerance},
        };
        let result = SpatialFaceBoundary::named(sketch,idx as usize,&witnesses,tolerance)
            .and_then(|f| f.sample_boundary(edge as usize,fraction,max_iterations)
                .map_err(|e| format!("invalid face boundary sample: {e:?}")));
        match result {
            Ok(p) => {
                for (i,v) in p.position.into_iter().chain(p.parameters).chain([p.incidence_error]).enumerate() {
                    out.add(i).write(v);
                }
                1
            }
            Err(message) => { set_error(message); 0 }
        }
    })
}
