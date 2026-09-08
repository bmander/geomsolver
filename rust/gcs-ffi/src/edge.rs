//! Finite edge access marshals endpoint witnesses and core solve controls.
use super::{guard,set_error,sk,Sketch};
use gcs_core::{edge::{EdgeTolerance,SpatialEdge},seam::{BoundarySeamTolerance,SeamTolerance}};

/// Endpoint parameters: six doubles, each vertex's canonical [u,v,roll] chart.
/// Output: position xyz followed by the edge seam's [u,v,roll], six doubles.
#[no_mangle]
pub unsafe extern "C" fn gcs_edge_sample(h: *mut Sketch,idx: i32,endpoints: *const f64,
    fraction: f64,position_tolerance: f64,normal_tolerance: f64,axis_tolerance: f64,
    normal_velocity_tolerance: f64,incidence_tolerance: f64,trim_tolerance: f64,
    max_iterations: u32,out: *mut f64) -> i32 {
    guard(0,move || {
        let parameters = std::array::from_fn(|i| std::array::from_fn(|j| endpoints.add(i*3+j).read()));
        let tolerance = EdgeTolerance {
            junction:SeamTolerance {position:position_tolerance,normal:normal_tolerance,axis:axis_tolerance},
            point:BoundarySeamTolerance {normal_velocity:normal_velocity_tolerance,
                incidence:incidence_tolerance,trim:trim_tolerance},
        };
        let result = SpatialEdge::named(sk(h),idx as usize,parameters,tolerance)
            .and_then(|e| e.sample(fraction,max_iterations).map_err(|e| format!("invalid edge sample: {e:?}")));
        match result {
            Ok(p) => {
                for (i,v) in p.position.into_iter().chain(p.parameters).enumerate() { out.add(i).write(v); }
                1
            }
            Err(message) => { set_error(message); 0 }
        }
    })
}
