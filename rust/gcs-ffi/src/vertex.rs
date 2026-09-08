//! Checked spatial vertex access. The ABI only marshals core snapshots.
use super::{guard,set_error,sk,Sketch};
use gcs_core::{seam::{BoundarySeamTolerance,SeamTolerance},vertex::{BoundaryVertex,JunctionVertex}};

unsafe fn write<const N: usize>(result: Result<[f64;N],String>,out: *mut f64) -> i32 {
    match result {
        Ok(values) => {
            for (i,v) in values.into_iter().enumerate() { out.add(i).write(v); }
            1
        }
        Err(message) => { set_error(message); 0 }
    }
}

fn domain(d: [[f64;2];3]) -> [f64;6] { std::array::from_fn(|i| d[i/2][i%2]) }

/// Search domain of a two-boundary vertex, six doubles in [u,v,roll] order.
#[no_mangle]
pub unsafe extern "C" fn gcs_boundary_vertex_domain(h: *mut Sketch,idx: i32,
    axis_tolerance: f64,out: *mut f64) -> i32 {
    guard(0,move || write(BoundaryVertex::named(sk(h),idx as usize,axis_tolerance)
        .map(|v| domain(v.domain())),out))
}

/// Retained position only (three doubles); corners have no unique face normal.
#[no_mangle]
pub unsafe extern "C" fn gcs_boundary_vertex_position(h: *mut Sketch,idx: i32,
    u: f64,v: f64,roll: f64,axis_tolerance: f64,normal_velocity_tolerance: f64,
    incidence_tolerance: f64,trim_tolerance: f64,out: *mut f64) -> i32 {
    guard(0,move || {
        let tolerance = BoundarySeamTolerance {normal_velocity:normal_velocity_tolerance,
            incidence:incidence_tolerance,trim:trim_tolerance};
        write(BoundaryVertex::named(sk(h),idx as usize,axis_tolerance)
            .and_then(|vtx| vtx.position([u,v,roll],tolerance)
                .map_err(|e| format!("invalid boundary vertex: {e:?}"))),out)
    })
}

/// Search domain of a generating-junction vertex; first-source u is fixed.
#[no_mangle]
pub unsafe extern "C" fn gcs_junction_vertex_domain(h: *mut Sketch,idx: i32,
    position_tolerance: f64,normal_tolerance: f64,axis_tolerance: f64,out: *mut f64) -> i32 {
    guard(0,move || {
        let tolerance = SeamTolerance {position:position_tolerance,normal:normal_tolerance,axis:axis_tolerance};
        write(JunctionVertex::named(sk(h),idx as usize,tolerance).map(|v| domain(v.domain())),out)
    })
}

/// Retained junction vertex position, three doubles in the canonical first-face chart.
#[no_mangle]
pub unsafe extern "C" fn gcs_junction_vertex_position(h: *mut Sketch,idx: i32,
    u: f64,v: f64,roll: f64,position_tolerance: f64,normal_tolerance: f64,axis_tolerance: f64,
    normal_velocity_tolerance: f64,incidence_tolerance: f64,trim_tolerance: f64,out: *mut f64) -> i32 {
    guard(0,move || {
        let tolerance = SeamTolerance {position:position_tolerance,normal:normal_tolerance,axis:axis_tolerance};
        let point_tolerance = BoundarySeamTolerance {normal_velocity:normal_velocity_tolerance,
            incidence:incidence_tolerance,trim:trim_tolerance};
        write(JunctionVertex::named(sk(h),idx as usize,tolerance)
            .and_then(|vtx| vtx.position([u,v,roll],point_tolerance)
                .map_err(|e| format!("invalid junction vertex: {e:?}"))),out)
    })
}
