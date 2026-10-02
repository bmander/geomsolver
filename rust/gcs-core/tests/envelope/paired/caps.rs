use super::*;
use gcs_core::seam::SurfaceSeamOptions;

#[test]
fn end_circles_match_independent_cone_sphere_sections_for_every_pair_configuration() {
    for teeth in [[24,48],[32,32],[28,49]] {
        for module in [0.2,2.,25.4] {
            let pair = Pair::read(teeth,module);
            assert_eq!(pair.surface_seams.len(),12);
            for member in 0..2 {
                let name = ["pinion","gear"][member];
                let frame = pair.local_frame(member);
                for (role,depth) in [("tip",1.),("root",-1.25),("back",-4.)] {
                    let d = depth*module*35f64.to_radians().cos();
                    for (end,fraction) in [("near",0.9),("far",1.1)] {
                        let rho = fraction*pair.rm;
                        // In the member's axial meridian, the cone equation is
                        // r*cos(delta)-z*sin(delta)=d and r²+z²=rho².
                        let angle = pair.delta[member]+(d/rho).asin();
                        let radius = rho*angle.sin();
                        let z = rho*angle.cos();
                        let transverse = (1.-(d/rho).powi(2)).sqrt();
                        let seam = &pair.surface_seams[&format!("faces.{name}_ends.{role}.{end}")];
                        for i in 0..=8 {
                            let v = i as f64/8.;
                            let found = seam.intersect(|p,_| p[1]-v,[fraction-0.5,0.5],
                                SurfaceSeamOptions {bounds:seam.domain(),parameter_scale:[1.;2],
                                    residual_tolerance:[module*1e-10,1e-10],
                                    min_transversality:0.9,max_iterations:100}).unwrap();
                            let p = frame.point(found.point.position);
                            assert!((p[0].hypot(p[1])-radius).abs() < module*1e-8);
                            assert!((p[2]-z).abs() < module*1e-8);
                            assert!((found.point.transversality-transverse).abs() < 1e-9);
                            assert!(found.point.incidence_error <= module*1e-10);
                            assert!((found.point.parameters[0][1]-v).abs() < 1e-10);
                            let tangent = frame.vector(found.point.tangent);
                            assert!(tangent[2].abs() < 1e-9);
                            assert!((p[0]*tangent[0]+p[1]*tangent[1]).abs() < module*1e-8);
                        }
                    }
                }
            }
        }
    }
}
