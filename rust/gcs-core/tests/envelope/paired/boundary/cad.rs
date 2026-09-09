//! CAD reference exports, separate from the nominal spherical-section model.
use super::*;

#[test]
#[ignore = "exports bounded straight-flank references and solved tip cones"]
fn export_cad_flank_references() {
    use gcs_core::interval::Interval as I;
    let path = std::env::var_os("SOLVENT_CAD_FLANKS_OUTPUT").expect("set output JSON path");
    let pair = Pair::read([24,48],2.);
    let mut records = vec![];
    for member in 0..2 {
        let tip: Vec<V> = [0.,1.].iter().map(|&u| {
            let p = pair.local_frame(member).point(pair.limits[member][0].at(u,0.).unwrap().position);
            [p[0].hypot(p[1]),0.,p[2]]
        }).collect();
        for side in 0..2 {
            let name = if member == side { "outer" } else { "inner" };
            let patch = pair.patch(member,side,name);
            let [start,delta,second] = patch.generating_profile_jet_bounds(I::ZERO).unwrap();
            assert!(second.iter().all(|v| v.bounds() == [0.,0.]));
            let join = 1.-pair.seam(&patch,false).endpoint_parameters()[1];
            let index_angle = if member == 1 && side == 1 { TAU/pair.teeth[1] } else { 0. };
            records.push(format!("{{\"member\":{member},\"side\":{side},\"name\":\"{}\",\"start\":{:?},\"delta\":{:?},\"join_parameter\":{join},\"tip_meridian\":{tip:?},\"index_angle\":{index_angle},\"roll_domain\":{:?}}}",
                patch.name,start.map(I::bounds),delta.map(I::bounds),pair.domain(&patch)[2]));
        }
    }
    std::fs::write(path,format!("{{\"teeth\":{:?},\"module_mm\":{},\"rho_range\":{:?},\"crown_center\":{:?},\"flanks\":[{}],\"scope\":\"Enclosed solved straight meridians and nominal tip cones; source-solve and assembly error separate\"}}\n",
        pair.teeth,pair.module,[0.9*pair.rm,1.1*pair.rm],pair.offset,records.join(","))).unwrap();
}

#[test]
#[ignore = "exports bounded generating-meridian coefficients for CAD fillet accuracy"]
fn export_cad_fillet_references() {
    use gcs_core::interval::Interval as I;
    let path = std::env::var_os("SOLVENT_CAD_FILLETS_OUTPUT").expect("set output JSON path");
    let pair = Pair::read([24,48],2.);
    let mut records = vec![];
    for member in 0..2 {
        for side in 0..2 {
            let name = if member == side { "outer" } else { "inner" };
            let flank = pair.patch(member,side,name);
            let patch = pair.patch(member,side,&format!("{name}_round"));
            let join = pair.seam(&flank,false).endpoint_parameters()[1];
            let index = pair.model.map.ent_named(&patch.name).unwrap().i();
            let edge = pair.model.sketch.surfaces[index].edge;
            assert_eq!(edge.kind,gcs_core::model::EntKind::Arc);
            let (start,end) = pair.model.sketch.arc_angles(edge.i());
            let sweep = end-start;
            let omega = I::point(sweep).unwrap();
            let [p,d,dd] = patch.generating_profile_jet_bounds(I::ZERO).unwrap();
            // Recover interval enclosures of the private snapshot coefficients:
            // p(0)=center+a, p'(0)=b*omega, p''(0)=-a*omega^2.
            // No rounded midpoint is substituted for a coefficient enclosure.
            let a: [I;3] = std::array::from_fn(|k| dd[k].neg().div(omega.square().unwrap()).unwrap());
            let b: [I;3] = std::array::from_fn(|k| d[k].div(omega).unwrap());
            let center: [I;3] = std::array::from_fn(|k| p[k].sub(a[k]).unwrap());
            let u = [1.-join,join];
            let index_angle = if member == 1 && side == 1 { TAU/pair.teeth[1] } else { 0. };
            records.push(format!("{{\"member\":{member},\"side\":{side},\"name\":\"{}\",\"center\":{:?},\"a\":{:?},\"b\":{:?},\"sweep\":{sweep},\"u_range\":{u:?},\"index_angle\":{index_angle},\"roll_domain\":{:?}}}",
                patch.name,center.map(I::bounds),a.map(I::bounds),b.map(I::bounds),pair.domain(&patch)[2]));
        }
    }
    std::fs::write(path,format!("{{\"teeth\":{:?},\"module_mm\":{},\"rho_range\":{:?},\"crown_center\":{:?},\"fillets\":[{}],\"scope\":\"Enclosed solved meridian coefficients; ideal common-crown generation, source-solve and assembly error separate\"}}\n",
        pair.teeth,pair.module,[0.9*pair.rm,1.1*pair.rm],pair.offset,records.join(","))).unwrap();
}

#[test]
#[ignore = "exports solved tooth-space samples for the isolated CAD-kernel experiment"]
fn export_tooth_space_sections_for_cad_backend() {
    let path = std::env::var_os("SOLVENT_CAD_SECTIONS_OUTPUT").expect("set output JSON path");
    let pair = Pair::read_cad_export();
    // Save the actual linked source, including configured module arguments, so
    // the ordinary CLI/editor can open exactly the model behind these samples.
    if let Some(directory) = std::env::var_os("SOLVENT_CAD_MODEL_OUTPUT") {
        let directory = std::path::PathBuf::from(directory);
        std::fs::create_dir(&directory).expect("model output directory must be new");
        std::fs::write(directory.join("pair.sv"),pair.model.program.text()).unwrap();
        for module in &pair.model.program.modules {
            let file = directory.join(format!("{}.sv",module.name.replace('.',"/")));
            std::fs::create_dir_all(file.parent().unwrap()).unwrap();
            std::fs::write(file,&module.text).unwrap();
        }
    }
    let mut members = vec![];
    for member in 0..2 {
        let mut levels = vec![];
        for n in [8,16,32] {
            let sides: Vec<Vec<Vec<V>>> = (0..2).map(|side| (0..=n).map(|row| {
                let rho = pair.rm*(0.9+0.2*row as f64/n as f64);
                let name = if member == side { "outer" } else { "inner" };
                let flank = pair.patch(member,side,name);
                let round = pair.patch(member,side,&format!("{name}_round"));
                let seam = pair.seam(&flank,false);
                let join = seam.endpoint_parameters()[1];
                let junction = pair.seam_at(member,seam,rho);
                let tip = pair.tip(member,&flank,rho,junction.parameters);
                // The cutter meridian is smooth through each patch. Axial height
                // becomes singular at a tangent root and is a poor spline parameter.
                let sample = |patch: &RevolvedSurface,u| {
                    let reference = pair.analytic(member,patch,u,rho);
                    let found = pair.at_seed(member,patch,u,rho,reference.parameters);
                    near(found.contact.position,reference.contact.position,pair.module*1e-8);
                    pair.check_trim(patch,found.parameters);
                    found.contact.position
                };
                (0..=n).map(|i| sample(&round,1.-join+(2.*join-1.)*i as f64/n as f64))
                    .chain((1..=n).map(|i| sample(&flank,1.-join+
                        (tip.parameters[0]-(1.-join))*i as f64/n as f64))).collect()
            }).collect()).collect();
            levels.push(format!("{{\"subdivisions\":{n},\"sides\":{sides:?}}}"));
        }
        let blank: Vec<Vec<V>> = [0.9*pair.rm,1.1*pair.rm].iter().map(|&rho|
            [2,0].iter().map(|&boundary| {
                let hits = pair.limits[member][boundary].line_on_sphere([0.;3],rho,0.).unwrap();
                assert_eq!(hits.len(),1);
                let p = pair.local_frame(member).point(hits[0].position);
                [p[0].hypot(p[1]),0.,p[2]]
            }).collect()).collect();
        members.push(format!("{{\"member\":{member},\"teeth\":{},\"blank_meridian\":{blank:?},\"levels\":[{}]}}",
            pair.teeth[member],levels.join(",")));
    }
    std::fs::write(path,format!("{{\"scope\":\"Local generated envelopes; not a global material certificate\",\"module_mm\":{},\"mean_distance\":{},\"members\":[{}]}}\n",
        pair.module,pair.rm,members.join(","))).unwrap();
}

#[test]
#[ignore = "exports solved closure supports for independent CAD surface bounds"]
fn export_cad_boundary_supports() {
    let path = std::env::var_os("SOLVENT_CAD_SUPPORTS_OUTPUT").expect("set output JSON path");
    let pair = Pair::read([24,48],2.);
    let radii = [0.9*pair.rm,1.1*pair.rm];
    let mut members = vec![];
    for member in 0..2 {
        let local = |p| {
            let p = pair.local_frame(member).point(p);
            [p[0].hypot(p[1]),0.,p[2]]
        };
        let cones: Vec<Vec<V>> = pair.limits[member].iter().map(|s|
            [0.,1.].iter().map(|&u| local(s.at(u,0.).unwrap().position)).collect()).collect();
        let blank: Vec<Vec<V>> = radii.iter().map(|&rho| [2,0].iter().map(|&boundary| {
            let hits = pair.limits[member][boundary].line_on_sphere([0.;3],rho,0.).unwrap();
            assert_eq!(hits.len(),1);
            local(hits[0].position)
        }).collect()).collect();
        members.push(format!("{{\"member\":{member},\"teeth\":{},\"cones_tip_root_back\":{cones:?},\"blank_meridian\":{blank:?}}}",pair.teeth[member]));
    }
    std::fs::write(path,format!("{{\"module_mm\":{},\"sphere_radii_mm\":{radii:?},\"members\":[{}],\"scope\":\"Exact binary64 nominal support coefficients extracted from the solved source; source-solve error is separate\"}}\n",pair.module,members.join(","))).unwrap();
}
