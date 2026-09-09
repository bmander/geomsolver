use super::*;

#[test]
fn meridian_chart_crosses_a_sphere_contact_turn_without_a_missing_root() {
    let e = swept("");
    let sweep = SweepContacts::read(&e.sketch,e.map.ent_named("swept").unwrap().i(),1e-10).unwrap();
    let surface = &sweep.patches()[0];
    let chart = surface.angular_chart([-0.1,0.1]).unwrap();
    let motion = Motion::translation([0.;3],[1.,0.,1.]).unwrap();
    assert!(surface.contacts(0.24,motion,1e-10).unwrap().is_empty());
    assert_eq!(surface.contacts(0.25,motion,1e-10).unwrap_err(),Error::Degenerate);
    assert_eq!(surface.contacts(0.26,motion,1e-10).unwrap().len(),2);
    for i in -20..=20 {
        let v = i as f64/200.;
        let roots = chart.meridian_contacts(v,motion,1e-10).unwrap();
        assert_eq!(roots.len(),1);
        let expected = 1_f64.atan2((2.*PI*v).cos())/PI;
        assert!((roots[0].u-expected).abs() < 1e-12);
        let p = roots[0].contact.position;
        assert!((p[0]-3.+p[2]).abs() < 1e-12);
    }
    assert_eq!(chart.meridian_contacts(0.,Motion::identity(),1e-10).unwrap_err(),Error::Degenerate);
    assert_eq!(chart.meridian_contacts(0.,motion,0.).unwrap_err(),Error::InvalidOptions);
    assert_eq!(chart.meridian_contacts(0.2,motion,1e-10).unwrap_err(),Error::OutsideDomain);
    assert_eq!(sweep.at_angle(0,0.2,2.,1e-10).unwrap_err(),Error::OutsideDomain);
    assert_eq!(sweep.at_angle(100,0.2,0.,1e-10).unwrap_err(),Error::OutsideDomain);
}

#[test]
fn line_and_round_meridians_match_independent_normal_velocity_crossings() {
    let src = include_str!("../../../../examples/spiral_bevel/reference.sv");
    let (p,errors,links) = gcs_core::library::parse_linked(src);
    assert!(errors.is_empty() && links.is_empty());
    let mut e = gcs_core::program::elaborate(&p);
    assert!(e.ok());
    assert!(gcs_core::solve::solve(&mut e.sketch,Default::default()).success);
    let placement = Motion::rotation([1.,2.,3.],0.7,0.).unwrap()
        .then(Motion::translation([3.,-4.,2.],[0.;3]).unwrap());
    let mut counts = [0;2];
    for sense in [gcs_core::model::Sense::Ccw,gcs_core::model::Sense::Cw] {
    let si = e.map.ent_named("crown").unwrap().i();
    let gcs_core::model::SolidDef::Revolve {sense:direction,..} = &mut e.sketch.solids[si].def else { panic!() };
    *direction = sense;
    for (kind,name) in ["outer","outer_round","inner","inner_round"].into_iter().enumerate() {
        let source = RevolvedSurface::named(&e.sketch,e.map.ent_named(name).unwrap().i()).unwrap();
        for surface in [source.clone(),source.placed(placement)] {
            for v in [0.13,0.31,0.57,0.81] { for t in [-0.4,0.,0.3] {
                let motion = Motion::rotation([1.,2.,3.],t,1.).unwrap()
                    .then(Motion::translation([2.,-3.,1.],[0.4,-0.2,0.3]).unwrap())
                    .then(Motion::rotation([1.,0.,0.],-2.*t,-2.).unwrap().inverse());
                let roots = surface.meridian_contacts(v,motion,1e-10).unwrap();
                let values: Vec<_> = (0..=1000).map(|i| envelope::contact(
                    surface.at(i as f64/1000.,v).unwrap(),motion).unwrap().normal_velocity).collect();
                let crossings = values.windows(2).filter(|v| v[0]*v[1] < 0.).count();
                assert_eq!(roots.len(),crossings,"{name} {v} {t}");
                for root in roots {
                    assert!(root.contact.normal_velocity.abs() < 1e-10);
                    counts[kind%2] += 1;
                }
            } }
        }
    }
    }
    assert!(counts.iter().all(|n| *n > 0),"both line and round contacts must be exercised: {counts:?}");
    let line = RevolvedSurface::named(&e.sketch,e.map.ent_named("outer").unwrap().i()).unwrap();
    assert_eq!(line.meridian_contacts(0.13,Motion::identity(),1e-10).unwrap_err(),Error::Degenerate);
    assert!(line.meridian_contacts(0.13,Motion::translation([0.;3],[0.,0.,1.]).unwrap(),1e-10).unwrap().is_empty());
}

#[test]
fn a_full_round_meridian_retains_both_isolated_roots_without_seam_duplication() {
    let e = read("unit mm\npoint a hint(x: 0,y: 0)\nground a\n\
        point b hint(x: 0,y: 1)\nground b\nline axis(a,b)\n\
        point c hint(x: 10,y: 2)\nground c\ncircle ring(center: c)\nradius(2) ring\n\
        solid tool(face(ring),about: axis)\nsurface wall(tool,ring)\n");
    let surface = RevolvedSurface::named(&e.sketch,e.map.ent_named("wall").unwrap().i()).unwrap();
    let motion = Motion::translation([0.;3],[0.,0.,1.]).unwrap();
    for v in [0.,0.13,0.79,1.] {
        let roots = surface.meridian_contacts(v,motion,1e-10).unwrap();
        assert_eq!(roots.len(),2);
        let mut coordinates: Vec<_> = roots.iter().map(|r| r.u).collect();
        coordinates.sort_by(f64::total_cmp);
        assert!(coordinates[0].abs() < 1e-12 && (coordinates[1]-0.5).abs() < 1e-12);
        for root in roots { assert!((root.contact.position[2]-2.).abs() < 1e-12); }
    }
}
