use super::*;

#[test]
fn declared_tooth_faces_have_shared_oriented_boundaries_on_their_exact_supports() {
    for teeth in [[24,48],[32,32],[28,49]] {
        for module in [0.2,2.,25.4] {
            let pair = Pair::read(teeth,module);
            assert_eq!(pair.faces.len(),8);
            for member in 0..2 {
                let frame = pair.local_frame(member);
                for side in 0..2 {
                    let name = if member == side { "outer" } else { "inner" };
                    let flank = pair.patch(member,side,name);
                    let round = pair.patch(member,side,&format!("{name}_round"));
                    let working = &pair.faces[&format!("{}_faces.working",flank.name)];
                    let transition = &pair.faces[&format!("{}_faces.transition",flank.name)];
                    assert_eq!(working.edge_uses()[3].edge,transition.edge_uses()[1].edge);
                    assert_eq!(working.edge_uses()[3].direction,transition.edge_uses()[1].direction.reversed());
                    for (face,surface) in [(working,&flank),(transition,&round)] {
                        for edge in 0..4 {
                            let a = face.sample_boundary(edge,1.,100).unwrap();
                            let b = face.sample_boundary((edge+1)%4,0.,100).unwrap();
                            assert_eq!(a.position,b.position,"{}: corner {edge}",face.name);
                            for f in [0.,0.25,0.5,0.75,1.] {
                                let p = face.sample_boundary(edge,f,100).unwrap();
                                let rho = p.position[0].hypot(p.position[1]).hypot(p.position[2]);
                                let expected = pair.analytic(member,surface,p.parameters[0],rho);
                                near(p.parameters,expected.parameters,1e-8);
                                near(frame.point(p.position),expected.contact.position,module*1e-7);
                                assert!(p.incidence_error <= module*1e-10);
                            }
                        }
                    }
                }
            }
        }
    }
}
