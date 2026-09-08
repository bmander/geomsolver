//! Complete cross sections of the nominal rim, before surface assembly and export.
use super::*;
use gcs_core::{csg::Piece,mesh};

mod interference;

type V = [f64;3];
fn polar(p: V) -> f64 { p[0].hypot(p[1]).atan2(p[2]) }
fn azimuth(p: V) -> f64 { p[1].atan2(p[0]) }
fn spherical(rho: f64, theta: f64, phi: f64) -> V {
    [rho*theta.sin()*phi.cos(),rho*theta.sin()*phi.sin(),rho*theta.cos()]
}

impl Pair {
    pub(super) fn tip(&self, member: usize, surface: &RevolvedSurface, rho: f64,
        seed: [f64;3]) -> envelope::Intersection {
        let seam = &self.boundary_seams[&format!("{}_tip_edge",surface.name)];
        let mut result = seam.intersect(
            |c| {
                let p = c.position;
                p[0].hypot(p[1]).hypot(p[2])-rho
            },seed,IntersectionOptions {
                bounds: seam.domain(),
                parameter_scale: [1.;3],residual_tolerance: [1e-10*self.module;3],
                max_iterations: 100,
            },1e-8*self.module).unwrap_or_else(|e| panic!("{} at {rho}: {e:?}",seam.name));
        let frame = self.local_frame(member);
        result.contact = envelope::Contact {position:frame.point(result.contact.position),
            normal:frame.vector(result.contact.normal),velocity:frame.vector(result.contact.velocity),
            normal_velocity:result.contact.normal_velocity};
        result
    }

    // From the root cone through its generated fillet and flank to the addendum cone.
    fn side(&self, member: usize, side: usize, rho: f64, n: usize) -> Vec<envelope::Contact> {
        self.side_with_end(member,side,rho,n,None)
    }

    fn side_with_end(&self,member: usize,side: usize,rho: f64,n: usize,end: Option<&str>)
        -> Vec<envelope::Contact> {
        let outer = member == side;
        let edge = if outer { "outer" } else { "inner" };
        let flank = self.patch(member,side,edge);
        let round = self.patch(member,side,&format!("{edge}_round"));
        if let Some(end) = end {
            // End-face contours now read finite source edges. Sampling is uniform
            // in the declared axial coordinate, with exact shared endpoint reuse.
            let sample = |surface: &RevolvedSurface,fraction| {
                let (base,role) = surface.name.strip_suffix("_round")
                    .map_or((surface.name.as_str(),"working"),|base| (base,"transition"));
                let face = &self.faces[&format!("{base}_faces.{role}")];
                let (slot,fraction) = if end == "toe" { (0,fraction) } else { (2,1.-fraction) };
                let p = face.sample_boundary(slot,fraction,100)
                    .unwrap_or_else(|e| panic!("{}: {e:?}",face.name));
                let c = self.region(surface).envelope_at(p.parameters,self.module*1e-8,self.module*1e-8)
                    .unwrap();
                near(c.position,p.position,self.module*1e-8);
                let frame = self.local_frame(member);
                envelope::Contact {position:frame.point(p.position),normal:frame.vector(c.normal),
                    velocity:frame.vector(c.velocity),normal_velocity:c.normal_velocity}
            };
            return (0..=n).map(|i| sample(&round,i as f64/n as f64))
                .chain((1..=n).map(|i| sample(&flank,i as f64/n as f64))).collect();
        }
        let seam = self.seam(&flank,false);
        let join = seam.endpoint_parameters()[1];
        let a = self.seam_at(member,seam,rho);
        let root_end = self.seam_at(member,self.seam(&flank,true),rho);
        let tip = self.tip(member,&flank,rho,a.parameters);
        // Interior rows use the same axial parameter as the declared toe/heel
        // edges, so matching mesh columns follow one consistent surface chart.
        let at_axial = |surface: &RevolvedSurface,z: f64,seed| {
            let frame = self.local_frame(member);
            self.intersection(member,surface,|_,c| [
                c.position[0].hypot(c.position[1]).hypot(c.position[2])-rho,
                frame.point(c.position)[2]-z,
            ],seed,IntersectionOptions {bounds:self.domain(surface),parameter_scale:[1.;3],
                residual_tolerance:[1e-10*self.module;3],max_iterations:100})
        };
        let mut root = vec![];
        let mut seed = a.parameters;
        for i in 0..=n {
            let u = join+(1.-2.*join)*i as f64/n as f64;
            let p = if i == 0 {
                let mut p = a; p.parameters[0] = join; p
            } else if i == n { root_end } else {
                seed[0] = u;
                let f = i as f64/n as f64;
                at_axial(&round,(1.-f)*a.contact.position[2]+f*root_end.contact.position[2],seed)
            };
            self.check_trim(&round,p.parameters);
            seed = p.parameters;
            root.push(p.contact);
        }
        root.reverse();
        seed = a.parameters;
        for i in 1..=n {
            let u = 1.-join+(tip.parameters[0]-(1.-join))*i as f64/n as f64;
            let p = if i == n { tip } else {
                seed[0] = u;
                let f = i as f64/n as f64;
                at_axial(&flank,(1.-f)*a.contact.position[2]+f*tip.contact.position[2],seed)
            };
            self.check_trim(&flank,p.parameters);
            seed = p.parameters;
            root.push(p.contact);
        }
        root
    }

    fn rim_section(&self, member: usize, rho: f64, n: usize) -> Vec<V> {
        self.rim_section_with_end(member,rho,n,None)
    }

    fn rim_section_with_end(&self,member: usize,rho: f64,n: usize,end: Option<&str>) -> Vec<V> {
        let pitch = TAU/self.teeth[member];
        let lower: Vec<V> = self.side_with_end(member,1,rho,n,end).iter().map(|c| c.position).collect();
        let index = rotate(2,if member == 0 { pitch } else { 0. },0.);
        let upper: Vec<V> = self.side_with_end(member,0,rho,n,end).iter().rev()
            .map(|c| index.point(c.position)).collect();
        let arc = |a: V,b: V| -> Vec<V> {
            let span = (azimuth(b)-azimuth(a)).rem_euclid(TAU);
            assert!(span > 0. && span < pitch,"member {member}: invalid tip/root width {span}");
            assert!((polar(a)-polar(b)).abs() < 1e-8);
            (1..n).map(|i| spherical(rho,polar(a),azimuth(a)+span*i as f64/n as f64)).collect()
        };
        let tip = arc(*lower.last().unwrap(),upper[0]);
        let root = arc(*upper.last().unwrap(),rotate(2,pitch,0.).point(lower[0]));
        let tooth: Vec<V> = lower.into_iter().chain(tip).chain(upper).chain(root).collect();
        let mut section = vec![];
        for i in 0..self.teeth[member] as usize {
            let r = rotate(2,i as f64*pitch,0.);
            section.extend(tooth.iter().map(|&p| r.point(p)));
        }
        // This fixture must have a single boundary at every azimuth. Reject an untrimmed
        // undercut or a crossed tip/root connection before trying to cap or mesh it.
        for i in 0..section.len() {
            let step = (azimuth(section[(i+1)%section.len()])-azimuth(section[i])).rem_euclid(TAU);
            assert!(step > 1e-12 && step < pitch,"member {member}: reversed contour {step}");
        }
        section
    }

    // A rim bounded by the true generated tooth surfaces, spherical toe/heel faces,
    // and a cone parallel to the pitch cone on its back. No arbitrary root blend.
    fn rim(&self, member: usize, n: usize) -> Rim {
        let mut points = vec![];
        let mut width = 0;
        let end_distance = self.ends.each_ref().map(|s| {
            let p = s.at(0.5,0.).unwrap().position;
            p[0].hypot(p[1]).hypot(p[2])
        });
        for i in 0..=n {
            let rho = end_distance[0]+(end_distance[1]-end_distance[0])*i as f64/n as f64;
            let end = if i == 0 { Some("toe") } else if i == n { Some("heel") } else { None };
            let section = self.rim_section_with_end(member,rho,n,end);
            width = section.len();
            let back_section = self.limits[member][2].line_on_sphere([0.;3],rho,0.).unwrap();
            assert_eq!(back_section.len(),1,"the declared back cone must have one section");
            let back = polar(self.local_frame(member).point(back_section[0].position));
            let rear: Vec<V> = section.iter().map(|&p| {
                assert!(back > 0. && back < polar(p));
                spherical(rho,back,azimuth(p))
            }).collect();
            points.extend(section);
            points.extend(rear);
        }
        let mut triangles = vec![];
        let vertex = |row,side,j| row*width*2+side*width+j%width;
        let mut quad = |a,b,c,d| { triangles.push([a,b,c]); triangles.push([a,c,d]); };
        for i in 0..n {
            for j in 0..width {
                quad(vertex(i,0,j),vertex(i,0,j+1),vertex(i+1,0,j+1),vertex(i+1,0,j));
                quad(vertex(i,1,j),vertex(i+1,1,j),vertex(i+1,1,j+1),vertex(i,1,j+1));
            }
        }
        for j in 0..width {
            quad(vertex(0,1,j),vertex(0,1,j+1),vertex(0,0,j+1),vertex(0,0,j));
            quad(vertex(n,1,j),vertex(n,0,j),vertex(n,0,j+1),vertex(n,1,j+1));
        }
        let shell = gcs_core::topology::ClosedShell::from_triangles(points.len(),&triangles)
            .unwrap_or_else(|e| panic!("invalid rim topology: {e:?}"));
        assert_eq!(shell.genus(),1,"the complete annular rim must be one toroidal shell");
        Rim {points,shell}
    }
}

struct Rim {
    points: Vec<V>,
    shell: gcs_core::topology::ClosedShell,
}

impl Rim {
    fn pieces(&self, pose: Motion) -> Vec<Piece> {
        self.shell.faces().iter().enumerate().map(|(face,_)| {
            let pts: Vec<V> = self.shell.loop_vertices(face,0).unwrap()
                .map(|i| pose.point(self.points[i])).collect();
            let a = nalgebra::Vector3::from_fn(|i,_| pts[1][i]-pts[0][i]);
            let b = nalgebra::Vector3::from_fn(|i,_| pts[2][i]-pts[0][i]);
            let n = a.cross(&b);
            assert!(n.norm() > 1e-12);
            Piece {pts,n: n.normalize().into(),path: "rim".into(),prim: 0,smooth: true}
        }).collect()
    }

}

#[test]
fn full_flanks_reach_the_addendum_without_folding_back_through_their_roots() {
    let pair = Pair::read([24,48],2.);
    for member in 0..2 {
        for side in 0..2 {
            for i in 0..=10 {
                let rho = (0.9+0.02*i as f64)*pair.rm;
                let points = pair.side(member,side,rho,40);
                for segment in points.windows(2) {
                    let [a,b] = [&segment[0],&segment[1]];
                    assert!(polar(b.position) > polar(a.position),
                        "folded root/flank: member {member}, side {side}, rho {rho}, {a:?} -> {b:?}");
                }
            }
        }
    }
}

#[test]
fn generated_rims_close_with_consistent_winding_and_converging_volume() {
    let pair = Pair::read([24,48],2.);
    for member in 0..2 {
        let mut volumes = vec![];
        for n in [4,8,16] {
            let rim = pair.rim(member,n);
            let pieces = rim.pieces(pair.body(member,0.));
            let volume = mesh::volume(&pieces);
            assert!(volume > 0.);
            volumes.push(volume);
            if n == 16 {
                let stl = mesh::checked_stl(&pieces,"experimental generated rim").unwrap();
                let encoded = mesh::stl_topology(&stl).unwrap();
                assert_eq!(encoded.genus(),1,"the encoded STL must retain the annular rim topology");
                if let Some(dir) = std::env::var_os("SOLVENT_GEAR_OUTPUT") {
                    let name = if member == 0 { "pinion.stl" } else { "gear.stl" };
                    std::fs::write(std::path::Path::new(&dir).join(name),stl).unwrap();
                }
            }
        }
        let coarse = (volumes[1]-volumes[0]).abs();
        let fine = (volumes[2]-volumes[1]).abs();
        assert!(fine < coarse*0.4,"member {member}: volumes {volumes:?}");
        assert!(fine/volumes[2] < 0.001,"member {member}: volumes {volumes:?}");
        eprintln!("member {member} rim volumes: {volumes:?}");
    }
}
