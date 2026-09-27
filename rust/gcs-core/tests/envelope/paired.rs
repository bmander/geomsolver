//! Matched references read from Solvent, with independent contact and indexing checks.
use super::*;
use gcs_core::{diagnose,library,modules,program,solid::RevolvedSurface,solve,syntax};
use std::f64::consts::{FRAC_PI_2,PI,TAU};

mod boundary;
mod analytic;
mod intervals;
mod faces;
mod caps;
mod swept;

struct Pair {
    model: program::Elaborated,
    teeth: [f64;2],
    module: f64,
    delta: [f64;2],
    rm: f64,
    offset: [f64;3],
    motion_families: [gcs_core::motion::Family;2],
    limits: [[RevolvedSurface;3];2],
    ends: [RevolvedSurface;2],
    regions: std::collections::BTreeMap<String,gcs_core::patch::TrimmedPatch>,
    seams: std::collections::BTreeMap<String,gcs_core::seam::EnvelopeSeam>,
    boundary_seams: std::collections::BTreeMap<String,gcs_core::seam::BoundarySeam>,
    surface_seams: std::collections::BTreeMap<String,gcs_core::seam::SurfaceSeam>,
    corners: std::collections::BTreeMap<String,gcs_core::vertex::SolvedVertex>,
    edges: std::collections::BTreeMap<String,gcs_core::edge::SpatialEdge>,
    faces: std::collections::BTreeMap<String,gcs_core::spatial_face::SpatialFaceBoundary>,
    /// Each generated flank's surface by what the checks call it, `faces.pinion_outer`: the
    /// source makes one per edge of each rack section's profile (`repeat e in …`), so its
    /// own name is a copy's (`faces.#<id>.<k>.pinion`), and the rack and the edge it is
    /// read by are the surface's own name and its edge's.
    flanks: std::collections::BTreeMap<String,usize>,
    labels: std::collections::BTreeMap<String,String>,
}

fn read_model(src: &str,teeth: [u32;2],module: f64) -> program::Elaborated {
    let (mut p,errors) = syntax::parse(&src);
    let link = modules::link(&mut p,&mut |name| match name {
        // The reference checks assume the common apex, so the offset is zero here.
        "configuration" => Some(format!("param pinion_teeth = {}\nparam gear_teeth = {}\n\
            param mean_module = {module}mm\nparam offset_angle = 0deg\nparam pressure_shift = 0deg\nparam spiral_angle = 35deg\n",teeth[0],teeth[1])),
        "paired_references" => Some(include_str!("../../../examples/spiral_bevel/paired_references.sv").into()),
        "verification" => Some(include_str!("../../../examples/spiral_bevel/verification.sv").into()),
        "matched_pair" => Some(include_str!("../../../examples/spiral_bevel/matched_pair.sv").into()),
        "cutters" => Some(include_str!("../../../examples/spiral_bevel/cutters.sv").into()),
        "reference" => Some(include_str!("../../../examples/spiral_bevel/reference.sv").into()),
        "boundaries" => Some(include_str!("../../../examples/spiral_bevel/boundaries.sv").into()),
        _ => library::resolve(name),
    });
    assert!(errors.is_empty() && link.is_empty(),"{errors:?} {link:?}");
    let mut model = program::elaborate(&p);
    assert!(model.ok(),"{:?}",model.diags);
    for i in 0..model.sketch.points.len() {
        for p in model.sketch.point_params(i) {
            if !model.sketch.params[p as usize].fixed {
                model.sketch.params[p as usize].value += module*0.001*(i as f64).sin();
            }
        }
    }
    // Shared analytic boundaries need tighter accuracy than interactive editing.
    // The solver owns method selection and retry; geometry still checks its own
    // position, normal and incidence errors independently of equation residuals.
    let solved = solve::solve(&mut model.sketch,solve::SolveOpts {
        tol:1e-16,acceptance_tol:1e-12,..Default::default()});
    assert!(solved.success,
        "teeth {teeth:?}, module {module}: {solved:?}");
    assert_eq!(diagnose::diagnose(&mut model.sketch,Default::default()).dof,0);
    model
}

impl Pair {
    // Only explicit CAD export entry points consume these overrides; ordinary
    // regression tests keep their declared configurations.
    fn read_cad_export() -> Self {
        let integer = |name,default: u32| std::env::var(name).map(|s|
            s.parse::<u32>().expect("CAD tooth count must be an integer")).unwrap_or(default);
        let teeth = [integer("SOLVENT_CAD_PINION_TEETH",24),integer("SOLVENT_CAD_GEAR_TEETH",48)];
        let module = std::env::var("SOLVENT_CAD_MODULE_MM").map(|s|
            s.parse::<f64>().expect("CAD module must be a number")).unwrap_or(2.);
        assert!(teeth.iter().all(|&n| n > 0) && module.is_finite() && module > 0.,
            "CAD tooth counts and module must be positive and finite");
        Self::read(teeth,module)
    }

    fn read(teeth: [u32;2], module: f64) -> Self {
        let src = include_str!("../../../examples/spiral_bevel/pair.sv");
        let model = read_model(src,teeth,module);
        let teeth = teeth.map(|n| n as f64);
        let delta = [teeth[0].atan2(teeth[1]),teeth[1].atan2(teeth[0])];
        let rm = module*teeth[0].hypot(teeth[1])/2.;
        let theta = (35f64-90.).to_radians();
        let offset = [rm-0.8*rm*theta.cos(),-0.8*rm*theta.sin(),0.];
        let motion_families = ["pair.pinion_generation","pair.gear_generation"].map(|name|
            gcs_core::motion::Family::read(&model.sketch,model.map.ent_named(name).unwrap().i()).unwrap());
        let read_boundary = |name: &str| RevolvedSurface::named(&model.sketch,
            model.map.ent_named(&format!("pair.{name}.wall")).unwrap().i()).unwrap();
        let ends = ["toe","heel"].map(read_boundary);
        let limits = ["pinion","gear"].map(|member| ["tip","root","back"].map(|role|
            read_boundary(&format!("{member}_{role}_boundary"))));
        let regions = model.sketch.patches.iter().enumerate().map(|(i,p)|
            (p.name.clone(),gcs_core::patch::TrimmedPatch::named(&model.sketch,i,module*1e-10).unwrap()))
            .collect();
        use gcs_core::seam::{kind,SeamKind};
        let seams = model.sketch.seams.iter().enumerate()
            .filter(|(i,_)| kind(&model.sketch,*i).unwrap() == SeamKind::Generating).map(|(i,s)|
            (s.name.clone(),gcs_core::seam::EnvelopeSeam::named(&model.sketch,i,
                gcs_core::seam::SeamTolerance {position:module*1e-8,normal:1e-8,axis:module*1e-10})
                .unwrap_or_else(|e| panic!("teeth {teeth:?}, module {module}: {e}")))).collect();
        let boundary_seams = model.sketch.seams.iter().enumerate()
            .filter(|(i,_)| kind(&model.sketch,*i).unwrap() == SeamKind::Boundary).map(|(i,s)|
                (s.name.clone(),gcs_core::seam::BoundarySeam::named(&model.sketch,i,module*1e-10)
                    .unwrap())).collect();
        let surface_seams = model.sketch.seams.iter().enumerate()
            .filter(|(i,_)| kind(&model.sketch,*i).unwrap() == SeamKind::Surfaces).map(|(i,s)|
                (s.name.clone(),gcs_core::seam::SurfaceSeam::named(&model.sketch,i).unwrap())).collect();
        let corners: std::collections::BTreeMap<_,_> = model.sketch.vertices.iter().enumerate().map(|(i,v)| {
            use gcs_core::vertex::{self,BoundaryVertex,JunctionVertex,VertexKind};
            let result = match vertex::kind(&model.sketch,i).unwrap() {
                VertexKind::Boundaries => {
                    let vertex = BoundaryVertex::named(&model.sketch,i,module*1e-10).unwrap();
                    let bounds = vertex.domain();
                    vertex.solve(bounds.map(|b| (b[0]+b[1])/2.),IntersectionOptions {
                        bounds,parameter_scale:[1.;3],residual_tolerance:[module*1e-10;3],max_iterations:100,
                    },module*1e-8)
                }
                VertexKind::Junction => {
                    let vertex = JunctionVertex::named(&model.sketch,i,gcs_core::seam::SeamTolerance {
                        position:module*1e-8,normal:1e-8,axis:module*1e-10}).unwrap();
                    let [_,v,t] = vertex.domain();
                    vertex.solve([v,t].map(|b| (b[0]+b[1])/2.),gcs_core::seam::SeamIntersectionOptions {
                        bounds:[v,t],parameter_scale:[1.;2],normal_tolerance:module*1e-10,
                        section_tolerance:module*1e-10,max_iterations:100,
                    },module*1e-8)
                }
            }.unwrap_or_else(|e| panic!("{}, module {module}: {e:?}",v.name));
            (v.name.clone(),result)
        }).collect();
        let edges = model.sketch.edges.iter().enumerate().map(|(i,e)| {
            let parameters = [e.start,e.end].map(|v| corners[&model.sketch.vertices[v as usize].name].parameters);
            let edge = gcs_core::edge::SpatialEdge::named(&model.sketch,i,parameters,gcs_core::edge::EdgeTolerance {
                junction:gcs_core::seam::SeamTolerance {position:module*1e-8,normal:1e-8,axis:module*1e-10},
                point:gcs_core::seam::BoundarySeamTolerance {
                    normal_velocity:module*1e-10,incidence:module*1e-10,trim:module*1e-8},
            }).unwrap_or_else(|error| panic!("{}: {error}",e.name));
            (e.name.clone(),edge)
        }).collect();
        let witnesses: Vec<_> = model.sketch.vertices.iter().map(|v| corners[&v.name].parameters).collect();
        let faces = model.sketch.faces.iter().enumerate().filter(|(_,f)| f.on().is_some()).map(|(i,f)| {
            let face = gcs_core::spatial_face::SpatialFaceBoundary::named(&model.sketch,i,&witnesses,
                gcs_core::edge::EdgeTolerance {
                    junction:gcs_core::seam::SeamTolerance {position:module*1e-8,normal:1e-8,axis:module*1e-10},
                    point:gcs_core::seam::BoundarySeamTolerance {
                        normal_velocity:module*1e-10,incidence:module*1e-10,trim:module*1e-8},
                }).unwrap_or_else(|e| panic!("{}: {e}",f.name));
            (f.name.clone(),face)
        }).collect();
        let mut flanks = std::collections::BTreeMap::new();
        let mut labels = std::collections::BTreeMap::new();
        for (i,s) in model.sketch.surfaces.iter().enumerate().filter(|(_,s)| s.name.starts_with("faces.")) {
            let rack = s.name.rsplit('.').next().unwrap();
            let edge = model.map.name_of(s.edge).unwrap();
            let label = format!("faces.{rack}_{}",edge.rsplit('.').next().unwrap());
            labels.insert(s.name.clone(),label.clone());
            assert!(flanks.insert(label,i).is_none(),"two flanks on {edge}");
        }
        Self {model,teeth,module,delta,rm,offset,motion_families,limits,ends,regions,seams,boundary_seams,surface_seams,corners,edges,faces,flanks,labels}
    }

    fn patch(&self, member: usize, side: usize, edge: &str) -> RevolvedSurface {
        let rack = if member == 0 { "pinion" }
            else if side == 0 { "gear_outer" } else { "gear_inner" };
        RevolvedSurface::named(&self.model.sketch,self.flanks[&format!("faces.{rack}_{edge}")]).unwrap()
    }

    /// What the checks call a flank, `faces.pinion_outer` — the stem of its seams' and faces'
    /// names, which the source writes out by edge.
    fn label(&self,surface: &RevolvedSurface) -> &str {
        &self.labels[&surface.name]
    }

    fn body(&self, member: usize, t: f64) -> Motion {
        let d = self.delta[member];
        let sign = if member == 0 { 1. } else { -1. };
        rotate(2,sign*t/d.sin(),sign/d.sin())
            .then(rotate(1,FRAC_PI_2-sign*d,0.))
    }

    fn region(&self,surface: &RevolvedSurface) -> &gcs_core::patch::TrimmedPatch {
        &self.regions[&format!("{}_region.bounded",surface.name)]
    }

    fn seam(&self,flank: &RevolvedSurface,root: bool) -> &gcs_core::seam::EnvelopeSeam {
        &self.seams[&format!("{}_{}join",self.label(flank),if root { "root_" } else { "" })]
    }

    fn seam_at(&self,member: usize,seam: &gcs_core::seam::EnvelopeSeam,rho: f64)
        -> envelope::Intersection {
        let [_,v,roll] = seam.domain();
        let bounds = [v,roll];
        let mut result = seam.intersect(|c| c.position[0].hypot(c.position[1]).hypot(c.position[2])-rho,
            bounds.map(|b| (b[0]+b[1])/2.),gcs_core::seam::SeamIntersectionOptions {
                bounds,parameter_scale:[1.;2],normal_tolerance:1e-10*self.module,
                section_tolerance:1e-10*self.module,max_iterations:100},
            1e-8*self.module).unwrap_or_else(|e| panic!("{}, rho {rho}: {e:?}",seam.name));
        let frame = self.local_frame(member);
        result.contact = envelope::Contact {position:frame.point(result.contact.position),
            normal:frame.vector(result.contact.normal),velocity:frame.vector(result.contact.velocity),
            normal_velocity:result.contact.normal_velocity};
        result
    }

    fn domain(&self,surface: &RevolvedSurface) -> [[f64;2];3] {
        let e = self.model.map.ent_named(&format!("{}_envelope",surface.name)).unwrap();
        let [u,v] = surface.domain();
        [u,v,self.model.sketch.envelopes[e.i()].roll]
    }

    fn check_trim(&self,surface: &RevolvedSurface,parameters: [f64;3]) {
        self.region(surface).envelope_at(parameters,1e-8*self.module,1e-8*self.module)
            .unwrap_or_else(|e| panic!("{} at {parameters:?}: {e:?}",surface.name));
    }

    fn local_frame(&self, member: usize) -> Motion {
        let sign = if member == 0 { 1. } else { -1. };
        rotate(1,FRAC_PI_2-sign*self.delta[member],0.).inverse()
    }

    fn motion(&self, member: usize, t: f64) -> Motion {
        // A coordinate convention only: geometry and rolling come from the source.
        self.motion_families[member].at(t).unwrap().then(self.local_frame(member))
    }

    fn intersection(&self, member: usize, patch: &RevolvedSurface,
        section: impl Fn([f64;3],&envelope::Contact) -> [f64;2],
        seed: [f64;3], options: IntersectionOptions) -> envelope::Intersection {
        let name = format!("{}_envelope",patch.name);
        let index = self.model.map.ent_named(&name).unwrap().i();
        let generated = envelope::GeneratedEnvelope::named(&self.model.sketch,index).unwrap();
        let frame = self.local_frame(member);
        let local = |c: &envelope::Contact| envelope::Contact {
            position:frame.point(c.position),normal:frame.vector(c.normal),
            velocity:frame.vector(c.velocity),normal_velocity:c.normal_velocity,
        };
        let mut result = generated.intersect(section,seed,options)
            .unwrap_or_else(|e| panic!("member {member}, {} at {seed:?}: {e:?}",patch.name));
        result.contact = local(&result.contact);
        result
    }

    fn at(&self, member: usize, patch: &RevolvedSurface, u: f64, rho: f64)
        -> envelope::Intersection {
        let [from,to] = patch.domain()[1];
        let v = (from+to)/2.;
        self.at_seed(member,patch,u,rho,[u,v,0.])
    }

    fn at_seed(&self, member: usize, patch: &RevolvedSurface, u: f64, rho: f64,
        mut seed: [f64;3]) -> envelope::Intersection {
        seed[0] = u;
        self.intersection(member,patch,
            |p,c| [p[0]-u,c.position[0].hypot(c.position[1]).hypot(c.position[2])-rho],
            seed,IntersectionOptions {
                bounds: [[u,u],self.domain(patch)[1],self.domain(patch)[2]],
                parameter_scale: [1.;3],
                residual_tolerance: [1e-10*self.module,1e-11,1e-10*self.module],
                max_iterations: 100,
            })
    }

    fn at_height(&self, member: usize, patch: &RevolvedSurface, height: f64, rho: f64)
        -> envelope::Intersection {
        let a = patch.at(0.,patch.domain()[1][0]).unwrap();
        self.at(member,patch,(height-a.position[2])/a.du[2],rho)
    }
}

#[test]
fn finite_edges_follow_independent_characteristics_and_declared_axial_slices() {
    for teeth in [[24,48],[32,32],[28,49]] {
        for module in [0.2,2.,25.4] {
            let pair = Pair::read(teeth,module);
            assert_eq!(pair.edges.len(),28);
            for member in 0..2 {
                let frame = pair.local_frame(member);
                for side in 0..2 {
                    let outer = member == side;
                    let name = if outer { "outer" } else { "inner" };
                    let flank = pair.patch(member,side,name);
                    let round = pair.patch(member,side,&format!("{name}_round"));
                    for role in ["tip","join","root","toe","heel","round_toe","round_heel"] {
                        let edge = &pair.edges[&format!("{}_{role}_span",pair.label(&flank))];
                        let source = if role == "root" || role.starts_with("round_") { &round } else { &flank };
                        let [a,b] = edge.endpoints().map(|p| frame.point(p.position));
                        for fraction in [0.,0.25,0.5,0.75,1.] {
                            let p = edge.sample(fraction,100)
                                .unwrap_or_else(|e| panic!("{} at {fraction}: {e:?}",edge.name));
                            let actual = frame.point(p.position);
                            let rho = if role.ends_with("toe") { 0.9*pair.rm }
                                else if role.ends_with("heel") { 1.1*pair.rm }
                                else { actual[0].hypot(actual[1]).hypot(actual[2]) };
                            assert!(rho >= 0.9*pair.rm-module*1e-8 && rho <= 1.1*pair.rm+module*1e-8);
                            let expected = pair.analytic(member,source,p.parameters[0],rho);
                            near(p.parameters,expected.parameters,1e-8);
                            near(actual,expected.contact.position,module*1e-7);
                            assert!((actual[2]-((1.-fraction)*a[2]+fraction*b[2])).abs() < module*1e-8);
                            if role == "tip" {
                                let angle = pair.delta[member]+(module*35f64.to_radians().cos()/rho).asin();
                                assert!((actual[0].hypot(actual[1]).atan2(actual[2])-angle).abs() < 1e-8);
                            } else if role == "join" || role == "root" {
                                assert_eq!(p.parameters[0],if outer { 1. } else { 0. });
                            }
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn declared_boundary_seams_match_independent_spheres_and_tip_cones_across_sizes_and_ratios() {
    for teeth in [[24,48],[32,32],[28,49]] {
        for module in [0.2,2.,25.4] {
            let pair = Pair::read(teeth,module);
            assert_eq!(pair.boundary_seams.len(),20);
            assert_eq!(pair.corners.len(),24);
            for member in 0..2 {
                for side in 0..2 {
                    let outer = member == side;
                    let edge = if outer { "outer" } else { "inner" };
                    let flank = pair.patch(member,side,edge);
                    let round = pair.patch(member,side,&format!("{edge}_round"));
                    let join = if outer { 1. } else { 0. };
                    for fraction in [0.9,1.,1.1] {
                        let rho = fraction*pair.rm;
                        // Independent scalar bracketing of the addendum cone on
                        // the closed-form characteristic; no boundary projector.
                        let theta = pair.delta[member]+(module*35f64.to_radians().cos()/rho).asin();
                        let at = |s| pair.analytic(member,&flank,join+s*(1.-2.*join),rho);
                        let polar = |p: [f64;3]| p[0].hypot(p[1]).atan2(p[2]);
                        let (mut lo,mut hi) = (0.,1.);
                        assert!(polar(at(lo).contact.position) < theta);
                        assert!(polar(at(hi).contact.position) > theta);
                        for _ in 0..48 {
                            let mid = (lo+hi)/2.;
                            if polar(at(mid).contact.position) < theta { lo = mid; } else { hi = mid; }
                        }
                        let expected = at((lo+hi)/2.);
                        let seed = pair.seam_at(member,pair.seam(&flank,false),rho).parameters;
                        let tip = pair.tip(member,&flank,rho,seed);
                        near(tip.parameters,expected.parameters,1e-8);
                        near(tip.contact.position,expected.contact.position,module*1e-7);
                        if fraction == 1. { continue; }
                        let end = if fraction < 1. { "toe" } else { "heel" };
                        let corner = pair.corners[&format!("{}_tip_{end}",pair.label(&flank))];
                        near(corner.parameters,expected.parameters,1e-8);
                        near(pair.local_frame(member).point(corner.position),expected.contact.position,module*1e-7);
                        for source in [&flank,&round] {
                            let seam = &pair.boundary_seams[&format!("{}_{end}_edge",pair.label(source))];
                            for s in [0.2,0.5,0.8] {
                                let u = if source.name == flank.name {
                                    join+s*(expected.parameters[0]-join)
                                } else { s };
                                let expected = pair.analytic(member,source,u,rho);
                                let frame = pair.local_frame(member);
                                let bounds = seam.domain();
                                let seed = std::array::from_fn(|i|
                                    (expected.parameters[i]+0.002).clamp(bounds[i][0],bounds[i][1]));
                                let point = seam.intersect(|c|
                                    frame.point(c.position)[2]-expected.contact.position[2],seed,
                                    IntersectionOptions {bounds,parameter_scale:[1.;3],
                                        residual_tolerance:[1e-10*module;3],max_iterations:100},
                                    1e-8*module).unwrap_or_else(|e| panic!("{}: {e:?}",seam.name));
                                near(point.parameters,expected.parameters,1e-8);
                                near(frame.point(point.contact.position),expected.contact.position,module*1e-7);
                                near(frame.vector(point.contact.normal),expected.contact.normal,1e-8);
                            }
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn declared_seams_agree_with_independent_characteristics_across_ratios_and_sizes() {
    for teeth in [[24,48],[32,32],[28,49]] {
        for module in [0.2,2.,25.4] {
            let pair = Pair::read(teeth,module);
            assert_eq!(pair.seams.len(),8);
            for member in 0..2 {
                for side in 0..2 {
                    let outer = member == side;
                    let edge = if outer { "outer" } else { "inner" };
                    let flank = pair.patch(member,side,edge);
                    for root in [false,true] {
                        let seam = pair.seam(&flank,root);
                        let source = pair.patch(member,side,
                            &if root { format!("{edge}_round") } else { edge.into() });
                        let u = if outer { 1. } else { 0. };
                        assert_eq!(seam.endpoint_parameters(),[u,1.-u]);
                        for fraction in [0.9,1.,1.1] {
                            let rho = fraction*pair.rm;
                            let actual = pair.seam_at(member,seam,rho);
                            let expected = pair.analytic(member,&source,u,rho);
                            near(actual.parameters,expected.parameters,1e-8);
                            near(actual.contact.position,expected.contact.position,module*1e-7);
                            near(actual.contact.normal,expected.contact.normal,1e-8);
                            if fraction != 1. {
                                let end = if fraction < 1. { "toe" } else { "heel" };
                                let role = if root { "root" } else { "join" };
                                let corner = pair.corners[&format!("{}_{role}_{end}",pair.label(&flank))];
                                near(corner.parameters,expected.parameters,1e-8);
                                near(pair.local_frame(member).point(corner.position),expected.contact.position,module*1e-7);
                            }
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn declarative_mates_have_opposing_normals_and_exact_half_pitch_spacing() {
    for teeth in [[24,48],[32,32],[28,49]] {
        for module in [0.2,2.,25.4] {
            let pair = Pair::read(teeth,module);
            let mut angles = [[0.;2];2];
            for (side,edges) in [["outer","inner"],["inner","outer"]].iter().enumerate() {
                let p = pair.patch(0,side,edges[0]);
                let g = pair.patch(1,side,edges[1]);
                for face in [0.9,1.,1.1] {
                    for height in [-0.2,0.,0.2] {
                        let a = pair.at_height(0,&p,height*module,face*pair.rm);
                        let b = pair.at_height(1,&g,height*module,face*pair.rm);
                        let t = a.parameters[2];
                        assert!((t-b.parameters[2]).abs() < 1e-8);
                        let ap = pair.body(0,t).point(a.contact.position);
                        let bp = pair.body(1,t).point(b.contact.position);
                        near(ap,bp,module*1e-7);
                        let an = pair.body(0,t).vector(a.contact.normal);
                        let bn = pair.body(1,t).vector(b.contact.normal);
                        near(an,bn.map(|x| -x),1e-7);
                        // Actual shaft velocities, independent of the crown-envelope residual.
                        let av = pair.body(0,t).velocity(a.contact.position);
                        let bv = pair.body(1,t).velocity(b.contact.position);
                        let relative_normal = (0..3).map(|k| an[k]*(av[k]-bv[k])).sum::<f64>();
                        assert!(relative_normal.abs() < module*1e-8,
                            "{teeth:?}, module {module}, side {side}, face {face}, height {height}: {relative_normal}");
                        if face == 1. && height == 0. {
                            for (m,c) in [a.contact,b.contact].iter().enumerate() {
                                angles[m][side] = c.position[1].atan2(c.position[0]);
                            }
                        }
                    }
                }
            }
            for member in 0..2 {
                let width = (angles[member][1]-angles[member][0]+PI).rem_euclid(TAU)-PI;
                assert!((width.abs()-PI/pair.teeth[member]).abs() < 1e-8,
                    "{teeth:?} module {module}: angular width {width}");
            }
        }
    }
}

#[test]
fn both_members_have_tangent_generated_root_transitions_on_both_sides() {
    let pair = Pair::read([24,48],2.);
    for member in 0..2 {
        for side in 0..2 {
            let outer = member == side;
            let edge = if outer { "outer" } else { "inner" };
            let flank = pair.patch(member,side,edge);
            let round = pair.patch(member,side,&format!("{edge}_round"));
            let tip = pair.patch(member,side,"tip");
            let join = if outer { 0. } else { 1. };
            for face in [0.9,1.,1.1] {
                let rho = face*pair.rm;
                let a = pair.at(member,&flank,1.-join,rho);
                let b = pair.at_seed(member,&round,join,rho,a.parameters);
                near(a.contact.position,b.contact.position,1e-7);
                near(a.contact.normal,b.contact.normal,1e-7);
                let mut previous = b.contact.position;
                let mut seed = b.parameters;
                let mut min_segment = f64::INFINITY;
                for i in 1..=40 {
                    let u = join+(1.-2.*join)*i as f64/40.;
                    let point = pair.at_seed(member,&round,u,rho,seed);
                    seed = point.parameters;
                    let c = point.contact;
                    let segment = (0..3).map(|k| (c.position[k]-previous[k]).powi(2))
                        .sum::<f64>().sqrt();
                    min_segment = min_segment.min(segment);
                    previous = c.position;
                }
                assert!(min_segment > 0.001,"member {member} side {side}: {min_segment}");
                let end = pair.at_seed(member,&round,1.-join,rho,seed);
                let root = pair.at_seed(member,&tip,join,rho,seed);
                near(end.contact.position,root.contact.position,1e-7);
                near(end.contact.normal,root.contact.normal,1e-7);
                let p = end.contact.position;
                let d = pair.delta[member];
                let depth = p[0].hypot(p[1])*d.cos()-p[2]*d.sin();
                let declared = tip.at(0.,tip.domain()[1][0]).unwrap().position[2].abs();
                assert!((depth+declared).abs() < 1e-7,"root depth {depth}, declared {declared}");
            }
        }
    }
}

#[test]
fn named_rolling_motions_match_independent_shaft_kinematics() {
    for teeth in [[24,48],[32,32],[28,49]] {
        let pair = Pair::read(teeth,2.);
        for member in 0..2 {
            for t in [-0.2,0.,0.3] {
                let old = rotate(2,t,1.).then(pair.body(member,t).inverse());
                let named = pair.motion(member,t);
                for p in [[0.;3],[1.,2.,3.],[-40.,11.,5.]] {
                    near(named.point(p),old.point(p),1e-8);
                    near(named.vector(p),old.vector(p),1e-8);
                    near(named.velocity(p),old.velocity(p),1e-8);
                }
            }
        }
    }
}

#[test]
fn declared_boundaries_match_independent_spheres_and_parallel_pitch_cones() {
    for teeth in [[24,48],[32,32],[28,49]] {
        for module in [0.2,2.,25.4] {
            let pair = Pair::read(teeth,module);
            let read = |name: &str| RevolvedSurface::named(&pair.model.sketch,
                pair.model.map.ent_named(&format!("pair.{name}.wall")).unwrap().i()).unwrap();
            let material = |name: &str| {
                let e = pair.model.map.ent_named(&format!("pair.{name}.wall")).unwrap();
                gcs_core::solid::RevolvedRegion::read(&pair.model.sketch,
                    pair.model.sketch.surfaces[e.i()].solid as usize,module*1e-10).unwrap()
            };
            for (name,fraction) in [("toe",0.9),("heel",1.1)] {
                let s = read(name);
                let projector = s.projector().unwrap();
                let region = material(name);
                for u in [0.,0.2,0.5,0.8,1.] {
                    for v in [0.,0.2,0.7,1.] {
                        let p = s.at(u,v).unwrap().position;
                        let r = p[0].hypot(p[1]).hypot(p[2]);
                        assert!((r-fraction*pair.rm).abs() < module*1e-8);
                        let on = projector.project(p).unwrap();
                        assert!(on.incidence_error < module*1e-8,"{name}: {on:?}");
                        for offset in [-0.01,0.01] {
                            let q = p.map(|x| x*(1.+offset*module/r));
                            let sample = region.classify(q,module*1e-10).unwrap();
                            assert!((sample.signed_distance-offset*module).abs() < module*1e-8);
                        }
                    }
                }
            }
            for member in 0..2 {
                let member_name = if member == 0 { "pinion" } else { "gear" };
                let d = pair.delta[member];
                for (role,depth) in [("tip",1.),("root",-1.25),("back",-4.)] {
                    let s = read(&format!("{member_name}_{role}_boundary"));
                    let projector = s.projector().unwrap();
                    let region = material(&format!("{member_name}_{role}_boundary"));
                    for u in [0.,0.3,0.8,1.] {
                        for v in [0.,0.2,0.6,1.] {
                            let sample = s.at(u,v).unwrap();
                            let p = pair.local_frame(member).point(sample.position);
                            let value = p[0].hypot(p[1])*d.cos()-p[2]*d.sin();
                            assert!((value-depth*module*35f64.to_radians().cos()).abs() < module*1e-8);
                            let on = projector.project(sample.position).unwrap();
                            assert!(on.incidence_error < module*1e-8,"{role}: {on:?}");
                            let normal = envelope::contact(sample,Motion::identity()).unwrap().normal;
                            near(on.normal,normal,1e-8);
                            if u > 0. && u < 1. {
                                let phi = p[1].atan2(p[0]);
                                let outward = [d.cos()*phi.cos(),d.cos()*phi.sin(),-d.sin()];
                                for offset in [-0.01,0.01] {
                                    let local = std::array::from_fn(|i| p[i]+offset*module*outward[i]);
                                    let q = pair.local_frame(member).inverse().point(local);
                                    let value = region.classify(q,module*1e-10).unwrap();
                                    assert!((value.signed_distance-offset*module).abs() < module*1e-8);
                                }
                            }
                        }
                    }
                    for fraction in [0.9,1.,1.1] {
                        let points = s.line_on_sphere([0.;3],fraction*pair.rm,0.31).unwrap();
                        assert_eq!(points.len(),1);
                        let p = points[0].position;
                        assert!((p[0].hypot(p[1]).hypot(p[2])-fraction*pair.rm).abs() < module*1e-8);
                    }
                }
            }
        }
    }
}

#[test]
fn named_boundary_intersections_find_tooth_corners_and_refuse_surface_continuations() {
    let pair = Pair::read([24,48],2.);
    for member in 0..2 {
        for side in 0..2 {
            let edge = if member == side { "outer" } else { "inner" };
            let patch = pair.patch(member,side,edge);
            let index = pair.model.map.ent_named(&format!("{}_envelope",patch.name)).unwrap().i();
            let generated = envelope::GeneratedEnvelope::named(&pair.model.sketch,index).unwrap();
            let tip = pair.limits[member][0].projector().unwrap();
            for end in 0..2 {
                let face = pair.ends[end].projector().unwrap();
                let p = pair.ends[end].at(0.5,0.).unwrap().position;
                let rho = p[0].hypot(p[1]).hypot(p[2]);
                let seed = pair.at(member,&patch,0.5,rho).parameters;
                let options = IntersectionOptions {
                    bounds:pair.domain(&patch),
                    parameter_scale:[1.;3],residual_tolerance:[1e-9*pair.module;3],max_iterations:100,
                };
                let corner = pair.region(&patch).intersect_boundaries([&tip,&face],seed,options,
                    1e-8*pair.module).unwrap();
                let expected = pair.tip(member,&patch,rho,seed);
                near(pair.local_frame(member).point(corner.contact.position),expected.contact.position,1e-7);
                if member == 0 && side == 0 && end == 0 {
                    // Same sphere equation, but a narrow angular patch away from the corner.
                    let mut sk = pair.model.sketch.clone();
                    let e = pair.model.map.ent_named("pair.toe.wall").unwrap();
                    let solid = sk.surfaces[e.i()].solid as usize;
                    let gcs_core::model::SolidDef::Revolve {sweep,..} = &mut sk.solids[solid].def else { unreachable!() };
                    sweep.value = 0.001;
                    let narrow = gcs_core::solid::SurfaceProjector::named(&sk,e.i()).unwrap();
                    assert!(narrow.project(corner.contact.position).unwrap().incidence_error > 0.01);
                    assert_eq!(generated.intersect_boundaries([&tip,&narrow],seed,options).unwrap_err(),
                        envelope::Error::OutsideDomain);
                }
            }
        }
    }
}
