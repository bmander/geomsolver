//! Complete member material: a finite blank minus every indexed generating sweep.
use super::*;
use gcs_core::{solid::{SpatialField,MaterialField,MaterialEvaluator},motion::{Family,MotionBounds}};

mod walk;
mod cad;

fn axis(pair: &Pair,name: &str) -> ([f64;3],[f64;3]) {
    let sk = &pair.model.sketch;
    let line = &sk.lines[pair.model.map.ent_named(name).unwrap().i()];
    let origin = sk.world_point(line.p1 as usize);
    let end = sk.world_point(line.p2 as usize);
    (origin,std::array::from_fn(|k| end[k]-origin[k]))
}

fn blank(pair: &Pair,member: usize) -> (SpatialField,String) {
    let b = blank_parts(pair,member); (b.field,b.definition)
}

struct Blank {field:SpatialField,angular:SpatialField,definition:String,radii:[f64;2]}

fn blank_parts(pair: &Pair,member: usize) -> Blank {
    let name = ["pinion","gear"][member];
    let (origin,axis) = axis(pair,&format!("pair.{name}_axis"));
    let length = axis[0].hypot(axis[1]).hypot(axis[2]);
    let unit = axis.map(|v| v/length);
    let coordinates = |p: [f64;3]| {
        let q: [f64;3] = std::array::from_fn(|k| p[k]-origin[k]);
        let z: f64 = (0..3).map(|k| q[k]*unit[k]).sum();
        let radial: [f64;3] = std::array::from_fn(|k| q[k]-z*unit[k]);
        [radial[0].hypot(radial[1]).hypot(radial[2]),z]
    };
    let cone = |role: usize| {
        let surface = &pair.limits[member][role];
        let a = coordinates(surface.at(0.,surface.domain()[1][0]).unwrap().position);
        let b = coordinates(surface.at(1.,surface.domain()[1][0]).unwrap().position);
        let normal = [b[1]-a[1],a[0]-b[0]];
        (PlanarField::half_plane(a,normal).unwrap(),format!("{{\"half_plane\":{{\"through\":{a:?},\"normal\":{normal:?}}}}}"))
    };
    let radii = pair.ends.each_ref().map(|s| {
        let p = coordinates(s.at(0.5,0.).unwrap().position); p[0].hypot(p[1])
    });
    assert!(radii[0] < radii[1]);
    let (tip,tip_def) = cone(0); let (back,back_def) = cone(2);
    let [toe,heel] = radii;
    let angular = SpatialField::from(RevolvedField::new(tip.clone().difference(back.clone()).unwrap(),origin,axis).unwrap());
    let section = PlanarField::disk([0.;2],heel).unwrap()
        .difference(PlanarField::disk([0.;2],radii[0]).unwrap()).unwrap()
        .intersection(tip).unwrap().difference(back).unwrap();
    let profile = format!("{{\"difference\":[{{\"intersection\":[{{\"difference\":[{{\"disk\":{{\"center\":[0,0],\"radius\":{heel}}}}},{{\"disk\":{{\"center\":[0,0],\"radius\":{toe}}}}}]}},{tip_def}]}},{back_def}]}}");
    Blank {field:SpatialField::from(RevolvedField::new(section,origin,axis).unwrap()),angular,radii,
        definition:format!("{{\"origin\":{origin:?},\"axis\":{axis:?},\"profile\":{profile}}}")}
}

struct Member {
    blank: SpatialField,
    field: MaterialEvaluator,
    // Continuation of the same tooth-facing field through the spherical ends.
    // Used only to locate candidate intersections with those end boundaries.
    section: MaterialEvaluator,
    radii: [f64;2],
    // Certificate schema lists every declared index, even when identical query
    // boxes share one core evaluation (notably at the apex).
    indices: Vec<MotionBounds>,
    definition: String,
}

struct MaterialQuery {
    value: I,
    blank: I,
    sweeps: Vec<minimum::Minimum>,
    positive_covers: Vec<Option<PositiveCover>>,
}

#[derive(Clone)]
struct PositiveCover {lower:f64,intervals:Vec<[f64;2]>}

impl MaterialQuery {
    fn row(&self,p: [f64;3],label: &str,inside: bool) -> String {
        let value = self.value.bounds(); let blank = self.blank.bounds();
        let sweeps: Vec<_> = self.sweeps.iter().zip(&self.positive_covers).map(|(s,cover)| {
            let value = s.value.bounds();
            let cover = if inside {
                let cover = cover.as_ref().expect("retained material needs every positive roll cover");
                format!("{{\"lower\":{},\"intervals\":{:?}}}",cover.lower,cover.intervals)
            } else { "null".into() };
            format!("{{\"minimum\":{value:?},\"roll_witness\":{},\"evaluations\":{},\"status\":\"{:?}\",\"positive_cover\":{cover}}}",s.witness,s.evaluations,s.status)
        }).collect();
        format!("{{\"label\":\"{label}\",\"position_mm\":{p:?},\"expected_inside\":{inside},\"blank\":{blank:?},\"material\":{value:?},\"sweeps\":[{}]}}",sweeps.join(","))
    }
}

impl Member {
    fn read(pair: &Pair,member: usize) -> Self {
        let (generator,definition) = if member == 0 {
            let (field,definition) = functional_generator(pair,0);
            (SpatialField::from(field),definition)
        } else { let space = closure::GearSpace::read(pair); (space.field,space.definition) };
        let name = ["pinion","gear"][member];
        let id = pair.model.map.ent_named(&format!("pair.{name}_roll")).unwrap().i();
        let mut sk = pair.model.sketch.clone();
        let mut index_definitions = vec![];
        let (origin,axis) = axis(pair,&format!("pair.{name}_axis"));
        let indices: Vec<_> = (0..pair.teeth[member] as usize).map(|index| {
            let gcs_core::model::MotionDef::Rotation {ratio,phase,..} = &mut sk.motions[id].def else { panic!() };
            *ratio = 0.; *phase = TAU*index as f64/pair.teeth[member];
            index_definitions.push(format!("{{\"origin\":{origin:?},\"axis\":{axis:?},\"ratio\":0,\"phase\":{phase}}}"));
            Family::read(&sk,id).unwrap()
        }).collect();
        let patch = pair.patch(member,0,if member == 0 { "outer" } else { "inner" });
        let Blank {field:blank,angular,definition:blank_definition,radii} = blank_parts(pair,member);
        let motion_definition = functional_motion_definition(pair,member);
        let domain = pair.domain(&patch)[2];
        let teeth = pair.teeth[member] as usize;
        let definition = format!("{{\"member\":\"{name}\",\"teeth\":{teeth},\"blank\":{blank_definition},\"generator\":{definition},\"motion_definition\":{motion_definition},\"roll_domain\":{domain:?},\"indices\":[{}]}}",index_definitions.join(","));
        let sweep = MaterialField::from(SweptField::new(generator,pair.motion_families[member].clone(),
            I::new(domain[0],domain[1]).unwrap()));
        let field = indices.iter().fold(MaterialField::from(blank.clone()),|field,index|
            field.difference(sweep.clone().transformed(index,0.).unwrap()).unwrap()).evaluator(1_000_000);
        let section = indices.iter().fold(MaterialField::from(angular),|field,index|
            field.difference(sweep.clone().transformed(index,0.).unwrap()).unwrap()).evaluator(1_000_000);
        let indices = indices.iter().map(|index| index.bounds(I::ZERO).unwrap()).collect();
        Self {blank,field,section,radii,indices,definition}
    }

    // Options apply to each indexed roll minimization. Every index contributes
    // an enclosure even when its refinement budget is exhausted. A whole input
    // box is propagated through indexing, motion and field composition.
    fn bounds(&mut self,p: [I;3],options: Options) -> MaterialQuery {
        self.query(p,options,None)
    }

    fn query(&mut self,p: [I;3],options: Options,band: Option<I>) -> MaterialQuery {
        let blank = self.blank.bounds(p).unwrap();
        let mut positive: Vec<Vec<([f64;2],f64)>> = vec![];
        let observe = |query: usize,domain: I,bound: I| {
            positive.resize_with(query+1,Vec::new);
            let domain = domain.bounds();
            if bound.bounds()[0] > 0. && domain[0] < domain[1] { positive[query].push((domain,bound.bounds()[0])); }
        };
        let result = if let Some(band) = band {
            self.field.bounds_outside_with_observer(p,band,options,observe)
        } else { self.field.bounds_with_observer(p,options,observe) }.unwrap();
        assert_eq!(positive.len(),result.sweeps.len());
        let covers: Vec<_> = result.sweeps.iter().zip(positive).map(|(query,positive)| {
            let domain = query.domain.bounds();
            let found = query.minimum;
            let lower = found.value.bounds()[0];
            if lower > 0. {
                // Certify a useful positive margin, without claiming the tight
                // reported minimum has itself been independently reproduced.
                let lower = if lower*0.5 > 0. { lower*0.5 } else { lower };
                positive_cover(domain,positive.into_iter().filter_map(|(d,b)| (b >= lower).then_some(d)).collect())
                    .map(|intervals| PositiveCover {lower,intervals})
            } else { None }
        }).collect();
        let mut sweeps = vec![]; let mut positive_covers = vec![];
        for index in &self.indices {
            let indexed = index.inverse_point(p).unwrap().map(|v| v.bounds().map(f64::to_bits));
            let i = result.sweeps.iter().position(|query|
                query.point_box.map(|v| v.bounds().map(f64::to_bits)) == indexed).unwrap();
            sweeps.push(result.sweeps[i].minimum);
            positive_covers.push(covers[i].clone());
        }
        MaterialQuery {value:result.value,blank,sweeps,positive_covers}
    }
}

fn point(p: [f64;3]) -> [I;3] { p.map(|v| I::point(v).unwrap()) }

// Explicit export experiment. Ordinary member checks do not pay for a complete
// boundary extraction; setting the output path runs the same core field through
// spatial certification and encoded-STL topology/quantization checks.
fn export_boundary(material: &mut Member,path: std::path::PathBuf) {
    use gcs_core::solid::BoundaryOptions;
    let tolerance = std::env::var("SOLVENT_MEMBER_BOUNDARY_TOLERANCE").map_or(20.,|v| v.parse().unwrap());
    let max_cells = std::env::var("SOLVENT_MEMBER_BOUNDARY_CELLS").map_or(100000,|v| v.parse().unwrap());
    eprintln!("extracting complete pinion field: tolerance {tolerance} mm, {max_cells} cell budget");
    let start = std::time::Instant::now();
    let mut previous = None; let mut last = start;
    let result = material.field.boundary_with_observer(BoundaryOptions {spatial_tolerance:tolerance,max_depth:16,max_cells,
        sweep:Options {value_tolerance:0.0002,max_evaluations:20000}},|stage,completed| {
            if previous != Some(stage) || last.elapsed().as_secs() >= 5 {
                eprintln!("pinion {stage:?}: {completed} cells, {:.1}s",start.elapsed().as_secs_f64());
                previous = Some(stage); last = std::time::Instant::now();
            }
        });
    let mesh = result.unwrap_or_else(|e| {
        if let gcs_core::solid::BoundaryError::Topology {error,components} = &e {
            panic!("field boundary refused after {:?}: {error:?}, {} components; largest: {:?}",
                start.elapsed(),components.len(),components.iter().take(8).collect::<Vec<_>>());
        }
        panic!("field boundary refused after {:?}: {e:?}",start.elapsed());
    });
    let pieces: Vec<_> = mesh.triangles().iter().map(|t| gcs_core::csg::Piece {
        pts:t.iter().map(|&i| mesh.vertices()[i]).collect(),n:[0.;3],path:"pinion".into(),prim:0,smooth:false,
    }).collect();
    let bytes = gcs_core::mesh::checked_stl(&pieces,"experimental functional pinion").unwrap();
    assert_eq!(u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize,mesh.triangles().len());
    let encoded = gcs_core::mesh::stl_topology(&bytes).unwrap();
    assert_eq!(encoded.genus(),1,"the pinion's through-opening must remain");
    assert_eq!(encoded.genus(),mesh.shell().genus());
    let mut quantization = 0_f64;
    for p in mesh.vertices() {
        let mut squared = I::ZERO;
        for &v in p { squared = squared.add(I::point(v).unwrap().sub(I::point(f64::from(v as f32)).unwrap()).unwrap().square().unwrap()).unwrap(); }
        quantization = quantization.max(I::new(squared.bounds()[0].max(0.),squared.bounds()[1]).unwrap().sqrt().unwrap().bounds()[1]);
    }
    let error = I::point(mesh.spatial_error_bound()).unwrap().add(I::point(quantization).unwrap()).unwrap().bounds()[1];
    assert!(error <= tolerance,"encoded distance bound {error} exceeds requested {tolerance}");
    let triangles = mesh.triangles().len(); let cells = mesh.cells().len(); let seconds = start.elapsed().as_secs_f64();
    let definition = &material.definition;
    std::fs::write(path.with_extension("json"),format!("{{\"units\":\"mm\",\"definition\":{definition},\"triangles\":{triangles},\"partition_cells\":{cells},\"encoded_spatial_error_bound_mm\":{error},\"source_error\":\"not certified\",\"scope\":\"experimental distance-bounded field export; source accuracy, geometric embedding and mating verification incomplete\",\"seconds\":{seconds}}}")).unwrap();
    std::fs::write(path,bytes).unwrap();
    eprintln!("functional pinion: {triangles} triangles, {cells} cells, {error} mm encoded distance bound, {seconds}s");
}

// Retain whole oracle intervals whose lower bound is strictly positive. These
// candidates need not partition the domain; choose a closed overlapping cover.
// The independent checker recomputes their bounds and exact coverage, rather
// than trusting the primary refiner's heap, pruning history or reported minimum.
fn positive_cover(domain: [f64;2],mut candidates: Vec<[f64;2]>) -> Option<Vec<[f64;2]>> {
    if domain[0] >= domain[1] || candidates.iter().any(|c|
        !(domain[0] <= c[0] && c[0] < c[1] && c[1] <= domain[1])) { return None; }
    candidates.sort_by(|a,b| a[0].total_cmp(&b[0]).then_with(|| b[1].total_cmp(&a[1])));
    let mut end = domain[0]; let mut i = 0; let mut cover = vec![];
    while end < domain[1] {
        let mut best = None;
        let mut next = end;
        while i < candidates.len() && candidates[i][0] <= end {
            if candidates[i][1] > next { next = candidates[i][1]; best = Some(candidates[i]); }
            i += 1;
        }
        cover.push(best?); end = next;
    }
    Some(cover)
}

#[test]
fn positive_roll_cover_accepts_overlap_but_never_fills_a_gap() {
    let domain = [0.,1.];
    assert_eq!(positive_cover(domain,vec![[0.,0.75],[0.25,1.],[0.1,0.2]]),Some(vec![[0.,0.75],[0.25,1.]]));
    assert!(positive_cover(domain,vec![[0.,0.5],[0.5_f64.next_up(),1.]]).is_none());
    assert!(positive_cover(domain,vec![[0.,0.9]]).is_none());
    assert!(positive_cover(domain,vec![[-0.1,1.]]).is_none());
}

#[test]
fn isolated_coarse_pinion_sample_connects_continuously_into_the_rim() {
    let pair = Pair::read([24,48],2.);
    let mut member = Member::read(&pair,0);
    // Center of the isolated occupied grid sample found by the 20 mm export
    // experiment. This is a witness location, not an edit to the generating solid.
    let p = [34.576585140122586,-7.409268244311984,46.92536554730923];
    let frame = pair.local_frame(0);
    let local = frame.point(p);
    let rho = local[0].hypot(local[1]).hypot(local[2]);
    let phi = local[1].atan2(local[0]);
    let mn = pair.module*35_f64.to_radians().cos();
    let theta = pair.delta[0]+(-3.*mn/rho).asin();
    let q = frame.inverse().point([rho*theta.sin()*phi.cos(),rho*theta.sin()*phi.sin(),rho*theta.cos()]);
    let options = Options {value_tolerance:0.0002,max_evaluations:20000};
    let a = member.field.bounds(point(p),options).unwrap().value;
    let b = member.field.bounds(point(q),options).unwrap().value;
    eprintln!("coarse fragment path {p:?} -> {q:?}; endpoint bounds {a:?}, {b:?}");
    assert!(a.bounds()[1] < 0. && b.bounds()[1] < 0.);
    let delta = std::array::from_fn::<_,3,_>(|k| I::point(q[k]).unwrap().sub(I::point(p[k]).unwrap()).unwrap());
    let length_squared = delta.iter().try_fold(I::ZERO,|sum,d| sum.add(d.square()?)).unwrap();
    let length = I::new(length_squared.bounds()[0].max(0.),length_squared.bounds()[1]).unwrap().sqrt().unwrap();
    // Bound the maximum member field along the entire straight segment. Each
    // center is an interval enclosure of the exact affine point, and the field's
    // one-Lipschitz contract supplies the complete segment-cell enclosure.
    let result = minimum::enclose(I::new(0.,1.).unwrap(),|t| {
        let [lo,hi] = t.bounds(); let mid = lo*0.5+hi*0.5;
        let mut center = [I::ZERO;3];
        for k in 0..3 { center[k] = I::point(p[k]).unwrap().add(delta[k].mul(I::point(mid).unwrap()).unwrap()).unwrap(); }
        let value = member.field.bounds(center,options)?.value;
        let dt = t.sub(I::point(mid).unwrap()).unwrap().bounds();
        let travel = length.mul(I::point(dt[0].abs().max(dt[1].abs())).unwrap()).unwrap().bounds()[1];
        Ok::<_,gcs_core::solid::SweepError>(value.add(I::new(-travel,travel).unwrap()).unwrap().neg())
    },Options {value_tolerance:0.005,max_evaluations:1024}).unwrap();
    eprintln!("complete path material margin: {result:?}");
    assert!(result.value.bounds()[0] > 0.,"the isolated sample must have a proven material connection to the rim");
}

#[test]
fn band_queries_classify_member_boxes_with_less_roll_refinement() {
    let pair = Pair::read([24,48],2.);
    let mut member = Member::read(&pair,0);
    let band = I::new(-0.1,0.1).unwrap();
    let options = Options {value_tolerance:0.0002,max_evaluations:100000};
    let mut full_work = 0; let mut band_work = 0;
    for (p,inside) in [([34.576585140122586,-7.409268244311984,46.92536554730923],true),
        ([39.422487402678954,-5.865393179148338,43.172816417932836],true),([0.;3],false),([70.,0.,0.],false)] {
        let box_p = p.map(|v| I::new(v-1e-5,v+1e-5).unwrap());
        let full = member.field.bounds(box_p,options).unwrap();
        let fast = member.field.bounds_outside(box_p,band,options).unwrap();
        assert!(fast.value.bounds()[0] <= full.value.bounds()[0] && fast.value.bounds()[1] >= full.value.bounds()[1]);
        if inside { assert!(fast.value.bounds()[1] < -0.1); }
        else { assert!(fast.value.bounds()[0] > 0.1); }
        for s in &fast.sweeps {
            let b = s.separation_band.unwrap().bounds(); let v = s.minimum.value.bounds();
            if s.minimum.status == Status::Separated { assert!(v[1] < b[0] || v[0] > b[1]); }
        }
        full_work += full.sweeps.iter().map(|s| s.minimum.evaluations).sum::<usize>();
        band_work += fast.sweeps.iter().map(|s| s.minimum.evaluations).sum::<usize>();
    }
    eprintln!("pinion box classification: {full_work} full-refinement evaluations, {band_work} band evaluations");
    assert!(band_work < full_work);
}

#[test]
fn functional_blanks_match_independent_spherical_and_conical_limits() {
    let pair = Pair::read([24,48],2.);
    for member in 0..2 {
        let field = blank(&pair,member).0;
        let frame = pair.local_frame(member).inverse();
        let d = pair.delta[member]; let mn = pair.module*35_f64.to_radians().cos();
        for rho in [0.89,0.9,0.95,1.,1.05,1.1,1.11].map(|r| r*pair.rm) {
            for depth in [-4.1,-4.,-3.,-1.,0.,1.,1.1].map(|v| v*mn) {
                let theta = d+(depth/rho).asin();
                for phi in [0_f64,0.7,2.8] {
                    let p = frame.point([rho*theta.sin()*phi.cos(),rho*theta.sin()*phi.sin(),rho*theta.cos()]);
                    let expected = (rho-1.1*pair.rm).max(0.9*pair.rm-rho).max(depth-mn).max(-4.*mn-depth);
                    let value = field.bounds(point(p)).unwrap().bounds();
                    assert!(value[0] <= expected+1e-10 && value[1] >= expected-1e-10,"member {member}: {value:?}, {expected}");
                    if expected.abs() > 1e-8 { assert_eq!(value[0] > 0.,expected > 0.); assert_eq!(value[1] < 0.,expected < 0.); }
                }
            }
        }
    }
}

#[test]
fn complete_members_bound_material_across_flanks_blanks_and_indexing() {
    let pair = Pair::read([24,48],2.);
    let options = Options {value_tolerance:pair.module*1e-4,max_evaluations:100000};
    let displacement = pair.module*0.005;
    let mut count = 0;
    let mut cases = vec![];
    for member in 0..2 {
        let mut material = Member::read(&pair,member);
        assert!(material.field.support_bounds().unwrap().is_some());
        let mut rows = vec![];
        let mut retained = None;
        let frame = pair.local_frame(member).inverse();
        for side in 0..2 {
            let edge = if member == side { "outer" } else { "inner" };
            let flank = pair.patch(member,side,edge);
            let tip = pair.tip(member,&flank,pair.rm,pair.analytic(member,&flank,0.5,pair.rm).parameters);
            let join = pair.seam(&flank,false).endpoint_parameters()[0];
            let c = pair.analytic(member,&flank,(tip.parameters[0]+join)/2.,pair.rm);
            let s = pair.model.map.ent_named(&flank.name).unwrap().i();
            let region = RevolvedRegion::read(&pair.model.sketch,pair.model.sketch.surfaces[s].solid as usize,pair.module*1e-10).unwrap();
            let plus = std::array::from_fn(|k| c.contact.position[k]+displacement*c.contact.normal[k]);
            let sign = region.classify(pair.motion_families[member].at(c.parameters[2]).unwrap().inverse().point(frame.point(plus)),0.).unwrap().signed_distance.signum();
            for offset in [-displacement,displacement] {
                let p = frame.point(std::array::from_fn(|k| c.contact.position[k]+sign*offset*c.contact.normal[k]));
                let query = material.bounds(point(p),options);
                assert!(query.sweeps.iter().all(|s| s.status == Status::Converged));
                let [lo,hi] = query.value.bounds();
                // Positive generator offset is retained body material.
                if offset > 0. { assert!(hi < 0.,"member {member} side {side}: {lo}, {hi}"); }
                else { assert!(lo > 0.,"member {member} side {side}: {lo}, {hi}"); }
                if offset > 0. { retained = Some((p,query.value)); }
                rows.push(query.row(p,&format!("flank_{side}"),offset > 0.));
                count += 1;
            }
        }
        // The back lies below every intended root. Probe retained rim and
        // beyond both spherical ends at that same angle.
        let mn = pair.module*35_f64.to_radians().cos();
        let theta = pair.delta[member]+(-3.*mn/pair.rm).asin();
        for (fraction,inside) in [(0.89,false),(1.,true),(1.11,false)] {
            let rho = fraction*pair.rm;
            let p = frame.point([rho*theta.sin(),0.,rho*theta.cos()]);
            let query = material.bounds(point(p),options);
            assert!(query.sweeps.iter().all(|s| s.status == Status::Converged));
            let [lo,hi] = query.value.bounds();
            if inside { assert!(hi < 0.,"member {member}: lost rim {lo}, {hi}"); }
            else { assert!(lo > 0.,"member {member}: ignored blank {lo}, {hi}"); }
            rows.push(query.row(p,"rim_and_ends",inside));
            count += 1;
        }
        let back_theta = pair.delta[member]+(-4.1*mn/pair.rm).asin();
        let opening = frame.point([pair.rm*back_theta.sin(),0.,pair.rm*back_theta.cos()]);
        for (label,p) in [("outside_back",opening),("apex_opening",[0.;3])] {
            let query = material.bounds(point(p),options);
            assert!(query.sweeps.iter().all(|s| s.status == Status::Converged));
            assert!(query.value.bounds()[0] > 0.);
            rows.push(query.row(p,label,false));
            count += 1;
        }
        let (p,fine) = retained.unwrap();
        let box_query = material.bounds(p.map(|v| I::new(v-1e-5,v+1e-5).unwrap()),options);
        assert!(box_query.sweeps.iter().all(|s| s.status == Status::Converged));
        assert!(box_query.value.bounds()[1] < 0.,"lost retained point box");
        assert!(box_query.value.bounds()[0] <= fine.bounds()[1] && box_query.value.bounds()[1] >= fine.bounds()[0]);
        let coarse = material.bounds(point(p),Options {max_evaluations:4,..options});
        assert!(coarse.sweeps.iter().any(|s| s.status == Status::BudgetExhausted));
        assert!(coarse.value.bounds()[0] <= fine.bounds()[0] && coarse.value.bounds()[1] >= fine.bounds()[1]);
        let definition = &material.definition;
        cases.push(format!("{{\"definition\":{definition},\"points\":[{}]}}",rows.join(",\n")));
        if member == 0 { if let Some(path) = std::env::var_os("SOLVENT_MEMBER_BOUNDARY_OUTPUT") {
            export_boundary(&mut material,path.into());
        } }
    }
    assert_eq!(count,18);
    if let Some(path) = std::env::var_os("SOLVENT_MEMBER_MATERIAL_OUTPUT") {
        let tolerance = options.value_tolerance;
        std::fs::write(path,format!("{{\"schema\":5,\"units\":{{\"length\":\"mm\",\"angle\":\"rad\"}},\"source_error\":\"not certified\",\"value_tolerance_mm\":{tolerance},\"cases\":[{}]}}\n",cases.join(",\n"))).unwrap();
    }
}
