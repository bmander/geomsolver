//! Candidate annular shell. The chart supplies connectivity, not a proof that
//! the full material has this topology or no additional boundary components.
use super::*;
use std::collections::BTreeMap;
use gcs_core::{csg::Piece,mesh,topology::ClosedShell};

#[derive(Clone)]
struct Vertex {parameter:V,point:V,uncertainty:f64}

struct Candidate {vertices:Vec<Vertex>,triangles:Vec<[usize;3]>}

fn edge(a:usize,b:usize) -> [usize;2] { [a.min(b),a.max(b)] }

fn midpoint(a:V,b:V) -> V {
    let mut p = std::array::from_fn(|k| a[k]*0.5+b[k]*0.5);
    // The annular chart is periodic in azimuth only. Identity at its seam is
    // shared from construction onward, without coordinate-tolerance welding.
    if (a[1]-b[1]).abs() > 0.5 { p[1] = (p[1]+0.5)%1.; }
    p
}

impl Candidate {
    fn new(n:usize,at:&mut impl FnMut(V)->Vertex) -> Self {
        assert!(n >= 4);
        let mut vertices = vec![];
        for u in [0.,1.] { for w in [0.,1.] { for j in 0..n {
            vertices.push(at([u,j as f64/n as f64,w]));
        } } }
        let index = |u:usize,w:usize,j:usize| u*2*n+w*n+j%n;
        let mut triangles = vec![];
        let mut quad = |a,b,c,d| { triangles.push([a,b,c]); triangles.push([a,c,d]); };
        for j in 0..n {
            quad(index(0,1,j),index(0,1,j+1),index(1,1,j+1),index(1,1,j));
            quad(index(0,0,j),index(1,0,j),index(1,0,j+1),index(0,0,j+1));
            quad(index(0,0,j),index(0,0,j+1),index(0,1,j+1),index(0,1,j));
            quad(index(1,0,j),index(1,1,j),index(1,1,j+1),index(1,0,j+1));
        }
        Self {vertices,triangles}
    }

    fn refine(&mut self,tolerance:f64,at:&mut impl FnMut(V)->Vertex) -> (usize,f64) {
        let mut samples = BTreeMap::new();
        for iteration in 0..16 {
            let mut split = BTreeMap::new();
            let mut maximum = 0_f64;
            let mut worst = (0_f64,[0;2]);
            for t in &self.triangles { for k in 0..3 {
                let key = edge(t[k],t[(k+1)%3]);
                if split.contains_key(&key) { continue; }
                let [a,b] = key.map(|i| &self.vertices[i]);
                let m = samples.entry(key).or_insert_with(|| at(midpoint(a.parameter,b.parameter)));
                // A sampled refinement indicator, not a whole-edge error bound.
                let deviation = segment_distance(m.point,a.point,b.point)
                    +m.uncertainty+a.uncertainty.max(b.uncertainty);
                if deviation > worst.0 { worst = (deviation,key); }
                if deviation > tolerance { split.insert(key,m.clone()); }
                else { maximum = maximum.max(deviation); }
            } }
            if split.is_empty() { return (iteration,maximum); }
            // Close refinement requests over longest edges. Otherwise splitting
            // a short edge can keep replacing a long opposite diagonal with
            // another almost-as-long diagonal, indefinitely. Requests propagate
            // through shared edge identities before any triangles are replaced.
            loop {
                let mut extra = vec![];
                for t in &self.triangles {
                    let keys = std::array::from_fn::<_,3,_>(|k| edge(t[k],t[(k+1)%3]));
                    if keys.iter().any(|key| split.contains_key(key)) {
                        let longest = *keys.iter().max_by(|a,b| {
                            let length = |e:&&[usize;2]|
                                distance(self.vertices[e[0]].point,self.vertices[e[1]].point);
                            length(a).total_cmp(&length(b))
                        }).unwrap();
                        if !split.contains_key(&longest) { extra.push(longest); }
                    }
                }
                if extra.is_empty() { break; }
                for key in extra { split.insert(key,samples[&key].clone()); }
            }
            eprintln!("candidate round {iteration}: {} triangles, {} split edges; worst {} at {:?}",
                self.triangles.len(),split.len(),worst.0,worst.1.map(|i| self.vertices[i].parameter));
            assert!(self.vertices.len()+split.len() < 20000,"candidate vertex budget exhausted");
            let mut indices = BTreeMap::new();
            for (key,v) in split {
                indices.insert(key,self.vertices.len()); self.vertices.push(v);
            }
            let mut triangles = vec![];
            for &t in &self.triangles {
                let cuts = std::array::from_fn::<_,3,_>(|k| indices.get(&edge(t[k],t[(k+1)%3])).copied());
                match cuts.iter().filter(|c| c.is_some()).count() {
                    0 => triangles.push(t),
                    1 => {
                        let k = cuts.iter().position(Option::is_some).unwrap();
                        let [a,b,c] = std::array::from_fn(|j| t[(k+j)%3]);
                        let m = cuts[k].unwrap();
                        triangles.extend([[a,m,c],[m,b,c]]);
                    }
                    2 => {
                        let k = (cuts.iter().position(Option::is_none).unwrap()+1)%3;
                        let [a,b,c] = std::array::from_fn(|j| t[(k+j)%3]);
                        let x = cuts[k].unwrap(); let y = cuts[(k+1)%3].unwrap();
                        triangles.push([b,y,x]);
                        // Keep the new diagonal short after refinement closure.
                        let length = |i:usize,j:usize|
                            distance(self.vertices[i].point,self.vertices[j].point);
                        if length(a,y) < length(x,c) {
                            triangles.extend([[a,x,y],[a,y,c]]);
                        } else { triangles.extend([[a,x,c],[x,y,c]]); }
                    }
                    3 => {
                        let [a,b,c] = t; let [ab,bc,ca] = cuts.map(Option::unwrap);
                        triangles.extend([[a,ab,ca],[ab,b,bc],[ca,bc,c],[ab,bc,ca]]);
                    }
                    _ => unreachable!(),
                }
            }
            self.triangles = triangles;
        }
        panic!("candidate refinement iteration budget exhausted");
    }

    fn checked_pieces(&self) -> Vec<Piece> {
        let shell = ClosedShell::from_triangles(self.vertices.len(),&self.triangles).unwrap();
        assert_eq!(shell.genus(),1);
        self.triangles.iter().map(|t| Piece {
            pts:t.iter().map(|&i| self.vertices[i].point).collect(),n:[0.;3],
            path:"candidate".into(),prim:0,smooth:false,
        }).collect()
    }
}

struct Chart<'a> {
    walk:Walk<'a>,
    top:BTreeMap<[u64;2],Crossing>,
    back:BTreeMap<u64,f64>,
}

impl Chart<'_> {
    fn at(&mut self,parameter:V) -> Vertex {
        let [u,v,w] = parameter;
        let [toe,heel] = self.walk.material.radii;
        self.walk.rho = toe*(1.-u)+heel*u;
        let phi = TAU*v;
        let back = *self.back.entry(u.to_bits()).or_insert_with(|| {
            let points = self.walk.pair.limits[self.walk.member][2]
                .line_on_sphere([0.;3],self.walk.rho,0.).unwrap();
            assert_eq!(points.len(),1);
            let p = self.walk.pair.local_frame(self.walk.member).point(points[0].position);
            p[0].hypot(p[1]).atan2(p[2])
        });
        if w == 0. {
            return Vertex {parameter,point:self.walk.position(phi,back),uncertainty:0.};
        }
        let key = [u.to_bits(),v.to_bits()];
        if !self.top.contains_key(&key) {
            let guess = self.top.iter().min_by(|(a,_),(b,_)| {
                let metric = |k:&[u64;2]| {
                    // Tooth periodicity selects a predictor only. Every new
                    // point still gets strict signs from ALL indexed sweeps;
                    // no witness or interval is copied across tooth indices.
                    let period = 1./self.walk.pair.teeth[self.walk.member];
                    let dv = (v-f64::from_bits(k[1])).rem_euclid(period);
                    (u-f64::from_bits(k[0])).abs()+dv.min(period-dv)
                };
                metric(a).total_cmp(&metric(b))
            }).map_or(self.walk.pair.delta[self.walk.member],|(_,c)| c.theta);
            let crossing = self.walk.crossing(phi,guess);
            self.top.insert(key,crossing);
        }
        let top = &self.top[&key];
        assert!(back > 0. && top.theta > back);
        let point = if w == 1. { top.point }
            else { self.walk.position(phi,back*(1.-w)+top.theta*w) };
        Vertex {parameter,point,uncertainty:top.radius}
    }
}

#[test]
fn adaptive_shell_refinement_preserves_the_annular_surface() {
    let mut at = |parameter:V| {
        let [u,v,w] = parameter; let phi = TAU*v; let r = 3.+w;
        Vertex {parameter,point:[r*phi.cos(),r*phi.sin(),u],uncertainty:0.}
    };
    let mut candidate = Candidate::new(8,&mut at);
    let initial = candidate.triangles.len();
    let (iterations,maximum) = candidate.refine(0.02,&mut at);
    assert!(iterations > 0 && candidate.triangles.len() > initial);
    assert!(maximum <= 0.02);
    let pieces = candidate.checked_pieces();
    let volume = mesh::volume(&pieces);
    assert!((volume-7.*std::f64::consts::PI).abs() < 0.2);
    let stl = mesh::checked_stl(&pieces,"annular refinement fixture").unwrap();
    assert_eq!(mesh::stl_topology(&stl).unwrap().genus(),1);
}

#[test]
fn adaptive_shell_refinement_converges_across_twisted_ridges() {
    let mut at = |parameter:V| {
        let [u,v,w] = parameter; let phi = TAU*v;
        let r = 3.+w*(1.+0.4*(TAU*(4.*v+0.7*u)).sin());
        Vertex {parameter,point:[r*phi.cos(),r*phi.sin(),10.*u],uncertainty:0.}
    };
    let mut candidate = Candidate::new(16,&mut at);
    let (iterations,maximum) = candidate.refine(0.05,&mut at);
    assert!(iterations > 0 && maximum <= 0.05);
    let pieces = candidate.checked_pieces();
    // Integrating r_outer^2-r_inner^2 over azimuth removes the sine term;
    // the sine-squared contribution is 0.4^2/2 at every height.
    let expected = 70.8*std::f64::consts::PI;
    assert!((mesh::volume(&pieces)-expected).abs()/expected < 0.015);
    assert_eq!(mesh::stl_topology(&mesh::checked_stl(&pieces,"twisted ridges fixture").unwrap()).unwrap().genus(),1);
}

#[test]
fn export_surface_following_pair_candidates() {
    let Some(output) = std::env::var_os("SOLVENT_SURFACE_SHELL_OUTPUT") else { return; };
    let output = std::path::PathBuf::from(output);
    std::fs::create_dir_all(&output).unwrap();
    let pair = Pair::read([24,48],2.);
    let tolerance = std::env::var("SOLVENT_SURFACE_SHELL_TOLERANCE")
        .map_or(2.,|s| s.parse::<f64>().unwrap());
    assert!(tolerance.is_finite() && tolerance > 0.);
    for member in 0..2 {
        let name = ["pinion","gear"][member];
        let start = std::time::Instant::now();
        let walk = Walk {pair:&pair,member,material:Member::read(&pair,member),rho:pair.rm,
            queries:0,roll_evaluations:0,point_tolerance:tolerance*0.05,chord_tolerance:tolerance,
            max_midpoint_deviation:0.,corrections:0,continue_through_ends:true};
        let mut chart = Chart {walk,top:BTreeMap::new(),back:BTreeMap::new()};
        let mut candidate = Candidate::new(pair.teeth[member] as usize*4,&mut |p| chart.at(p));
        let (iterations,maximum) = candidate.refine(tolerance,&mut |p| chart.at(p));
        let pieces = candidate.checked_pieces();
        let volume = mesh::volume(&pieces); assert!(volume > 0.);
        let stl = mesh::checked_stl(&pieces,"surface-following candidate; not certified").unwrap();
        assert_eq!(mesh::stl_topology(&stl).unwrap().genus(),1);
        assert_eq!(u32::from_le_bytes(stl[80..84].try_into().unwrap()) as usize,candidate.triangles.len());
        let seconds = start.elapsed().as_secs_f64();
        let walk = &chart.walk;
        eprintln!("surface shell {name}: {} triangles, {} vertices, {iterations} refinement iterations, {} field queries, {} roll evaluations, {seconds:.3}s, midpoint indicator {maximum}, volume {volume}",
            candidate.triangles.len(),candidate.vertices.len(),walk.queries,walk.roll_evaluations);
        let vertices:Vec<_> = candidate.vertices.iter().map(|v|
            format!("{{\"parameter\":{:?},\"position\":{:?}}}",v.parameter,v.point)).collect();
        let crossings:Vec<_> = chart.top.iter().map(|(key,c)| format!(
            "{{\"uv\":{:?},\"position\":{:?},\"inside\":{{\"point\":{:?},\"value\":{:?}}},\"outside\":{{\"point\":{:?},\"value\":{:?}}},\"continuation_boundary_distance_bound_mm\":{}}}",
            key.map(f64::from_bits),c.point,c.inside.point,c.inside.value.bounds(),c.outside.point,c.outside.value.bounds(),c.radius)).collect();
        let report = format!("{{\"units\":\"mm\",\"definition\":{},\"refinement_target_mm\":{tolerance},\"maximum_midpoint_indicator_mm\":{maximum},\"refinement_iterations\":{iterations},\"field_queries\":{},\"roll_evaluations\":{},\"seconds\":{seconds},\"volume_mm3\":{volume},\"vertices\":[{}],\"triangles\":{:?},\"continuation_crossings\":[{}],\"scope\":\"candidate annular shell with checked mesh and encoded-STL topology; chart uniqueness, missing components, whole-edge and surface error, embedding, source accuracy and mating not certified; crossing witnesses refer to the same field with only spherical toe and heel clipping omitted, not the final clipped field\"}}",
            walk.material.definition,walk.queries,walk.roll_evaluations,vertices.join(","),candidate.triangles,crossings.join(","));
        std::fs::write(output.join(format!("{name}-candidate.json")),report).unwrap();
        std::fs::write(output.join(format!("{name}-candidate.stl")),stl).unwrap();
    }
}
