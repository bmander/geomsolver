//! Contour tracing on spherical sections; the shell module uses a structured
//! chart with edge refinement, not an advancing-front surface march.
//! These contours are candidates, not a complete surface or topology certificate.
use super::*;

mod shell;

type V = [f64;3];

fn distance(a: V,b: V) -> f64 { (a[0]-b[0]).hypot(a[1]-b[1]).hypot(a[2]-b[2]) }
fn segment_distance(p: V,a: V,b: V) -> f64 {
    let d: V = std::array::from_fn(|k| b[k]-a[k]);
    let squared: f64 = d.iter().map(|x| x*x).sum();
    let t = if squared > 0. { ((0..3).map(|k| (p[k]-a[k])*d[k]).sum::<f64>()/squared).clamp(0.,1.) } else { 0. };
    distance(p,std::array::from_fn(|k| a[k]+t*d[k]))
}

#[derive(Clone)]
struct Sample {theta:f64,point:V,value:I}

#[derive(Clone)]
struct Crossing {phi:f64,theta:f64,point:V,inside:Sample,outside:Sample,radius:f64}

struct Walk<'a> {
    pair:&'a Pair,
    member:usize,
    material:Member,
    rho:f64,
    queries:usize,
    roll_evaluations:usize,
    point_tolerance:f64,
    chord_tolerance:f64,
    max_midpoint_deviation:f64,
    corrections:usize,
    continue_through_ends:bool,
}

impl Walk<'_> {
    fn position(&self,phi: f64,theta: f64) -> V {
        let local = [self.rho*theta.sin()*phi.cos(),self.rho*theta.sin()*phi.sin(),self.rho*theta.cos()];
        self.pair.local_frame(self.member).inverse().point(local)
    }

    fn sample(&mut self,phi: f64,theta: f64) -> Sample {
        assert!(self.queries < 50000,"surface walk exhausted its query budget");
        let p = self.position(phi,theta);
        let field = if self.continue_through_ends { &mut self.material.section } else { &mut self.material.field };
        let query = field.query(point(p),Stop::Outside(I::ZERO),
            Options {value_tolerance:0.0002,max_evaluations:20000},None).unwrap();
        self.queries += 1;
        self.roll_evaluations += query.sweeps.iter().map(|q| q.minimum.evaluations).sum::<usize>();
        Sample {theta,point:p,value:query.value}
    }

    fn crossing(&mut self,phi: f64,guess: f64) -> Crossing {
        let mn = self.pair.module*35_f64.to_radians().cos();
        let delta = self.pair.delta[self.member];
        // Search the tooth-facing boundary from the retained rim toward the
        // exterior above the tip. Every bracket is checked against ALL cutters.
        // A single-valued radial chart is an experimental candidate assumption;
        // this bracket does not assert there are no other crossings or components.
        let lo = delta+(-3.*mn/self.rho).asin();
        let hi = delta+(1.2*mn/self.rho).asin();
        let guess = guess.clamp(lo,hi);
        let mut width = self.point_tolerance/self.rho;
        let (mut a,mut b) = loop {
            let a = self.sample(phi,(guess-width).max(lo));
            let b = self.sample(phi,(guess+width).min(hi));
            if a.value.bounds()[1] < 0. && b.value.bounds()[0] > 0. { break (a,b); }
            assert!(a.theta > lo || b.theta < hi,
                "no retained-to-exterior bracket: member {} phi {phi}: {:?}, {:?}",self.member,a.value,b.value);
            width *= 2.;
        };
        for _ in 0..40 {
            let p = self.position(phi,a.theta*0.5+b.theta*0.5);
            let mut radius = 0_f64;
            for endpoint in [a.point,b.point] {
                let mut squared = I::ZERO;
                for k in 0..3 {
                    squared = squared.add(I::point(endpoint[k]).unwrap().sub(I::point(p[k]).unwrap()).unwrap().square().unwrap()).unwrap();
                }
                radius = radius.max(I::new(squared.bounds()[0].max(0.),squared.bounds()[1]).unwrap().sqrt().unwrap().bounds()[1]);
            }
            // Continuity between strict signs establishes a real material
            // boundary within this outward-rounded distance of the chart point.
            if radius <= self.point_tolerance {
                return Crossing {phi,theta:a.theta*0.5+b.theta*0.5,point:p,inside:a,outside:b,radius};
            }
            let mut shrunk = false;
            let left = a.theta; let span = b.theta-left;
            for fraction in [0.5,0.25,0.75] {
                let theta = left+span*fraction;
                if theta <= a.theta || theta >= b.theta { continue; }
                let c = self.sample(phi,theta); self.corrections += 1;
                if c.value.bounds()[1] < 0. { a = c; shrunk = true; }
                else if c.value.bounds()[0] > 0. { b = c; shrunk = true; }
                if shrunk { break; }
            }
            assert!(shrunk,"ambiguous correction at member {} phi {phi}",self.member);
        }
        panic!("surface correction exhausted its subdivision limit");
    }

    fn refine(&mut self,a: &Crossing,b: &Crossing,depth: usize,out: &mut Vec<Crossing>) {
        let mid = self.crossing(a.phi*0.5+b.phi*0.5,a.theta*0.5+b.theta*0.5);
        let deviation = segment_distance(mid.point,a.point,b.point)+mid.radius+a.radius.max(b.radius);
        if deviation > self.chord_tolerance {
            assert!(depth < 16,"surface edge refinement exhausted its depth");
            self.refine(a,&mid,depth+1,out); self.refine(&mid,b,depth+1,out);
        } else {
            self.max_midpoint_deviation = self.max_midpoint_deviation.max(deviation);
            out.push(b.clone());
        }
    }

    fn trace(&mut self,full: bool) -> Vec<Crossing> {
        let flank = self.pair.patch(self.member,0,if self.member == 0 { "outer" } else { "inner" });
        let tip = self.pair.tip(self.member,&flank,self.rho,
            self.pair.analytic(self.member,&flank,0.5,self.rho).parameters);
        let join = self.pair.seam(&flank,false).endpoint_parameters()[0];
        let seed = self.pair.analytic(self.member,&flank,(tip.parameters[0]+join)*0.5,self.rho);
        let p = seed.contact.position; // Pair's analytic workbench returns member-local coordinates.
        let phi = p[1].atan2(p[0]); let theta = p[0].hypot(p[1]).atan2(p[2]);
        let first = self.crossing(phi,theta);
        let mut result = vec![first.clone()];
        let teeth = if full { self.pair.teeth[self.member] as usize } else { 1 };
        let steps = teeth*4;
        let span = TAU*teeth as f64/self.pair.teeth[self.member];
        for i in 1..=steps {
            let previous = result.last().unwrap().clone();
            let next_phi = phi+span*i as f64/steps as f64;
            let guess = if result.len() > 1 {
                let a = &result[result.len()-2];
                previous.theta+(previous.theta-a.theta)*(next_phi-previous.phi)/(previous.phi-a.phi)
            } else { previous.theta };
            let next = if full && i == steps {
                // Periodic chart seam uses the same actual field witness and
                // vertex identity, not a proximity weld of two rounded points.
                Crossing {phi:next_phi,..first.clone()}
            } else { self.crossing(next_phi,guess) };
            self.refine(&previous,&next,0,&mut result);
        }
        result
    }
}

#[test]
fn surface_following_tracks_indexed_pinion_and_gear_sections() {
    let pair = Pair::read([24,48],2.);
    let output = std::env::var_os("SOLVENT_SURFACE_WALK_OUTPUT");
    let full = output.is_some();
    let tolerance = if full { 2. } else { 0.1 };
    let mut cases = vec![];
    for member in 0..2 {
        let mut walk = Walk {pair:&pair,member,material:Member::read(&pair,member),rho:pair.rm,
            queries:0,roll_evaluations:0,point_tolerance:tolerance*0.05,chord_tolerance:tolerance,
            max_midpoint_deviation:0.,corrections:0,continue_through_ends:false};
        let start = std::time::Instant::now();
        let contour = walk.trace(full);
        let seconds = start.elapsed().as_secs_f64();
        let reference: Vec<_> = pair.rim_section(member,pair.rm,24).into_iter()
            .map(|p| pair.local_frame(member).inverse().point(p)).collect();
        let mut maximum_reference_distance = 0_f64;
        for c in &contour {
            assert!(c.inside.value.bounds()[1] < 0. && c.outside.value.bounds()[0] > 0.);
            assert!(c.radius <= walk.point_tolerance);
            let d = (0..reference.len()).map(|i| segment_distance(c.point,reference[i],reference[(i+1)%reference.len()]))
                .fold(f64::INFINITY,f64::min);
            maximum_reference_distance = maximum_reference_distance.max(d);
            assert!(d <= walk.point_tolerance+pair.module*0.01,
                "member {member}, phi {}, reference distance {d}",c.phi);
        }
        assert!(walk.max_midpoint_deviation <= tolerance);
        if full { assert_eq!(contour.first().unwrap().point,contour.last().unwrap().point); }
        eprintln!("surface walk member {member}: {} edges, {} field queries, {} roll evaluations, {seconds:.3}s; midpoint deviation {}, analytic-reference distance {maximum_reference_distance}",
            contour.len()-1,walk.queries,walk.roll_evaluations,walk.max_midpoint_deviation);
        let rows: Vec<_> = contour.iter().map(|c| format!("{{\"position\":{:?},\"phi\":{},\"inside\":{{\"point\":{:?},\"value\":{:?}}},\"outside\":{{\"point\":{:?},\"value\":{:?}}},\"boundary_distance_bound_mm\":{}}}",
            c.point,c.phi,c.inside.point,c.inside.value.bounds(),c.outside.point,c.outside.value.bounds(),c.radius)).collect();
        cases.push(format!("{{\"definition\":{},\"rho_mm\":{},\"closed\":{full},\"field_queries\":{},\"roll_evaluations\":{},\"seconds\":{seconds},\"points\":[{}]}}",
            walk.material.definition,walk.rho,walk.queries,walk.roll_evaluations,rows.join(",")));
    }
    if let Some(path) = output {
        std::fs::write(path,format!("{{\"units\":\"mm\",\"chord_refinement_target_mm\":{tolerance},\"scope\":\"candidate mean-distance surface contours with bounded vertex crossings; whole-edge error, missing components, radial-chart uniqueness and complete surface extraction are not certified\",\"cases\":[{}]}}",cases.join(","))).unwrap();
    }
}
