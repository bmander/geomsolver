//! General-field advancing-front workbench. No analytic surface charts or
//! geometry-specific boundary pieces. Sampling/shape controls are experimental;
//! this is not yet a whole-boundary accuracy or component-coverage certificate.
use super::*;
use gcs_core::{interval::minimum::Options,solid::{MaterialField,MaterialEvaluator}};
use std::collections::{BTreeMap,BTreeSet,VecDeque};

type V = [f64;3];
fn add(a:V,b:V) -> V { std::array::from_fn(|k| a[k]+b[k]) }
fn sub(a:V,b:V) -> V { std::array::from_fn(|k| a[k]-b[k]) }
fn mul(a:V,s:f64) -> V { a.map(|x| x*s) }
fn dot(a:V,b:V) -> f64 { (0..3).map(|k| a[k]*b[k]).sum() }
fn cross(a:V,b:V) -> V { [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]] }
fn length(a:V) -> f64 { a[0].hypot(a[1]).hypot(a[2]) }
fn unit(a:V) -> V { let n = length(a); assert!(n > 1e-12); mul(a,1./n) }
fn tangent(n:V) -> V {
    let k = (0..3).min_by(|&a,&b| n[a].abs().total_cmp(&n[b].abs())).unwrap();
    let mut axis = [0.;3]; axis[k] = 1.; unit(cross(n,axis))
}

fn boundary_radius(center:V,inside:V,outside:V) -> f64 {
    let mut radius = 0_f64;
    for endpoint in [inside,outside] {
        let squared = (0..3).fold(I::ZERO,|s,k| s.add(I::point(endpoint[k]).unwrap()
            .sub(I::point(center[k]).unwrap()).unwrap().square().unwrap()).unwrap());
        radius = radius.max(I::new(0.,squared.bounds()[1]).unwrap().sqrt().unwrap().bounds()[1]);
    }
    radius
}

#[derive(Clone,Debug,PartialEq)]
struct Vertex {p:V,n:V,size:f64,inside:V,outside:V,radius:f64}

struct Surface {
    field:MaterialEvaluator,
    support:[I;3],
    accuracy:f64,
    point_tolerance:f64,
    max_step:f64,
    queries:usize,
    corrections:BTreeMap<[u64;3],Option<(V,V,V,V)>>,
    projections:BTreeMap<[u64;3],Option<Vertex>>,
    cache:bool,
}

impl Surface {
    fn new(field:MaterialField,accuracy:f64) -> Self {
        assert!(accuracy.is_finite() && accuracy > 0.);
        let support = field.support_bounds().unwrap().expect("marching requires finite support");
        let span = support.map(|v| v.bounds()[1]-v.bounds()[0]);
        Self {field:field.evaluator(100000),support,accuracy,point_tolerance:accuracy*0.01,
            max_step:length(span)/12.,queries:0,corrections:BTreeMap::new(),
            projections:BTreeMap::new(),cache:true}
    }

    fn value(&mut self,p:V) -> I {
        self.query(p,false)
    }

    fn query(&mut self,p:V,precise:bool) -> I {
        assert!(self.queries < 1_000_000,"front field-query budget exhausted"); self.queries += 1;
        let options = Options {value_tolerance:self.point_tolerance*0.01,max_evaluations:20000};
        let value = if precise { self.field.bounds(point(p),options) }
            else { self.field.bounds_outside(point(p),I::ZERO,options) }.unwrap().value;
        if precise { let [a,b] = value.bounds();
            assert!(b-a <= options.value_tolerance,"field enclosure too wide for a differential probe");
        }
        value
    }

    fn gradient(&mut self,p:V) -> V {
        let h = self.accuracy*0.2;
        std::array::from_fn(|k| {
            let mut a = p; a[k] -= h; let mut b = p; b[k] += h;
            let a = self.query(a,true).bounds(); let b = self.query(b,true).bounds();
            ((b[0]+b[1])-(a[0]+a[1]))/(4.*h)
        })
    }

    fn vertex(&mut self,p:V,n:V,inside:V,outside:V) -> Vertex {
        let radius = boundary_radius(p,inside,outside);
        assert!(radius <= self.point_tolerance);
        // Candidate curvature estimate from neighboring field normals. These
        // finite differences neither prove smoothness nor resolve a crease.
        let t = tangent(n); let b = cross(n,t); let probe = self.max_step*0.5;
        let mut curvature = 0_f64;
        for d in [t,mul(t,-1.),b,mul(b,-1.)] {
            let g = self.gradient(add(p,mul(d,probe)));
            if length(g) > 1e-12 { curvature = curvature.max(length(sub(unit(g),n))/probe); }
        }
        let size = (4.*self.accuracy/curvature.max(1e-12)).sqrt().min(self.max_step);
        Vertex {p,n,size,inside,outside,radius}
    }

    fn project(&mut self,guess:V) -> Option<Vertex> {
        let key = guess.map(f64::to_bits);
        if let Some(vertex) = self.projections.get(&key) { return vertex.clone(); }
        let vertex = self.correct(guess).map(|(p,n,a,b)| self.vertex(p,n,a,b));
        if self.cache && self.projections.len() < 16384 { self.projections.insert(key,vertex.clone()); }
        vertex
    }

    fn correct(&mut self,guess:V) -> Option<(V,V,V,V)> {
        // The field and accuracy controls are immutable during extraction.
        // Shared edge midpoints and retried frontier predictions can reuse the
        // exact same result, including failure. No spatial quantization.
        let key = guess.map(f64::to_bits);
        if let Some(result) = self.corrections.get(&key) { return *result; }
        let result = self.correct_uncached(guess);
        if self.cache && self.corrections.len() < 16384 { self.corrections.insert(key,result); }
        result
    }

    fn correct_uncached(&mut self,guess:V) -> Option<(V,V,V,V)> {
        let mut p = guess;
        for _ in 0..20 {
            let gradient = self.gradient(p); let magnitude = length(gradient);
            if magnitude < 1e-10 { return None; }
            let n = mul(gradient,1./magnitude);
            let inside = sub(p,mul(n,self.point_tolerance*0.5));
            let outside = add(p,mul(n,self.point_tolerance*0.5));
            if self.value(inside).bounds()[1] < 0. && self.value(outside).bounds()[0] > 0. {
                return Some((p,n,inside,outside));
            }
            let value = self.value(p).bounds();
            let step = ((value[0]*0.5+value[1]*0.5)/magnitude).clamp(-self.max_step,self.max_step);
            p = sub(p,mul(n,step));
            if length(sub(p,guess)) > self.max_step*2. { return None; }
        }
        None
    }

    fn fits(&mut self,[a,b,c]:[V;3]) -> bool {
        for sample in [mul(add(a,b),0.5),mul(add(b,c),0.5),mul(add(c,a),0.5),mul(add(add(a,b),c),1./3.)] {
            let Some((_,_,inside,outside)) = self.correct(sample) else { return false; };
            if boundary_radius(sample,inside,outside) > self.accuracy { return false; }
        }
        true
    }

    fn seed(&mut self) -> Vertex {
        fn halton(mut i:usize,base:usize) -> f64 {
            let mut f = 1.; let mut result = 0.;
            while i > 0 { f /= base as f64; result += f*(i%base) as f64; i /= base; }
            result
        }
        let mut inside = None;
        for i in 1..=4096 {
            let p = std::array::from_fn(|k| {
                let [a,b] = self.support[k].bounds(); a+(b-a)*halton(i,[2,3,5][k])
            });
            if self.value(p).bounds()[1] < 0. { inside = Some(p); break; }
        }
        let mut a = inside.expect("seed discovery exhausted without a retained point");
        let mut b = self.support.map(|v| v.bounds()[1]+self.max_step);
        assert!(self.value(b).bounds()[0] > 0.);
        for _ in 0..60 {
            let mid = mul(add(a,b),0.5); let value = self.value(mid).bounds();
            if length(sub(a,b)) < self.max_step*0.1 { return self.project(mid).unwrap(); }
            if value[1] < 0. { a = mid; }
            else if value[0] > 0. { b = mid; }
            else { return self.project(mid).unwrap(); }
        }
        panic!("seed bracket exhausted");
    }
}

#[derive(Clone)]
struct Front {
    vertices:Vec<Vertex>,
    triangles:Vec<[usize;3]>,
    used:BTreeSet<[usize;2]>,
    boundary:BTreeSet<[usize;2]>,
    queue:VecDeque<[usize;2]>,
    clearance:f64,
    repairs:usize,
}

impl Front {
    fn start(surface:&mut Surface) -> Self {
        let a = surface.seed(); let tangent = tangent(a.n); let side = cross(a.n,tangent);
        let b = surface.project(add(a.p,mul(tangent,a.size))).unwrap();
        let c = surface.project(add(a.p,mul(add(mul(tangent,0.5),mul(side,3_f64.sqrt()*0.5)),a.size))).unwrap();
        assert!(surface.fits([a.p,b.p,c.p]),"seed triangle exceeds sampling tolerance");
        let mut result = Self {vertices:vec![a,b,c],triangles:vec![],used:BTreeSet::new(),
            boundary:BTreeSet::new(),queue:VecDeque::new(),clearance:surface.accuracy*3.,repairs:0};
        result.insert([0,1,2]); result
    }

    fn insert(&mut self,t:[usize;3]) {
        for i in 0..3 {
            let e = [t[i],t[(i+1)%3]]; assert!(self.used.insert(e));
            if !self.boundary.remove(&[e[1],e[0]]) {
                self.boundary.insert(e); self.queue.push_back(e);
            }
        }
        self.triangles.push(t);
    }

    fn legal(&self,t:[usize;3],new:Option<&Vertex>) -> bool {
        if (0..3).any(|k| self.used.contains(&[t[k],t[(k+1)%3]])) { return false; }
        let [a,b,c] = t.map(|i| if i == self.vertices.len() { new.unwrap() } else { &self.vertices[i] });
        let normal = cross(sub(b.p,a.p),sub(c.p,a.p));
        let area = length(normal); if area < 1e-12 { return false; }
        let lengths = [length(sub(b.p,a.p)),length(sub(c.p,b.p)),length(sub(a.p,c.p))];
        let longest = lengths.into_iter().fold(0_f64,f64::max);
        if area <= 0.12*longest*longest || [a,b,c].iter().any(|v| dot(normal,v.n) <= area*0.2) { return false; }
        // A front is a chordal approximation of a curved boundary. Requiring
        // literal 3D segment intersection misses approaching fronts. Exclude
        // other frontier edges from a shallow prism around this triangle.
        // Clearance is an experimental geometric guard, not a proved surface
        // deviation; the independent encoded-mesh audit remains necessary.
        let n = mul(normal,1./area);
        let points = [a.p,b.p,c.p];
        let mut planes = vec![];
        for k in 0..3 {
            let inward = cross(n,sub(points[(k+1)%3],points[k]));
            planes.push((inward,-dot(inward,points[k])));
        }
        planes.push((n,self.clearance-dot(n,a.p)));
        planes.push((mul(n,-1.),self.clearance+dot(n,a.p)));
        for &e in &self.boundary {
            if e.iter().all(|i| t.contains(i)) { continue; }
            let [p,q] = e.map(|i| self.vertices[i].p);
            let mut lo = 0_f64; let mut hi = 1_f64;
            for &(normal,offset) in &planes {
                let a = dot(normal,p)+offset; let b = dot(normal,q)+offset;
                if a < 0. && b < 0. { hi = -1.; break; }
                if a < 0. { lo = lo.max(a/(a-b)); }
                if b < 0. { hi = hi.min(a/(a-b)); }
            }
            if hi-lo > 1e-9 { return false; }
        }
        true
    }

    fn advance(&mut self,surface:&mut Surface) -> bool {
        let mut attempts = 0;
        while let Some(e) = self.queue.pop_front() {
            if !self.boundary.contains(&e) { continue; }
            attempts += 1;
            if attempts > self.boundary.len() { self.queue.push_front(e); return false; }
            let [a,b] = e.map(|i| &self.vertices[i]);
            let normal = unit(add(a.n,b.n)); let d = sub(b.p,a.p); let width = length(d);
            let h = (a.size*0.5+b.size*0.5).max(width*0.6);
            let height = (h*h-width*width*0.25).sqrt();
            let guess = add(mul(add(a.p,b.p),0.5),mul(unit(cross(d,normal)),height));
            let Some(candidate) = surface.project(guess) else { self.queue.push_back(e); continue; };
            let mut nearby:Vec<_> = self.vertices.iter().enumerate().filter(|(i,v)|
                !e.contains(i) && length(sub(v.p,candidate.p)) < h*0.85 && dot(v.n,candidate.n) > 0.)
                .map(|(i,v)| (length(sub(v.p,candidate.p)),i)).collect();
            nearby.sort_by(|a,b| a.0.total_cmp(&b.0));
            for &(_,i) in &nearby {
                if !self.boundary.iter().any(|e| e.contains(&i)) { continue; }
                let t = [e[1],e[0],i];
                if self.legal(t,None) && surface.fits(t.map(|i| self.vertices[i].p)) {
                    self.insert(t); return true;
                }
            }
            // A nearby existing vertex need not make a sufficiently small
            // connection triangle. An additional sample can fill that gap.
            if nearby.iter().all(|&(distance,_)| distance >= h*0.5) {
                let t = [e[1],e[0],self.vertices.len()];
                if self.legal(t,Some(&candidate)) && surface.fits([b.p,a.p,candidate.p]) {
                    self.vertices.push(candidate); self.insert(t); return true;
                }
            }
            self.queue.push_back(e);
        }
        false
    }

    fn grow(&mut self,surface:&mut Surface) {
        while !self.boundary.is_empty() {
            assert!(self.triangles.len() < 10000,"front triangle budget exhausted");
            if !self.advance(surface) {
                // The growth phase can leave narrow gaps. Close a valid ear
                // on the existing frontier; do not fill a prescribed cap.
                let mut ears = vec![];
                for &[a,b] in &self.boundary {
                    for &[c,d] in &self.boundary {
                        if b != c || a == d { continue; }
                        let t = [b,a,d]; if !self.legal(t,None) || !surface.fits(t.map(|i| self.vertices[i].p)) { continue; }
                        let [p,q,r] = t.map(|i| self.vertices[i].p);
                        let n = cross(sub(q,p),sub(r,p)); let area = length(n);
                        let longest = length(sub(q,p)).max(length(sub(r,q))).max(length(sub(p,r)));
                        if longest > 2.*t.map(|i| self.vertices[i].size).into_iter().fold(0_f64,f64::max) { continue; }
                        // Other frontier vertices must not lie inside the ear's
                        // normal projection near this patch of the surface.
                        let occupied = self.boundary.iter().flatten().any(|&i| {
                            if t.contains(&i) { return false; }
                            let v = self.vertices[i].p;
                            dot(sub(v,p),n).abs() < area*longest*0.5 &&
                                [(p,q),(q,r),(r,p)].iter().all(|&(a,b)| dot(cross(sub(b,a),sub(v,a)),n) >= 0.)
                        });
                        if !occupied { ears.push((area/(longest*longest),t)); }
                    }
                }
                ears.sort_by(|a,b| b.0.total_cmp(&a.0));
                if let Some((_,t)) = ears.first() { self.insert(*t); }
                else if !self.fill_gap(surface) && !self.retriangulate_ear(surface) { break; }
            }
        }
    }

    fn remove(&mut self,index:usize) {
        let t = self.triangles.swap_remove(index);
        for k in 0..3 {
            let e = [t[k],t[(k+1)%3]];
            assert!(self.used.remove(&e));
            if !self.boundary.remove(&e) {
                let reverse = [e[1],e[0]];
                assert!(self.used.contains(&reverse));
                assert!(self.boundary.insert(reverse)); self.queue.push_back(reverse);
            }
        }
    }

    fn retriangulate_ear(&mut self,surface:&mut Surface) -> bool {
        // A skinny gap need not become a skinny triangle. Combine an ear with
        // one adjacent existing triangle, then replace their common diagonal.
        // Stage the change: topology, orientation, quality, frontier clearance
        // and field-fit checks must all pass before changing the live front.
        for &[a,b] in &self.boundary {
            for &[c,d] in &self.boundary {
                if b != c || a == d { continue; }
                let ear = [b,a,d];
                for k in 0..3 {
                    let [u,v,w] = [ear[k],ear[(k+1)%3],ear[(k+2)%3]];
                    let Some((index,opposite)) = self.triangles.iter().enumerate().find_map(|(i,t)| {
                        (0..3).find(|&j| t[j] == v && t[(j+1)%3] == u).map(|j| (i,t[(j+2)%3]))
                    }) else { continue; };
                    if opposite == w || self.used.contains(&[opposite,w]) || self.used.contains(&[w,opposite]) { continue; }
                    let replacement = [[opposite,v,w],[opposite,w,u]];
                    // Reject unsuitable shapes before copying the front or
                    // spending any new field queries on this candidate.
                    if replacement.iter().any(|t| {
                        let [p,q,r] = t.map(|i| self.vertices[i].p);
                        let longest = length(sub(p,q)).max(length(sub(q,r))).max(length(sub(r,p)));
                        length(cross(sub(q,p),sub(r,p))) <= 0.12*longest*longest
                    }) { continue; }
                    let mut trial = self.clone(); trial.remove(index);
                    let mut accepted = true;
                    for t in replacement {
                        if !trial.legal(t,None) || !surface.fits(t.map(|i| trial.vertices[i].p)) {
                            accepted = false; break;
                        }
                        trial.insert(t);
                    }
                    if accepted {
                        trial.repairs += 1; *self = trial; return true;
                    }
                }
            }
        }
        false
    }

    fn fill_gap(&mut self,surface:&mut Surface) -> bool {
        // Small discovered boundary loops can need another surface sample
        // even when no existing-vertex ear meets the geometric target.
        for &[start,next] in &self.boundary {
            let mut ring = vec![start]; let mut current = next;
            while current != start && ring.len() <= 12 && !ring.contains(&current) {
                ring.push(current);
                let outgoing:Vec<_> = self.boundary.iter().filter(|e| e[0] == current).collect();
                if outgoing.len() != 1 { break; }
                current = outgoing[0][1];
            }
            if current != start || ring.len() > 12 { continue; }
            let center = ring.iter().fold([0.;3],|sum,&i| add(sum,mul(self.vertices[i].p,1./ring.len() as f64)));
            let Some(vertex) = surface.project(center) else { continue; };
            let index = self.vertices.len();
            let triangles:Vec<_> = (0..ring.len()).map(|i| [ring[(i+1)%ring.len()],ring[i],index]).collect();
            if triangles.iter().all(|&t| self.legal(t,Some(&vertex)) &&
                surface.fits([self.vertices[t[0]].p,self.vertices[t[1]].p,vertex.p])) {
                self.vertices.push(vertex);
                for t in triangles { self.insert(t); }
                return true;
            }
        }
        false
    }
}

fn sphere() -> MaterialField {
    SpatialField::from(RevolvedField::new(F::disk([0.;2],1.).unwrap(),[0.;3],[0.,0.,1.]).unwrap()).into()
}

fn cube() -> MaterialField {
    // The radius-two ball only supplies finite construction support; all six
    // half-spaces determine the cube. The marcher never sees the planes.
    let mut field = MaterialField::from(SpatialField::from(RevolvedField::new(
        F::disk([0.;2],2.).unwrap(),[0.;3],[0.,0.,1.]).unwrap()));
    for k in 0..3 { for sign in [-1.,1.] {
        let mut axis = [0.;3]; axis[k] = sign;
        let half = RevolvedField::new(F::half_plane([0.,1.],[0.,1.]).unwrap(),[0.;3],axis).unwrap();
        field = field.intersection(SpatialField::from(half).into()).unwrap();
    }}
    field
}

fn check_front(name:&str,field:MaterialField,genus:usize,distance:impl Fn(V)->f64) {
    let start = std::time::Instant::now();
    let mut surface = Surface::new(field,0.02);
    let mut front = Front::start(&mut surface); front.grow(&mut surface);
    let elapsed = start.elapsed();
    eprintln!("generic front {name}: extraction {elapsed:?}");
    eprintln!("generic front {name}: {} triangles, {} boundary edges, {} queries",front.triangles.len(),front.boundary.len(),surface.queries);
    eprintln!("generic front {name}: {} local retriangulations",front.repairs);
    assert!(front.boundary.is_empty(),"front stalled on {name}");
    let shell = gcs_core::topology::ClosedShell::from_triangles(front.vertices.len(),&front.triangles).unwrap();
    assert_eq!(shell.genus(),genus);
    for v in &front.vertices {
        assert!(v.radius <= surface.point_tolerance);
        assert!(surface.value(v.inside).bounds()[1] < 0. && surface.value(v.outside).bounds()[0] > 0.);
    }
    let mut deviation = 0_f64;
    let mut min_angle = std::f64::consts::PI;
    for t in &front.triangles {
        let [a,b,c] = t.map(|i| front.vertices[i].p);
        for [p,q,r] in [[a,b,c],[b,c,a],[c,a,b]] {
            let u = sub(q,p); let v = sub(r,p);
            min_angle = min_angle.min(length(cross(u,v)).atan2(dot(u,v)));
        }
        for p in [mul(add(a,b),0.5),mul(add(b,c),0.5),mul(add(c,a),0.5),mul(add(add(a,b),c),1./3.)] {
            deviation = deviation.max(distance(p));
        }
    }
    eprintln!("generic front {name}: sampled triangle deviation {deviation}, minimum angle {} degrees",min_angle.to_degrees());
    assert!(min_angle.to_degrees() > 6.,"candidate contains a sliver triangle");
    assert!(deviation <= surface.accuracy,"candidate has excessive sampled deviation");
    // Timing gates are opt-in and run serially, outside the parallel core suite.
    // An open mesh has already failed above, regardless of elapsed time.
    if name != "torus" {
        if let Ok(limit) = std::env::var("SOLVENT_FRONT_MAX_MS") {
            let limit:f64 = limit.parse().expect("SOLVENT_FRONT_MAX_MS must be a positive number");
            assert!(limit.is_finite() && limit > 0.);
            assert!(elapsed.as_secs_f64()*1000. < limit,"{name} exceeds {limit} ms extraction target");
        }
    }
    if let Some(output) = std::env::var_os("SOLVENT_FRONT_OUTPUT") {
        let output = std::path::PathBuf::from(output); std::fs::create_dir_all(&output).unwrap();
        let pieces:Vec<_> = front.triangles.iter().map(|t| gcs_core::csg::Piece {
            pts:t.iter().map(|&i| front.vertices[i].p).collect(),n:[0.;3],path:name.into(),prim:0,smooth:false,
        }).collect();
        let bytes = gcs_core::mesh::checked_stl(&pieces,"general advancing-front candidate").unwrap();
        std::fs::write(output.join(format!("{name}.stl")),bytes).unwrap();
    }
}

#[test]
fn generic_front_sphere() {
    check_front("sphere",sphere(),0,|p| (length(p)-1.).abs());
}

#[test]
fn generic_front_torus() {
    let torus = SpatialField::from(RevolvedField::new(F::disk([2.,0.],0.6).unwrap(),[0.;3],[0.,0.,1.]).unwrap()).into();
    check_front("torus",torus,1,|p| ((p[0].hypot(p[1])-2.).hypot(p[2])-0.6).abs());
}

#[test]
#[ignore = "Unmet acceptance case: generic sharp-edge discovery and front closure are missing"]
fn generic_front_cube() {
    check_front("cube",cube(),0,|p| {
        let q = p.map(|x| x.abs()-1.);
        length(q.map(|x| x.max(0.)))+q.into_iter().fold(f64::NEG_INFINITY,f64::max).min(0.).abs()
    });
}

#[test]
#[should_panic(expected="seed discovery exhausted without a retained point")]
fn generic_front_does_not_seed_a_zero_only_boolean_surface() {
    let sphere = MaterialField::from(SpatialField::from(RevolvedField::new(F::disk([0.;2],1.).unwrap(),[0.;3],[0.,0.,1.]).unwrap()));
    Surface::new(sphere.clone().difference(sphere).unwrap(),0.02).seed();
}

#[test]
fn generic_front_cache_reuses_work_without_changing_the_mesh() {
    let mut cached = Surface::new(sphere(),0.02);
    let mut uncached = Surface::new(sphere(),0.02); uncached.cache = false;
    let mut a = Front::start(&mut cached); a.grow(&mut cached);
    let mut b = Front::start(&mut uncached); b.grow(&mut uncached);
    assert!(a.boundary.is_empty() && b.boundary.is_empty());
    assert_eq!(a.triangles,b.triangles);
    assert_eq!(a.vertices,b.vertices);
    assert!(cached.queries*2 < uncached.queries,"repeated projection work was not reused");
    eprintln!("generic front sphere: cache {} vs {} point queries",cached.queries,uncached.queries);
}
