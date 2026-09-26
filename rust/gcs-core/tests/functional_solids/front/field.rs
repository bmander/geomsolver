//! Bounded field queries, projections, sizing and interior seed search.
use super::*;

pub(super) struct Surface {
    pub(super) field:MaterialEvaluator,
    pub(super) support:[I;3],
    pub(super) accuracy:f64,
    pub(super) point_tolerance:f64,
    pub(super) max_step:f64,
    pub(super) queries:usize,
    pub(super) box_queries:usize,
    pub(super) corrections:BTreeMap<([u64;3],u64),Option<(V,V,V,V)>>,
    pub(super) projections:BTreeMap<[u64;3],Option<Vertex>>,
    pub(super) features:BTreeMap<([u64;3],u64),Option<Vertex>>,
    pub(super) cache:bool,
}

impl Surface {
    pub(super) fn new(field:MaterialField,accuracy:f64) -> Self {
        assert!(accuracy.is_finite() && accuracy > 0.);
        let support = field.support_bounds().unwrap().expect("marching requires finite support");
        let span = support.map(|v| v.bounds()[1]-v.bounds()[0]);
        Self {field:field.evaluator(100000),support,accuracy,point_tolerance:accuracy*0.01,
            max_step:length(span)/12.,queries:0,box_queries:0,corrections:BTreeMap::new(),
            projections:BTreeMap::new(),features:BTreeMap::new(),cache:true}
    }

    pub(super) fn value(&mut self,p:V) -> I {
        self.query(p,false)
    }

    pub(super) fn query(&mut self,p:V,precise:bool) -> I {
        assert!(self.queries < 1_000_000,"front field-query budget exhausted"); self.queries += 1;
        let options = Options {value_tolerance:self.point_tolerance*0.01,max_evaluations:20000};
        let value = if precise { self.field.bounds(point(p),options) }
            else { self.field.query(point(p),Stop::Outside(I::ZERO),options,None) }.unwrap().value;
        if precise { let [a,b] = value.bounds();
            assert!(b-a <= options.value_tolerance,"field enclosure too wide for a differential probe");
        }
        value
    }

    pub(super) fn gradient(&mut self,p:V) -> V {
        self.gradient_with_step(p,self.accuracy*0.2)
    }

    pub(super) fn gradient_with_step(&mut self,p:V,h:f64) -> V {
        self.gradient_measurement(p,h).0
    }

    pub(super) fn gradient_measurement(&mut self,p:V,h:f64) -> (V,f64) {
        assert!(h.is_finite() && h > 0.);
        let mut error = [0.;3];
        let g = std::array::from_fn(|k| {
            let mut a = p; a[k] -= h; let mut b = p; b[k] += h;
            let a = self.query(a,true).bounds(); let b = self.query(b,true).bounds();
            error[k] = ((b[1]-b[0])+(a[1]-a[0]))/(4.*h);
            ((b[0]+b[1])-(a[0]+a[1]))/(4.*h)
        });
        (g,length(error))
    }

    pub(super) fn vertex(&mut self,p:V,n:V,inside:V,outside:V) -> Vertex {
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
        Vertex {p,n,size,inside,outside,radius,branches:vec![]}
    }

    pub(super) fn project(&mut self,guess:V) -> Option<Vertex> {
        let key = guess.map(f64::to_bits);
        if let Some(vertex) = self.projections.get(&key) { return vertex.clone(); }
        let vertex = self.correct(guess).map(|(p,n,a,b)| self.vertex(p,n,a,b));
        if self.cache && self.projections.len() < 16384 { self.projections.insert(key,vertex.clone()); }
        vertex
    }

    pub(super) fn correct(&mut self,guess:V) -> Option<(V,V,V,V)> {
        self.correct_with_step(guess,self.accuracy*0.2)
    }

    pub(super) fn correct_with_step(&mut self,guess:V,h:f64) -> Option<(V,V,V,V)> {
        // The field and accuracy controls are immutable during extraction.
        // Shared edge midpoints and retried frontier predictions can reuse the
        // exact same result, including failure. No spatial quantization.
        let key = (guess.map(f64::to_bits),h.to_bits());
        if let Some(result) = self.corrections.get(&key) { return *result; }
        let result = self.correct_uncached(guess,h);
        if self.cache && self.corrections.len() < 16384 { self.corrections.insert(key,result); }
        result
    }

    pub(super) fn correct_uncached(&mut self,guess:V,h:f64) -> Option<(V,V,V,V)> {
        let mut p = guess;
        for _ in 0..20 {
            let (gradient,error) = self.gradient_measurement(p,h); let magnitude = length(gradient);
            if magnitude < 1e-10 || error > magnitude*0.01 { return None; }
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

    pub(super) fn fits(&mut self,[a,b,c]:[V;3]) -> bool {
        let normal = cross(sub(b,a),sub(c,a)); let area = length(normal);
        if !area.is_finite() || area <= 1e-12 { return false; }
        let normal = mul(normal,1./area);
        let samples = [mul(add(a,b),0.5),mul(add(b,c),0.5),mul(add(c,a),0.5),mul(add(add(a,b),c),1./3.)];
        for (i,sample) in samples.into_iter().enumerate() {
            let Some((p,_,inside,outside)) = self.correct(sample) else { return false; };
            if boundary_radius(sample,inside,outside) > self.accuracy { return false; }
            if i == 3 {
                // Different incident normals at the vertices can each approve
                // an inward triangle. Check orientation at its interior using
                // actual material/exterior signs along the triangle normal.
                let offset = mul(normal,self.point_tolerance);
                if self.value(sub(p,offset)).bounds()[1] >= 0. ||
                    self.value(add(p,offset)).bounds()[0] <= 0. { return false; }
            }
        }
        true
    }

    pub(super) fn interior_seed(&mut self) -> Option<V> {
        fn halton(mut i:usize,base:usize) -> f64 {
            let mut f = 1.; let mut result = 0.;
            while i > 0 { f /= base as f64; result += f*(i%base) as f64; i /= base; }
            result
        }
        for i in 1..=64 {
            let p = std::array::from_fn(|k| {
                let [a,b] = self.support[k].bounds(); a+(b-a)*halton(i,[2,3,5][k])
            });
            if self.value(p).bounds()[1] < 0. { return Some(p); }
        }
        // Thin retained volumes can fall between every point in a short stab
        // sequence. Search only for an interior seed, pruning boxes whose
        // field lower bound excludes strict material. This is not meshing the
        // volume, nor a component-coverage claim after the first seed is found.
        let mut queue = VecDeque::from([self.support]);
        for _ in 0..4096 {
            let Some(cell) = queue.pop_front() else { break; };
            self.box_queries += 1;
            let value = self.field.query(cell,Stop::Outside(I::ZERO),Options {
                value_tolerance:self.point_tolerance*0.01,max_evaluations:20000,
            },None).unwrap().value;
            if value.bounds()[0] >= 0. { continue; }
            let center = cell.map(|v| { let [a,b] = v.bounds(); a*0.5+b*0.5 });
            if self.value(center).bounds()[1] < 0. { return Some(center); }
            let span = cell.map(|v| { let [a,b] = v.bounds(); b-a });
            let k = (0..3).max_by(|&a,&b| span[a].total_cmp(&span[b])).unwrap();
            let [lo,hi] = cell[k].bounds(); let mid = center[k];
            if !(lo < mid && mid < hi) { continue; }
            let mut left = cell; left[k] = I::new(lo,mid).unwrap();
            let mut right = cell; right[k] = I::new(mid,hi).unwrap();
            queue.push_back(left); queue.push_back(right);
        }
        None
    }

    pub(super) fn seed(&mut self) -> Vertex {
        let mut a = self.interior_seed().expect("seed discovery exhausted without a retained point");
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
