//! Frontier topology, candidate growth, collision guards and local repair.
use super::*;

#[derive(Clone,Copy,Debug,PartialEq,Eq,PartialOrd,Ord)]
pub(super) enum Refusal { DirectedEdge,Degenerate,Quality,Orientation,Clearance([usize;2]) }

#[derive(Clone)]
pub(super) struct Front {
    pub(super) vertices:Vec<Vertex>,
    pub(super) triangles:Vec<[usize;3]>,
    pub(super) used:BTreeSet<[usize;2]>,
    pub(super) boundary:BTreeSet<[usize;2]>,
    pub(super) queue:VecDeque<[usize;2]>,
    pub(super) clearance:f64,
    pub(super) repairs:usize,
}

impl Front {
    pub(super) fn start(surface:&mut Surface) -> Self {
        let a = surface.seed(); let tangent = tangent(a.n); let side = cross(a.n,tangent);
        let b = surface.project(add(a.p,mul(tangent,a.size))).unwrap();
        let c = surface.project(add(a.p,mul(add(mul(tangent,0.5),mul(side,3_f64.sqrt()*0.5)),a.size))).unwrap();
        Self::from_seed(surface,[a,b,c])
    }

    pub(super) fn from_seed(surface:&mut Surface,[a,b,c]:[Vertex;3]) -> Self {
        assert!(surface.fits([a.p,b.p,c.p]),"seed triangle exceeds sampling tolerance");
        let mut result = Self {vertices:vec![a,b,c],triangles:vec![],used:BTreeSet::new(),
            boundary:BTreeSet::new(),queue:VecDeque::new(),clearance:surface.accuracy*3.,repairs:0};
        assert!(result.legal([0,1,2],None),"seed triangle fails orientation or shape checks");
        result.insert([0,1,2]); result
    }

    pub(super) fn insert(&mut self,t:[usize;3]) {
        for i in 0..3 {
            let e = [t[i],t[(i+1)%3]]; assert!(self.used.insert(e));
            if !self.boundary.remove(&[e[1],e[0]]) {
                self.boundary.insert(e); self.queue.push_back(e);
            }
        }
        self.triangles.push(t);
    }

    pub(super) fn legal(&self,t:[usize;3],new:Option<&Vertex>) -> bool {
        self.refusal(t,new).is_none()
    }

    pub(super) fn refusal(&self,t:[usize;3],new:Option<&Vertex>) -> Option<Refusal> {
        if (0..3).any(|k| self.used.contains(&[t[k],t[(k+1)%3]])) { return Some(Refusal::DirectedEdge); }
        let [a,b,c] = t.map(|i| if i == self.vertices.len() { new.unwrap() } else { &self.vertices[i] });
        let normal = cross(sub(b.p,a.p),sub(c.p,a.p));
        let area = length(normal); if area < 1e-12 { return Some(Refusal::Degenerate); }
        if !quality::acceptable([a,b,c]) { return Some(Refusal::Quality); }
        if [a,b,c].iter().any(|v|
            v.normals().iter().all(|&n| dot(normal,n) <= area*0.2)) { return Some(Refusal::Orientation); }
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
        let mut crease_neighbors = BTreeSet::new();
        for k in 0..3 {
            let (u,v) = (t[k],t[(k+1)%3]);
            if u >= self.vertices.len() || v >= self.vertices.len() ||
                self.vertices[u].branches.is_empty() || self.vertices[v].branches.is_empty() { continue; }
            if let Some(old) = self.triangles.iter().find(|old|
                (0..3).any(|j| old[j] == v && old[(j+1)%3] == u)) {
                let [p,q,r] = old.map(|i| self.vertices[i].p);
                let old_normal = cross(sub(q,p),sub(r,p));
                if length(cross(normal,old_normal)) > area*length(old_normal)*1e-8 {
                    // Distinct face planes sharing this crease meet along the
                    // common edge. Their other edges enter each other's thick
                    // clearance prisms near the crease, without crossing the
                    // actual triangles. Keep fences for nonadjacent fronts.
                    for j in 0..3 { crease_neighbors.insert([old[j],old[(j+1)%3]]); }
                }
            }
        }
        for &e in &self.boundary {
            if e.iter().all(|i| t.contains(i)) || crease_neighbors.contains(&e) { continue; }
            let [p,q] = e.map(|i| self.vertices[i].p);
            let mut lo = 0_f64; let mut hi = 1_f64;
            for &(normal,offset) in &planes {
                let a = dot(normal,p)+offset; let b = dot(normal,q)+offset;
                if a < 0. && b < 0. { hi = -1.; break; }
                if a < 0. { lo = lo.max(a/(a-b)); }
                if b < 0. { hi = hi.min(a/(a-b)); }
            }
            if hi-lo > 1e-9 {
                // Nearby faces of a thin solid can enter the same thick
                // prism. Keep the proximity guard on compatible patches;
                // for a different support, require interval separation of
                // the actual encoded edge before allowing this candidate.
                let [a,b] = e.map(|i| &self.vertices[i]);
                let common:Vec<_> = a.normals().iter().copied().filter(|&u|
                    b.normals().iter().any(|&v| dot(u,v) > 1.-1e-8)).collect();
                if !common.is_empty() && common.iter().all(|&u| dot(u,n) < 0.95) &&
                    clearance::separated_except_shared([p,q],points,e.map(|i| t.contains(&i))) { continue; }
                return Some(Refusal::Clearance(e));
            }
        }
        None
    }

    pub(super) fn growth_normal(&self,[a,b]:[usize;2]) -> V {
        let [va,vb] = [&self.vertices[a],&self.vertices[b]];
        if !va.branches.is_empty() && !vb.branches.is_empty() {
            // A discovered crease has more than one incident surface normal.
            // Continue onto the unoccupied branch, rather than predicting in
            // a tangent plane made from an average across the crease.
            if let Some(t) = self.triangles.iter().find(|t|
                (0..3).any(|k| t[k] == a && t[(k+1)%3] == b)) {
                let [p,q,r] = t.map(|i| self.vertices[i].p);
                let occupied = unit(cross(sub(q,p),sub(r,p)));
                let mut choices = vec![];
                for &n in &va.branches { for &m in &vb.branches {
                    if dot(n,m) > 0.95 {
                        let candidate = unit(add(n,m));
                        if dot(candidate,occupied) < 0.95 { choices.push(candidate); }
                    }
                }}
                choices.sort_by(|&n,&m| dot(n,occupied).total_cmp(&dot(m,occupied)));
                if let Some(&n) = choices.first() { return n; }
            }
        }
        unit(add(va.n,vb.n))
    }

    pub(super) fn candidate(&self,surface:&mut Surface,e:[usize;2]) -> Option<(Vertex,f64)> {
        let [a,b] = e.map(|i| &self.vertices[i]);
        let normal = self.growth_normal(e); let d = sub(b.p,a.p); let width = length(d);
        let h = (a.size*0.5+b.size*0.5).max(width*0.6);
        let height = (h*h-width*width*0.25).sqrt();
        let guess = add(mul(add(a.p,b.p),0.5),mul(unit(cross(d,normal)),height));
        let projected = surface.project(guess);
        let feature = if projected.as_ref().is_none_or(|v| dot(v.n,normal) < 0.9) {
            surface.feature_vertex(guess,h)
        } else { None };
        let candidate = feature.or(projected)?;
        Some((candidate,h))
    }

    pub(super) fn advance(&mut self,surface:&mut Surface) -> bool {
        let mut attempts = 0;
        while let Some(e) = self.queue.pop_front() {
            if !self.boundary.contains(&e) { continue; }
            attempts += 1;
            if attempts > self.boundary.len() { self.queue.push_front(e); return false; }
            let [a,b] = e.map(|i| &self.vertices[i]);
            let Some((candidate,h)) = self.candidate(surface,e) else { self.queue.push_back(e); continue; };
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

    pub(super) fn grow(&mut self,surface:&mut Surface) {
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

    pub(super) fn remove(&mut self,index:usize) {
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

    pub(super) fn retriangulate_ear(&mut self,surface:&mut Surface) -> bool {
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
                    if replacement.iter().any(|t| !quality::acceptable(t.map(|i| &self.vertices[i]))) { continue; }
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

    pub(super) fn fill_gap(&mut self,surface:&mut Surface) -> bool {
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
