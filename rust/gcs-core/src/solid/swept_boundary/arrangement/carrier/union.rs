//! Union of regular carried-circle domains in a shared (radial magnitude,
//! projected coordinate) chart. Analytic boundaries become straight lines in
//! this chart. Predicate uncertainty refuses the arrangement, never welds it.
use super::{Correspondence,Mapped,Error as MapError};
use crate::{interval::{Interval,wide::Wide as W},solid::{ToolFace, surface::CoordinateCircle}};
use std::collections::BTreeMap;
type Result<T> = std::result::Result<T,Error>;
#[derive(Clone,Debug,PartialEq)]
pub enum Error { InvalidOptions, CorrespondenceUnresolved, UnsupportedChart,
    Predicate, Budget, Topology, Bounds(crate::interval::Error), Map(MapError) }
impl From<crate::interval::Error> for Error { fn from(e: crate::interval::Error) -> Self { Self::Bounds(e) } }
impl From<MapError> for Error { fn from(e: MapError) -> Self { Self::Map(e) } }
#[derive(Clone,Copy,Debug)]
pub struct Options { pub max_events: usize, pub max_cells: usize }
#[derive(Clone,Debug)]
pub struct Vertex { pub coordinates: [f64;2], pub enclosure: [Interval;2] }
#[derive(Clone,Copy,Debug,PartialEq,Eq,PartialOrd,Ord)]
pub enum Curve { Line(usize), Cut(usize) }
#[derive(Clone,Copy,Debug)]
pub struct Use { pub cell: usize, pub forward: bool }
#[derive(Clone,Debug)]
pub struct Edge { pub curve: Curve, pub vertices: [usize;2], pub uses: Vec<Use> }
#[derive(Clone,Debug)]
pub struct Cell {
    /// Indices into the input correspondence's regular regions. Each region
    /// retains its band, edge, native rectangle and inverse branch separately.
    pub consumers: Vec<usize>,
    /// Directed boundary uses, counterclockwise in this common chart.
    pub edges: Vec<(usize,bool)>,
    cut: usize, lower: usize, upper: usize,
}
#[derive(Clone,Debug)]
pub struct Boundary { pub edges: Vec<(usize,bool)> }
#[derive(Clone,Debug,PartialEq,Eq,PartialOrd,Ord)]
enum Angle { Point(i128), Raw([u64;3]) }
#[derive(Clone,Debug,PartialEq,Eq,PartialOrd,Ord)]
enum Expr { Constant(u64), Trig {sine:bool,angle:Angle,scale:u64} }
#[derive(Clone,Debug)]
struct Scalar { key: Expr, value: W }
#[derive(Clone,Debug)]
struct Line { a: Scalar, b: Scalar }
impl Line {
    fn key(&self) -> (Expr,Expr) { (self.a.key.clone(),self.b.key.clone()) }
    fn at(&self,w: W) -> Result<W> { Ok(self.a.value.mul(w)?.add(self.b.value)?) }
}
#[derive(Clone,Debug)]
struct Cut { value: W, intersections: Vec<[usize;2]> }
struct Trap { region: usize, range: [W;2], ends: [Vec<u64>;2], lower: usize, upper: usize }
pub struct Union<'a,'s> {
    correspondence: &'a Correspondence<'s>,
    axis_sign: f64,
    radius2: f64,
    retained: usize,
    lines: Vec<Line>, cuts: Vec<Cut>,
    vertices: Vec<Vertex>, edges: Vec<Edge>, cells: Vec<Cell>, boundaries: Vec<Boundary>,
}
fn cmp(a: W,b: W) -> Result<std::cmp::Ordering> {
    use std::cmp::Ordering::*;
    if a.hi < b.lo { Ok(Less) } else if b.hi < a.lo { Ok(Greater) }
    else if a.lo == a.hi && a == b { Ok(Equal) } else { Err(Error::Predicate) }
}
fn between(a: W,b: W) -> Result<W> {
    if a.hi >= b.lo { return Err(Error::Predicate); }
    let m = a.hi/2+b.lo/2;
    if m <= a.hi || m >= b.lo { return Err(Error::Predicate); }
    Ok(W {lo:m,hi:m})
}
fn scalar(rate:f64,phase:f64,roll:f64,sine:bool,mut scale:f64) -> Result<Scalar> {
    let constant = |x:f64| -> Result<Scalar> {
        Ok(Scalar {key:Expr::Constant(x.to_bits()),value:W::point(x)?})
    };
    if scale == 0. { return constant(0.); }
    let mut angle = W::point(rate)?.mul(W::point(roll)?)?.add(W::point(phase)?)?;
    let key = if angle.lo == angle.hi {
        if angle.lo < 0 { angle = angle.neg(); if sine { scale = -scale; } }
        if angle == W::ZERO { return constant(if sine { 0. } else { scale }); }
        Angle::Point(angle.lo)
    } else { Angle::Raw([rate.to_bits(),phase.to_bits(),roll.to_bits()]) };
    let (s,c) = angle.sin_cos()?;
    Ok(Scalar {key:Expr::Trig {sine,angle:key,scale:scale.to_bits()},
        value:(if sine {s} else {c}).mul(W::point(scale)?)?})
}
fn difference(a:&Scalar,b:&Scalar) -> Result<W> {
    if a.key == b.key { Ok(W::ZERO) } else { Ok(a.value.sub(b.value)?) }
}
fn circle_key(c: CoordinateCircle) -> Vec<u64> {
    let mut k = vec![c.axis as u64,c.direction.to_bits()];
    k.extend(c.centre.map(f64::to_bits)); k.extend(c.radial.map(f64::to_bits)); k
}
fn source_w(c: CoordinateCircle,angular:[f64;2],sweep:f64,t:f64,k:usize,sign:f64)
    -> Result<(W,Vec<u64>)> {
    let angle = W::point(angular[0])?.add(W::point(t)?.mul(
        W::point(angular[1])?.sub(W::point(angular[0])?)?)?)?.mul(W::point(sweep)?)?;
    let m = 3-c.axis-k;
    let b = c.direction*c.radial[m]*if (c.axis+1)%3 == k {-1.} else {1.};
    let (s,co) = angle.sin_cos()?;
    let value = W::point(c.radial[k])?.mul(co)?.add(W::point(b)?.mul(s)?)?.mul(W::point(sign)?)?;
    let mut key = circle_key(c); key.extend([angular[0].to_bits(),angular[1].to_bits(),
        sweep.to_bits(),t.to_bits(),sign.to_bits()]);
    Ok((value,key))
}
impl<'a,'s> Union<'a,'s> {
    pub fn build(c:&'a Correspondence<'s>,options:Options) -> Result<Self> {
        if options.max_cells == 0 || options.max_events == 0 { return Err(Error::Budget); }
        if !c.unresolved.is_empty() { return Err(Error::CorrespondenceUnresolved); }
        if c.regions.is_empty() { return Err(Error::UnsupportedChart); }
        let motion = c.sweep.motion().coordinate_rotation().ok_or(Error::UnsupportedChart)?;
        let chart = c.regions[0].chart;
        let retained = 3-c.axis-chart.omitted;
        let mut line_map = BTreeMap::new(); let mut raw = Vec::new(); let mut family = None;
        for (region,r) in c.regions.iter().enumerate() {
            if r.chart != chart { return Err(Error::UnsupportedChart); }
            let native = &c.circles[r.band];
            let ToolFace::Revolved(f) = &c.sweep.faces()[native.face] else { unreachable!() };
            let circle = f.coordinate_circle(native.u).ok_or(Error::UnsupportedChart)?;
            let k = 3-circle.axis-c.axis;
            let d = r.domain;
            let v = Interval::new(d[0][0],d[0][1])?;
            let v = Interval::point(native.angular[0])?.add(v.mul(Interval::point(native.angular[1])?
                .sub(Interval::point(native.angular[0])?)?)?)?;
            let bounds = f.angular_chart(v.bounds()).map_err(|_| Error::UnsupportedChart)?
                .bounds(Interval::point(native.u)?,v)?;
            let x = bounds.position[c.axis].sub(Interval::point(c.sphere.centre[c.axis])?)?;
            let q = bounds.position[k].sub(Interval::point(c.sphere.centre[k])?)?;
            if x.contains(0.) || q.contains(0.) { return Err(Error::UnsupportedChart); }
            let axis_sign = if x.bounds()[0] > 0. {1.} else {-1.};
            let sign = if q.bounds()[0] > 0. {1.} else {-1.};
            let h = crate::interval::exact_difference(circle.centre[circle.axis],c.sphere.centre[circle.axis])
                .ok_or(Error::UnsupportedChart)?;
            let mut radial = circle.radial.map(f64::abs); radial.sort_by(f64::total_cmp);
            let key = (circle.axis,radial.map(f64::to_bits),axis_sign as i8);
            if family.is_some_and(|old| old != key) { return Err(Error::UnsupportedChart); }
            family = Some(key);
            let a = source_w(circle,native.angular,f.sweep(),d[0][0],k,sign)?;
            let b = source_w(circle,native.angular,f.sweep(),d[0][1],k,sign)?;
            let ends = if cmp(a.0,b.0)? == std::cmp::Ordering::Less { [a,b] } else { [b,a] };
            let cross_sign = if (c.axis+1)%3 == k {-1.} else {1.};
            let mut keys = Vec::new();
            for roll in d[1] {
                let (sa,sb) = if retained == k { (false,true) } else { (true,false) };
                let (scale_a,scale_b) = if retained == k { (sign,cross_sign*h) } else { (-cross_sign*sign,h) };
                let line = Line {a:scalar(motion.rate,motion.phase,roll,sa,scale_a)?,
                    b:scalar(motion.rate,motion.phase,roll,sb,scale_b)?};
                keys.push(line.key()); line_map.entry(line.key()).or_insert(line);
            }
            raw.push((region,ends,keys));
        }
        let keys:Vec<_> = line_map.keys().cloned().collect();
        let lines:Vec<_> = line_map.into_values().collect();
        let mut traps = Vec::new();
        for (region,ends,keys0) in raw {
            let mut ids = [keys.binary_search(&keys0[0]).unwrap(),keys.binary_search(&keys0[1]).unwrap()];
            let mid = between(ends[0].0,ends[1].0)?;
            if cmp(lines[ids[0]].at(mid)?,lines[ids[1]].at(mid)?)? == std::cmp::Ordering::Greater { ids.swap(0,1); }
            if ids[0] == ids[1] { return Err(Error::Predicate); }
            traps.push(Trap {region,range:[ends[0].0,ends[1].0],ends:ends.map(|e| e.1),lower:ids[0],upper:ids[1]});
        }
        let mut cuts:Vec<Cut> = Vec::new(); let mut endpoint_keys = BTreeMap::new();
        for trap in &traps { for k in 0..2 {
            if !endpoint_keys.contains_key(&trap.ends[k]) {
                endpoint_keys.insert(trap.ends[k].clone(),cuts.len());
                cuts.push(Cut {value:trap.range[k],intersections:Vec::new()});
            }
        } }
        let low = cuts.iter().map(|c| c.value.lo).min().unwrap();
        let high = cuts.iter().map(|c| c.value.hi).max().unwrap();
        let mut events = cuts.len();
        if events > options.max_events { return Err(Error::Budget); }
        for i in 0..lines.len() { for j in i+1..lines.len() {
            events += 1; if events > options.max_events { return Err(Error::Budget); }
            let a = difference(&lines[i].a,&lines[j].a)?;
            let b = difference(&lines[j].b,&lines[i].b)?;
            if a == W::ZERO {
                if !b.positive() && !b.negative() { return Err(Error::Predicate); }
                continue;
            }
            let root = b.div(a).map_err(|_| Error::Predicate)?;
            if root.hi < low || root.lo > high { continue; }
            cuts.push(Cut {value:root,intersections:vec![[i,j]]});
        } }
        cuts.sort_by_key(|c| c.value.lo);
        let mut unique:Vec<Cut> = Vec::new();
        for cut in cuts {
            if let Some(last) = unique.last_mut() {
                if cmp(last.value,cut.value)? == std::cmp::Ordering::Equal {
                    last.intersections.extend(cut.intersections); continue;
                }
            }
            unique.push(cut);
        }
        let cuts = unique;
        let mut cells = Vec::new();
        for cut in 0..cuts.len()-1 {
            let mid = between(cuts[cut].value,cuts[cut+1].value)?;
            let mut active = Vec::new();
            for t in &traps {
                let a = cmp(t.range[0],mid)?; let b = cmp(mid,t.range[1])?;
                if a == std::cmp::Ordering::Equal || b == std::cmp::Ordering::Equal { return Err(Error::Predicate); }
                if a == std::cmp::Ordering::Less && b == std::cmp::Ordering::Less { active.push(t); }
            }
            let mut order:Vec<_> = active.iter().flat_map(|t| [t.lower,t.upper]).collect();
            order.sort(); order.dedup();
            let mut heights = order.iter().map(|&i| Ok((lines[i].at(mid)?,i))).collect::<Result<Vec<_>>>()?;
            heights.sort_by_key(|v| v.0.lo); order = heights.into_iter().map(|v| v.1).collect();
            for w in order.windows(2) {
                if cmp(lines[w[0]].at(mid)?,lines[w[1]].at(mid)?)? != std::cmp::Ordering::Less { return Err(Error::Predicate); }
                let consumers:Vec<_> = active.iter().filter(|t| {
                    let a = order.iter().position(|&i| i == t.lower).unwrap();
                    let b = order.iter().position(|&i| i == t.upper).unwrap();
                    let at = order.iter().position(|&i| i == w[0]).unwrap();
                    a <= at && at < b
                }).map(|t| t.region).collect();
                if consumers.is_empty() { continue; }
                if cells.len() >= options.max_cells { return Err(Error::Budget); }
                cells.push(Cell {consumers,edges:Vec::new(),cut,lower:w[0],upper:w[1]});
            }
        }
        let (_,radial,sign) = family.unwrap();
        let radius2 = radial.map(f64::from_bits).iter().map(|x| x*x).sum();
        let mut out = Self {correspondence:c,axis_sign:sign as f64,radius2,retained,
            lines,cuts,vertices:Vec::new(),edges:Vec::new(),cells,boundaries:Vec::new()};
        out.topology()?;
        Ok(out)
    }
    fn topology(&mut self) -> Result<()> {
        let n = self.lines.len();
        let mut on_cut = Vec::new(); let mut cut_order = Vec::new();
        for cut in &self.cuts {
            let mut parent:Vec<_> = (0..n).collect();
            fn root(parent:&[usize],mut i:usize) -> usize {
                while parent[i] != i { i = parent[i]; } i
            }
            for &[a,b] in &cut.intersections {
                let (a,b) = (root(&parent,a),root(&parent,b));
                parent[a.max(b)] = a.min(b);
            }
            let mut groups:BTreeMap<usize,Vec<usize>> = BTreeMap::new();
            for i in 0..n { groups.entry(root(&parent,i)).or_default().push(i); }
            let mut heights = Vec::new();
            for ids in groups.into_values() {
                let value = self.lines[ids[0]].at(cut.value)?;
                heights.push((value,ids));
            }
            heights.sort_by_key(|p| p.0.lo);
            for w in heights.windows(2) {
                if cmp(w[0].0,w[1].0)? != std::cmp::Ordering::Less { return Err(Error::Predicate); }
            }
            let mut ids = vec![0;n]; let mut ordered = Vec::new();
            for (height,lines) in heights {
                let v = self.vertices.len();
                self.vertices.push(Vertex {coordinates:[cut.value.mid(),height.mid()],
                    enclosure:[cut.value.interval()?,height.interval()?]});
                for line in lines { ids[line] = v; }
                ordered.push(v);
            }
            on_cut.push(ids); cut_order.push(ordered);
        }
        let mut edge_ids = BTreeMap::new();
        for cell in 0..self.cells.len() {
            let c = &self.cells[cell]; let (left,right) = (c.cut,c.cut+1);
            let (ll,lh) = (on_cut[left][c.lower],on_cut[left][c.upper]);
            let (rl,rh) = (on_cut[right][c.lower],on_cut[right][c.upper]);
            let mut directed = vec![(Curve::Line(c.lower),ll,rl)];
            for (cut,a,b,reverse) in [(right,rl,rh,false),(left,ll,lh,true)] {
                let order = &cut_order[cut];
                let lo = order.iter().position(|&v| v == a).unwrap();
                let hi = order.iter().position(|&v| v == b).unwrap();
                if lo > hi { return Err(Error::Topology); }
                let mut segments:Vec<_> = order[lo..=hi].windows(2).map(|w|
                    (Curve::Cut(cut),if reverse {w[1]} else {w[0]},if reverse {w[0]} else {w[1]})).collect();
                if reverse { segments.reverse(); }
                directed.extend(segments);
                if !reverse { directed.push((Curve::Line(c.upper),rh,lh)); }
            }
            for (curve,a,b) in directed {
                if a == b { continue; }
                let vertices = [a.min(b),a.max(b)];
                let id = *edge_ids.entry((curve,vertices)).or_insert_with(|| {
                    let id = self.edges.len(); self.edges.push(Edge {curve,vertices,uses:Vec::new()}); id
                });
                let forward = a == vertices[0];
                self.edges[id].uses.push(Use {cell,forward});
                self.cells[cell].edges.push((id,forward));
            }
        }
        let mut outgoing = BTreeMap::new(); let mut incoming = BTreeMap::new();
        for (i,e) in self.edges.iter().enumerate() {
            match e.uses.as_slice() {
                [a,b] if a.forward != b.forward => {},
                [a] => {
                    let [from,to] = if a.forward {e.vertices} else {[e.vertices[1],e.vertices[0]]};
                    if outgoing.insert(from,(to,i,a.forward)).is_some() || incoming.insert(to,from).is_some() {
                        return Err(Error::Topology);
                    }
                }
                _ => return Err(Error::Topology),
            }
        }
        if outgoing.keys().ne(incoming.keys()) { return Err(Error::Topology); }
        while let Some((&start,_)) = outgoing.first_key_value() {
            let mut at = start; let mut edges = Vec::new();
            loop {
                let (next,edge,forward) = outgoing.remove(&at).ok_or(Error::Topology)?;
                edges.push((edge,forward)); at = next;
                if at == start { break; }
            }
            self.boundaries.push(Boundary {edges});
        }
        Ok(())
    }
    pub fn vertices(&self) -> &[Vertex] { &self.vertices }
    pub fn edges(&self) -> &[Edge] { &self.edges }
    pub fn cells(&self) -> &[Cell] { &self.cells }
    pub fn boundaries(&self) -> &[Boundary] { &self.boundaries }
    /// Native (edge, roll) orientation into (w, projected coordinate), not an
    /// outward material normal. Every cell boundary uses the latter chart's CCW.
    pub fn consumer_orientation(&self,region:usize) -> Result<i8> {
        let r = self.correspondence.regions.get(region).ok_or(Error::InvalidOptions)?;
        Ok(-self.axis_sign as i8*r.orientation())
    }
    /// Unique region indices, in the same order as `edge_at` returns their maps.
    pub fn edge_consumers(&self,edge:usize) -> Result<Vec<usize>> {
        let e = self.edges.get(edge).ok_or(Error::InvalidOptions)?;
        let mut consumers:Vec<_> = e.uses.iter().flat_map(|u| self.cells[u.cell].consumers.iter().copied()).collect();
        consumers.sort(); consumers.dedup(); Ok(consumers)
    }
    /// Numerical chart coordinates within a cell. The analytic cut and line
    /// enclosures remain available at the vertices; rounding is not topology.
    pub fn coordinates(&self,cell:usize,uv:[f64;2]) -> Result<[f64;2]> {
        if uv.iter().any(|v| !v.is_finite() || !(0. ..=1.).contains(v)) { return Err(Error::InvalidOptions); }
        let c = self.cells.get(cell).ok_or(Error::InvalidOptions)?;
        let [lo,hi] = [self.cuts[c.cut].value.mid(),self.cuts[c.cut+1].value.mid()];
        let w = lo*(1.-uv[0])+hi*uv[0];
        let lower = self.lines[c.lower].at(W::point(w)?)?.mid();
        let upper = self.lines[c.upper].at(W::point(w)?)?.mid();
        Ok([w,lower*(1.-uv[1])+upper*uv[1]])
    }
    /// All original consumers of the cell, evaluated on their own native charts.
    /// Numerical inversion is checked by 3c; this does not classify visibility.
    pub fn consumers_at(&self,cell:usize,uv:[f64;2],agreement:f64) -> Result<Vec<Mapped>> {
        let p = self.coordinates(cell,uv)?; let c = self.cells.get(cell).ok_or(Error::InvalidOptions)?;
        self.map_consumers(p,&c.consumers,agreement)
    }
    /// Evaluate a shared arc in canonical vertex order on every incident consumer.
    /// Lines are straight only in this chart; the returned 3D positions lie on
    /// the source surfaces. Adjacent cell ownership does not duplicate a consumer.
    pub fn edge_at(&self,edge:usize,t:f64,agreement:f64) -> Result<Vec<Mapped>> {
        if !t.is_finite() || !(0. ..=1.).contains(&t) { return Err(Error::InvalidOptions); }
        let e = self.edges.get(edge).ok_or(Error::InvalidOptions)?;
        let [a,b] = e.vertices.map(|i| self.vertices[i].coordinates);
        let p = std::array::from_fn(|k| a[k]*(1.-t)+b[k]*t);
        self.map_consumers(p,&self.edge_consumers(edge)?,agreement)
    }
    fn map_consumers(&self,p:[f64;2],consumers:&[usize],agreement:f64) -> Result<Vec<Mapped>> {
        let x = self.radius2-p[0]*p[0];
        if !(x > 0.) { return Err(Error::Predicate); }
        let common = [self.correspondence.centre()[self.correspondence.axis]+self.axis_sign*x.sqrt(),
            self.correspondence.centre()[self.retained]+p[1]];
        consumers.iter().map(|&region| {
            let seed = self.correspondence.regions[region].domain.map(|b| 0.5*(b[0]+b[1]));
            Ok(self.correspondence.inverse(region,common,seed,agreement)?)
        }).collect()
    }
    /// A sampled chart-area diagnostic, not surface area or an interval proof.
    pub fn chart_area(&self) -> f64 {
        self.cells.iter().map(|c| {
            let (a,b) = (self.cuts[c.cut].value.mid(),self.cuts[c.cut+1].value.mid());
            let height = |w| (self.lines[c.upper].a.value.mid()-self.lines[c.lower].a.value.mid())*w
                +self.lines[c.upper].b.value.mid()-self.lines[c.lower].b.value.mid();
            0.5*(b-a)*(height(a)+height(b))
        }).sum()
    }
}
