//! A census of continuous sweeps outside the admitted classes (#239's first step,
//! `docs/general-sweeps.md`): before any construction, what the envelope of each looks like.
//! For every face of the tool (and every convex crease's fan of normals, and a prism's caps) the
//! contact condition f(u, v, t) = n·velocity is sampled on a grid over the face and the roll, and
//! its zero set read as the marching cubes read a field's:
//!
//! - **roots per tool point**: sign changes of f along the roll at one (u, v);
//! - **branches per latitude**: the pieces of {f = 0} over (v, t) at one u (a ring of a revolution,
//!   a line across a prism), those reaching a roll limit and those closing on themselves;
//! - **sheets**: the pieces of {f = 0} over (u, v, t);
//! - **folds**: where g = f_t − a f_u − b f_v changes sign along the zero set, (a, b) the velocity
//!   in the moved tangent frame — the generated area factor times the rate of the contact
//!   condition, admission's E3 sign (`solid::admission`), changing only where the envelope folds;
//! - **crossings**: contacts near one another in space whose tool points lie far apart, at an
//!   angle: where sheets cross one another or themselves;
//! - **exposure**: which of all these the swept solid's material field (`MaterialField`, the
//!   referee) puts on its boundary, a step out along the tool's outward normal reading outside —
//!   and the same of the tool at each roll limit (the end caps).
//!
//! A census, not a construction: counts depend on the grid and say so. The cases are the swept
//! stock the corpus writes (`swept_tumble`, `swept_spring`, `swept_groove`'s cutter) and the
//! negative controls the removed general sweep closure left refused (`docs/generating-sweeps.md`),
//! as its tests wrote them. `the_census_of_every_case` prints the table the doc records;
//! the assertions hold the facts the doc's conclusions rest on.
use fixtures::{motions,tools};
use gcs_core::{envelope::Motion,motion::Family,solid::{MaterialField,SpatialField,SweepContacts,ToolSurface}};
use std::collections::{BTreeMap,BTreeSet};

type V = [f64;3];

fn sub(a: V,b: V) -> V { std::array::from_fn(|k| a[k]-b[k]) }
fn add(a: V,b: V) -> V { std::array::from_fn(|k| a[k]+b[k]) }
fn scale(a: V,s: f64) -> V { a.map(|x| x*s) }
fn dot(a: V,b: V) -> f64 { a[0]*b[0]+a[1]*b[1]+a[2]*b[2] }
fn cross(a: V,b: V) -> V { [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]] }
fn norm(a: V) -> f64 { dot(a,a).sqrt() }
fn unit(a: V) -> Option<V> { let n = norm(a); (n > 1e-14 && n.is_finite()).then(|| scale(a,1./n)) }

/// How finely the census samples: `rows` steps of u, `columns` of v, `times` of the roll.
#[derive(Clone,Copy)]
struct Grid { rows: usize,columns: usize,times: usize }

const GRID: Grid = Grid {rows:24,columns:72,times:120};

/// What a chart is of: a face of the tool, the fan of normals a convex crease sweeps, a prism's cap.
#[derive(Clone,Copy,PartialEq,Debug)]
enum Kind { Face,Fan,Cap }

/// A face the census walks: (u, v) in [0, 1]², v round a turn where `periodic`, to a tool point at
/// rest and its outward unit normal, or none off the tool's boundary (a face a source Boolean hid).
struct Chart<'a> {
    name: String,
    kind: Kind,
    periodic: bool,
    at: Box<dyn Fn(f64,f64) -> Option<(V,V)>+Sync+'a>,
}

/// The tool's material, for the outward normal and whether a point is on the tool's boundary.
struct Tool<'a> { field: &'a SpatialField,step: f64 }

impl Tool<'_> {
    fn on(&self,p: V) -> bool { self.field.value(p).abs() < 1e-6 }
    /// `n` turned to point out of the tool at `p`.
    fn outward(&self,p: V,n: V) -> V {
        if self.field.value(add(p,scale(n,self.step))) >= self.field.value(sub(p,scale(n,self.step))) { n } else { scale(n,-1.) }
    }
}

/// The charts of a sweep's tool: each face; each convex crease's fan; each prism loop's two caps.
fn charts<'a>(c: &'a SweepContacts,tool: &'a Tool<'a>) -> Vec<Chart<'a>> {
    let mut out = Vec::new();
    for s in c.patches() {
        let [du,dv] = s.domain();
        let at = move |u: f64,v: f64| -> Option<(V,V)> {
            let p = s.at(du[0]+u*(du[1]-du[0]),dv[0]+v*(dv[1]-dv[0])).ok()?;
            if !tool.on(p.position) { return None; }
            let n = unit(cross(p.du,p.dv))?;
            Some((p.position,tool.outward(p.position,n)))
        };
        out.push(Chart {name:s.name().to_string(),kind:Kind::Face,periodic:s.periodic(),at:Box::new(at)});
    }
    for e in c.edges() {
        let (a,b) = (&c.patches()[e.patches[0]],&c.patches()[e.patches[1]]);
        let [ua,ub] = e.ends;
        let face = |s: &'a ToolSurface,u: f64,v: f64| -> Option<(V,V,V)> {
            let [du,dv] = s.domain();
            let p = s.at(du[0]+u*(du[1]-du[0]),dv[0]+v*(dv[1]-dv[0])).ok()?;
            // a point a little into the face from the edge
            let inner = s.at(du[0]+(if u == 0. { 1e-4 } else { 1.-1e-4 })*(du[1]-du[0]),dv[0]+v*(dv[1]-dv[0])).ok()?.position;
            Some((p.position,tool.outward(p.position,unit(cross(p.du,p.dv))?),inner))
        };
        let at = move |s: f64,v: f64| -> Option<(V,V)> {
            let ((p,na,_),(q,nb,inner)) = (face(a,ua,v)?,face(b,ub,v)?);
            if norm(sub(p,q)) > 1e-7*(1.+norm(p)) || !tool.on(p) { return None; }
            // convex where the second face falls away behind the first's tangent plane; a concave
            // crease (or a tangent one) sweeps nothing of its own
            let turn = dot(na,nb).clamp(-1.,1.).acos();
            if turn < 1e-3 || dot(na,sub(inner,p)) >= 0. { return None; }
            let w = turn*s;
            let n = add(scale(na,(turn-w).sin()/turn.sin()),scale(nb,w.sin()/turn.sin()));
            Some((p,unit(n)?))
        };
        out.push(Chart {name:format!("{}|{}",a.name(),b.name()),kind:Kind::Fan,periodic:a.periodic(),at:Box::new(at)});
    }
    // a prism's caps: each loop of its sides, walked by the edges between them, closed by its
    // centre (a census of a convex profile; a point off a concave one's cap is off the tool)
    let sides: Vec<usize> = (0..c.patches().len()).filter(|&k| matches!(c.patches()[k],ToolSurface::Extruded(_))).collect();
    let mut seen = BTreeSet::new();
    for &first in &sides {
        if seen.contains(&first) { continue; }
        let mut chain = vec![first];
        seen.insert(first);
        while let Some(next) = c.edges().iter().find_map(|e| {
            let last = *chain.last().unwrap();
            if e.patches[0] == last && !seen.contains(&e.patches[1]) { Some(e.patches[1]) }
            else if e.patches[1] == last && !seen.contains(&e.patches[0]) { Some(e.patches[0]) } else { None }
        }) { seen.insert(next); chain.push(next); }
        for cap in [0.,1.] {
            let chain = chain.clone();
            let (m,head) = (chain.len(),chain[0]);
            let rim = move |v: f64| -> Option<V> {
                let at = v.rem_euclid(1.)*m as f64;
                let k = (at.floor() as usize).min(m-1);
                c.patches()[chain[k]].at(at-k as f64,cap).ok().map(|p| p.position)
            };
            let n = 64;
            let points: Vec<V> = (0..n).filter_map(|k| rim(k as f64/n as f64)).collect();
            if points.len() < 3 { continue; }
            let centre = scale(points.iter().fold([0.;3],|s,&p| add(s,p)),1./points.len() as f64);
            let side = &c.patches()[head];
            let (Ok(p0),Ok(p1)) = (side.at(0.5,0.),side.at(0.5,1.)) else { continue };
            let Some(along) = unit(sub(p1.position,p0.position)) else { continue };
            let at = move |u: f64,v: f64| -> Option<(V,V)> {
                let p = add(centre,scale(sub(rim(v)?,centre),u));
                if !tool.on(p) { return None; }
                Some((p,tool.outward(p,along)))
            };
            out.push(Chart {name:format!("{} cap {}",side.name(),cap as usize),kind:Kind::Cap,periodic:true,at:Box::new(at)});
        }
    }
    out
}

/// The contact condition at (u, v) of `chart` at roll `t`: the moved outward normal against the
/// point's velocity.
fn condition(chart: &Chart,motion: &Family,u: f64,v: f64,t: f64) -> Option<f64> {
    let (p,n) = (chart.at)(u,v)?;
    let pose: Motion = motion.at(t).ok()?;
    Some(dot(pose.vector(n),pose.velocity(p)))
}

/// One contact the census found, on a grid edge where f changes sign, the root refined.
#[derive(Clone,Copy,Debug)]
struct Contact {
    /// where on the tool at rest, and where in space at its time, with the moved outward normal
    source: V,position: V,normal: V,
    /// the fold sign g (none where it cannot be read: a fan, a degenerate chart)
    fold: Option<f64>,
    exposed: bool,
    sheet: usize,
}

/// The fold sign at (u, v, t): g = f_t − a f_u − b f_v, (a, b) the point's velocity in the moved
/// tangent frame. Zero where the generated surface folds (admission's E3 sign, up to the tool's area).
fn fold_sign(chart: &Chart,motion: &Family,u: f64,v: f64,t: f64) -> Option<f64> {
    if chart.kind == Kind::Fan { return None; }
    let h = 1e-5;
    let (ua,ub) = ((u-h).max(0.),(u+h).min(1.));
    let (va,vb) = if chart.periodic { (v-h,v+h) } else { ((v-h).max(0.),(v+h).min(1.)) };
    let wrap = |v: f64| if chart.periodic { v.rem_euclid(1.) } else { v };
    let f = |u: f64,v: f64,t: f64| condition(chart,motion,u,wrap(v),t);
    let ft = (f(u,v,t+h)?-f(u,v,t-h)?)/(2.*h);
    let fu = (f(ub,v,t)?-f(ua,v,t)?)/(ub-ua);
    let fv = (f(u,vb,t)?-f(u,va,t)?)/(vb-va);
    let p = |u: f64,v: f64| (chart.at)(u,wrap(v)).map(|x| x.0);
    let pose = motion.at(t).ok()?;
    let su = pose.vector(scale(sub(p(ub,v)?,p(ua,v)?),1./(ub-ua)));
    let sv = pose.vector(scale(sub(p(u,vb)?,p(u,va)?),1./(vb-va)));
    let vel = pose.velocity((chart.at)(u,v)?.0);
    // V = a su + b sv, by the normal equations
    let (m11,m12,m22) = (dot(su,su),dot(su,sv),dot(sv,sv));
    let det = m11*m22-m12*m12;
    if det <= 1e-12*m11*m22 { return None; }
    let (r1,r2) = (dot(vel,su),dot(vel,sv));
    let (a,b) = ((r1*m22-r2*m12)/det,(r2*m11-r1*m12)/det);
    Some(ft-a*fu-b*fv)
}

/// A disjoint-set forest over grid cells.
struct Sets { parent: Vec<usize> }
impl Sets {
    fn new(n: usize) -> Self { Sets {parent:(0..n).collect()} }
    fn find(&mut self,mut x: usize) -> usize {
        while self.parent[x] != x { self.parent[x] = self.parent[self.parent[x]]; x = self.parent[x]; }
        x
    }
    fn join(&mut self,a: usize,b: usize) { let (a,b) = (self.find(a),self.find(b)); if a != b { self.parent[a.max(b)] = a.min(b); } }
}

/// What one latitude (a ring of a face at one u) holds.
#[derive(Clone,Default,Debug)]
struct Latitude { pieces: usize,loops: usize,folds: usize,exposed_folds: usize,limit_folds: usize }

/// What one chart's census found.
#[derive(Clone,Debug)]
struct ChartCensus {
    name: String,
    kind: Kind,
    /// most contact times one tool point has within the roll, and how many points have each count
    most_roots: usize,
    roots: BTreeMap<usize,usize>,
    /// f does not change over the roll at any point (the motion's twist is constant), and how many
    /// points are in contact at every time
    stationary: bool,
    always: usize,
    latitudes: Vec<Latitude>,
    /// the pieces of {f = 0} over (u, v, t), and how many reach a roll limit
    sheets: usize,
    open_sheets: usize,
    /// sheets whose fold sign takes both values, and the latitudes' fold points summed
    folded_sheets: usize,
    /// which of the census's sheets (numbered across charts) fold
    folded: BTreeSet<usize>,
    folds: usize,
    contacts: Vec<Contact>,
    /// the tool at each roll limit: points on the tool, and of those on the swept solid's boundary
    ends: [(usize,usize);2],
}

fn census_of(chart: &Chart,motion: &Family,roll: [f64;2],field: &MaterialField,step: f64,grid: Grid,sheet_base: usize) -> ChartCensus {
    let (nu,nt) = (grid.rows,grid.times);
    let nv = if chart.periodic { grid.columns } else { grid.columns+1 };
    let jv = if chart.periodic { nv } else { nv-1 };
    // Nodes stepped off the faces' own lines of symmetry by irrational fractions of a step, so a
    // characteristic a symmetric fixture lays along a meridian or a ring falls between nodes: a node
    // in contact at every time is then one of a face wholly in contact.
    let u_at = |i: usize| (i as f64+0.381966)/(nu+1) as f64;
    let v_at = |j: usize| ((j%nv) as f64+0.618034)/nv as f64;
    let t_at = |k: usize| roll[0]+(roll[1]-roll[0])*k as f64/nt as f64;
    let poses: Vec<Motion> = (0..=nt).map(|k| motion.at(t_at(k)).unwrap()).collect();
    // f on the grid, NaN off the tool
    let mut rows: Vec<Vec<f64>> = gcs_core::par::indices(nu+1,|i| {
        let mut row = vec![f64::NAN;nv*(nt+1)];
        for j in 0..nv {
            let Some((p,n)) = (chart.at)(u_at(i),v_at(j)) else { continue };
            for k in 0..=nt { row[j*(nt+1)+k] = dot(poses[k].vector(n),poses[k].velocity(p)); }
        }
        row
    });
    // A point in contact at every time (a cap square to a turn's axis, a side along a slide)
    // has f nothing throughout: no root, no sheet, only rounding's signs. It is counted and
    // taken out, as off the tool, before the zero set is read.
    let largest = rows.iter().flatten().filter(|x| x.is_finite()).fold(0_f64,|m,x| m.max(x.abs()));
    let tolerance = 1e-9*(1.+largest);
    let mut always = 0;
    let mut stationary = true;
    for row in rows.iter_mut() { for j in 0..nv {
        let column = &mut row[j*(nt+1)..(j+1)*(nt+1)];
        if !column[0].is_finite() { continue; }
        if column.iter().all(|x| x.abs() <= tolerance) { always += 1; column.fill(f64::NAN); continue; }
        let (lo,hi) = column.iter().fold((f64::INFINITY,f64::NEG_INFINITY),|(lo,hi),&x| (lo.min(x),hi.max(x)));
        if hi-lo > tolerance { stationary = false; }
    } }
    let f = |i: usize,j: usize,k: usize| rows[i][(j%nv)*(nt+1)+k];
    let sign = |x: f64| x > 0.;
    // roots per point
    let mut roots: BTreeMap<usize,usize> = BTreeMap::new();
    for i in 0..=nu { for j in 0..nv {
        if !f(i,j,0).is_finite() { continue; }
        *roots.entry((0..nt).filter(|&k| sign(f(i,j,k)) != sign(f(i,j,k+1))).count()).or_default() += 1;
    } }
    // the contact on an edge where f changes sign, refined by bisection: the edge from node
    // (i, j, k) one step along v (dir 0) or t (dir 1)
    let refine = |i: usize,j: usize,k: usize,dir: usize| -> Option<(f64,f64,f64)> {
        let u = u_at(i);
        let (mut a,mut b) = (0.,1.);
        let at = |s: f64| if dir == 0 {
            let v0 = v_at(j); let v1 = if chart.periodic && j+1 == nv { v_at(0)+1. } else { v_at(j+1) };
            (v0+s*(v1-v0),t_at(k))
        } else { (v_at(j),t_at(k)+s*(t_at(k+1)-t_at(k))) };
        let wrap = |v: f64| if chart.periodic { v.rem_euclid(1.) } else { v };
        let value = |s: f64| { let (v,t) = at(s); condition(chart,motion,u,wrap(v),t) };
        let fa = value(a)?;
        for _ in 0..48 {
            let m = 0.5*(a+b);
            let fm = value(m)?;
            if sign(fm) == sign(fa) { a = m; } else { b = m; }
        }
        let (v,t) = at(0.5*(a+b));
        Some((u,wrap(v),t))
    };
    // the sheets: pieces of {f = 0} over the grid's cubes
    let cube = |i: usize,j: usize,k: usize| (i*jv+j)*nt+k;
    let corners = |i: usize,j: usize,k: usize| [(i,j,k),(i+1,j,k),(i,j+1,k),(i+1,j+1,k),(i,j,k+1),(i+1,j,k+1),(i,j+1,k+1),(i+1,j+1,k+1)];
    let mixed = |nodes: &[(usize,usize,usize)]| {
        let values: Vec<f64> = nodes.iter().map(|&(i,j,k)| f(i,j,k)).collect();
        values.iter().all(|x| x.is_finite()) && values.iter().any(|&x| sign(x)) && values.iter().any(|&x| !sign(x))
    };
    let active: Vec<bool> = (0..nu*jv*nt).map(|c| { let (i,j,k) = (c/(jv*nt),(c/nt)%jv,c%nt); mixed(&corners(i,j,k)) }).collect();
    let mut sets = Sets::new(nu*jv*nt);
    for i in 0..nu { for j in 0..jv { for k in 0..nt {
        if !active[cube(i,j,k)] { continue; }
        if i+1 < nu && active[cube(i+1,j,k)] && mixed(&[(i+1,j,k),(i+1,j+1,k),(i+1,j,k+1),(i+1,j+1,k+1)]) { sets.join(cube(i,j,k),cube(i+1,j,k)); }
        let jn = j+1;
        if (jn < jv || chart.periodic) && active[cube(i,jn%jv,k)] && mixed(&[(i,jn,k),(i+1,jn,k),(i,jn,k+1),(i+1,jn,k+1)]) { sets.join(cube(i,j,k),cube(i,jn%jv,k)); }
        if k+1 < nt && active[cube(i,j,k+1)] && mixed(&[(i,j,k+1),(i+1,j,k+1),(i,j+1,k+1),(i+1,j+1,k+1)]) { sets.join(cube(i,j,k),cube(i,j,k+1)); }
    } } }
    let mut sheets: BTreeMap<usize,usize> = BTreeMap::new();
    let mut open: BTreeSet<usize> = BTreeSet::new();
    for i in 0..nu { for j in 0..jv { for k in 0..nt {
        if !active[cube(i,j,k)] { continue; }
        let root = sets.find(cube(i,j,k));
        let next = sheets.len();
        sheets.entry(root).or_insert(next);
        let limit = |k: usize| mixed(&[(i,j,k),(i+1,j,k),(i,j+1,k),(i+1,j+1,k)]);
        if (k == 0 && limit(0)) || (k+1 == nt && limit(nt)) { open.insert(root); }
    } } }
    let sheet_of = |sets: &mut Sets,i: usize,j: usize,k: usize,dir: usize| -> Option<usize> {
        // the column before: round a turn the last, along a prism none before the first
        let before = if j > 0 { j-1 } else if chart.periodic { jv-1 } else { usize::MAX };
        let near: Vec<(usize,usize,usize)> = if dir == 0 {
            [(i.wrapping_sub(1),j,k.wrapping_sub(1)),(i.wrapping_sub(1),j,k),(i,j,k.wrapping_sub(1)),(i,j,k)].into()
        } else {
            [(i.wrapping_sub(1),before,k),(i.wrapping_sub(1),j,k),(i,before,k),(i,j,k)].into()
        };
        near.into_iter().filter(|&(i,j,k)| i < nu && j < jv && k < nt && active[cube(i,j,k)])
            .map(|(i,j,k)| sheets[&sets.find(cube(i,j,k))]).next()
    };
    // every contact on a v or t edge, with its fold sign and whether the solid's boundary holds it
    let mut edges: Vec<(usize,usize,usize,usize)> = Vec::new();
    for i in 0..=nu { for j in 0..nv { for k in 0..=nt {
        let here = f(i,j,k);
        if !here.is_finite() { continue; }
        if j < jv && f(i,j+1,k).is_finite() && sign(here) != sign(f(i,j+1,k)) { edges.push((i,j,k,0)); }
        if k < nt && f(i,j,k+1).is_finite() && sign(here) != sign(f(i,j,k+1)) { edges.push((i,j,k,1)); }
    } } }
    let found: Vec<Option<(V,V,V,Option<f64>,bool)>> = gcs_core::par::indices(edges.len(),|e| {
        let (i,j,k,dir) = edges[e];
        let (u,v,t) = refine(i,j,k,dir)?;
        let (p,n) = (chart.at)(u,v)?;
        let pose = motion.at(t).ok()?;
        let (position,normal) = (pose.point(p),pose.vector(n));
        let exposed = field.side(add(position,scale(normal,step))) > 0.;
        Some((p,position,normal,fold_sign(chart,motion,u,v,t),exposed))
    });
    let mut contacts = Vec::new();
    let mut index: BTreeMap<(usize,usize,usize,usize),usize> = BTreeMap::new();
    let mut signs: BTreeMap<usize,[bool;2]> = BTreeMap::new();
    for (&(i,j,k,dir),x) in edges.iter().zip(found) {
        let Some((source,position,normal,fold,exposed)) = x else { continue };
        let Some(sheet) = sheet_of(&mut sets,i,j,k,dir) else { continue };
        if let Some(g) = fold { if g != 0. { signs.entry(sheet).or_default()[usize::from(g > 0.)] = true; } }
        index.insert((i,j,k,dir),contacts.len());
        contacts.push(Contact {source,position,normal,fold,exposed,sheet:sheet_base+sheet});
    }
    // the latitudes: pieces of {f = 0} over (v, t) at each u, by the squares of that slice
    let mut latitudes = Vec::new();
    for i in 0..=nu {
        let square = |j: usize,k: usize| j*nt+k;
        let mixed2 = |nodes: &[(usize,usize)]| mixed(&nodes.iter().map(|&(j,k)| (i,j,k)).collect::<Vec<_>>());
        let on: Vec<bool> = (0..jv*nt).map(|s| { let (j,k) = (s/nt,s%nt); mixed2(&[(j,k),(j+1,k),(j,k+1),(j+1,k+1)]) }).collect();
        let mut sets = Sets::new(jv*nt);
        for j in 0..jv { for k in 0..nt {
            if !on[square(j,k)] { continue; }
            if (j+1 < jv || chart.periodic) && on[square((j+1)%jv,k)] && mixed2(&[(j+1,k),(j+1,k+1)]) { sets.join(square(j,k),square((j+1)%jv,k)); }
            if k+1 < nt && on[square(j,k+1)] && mixed2(&[(j,k+1),(j+1,k+1)]) { sets.join(square(j,k),square(j,k+1)); }
        } }
        let mut pieces: BTreeSet<usize> = BTreeSet::new();
        let mut reaching: BTreeSet<usize> = BTreeSet::new();
        let (mut folds,mut exposed_folds,mut limit_folds) = (0,0,0);
        for j in 0..jv { for k in 0..nt {
            if !on[square(j,k)] { continue; }
            let root = sets.find(square(j,k));
            pieces.insert(root);
            if (k == 0 && mixed2(&[(j,0),(j+1,0)])) || (k+1 == nt && mixed2(&[(j,nt),(j+1,nt)])) { reaching.insert(root); }
            // the square's crossings: a fold between two whose signs differ
            let ends: Vec<&Contact> = [(j,k,0),(j,k+1,0),(j,k,1),((j+1)%nv,k,1)].into_iter()
                .filter_map(|(j,k,d)| index.get(&(i,j,k,d)).map(|&c| &contacts[c])).collect();
            if let [a,b] = ends[..] { if let (Some(fa),Some(fb)) = (a.fold,b.fold) { if fa*fb < 0. {
                folds += 1;
                // a fold may reach the boundary where it runs into the tool's last pose, the end
                // cap's edge: counted apart from those in the roll
                if a.exposed && b.exposed { if k < 2 || k+2 >= nt { limit_folds += 1; } else { exposed_folds += 1; } }
            } } }
        } }
        latitudes.push(Latitude {pieces:pieces.len(),loops:pieces.len()-reaching.len(),folds,exposed_folds,limit_folds});
    }
    // the tool at each roll limit
    let mut ends = [(0,0);2];
    if chart.kind != Kind::Fan {
        for (end,&t) in roll.iter().enumerate() {
            let pose = motion.at(t).unwrap();
            for i in 0..=nu { for j in 0..nv {
                let Some((p,n)) = (chart.at)(u_at(i),v_at(j)) else { continue };
                ends[end].0 += 1;
                if field.side(add(pose.point(p),scale(pose.vector(n),step))) > 0. { ends[end].1 += 1; }
            } }
        }
    }
    let folds = latitudes.iter().map(|l| l.folds).sum();
    ChartCensus {name:chart.name.clone(),kind:chart.kind,most_roots:roots.keys().copied().max().unwrap_or(0),roots,stationary,always,
        latitudes,sheets:sheets.len(),open_sheets:open.len(),folded_sheets:signs.values().filter(|s| s[0] && s[1]).count(),
        folded:signs.iter().filter(|(_,s)| s[0] && s[1]).map(|(&k,_)| sheet_base+k).collect(),
        folds,contacts,ends}
}

/// Where the census's contacts cross: pairs near one another in space (within the sampling's own
/// spacing) whose tool points lie far apart and whose normals differ by more than 11°, binned into
/// sites four spacings wide; a site is steep where some pair there differs by more than 26°.
/// Shallow sites are as often two layers of one sheet doubled back beside a fold as a crossing.
#[derive(Clone,Debug,Default)]
struct Crossings { spacing: f64,pairs: usize,sites: usize,steep_sites: usize,exposed_sites: usize,self_sites: usize,tangent_pairs: usize }

fn crossings(contacts: &[Contact]) -> Crossings {
    if contacts.len() < 2 { return Crossings::default(); }
    // the spacing: the median distance from a contact to its nearest neighbour
    let (lo,hi) = contacts.iter().fold(([f64::INFINITY;3],[f64::NEG_INFINITY;3]),|(lo,hi),c|
        (std::array::from_fn(|k| lo[k].min(c.position[k])),std::array::from_fn(|k| hi[k].max(c.position[k]))));
    let guess = norm(sub(hi,lo))/(contacts.len() as f64).sqrt();
    let bins = |size: f64| {
        let mut bins: BTreeMap<[i64;3],Vec<usize>> = BTreeMap::new();
        for (k,c) in contacts.iter().enumerate() { bins.entry(c.position.map(|x| (x/size).floor() as i64)).or_default().push(k); }
        bins
    };
    let near = |bins: &BTreeMap<[i64;3],Vec<usize>>,size: f64,k: usize| -> Vec<usize> {
        let key = contacts[k].position.map(|x| (x/size).floor() as i64);
        let mut out = Vec::new();
        for dx in -1..=1 { for dy in -1..=1 { for dz in -1..=1 {
            if let Some(list) = bins.get(&[key[0]+dx,key[1]+dy,key[2]+dz]) { out.extend(list.iter().copied().filter(|&m| m != k)); }
        } } }
        out
    };
    let coarse = bins(guess);
    let mut nearest: Vec<f64> = (0..contacts.len()).map(|k| near(&coarse,guess,k).into_iter()
        .map(|m| norm(sub(contacts[k].position,contacts[m].position))).filter(|&d| d > 1e-12).fold(f64::INFINITY,f64::min))
        .filter(|d| d.is_finite()).collect();
    nearest.sort_by(f64::total_cmp);
    let spacing = nearest.get(nearest.len()/2).copied().unwrap_or(guess);
    let fine = bins(spacing);
    let mut out = Crossings {spacing,..Default::default()};
    let mut sites: BTreeMap<[i64;3],[bool;3]> = BTreeMap::new();
    for k in 0..contacts.len() {
        let a = &contacts[k];
        for m in near(&fine,spacing,k) {
            if m <= k { continue; }
            let b = &contacts[m];
            if norm(sub(a.position,b.position)) > 0.75*spacing || norm(sub(a.source,b.source)) < 8.*spacing { continue; }
            let cosine = dot(a.normal,b.normal).abs();
            if cosine > 0.98 { out.tangent_pairs += 1; continue; }
            out.pairs += 1;
            let site = sites.entry(scale(add(a.position,b.position),0.5).map(|x| (x/(4.*spacing)).floor() as i64)).or_default();
            site[0] |= a.exposed && b.exposed;
            site[1] |= a.sheet == b.sheet;
            site[2] |= cosine < 0.9;
        }
    }
    out.sites = sites.len();
    out.exposed_sites = sites.values().filter(|s| s[0]).count();
    out.self_sites = sites.values().filter(|s| s[1]).count();
    out.steep_sites = sites.values().filter(|s| s[2]).count();
    out
}

/// One case's census: every chart's, and where their contacts cross.
struct Census { name: String,motion: String,charts: Vec<ChartCensus>,crossings: Crossings }

impl Census {
    fn contacts(&self) -> impl Iterator<Item=&Contact> { self.charts.iter().flat_map(|c| c.contacts.iter()) }
    fn most_roots(&self) -> usize { self.charts.iter().map(|c| c.most_roots).max().unwrap_or(0) }
    fn most_pieces(&self) -> usize { self.charts.iter().flat_map(|c| c.latitudes.iter().map(|l| l.pieces)).max().unwrap_or(0) }
    fn loops(&self) -> usize { self.charts.iter().flat_map(|c| c.latitudes.iter().map(|l| l.loops)).max().unwrap_or(0) }
    fn sheets(&self) -> usize { self.charts.iter().map(|c| c.sheets).sum() }
    fn folded_sheets(&self) -> usize { self.charts.iter().map(|c| c.folded_sheets).sum() }
    fn folds(&self) -> usize { self.charts.iter().map(|c| c.folds).sum() }
    fn exposed_folds(&self) -> usize { self.charts.iter().flat_map(|c| c.latitudes.iter().map(|l| l.exposed_folds)).sum() }
    fn limit_folds(&self) -> usize { self.charts.iter().flat_map(|c| c.latitudes.iter().map(|l| l.limit_folds)).sum() }
    fn stationary(&self) -> bool { self.charts.iter().filter(|c| c.kind != Kind::Fan).all(|c| c.stationary) }
    fn exposed(&self) -> f64 {
        let (n,e) = self.contacts().fold((0,0),|(n,e),c| (n+1,e+usize::from(c.exposed)));
        e as f64/n.max(1) as f64
    }
    fn end_exposed(&self,end: usize) -> f64 {
        let (n,e) = self.charts.iter().fold((0,0),|(n,e),c| (n+c.ends[end].0,e+c.ends[end].1));
        e as f64/n.max(1) as f64
    }
    fn row(&self) -> String {
        let x = &self.crossings;
        format!("| {} | {} | {} | {} | {} | {} | {} ({} folded) | {} ({} exposed, {} at a limit) | {}/{}/{}/{} | {:.0}% | {:.0}% / {:.0}% |",
            self.name,self.motion,if self.stationary() { "constant" } else { "varying" },self.most_roots(),self.most_pieces(),self.loops(),
            self.sheets(),self.folded_sheets(),self.folds(),self.exposed_folds(),self.limit_folds(),x.sites,x.steep_sites,x.exposed_sites,x.self_sites,
            100.*self.exposed(),100.*self.end_exposed(0),100.*self.end_exposed(1))
    }
    fn detail(&self) -> String {
        let mut s = format!("== {} ({}), crossing spacing {:.4}, {} pairs, {} tangent\n",self.name,self.motion,
            self.crossings.spacing,self.crossings.pairs,self.crossings.tangent_pairs);
        for c in &self.charts {
            let exposed = c.contacts.iter().filter(|x| x.exposed).count();
            let pieces: BTreeMap<usize,usize> = c.latitudes.iter().fold(BTreeMap::new(),|mut m,l| { *m.entry(l.pieces).or_default() += 1; m });
            s += &format!("   {:?} {}: roots per point {:?}{}, pieces per latitude {:?}, {} sheets ({} open, {} folded), \
                {} fold points, {} contacts ({} exposed), ends {:?}\n",c.kind,c.name,c.roots,
                if c.stationary { format!(" (stationary, {} always in contact)",c.always) } else { format!(" ({} always in contact)",c.always) },
                pieces,c.sheets,c.open_sheets,c.folded_sheets,c.folds,c.contacts.len(),exposed,c.ends);
            let mut sheets: BTreeMap<usize,(usize,usize)> = BTreeMap::new();
            for x in &c.contacts { let e = sheets.entry(x.sheet).or_default(); e.0 += 1; e.1 += usize::from(x.exposed); }
            for (k,(n,e)) in sheets {
                s += &format!("      sheet {k}: {n} contacts, {:.0}% exposed{}\n",100.*e as f64/n as f64,if c.folded.contains(&k) { ", folds" } else { "" });
            }
        }
        s
    }
}

/// The census of the swept solid `swept` of `source`, under a motion described as `motion`.
fn census(name: &str,motion: &str,source: &str,swept: &str,grid: Grid) -> Census {
    let e = fixtures::read(source);
    let swept = fixtures::solid(&e,swept);
    let c = SweepContacts::read(&e.sketch,swept,1e-10).unwrap();
    let field = MaterialField::read(&e.sketch,swept,1e-10).unwrap();
    let tool_field = c.source_material();
    // a step out of the solid a ten-thousandth of the tool's size
    let size = c.patches().iter().flat_map(|s| [0.,0.5,1.].into_iter().filter_map(move |u| s.at(u,0.).ok().map(|p| p.position)))
        .fold(([f64::INFINITY;3],[f64::NEG_INFINITY;3]),|(lo,hi),p| (std::array::from_fn(|k| lo[k].min(p[k])),std::array::from_fn(|k| hi[k].max(p[k]))));
    let step = 1e-4*norm(sub(size.1,size.0)).max(1e-3);
    let tool = Tool {field:tool_field,step};
    let mut charts_census = Vec::new();
    let mut base = 0;
    for chart in charts(&c,&tool) {
        let x = census_of(&chart,c.motion(),c.domain(),&field,step,grid,base);
        base += x.sheets;
        charts_census.push(x);
    }
    let all: Vec<Contact> = charts_census.iter().flat_map(|c| c.contacts.iter().copied()).collect();
    Census {name:name.into(),motion:motion.into(),crossings:crossings(&all),charts:charts_census}
}

fn case(tool: &str,motion: &str,line: String) -> String { format!("{tool}{motion}{line}") }

/// The cases: the corpus's swept stock, then the negative controls as the removed closure's
/// tests wrote them (`git show 4bedee36~1:rust/gcs-core/tests/sweep_mesh/grazing.rs`).
fn cases() -> Vec<(&'static str,&'static str,String,&'static str)> {
    vec![
        ("swept_tumble","turn relative to a skew turn",include_str!("../../examples/swept_tumble.sv").into(),"tumbled"),
        ("swept_spring","screw",include_str!("../../examples/swept_spring.sv").into(),"spring"),
        ("swept_groove (cutter)","single turn",include_str!("../../examples/swept_groove.sv").into(),"groove"),
        ("tumbling cylinder","single turn",case(tools::CYLINDER,motions::TUMBLE,motions::swept("turn",-30.,30.)),"swept"),
        ("tilted cylinder 30","single turn",case(&tools::tilted_cylinder(30.),motions::TURN_SPINDLE,motions::swept("turn",-60.,60.)),"swept"),
        ("tilted cylinder 85","single turn",case(&tools::tilted_cylinder(85.),motions::TURN_SPINDLE,motions::swept("turn",-60.,60.)),"swept"),
        ("tilted cylinder 5, slid","translation",case(&tools::tilted_cylinder(5.),&motions::slide_x(6.),motions::swept("feed",0.,360.)),"swept"),
        ("thin plate under a roll","turn relative to a turn",case(&tools::thin_plate(0.1),&motions::roll(1.05),motions::swept("turn",0.,120.)),"swept"),
        ("sliding dumbbell","translation",case(tools::DUMBBELL,&motions::slide_x(4.),motions::swept("feed",0.,360.)),"swept"),
    ]
}

/// Every case's census, printed as `docs/general-sweeps.md` records it (`SWEEP_CENSUS_FINE=1` reads
/// each at twice the grid in every direction), and the facts the doc's conclusions rest on.
#[test]
fn the_census_of_every_case() {
    eprintln!("| case | motion | twist | roots/point | pieces/latitude | loops | sheets | fold points | crossing sites (steep/exposed/self) | envelope exposed | ends exposed |");
    let mut details = String::new();
    let grid = match std::env::var("SWEEP_CENSUS_FINE") { Ok(_) => Grid {rows:48,columns:144,times:240},Err(_) => GRID };
    let mut all = Vec::new();
    for (name,motion,source,swept) in cases() {
        let began = std::time::Instant::now();
        let c = census(name,motion,&source,swept,grid);
        eprintln!("{}",c.row());
        details += &c.detail();
        details += &format!("   ({:.1?})\n",began.elapsed());
        all.push(c);
    }
    eprintln!("{details}");
    for c in &all {
        // no tool point touches the envelope twice within the roll, and no ring of the tool holds
        // more than two pieces of contact: the several-contacts difficulty does not arise here
        assert!(c.most_roots() <= 1,"{}: {} roots at one tool point",c.name,c.most_roots());
        assert!(c.most_pieces() <= 2,"{}: {} pieces at one latitude",c.name,c.most_pieces());
        // a single turn, a translation and a screw have a constant twist (the contact fixed on the
        // tool, its characteristic); a turn seen from another turn does not
        assert_eq!(c.stationary(),!c.motion.contains("relative"),"{}",c.name);
        // swept stock: the tool at each roll limit is boundary, which no admitted class builds
        for end in 0..2 { assert!(c.end_exposed(end) > 0.1,"{}: the tool at limit {end} is {:.0}% boundary",c.name,100.*c.end_exposed(end)); }
        // folds lie off the boundary, but where one runs into the tool's last pose
        assert_eq!(c.exposed_folds(),0,"{}",c.name);
        let x = &c.crossings;
        if c.name == "swept_tumble" {
            // two of its three sheets fold, each doubling back on itself beside its folds, all of
            // that buried: half its envelope is boundary
            assert_eq!((c.sheets(),c.folded_sheets()),(3,2));
            assert!(x.sites > 0 && x.exposed_sites == 0 && x.self_sites == x.sites,"{x:?}");
            assert!(c.exposed() < 0.6,"{:.2}",c.exposed());
        } else {
            assert_eq!((c.folded_sheets(),x.sites),(0,0),"{}",c.name);
        }
    }
}
