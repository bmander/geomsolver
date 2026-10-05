//! The configured hypoid pair checked from its fabrication files (docs/native-hypoid-plan.md):
//! the two STLs `solventc --tolerance` writes, read as triangles and nothing else.
//!
//! - **The shafts.** Each member's axis is the axis of its solid's inertia (a body with N-fold
//!   symmetry, N ≥ 3, is transversely isotropic about it), through its centroid; the shaft angle
//!   is the angle between them and the offset the length of their common perpendicular.
//! - **The mesh.** The gear turns through one of its pitches and the pinion the tooth ratio as
//!   far, the sense found by trying both. At every pose the least distance between the two meshes
//!   is measured by a bounding-volume search over triangle pairs, exactly (edge against edge,
//!   corner against face, and zero where an edge pierces a face), once over the pinion's triangles
//!   facing its positive turn and once over those facing the other way: the clearance on each
//!   flank pair. A positive distance at every pose is no overlap, the members' shells being closed
//!   and disjoint.
//! - **The backlash.** The design stands every flank a quarter of the normal backlash inside its
//!   conjugate, so with the pair as laid out each flank pair is apart by half of it along their
//!   common normal. Turning the pinion alone until one flank pair touches (the least clearance
//!   over the whole pitch zero) opens the other: that clearance, along the normal, with the other
//!   flanks touching, is the normal backlash as the configuration states it.
//! - **The contact pattern.** With one flank pair touching, the pinion's triangles within a film
//!   of marking compound of the gear at some pose of the pitch, where they lie along the face.
//!
//! Distances are between the files' triangles, each within the export's tolerance of the exact
//! surface, so a clearance is good to about twice it.
use gcs_core::space::{add,cross,dot,norm,scale,sub};

type V = [f64;3];
type Tri = [V;3];

// --- vectors and rotations ---

fn unit(a: V) -> V { scale(a,1./norm(a)) }

/// The rotation by `angle` radians about the unit `axis`, as a matrix (rows).
fn rotation(axis: V,angle: f64) -> [V;3] {
    let (s,c) = angle.sin_cos();
    let [x,y,z] = axis;
    let t = 1.-c;
    [[c+x*x*t,x*y*t-z*s,x*z*t+y*s],[y*x*t+z*s,c+y*y*t,y*z*t-x*s],[z*x*t-y*s,z*y*t+x*s,c+z*z*t]]
}
fn apply(m: &[V;3],v: V) -> V { [dot(m[0],v),dot(m[1],v),dot(m[2],v)] }
fn compose(a: &[V;3],b: &[V;3]) -> [V;3] {
    std::array::from_fn(|i| std::array::from_fn(|j| (0..3).map(|k| a[i][k]*b[k][j]).sum()))
}

/// An isometry `x -> m x + t`.
#[derive(Clone,Copy)]
struct Pose { m: [V;3], t: V }
impl Pose {
    fn identity() -> Self { Pose { m:[[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]], t:[0.;3] } }
    /// The turn by `angle` about the line through `at` along the unit `axis`.
    fn turn(at: V,axis: V,angle: f64) -> Self {
        let m = rotation(axis,angle);
        Pose { m, t:sub(at,apply(&m,at)) }
    }
    fn then(self,next: Pose) -> Pose { Pose { m:compose(&next.m,&self.m), t:add(apply(&next.m,self.t),next.t) } }
    fn of(&self,v: V) -> V { add(apply(&self.m,v),self.t) }
}

// --- the solid's inertia ---

/// Volume, centroid and the covariance of the material about the centroid, by the divergence
/// theorem over the closed shell's triangles (outward), about a local origin for roundoff.
fn mass(tris: &[Tri]) -> (f64,V,[V;3]) {
    let o = tris[0][0];
    let (mut vol,mut first,mut second) = (0.,[0.;3],[[0.;3];3]);
    for t in tris {
        let [a,b,c] = t.map(|p| sub(p,o));
        let v = dot(a,cross(b,c))/6.;
        let s = add(add(a,b),c);
        vol += v;
        first = add(first,scale(s,v/4.));
        for i in 0..3 { for j in 0..3 {
            second[i][j] += v/20.*(a[i]*a[j]+b[i]*b[j]+c[i]*c[j]+s[i]*s[j]);
        }}
    }
    let centre = scale(first,1./vol);
    let cov = std::array::from_fn(|i| std::array::from_fn(|j| second[i][j]/vol-centre[i]*centre[j]));
    (vol,add(centre,o),cov)
}

/// The eigenvalues and unit eigenvectors (columns, as rows here) of a symmetric 3x3 matrix, by
/// Jacobi rotations.
fn eigen(mut a: [V;3]) -> ([f64;3],[V;3]) {
    let mut v = [[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]];
    for _ in 0..64 {
        let off = a[0][1].abs()+a[0][2].abs()+a[1][2].abs();
        if off < 1e-300 { break; }
        for (p,q) in [(0,1),(0,2),(1,2)] {
            if a[p][q].abs() < 1e-300 { continue; }
            let theta = (a[q][q]-a[p][p])/(2.*a[p][q]);
            let t = theta.signum().max(0.)*2.-1.;
            let t = t/(theta.abs()+(theta*theta+1.).sqrt());
            let t = if theta == 0. { 1. } else { t };
            let (c,s) = (1./(t*t+1.).sqrt(),t/(t*t+1.).sqrt());
            let mut b = a;
            for k in 0..3 {
                b[k][p] = c*a[k][p]-s*a[k][q];
                b[k][q] = s*a[k][p]+c*a[k][q];
            }
            let mut d = b;
            for k in 0..3 {
                d[p][k] = c*b[p][k]-s*b[q][k];
                d[q][k] = s*b[p][k]+c*b[q][k];
            }
            a = d;
            for row in v.iter_mut() {
                let (x,y) = (row[p],row[q]);
                row[p] = c*x-s*y;
                row[q] = s*x+c*y;
            }
        }
    }
    let values = [a[0][0],a[1][1],a[2][2]];
    (values,std::array::from_fn(|k| [v[0][k],v[1][k],v[2][k]]))
}

/// A member's axis: the principal direction whose moment stands apart from the other two, through
/// the centroid. Also how far the other two moments are from equal, relative (the symmetry).
fn axis(tris: &[Tri]) -> (V,V,f64,f64) {
    let (vol,centre,cov) = mass(tris);
    let (values,vectors) = eigen(cov);
    let apart = |k: usize| { let (i,j) = ((k+1)%3,(k+2)%3); (values[k]-values[i]).abs().min((values[k]-values[j]).abs()) };
    let k = (0..3).max_by(|&a,&b| apart(a).total_cmp(&apart(b))).unwrap();
    let (i,j) = ((k+1)%3,(k+2)%3);
    (centre,unit(vectors[k]),vol,(values[i]-values[j]).abs()/values[i].abs().max(values[j].abs()))
}

// --- exact distances between triangles (Ericson, Real-Time Collision Detection, 5.1) ---

/// The nearest point of a triangle to `p`.
fn closest_on_triangle(p: V,[a,b,c]: Tri) -> V { gcs_core::space::closest_on_triangle(p,a,b,c).0 }

/// The closest points of two segments.
fn closest_segments(p1: V,q1: V,p2: V,q2: V) -> (V,V) {
    let (d1,d2,r) = (sub(q1,p1),sub(q2,p2),sub(p1,p2));
    let (a,e,f) = (dot(d1,d1),dot(d2,d2),dot(d2,r));
    let eps = 1e-30;
    let (s,t);
    if a <= eps && e <= eps { return (p1,p2); }
    if a <= eps { s = 0.; t = (f/e).clamp(0.,1.); }
    else {
        let c = dot(d1,r);
        if e <= eps { t = 0.; s = (-c/a).clamp(0.,1.); }
        else {
            let b = dot(d1,d2);
            let denom = a*e-b*b;
            let s0 = if denom > 0. { ((b*f-c*e)/denom).clamp(0.,1.) } else { 0. };
            let t0 = (b*s0+f)/e;
            if t0 < 0. { t = 0.; s = (-c/a).clamp(0.,1.); }
            else if t0 > 1. { t = 1.; s = ((b-c)/a).clamp(0.,1.); }
            else { t = t0; s = s0; }
        }
    }
    (add(p1,scale(d1,s)),add(p2,scale(d2,t)))
}

/// Whether the segment pierces the triangle.
fn pierces(p: V,q: V,[a,b,c]: Tri) -> bool {
    let (e1,e2,d) = (sub(b,a),sub(c,a),sub(q,p));
    let h = cross(d,e2);
    let det = dot(e1,h);
    if det.abs() < 1e-30 { return false; }
    let inv = 1./det;
    let s = sub(p,a);
    let u = inv*dot(s,h);
    if !(0. ..=1.).contains(&u) { return false; }
    let qv = cross(s,e1);
    let v = inv*dot(d,qv);
    if v < 0. || u+v > 1. { return false; }
    (0. ..=1.).contains(&(inv*dot(e2,qv)))
}

/// The least distance between two triangles and where it is on each.
fn triangle_distance(x: Tri,y: Tri) -> (f64,V,V) {
    for i in 0..3 {
        if pierces(x[i],x[(i+1)%3],y) || pierces(y[i],y[(i+1)%3],x) {
            let m = scale(add(add(x[0],x[1]),x[2]),1./3.);
            return (0.,m,m);
        }
    }
    let mut best = (f64::INFINITY,[0.;3],[0.;3]);
    let mut take = |p: V,q: V| { let d = norm(sub(p,q)); if d < best.0 { best = (d,p,q); } };
    for i in 0..3 { for j in 0..3 {
        let (p,q) = closest_segments(x[i],x[(i+1)%3],y[j],y[(j+1)%3]);
        take(p,q);
    }}
    for i in 0..3 {
        take(x[i],closest_on_triangle(x[i],y));
        let q = closest_on_triangle(y[i],x);
        take(q,y[i]);
    }
    best
}

// --- bounding volumes ---

struct Node { lo: V, hi: V, centre: V, radius: f64, first: u32, count: u32, kids: [u32;2] }

/// A hierarchy of boxes over triangles (each node also bounded by a sphere, which a turn carries
/// as it is), leaves of up to four.
struct Bvh { nodes: Vec<Node>, tris: Vec<Tri> }

impl Bvh {
    fn new(mut tris: Vec<Tri>) -> Self {
        fn build(nodes: &mut Vec<Node>,tris: &mut [Tri],first: usize) -> u32 {
            let (mut lo,mut hi) = ([f64::INFINITY;3],[f64::NEG_INFINITY;3]);
            for t in tris.iter() { for p in t { for k in 0..3 { lo[k] = lo[k].min(p[k]); hi[k] = hi[k].max(p[k]); } } }
            let centre = scale(add(lo,hi),0.5);
            let radius = tris.iter().flatten().map(|p| norm(sub(*p,centre))).fold(0.,f64::max);
            let at = nodes.len();
            nodes.push(Node { lo, hi, centre, radius, first:first as u32, count:tris.len() as u32, kids:[0;2] });
            if tris.len() <= 4 { return at as u32; }
            let k = (0..3).max_by(|&a,&b| (hi[a]-lo[a]).total_cmp(&(hi[b]-lo[b]))).unwrap();
            let mid = tris.len()/2;
            let key = |t: &Tri| t[0][k]+t[1][k]+t[2][k];
            tris.select_nth_unstable_by(mid,|a,b| key(a).total_cmp(&key(b)));
            let (l,r) = tris.split_at_mut(mid);
            let kids = [build(nodes,l,first),build(nodes,r,first+mid)];
            nodes[at].kids = kids;
            nodes[at].count = 0;
            at as u32
        }
        let mut nodes = Vec::with_capacity(tris.len()/2);
        build(&mut nodes,&mut tris,0);
        Bvh { nodes, tris }
    }
}

fn box_distance(p: V,lo: V,hi: V) -> f64 {
    let d: V = std::array::from_fn(|k| (lo[k]-p[k]).max(0.).max(p[k]-hi[k]));
    norm(d)
}

/// The least distance between the moving mesh `a` (at `pose`) and the still mesh `b`, and where it
/// is on each; or, `within` a distance, every triangle of `a` that comes that near `b`.
fn search(a: &Bvh,b: &Bvh,pose: &Pose,within: Option<f64>) -> (f64,V,V,Vec<u32>) {
    let bound = |i: u32,j: u32| {
        let (x,y) = (&a.nodes[i as usize],&b.nodes[j as usize]);
        (box_distance(pose.of(x.centre),y.lo,y.hi)-x.radius).max(0.)
    };
    let mut best = (within.unwrap_or(f64::INFINITY),[0.;3],[0.;3]);
    let mut near = Vec::new();
    let mut stack = vec![(0u32,0u32,bound(0,0))];
    while let Some((i,j,lb)) = stack.pop() {
        if lb > best.0 { continue; }
        let (x,y) = (&a.nodes[i as usize],&b.nodes[j as usize]);
        if x.count > 0 && y.count > 0 {
            for ta in x.first..x.first+x.count {
                let t = a.tris[ta as usize].map(|p| pose.of(p));
                let (lo,hi) = t.iter().fold(([f64::INFINITY;3],[f64::NEG_INFINITY;3]),|(lo,hi),p|
                    (std::array::from_fn(|k| lo[k].min(p[k])),std::array::from_fn(|k| hi[k].max(p[k]))));
                for u in &b.tris[y.first as usize..(y.first+y.count) as usize] {
                    let gap: V = std::array::from_fn(|k| {
                        let (l,h) = u.iter().fold((f64::INFINITY,f64::NEG_INFINITY),|(l,h),p| (l.min(p[k]),h.max(p[k])));
                        (l-hi[k]).max(0.).max(lo[k]-h)
                    });
                    if norm(gap) > best.0 { continue; }
                    let (d,p,q) = triangle_distance(t,*u);
                    match within {
                        Some(eps) => if d <= eps { near.push(ta); break; },
                        None => if d < best.0 { best = (d,p,q); },
                    }
                }
            }
            continue;
        }
        let split_a = y.count > 0 || (x.count == 0 && x.radius >= norm(sub(y.hi,y.lo))/2.);
        let pairs = if split_a { x.kids.map(|k| (k,j)) } else { y.kids.map(|k| (i,k)) };
        let mut kids = pairs.map(|(p,q)| (p,q,bound(p,q)));
        if kids[0].2 < kids[1].2 { kids.swap(0,1); }
        for k in kids { if k.2 <= best.0 { stack.push(k); } }
    }
    (best.0,best.1,best.2,near)
}

// --- the pair ---

/// The pinion turns this many times as far as the gear: the tooth counts, 48 over 24.
const RATIO: f64 = 2.;
/// One gear pitch, degrees.
const GEAR_PITCH: f64 = 360./48.;
/// Poses a pitch is swept at (intervals).
const STEPS: usize = 30;
/// A film of marking compound, mm: a quarter of a thousandth of an inch.
const FILM: f64 = 0.00635;
/// How much of its turn's direction a pinion triangle's normal must face along to be a flank's.
const FACING: f64 = 0.25;

/// Where the exported files are: `SOLVENT_PAIR_DIR`, or `build/exports`.
fn exports() -> std::path::PathBuf {
    std::env::var_os("SOLVENT_PAIR_DIR").map(Into::into).unwrap_or_else(||
        std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../build/exports"))
}

fn read_stl(path: &std::path::Path) -> Vec<Tri> {
    let bytes = std::fs::read(path).unwrap_or_else(|e| panic!("{} := {e} (export the pair first)",path.display()));
    let (v,t) = gcs_core::solid::agreement::stl_triangles(&bytes,1.).unwrap();
    t.iter().map(|t| t.map(|i| v[i as usize])).collect()
}

/// `f` over `items` on every core, in order.
fn each<T: Sync,R: Send>(items: &[T],f: impl Fn(&T) -> R + Sync) -> Vec<R> {
    let next = std::sync::atomic::AtomicUsize::new(0);
    let cores = std::thread::available_parallelism().map_or(4,|n| n.get());
    let mut out: Vec<(usize,R)> = std::thread::scope(|s| {
        let workers: Vec<_> = (0..cores).map(|_| s.spawn(|| {
            let mut mine = Vec::new();
            loop {
                let i = next.fetch_add(1,std::sync::atomic::Ordering::Relaxed);
                if i >= items.len() { break mine; }
                mine.push((i,f(&items[i])));
            }
        })).collect();
        workers.into_iter().flat_map(|w| w.join().unwrap()).collect()
    });
    out.sort_by_key(|(i,_)| *i);
    out.into_iter().map(|(_,r)| r).collect()
}

/// The least clearance at one pose on one side: the distance, where it is on the pinion (its
/// distance from the pinion's apex and from its axis), and how fast the pinion's own turn closes
/// it, millimetres a radian (the lever).
#[derive(Clone,Copy)]
struct Gap { d: f64, cone: f64, radius: f64, lever: f64 }

struct Pair { pinion: [Bvh;2], whole: Bvh, gear: Bvh, cp: V, ap: V, cg: V, ag: V, apex: V, sense: f64 }

impl Pair {
    /// The pinion placed in the gear's frame with the gear turned `phi` and the pinion the ratio
    /// as far in the mesh's sense, plus `extra` (radians) of its own.
    fn pose(&self,phi: f64,extra: f64) -> Pose {
        Pose::turn(self.cp,self.ap,self.sense*RATIO*phi+extra).then(Pose::turn(self.cg,self.ag,-phi))
    }
    fn gap(&self,side: usize,phi: f64,extra: f64) -> Gap {
        let pose = self.pose(phi,extra);
        let (d,p,q,_) = search(&self.pinion[side],&self.gear,&pose,None);
        // the pinion's own turn moves `p` along `ap x (p - cp)` (in the gear's frame, the posed
        // axis); the clearance closes at the rate that motion has along `q - p`
        let (c,a) = (pose.of(self.cp),apply(&pose.m,self.ap));
        let lever = if d > 0. { dot(cross(a,sub(p,c)),scale(sub(q,p),1./d)) } else { f64::NAN };
        Gap { d, cone:norm(sub(p,pose.of(self.apex))), radius:norm(cross(sub(p,c),a)), lever }
    }
    /// Both sides at every pose of one gear pitch.
    fn sweep(&self,extra: f64) -> Vec<[Gap;2]> {
        let poses: Vec<(usize,f64)> = (0..=STEPS).flat_map(|k| [0,1].map(|s| (s,(k as f64*GEAR_PITCH/STEPS as f64).to_radians()))).collect();
        let gaps = each(&poses,|&(s,phi)| self.gap(s,phi,extra));
        gaps.chunks(2).map(|g| [g[0],g[1]]).collect()
    }
    /// The pinion's triangles on `side` within `film` of the gear at some pose of the pitch.
    fn pattern(&self,side: usize,extra: f64,film: f64) -> Vec<u32> {
        let poses: Vec<f64> = (0..=STEPS).map(|k| (k as f64*GEAR_PITCH/STEPS as f64).to_radians()).collect();
        let mut all: Vec<u32> = each(&poses,|&phi| search(&self.pinion[side],&self.gear,&self.pose(phi,extra),Some(film)).3)
            .into_iter().flatten().collect();
        all.sort();
        all.dedup();
        all
    }
}

fn area([a,b,c]: Tri) -> f64 { norm(cross(sub(b,a),sub(c,a)))/2. }

/// Two tetrahedra meshed as shells, apart by a known gap; the search reads it, by corner, edge and
/// face, and a turn carries a mesh as the pose says.
#[test]
fn the_pair_search_reads_a_known_clearance() {
    let tet = |o: V,s: f64| -> Vec<Tri> {
        let [a,b,c,d] = [o,add(o,[s,0.,0.]),add(o,[0.,s,0.]),add(o,[0.,0.,s])];
        vec![[a,c,b],[a,b,d],[a,d,c],[b,c,d]]
    };
    // corner of one 0.05 below the other's floor
    let a = Bvh::new(tet([0.,0.,0.],1.));
    let b = Bvh::new(tet([0.2,0.2,-1.05],1.).into_iter().map(|t| t.map(|p| [p[0],p[1],p[2]])).collect());
    let (d,_,_,_) = search(&a,&b,&Pose::identity(),None);
    // b's apex (0.2, 0.2, -0.05) against a's floor z = 0
    assert!((d-0.05).abs() < 1e-12,"{d}");
    // turned half a turn about the z axis through (0.1, 0.1, 0): the apex still stands under the floor
    let d2 = search(&a,&b,&Pose::turn([0.1,0.1,0.],[0.,0.,1.],std::f64::consts::PI),None).0;
    assert!((d2-0.05).abs() < 1e-12,"{d2}");
    // edge against edge: two crossed segments' triangles 0.3 apart
    let x = [[0.,-1.,0.],[0.,1.,0.],[-0.001,0.,-1.]];
    let y = [[-1.,0.,0.3],[1.,0.,0.3],[0.,0.001,1.]];
    assert!((triangle_distance(x,y).0-0.3).abs() < 1e-12);
    // piercing reads zero, and the film finds a's triangles near b
    let c = Bvh::new(tet([0.2,0.2,-0.5],1.));
    assert_eq!(search(&a,&c,&Pose::identity(),None).0,0.);
    let near = search(&a,&b,&Pose::identity(),Some(0.051)).3;
    assert!(!near.is_empty());
    assert!(search(&a,&b,&Pose::identity(),Some(0.049)).3.is_empty());
    // the inertia axis of a long box, and its moments equal about it
    let bx: Vec<Tri> = {
        let (w,h) = (1.,5.);
        let p = |x: f64,y: f64,z: f64| [x*w,y*w,z*h];
        let q = [p(0.,0.,0.),p(1.,0.,0.),p(1.,1.,0.),p(0.,1.,0.),p(0.,0.,1.),p(1.,0.,1.),p(1.,1.,1.),p(0.,1.,1.)];
        [[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],[1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]]
            .iter().map(|f: &[usize;3]| f.map(|i| q[i])).collect()
    };
    let (centre,dir,vol,asym) = axis(&bx);
    assert!((vol-5.).abs() < 1e-12 && norm(sub(centre,[0.5,0.5,2.5])) < 1e-12);
    assert!((dir[2].abs()-1.).abs() < 1e-12 && asym < 1e-12,"{dir:?} {asym}");
}

#[test]
#[ignore = "a tool: export both members (examples/spiral_bevel/README.md, Making the pair), then \
    cargo test --manifest-path rust/Cargo.toml -p gcs-cli --test pair_check -- --ignored --nocapture"]
fn the_exported_pair_meshes_with_its_backlash() {
    let started = std::time::Instant::now();
    let dir = exports();
    let (pinion,gear) = (read_stl(&dir.join("hypoid-pinion.stl")),read_stl(&dir.join("hypoid-gear.stl")));
    eprintln!("read {} and {} triangles from {} ({:.1} s)",pinion.len(),gear.len(),dir.display(),started.elapsed().as_secs_f64());

    // The shafts, from the solids, against the model's axes.
    let (cp,ap,vp,sp) = axis(&pinion);
    let (cg,ag,vg,sg) = axis(&gear);
    let n = cross(ap,ag);
    let shaft = dot(ap,ag).abs().acos().to_degrees();
    let offset = dot(sub(cg,cp),n).abs()/norm(n);
    println!("pinion: {vp:.3} mm³, centroid ({:.4}, {:.4}, {:.4}), transverse moments equal within {sp:.1e}",cp[0],cp[1],cp[2]);
    println!("gear:   {vg:.3} mm³, centroid ({:.4}, {:.4}, {:.4}), transverse moments equal within {sg:.1e}",cg[0],cg[1],cg[2]);
    println!("shaft angle {shaft:.5}°, offset {offset:.5} mm (the configuration: 90°, 25 mm)");
    let e = fixtures::gear::read_as_configured();
    let line = |name: &str| {
        let l = &e.sketch.lines[e.map.ent_named(name).unwrap_or_else(|| panic!("no `{name}`")).i()];
        (e.sketch.world_point(l.p1 as usize),e.sketch.world_point(l.p2 as usize))
    };
    for (member,c,a) in [("pinion",cp,ap),("gear",cg,ag)] {
        let (o,t) = line(&format!("pair.reference.{member}.ax"));
        let m = unit(sub(t,o));
        println!("{member}'s axis {:.2e}° off the model's, its centroid {:.2e} mm off it",
            dot(a,m).abs().min(1.).acos().to_degrees(),norm(cross(sub(c,o),m)));
    }
    assert!((shaft-90.).abs() < 0.01 && (offset-25.).abs() < 0.01,"shaft angle {shaft}, offset {offset}");

    // The pinion's flanks by which way they face its turn: a triangle whose normal has at least
    // FACING of the direction its turn moves it along (or against) it. The blank's faces (the
    // cones, the spheres and the end relief's bands) are turned faces, square to that direction,
    // and are no flank.
    let mut sides = [Vec::new(),Vec::new()];
    let mut blank = 0;
    for &t in &pinion {
        let m = scale(add(add(t[0],t[1]),t[2]),1./3.);
        let Some(n) = gcs_core::space::triangle_normal(t[0],t[1],t[2]) else { continue };
        let facing = dot(n,unit(cross(ap,sub(m,cp))));
        if facing.abs() < FACING { blank += 1; continue; }
        sides[usize::from(facing < 0.)].push(t);
    }
    eprintln!("flanks: {} and {} triangles, {blank} others",sides[0].len(),sides[1].len());
    let names = ["facing the pinion's turn","facing against it"];
    let [plus,minus] = sides.map(Bvh::new);
    let (apex,mean) = line("pair.reference.pinion.pitch_line");
    let mut pair = Pair { pinion:[plus,minus], whole:Bvh::new(pinion), gear:Bvh::new(gear), cp, ap, cg, ag, apex, sense:1. };
    eprintln!("bounding volumes built ({:.1} s)",started.elapsed().as_secs_f64());

    // The mesh's sense: turned a little both ways, the wrong one closes at once.
    let trial = |pair: &Pair| (0..2).map(|s| pair.gap(s,0.5f64.to_radians(),0.).d).fold(f64::INFINITY,f64::min);
    let forward = trial(&pair);
    pair.sense = -1.;
    let backward = trial(&pair);
    pair.sense = if forward > backward { 1. } else { -1. };
    println!("the gear turned 0.5° and the pinion 1° with it: least flank clearance {forward:.4} mm one way, \
        {backward:.4} mm the other, which is the mesh's sense");

    // The pair as laid out, through one gear pitch.
    let report = |label: &str,sweep: &[[Gap;2]]| {
        println!("{label}:");
        for (k,g) in sweep.iter().enumerate() {
            if k % 3 == 0 || k == STEPS {
                println!("  gear {:5.2}°  pinion {:6.2}°  clearance {:.4} / {:.4} mm, sum {:.4}",
                    k as f64*GEAR_PITCH/STEPS as f64,pair.sense*RATIO*k as f64*GEAR_PITCH/STEPS as f64,g[0].d,g[1].d,g[0].d+g[1].d);
            }
        }
        let nearest = |s: usize| *sweep.iter().map(|g| &g[s]).min_by(|a,b| a.d.total_cmp(&b.d)).unwrap();
        let most = |s: usize| sweep.iter().map(|g| g[s].d).fold(0.,f64::max);
        for s in 0..2 {
            let g = nearest(s);
            println!("  {}: least {:.4} mm ({:.2} mm from the pinion's apex, radius {:.2} mm, lever {:.2} mm), most {:.4} mm",
                names[s],g.d,g.cone,g.radius,g.lever,most(s));
        }
        [nearest(0).d,nearest(1).d]
    };
    let nominal = pair.sweep(0.);
    let least = report("as laid out (each flank pair half the backlash apart by design)",&nominal);
    // no overlap: every triangle of each, at every pose of the pitch
    let poses: Vec<f64> = (0..=STEPS).map(|k| (k as f64*GEAR_PITCH/STEPS as f64).to_radians()).collect();
    let whole = each(&poses,|&phi| search(&pair.whole,&pair.gear,&pair.pose(phi,0.),None).0);
    let apart = whole.iter().copied().fold(f64::INFINITY,f64::min);
    println!("  the members' least distance over the pitch, every triangle of each: {apart:.4} mm");
    eprintln!("({:.1} s)",started.elapsed().as_secs_f64());
    assert!(apart > 0. && least.iter().all(|&d| (d-0.025).abs() < 0.01),"overlap {apart}, flank clearances {least:?}");

    // Each flank pair closed in turn: the pinion turned alone (the gear held) by the least of
    // clearance over lever across the pitch, which first brings that side into touch.
    for side in 0..2 {
        let sign = if side == 0 { 1. } else { -1. };
        let turn = nominal.iter().map(|g| g[side].d/g[side].lever.abs()).fold(f64::INFINITY,f64::min)*sign;
        let closed = pair.sweep(turn);
        let label = format!("the pinion turned {:.5}° alone, closing the flanks {}",turn.to_degrees(),names[side]);
        let [touch,open] = { let l = report(&label,&closed); [l[side],l[1-side]] };
        println!("  normal backlash (the other flanks' least clearance with these touching): {open:.4} mm; \
            the touching flanks {touch:.4} mm apart at their nearest");
        let pattern = pair.pattern(side,turn,FILM);
        let tris: Vec<Tri> = pattern.iter().map(|&i| pair.pinion[side].tris[i as usize]).collect();
        let cone = |p: V| norm(sub(p,apex));
        let along = |t: &Tri| cone(scale(add(add(t[0],t[1]),t[2]),1./3.));
        let radius = |t: &Tri| { let m = scale(add(add(t[0],t[1]),t[2]),1./3.); norm(cross(sub(m,cp),ap)) };
        let span = |f: &dyn Fn(&Tri) -> f64| tris.iter().map(|t| f(t)).fold((f64::INFINITY,f64::NEG_INFINITY),|(l,h),x| (l.min(x),h.max(x)));
        let (a0,a1) = span(&along);
        let (r0,r1) = span(&radius);
        println!("  contact pattern (pinion within {FILM} mm of the gear over the pitch): {} triangles, {:.2} mm², \
            {:.2} to {:.2} mm from the pinion's apex (mean point {:.2}), radius {:.2} to {:.2} mm",
            tris.len(),tris.iter().map(|t| area(*t)).sum::<f64>(),a0,a1,cone(mean),r0,r1);
        eprintln!("({:.1} s)",started.elapsed().as_secs_f64());
        assert!(touch < 0.002 && (open-0.05).abs() < 0.01,"flanks {} touching at {touch}, backlash {open}",names[side]);
    }
}
