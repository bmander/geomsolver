//! Closed-form baselines only; these do not yet adapt Solvent's swept-field evaluator.
use fidget::context::Tree;

type V = [f64;3];
fn add(a:V,b:V) -> V { std::array::from_fn(|k| a[k]+b[k]) }
fn sub(a:V,b:V) -> V { std::array::from_fn(|k| a[k]-b[k]) }
fn mul(a:V,s:f64) -> V { a.map(|x| x*s) }
fn dot(a:V,b:V) -> f64 { (0..3).map(|k| a[k]*b[k]).sum() }
fn cross(a:V,b:V) -> V { [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]] }
fn unit(p:V) -> V { mul(p,1./dot(p,p).sqrt()) }
fn rotate(p:V,angle:f64) -> V {
    let axis = unit([1.,2.,3.]); let (s,c) = angle.sin_cos();
    add(add(mul(p,c),mul(cross(axis,p),s)),mul(axis,dot(axis,p)*(1.-c)))
}
fn linear(p:[Tree;3],n:V) -> Tree {
    p[0].clone()*n[0]+p[1].clone()*n[1]+p[2].clone()*n[2]
}
fn axes() -> [Tree;3] { [Tree::x(),Tree::y(),Tree::z()] }
fn ball(center:V,radius:f64) -> Tree {
    let [x,y,z] = std::array::from_fn(|k| axes()[k].clone()-center[k]);
    (x.square()+y.square()+z.square()).sqrt()-radius
}
fn cube(angle:f64,half:V) -> Tree {
    let basis = [[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]].map(|n| rotate(n,angle));
    (0..3).map(|k| linear(axes(),basis[k]).abs()-half[k]).reduce(|a,b| a.max(b)).unwrap()
}
fn tetrahedron(angle:f64) -> Tree {
    let r = 0.06;
    let p = [[0.,0.,1.5],[r,0.,-0.5],[-r*0.5,r*3_f64.sqrt()*0.5,-0.5],
        [-r*0.5,-r*3_f64.sqrt()*0.5,-0.5]].map(|p| rotate(p,angle));
    let center = mul(p.into_iter().fold([0.;3],add),0.25);
    [[0,1,2],[0,2,3],[0,3,1],[1,3,2]].into_iter().map(|t| {
        let [a,b,c] = t.map(|i| p[i]);
        let mut n = unit(cross(sub(b,a),sub(c,a)));
        if dot(n,sub(center,a)) > 0. { n = mul(n,-1.); }
        linear(axes(),n)-dot(n,a)
    }).reduce(|a,b| a.max(b)).unwrap()
}

pub const NAMES:&[&str] = &["sphere","cube","rotated_cube","torus","spiky_tetrahedron",
    "rotated_tetrahedron","thin_plate","disconnected","zero_only"];

pub fn build(name:&str) -> (Tree,f32) {
    match name {
        "sphere" => (ball([0.;3],1.),1.25),
        "cube" => (cube(0.,[1.;3]),1.25),
        "rotated_cube" => (cube(0.47,[1.;3]),1.75),
        "torus" => {
            let radial = (Tree::x().square()+Tree::y().square()).sqrt()-2.;
            ((radial.square()+Tree::z().square()).sqrt()-0.6,3.)
        }
        "spiky_tetrahedron" => (tetrahedron(0.),1.75),
        "rotated_tetrahedron" => (tetrahedron(0.47),1.75),
        "thin_plate" => (cube(0.47,[0.02,0.7,0.7]),1.25),
        "disconnected" => (ball([-0.4,0.,0.],0.3).min(ball([0.53,0.11,0.07],0.035)),1.),
        "zero_only" => { let a = ball([0.;3],1.); (a.max(-a.clone()),1.25) }
        _ => panic!("unknown fixture {name}"),
    }
}
