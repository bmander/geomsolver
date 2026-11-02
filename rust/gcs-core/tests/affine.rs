//! **Affine maps of solids** (`Brep::mapped`, `affine`): every kind of face the kernel builds —
//! planes, cylinders, cones, spheres, tori, revolutions and extrusions of splines, a blend screwed
//! about an axis (a canal) — under a shear, a stretch along skew axes, a map that
//! reverses space, and a similarity. Each image is a valid closed boundary whose volume is the
//! original's times `|det M|`; mapped back it is the original again; its affine faces' STEP nets
//! are their surfaces at the nets' parameters, and the file reads back face for face; it meshes
//! closed; and a Boolean of the image is the image of the Boolean of the preimages — the map and the
//! Boolean commute. Face families' walls are held likewise in `face_family.rs`.
use gcs_core::brep::affine::Affine;
use gcs_core::brep::boolean::{combined,Op};
use gcs_core::brep::build::{canal,cone,prism,revolve,sphere,square,torus,Profile,ProfileEdge};
use gcs_core::brep::geom::{tolerance,Curve,Frame,Rigid,Surface,V};
use gcs_core::brep::nurbs::BSpline;
use gcs_core::brep::props::volume;
use gcs_core::brep::topo::{Brep,EdgeCurve};
use gcs_core::rng::Rng;
use std::f64::consts::{FRAC_1_SQRT_2,PI,TAU};
use std::sync::Arc;

fn close(a: f64,b: f64,tol: f64,what: &str) { assert!((a-b).abs() <= tol*b.abs().max(1.),"{what}: {a} against {b} ({:e})",(a-b)/b.abs().max(1.)); }
fn dist(a: V,b: V) -> f64 { ((a[0]-b[0]).powi(2)+(a[1]-b[1]).powi(2)+(a[2]-b[2]).powi(2)).sqrt() }

const Z: V = [0.,0.,1.];

/// A solid of every kind of face: its name and its B-rep.
fn solids() -> Vec<(&'static str,Brep)> {
    let tilted = [0.2,-0.3,1.];
    // a half ellipse from pole to pole, two rational quarters, turned once: a revolution
    let (south,north): (V,V) = ([0.,0.,-1.],[0.,0.,1.]);
    let arc = BSpline::rational(2,vec![0.,0.,0.,0.5,0.5,1.,1.,1.],vec![south,[2.,0.,-1.],[2.,0.,0.],[2.,0.,1.],north],
        vec![1.,FRAC_1_SQRT_2,1.,FRAC_1_SQRT_2,1.]).unwrap();
    let half = Profile {origin:south,normal:[0.,-1.,0.],names:vec![],
        loops:vec![vec![ProfileEdge::Spline(Arc::new(arc)),ProfileEdge::Line {a:north,b:south}]]};
    // a cubic bump closed by a line: an extrusion
    let bump = BSpline::new(3,vec![0.,0.,0.,0.,1.,1.,1.,1.],vec![[2.,0.,0.],[2.,1.5,0.],[0.,1.5,0.],[0.,0.,0.]]).unwrap();
    let lobe = Profile {origin:[0.;3],normal:Z,names:vec![],
        loops:vec![vec![ProfileEdge::Line {a:[0.,0.,0.],b:[2.,0.,0.]},ProfileEdge::Spline(Arc::new(bump))]]};
    vec![
        ("box",prism(&square([0.3,0.2,0.1],Z,1.),0.,1.5).unwrap()),
        ("cylinder",cone([0.,0.,0.],tilted,1.2,1.2,3.).unwrap()),
        ("frustum",cone([0.,0.,0.],tilted,1.5,0.6,2.).unwrap()),
        ("cone",cone([0.,0.,0.],tilted,1.5,0.,2.).unwrap()),
        ("sphere",sphere([0.5,0.,0.],tilted,1.3).unwrap()),
        ("torus",torus([0.,0.,0.],tilted,3.,1.).unwrap()),
        ("spheroid",revolve(&half,[0.;3],Z,TAU).unwrap()),
        ("lobe",prism(&lobe,0.,1.).unwrap()),
        ("canal",canal([3.,0.,0.],0.5,[0.;3],Z,1.5,2.,"canal").unwrap()),
    ]
}

/// A map of each sort: a shear, a stretch along skew axes, one reversing space, a similarity.
fn maps() -> Vec<(&'static str,Affine)> {
    let turn = Rigid::turn([0.3,0.,0.],[1.,2.,0.5],0.8);
    vec![
        ("shear",Affine::new([[1.,0.4,0.2],[0.,1.,-0.3],[0.1,0.,1.]],[1.,-2.,0.5]).unwrap()),
        ("stretch",Affine::stretch(&Frame::new([0.5,0.,0.],[0.3,0.4,1.],[1.,-0.2,0.]),[2.,0.6,1.4]).unwrap()),
        ("reversing",Affine::new([[-1.2,0.3,0.],[0.,0.9,0.2],[0.,0.,1.1]],[0.,1.,0.]).unwrap()),
        ("similarity",Affine::new(turn.r.map(|row| row.map(|x| 2.*x)),[1.,1.,-1.]).unwrap()),
    ]
}

/// The map's algebra: its inverse undoes it, composition is its product, a normal goes where a
/// plane's does (`M⁻ᵀ`), its stretches are the singular values, and a similarity is recognised.
#[test]
fn an_affine_maps_algebra_holds() {
    let mut rng = Rng::new(17);
    for (name,a) in maps() {
        let back = a.inverse();
        for _ in 0..20 {
            let p: V = std::array::from_fn(|_| rng.uniform(-3.,3.));
            let v: V = std::array::from_fn(|_| rng.uniform(-1.,1.));
            assert!(dist(back.point(a.point(p)),p) <= 1e-13*(1.+dist(p,[0.;3])),"{name}");
            assert!(dist(a.then(&back).point(p),p) <= 1e-13*(1.+dist(p,[0.;3])),"{name}");
            // a normal stays square to every direction the plane holds
            let n: V = std::array::from_fn(|_| rng.uniform(-1.,1.));
            let w = [n[1]*v[2]-n[2]*v[1],n[2]*v[0]-n[0]*v[2],n[0]*v[1]-n[1]*v[0]];
            let (nn,ww) = (a.normal(n),a.vector(w));
            assert!((nn[0]*ww[0]+nn[1]*ww[1]+nn[2]*ww[2]).abs() <= 1e-12,"{name}: a normal off its plane");
        }
        let s = a.stretches();
        let mtm: [V;3] = std::array::from_fn(|i| std::array::from_fn(|j| (0..3).map(|k| a.m[k][i]*a.m[k][j]).sum()));
        let det = mtm[0][0]*(mtm[1][1]*mtm[2][2]-mtm[1][2]*mtm[2][1])-mtm[0][1]*(mtm[1][0]*mtm[2][2]-mtm[1][2]*mtm[2][0])
            +mtm[0][2]*(mtm[1][0]*mtm[2][1]-mtm[1][1]*mtm[2][0]);
        close(s[0]*s[1]*s[2],det.sqrt(),1e-12,name);
        assert_eq!(a.similarity().is_some(),name == "similarity","{name}");
    }
    assert!(Affine::new([[1.,2.,3.],[2.,4.,6.],[0.,0.,1.]],[0.;3]).is_err(),"a singular map");
}

/// A curve's image is read at its own parameter run as `Curve::mapped` says: a line at the speed
/// the map gives it, a circle an ellipse about its principal axes from the turn it starts at, a
/// spline its poles mapped.
#[test]
fn a_curves_image_is_its_points_images() {
    let f = Frame::new([1.,0.,2.],[0.3,0.2,1.],[1.,0.,0.]);
    let curves = [Curve::Line {p:[1.,2.,3.],d:[0.6,0.8,0.]},Curve::Circle(f,1.5),Curve::Ellipse(f,2.,0.7),
        Curve::BSpline(Arc::new(BSpline::new(3,vec![0.,0.,0.,0.,1.,1.,1.,1.],vec![[0.;3],[1.,2.,0.],[2.,-1.,1.],[3.,0.,0.]]).unwrap()))];
    for (name,a) in maps() {
        for c in &curves {
            let (image,[al,be]) = c.mapped(&a);
            assert!(al > 0.,"{name}");
            if let Curve::Ellipse(_,ea,eb) = image { assert!(ea >= eb,"{name}: the longer axis first"); }
            for k in 0..=16 {
                let t = k as f64/16.*if c.period().is_some() { TAU } else { 1. };
                assert!(dist(image.point(al*t+be),a.point(c.point(t))) <= 1e-12,"{name}: a {} at {t}",c.kind());
            }
        }
    }
}

/// An elliptic cylinder's foot (a cylinder's image) is its nearest point: on the surface, the point
/// on its normal there, and no sample nearer.
#[test]
fn an_elliptic_cylinders_foot_is_its_nearest_point() {
    let a = &maps()[0].1;
    let s = Surface::Cylinder(Frame::new([0.;3],[0.2,0.1,1.],[1.,0.,0.]),1.5).mapped(a).0;
    let Surface::Affine(m) = &s else { panic!("a sheared cylinder is its image") };
    assert!(m.quadric.is_some());
    let mut rng = Rng::new(31);
    for _ in 0..100 {
        let q: V = std::array::from_fn(|_| rng.uniform(-4.,4.));
        let uv = s.inverse(q);
        let (x,su,sv) = s.d1(uv);
        let d = [q[0]-x[0],q[1]-x[1],q[2]-x[2]];
        let dot = |a: V,b: V| a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
        assert!(dot(d,su).abs() <= 1e-9*(1.+dist(q,x))*dot(su,su).sqrt(),"{q:?}: off the normal at its foot");
        assert!(dot(d,sv).abs() <= 1e-9*(1.+dist(q,x))*dot(sv,sv).sqrt(),"{q:?}: off the normal at its foot");
        // the nearest of a fine grid about the foot's height
        let found = (0..2000).flat_map(|i| (-40..=40).map(move |j| [TAU*i as f64/2000.,uv[1]+j as f64*0.01]))
            .map(|w| dist(s.point(w),q)).fold(f64::INFINITY,f64::min);
        assert!(dist(q,x) <= found+1e-9,"{q:?}: a sample {found} nearer than its foot {}",dist(q,x));
        close(s.implicit(q).abs(),dist(q,x),1e-9,"its distance");
    }
}

/// The volume a mesh of `m` holds.
fn mesh_volume(m: &gcs_core::brep::mesh::Mesh) -> f64 {
    m.tris.iter().map(|t| {
        let [a,b,c] = t.map(|i| m.pts[i as usize]);
        (a[0]*(b[1]*c[2]-b[2]*c[1])-a[1]*(b[0]*c[2]-b[2]*c[0])+a[2]*(b[0]*c[1]-b[1]*c[0]))/6.
    }).sum()
}

/// **Every kind of face under every sort of map**: the image valid, its volume `|det M|` times
/// the original's, mapped back the original; each affine face's net its surface at the net's
/// parameters along every use; the STEP file read back face for face; meshed closed, holding the
/// volume to its bar.
#[test]
fn every_kind_of_face_maps_to_its_image() {
    for (solid,b) in solids() {
        b.check(tolerance(b.size())).unwrap_or_else(|e| panic!("{solid}: {e}"));
        let whole = volume(&b);
        for (map,a) in maps() {
            let what = format!("{solid} under the {map}");
            let m = b.mapped(&a);
            m.check(tolerance(m.size())).unwrap_or_else(|e| panic!("{what}: {e}"));
            let v = volume(&m);
            close(v,a.det().abs()*whole,1e-9,&what);
            let back = m.mapped(&a.inverse());
            back.check(tolerance(back.size())).unwrap_or_else(|e| panic!("{what}, mapped back: {e}"));
            close(volume(&back),whole,1e-9,&format!("{what}, mapped back"));
            // each affine face's net is its surface where the face's uses run
            for (fi,f) in m.faces.iter().enumerate() {
                let Some(net) = gcs_core::brep::step::affine_net(&m,fi) else { continue };
                let net = net.unwrap_or_else(|e| panic!("{what}: {e}"));
                // a loft's or a family's net is fitted within the file's tolerance, then mapped; any
                // other's is exact
                let Surface::Affine(w) = &f.surface else { unreachable!() };
                let near = if matches!(w.of,Surface::Blend(..) | Surface::Family(..)) { 4.*gcs_core::brep::step::FIT*w.stretch[1] }
                    else { 1e-9*m.size() };
                for c in f.loops.iter().flatten() {
                    let e = &m.edges[c.edge as usize];
                    for k in 0..=8 {
                        let t = e.t[0]+(e.t[1]-e.t[0])*k as f64/8.;
                        let uv = c.pcurve.at(t,e,&f.surface,&m.vertices);
                        let ([s,r],_) = net.uv(uv);
                        assert!(dist(net.net.point(s,r),f.surface.point(uv)) <= near,"{what}: face {fi}'s net strays at {uv:?}");
                    }
                }
            }
            let text = gcs_core::brep::step::write(&m,&what,1e-4).unwrap_or_else(|e| panic!("{what}: {e}"));
            gcs_core::brep::step_check::verify(&text,&gcs_core::brep::step_check::Solid::of(&m)).unwrap_or_else(|e| panic!("{what}: {e}"));
            let bar = 2e-3*m.size();
            let mesh = gcs_core::brep::mesh::mesh(&m,bar,0.2).unwrap_or_else(|e| panic!("{what}: {e}"));
            gcs_core::mesh::stl_shells(&gcs_core::mesh::stl_of(&mesh.triangles(),&what)).unwrap_or_else(|e| panic!("{what}: {e}"));
            assert_eq!(mesh.turned,0,"{what}: triangles facing against their surfaces");
            close(mesh_volume(&mesh),v,24.*bar/v.abs().powf(1./3.),&format!("{what}, meshed"));
        }
    }
}

/// **The map and the Boolean commute**: an image cut by a block is the image of its preimage cut
/// by the block carried back — the one through the pulled-back closed forms and traces in space,
/// the other through the preimages' own kinds — the same volume, both valid.
#[test]
fn a_boolean_of_images_is_the_image_of_the_boolean() {
    for (solid,b) in solids() {
        for (map,a) in maps() {
            let what = format!("{solid} under the {map}");
            let m = b.mapped(&a);
            let (lo,hi) = m.bounds();
            // a block above a plane leaning across the image, off any pole or seam's plane
            let middle: V = std::array::from_fn(|k| 0.5*(lo[k]+hi[k]));
            let reach = 4.*dist(lo,hi);
            let n = [0.21,-0.13,1.];
            let block = prism(&square(middle,n,reach),0.,reach).unwrap();
            let pre = combined(&b,&block.mapped(&a.inverse()),Op::Common,0.).unwrap_or_else(|e| panic!("{what}, its preimage: {e}"));
            // a traced meeting is fitted to its trace's tolerance, which a cone's with a leaning plane
            // reaches to a hundred times the construction's own (the preimage's alike)
            pre.check(100.*tolerance(pre.size())).unwrap_or_else(|e| panic!("{what}, its preimage cut: {e}"));
            let image = combined(&m,&block,Op::Common,0.).unwrap_or_else(|e| panic!("{what}: {e}"));
            image.check(100.*tolerance(image.size())).unwrap_or_else(|e| panic!("{what}, cut: {e}"));
            close(volume(&image),a.det().abs()*volume(&pre),1e-7,&what);
        }
    }
}

/// Where a map carries a kind to its own it is written so: a similarity keeps a sphere a sphere and
/// a cone a cone, a plane is a plane, a line a line, a circle an ellipse (or a circle), a spline its
/// poles; only what has no image of its own kind is wrapped.
#[test]
fn a_kind_is_its_own_image_where_it_has_one() {
    let maps = maps();
    let shear = &maps[0].1;
    let similar = &maps[3].1;
    let ball = sphere([0.;3],Z,1.).unwrap();
    assert!(matches!(ball.mapped(similar).faces[0].surface,Surface::Sphere(_,r) if (r-2.).abs() <= 1e-12));
    assert!(matches!(&ball.mapped(shear).faces[0].surface,Surface::Affine(m) if m.quadric.is_some()));
    let c = cone([0.;3],Z,1.5,0.5,2.).unwrap();
    for f in &c.mapped(similar).faces { assert!(matches!(f.surface,Surface::Plane(_) | Surface::Cone(..)),"{}",f.surface.kind()); }
    let block = prism(&square([0.;3],Z,1.),0.,1.).unwrap().mapped(shear);
    for f in &block.faces { assert!(matches!(f.surface,Surface::Plane(_))); }
    for e in &block.edges { assert!(matches!(e.curve,EdgeCurve::Curve(Curve::Line {..}))); }
    for e in &c.mapped(shear).edges {
        assert!(matches!(e.curve,EdgeCurve::Curve(Curve::Ellipse(..) | Curve::Circle(..) | Curve::Line {..})),"an edge left wrapped");
    }
    // the identity is no map at all, and a rigid motion moves
    let turned = Affine::of_rigid(&Rigid::turn([0.;3],[1.,1.,0.],0.6));
    assert!(matches!(ball.mapped(&turned).faces[0].surface,Surface::Sphere(..)));
    assert!(matches!(ball.mapped(&Affine::IDENTITY).faces[0].surface,Surface::Sphere(..)));
    // and two maps of one ball are one ellipsoid where they agree
    let e = ball.mapped(shear);
    let again = ball.mapped(&shear.then(&Affine::IDENTITY));
    assert!(gcs_core::brep::ssi::same(&e.faces[0].surface,&again.faces[0].surface,1e-9));
    let _ = PI;
}
