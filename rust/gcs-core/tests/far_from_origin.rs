//! **A solid is the same wherever it stands** (#1002): moving every input by one step decides
//! nothing the kernel reads but the rounding of its coordinates. The issue's blade (a face family
//! along a line) united with its hub, built at the origin and refused a metre from it; every kind
//! of solid the kernel builds cut where it stands 10 to 100 000 times its size from the origin, to
//! the volume it is cut to at it; a point of a patch its own foot wherever the patch stands; and
//! every kind of surface enclosed as closely far from the origin as at it.
//!
//! Three readings measured a patch by where it stood (`geom::NEAR`, `enclose`'s `lift` and the
//! blend's point): how near its point a foot found from the last one is to be taken for the
//! nearest, as a part of the point's distance from the origin; a rational spline's quotient, each
//! term read from the origin; and a blend's point between its sections, alike.
use crate::brep::{nurbs_ball,total_area};
use crate::brep_export::exact_named;
use crate::common::read_solved;
use crate::enclosures::surfaces;
use gcs_core::brep::boolean::{combined,face_parameters,Op};
use gcs_core::brep::build::{prism,square};
use gcs_core::brep::enclose::Enclosed;
use gcs_core::brep::geom::{tolerance,Rigid,Surface,V};
use gcs_core::brep::props;
use gcs_core::brep::topo::Brep;
use gcs_core::interval::Iv;
use gcs_core::rng::Rng;
use gcs_core::space::{add,cross,distance,norm,normalised,scale};

/// The issue's document: a lens section on its mid-chord lofted along a line — chord, pitch and
/// thickness falling along the span — and the hub's cylinder it is united with, every point of
/// both moved by the one length `x0`.
const BLADE: &str = "\
unit mm
use std
use std.sets
use std.solids (union)
use std.stations
use std.place (coords)

hand := 1
component BladeSection(g: line, up: axis, s: Scalar) {
  st := std.stations.OnLine(g, up: up, s: s)
  c := 40mm * (1 - 0.6 * s)
  t := 6mm * (1 - 0.7 * s)
  beta := (70 - 40 * s) * 1deg
  cb := cos(beta)
  sb := sin(beta)
  in st.n {
    le := point
    le coords(du: 0.5 * c * cb, dv: 0.5 * c * sb) st.n
    u := point
    u coords(du: -0.5 * t * sb, dv: 0.5 * t * cb) st.n
    l := point
    l coords(du: 0.5 * t * sb, dv: -0.5 * t * cb) st.n
    tu := point
    tu coords(du: -0.5 * c * cb - 0.25mm * sb, dv: -0.5 * c * sb + 0.25mm * cb) st.n
    tl := point
    tl coords(du: -0.5 * c * cb + 0.25mm * sb, dv: -0.5 * c * sb - 0.25mm * cb) st.n
    skin := line(tu, u) -> line(u, le) -> line(le, l) -> line(l, tl)
    foil := spline(through: skin)
    te := line(tl, tu)
  }
  profile := face(foil, te)
}

x0 := 6318mm
root := point
fix((x0, 0mm, 24mm)) root
tip := point
fix((x0, 0mm, 135mm)) tip
stack := line(root, tip)
fwd := point
fix((x0 - 22mm, 0mm, 0mm)) fwd
aft := point
fix((x0 + 18mm, 0mm, 0mm)) aft
ax := line(fwd, aft)
barrel := solid(std.sets.Rod(ax, r: 24mm))
param s: Scalar over (-0.06, 1)
blade := solid({ p | some s: p in BladeSection(stack, std.x).profile at s })
part := solid(barrel)
blade union part
";

/// The solid the issue's document calls `name`, drawn `x0` millimetres along x, built exactly.
fn drawn(x0: f64,name: &str) -> Brep {
    let (e,sk) = read_solved(&BLADE.replace("x0 := 6318mm",&format!("x0 := {x0}mm")));
    exact_named(&e,&sk,name,&format!("drawn {x0} mm out"))
}

/// A step of `k` sizes, off every axis.
fn step(size: f64,k: f64) -> Rigid { Rigid {t:[0.9*k*size,0.3*k*size,-0.3*k*size],..Rigid::identity()} }

/// `part` is the solid `at_origin` is, somewhere else: valid, of as many faces and edges, and of
/// its volume within the tolerance's band over its boundary.
fn the_origins(part: &Brep,at_origin: &Brep,what: &str) {
    part.check(10.*tolerance(part.size())).unwrap_or_else(|e| panic!("{what}: {e}"));
    assert_eq!((part.faces.len(),part.edges.len()),(at_origin.faces.len(),at_origin.edges.len()),"{what}");
    let (v,whole,band) = (props::volume(part),props::volume(at_origin),tolerance(at_origin.size())*total_area(at_origin));
    assert!((v-whole).abs() <= band,"{what}: {v} against the origin's {whole} ({:e} past a band of {band:e})",(v-whole).abs());
}

/// **The issue's blade unites with its hub wherever it is drawn** (#1002): the document at the
/// origin and as the issue writes it, 6.318 m along x — where it was refused "a cylinder's pieces do
/// not close" — is one solid, valid, of seven faces and fifteen edges, its volume the origin's
/// within the tolerance's band over its boundary.
#[test]
fn the_issues_blade_unites_with_its_hub_wherever_it_is_drawn() {
    let at_origin = drawn(0.,"part");
    at_origin.check(10.*tolerance(at_origin.size())).unwrap_or_else(|e| panic!("drawn at the origin: {e}"));
    assert_eq!((at_origin.faces.len(),at_origin.edges.len()),(7,15));
    the_origins(&drawn(6318.,"part"),&at_origin,"drawn 6318 mm out");
}

/// **…and its two solids moved unite as they do at the origin** (#1002): the blade and the hub's
/// cylinder as the document builds them at the origin, both moved 1, 2 and 4 m along x — where the
/// issue's document was refused a different way at each ("a shell of the arrangement is kept on one
/// side of a cylinder and dropped on another", "a plane's pieces do not close", "a cylinder's pieces
/// do not close") — united by the kernel alone: the origin's solid each, face for face and to its
/// volume within the tolerance's band.
#[test]
fn the_issues_blade_and_its_hub_moved_unite_as_at_the_origin() {
    let (blade,barrel) = (drawn(0.,"blade"),drawn(0.,"barrel"));
    let united = |x: f64| {
        let out = Rigid {t:[x,0.,0.],..Rigid::identity()};
        combined(&barrel.moved(&out),&blade.moved(&out),Op::Union,0.).unwrap_or_else(|e| panic!("moved {x} mm: {e}"))
    };
    let at_origin = united(0.);
    at_origin.check(10.*tolerance(at_origin.size())).unwrap_or_else(|e| panic!("at the origin: {e}"));
    for x in [1000.,2000.,4000.] { the_origins(&united(x),&at_origin,&format!("moved {x} mm")); }
}

/// A solid of every kind of face the kernel builds, each with its name: `affine`'s (planes, a
/// cylinder, a cone and its frustum, a sphere, a torus, a rational spline turned, a spline
/// extruded, a canal's blend), a rational net closed onto its poles (`brep::nurbs_ball`), and the
/// issue's blade, a face family's walls.
fn solids() -> Vec<(&'static str,Brep)> {
    let mut out = crate::affine::solids();
    out.push(("a NURBS ball",nurbs_ball()));
    out.push(("the issue's blade",drawn(0.,"blade")));
    out
}

/// **A cut far from the origin is the cut at it** (#1002): every kind of solid, where it is and
/// moved 10, 1000 and 100 000 times its size off, cut by a block above a plane leaning through its
/// middle (off any pole or seam's plane) — a valid solid each, of the volume cut at the origin to a
/// part in ten million. The NURBS ball was refused ten sizes out ("a piece of a plane touches the
/// other solid's boundary … along no face of either": every ray placing the plane's piece against
/// it refused, its sheet's rates enclosed ten times too wide) after three minutes, where its cut at
/// the origin takes a tenth of a second; and the canal's cut took 700 times as long ten thousand
/// sizes out and was refused a hundred thousand out. The blade's walls are the issue's.
#[test]
fn a_cut_far_from_the_origin_is_the_cut_at_it() {
    for (name,b) in solids() {
        let size = b.size();
        let cut = |k: f64| -> f64 {
            let what = format!("{name}, {k} sizes out");
            let m = b.moved(&step(size,k));
            let (lo,hi) = m.bounds();
            let middle: V = std::array::from_fn(|i| 0.5*(lo[i]+hi[i]));
            let block = prism(&square(middle,[0.21,-0.13,1.],4.*size),0.,4.*size).unwrap();
            let part = combined(&m,&block,Op::Common,0.).unwrap_or_else(|e| panic!("{what}: {e}"));
            part.check(100.*tolerance(part.size())).unwrap_or_else(|e| panic!("{what}: {e}"));
            props::volume(&part)
        };
        let whole = cut(0.);
        // (the blade's cut is traced along both its walls, seconds a cut: once, a thousand out)
        let steps: &[f64] = if name.contains("blade") { &[1e3] } else { &[10.,1e3,1e5] };
        for &k in steps {
            let v = cut(k);
            assert!((v-whole).abs() <= 1e-7*whole,"{name}, {k} sizes out: {v} against {whole} at the origin");
        }
    }
}

/// **A point of a patch is its own foot wherever the patch stands** (#1002): on every face of
/// every solid that lies on a patch whose foot is searched for — a family's wall, a blend, a
/// B-spline sheet, a spline swept or turned — points over the parameters the face reaches, on the
/// patch and a millionth of its size off it along its normal, asked in no order (so each from the
/// last one's foot, a long way off), have their feet at themselves, and stand as far off the patch
/// as they were put: within a part in 10⁹ of the solid's size, and the part in 10¹² of the point's
/// coordinates a sheet's own Newton steps stop within (`geom::net_newton`, not yet a part of the
/// sheet's size). Where the patch stands, and a thousand sizes out — as far as that stop keeps
/// within the tolerance: a hundred thousand out a rational net's feet stand a hundred tolerances
/// off, and by its poles more. On the blade's wall a metre out the feet stood a millimetre off,
/// and 6.3 m out six.
#[test]
fn a_point_of_a_patch_is_its_own_foot_wherever_the_patch_stands() {
    for (name,b) in solids() {
        let size = b.size();
        for k in [0.,1e3] {
            let m = b.moved(&step(size,k));
            for (fi,f) in m.faces.iter().enumerate() {
                if !matches!(f.surface,Surface::Family(..) | Surface::Blend(..) | Surface::BSpline(..) | Surface::Extrusion(..) | Surface::Revolution(..)) { continue }
                let [u,v] = face_parameters(&m,fi);
                let mut rng = Rng::new(1002+fi as u32);
                for off in [0.,1e-6*size] {
                    for _ in 0..200 {
                        // (clear of the face's edges, where a pole or a seam has many feet)
                        let uv = [u[0]+(u[1]-u[0])*rng.uniform(0.05,0.95),v[0]+(v[1]-v[0])*rng.uniform(0.05,0.95)];
                        let (x,su,sv) = f.surface.d1(uv);
                        let Some(n) = normalised(cross(su,sv)) else { continue };
                        let p = add(x,scale(n,off));
                        let foot = f.surface.point(f.surface.inverse(p));
                        let what = format!("{name}, {k} sizes out, face {fi} ({}) at {uv:?}, {off:e} off",f.surface.kind());
                        let bar = 1e-9*size+1e-12*norm(p);
                        assert!(distance(foot,x) <= bar,"{what}: its foot {:e} from it",distance(foot,x));
                        assert!((f.surface.implicit(p).abs()-off).abs() <= bar,"{what}: read {:e} off",f.surface.implicit(p));
                    }
                }
            }
        }
    }
}

/// **A surface is enclosed as closely far from the origin as at it** (#1002): every kind of
/// surface (`enclosures`' own), where it is and moved 30 000 out, over the same random boxes of
/// its parameters — each sample of its point and partials inside its enclosure there too, and the
/// enclosure's point and rates no wider than at the origin but by the rounding of coordinates that
/// far out. A rational net's rates (`A' − P W'`, each term read from the origin) and a blend's point
/// (`(1 − v) A + v B`) were as wide as the distance times the box.
#[test]
fn a_surface_is_enclosed_as_closely_far_from_the_origin_as_at_it() {
    let far = [3e4,-1e4,2e4];
    let out = Rigid {t:far,..Rigid::identity()};
    // what rounding coordinates that far out may widen an enclosure by: a part in 10¹⁰ of them
    let rounding = 1e-10*norm(far);
    let mut rng = Rng::new(1002);
    for (what,s,[du,dv]) in surfaces() {
        // (an offset's enclosure says nothing, anywhere)
        if matches!(s,Surface::Offset(..)) { continue }
        let moved = s.moved(&out);
        for _ in 0..40 {
            let x = [rng.uniform(du[0],du[1]),rng.uniform(dv[0],dv[1])];
            let w = [(du[1]-du[0])*rng.uniform(0.,1.).powi(3),(dv[1]-dv[0])*rng.uniform(0.,1.).powi(3)];
            let (u,v) = ([x[0],(x[0]+w[0]).min(du[1])],[x[1],(x[1]+w[1]).min(dv[1])]);
            let (near,there) = (s.enclose([u,v]),moved.enclose([u,v]));
            assert!(near.bounded() && there.bounded(),"{what} over {u:?} × {v:?}: no enclosure");
            for i in 0..=4 { for j in 0..=4 {
                let uv = [u[0]+(u[1]-u[0])*i as f64/4.,v[0]+(v[1]-v[0])*j as f64/4.];
                let (p,su,sv) = moved.d1(uv);
                let held = |iv: Iv,x: f64| iv.lo()-rounding <= x && x <= iv.hi()+rounding;
                assert!((0..3).all(|c| held(there.at[c],p[c]) && held(there.d[0][c],su[c]) && held(there.d[1][c],sv[c])),
                    "{what}, moved, over {u:?} × {v:?} at {uv:?}: {p:?} {su:?} {sv:?} outside {there:?}");
            } }
            let width = |iv: Iv| iv.hi()-iv.lo();
            for c in 0..3 {
                for (which,a,b) in [("point",near.at[c],there.at[c]),("rate along u",near.d[0][c],there.d[0][c]),("rate along v",near.d[1][c],there.d[1][c])] {
                    assert!(width(b) <= 1.01*width(a)+rounding,"{what} over {u:?} × {v:?}: its {which} enclosed {:e} wide at the origin and {:e} moved",
                        width(a),width(b));
                }
            }
        }
    }
}
