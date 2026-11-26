//! **Faces trimmed on nets singular outside them** (#1013): the four capabilities the crossed
//! tumbling tori's Booleans lack, each pinned alone by a block under a Bézier patch — its top face
//! on the whole patch, trimmed — cut by a block or by a plane, with no sweep in it. The patch is
//! singular only outside the face: folded back over the face's rim, a second layer just beside
//! the face, a cusp, a pinch to a point. Each is brep-bench's `nurbs/` case of the same name.
//!
//! Every face is read on the part of its net it uses: its parameter box grown a sixteenth each way
//! only while the net stays regular (`Located::grown_parameters`), so a fold or a pinch just past
//! the face is never in it. The `…_trimmed_nearer_…` cases trim the face within that sixteenth of
//! the cusp or the pinch.
//!
//! Each case comes twice. Its **control** is the block with its top face on the patch cut down to
//! the face's own part (`Net::segment`: the same surface at the same parameters, nothing of the
//! patch past the face), which builds to the closed form today; that it does is the evidence that
//! the gate's case and answer are right. Its **gate** is the same block on the whole patch; one the
//! Boolean still refuses is ignored, its `ignore` saying how it fails. Each is read by all three
//! Booleans (or, cut by a plane, on both sides of it), each `check`ed and its volume held to the
//! closed form and to inclusion–exclusion (|A ∪ B| + |A ∩ B| = |A| + |B|, |A − B| = |A| − |A ∩ B|).
use gcs_core::brep::boolean::{boolean,Op};
use gcs_core::brep::build::{block,kept,sheet};
use gcs_core::brep::geom::{Surface,V};
use gcs_core::brep::nurbs::Net;
use gcs_core::brep::props::volume;
use gcs_core::brep::topo::Brep;
use std::sync::Arc;

const TOL: f64 = 1e-7;

/// The Bézier net of degree (`poles.len() − 1`, `poles[0].len() − 1`) on `poles`.
pub fn bezier(poles: Vec<Vec<V>>) -> Net {
    let clamped = |d: usize| [vec![0.;d+1],vec![1.;d+1]].concat();
    let (du,dv) = (poles.len()-1,poles[0].len()-1);
    Net {du,dv,uknots:clamped(du),vknots:clamped(dv),poles,weights:None}
}

/// A net of degree 1 in `v` whose rows at `v = 0` and `v = 1` are those of `z` over `x`, the second
/// a tenth higher: `(x(u), v, z(u) + v/10)`.
pub fn rising(x: &[f64],z: &[f64]) -> Net {
    bezier(x.iter().zip(z).map(|(&x,&z)| vec![[x,0.,z],[x,1.,z+0.1]]).collect())
}

/// The block standing on the floor `z = 0` under the part `us × vs` of `patch` (a graph over the
/// floor), its walls upright, sewn: its top face on the whole of `patch`, trimmed to the part
/// (`whole`), or on the part alone, cut down to it (`Net::segment`, the same parameters, so the
/// face's trims are the same either way).
pub fn under(patch: &Net,us: [f64;2],vs: [f64;2],whole: bool) -> Brep {
    let top = patch.segment(us,vs);
    let floored = |r: &Vec<V>| r.iter().map(|p| [p[0],p[1],0.]).collect();
    let floor = Net {poles:top.poles.iter().map(floored).collect(),..top.clone()};
    let wall = |pts: Vec<V>,knots: &[f64],d: usize| Net {du:d,dv:1,uknots:knots.to_vec(),
        vknots:vec![0.,0.,1.,1.],poles:pts.iter().map(|p| vec![[p[0],p[1],0.],*p]).collect(),weights:None};
    let (last,lastv) = (top.poles.len()-1,top.poles[0].len()-1);
    let walls = [
        wall(top.poles[0].clone(),&top.vknots,top.dv),
        wall(top.poles[last].clone(),&top.vknots,top.dv),
        wall(top.poles.iter().map(|r| r[0]).collect(),&top.uknots,top.du),
        wall(top.poles.iter().map(|r| r[lastv]).collect(),&top.uknots,top.du),
    ];
    let mut pieces = vec![sheet(top.clone()).unwrap(),sheet(floor).unwrap()];
    pieces[0].faces[0].name = "top".into();
    pieces.extend(walls.into_iter().map(|w| sheet(w).unwrap()));
    let mut b = gcs_core::brep::sew::sewn(&pieces,1e-9).unwrap();
    if whole {
        let f = b.faces.iter_mut().find(|f| f.name == "top").unwrap();
        let Surface::BSpline(frame,_) = f.surface else { unreachable!("the top on its patch") };
        f.surface = Surface::BSpline(frame,Arc::new(patch.clone()));
    }
    b
}

/// `got` within `rel` of `want`, said as `name`.
fn close(name: &str,got: f64,want: f64,rel: f64) {
    assert!((got-want).abs() <= rel*want,"{name}: {got} against {want} ({:e})",(got-want)/want);
}

/// `a` cut by the plane through `o`, kept on each side of it in turn (`build::kept`, as
/// brep-bench's implementation keeps it): the side `keep` points to holding `common`, to `rel`, and
/// the two sides together `va`.
fn sliced(a: &Brep,o: V,keep: V,va: f64,common: f64,rel: f64) {
    close("A",volume(a),va,rel);
    for (name,k,want) in [("kept",keep,common),("cut off",keep.map(|x| -x),va-common)] {
        let r = kept(a,o,k,0.).unwrap_or_else(|e| panic!("{name}: {e}"));
        assert!(r.check(TOL).is_ok(),"{name}: {:?}",r.check(TOL).err());
        close(name,volume(&r),want,rel);
    }
}

/// `a` and `b` by all three Booleans: each built and checked, its volume the closed form's (`va`,
/// `vb` and `common`, |A ∩ B|) to `rel`, so they hold inclusion–exclusion.
fn held(a: &Brep,b: &Brep,va: f64,vb: f64,common: f64,rel: f64) {
    close("A",volume(a),va,rel);
    let ops = [("A ∪ B",Op::Union,va+vb-common),("A ∩ B",Op::Common,common),("A − B",Op::Cut,va-common)];
    for (name,op,want) in ops {
        let r = boolean(a,b,op,TOL).unwrap_or_else(|e| panic!("{name}: {e}"));
        assert!(r.check(TOL).is_ok(),"{name}: {:?}",r.check(TOL).err());
        close(name,volume(&r),want,rel);
    }
}

/// The patch folded over its own rim: `(s², v, 1/2 + 3s⁴/10 + v/10)`, `s = 2u − 1`, over
/// `u ≥ 1/2` the graph of `z = 1/2 + 3x²/10 + y/10` and over `u ≤ 1/2` the same graph again, the
/// patch folding back along `u = 1/2` (`x = 0`). The face is `u ≥ 13/20` (`x ≥ 9/100`); the block B
/// notches its corner, its side `y = 1/2` crossing the face's rim `u = 13/20`.
fn folded(whole: bool) {
    let patch = rising(&[1.,0.,-1./3.,0.,1.],&[0.8,0.2,0.8,0.2,0.8]);
    // |A| = ∫∫ (1/2 + 3x²/10 + y/10) over 9/100 ≤ x ≤ 1, 0 ≤ y ≤ 1; |A ∩ B| over x ≤ 1/2, y ≤ 1/2
    held(&under(&patch,[0.65,1.],[0.,1.],whole),&block([0.05,-1.,-1.],[0.5,0.5,3.]).unwrap(),0.6004271,0.45*1.5*4.,
        0.11383855,1e-10);
}

/// The patch with its folded-back layer just beneath the face: `(s², v, 1/2 + 3s⁴/10 + s³/1000 +
/// v/10)`, the layer `u ≤ 1/2` standing `x^{3/2}/500` below the face's (1.8e-4 to 5.7e-4 across the
/// hole, as the crossed tori's layers stand 3.5e-4 apart); B a square hole through it.
fn layered(whole: bool) {
    let patch = rising(&[1.,0.,-1./3.,0.,1.],&[0.799,0.2005,0.8,0.1995,0.801]);
    // |A| = 0.6004271 + ∫∫ x^{3/2}/1000 = 0.6004271 + (1 − 0.09^{5/2})/2500; |A ∩ B| by quadrature
    held(&under(&patch,[0.65,1.],[0.,1.],whole),&block([0.4,0.3,-1.],[0.7,0.6,3.]).unwrap(),0.600826128,0.3*0.3*4.,
        0.057457052463345157,1e-10);
}

/// The cuspidal patch: `(s², v, 1/2 + s³ + v/10)`, its two layers meeting in a cusp along
/// `u = 1/2`; the face `u ≥ 11/20` (`x ≥ 1/100`), cut by the plane `z = 11y/10`, which meets the
/// face in `y = 1/2 + x^{3/2}` and runs on through the cusp a hundredth past the face's rim.
fn cuspidal(whole: bool) {
    let patch = rising(&[1.,-1./3.,-1./3.,1.],&[-0.5,1.5,-0.5,1.5]);
    let l = 2.21f64.sqrt();
    // kept below the plane: |A| = (99/100)(11/20) + (2/5)(1 − 10⁻⁵); |A ∩ B| the column
    // ∫₀^{y*} 11y/10 + ∫_{y*}^1 (1/2 + x^{3/2} + y/10), y* = min(1, 1/2 + x^{3/2}), over x
    sliced(&under(&patch,[0.55,1.],[0.,1.],whole),[0.5,0.5,0.55],[0.,1.1/l,-1./l],0.944496,0.51031272172170669,1e-8);
}

/// The patch pinched to a point: `(s³, t³, 1/2 + st)`, `t = 2v − 1`, the graph of
/// `z = 1/2 + ∛(xy)`, its parametrization singular at `u = v = 1/2` alone; the face
/// `u, v ≥ 11/20` (`x, y ≥ 1/1000`), cut by the plane `−x + y + z/10 = 13/250`, which passes 0.002
/// beside the pinch.
fn pinched(whole: bool) {
    let (x,s) = ([-1.,1.,-1.,1.],[-1.,-1./3.,1./3.,1.]);
    let row = |i: usize| (0..4).map(|j| [x[i],x[j],0.5+s[i]*s[j]]).collect();
    let patch = bezier((0..4).map(row).collect());
    let l = 2.01f64.sqrt();
    let a = 0.75*(1.-0.001f64.powf(4./3.));
    // kept where −x + y + z/10 ≤ 13/250: |A| = (999/1000)²/2 + (3/4 (1 − 10^{−4}))²; |A ∩ B| by
    // quadrature
    sliced(&under(&patch,[0.55,1.],[0.55,1.],whole),[0.5,0.5+0.052,0.],[1./l,-1./l,-0.1/l],0.5*0.999*0.999+a*a,
        0.5244906492156999,1e-8);
}

#[test]
fn a_patch_folded_over_its_rim_cut_down_to_its_face_is_notched() { folded(false) }

#[test]
fn a_patch_folded_over_its_rim_is_notched_across_the_rim() { folded(true) }

#[test]
fn a_patch_over_a_near_layer_cut_down_to_its_face_is_pierced() { layered(false) }

#[test]
fn a_patch_over_a_near_layer_is_pierced() { layered(true) }

/// The cuspidal patch's face trimmed nearer its cusp, `u ≥ 13/25` (`x ≥ 0.0016`), within a
/// sixteenth of its parameters of it, and cut by the steep plane `z = 51y/10 − 2` through the cusp:
/// the meeting traced on the net cut to the face's grown box reaches the cusp still (T3 itself, not
/// the bound T2's reading gives it).
fn cuspidal_nearer(whole: bool) {
    let patch = rising(&[1.,-1./3.,-1./3.,1.],&[-0.5,1.5,-0.5,1.5]);
    let l = 27.01f64.sqrt();
    // |A| = (1 − 0.0016)(11/20) + (2/5)(1 − 0.0016^{5/2}); |A ∩ B| the column
    // ∫_{20/51}^{y*} (51y/10 − 2) + ∫_{y*}^1 (1/2 + x^{3/2} + y/10), y* = 1/2 + x^{3/2}/5, over x,
    // by quadrature
    sliced(&under(&patch,[0.52,1.],[0.,1.],whole),[0.5,0.5,0.55],[0.,5.1/l,-1./l],0.94911995904,0.49164939128486972,1e-8);
}

/// The pinched patch's face trimmed nearer its pinch, `u, v ≥ 51/100` (`x, y ≥ 8·10⁻⁶`), cut by the
/// plane `−3x + y + z/10 = 0.0502`, 2e-4 beside the pinch: as `cuspidal_nearer`, T4 itself.
fn pinched_nearer(whole: bool) {
    let (x,s) = ([-1.,1.,-1.,1.],[-1.,-1./3.,1./3.,1.]);
    let row = |i: usize| (0..4).map(|j| [x[i],x[j],0.5+s[i]*s[j]]).collect();
    let patch = bezier((0..4).map(row).collect());
    let l = 10.01f64.sqrt();
    // |A| = (1 − 8·10⁻⁶)²/2 + (3/4 (1 − (8·10⁻⁶)^{4/3}))²; |A ∩ B| by quadrature
    sliced(&under(&patch,[0.51,1.],[0.51,1.],whole),[0.,0.0002,0.5],[3./l,-1./l,-0.1/l],1.0624918200320144,0.91479074469600325,1e-8);
}

#[test]
fn a_cuspidal_patch_cut_down_to_its_face_is_sliced() { cuspidal(false) }

#[test]
fn a_cuspidal_patch_is_sliced_through_its_cusp() { cuspidal(true) }

#[test]
fn a_pinched_patch_cut_down_to_its_face_is_sliced() { pinched(false) }

#[test]
fn a_pinched_patch_is_sliced_beside_its_pinch() { pinched(true) }

#[test]
fn a_cuspidal_patch_trimmed_nearer_cut_down_to_its_face_is_sliced() { cuspidal_nearer(false) }

#[test]
fn a_cuspidal_patch_trimmed_nearer_is_sliced_through_its_cusp() { cuspidal_nearer(true) }

#[test]
fn a_pinched_patch_trimmed_nearer_cut_down_to_its_face_is_sliced() { pinched_nearer(false) }

#[test]
fn a_pinched_patch_trimmed_nearer_is_sliced_beside_its_pinch() { pinched_nearer(true) }
