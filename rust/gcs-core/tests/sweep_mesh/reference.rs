//! A deliberately **non-rigorous** reference surface of the swept material, for *looking at*.
//!
//! Everything else in this suite asks the field one point at a time and reasons about the answers.
//! That is how a whole session went by without anyone noticing that the tumbling cylinder was
//! missing its entire equatorial band: the numbers were all there (a refused area of 13.63 against
//! a solid volume of 6.65) and none of them said so out loud. A picture said it at once.
//!
//! So this contours the field on a uniform grid and writes an STL. It claims **nothing**, and it is
//! not the certified extractor: `MaterialEvaluator::boundary` refuses on an ambiguous point or a
//! spent budget, exactly as it should, which is why it cannot draw a picture. This one cannot
//! refuse. Two things it is good for and one it is not: it answers **"what is grossly missing?"**
//! and its **volume** is the one-number alarm whose absence cost a session — but it does *not*
//! answer "is this surface right", being blind to anything thinner than a cell, and this project's
//! hard cases are the thin ones. Never assert on its surface.
//!
//! **It reads signs, never values, and that was learned the hard way.** The first version sampled
//! the midpoint of the field's value enclosure as a scalar and contoured its zero set. That is
//! wrong twice over, and the control caught both: the turned box came out at 15.8989 and then
//! 15.8632 against a closed form of **15.3237**, and the box it located for that case spanned 6.4
//! units in z where the object is 2 tall — the coarse pass had found "material" far outside the
//! solid. The reason is that for a sweep the enclosure is a **conservative bracket**, not an
//! approximate value: a point well outside can come back as something like [−20, +0.1], whose
//! midpoint is negative. Tightening the band changed nothing because the band was never the cause.
//!
//! So it does what this project's first principle says instead — *a strict sign change brackets the
//! boundary* — classifying each grid point by its strict sign and placing every crossing by
//! **bisection**. Nothing is interpolated from a number the field never promised.
//!
//! Marching **tetrahedra**, not cubes: six tets fanned about one diagonal is three cases and thirty
//! lines, where marching cubes is a 256-entry table that can be silently wrong. More triangles,
//! watertight by construction, and a reference instrument must not itself need debugging.

use super::{creases,harness::{self,V3}};
use gcs_core::solid::MaterialField;
use gcs_core::solid::swept_boundary::{FieldJudge,Sign,SweptBoundaryOptions};

/// The six tetrahedra of a cube, fanned about the 0-6 diagonal. Every cell numbers its corners the
/// same way, so two neighbours cut their shared face along the same diagonal and the surface has no
/// cracks between cells — the whole reason for preferring tets to a cube table here.
const TETS: [[usize;4];6] = [[0,1,2,6],[0,2,3,6],[0,3,7,6],[0,7,4,6],[0,4,5,6],[0,5,1,6]];

/// Corner `k` of a cell, as offsets in x, y, z.
fn corner(k: usize) -> [usize;3] {
    [[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]][k]
}

fn sub(a: V3,b: V3) -> V3 { [a[0]-b[0],a[1]-b[1],a[2]-b[2]] }
fn cross(a: V3,b: V3) -> V3 { [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]] }
fn dot(a: V3,b: V3) -> f64 { a[0]*b[0]+a[1]*b[1]+a[2]*b[2] }

/// Whether the point is material. The field's material is the closure of `{f < 0}`, so a point the
/// judge can only call `Near` counts as material. Cheap query first, and only a point it cannot
/// decide is asked again at the near tolerance.
fn material(judge: &mut FieldJudge,p: V3,probe: f64) -> bool {
    match judge.deep_sign(p,probe) {
        Ok((Sign::Material,_)) => true,
        Ok((Sign::Exterior,_)) => false,
        _ => matches!(judge.sign(p),Ok((Sign::Material,_)) | Ok((Sign::Near {..},_))),
    }
}

/// Where the boundary crosses a segment with material at one end and not the other, by bisection —
/// which is what a strict sign change licenses, and all it licenses.
fn crossing(judge: &mut FieldJudge,inside: V3,outside: V3,probe: f64) -> V3 {
    let (mut a,mut b) = (inside,outside);
    for _ in 0..7 {
        let mid: V3 = std::array::from_fn(|k| 0.5*(a[k]+b[k]));
        if material(judge,mid,probe) { a = mid; } else { b = mid; }
    }
    std::array::from_fn(|k| 0.5*(a[k]+b[k]))
}

/// The signed volume a closed triangle mesh encloses.
fn volume(tris: &[[V3;3]]) -> f64 {
    tris.iter().map(|[a,b,c]| dot(*a,cross(*b,*c))/6.).sum()
}

/// Sample the field over its own support and contour it. Returns the triangles and what it cost.
pub(super) fn rough(source: &str,h: f64) -> (Vec<[V3;3]>,String) {
    let e = harness::read(source);
    let swept = harness::solid(&e,"swept");
    let options = SweptBoundaryOptions {sagitta:0.02,spacing:0.5,..Default::default()};
    let field = MaterialField::read(&e.sketch,swept,1e-10).unwrap();
    // The extent comes from the **field**, never from the mesh under test: a mesh missing a region
    // could hand over a box that hides it. But `support_bounds` is deliberately conservative —
    // about 20 by 34 by 32 where the turned box is some 2 by 3 by 2.5, which is 22 million points
    // at h = 0.1 — so the field locates itself first on a grid eight times coarser, and the fine
    // pass runs over where material was actually found, padded and clamped back to the support.
    let support = field.support_bounds().unwrap().expect("the material has no finite support");
    let (slo,shi): (V3,V3) = (std::array::from_fn(|k| support[k].bounds()[0]),
                              std::array::from_fn(|k| support[k].bounds()[1]));
    let probe = options.probe_distance();
    let mut judge = FieldJudge::new(field,options.judge_tolerance(),4000,1000,4096);
    let started = std::time::Instant::now();
    let coarse = h*8.;
    let cn: [usize;3] = std::array::from_fn(|k| (((shi[k]-slo[k])/coarse).ceil() as usize+1).max(2));
    let (mut flo,mut fhi) = ([f64::INFINITY;3],[f64::NEG_INFINITY;3]);
    for i in 0..cn[0] { for j in 0..cn[1] { for k in 0..cn[2] {
        let p: V3 = [slo[0]+i as f64*coarse,slo[1]+j as f64*coarse,slo[2]+k as f64*coarse];
        if material(&mut judge,p,probe) { for d in 0..3 { flo[d] = flo[d].min(p[d]); fhi[d] = fhi[d].max(p[d]); } }
    } } }
    assert!(flo[0].is_finite(),"the coarse pass found no material anywhere in the support box");
    let (lo,hi): (V3,V3) = (std::array::from_fn(|k| (flo[k]-2.*coarse).max(slo[k])),
                            std::array::from_fn(|k| (fhi[k]+2.*coarse).min(shi[k])));
    let located = started.elapsed();
    let n: [usize;3] = std::array::from_fn(|k| (((hi[k]-lo[k])/h).ceil() as usize+1).max(2));
    let pos = |i: usize,j: usize,k: usize| -> V3 { [lo[0]+i as f64*h,lo[1]+j as f64*h,lo[2]+k as f64*h] };
    let mut inside = vec![false;n[0]*n[1]*n[2]];
    let fine = std::time::Instant::now();
    let mut spoke = fine;
    for i in 0..n[0] {
        for j in 0..n[1] { for k in 0..n[2] {
            inside[(i*n[1]+j)*n[2]+k] = material(&mut judge,pos(i,j,k),probe);
        } }
        if spoke.elapsed().as_secs() >= 3 {
            eprintln!("   sampling: plane {i} of {}, {:?} so far",n[0],fine.elapsed());
            spoke = std::time::Instant::now();
        }
    }
    let sampled = fine.elapsed();
    // contour, bisecting every crossing edge
    let walked = std::time::Instant::now();
    let at = |i: usize,j: usize,k: usize| inside[(i*n[1]+j)*n[2]+k];
    let mut tris: Vec<[V3;3]> = Vec::new();
    let mut edges = 0;
    for i in 0..n[0]-1 { for j in 0..n[1]-1 { for k in 0..n[2]-1 {
        let (p,s): (Vec<V3>,Vec<bool>) = (0..8).map(|c| {
            let d = corner(c);
            (pos(i+d[0],j+d[1],k+d[2]),at(i+d[0],j+d[1],k+d[2]))
        }).unzip();
        for tet in TETS {
            let ins: Vec<usize> = tet.iter().copied().filter(|&c| s[c]).collect();
            let out: Vec<usize> = tet.iter().copied().filter(|&c| !s[c]).collect();
            if ins.is_empty() || out.is_empty() { continue; }
            let mut cut = |a: usize,b: usize| -> V3 { edges += 1; crossing(&mut judge,p[a],p[b],probe) };
            let mut quad: Vec<V3> = Vec::new();
            match ins.len() {
                1 => { for &b in &out { quad.push(cut(ins[0],b)); } }
                3 => { for &a in &ins { quad.push(cut(a,out[0])); } }
                _ => {
                    quad.push(cut(ins[0],out[0])); quad.push(cut(ins[0],out[1]));
                    quad.push(cut(ins[1],out[1])); quad.push(cut(ins[1],out[0]));
                }
            }
            // wound away from the material, so the mesh's own volume comes out positive
            let anchor = p[ins[0]];
            let mut lay = |a: V3,b: V3,c: V3| {
                let centre: V3 = std::array::from_fn(|k| (a[k]+b[k]+c[k])/3.);
                if dot(cross(sub(b,a),sub(c,a)),sub(centre,anchor)) < 0. { tris.push([a,c,b]); }
                else { tris.push([a,b,c]); }
            };
            if quad.len() == 3 { lay(quad[0],quad[1],quad[2]); }
            else { lay(quad[0],quad[1],quad[2]); lay(quad[0],quad[2],quad[3]); }
        }
    } } }
    let report = format!("located in {located:?} ({}x{}x{} coarse at {coarse}); grid {}x{}x{} at \
        h={h} over {:?}..{:?}, {} points in {sampled:?}; {edges} edges bisected in {:?}; \
        {} triangles, volume {:.4}; {}",cn[0],cn[1],cn[2],n[0],n[1],n[2],
        lo.map(|x| (x*1e2).round()/1e2),hi.map(|x| (x*1e2).round()/1e2),n[0]*n[1]*n[2],
        walked.elapsed(),tris.len(),volume(&tris),judge.stats.report());
    (tris,report)
}

/// Whether the missing wedge is ever traced at all. The reference says the absent surface is two
/// wedges about **z = 0** (angle ≈ 0° and ≈ 180° about the tumble axis), spanning the whole x range
/// and radius 0.52 to 1.41 — and that sheets 1 to 4 stop dead on ±15° and ±165°, covering exactly
/// the angular complement. Stopping that precisely looks more like clipping than absence, so this
/// asks the tracer directly, before the pipeline touches anything: are there seed points in the
/// wedge?
///
/// Points present ⇒ the surface is traced and something downstream discards it, which is a trim
/// defect and means a new generator would fix nothing. Points absent ⇒ a family the tracer never
/// produces. The whole design depends on which, so it is asked rather than assumed.
#[test]
#[ignore]
fn whether_the_missing_wedge_is_ever_traced() {
    use gcs_core::solid::swept_boundary::seeds;
    let source = creases::tumbling_cylinder();
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let (_,sheets,_) = seeds(&e.sketch,swept,0.5,0.02,&|_| {}).unwrap();
    let angle = |p: &V3| p[2].atan2(p[1]).to_degrees();
    // the wedge the reference reports uncovered
    let in_wedge = |p: &V3| { let a = angle(p).abs(); a < 15. || a > 165. };
    eprintln!("== tumbling cylinder: do the tracer's own seed points reach the missing wedge?");
    eprintln!("   (the wedge is |angle| < 15° or > 165° about the tumble axis, i.e. z near 0)");
    let mut total = 0;
    for (i,s) in sheets.iter().enumerate() {
        let inside = s.points.iter().filter(|p| in_wedge(p)).count();
        total += inside;
        let (lo,hi) = s.points.iter().fold((f64::INFINITY,f64::NEG_INFINITY),
            |(lo,hi),p| { let a = angle(p); (lo.min(a),hi.max(a)) });
        let (xlo,xhi) = s.points.iter().fold((f64::INFINITY,f64::NEG_INFINITY),
            |(lo,hi),p| (lo.min(p[0]),hi.max(p[0])));
        eprintln!("   sheet {i}: {} points, {inside} in the wedge; angle {lo:>7.1}°..{hi:>7.1}°, \
            x {xlo:.2}..{xhi:.2}, times {:.3}..{:.3}",s.points.len(),
            s.times.first().copied().unwrap_or(f64::NAN),s.times.last().copied().unwrap_or(f64::NAN));
    }
    eprintln!("== {total} of {} traced points lie in the wedge",
        sheets.iter().map(|s| s.points.len()).sum::<usize>());
    eprintln!("   none ⇒ the tracer never makes that surface (a generation gap); some ⇒ it is made \
        and then lost (a trim defect, and a generator would fix nothing)");
}

/// Where the reference has surface and the construction has none — and, more to the point, **what
/// that surface is**. Before building any generator, this has to distinguish "a family the tracer
/// never produced" from "a family it produced too small" or "one it produced and something
/// downstream discarded". Getting that wrong means building a construction that fixes nothing and
/// hides the real gap.
///
/// The geometry says what to look for. For this fixture the wall is (x−3)² + y² = 1 turning about
/// x̂, so the velocity is v = (0, −z, y) and on the wall **n·v = −yz**: the grazing set is
/// {yz = 0}, three pieces — the two straight lines through the axis-piercing points (y = 0 at
/// x = 2 and x = 4) and the circle z = 0. Sweeping the first line gives the planar bowtie at
/// x = 2 of radius 1 over 60°, which is exactly sheet 6's measured box (x ≈ 2, y ∈ [−0.5, 0.5] =
/// ±sin 30°, z ∈ [−1, 1]). So if the uncovered region sits at angles the bowtie already covers,
/// the bowtie is too small; if it sits elsewhere — say on a band at many x, which is what the
/// z = 0 circle would sweep — then a whole family is absent and the bowtie is a red herring.
///
/// Reports the uncovered **area** directly, which is the number that matters.
#[test]
#[ignore]
fn where_the_reference_has_surface_and_the_construction_has_none() {
    use gcs_core::solid::swept_boundary::{Stage,SweptBoundaryOptions,construct};
    use std::collections::BTreeMap;
    let h: f64 = std::env::var("SOLVENT_REFERENCE_H").ok().and_then(|v| v.parse().ok()).unwrap_or(0.1);
    let source = creases::tumbling_cylinder();
    let (tris,report) = rough(&source,h);
    eprintln!("== the reference: {report}");
    // the construction, with each sheet's angular extent about the tumble axis
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let options = SweptBoundaryOptions {sagitta:0.02,spacing:0.5,..Default::default()};
    let mut built: Vec<V3> = Vec::new();
    let mut spread: BTreeMap<u32,(f64,f64,f64,f64)> = Default::default();
    let _ = construct(&e.sketch,swept,&options,&|_| {},&mut |stage,_| {
        if let Stage::Zipped {mesh,..} = stage {
            built = mesh.vertices.clone();
            for (i,t) in mesh.triangles.iter().enumerate() {
                for v in t {
                    let p = mesh.vertices[*v as usize];
                    let (ang,x) = (p[2].atan2(p[1]).to_degrees(),p[0]);
                    let s = spread.entry(mesh.sheet[i])
                        .or_insert((f64::INFINITY,f64::NEG_INFINITY,f64::INFINITY,f64::NEG_INFINITY));
                    s.0 = s.0.min(ang); s.1 = s.1.max(ang); s.2 = s.2.min(x); s.3 = s.3.max(x);
                }
            }
        }
    });
    assert!(!built.is_empty(),"the construction handed over no mesh");
    // every reference triangle: is there any constructed vertex near it?
    let area = |[a,b,c]: &[V3;3]| 0.5*(cross(sub(*b,*a),sub(*c,*a))[0].powi(2)
        +cross(sub(*b,*a),sub(*c,*a))[1].powi(2)+cross(sub(*b,*a),sub(*c,*a))[2].powi(2)).sqrt();
    let (mut whole,mut lost) = (0.,0.);
    let mut bins: BTreeMap<i64,f64> = Default::default();
    let mut by_x: BTreeMap<i64,f64> = Default::default();
    let (mut r_lo,mut r_hi) = (f64::INFINITY,f64::NEG_INFINITY);
    let (mut near_two_lo,mut near_two_hi) = (f64::INFINITY,f64::NEG_INFINITY);
    for tri in &tris {
        let a = area(tri);
        whole += a;
        let centre: V3 = std::array::from_fn(|k| (tri[0][k]+tri[1][k]+tri[2][k])/3.);
        let near = built.iter().map(|v| harness::distance(centre,*v)).fold(f64::INFINITY,f64::min);
        if near <= 0.15 { continue; }
        lost += a;
        let ang = centre[2].atan2(centre[1]).to_degrees();
        let r = (centre[1]*centre[1]+centre[2]*centre[2]).sqrt();
        r_lo = r_lo.min(r); r_hi = r_hi.max(r);
        *bins.entry(((ang+180.)/30.).floor() as i64).or_insert(0.) += a;
        *by_x.entry((centre[0]*2.).round() as i64).or_insert(0.) += a;
        if (centre[0]-2.).abs() < 0.15 { near_two_lo = near_two_lo.min(ang); near_two_hi = near_two_hi.max(ang); }
    }
    eprintln!("== reference area {whole:.3}, of it {lost:.3} ({:.1}%) farther than 0.15 from any \
        constructed vertex; uncovered radius from the axis {r_lo:.3}..{r_hi:.3}",100.*lost/whole);
    eprintln!("   uncovered area by angle about the axis (30° bins from −180°):");
    for (b,a) in &bins { eprintln!("     {:>5}°..{:>5}°: {a:.3}",-180+30*(*b as i32),-150+30*(*b as i32)); }
    eprintln!("   uncovered area by x (half-unit bins): {}",
        by_x.iter().map(|(b,a)| format!("{:.2}:{a:.2}",*b as f64/2.)).collect::<Vec<_>>().join("  "));
    if near_two_lo.is_finite() {
        eprintln!("   uncovered near x=2 spans {near_two_lo:.1}°..{near_two_hi:.1}° about the axis");
    }
    eprintln!("   each sheet's own angular and x extent (the bowtie should be sheet 6 at x≈2):");
    for (s,(alo,ahi,xlo,xhi)) in &spread {
        let who = if *s == u32::MAX { "zip".to_string() } else { format!("sheet {s}") };
        eprintln!("     {who:>8}: {alo:>7.1}°..{ahi:>7.1}°, x {xlo:.2}..{xhi:.2}");
    }
}

/// Write a rough reference surface for a case that closes and one that does not, and report each
/// one's volume. The closing case is the control: its reference volume must land near the closed
/// form, or the reference is what is wrong — which is exactly how the value-midpoint version of
/// this was caught.
#[test]
#[ignore]
fn rough_reference_surfaces() {
    let dir = std::path::PathBuf::from(std::env::var("SOLVENT_REFERENCE")
        .unwrap_or_else(|_| "/tmp".into()));
    std::fs::create_dir_all(&dir).unwrap();
    let h: f64 = std::env::var("SOLVENT_REFERENCE_H").ok().and_then(|v| v.parse().ok()).unwrap_or(0.1);
    let cases: [(&str,String,f64);2] = [
        // the control: certified at 15.3167, closed form 15.3237
        ("turned_box",creases::turned_box(),15.3237),
        ("tumbling_cylinder",creases::tumbling_cylinder(),f64::NAN),
    ];
    for (name,source,expected) in &cases {
        eprintln!("== {name}");
        let (tris,report) = rough(source,h);
        eprintln!("   {report}");
        if expected.is_finite() {
            let got = volume(&tris);
            eprintln!("   the control: {got:.4} against the closed form {expected:.4}, \
                {:+.2}% — the rest of this is trustworthy only as far as that is",
                100.*(got-expected)/expected);
        }
        let (vertices,triangles): (Vec<V3>,Vec<[u32;3]>) = {
            let mut v = Vec::with_capacity(tris.len()*3);
            let mut t = Vec::with_capacity(tris.len());
            for tri in &tris {
                let i = v.len() as u32;
                v.extend_from_slice(tri);
                t.push([i,i+1,i+2]);
            }
            (v,t)
        };
        let (bytes,_) = creases::stl(&vertices,&triangles,name);
        let path = dir.join(format!("{name}_reference.stl"));
        std::fs::write(&path,bytes).unwrap();
        eprintln!("   wrote {}",path.display());
    }
}
