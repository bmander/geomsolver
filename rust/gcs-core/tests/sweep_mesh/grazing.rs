//! Milestone 5a: the region a grazing planar face sweeps in its own plane, built in rows and
//! judged against areas worked out independently (closed forms, one-dimensional integrals of
//! the swept measure per ring, and exact slides), the rim on the curves it must lie on, and a
//! watertight, counter-clockwise mesh with as many boundary loops as the region has.
use super::{harness,motions,tools};
use gcs_core::solid::{PlanarEdge,SweepContacts};
use gcs_core::solid::swept_boundary::{boundary_loops,seeds,grazing::{PlaneMotion,Region,grazing_faces,swept_region}};
use std::f64::consts::{FRAC_PI_2,PI,TAU};

fn rect(x0: f64,y0: f64,x1: f64,y1: f64) -> Vec<PlanarEdge> {
    let c = [[x0,y0],[x1,y0],[x1,y1],[x0,y1]];
    (0..4).map(|k| PlanarEdge::Line {start:c[k],end:c[(k+1)%4]}).collect()
}

fn area(r: &Region) -> f64 {
    r.triangles.iter().map(|t| { let [p,q,s] = t.map(|v| r.points[v as usize]); 0.5*((q[0]-p[0])*(s[1]-p[1])-(q[1]-p[1])*(s[0]-p[0])) }).sum()
}

/// Every triangle counter-clockwise, every edge used once or twice, and the boundary loops.
fn loops(r: &Region) -> usize {
    let mut uses: std::collections::BTreeMap<(u32,u32),usize> = Default::default();
    for t in &r.triangles {
        let [p,q,s] = t.map(|v| r.points[v as usize]);
        assert!((q[0]-p[0])*(s[1]-p[1])-(q[1]-p[1])*(s[0]-p[0]) > 0.,"a triangle {t:?} is not counter-clockwise");
        for k in 0..3 { let (a,b) = (t[k],t[(k+1)%3]); *uses.entry((a.min(b),a.max(b))).or_default() += 1; }
    }
    // no vertex is claimed by two triangles walking one edge the same way: two cells of a row
    // that overlap instead of merging would each mesh the stretch they share
    let mut walks: std::collections::BTreeMap<(u32,u32),usize> = Default::default();
    for t in &r.triangles { for k in 0..3 { *walks.entry((t[k],t[(k+1)%3])).or_default() += 1; } }
    if let Some((&(a,b),&n)) = walks.iter().find(|(_,&n)| n > 1) {
        panic!("the edge {a}{:?}->{b}{:?} is walked {n} times",r.points[a as usize],r.points[b as usize]);
    }
    // no triangle flat enough for the construction to read as degenerate and drop: such a
    // triangle leaves a hole in the region whose boundary no zip can close
    for t in &r.triangles {
        let [p,q,s] = t.map(|v| { let x = r.points[v as usize]; [x[0],x[1],0.] });
        assert!(!gcs_core::space::degenerate(p,q,s),"the triangle {t:?} at {p:?} {q:?} {s:?} is degenerate");
    }
    if let Some((&(a,b),&n)) = uses.iter().find(|(_,&n)| n > 2) {
        let on: Vec<[u32;3]> = r.triangles.iter().filter(|t| t.contains(&a) && t.contains(&b)).copied().collect();
        panic!("the edge {a}-{b} at {:?} (rows {},{}) and {:?} is used {n} times, by {on:?}",r.points[a as usize],r.row[a as usize],r.row[b as usize],r.points[b as usize]);
    }
    boundary_loops(&r.triangles).len()
}

/// Simpson's rule on `n` (even) intervals.
fn integral(f: &dyn Fn(f64) -> f64,a: f64,b: f64,n: usize) -> f64 {
    let h = (b-a)/n as f64;
    (0..=n).map(|i| f(a+h*i as f64)*if i == 0 || i == n { 1. } else if i%2 == 1 { 4. } else { 2. }).sum::<f64>()*h/3.
}

/// The unit square about its centre turned by `alpha`: the disc to radius 1, and beyond it each
/// ring's four corner arcs, each widened by the turn, until they cover the ring.
fn square_about_centre(alpha: f64) -> f64 {
    let ring = |r: f64| { let w = FRAC_PI_2-2.*(1./r).acos(); r*(4.*(w+alpha)).min(TAU) };
    PI+integral(&ring,1.,2_f64.sqrt(),200_000)
}

const SAGITTA: f64 = 1e-4;
const SPACING: f64 = 0.1;

#[test]
fn a_square_turned_about_its_centre_sweeps_its_corners_arcs() {
    for degrees in [30.,90.,360.] {
        let alpha = (degrees as f64).to_radians();
        let poses = [0.,alpha/3.,alpha];
        let r = swept_region(&rect(-1.,-1.,1.,1.),&[],PlaneMotion::Turn {pivot:[0.,0.],sweep:alpha},SAGITTA,SPACING,&poses).unwrap();
        let (a,expected) = (area(&r),square_about_centre(alpha));
        // an inscribed mesh: never above, and short by at most a sagitta along its rim
        assert!(a <= expected+1e-9 && a >= expected-SAGITTA*12.,"{degrees}°: area {a} against {expected}");
        assert_eq!(loops(&r),1,"{degrees}°");
        // the rim is the corners' arcs and the edges at the two end poses
        for (p,_) in r.points.iter().zip(&r.rim).filter(|(_,rim)| **rim) {
            let on_arc = (p[0].hypot(p[1])-2_f64.sqrt()).abs() < 1e-9;
            let on_edge = [0.,alpha].iter().any(|&t| { let (x,y) = (p[0]*t.cos()+p[1]*t.sin(),-p[0]*t.sin()+p[1]*t.cos()); (x.abs().max(y.abs())-1.).abs() < 1e-9 });
            assert!(on_arc || on_edge,"{degrees}°: rim point {p:?} on neither");
        }
        // the corners' images at every pose are vertices
        for &psi in &poses {
            let c = [2_f64.sqrt()*(PI/4.+psi).cos(),2_f64.sqrt()*(PI/4.+psi).sin()];
            assert!(r.points.iter().any(|p| (p[0]-c[0]).hypot(p[1]-c[1]) < 1e-12),"{degrees}°: no vertex at the corner's image {c:?}");
        }
        assert_eq!(r,swept_region(&rect(-1.,-1.,1.,1.),&[],PlaneMotion::Turn {pivot:[0.,0.],sweep:alpha},SAGITTA,SPACING,&poses).unwrap(),"not deterministic");
    }
}

#[test]
fn a_turn_either_way_sweeps_the_same_region() {
    let a = swept_region(&rect(-1.,-1.,1.,1.),&[],PlaneMotion::Turn {pivot:[0.,0.],sweep:-0.5},SAGITTA,SPACING,&[]).unwrap();
    let expected = square_about_centre(0.5);
    assert!((area(&a)-expected).abs() < SAGITTA*12.,"area {} against {expected}",area(&a));
    assert_eq!(loops(&a),1);
}

#[test]
fn an_annular_sector_turned_about_its_centre_stays_an_annular_sector() {
    // radii 2 to 3 over 40°, turned 30°: the sector over 70°; its arcs are about the pivot
    let (a0,a1) = (0_f64,40_f64.to_radians());
    let face = vec![
        PlanarEdge::Line {start:[2.*a0.cos(),2.*a0.sin()],end:[3.*a0.cos(),3.*a0.sin()]},
        PlanarEdge::Arc {center:[0.,0.],radius:3.,start:a0,sweep:a1-a0},
        PlanarEdge::Line {start:[3.*a1.cos(),3.*a1.sin()],end:[2.*a1.cos(),2.*a1.sin()]},
        PlanarEdge::Arc {center:[0.,0.],radius:2.,start:a1,sweep:a0-a1},
    ];
    let alpha = 30_f64.to_radians();
    let r = swept_region(&face,&[],PlaneMotion::Turn {pivot:[0.,0.],sweep:alpha},SAGITTA,SPACING,&[]).unwrap();
    let expected = (a1-a0+alpha)*(9.-4.)/2.;
    assert!(area(&r) <= expected+1e-9 && area(&r) >= expected-SAGITTA*10.,"area {} against {expected}",area(&r));
    assert_eq!(loops(&r),1);
}

#[test]
fn a_square_turned_about_its_corner_sweeps_a_quarter_disc_and_more() {
    // the unit square with a corner on the pivot, turned 45°: to radius 1 each ring holds the
    // quarter the square covers widened by the turn, beyond it the arc between the square's two
    // far edges
    let alpha = 45_f64.to_radians();
    let r = swept_region(&rect(0.,0.,1.,1.),&[],PlaneMotion::Turn {pivot:[0.,0.],sweep:alpha},SAGITTA,SPACING,&[]).unwrap();
    let expected = 0.5*(FRAC_PI_2+alpha)+integral(&|r: f64| r*(FRAC_PI_2-2.*(1./r).acos()+alpha),1.,2_f64.sqrt(),200_000);
    assert!(area(&r) <= expected+1e-9 && area(&r) >= expected-SAGITTA*10.,"area {} against {expected}",area(&r));
    assert_eq!(loops(&r),1);
}

#[test]
fn a_rectangle_turned_about_a_pivot_beside_it_matches_ring_quadrature() {
    // [2, 4] x [-1, 1] turned 30° about the origin: a circle past x = 4 meets it in two arcs
    // (its far corners), which the turn may or may not join
    let alpha = 30_f64.to_radians();
    let r = swept_region(&rect(2.,-1.,4.,1.),&[],PlaneMotion::Turn {pivot:[0.,0.],sweep:alpha},SAGITTA,SPACING,&[]).unwrap();
    // independently: every ring's angles the rectangle covers, by sampling, widened and measured
    let (rings,steps) = (2000,20_000);
    let (r0,r1) = (2.,17_f64.sqrt());
    let reach = (alpha/(TAU/steps as f64)).round() as usize;
    let mut expected = 0.;
    for i in 0..rings {
        let rho = r0+(r1-r0)*(i as f64+0.5)/rings as f64;
        let on: Vec<bool> = (0..steps).map(|k| { let t = TAU*k as f64/steps as f64-PI; let (x,y) = (rho*t.cos(),rho*t.sin()); (2. ..=4.).contains(&x) && y.abs() <= 1. }).collect();
        // covered where some covered angle lies up to the turn behind
        let mut prefix = vec![0usize;2*steps+1];
        for k in 0..2*steps { prefix[k+1] = prefix[k]+on[k%steps] as usize; }
        let covered = (0..steps).filter(|&k| prefix[steps+k+1] > prefix[steps+k-reach]).count();
        expected += covered as f64/steps as f64*TAU*rho*(r1-r0)/rings as f64;
    }
    assert!((area(&r)-expected).abs() < 2e-3*expected,"area {} against {expected}",area(&r));
    assert_eq!(loops(&r),1);
}

#[test]
fn a_slide_sweeps_exactly() {
    // a 2 x 1 rectangle slid 3 along x, and 1 along (1, 1): a hexagon of its area plus the
    // advance times its width across it
    let r = swept_region(&rect(0.,0.,2.,1.),&[],PlaneMotion::Slide {advance:[3.,0.]},SAGITTA,SPACING,&[1.,2.]).unwrap();
    assert!((area(&r)-5.).abs() < 1e-12,"area {}",area(&r));
    assert_eq!(loops(&r),1);
    let r = swept_region(&rect(0.,0.,2.,1.),&[],PlaneMotion::Slide {advance:[1.,1.]},SAGITTA,SPACING,&[]).unwrap();
    let across = 3./2_f64.sqrt();
    assert!((area(&r)-(2.+2_f64.sqrt()*across)).abs() < 1e-12,"area {}",area(&r));
    assert_eq!(loops(&r),1);
}

#[test]
fn a_frame_slid_past_its_hole_closes_it() {
    // a 3 x 3 square less the unit square in its middle, slid along x: by 0.5 the hole's last
    // half stays open (two loops), by 1.5 it is covered (one)
    let hole = { let mut h = rect(1.,1.,2.,2.); h.reverse(); h.iter_mut().for_each(|e| if let PlanarEdge::Line {start,end} = e { std::mem::swap(start,end) }); h };
    let r = swept_region(&rect(0.,0.,3.,3.),&[hole.clone()],PlaneMotion::Slide {advance:[0.5,0.]},SAGITTA,SPACING,&[]).unwrap();
    assert!((area(&r)-10.).abs() < 1e-12,"by 0.5: area {}",area(&r));
    assert_eq!(loops(&r),2,"by 0.5");
    let r = swept_region(&rect(0.,0.,3.,3.),&[hole],PlaneMotion::Slide {advance:[1.5,0.]},SAGITTA,SPACING,&[]).unwrap();
    assert!((area(&r)-13.5).abs() < 1e-12,"by 1.5: area {}",area(&r));
    assert_eq!(loops(&r),1,"by 1.5");
}

/// The tool's faces under a motion, read once.
fn contacts(source: &str) -> SweepContacts {
    let e = harness::read(source);
    let swept = harness::solid(&e,"swept");
    SweepContacts::read(&e.sketch,swept,1e-10).unwrap()
}

#[test]
fn a_box_turned_about_its_face_centre_grazes_at_its_two_end_faces() {
    // the 2 x 3 x 2 box turned about the axis along x through (·, -1.5, 0): its two faces square
    // to that axis are carried within their own planes
    let source = format!("{}{}{}",tools::BOX,motions::turn_about(4.,-1.5,5.,-1.5),motions::swept("turn",0.,30.));
    let faces = grazing_faces(&contacts(&source)).unwrap();
    assert_eq!(faces.len(),2,"faces {:?}",faces.iter().map(|f| f.face).collect::<Vec<_>>());
    let mut pivots = Vec::new();
    for f in &faces {
        let PlaneMotion::Turn {pivot,sweep} = f.motion else { panic!("not a turn: {:?}",f.motion) };
        assert!((sweep.abs()-30_f64.to_radians()).abs() < 1e-12,"sweep {sweep}");
        // the pivot is where the turn's axis meets the face, at one end of the box or the other
        let p = f.lift(pivot);
        assert!((p[0]-2.).abs() < 1e-9 || (p[0]-4.).abs() < 1e-9,"pivot at {p:?}");
        // and the face faces away from the box, whose middle is at x = 3
        assert!((f.outward[0].abs()-1.).abs() < 1e-9 && f.outward[0]*(p[0]-3.) > 0.,"outward {:?} at x {}",f.outward,p[0]);
        pivots.push(p);
    }
    // both pivots stand on the one axis, which runs along x
    assert!((pivots[0][1]-pivots[1][1]).abs() < 1e-9 && (pivots[0][2]-pivots[1][2]).abs() < 1e-9
        && ((pivots[0][0]-pivots[1][0]).abs()-2.).abs() < 1e-9,"pivots {pivots:?}");
}

#[test]
fn a_box_slid_grazes_at_the_four_faces_along_the_slide() {
    let source = format!("{}{}{}",tools::BOX,motions::slide_x(10.),motions::swept("feed",0.,360.));
    let faces = grazing_faces(&contacts(&source)).unwrap();
    assert_eq!(faces.len(),4,"faces {:?}",faces.iter().map(|f| f.face).collect::<Vec<_>>());
    for f in &faces {
        let PlaneMotion::Slide {advance} = f.motion else { panic!("not a slide: {:?}",f.motion) };
        assert!((advance[0].hypot(advance[1])-10.).abs() < 1e-9,"advance {advance:?}");
        // a face along the slide faces across it
        assert!(f.outward[0].abs() < 1e-9,"outward {:?}",f.outward);
    }
}

#[test]
fn a_turning_prism_and_a_lens_graze_nowhere() {
    // the prism's own faces stand along the turn's axis, and the lens has no planar face: both
    // keep to the traced sheets
    for source in [format!("{}{}{}",tools::TRIANGLE_PRISM,motions::TURN_OFFSET,motions::swept("turn",-50.,50.)),
        format!("{}{}{}",tools::LENS,motions::TURN_SPINDLE,motions::swept("turn",-60.,60.))] {
        let faces = grazing_faces(&contacts(&source)).unwrap();
        assert!(faces.is_empty(),"{:?}",faces.iter().map(|f| f.face).collect::<Vec<_>>());
    }
}

#[test]
fn the_turned_box_traces_nothing_in_its_grazing_planes() {
    // the tracer leaves the two end faces to their regions: what it emitted there was the bands
    // those faces' edges sweep within the plane, folded at the envelope arc
    let source = format!("{}{}{}",tools::BOX,motions::turn_about(4.,-1.5,5.,-1.5),motions::swept("turn",0.,30.));
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let (_,sheets,faces) = seeds(&e.sketch,swept,0.5,0.02,&|_| {}).unwrap();
    assert_eq!(faces.len(),2);
    for (i,s) in sheets.iter().enumerate() {
        for t in &s.triangles {
            for f in &faces {
                let off = |v: u32| { let p = s.points[v as usize]; (0..3).map(|k| (p[k]-f.origin[k])*f.outward[k]).sum::<f64>().abs() };
                assert!(!t.iter().all(|&v| off(v) < 1e-9),"sheet {i} has a triangle in the plane of face {}",f.face);
            }
        }
    }
}

#[test]
fn the_turned_boxs_own_end_face_sweeps_a_sound_region() {
    // the face the turned box grazes at: y in [-1, 1], z in [-3, 0] about its centre, 30 degrees,
    // at the construction's own sagitta and spacing, with the tracer's nine columns as poses
    let alpha = 30_f64.to_radians();
    let poses: Vec<f64> = (0..9).map(|k| alpha*k as f64/8.).collect();
    let r = swept_region(&rect(-1.,-3.,1.,0.),&[],PlaneMotion::Turn {pivot:[0.,-1.5],sweep:alpha},0.02,0.5,&poses).unwrap();
    assert_eq!(loops(&r),1);
    // it holds the face at both ends, so its area is at least the face's
    assert!(area(&r) > 6.,"area {}",area(&r));
}

// --- milestone 5b: the cases the tracer's closedness and stationing are judged by ---

/// Whether the field itself can supply the surface an unpaired loop wants: for each loop the
/// construction refused, the material's boundary is extracted **within that loop's own box**
/// (`BoundaryOptions::domain`, grown by a sagitta so a patch would overlap the mesh it must be
/// stitched to) and what comes back is reported — nothing is laid, nothing is deleted.
///
/// This is the viability question for a gap fill, and it is the only licensed way to get surface
/// where the field calls a loop `Open`: a contour of the field is the field's own boundary, where
/// anything the stitch invents there is refused by the certificate. The refusal matters as much
/// as the success: `CellBudget` or `ResolutionLimit` says the extraction wants more room to work,
/// while `AmbiguousPoint`, `UnresolvedCell` or `Topology` says the field cannot resolve that
/// neighbourhood at all — and those want opposite answers.
#[test]
#[ignore]
fn what_the_field_yields_in_an_unpaired_loops_own_box() {
    use gcs_core::interval::{Interval as I,minimum::Options};
    use gcs_core::solid::{BoundaryOptions,MaterialField};
    use gcs_core::solid::swept_boundary::{Stage,SweptBoundaryOptions,construct};
    const SAGITTA: f64 = 0.02;
    for (name,source) in [
        ("tilted cylinder 30",format!("{}{}{}",tools::tilted_cylinder(30.),motions::TURN_SPINDLE,motions::swept("turn",-60.,60.))),
        ("thin plate under a roll",format!("{}{}{}",tools::thin_plate(0.1),motions::roll(1.05),motions::swept("turn",0.,120.))),
    ] {
        let e = harness::read(&source);
        let swept = harness::solid(&e,"swept");
        let options = SweptBoundaryOptions {sagitta:SAGITTA,spacing:0.5,..Default::default()};
        // the loops as the zip left them, with the points they stand on
        let mut boxes: Vec<(usize,[f64;3],[f64;3])> = Vec::new();
        let _ = construct(&e.sketch,swept,&options,&|_| {},&mut |stage,_| {
            if let Stage::Zipped {mesh,unpaired,..} = stage {
                for l in unpaired {
                    let (mut lo,mut hi) = ([f64::INFINITY;3],[f64::NEG_INFINITY;3]);
                    for &v in l { let p = mesh.vertices[v as usize];
                        for k in 0..3 { lo[k] = lo[k].min(p[k]); hi[k] = hi[k].max(p[k]); } }
                    boxes.push((l.len(),lo,hi));
                }
            }
        });
        eprintln!("== {name}: {} unpaired loops",boxes.len());
        for (n,lo,hi) in boxes.iter().take(2) {
            let began = std::time::Instant::now();
            let Ok(field) = MaterialField::read(&e.sketch,swept,1e-10) else { continue };
            // the loop's box, grown so a patch would overlap the mesh around it
            let domain: [I;3] = std::array::from_fn(|k| I::new(lo[k]-SAGITTA,hi[k]+SAGITTA).unwrap());
            // Coarse on purpose: this asks only whether the field yields anything here and what
            // it refuses with, and the depth rule (span/2^depth <= tolerance/4) makes a fine
            // tolerance ruinous — 0.02 over a box this size wants depth 7, some two million
            // cells, each costing a field query. A tolerance of a tenth keeps it to a few
            // thousand, and a small cell budget makes a runaway refuse in seconds.
            //
            // **The answer is no, and these settings are not a recommendation.** Three of the
            // four loops come back `AmbiguousPoint` — enclosures straddling zero, e.g. -0.00047
            // to 0.00050 — which is the extractor keeping its own guarantee that an interval
            // containing zero is never a sign: it cannot resolve those neighbourhoods, so a gap
            // fill cannot get licensed surface there. The fourth spent 40 seconds to reach
            // `CellBudget` at this deliberately coarse tenth, so a tolerance fine enough to
            // stitch would cost orders of magnitude more, across dozens of loops. Neither route
            // through the field is open; the missing surface is the tracer's to generate.
            let settings = BoundaryOptions {spatial_tolerance:0.1,max_depth:8,max_cells:20000,
                sweep:Options {value_tolerance:1e-3,max_evaluations:2000},domain:Some(domain)};
            match field.evaluator(0).boundary(settings) {
                Ok(b) => eprintln!("   loop of {n}: {} triangles, shell {}, spatial error {:.6} ({:?})",
                    b.triangles().len(),if b.shell().faces().is_empty() { "empty" } else { "built" },
                    b.spatial_error_bound(),began.elapsed()),
                Err(err) => eprintln!("   loop of {n}: refused {err:?} ({:?})",began.elapsed()),
            }
        }
    }
}

/// What a case's construction comes to, for a case that does not close yet.
fn outcome(source: &str) -> String {
    match super::closed::shell_at(source,0.02) {
        (mesh,Ok(c)) => format!("{} triangles, {} certified, {} thin, {} failed",mesh.triangles.len(),c.certified,c.unresolved.len(),c.failures.len()),
        (mesh,Err(e)) => format!("{} triangles, refused {e:?}",mesh.triangles.len()),
    }
}

#[test]
#[ignore]
fn the_milestone_5b_cases_as_they_stand() {
    for (name,source) in [
        ("tilted cylinder 30",format!("{}{}{}",tools::tilted_cylinder(30.),motions::TURN_SPINDLE,motions::swept("turn",-60.,60.))),
        ("tilted cylinder 85",format!("{}{}{}",tools::tilted_cylinder(85.),motions::TURN_SPINDLE,motions::swept("turn",-60.,60.))),
        ("tilted cylinder 5, slid",format!("{}{}{}",tools::tilted_cylinder(5.),motions::slide_x(6.),motions::swept("feed",0.,360.))),
        ("thin plate under a roll",format!("{}{}{}",tools::thin_plate(0.1),motions::roll(1.05),motions::swept("turn",0.,120.))),
        ("box turned a whole turn",format!("{}{}{}",tools::BOX,motions::turn_about(4.,-1.5,5.,-1.5),motions::swept("turn",0.,360.))),
    ] {
        eprintln!("== {name}: {}",outcome(&source));
    }
}

/// How often a contact curve's closedness flips between neighbouring parameters: the tracer
/// refuses to continue a curve across such a flip (`apart` is infinite when `closed` differs),
/// so the strip is cut there and a one-column strip is dropped altogether.
#[test]
#[ignore]
fn closedness_flickers_between_parameters() {
    use gcs_core::solid::SweepContacts;
    for (name,source) in [
        ("tilted cylinder 30",format!("{}{}{}",tools::tilted_cylinder(30.),motions::TURN_SPINDLE,motions::swept("turn",-60.,60.))),
        ("tilted cylinder 85",format!("{}{}{}",tools::tilted_cylinder(85.),motions::TURN_SPINDLE,motions::swept("turn",-60.,60.))),
        ("thin plate under a roll",format!("{}{}{}",tools::thin_plate(0.1),motions::roll(1.05),motions::swept("turn",0.,120.))),
        ("tumbling cylinder",format!("{}{}{}",tools::CYLINDER,motions::TUMBLE,motions::swept("turn",-30.,30.))),
    ] {
        let e = harness::read(&source);
        let swept = harness::solid(&e,"swept");
        let sweep = SweepContacts::read(&e.sketch,swept,1e-10).unwrap();
        let [from,to] = sweep.domain();
        let steps = 64;
        let mut counts: Vec<(usize,usize)> = Vec::new(); // curves, closed ones
        for i in 0..=steps {
            let t = from+(to-from)*i as f64/steps as f64;
            let curves = sweep.characteristics_over(t,1e-9).unwrap_or_default();
            counts.push((curves.len(),curves.iter().filter(|c| c.closed).count()));
        }
        let flips = counts.windows(2).filter(|w| w[0] != w[1]).count();
        eprintln!("== {name}: {flips} changes over {steps} steps; (curves, closed) {:?}",counts);
    }
}

/// Where the thin plate's contact shatters: the curves at parameters either side of the two
/// places its single closed loop becomes several open pieces, with their lengths and ends.
#[test]
#[ignore]
fn the_thin_plates_contact_shatters_at_two_parameters() {
    use gcs_core::solid::SweepContacts;
    let source = format!("{}{}{}",tools::thin_plate(0.1),motions::roll(1.05),motions::swept("turn",0.,120.));
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let sweep = SweepContacts::read(&e.sketch,swept,1e-10).unwrap();
    let [from,to] = sweep.domain();
    let at = |t: f64| -> String {
        let curves = sweep.characteristics_over(t,1e-9).unwrap_or_default();
        let ends = |c: &gcs_core::solid::Characteristic| { let p = c.points.first().copied().unwrap_or([0.;3]); let q = c.points.last().copied().unwrap_or([0.;3]); format!("{:?}..{:?}",p.map(|x| (x*1e2).round()/1e2),q.map(|x| (x*1e2).round()/1e2)) };
        format!("{} curves: {:?}",curves.len(),curves.iter().map(|c| format!("{} points, closed {}, {}",c.points.len(),c.closed,ends(c))).collect::<Vec<_>>())
    };
    // the first parameter, and either side of it
    for k in [0.,0.25,0.5,1.,2.] {
        let t = from+(to-from)*k/64.;
        eprintln!("== t {k}/64: {}",at(t));
    }
    // and the interior shatter at 46/64
    for k in [45.,45.5,46.,46.5,47.] {
        let t = from+(to-from)*k/64.;
        eprintln!("== t {k}/64: {}",at(t));
    }
}

/// The tilted cylinder's contact curves near the band where its cap's rim has no counterpart on
/// any traced sheet: which curves pass there, at which parameters, and how near they come.
#[test]
#[ignore]
fn the_tilted_cylinders_curves_near_its_unpaired_band() {
    use gcs_core::solid::SweepContacts;
    let source = format!("{}{}{}",tools::tilted_cylinder(30.),motions::TURN_SPINDLE,motions::swept("turn",-60.,60.));
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let sweep = SweepContacts::read(&e.sketch,swept,1e-10).unwrap();
    let [from,to] = sweep.domain();
    // the middle of the band the cap's rim runs through, in world coordinates
    let target = [1.09,-2.47,1.34];
    let steps = 40;
    for i in 0..=steps {
        let t = from+(to-from)*i as f64/steps as f64;
        let pose = sweep.motion().at(t).unwrap();
        let curves = sweep.characteristics_over(t,1e-9).unwrap_or_default();
        let mut best: Vec<String> = Vec::new();
        for (k,c) in curves.iter().enumerate() {
            let d = c.points.iter().map(|p| { let q = pose.point(*p); (0..3).map(|j| (q[j]-target[j]).powi(2)).sum::<f64>().sqrt() }).fold(f64::INFINITY,f64::min);
            if d < 0.25 { best.push(format!("curve {k} ({} points, closed {}) at {d:.4}",c.points.len(),c.closed)); }
        }
        if !best.is_empty() { eprintln!("NEAR t {:+.4} ({i}/{steps}): {}",t,best.join("; ")); }
    }
}

/// Whether the cap contains the sheets' end columns by identity, as the caps are meant to: the
/// sheet's first column beside the cap's own vertices, in the band where the tilted cylinder's
/// rims fail to pair.
#[test]
#[ignore]
fn the_tilted_cylinders_cap_against_its_end_column() {
    use gcs_core::solid::swept_boundary::{SweptBoundaryOptions,caps,seeds};
    let source = format!("{}{}{}",tools::tilted_cylinder(30.),motions::TURN_SPINDLE,motions::swept("turn",-60.,60.));
    let e = harness::read(&source);
    let swept = harness::solid(&e,"swept");
    let options = SweptBoundaryOptions {sagitta:0.02,spacing:0.5,..Default::default()};
    let (_,sheets,_) = seeds(&e.sketch,swept,options.spacing,options.sagitta,&|_| {}).unwrap();
    let caps = caps(&e.sketch,swept,&sheets,options.sagitta,options.snap(),options.spacing,&[]).unwrap();
    let target = [1.09,-2.47,1.34];
    let near = |p: [f64;3]| (0..3).map(|k| (p[k]-target[k]).powi(2)).sum::<f64>().sqrt() < 0.3;
    for (i,s) in sheets.iter().enumerate() {
        let first: Vec<[f64;3]> = s.column_vertices(0).into_iter().map(|v| s.points[v as usize]).filter(|p| near(*p)).collect();
        if !first.is_empty() { eprintln!("COLUMN sheet {i}: {} of its first column in the band, e.g. {:?}",first.len(),first.iter().take(3).map(|p| p.map(|x| (x*1e4).round()/1e4)).collect::<Vec<_>>()); }
    }
    for (k,c) in caps.iter().enumerate() {
        let mine: Vec<[f64;3]> = c.patch.points.iter().copied().filter(|p| near(*p)).collect();
        eprintln!("CAP {k} ({:?}): {} vertices in the band, e.g. {:?}",c.end,mine.len(),mine.iter().take(4).map(|p| p.map(|x| (x*1e4).round()/1e4)).collect::<Vec<_>>());
        // how near each cap vertex in the band comes to any sheet's first column
        let columns: Vec<[f64;3]> = sheets.iter().flat_map(|s| s.column_vertices(0).into_iter().map(|v| s.points[v as usize])).collect();
        let worst = mine.iter().map(|p| columns.iter().map(|q| (0..3).map(|j| (p[j]-q[j]).powi(2)).sum::<f64>().sqrt()).fold(f64::INFINITY,f64::min)).fold(0_f64,f64::max);
        eprintln!("CAP {k}: the farthest of those from any first column is {worst:.6}");
    }
}
