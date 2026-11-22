//! **A ring under any rigid motion** (`brep::swept::rings`, docs/swept-solid-design.md steps 6 and
//! 7 under K2): a tool of revolution clear of its axis — a torus, convex nowhere across its hole —
//! carried by a motion no constant twist takes, its contact read meridian by meridian in closed
//! form and cut where its sheet folds. Held to what is known apart from the road: the torus read as
//! one whole circle; its contact under `swept_tumble.sv`'s tumble two curves at every instant, the
//! outer touching all round and the inner folding four times; and every contact standing on the
//! side of the sweep its `s''` says, read against the ring itself at the instants beside.
use gcs_core::brep::swept::closed::Unbuilt;
use gcs_core::brep::swept::rings::{Corner,Ring,Side,admits,contacts};
use gcs_core::brep::topo::Brep;
use gcs_core::brep::trajectory::Trajectory;
use gcs_core::model::SolidDef;

/// The sweep `name` of `text` as the kernel takes it: its tool's B-rep, its trajectory and its roll.
fn sweep(text: &str,name: &str) -> (Brep,Trajectory,[f64;2]) {
    let e = fixtures::read(text);
    let sk = &e.sketch;
    let swept = fixtures::solid(&e,name);
    let SolidDef::Swept {source,motion,..} = &sk.solids[swept].def else { panic!("`{name}` is a sweep") };
    let path = gcs_core::motion::Family::read(sk,*motion as usize).unwrap().trajectory().unwrap();
    let roll = gcs_core::solid::roll::interval(sk,swept).unwrap();
    let (path,roll) = path.unpaced(roll).unwrap();
    let tool = gcs_core::brep::recipe::build(&gcs_core::solid::cad::recipe(sk,*source as usize).unwrap()).unwrap();
    (tool,path.clone(),roll)
}

/// `swept_tumble.sv`'s torus and tumble.
fn tumble() -> (Brep,Trajectory,[f64;2]) { sweep(include_str!("../../examples/swept_tumble.sv"),"tumbled") }

/// **A torus is a ring**: its one face its meridian's whole circle, turning counter-clockwise, no
/// corner, as thick as its tube; a tool reaching its axis (a bull-nose) is none, refused by name.
#[test]
fn a_torus_reads_as_a_ring_and_a_tool_on_its_axis_does_not() {
    let torus = gcs_core::brep::build::torus([1.,2.,3.],[0.,0.,1.],2.,0.5).unwrap();
    let ring = Ring::read(&torus).unwrap();
    assert_eq!(ring.elements.len(),1);
    let gcs_core::brep::swept::patches::Element::Arc {c,r,sweep,..} = ring.elements[0] else { panic!("{:?}",ring.elements) };
    assert!((c[0]-2.).abs() < 1e-9 && (r-0.5).abs() < 1e-9 && (sweep-std::f64::consts::TAU).abs() < 1e-9,"{:?}",ring.elements);
    assert_eq!(ring.corners,vec![Corner::Smooth]);
    assert!((ring.thickness-1.).abs() < 1e-6,"{}",ring.thickness);
    let bull = crate::swept_closed::bull_nose(0.,1.,0.3,1.);
    assert!(matches!(Ring::read(&bull),Err(Unbuilt::Curved {..})));
}

/// **The tumbling torus's contact, read meridian by meridian**: two curves at every instant over
/// the roll (the road admits it, every instant's pieces the first's) — the outer touching all
/// round, the inner folding four times: two stretches touching, two buried, and a bridge across
/// each fold between them.
#[test]
fn the_tumbling_torus_touches_along_two_curves_one_folding_four_times() {
    let (tool,path,roll) = tumble();
    admits(&tool,&path,roll).unwrap();
    let ring = Ring::read(&tool).unwrap();
    for t in [roll[0],0.,roll[1]] {
        let curves = contacts(&ring,&path,roll,t,1).unwrap();
        assert_eq!(curves.len(),2,"at {t}");
        let count = |c: &[([f64;3],Side)],s: Side| c.iter().filter(|p| p.1 == s).count();
        let (outer,inner) = if count(&curves[0],Side::Fold) == 0 { (&curves[0],&curves[1]) } else { (&curves[1],&curves[0]) };
        assert!(outer.iter().all(|p| p.1 == Side::Touching),"at {t}: {outer:?}");
        assert_eq!([count(inner,Side::Touching),count(inner,Side::Fold),count(inner,Side::Buried)],[2,4,2],"at {t}: {inner:?}");
    }
}

/// **Each contact stands on the side of the sweep its `s''` says**: at three rolls, every point of a
/// touching piece is held by the ring at no instant a little before or after (the sweep's boundary
/// there, short of what else holds it), and every point of a buried one by the ring at both (inside
/// the sweep) — read against the ring itself, carried back from those instants.
#[test]
fn each_contact_is_held_beside_its_instant_as_its_bend_says() {
    let (tool,path,roll) = tumble();
    let ring = Ring::read(&tool).unwrap();
    let d = 0.03;
    let mut read = 0;
    for t in [-1.,0.,1.] {
        let beside = [t-d,t+d].map(|s| path.pose(s).unwrap().inverse());
        for c in contacts(&ring,&path,roll,t,24).unwrap() {
            for (p,side) in c {
                let held = beside.map(|m| ring.holds(m.point(p)));
                match side {
                    Side::Touching => assert!(!held[0] && !held[1],"at {t}: a touching contact {p:?} held beside it: {held:?}"),
                    Side::Buried => assert!(held[0] && held[1],"at {t}: a buried contact {p:?} not held beside it: {held:?}"),
                    Side::Fold => continue,
                }
                read += 1;
            }
        }
    }
    assert_eq!(read,360,"contacts read");
}

/// **A tube that folds is entered where it stands outside the tool at both end poses**
/// (`rings::trimmed`): entered from its rim, the tumbling torus's inner tube lay over its own rim
/// beside its fold corners where it starts — a meeting of the tube with itself, which left the
/// torus's face a slit and the sweep's inside one cell with its outside. Trimmed, no point of either
/// tube stands inside the torus where it starts or where it stops, and every edge of each is
/// shared by two of its faces or lies on the torus at an end pose: a touching rim, or a cut. And the
/// tubes are no finer than they need be.
#[test]
fn a_tube_that_folds_is_entered_where_it_stands_outside_the_tool() {
    use gcs_core::brep::query::{Located,Place};
    use gcs_core::brep::swept::rings::{entered,fitted,posed};
    use gcs_core::brep::topo::EdgeCurve;
    let (tool,path,roll) = tumble();
    let ring = Ring::read(&tool).unwrap();
    let fit = fitted(&ring,&path,roll,1e-6).unwrap();
    // refined only where they miss (each interval halved where the net misses at its middle): the
    // two tubes in some fourteen thousand poles, where halving every interval at once made 170 840
    let poles: usize = fit.nets.iter().map(|n| n.poles.len()*n.poles[0].len()).sum();
    assert!(poles < 20_000,"{poles} poles");
    let [start,end] = roll.map(|t| posed(&tool,&ring,&path,&fit,t).unwrap());
    let poses = roll.map(|t| path.pose(t).unwrap());
    let (sheets,_) = entered(&fit,&ring,[&start,&end],poses,1e-5).unwrap();
    assert_eq!(sheets.len(),2,"a tube for each contact curve");
    // the tool's signed distance at each end pose, at a point in space
    let at = |e: usize,x: [f64;3]| ring.signed(poses[e].inverse().point(x)).0;
    let on_tool = |x: [f64;3]| (0..2).any(|e| at(e,x).abs() <= 1e-4);
    for (k,sheet) in sheets.iter().enumerate() {
        let uses = sheet.uses();
        for (i,e) in sheet.edges.iter().enumerate() {
            if uses[i].len() == 2 { continue }
            let EdgeCurve::Curve(c) = &e.curve else { panic!("tube {k}: edge {i} has no curve") };
            assert!(uses[i].len() == 1 && [0.,0.5,1.].iter().all(|&f| on_tool(c.point(e.t[0]+(e.t[1]-e.t[0])*f))),
                "tube {k}: edge {i} is used {} times and stands off the tool at an end pose",uses[i].len());
        }
        let located = Located::new(sheet,1e-5);
        let mut read = 0;
        for fi in 0..sheet.faces.len() {
            let [[u0,u1],[v0,v1]] = gcs_core::brep::boolean::face_parameters(sheet,fi);
            for i in 1..12 { for j in 1..12 {
                let p = sheet.faces[fi].surface.point([u0+(u1-u0)*i as f64/12.,v0+(v1-v0)*j as f64/12.]);
                if located.face_place(fi,p) != Place::In { continue }
                read += 1;
                for e in 0..2 { assert!(at(e,p) >= -1e-4,"tube {k}, face {fi}: {p:?} stands {:.2e} inside the tool at end pose {e}",at(e,p)) }
            } }
        }
        assert!(read > 50*sheet.faces.len(),"tube {k}: {read} points read");
    }
    // the inner tube, folding four times, cut: more faces than its twelve pieces
    assert!(sheets[1].faces.len() > 12,"{}",sheets[1].faces.len());
}

/// **The tumbling torus built closed** (the slow tier): the tubes fit (at the build's tolerance of
/// 1e-5, each interval refined only where it misses: 190 × 45 and 166 × 34 points, in a sixth of a
/// second), the inner one cut to
/// where it stands outside the torus where it starts and stops, the box split by them and the torus
/// at both end poses at once into six cells (in some eleven seconds) and classified (in under a
/// second), the selected solid checked and judged by the definition. Its volume made apart from
/// any kernel (#880 E2: the divergence theorem over the ribbon's offset sheets and the end poses,
/// each kept where membership says) is 25.2750727653, where cadrs's is within 1.1e-8;
/// the solid's own, read off its fitted faces, is 25.2750394, 3.3e-5 low (1.3e-6) at any fit
/// tolerance from 1e-5 to 1e-7, and its area 5.6e-4 high. Four of its dipped stretches
/// (`rings::dip`) stand wholly inside the sweep beside the end poses, up to 3.6e-3 deep, and bound
/// pockets of it the solid leaves out (#880 E2, R1's to mend). Built here, from the drawing, after
/// 0.6.3, it is 25.2750184, 5.4e-5 low; the bench's spelling of the same motion is held below
/// (`the_tumbling_torus_as_the_bench_spells_it_is_built_closed`). Held here within 1e-4 of the
/// volume made apart, to be tightened to 1e-7 when the pockets are mended. (A Monte Carlo of the
/// definition, 80 million points, read 25.260 ± 0.005: three of its deviations low, which the
/// volume made apart settles. Read point by point, 100 000 points apart, the pockets are too thin
/// to be met.)
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 15 s: the tumbling torus built closed")]
fn the_tumbling_torus_is_built_closed() {
    let (tool,path,roll) = tumble();
    let built = gcs_core::brep::swept::closed::ring(&tool,&path,roll,0.002,1e-5).unwrap_or_else(|u| panic!("{u:?}"));
    let v = gcs_core::brep::props::volume(&built);
    assert!((v-25.2750727653).abs() < 1e-4,"{v}");
}

/// **The tumbling torus as brep-bench spells it** (`swept/tumbling-torus`; #880 R1): the same torus
/// and roll, its motion two turns read as `Trajectory::Relative` — turned by t/4 about the vertical
/// line through (2, 0, 0), seen from a frame turned by t about the line through (0, 0.5, 0.5) along
/// x — not through the document. At the start pose a circle of the torus meets the inner tube's
/// fitted sheet flatly, where the sheet touches the torus along its rim: the fit crosses the circle
/// twice within the tolerance of the touch, and a search proving each crossing (restricted Bézier
/// enclosures, #873) found the one inside the sheet's domain, 2.7e-4 off, so the rim's crossing of
/// the circle and the root were two vertices and the tube's face did not close ("a bspline's pieces
/// do not close", cad-kernel 0.7.0). The root is that crossing, found beside the circle where the
/// rim lies along its face (`flat_roots`). Built, it is the solid 0.6.3 built, its volume held to
/// the one made apart within 1e-4 as the drawing's is (its pockets are R1's). In the fast tier
/// (about 5 s), so a change to how a Boolean reads a fitted sheet meets it on every push.
#[test]
fn the_tumbling_torus_as_the_bench_spells_it_is_built_closed() {
    use gcs_core::brep::trajectory::Screw;
    let tool = gcs_core::brep::build::torus([3.,0.,2.],[0.,0.,1.],1.,0.5).unwrap();
    let own = Trajectory::Twist(Screw {origin:[2.,0.,0.],axis:[0.,0.,1.],ratio:0.25,advance:0.});
    let frame = Trajectory::Twist(Screw {origin:[0.,0.5,0.5],axis:[1.,0.,0.],ratio:1.,advance:0.});
    let path = Trajectory::Relative {source:Box::new(own),observer:Box::new(frame)};
    let roll = [-75f64.to_radians(),75f64.to_radians()];
    let built = gcs_core::brep::swept::closed::ring(&tool,&path,roll,0.002,1e-5).unwrap_or_else(|u| panic!("{u:?}"));
    let v = gcs_core::brep::props::volume(&built);
    assert!((v-25.2750727653).abs() < 1e-4,"{v}");
}

/// **The tumbling torus is built at every spelling of its motion** (#942; the slow tier, about a
/// minute): brep-bench's 80° torus built only where its observer's origin was (0, 0.5, 0.5)
/// exactly. Each end pose's seam stands in the widest gap among the touched points' angles round
/// its axis (`closed::seam_away`), and those points stand evenly round it, every gap alike within
/// 1e-14: the gap is chosen by rounding, and any other spelling of the motion stood the start
/// torus's seam elsewhere. Where it crossed the touch just past a corner of the inner tube's
/// pieces, a short edge of that touch had the tube's fitted sheet leaning 2e-4 into the torus at
/// first order — past `TANGENT` — and was ordered so, while the rest of the touch was ordered by a
/// step in the other way: three cells one, a whole region of the sweep missing, and the judge
/// refused it. Two ways in that a step in turns the other way round, having stood apart by no
/// more than the tolerance, are tangent at it and ordered by the step in (`complex::turned_back`;
/// the kernel gate `arrangement::a_sheet_tangent_to_a_face_at_the_tolerance_is_ordered_round_its_rim_by_a_step_in`).
/// Built at the origin exactly, at `√½ (0, cos 45°, sin 45°)` (an ulp off in `y`), at 0.7071 as a
/// decimal, at the origin moved by 1e-6 in `z`, at the roll's end 1e-6° on, its axis leaning 1e-6
/// and its own turn an ulp faster: each checked at the tolerance and its volume the one made apart
/// within 1e-4, as the bench's spelling is held above, and those an ulp off the exact spelling's
/// within the tolerance's worth, the solid's area times the tolerance (measured 1.3e-8 at most;
/// the ones moved by 1e-6 move it by as much as 1.9e-5, their own).
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about a minute and a half: the tumbling torus built at eight spellings")]
fn the_tumbling_torus_is_built_at_every_spelling_of_its_motion() {
    use gcs_core::brep::trajectory::Screw;
    let tool = gcs_core::brep::build::torus([3.,0.,2.],[0.,0.,1.],1.,0.5).unwrap();
    let h = 0.5f64.sqrt();
    let (s45,c45) = 45f64.to_radians().sin_cos();
    let up = |x: f64| f64::from_bits(x.to_bits()+1);
    // (the observer's origin, its axis, the own turn's ratio, the roll's end in degrees, and whether
    // it is an ulp off the exact spelling)
    let spellings: Vec<([f64;3],[f64;3],f64,f64,bool)> = vec![
        ([0.,0.5,0.5],[1.,0.,0.],0.25,75.,true),
        ([0.,h*c45,h*s45],[1.,0.,0.],0.25,75.,true),
        ([0.,0.7071*c45,0.7071*s45],[1.,0.,0.],0.25,75.,false),
        ([0.,up(0.5),0.5],[1.,0.,0.],0.25,75.,true),
        ([0.,0.5,0.500001],[1.,0.,0.],0.25,75.,false),
        ([0.,0.5,0.5],[1.,0.,0.],0.25,75.000001,false),
        ([0.,0.5,0.5],gcs_core::space::normalised([1.,0.,1e-6]).unwrap(),0.25,75.,false),
        ([0.,0.5,0.5],[1.,0.,0.],up(0.25),75.,true),
    ];
    let mut exact = None;
    for (origin,axis,ratio,end,ulp) in spellings {
        let own = Trajectory::Twist(Screw {origin:[2.,0.,0.],axis:[0.,0.,1.],ratio,advance:0.});
        let frame = Trajectory::Twist(Screw {origin,axis,ratio:1.,advance:0.});
        let path = Trajectory::Relative {source:Box::new(own),observer:Box::new(frame)};
        let roll = [-75f64.to_radians(),end.to_radians()];
        let built = gcs_core::brep::swept::closed::ring(&tool,&path,roll,0.002,1e-5)
            .unwrap_or_else(|u| panic!("observer at {origin:?} along {axis:?}, ratio {ratio}, roll to {end}°: {u:?}"));
        built.check(1e-5).unwrap_or_else(|e| panic!("observer at {origin:?}: {e:?}"));
        let v = gcs_core::brep::props::volume(&built);
        assert!((v-25.2750727653).abs() < 1e-4,"observer at {origin:?}: {v}");
        let v0 = *exact.get_or_insert(v);
        let worth = 1e-5*built.faces.iter().map(|f| gcs_core::brep::props::area(&built,f)).sum::<f64>();
        assert!(!ulp || (v-v0).abs() <= worth,"observer at {origin:?} along {axis:?}, ratio {ratio}, roll to {end}°: {v} against {v0}");
    }
}

/// **The tumbling torus is built wherever its start pose's seam stands** (#942; the slow tier,
/// about a minute): the torus where the roll starts turned about its own axis — the same solid —
/// so its seam meridian stands at an angle of its own choosing (the seam vertex's, round the axis
/// from `Frame::about`'s `x`), and the rest built as the ring road builds it (`closed::ring_from`).
/// Between about −2.745 and −2.684 the seam crossed the start torus's touch just past a corner of
/// the inner tube's pieces, and the short edge of the touch between them had the tube's fitted
/// sheet leaning 2e-4 into the torus at first order while the rest leant 3e-6: ordered at first
/// order there and by a step in along the rest, the two ways round, three cells were one and the
/// build was refused (`complex::turned_back`). Built at angles before, across and after that
/// window, each checked at the tolerance, its volume the one made apart within 1e-4 and the
/// first's within the tolerance's worth, the solid's area times the tolerance (measured 4e-10
/// apart).
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about a minute: the tumbling torus built at five seams")]
fn the_tumbling_torus_is_built_wherever_its_start_poses_seam_stands() {
    use gcs_core::brep::geom::{Frame,Rigid,Surface};
    use gcs_core::brep::swept::closed::{ring_entered,ring_from};
    use gcs_core::brep::trajectory::Screw;
    let tool = gcs_core::brep::build::torus([3.,0.,2.],[0.,0.,1.],1.,0.5).unwrap();
    let own = Trajectory::Twist(Screw {origin:[2.,0.,0.],axis:[0.,0.,1.],ratio:0.25,advance:0.});
    let frame = Trajectory::Twist(Screw {origin:[0.,0.5,0.5],axis:[1.,0.,0.],ratio:1.,advance:0.});
    let path = Trajectory::Relative {source:Box::new(own),observer:Box::new(frame)};
    let roll = [-75f64.to_radians(),75f64.to_radians()];
    let mut first = None;
    for seam in [-2.78,-2.73,-2.71,-2.69,-2.65] {
        let mut entered = ring_entered(&tool,&path,roll,1e-5).unwrap();
        let start = &entered.ends[0];
        let Surface::Torus(f,..) = &start.faces[0].surface else { panic!("the start pose a torus") };
        let about = Frame::about(f.o,f.z);
        let l = about.local(start.vertices[0].p);
        let turned = start.moved(&Rigid::turn(f.o,f.z,seam-l[1].atan2(l[0])));
        entered.ends[0] = turned;
        let built = ring_from(&tool,&path,roll,0.002,1e-5,entered).unwrap_or_else(|u| panic!("its seam at {seam}: {u:?}"));
        built.check(1e-5).unwrap_or_else(|e| panic!("its seam at {seam}: {e:?}"));
        let v = gcs_core::brep::props::volume(&built);
        assert!((v-25.2750727653).abs() < 1e-4,"its seam at {seam}: {v}");
        let worth = 1e-5*built.faces.iter().map(|f| gcs_core::brep::props::area(&built,f)).sum::<f64>();
        let v0 = *first.get_or_insert(v);
        assert!((v-v0).abs() <= worth,"its seam at {seam}: {v} against {v0}");
    }
}

/// The closed polygon through `points` (in a plane), as a profile.
pub(crate) fn polygon(points: &[[f64;3]]) -> gcs_core::brep::build::Profile {
    use gcs_core::brep::build::{Profile,ProfileEdge};
    let n = points.len();
    let (u,v) = ([points[1][0]-points[0][0],points[1][1]-points[0][1],points[1][2]-points[0][2]],
        [points[2][0]-points[0][0],points[2][1]-points[0][1],points[2][2]-points[0][2]]);
    let normal = gcs_core::space::normalised([u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]]).unwrap();
    Profile {origin:points[0],normal,loops:vec![(0..n).map(|k| ProfileEdge::Line {a:points[k],b:points[(k+1)%n]}).collect()],names:Vec::new()}
}

/// **A ring pushed into a block at a lean, cut from it** (`closed::ring_cut_from`; brep-bench's
/// `swept/ring-pushed-in-at-a-lean`): the torus about `(sin 35°, 0, cos 35°)` through (0, 0, 4), of
/// radii 3 and 1, slid straight down 8 into the box (−8, −8, −8)–(8, 8, 0). The blank is split by
/// the ring's candidates at once, as `cut_from` splits it by any other tool's, and kept where the
/// sweep is not. What is left, by a Monte Carlo of the definition apart from the kernel (2 million
/// points of the box, each read against the torus's exact distance at 16 001 places along the push,
/// none beside where no place of 321 comes within the push's reach): 1881.39 ± 0.40.
#[test]
fn a_ring_pushed_in_at_a_lean_is_cut_from_a_block() {
    use gcs_core::brep::build::{prism,square,torus};
    use gcs_core::brep::trajectory::Screw;
    let block = prism(&square([0.,0.,-8.],[0.,0.,1.],8.),0.,8.).unwrap();
    let lean = 35f64.to_radians();
    let ring = torus([0.,0.,4.],[lean.sin(),0.,lean.cos()],3.,1.).unwrap();
    // the push down: a screw that does not turn, advancing 8 over a roll of τ
    let path = Trajectory::Twist(Screw {origin:[0.,0.,4.],axis:[0.,0.,-1.],ratio:0.,advance:8.});
    let built = gcs_core::brep::swept::closed::ring_cut_from(&block,&ring,&path,[0.,std::f64::consts::TAU],0.01,1e-5)
        .unwrap_or_else(|u| panic!("{u:?}"));
    let v = gcs_core::brep::props::volume(&built);
    assert!((v-1881.39).abs() < 1.6,"{v}");
}

/// **A torus turned across a curved block, cut from it** (the slow tier; brep-bench's
/// `swept/torus-turned-across-block`): the block everything 8 to 12 from the z axis, with z within 1
/// and polar angle within 0°–60°; the torus about `(0, sin 25°, cos 25°)` through (5, 0, 0), of radii
/// 2.5 and 2, turned about the z axis from −140° to 140°. Its axis skew to the turn's, its contact
/// is read meridian by meridian as a tumble's is. What is left, by a Monte Carlo of the definition
/// apart from the kernel (2 million points of the block, each read against the torus's exact
/// distance at 28 001 turns, none beside where no turn of 561 comes within the turn's reach):
/// 57.566 ± 0.028.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 17 s: a torus turned across a block, cut from it")]
fn a_torus_turned_across_a_block_is_cut_from_it() {
    use gcs_core::brep::build::{revolve,torus};
    use gcs_core::brep::trajectory::Screw;
    let block = revolve(&polygon(&[[8.,0.,-1.],[12.,0.,-1.],[12.,0.,1.],[8.,0.,1.]]),[0.,0.,0.],[0.,0.,1.],60f64.to_radians()).unwrap();
    let lean = 25f64.to_radians();
    let cutter = torus([5.,0.,0.],[0.,lean.sin(),lean.cos()],2.5,2.).unwrap();
    // the turn about z: a screw that does not advance
    let path = Trajectory::Twist(Screw {origin:[0.,0.,0.],axis:[0.,0.,1.],ratio:1.,advance:0.});
    let built = gcs_core::brep::swept::closed::ring_cut_from(&block,&cutter,&path,[-140f64.to_radians(),140f64.to_radians()],0.01,1e-5)
        .unwrap_or_else(|u| panic!("{u:?}"));
    let v = gcs_core::brep::props::volume(&built);
    assert!((v-57.566).abs() < 0.1,"{v}");
}

/// **A tube's chord given as its meeting with the tool is its own edge, laid once** (#942 F3's
/// class B gate, `boolean`'s given meetings): the ring of brep-bench's `ring-along-a-spline`, a
/// torus carried along a cubic in the plane, its two tubes entered where they cross the torus at
/// each end pose (`rings::entered`), each cut along that crossing and given it — a span of a
/// curve of several spans — as its meeting with the torus there. The tube has that chord as an
/// edge already, the very curve: arranged in the stock with the torus at the pose it ends on, each
/// tube is split into cells, where laying the chord a second time on itself left the tube's face
/// open ("a bspline's pieces do not close", "a bspline has a hole in no piece").
#[test]
fn a_tubes_chord_given_as_its_meeting_with_the_tool_is_its_own_edge_laid_once() {
    use gcs_core::brep::boolean::arrangement_given;
    use gcs_core::brep::swept::rings::{entered,fitted,posed};
    use gcs_core::brep::topo::EdgeCurve;
    use gcs_core::brep::trajectory::{Motion,Path};
    /// The frame along a curve in the plane `z = 0`: its `x` along the curve's tangent, its `z` up.
    #[derive(Debug)]
    struct Along(gcs_core::brep::nurbs::BSpline);
    impl Path for Along {
        fn at(&self,t: f64) -> Result<Motion,gcs_core::brep::KernelError> {
            let (point,d,dd) = self.0.d2(t);
            let speed2 = d[0]*d[0]+d[1]*d[1];
            let (c,s) = (d[0]/speed2.sqrt(),d[1]/speed2.sqrt());
            let turning = (d[0]*dd[1]-d[1]*dd[0])/speed2;
            let r = [[c,-s,0.],[s,c,0.],[0.,0.,1.]];
            let dr = [[-s*turning,-c*turning,0.],[c*turning,-s*turning,0.],[0.;3]];
            Ok(Motion::of_parts(r,dr,point,d))
        }
    }
    let stock = crate::brep::block([-6.,-6.,-8.],[36.,26.,0.]);
    let tool = gcs_core::brep::build::torus([0.,0.,0.5],[0.,1.,0.],1.5,0.5).unwrap();
    let curve = gcs_core::brep::nurbs::BSpline::new(3,vec![0.,0.,0.,0.,1.,1.,1.,1.],
        vec![[0.,0.,0.],[10.,0.,0.],[20.,20.,0.],[30.,20.,0.]]).unwrap();
    let path = Trajectory::General(std::sync::Arc::new(Along(curve)));
    let tol = gcs_core::brep::geom::tolerance(stock.size().max(tool.size()));
    let ring = Ring::read(&tool).unwrap();
    // (the build's fit, a tenth of its tolerance: the contact keeps its shape over the roll, and
    // its tubes cross themselves nowhere)
    let fit = fitted(&ring,&path,[0.,1.],0.1*tol).unwrap();
    assert!(fit.seeds.is_empty(),"no narrow pair's leaves cross");
    let ends = [0.,1.].map(|t| posed(&tool,&ring,&path,&fit,t).unwrap());
    let poses = [0.,1.].map(|t| path.pose(t).unwrap());
    let (tubes,given) = entered(&fit,&ring,[&ends[0],&ends[1]],poses,tol).unwrap();
    assert_eq!(tubes.len(),4,"a tube for each contact curve, and a strip for each meridian touched whole after them");
    for (k,tube) in tubes.iter().take(2).enumerate() {
        // a curve given is a span of several of a curve that is one of the tube's edges
        let own = |e: &gcs_core::brep::topo::Edge| given.iter().flat_map(|g| &g.curves)
            .any(|c| matches!(&e.curve,EdgeCurve::Curve(ec) if ec == c));
        assert!(tube.edges.iter().any(|e| own(e) && e.t[0] > 0.5),
            "tube {k}: no chord given that is a later span of its own edge");
        let complex = arrangement_given(&stock,&[&ends[k],tube],&given,tol)
            .unwrap_or_else(|e| panic!("tube {k} with the torus where it ends: {e}"));
        let cells = complex.cells().unwrap().len();
        assert!(cells >= 2,"tube {k}: {cells} cells");
        // and each chord laid once: no two of the complex's edges on one curve over one stretch
        let es = &complex.b.edges;
        for (i,e) in es.iter().enumerate() {
            let EdgeCurve::Curve(c) = &e.curve else { continue };
            let [a0,a1] = [e.t[0].min(e.t[1]),e.t[0].max(e.t[1])];
            assert!(!es[i+1..].iter().any(|f| matches!(&f.curve,EdgeCurve::Curve(d) if d == c)
                && f.t[0].min(f.t[1]) < a1 && f.t[0].max(f.t[1]) > a0),
                "tube {k}: edge {i}'s curve laid twice over one stretch");
        }
    }
}

/// brep-bench's `tumbling-torus-half-turn`: a torus of tube 0.16 turned 0.375 a radian about one
/// line and back a radian about another over half a turn, its roll and tolerance.
fn half_turn() -> (Brep,Trajectory,[f64;2],f64) {
    use gcs_core::brep::trajectory::Screw;
    let tool = gcs_core::brep::build::torus([3.,0.,2.],[0.,0.,1.],1.,0.16).unwrap();
    let (sl,cl) = 30f64.to_radians().sin_cos();
    let (sa,ca) = 20f64.to_radians().sin_cos();
    let own = Trajectory::Twist(Screw {origin:[2.,0.,0.],axis:[0.,0.,1.],ratio:0.375,advance:0.});
    let frame = Trajectory::Twist(Screw {origin:[0.,0.7071*cl,0.7071*sl],axis:[ca,sa*cl,sa*sl],ratio:1.,
        advance:0.});
    let path = Trajectory::Relative {source:Box::new(own),observer:Box::new(frame)};
    (tool,path,[-90f64.to_radians(),90f64.to_radians()],0.16/50_000.)
}

/// brep-bench's `ring-carried-along-a-spline` as the kernel takes it: the block (−6, −6, −8)–(36,
/// 26, 0), the torus about `y` through (0, 0, 0.5) of radii 1.5 and 0.5, its frame carried along a
/// cubic in the plane `z = 0` (`trajectory::Along`, the rotation-minimising frame) over its whole
/// parameter, and the tolerance it is built to.
fn along_a_spline() -> (Brep,Brep,Trajectory,f64) {
    let block = crate::brep::block([-6.,-6.,-8.],[36.,26.,0.]);
    let tool = gcs_core::brep::build::torus([0.,0.,0.5],[0.,1.,0.],1.5,0.5).unwrap();
    let curve = gcs_core::brep::nurbs::BSpline::new(3,vec![0.,0.,0.,0.,1.,1.,1.,1.],
        vec![[0.,0.,0.],[10.,0.,0.],[20.,20.,0.],[30.,20.,0.]]).unwrap();
    let path = Trajectory::General(std::sync::Arc::new(gcs_core::brep::trajectory::Along::new(curve).unwrap()));
    (block,tool,path,1e-5)
}

/// The bench runner's frame along its cubic: its `x` along the curve's tangent, its `z` up; a path
/// read only by evaluating it.
fn runner() -> Trajectory {
    #[derive(Debug)]
    struct Runner(gcs_core::brep::nurbs::BSpline);
    impl gcs_core::brep::trajectory::Path for Runner {
        fn at(&self,t: f64) -> Result<gcs_core::brep::trajectory::Motion,gcs_core::brep::KernelError> {
            let (point,d,dd) = self.0.d2(t);
            let speed2 = d[0]*d[0]+d[1]*d[1];
            let (c,s) = (d[0]/speed2.sqrt(),d[1]/speed2.sqrt());
            let turning = (d[0]*dd[1]-d[1]*dd[0])/speed2;
            let r = [[c,-s,0.],[s,c,0.],[0.,0.,1.]];
            let dr = [[-s*turning,-c*turning,0.],[c*turning,-s*turning,0.],[0.;3]];
            Ok(gcs_core::brep::trajectory::Motion::of_parts(r,dr,point,d))
        }
    }
    let curve = gcs_core::brep::nurbs::BSpline::new(3,vec![0.,0.,0.,0.,1.,1.,1.,1.],vec![[0.,0.,0.],[10.,0.,0.],[20.,20.,0.],[30.,20.,0.]]).unwrap();
    Trajectory::General(std::sync::Arc::new(Runner(curve)))
}

/// The bench's ring along a spline as its runner builds it: `closed::cut_from`, the runner's own
/// frame (its `x` along the curve's tangent, its `z` up), a bar of 0.005, the tolerance of its size;
/// checked closed. The cut and its path.
fn cut_by_the_runner() -> (Brep,Trajectory) {
    let (block,tool,..) = along_a_spline();
    let runner = runner();
    let tol = gcs_core::brep::geom::tolerance(block.size().max(tool.size()));
    let cut = gcs_core::brep::swept::closed::cut_from(&block,&tool,&runner,[0.,1.],0.005,tol).unwrap_or_else(|e| panic!("{e:?}"));
    cut.check(tol).unwrap();
    (cut,runner)
}

/// `n` seeded points over the box about the ring's groove read against the sweep's membership
/// along the runner's frame: each settled point clear of the cut's boundary is a member exactly
/// where the cut does not hold it (all of the box is in the block). Where `count` is given (the
/// block's volume), the groove's volume — the box's share of members — is held within three
/// standard errors of the block's less the cut's.
fn groove_counted(cut: &Brep,n: usize,count: Option<f64>) {
    let (_,tool,..) = along_a_spline();
    let runner = runner();
    let solid = gcs_core::brep::swept::member::Solid::new(&tool,0.002,1.).unwrap();
    let swept = gcs_core::brep::swept::member::Swept {sampled:true,tool:&solid,path:&runner,roll:[0.,1.]};
    let body = gcs_core::brep::swept::member::Solid::new(cut,0.002,1.).unwrap();
    let (lo,hi) = ([-2.,-2.,-1.5],[32.,22.,0.]);
    let mut rng = gcs_core::rng::Rng::new(9);
    let (mut members,mut held) = (0usize,0usize);
    for _ in 0..n {
        let p: [f64;3] = std::array::from_fn(|k| rng.uniform(lo[k],hi[k]));
        let Ok(member) = swept.member(p) else { continue };
        held += 1;
        if member { members += 1 }
        let d = body.signed_distance(p);
        if d.abs() < 0.01 { continue }
        assert_eq!(member,d > 0.,"at {p:?}: membership {member}, the cut's distance {d}");
    }
    assert!(held+n/100 >= n,"{held} of {n} settled");
    let Some(block) = count else { return };
    let boxed = (0..3).map(|k| hi[k]-lo[k]).product::<f64>();
    let f = members as f64/held as f64;
    let (counted,error) = (boxed*f,boxed*(f*(1.-f)/held as f64).sqrt());
    let groove = block-gcs_core::brep::props::volume(cut);
    assert!((counted-groove).abs() <= 3.*error,"the groove counted {counted} ± {error}, against {groove}");
    eprintln!("the groove counted {counted} ± {error}, against {groove}");
}

/// **The ring's groove counted** (#942, the depth trim): forty thousand seeded points over the box
/// about the groove, each settled point a member of the sweep exactly where the cut does not hold
/// it, the members' share of the box the groove's volume within three standard errors of the
/// block's less the cut's.
#[test]
#[cfg_attr(not(feature = "slow"),ignore = "slow tier, about a minute: forty thousand points read for membership")]
fn the_rings_groove_counted_is_the_blocks_volume_less_the_cuts() {
    let (cut,_) = cut_by_the_runner();
    groove_counted(&cut,40_000,Some(42.*32.*8.));
}

/// **A ring carried along a spline touches its top and bottom meridians whole at every instant,
/// and is cut from a block as the bench builds it** (#942, the depth trim): along a path in a plane
/// the torus's axis lies in, the velocity at each point of the meridian through the path's normal
/// lies in the meridian's own plane, so `n · v` is nought all round it, at every instant. Its sweep
/// is entered by depth (`swept::touched`): of each parallel's sheet the leaf depth keeps, cut at the
/// meridian; the strip the bottom meridian's lower half sweeps; and where the roll starts and
/// stops, the other leaf from its rim to where it crosses the tool again — every meeting given.
/// Built through `closed::cut_from` with the runner's own frame, a bar of 0.005 and the tolerance
/// of its size: closed, its volume 10696.461766 (the peer kernel's 10696.4618; the block alone is
/// 10752), five B-spline faces and the block's six planes, and held to membership at points over
/// the groove — each a member of the sweep exactly where it is in the block and not in the cut,
/// and the groove's volume counted by them within three standard errors of the block's less the
/// cut's. Without a blank to cut from it is still refused by name, at a meridian touched whole.
#[test]
fn a_ring_carried_along_a_spline_touching_its_meridians_whole_is_cut_from_a_block() {
    let (block,tool,path,_) = along_a_spline();
    let ring = Ring::read(&tool).unwrap();
    // the meridians touched whole at a pose: the top's and the bottom's, θ a quarter turn either way
    // of the ring's first axis — its frame's `x` along `z` here, so read by the point it lifts
    let w = gcs_core::brep::trajectory::Twist::body(path.at(0.37).unwrap());
    let (theta,k) = ring.touched_whole(&w).expect("a meridian touched whole");
    let top = ring.meridian_point(theta,k);
    assert!((top[0].abs() < 1e-9 && top[1].abs() < 1e-9) && ((top[2]-2.5).abs() < 1e-9 || (top[2]+1.5).abs() < 1e-9),"{top:?}");
    // the closed sweep alone, no blank to cut it from: refused by name at a meridian touched whole
    match gcs_core::brep::swept::closed::ring(&tool,&path,[0.,1.],0.01,1e-5) {
        Err(Unbuilt::Curved {at}) => assert!(at[0].abs() < 1e-9 && at[1].abs() < 1e-9 && ((at[2]-2.5).abs() < 1e-9 || (at[2]+1.5).abs() < 1e-9),"{at:?}"),
        other => panic!("{:?}",other.map(|b| gcs_core::brep::props::volume(&b))),
    }
    let (cut,_) = cut_by_the_runner();
    let v = gcs_core::brep::props::volume(&cut);
    assert!((v-10696.461766).abs() < 1e-5,"volume {v}");
    let kinds = |k: &str| cut.faces.iter().filter(|f| f.surface.kind() == k).count();
    assert_eq!((kinds("bspline"),kinds("plane")),(5,6));
    groove_counted(&cut,400,None);
    // a ring pushed in at a lean touches no meridian whole: its push leans to the meridians' planes
    let lean = 35f64.to_radians();
    let pushed = Ring::read(&gcs_core::brep::build::torus([0.,0.,4.],[lean.sin(),0.,lean.cos()],3.,1.).unwrap()).unwrap();
    let slide = gcs_core::brep::trajectory::Screw {origin:[0.,0.,4.],axis:[0.,0.,-1.],ratio:0.,advance:8.};
    assert!(pushed.touched_whole(&gcs_core::brep::trajectory::Twist::body(Trajectory::Twist(slide).at(1.).unwrap())).is_none());
}

/// **A frame carried along a spline is written whole** (#942 T1d-ring, `swept::dump`): a trim's
/// trajectory that is a frame along a B-spline (`trajectory::Along`) — refused by name by F5's dump,
/// read only by evaluating it — is written as its curve exactly, its home, which frame and its name,
/// and made again from them, so a trim of a sweep along it is run again apart from the road: read
/// back, its poses and rates are the frame's bit for bit, and it writes the same text again. A path
/// read only by evaluating it otherwise stays refused by name.
#[test]
fn a_frame_carried_along_a_spline_is_written_and_read_back_whole() {
    use gcs_core::brep::swept::dump::{dump_trim,load_trim};
    use gcs_core::brep::swept::trim::Trim;
    let (_,tool,path,tol) = along_a_spline();
    let trim = Trim {resolution:0.1,hair:4.*tol,reach:None};
    let dir = std::env::temp_dir().join(format!("along-dump-{}",std::process::id()));
    dump_trim(&dir,&tool,&path,[0.,1.],&[],&[],0.01,tol,trim,"a test").unwrap();
    let text = std::fs::read_to_string(dir.join("trim.json")).unwrap();
    let loaded = load_trim(&dir).unwrap();
    let _ = std::fs::remove_dir_all(&dir);
    for t in [0.,0.123,0.5,0.877,1.] {
        assert_eq!(format!("{:?}",loaded.path.at(t).unwrap()),format!("{:?}",path.at(t).unwrap()),"at {t}");
    }
    dump_trim(&dir,&loaded.tool,&loaded.path,loaded.roll,&[],&[],loaded.bar,loaded.tol,loaded.trim,"a test").unwrap();
    let again = std::fs::read_to_string(dir.join("trim.json")).unwrap();
    let _ = std::fs::remove_dir_all(&dir);
    assert_eq!(again,text);
    // (a caller's own path, read only by evaluating it, is still refused by name)
    #[derive(Debug)]
    struct Still;
    impl gcs_core::brep::trajectory::Path for Still {
        fn at(&self,_: f64) -> Result<gcs_core::brep::trajectory::Motion,gcs_core::brep::KernelError> {
            Ok(gcs_core::brep::trajectory::Motion::identity())
        }
    }
    let own = Trajectory::General(std::sync::Arc::new(Still));
    let refused = dump_trim(&dir,&tool,&own,[0.,1.],&[],&[],0.01,tol,trim,"a test").unwrap_err();
    let _ = std::fs::remove_dir_all(&dir);
    assert!(refused.contains("read only by evaluating it"),"{refused}");
}

/// **A ring's tubes beside the corners its meridians are touched whole at are arranged with it at
/// both ends** (#942 T1d-ring, `rings::traces` and `boolean`'s grid of seeds): the ring along a
/// spline's two contact curves, fitted, entered at both end poses and arranged in the block with
/// the torus where it starts and where it stops, all at the bench's tolerance. Two things refused
/// it. Beside the tube's vertex where a touching piece's rim meets a dipped one's, the tube and the
/// torus stand within the fit's error of one another along the seam between them (the
/// relative curvature across the rim nought at the vertex), and the fit's slope across the rim,
/// 2.7e-5, put a crossing of the seam 4e-4 of the roll up it: the chord traced from it zigzagged
/// 1.1e-5 off both, round a sliver face laid over its neighbour ("faces … coincide along it"). And
/// a grid of seeds across one tube's face, laid where the other tube's patch's distance changed
/// sign, found its zeros off the patch, 1.0 away from it, where the distance along a normal at a
/// foot clamped to the patch has zeros of its own ("meet at 0.19°, nearly touching"). Each chord
/// given stands within the curve's bar, a quarter of the tolerance, of the torus.
#[test]
fn a_rings_tubes_beside_its_whole_touched_meridians_are_arranged_with_it_at_both_ends() {
    use gcs_core::brep::boolean::arrangement_given;
    use gcs_core::brep::swept::rings::{entered,fitted,posed};
    let (block,tool,path,tol) = along_a_spline();
    let ring = Ring::read(&tool).unwrap();
    let fit = fitted(&ring,&path,[0.,1.],0.1*tol).unwrap();
    let ends = [0.,1.].map(|t| posed(&tool,&ring,&path,&fit,t).unwrap());
    let poses = [0.,1.].map(|t| path.pose(t).unwrap());
    let (tubes,given) = entered(&fit,&ring,[&ends[0],&ends[1]],poses,tol).unwrap();
    // (the two tubes, and the two meridians' strips after them)
    assert_eq!(tubes.len(),4);
    for (g,m) in given.iter().enumerate() {
        let torus = &m.surfaces[0];
        if torus.kind() != "torus" { continue }
        for c in &m.curves {
            let off = (0..=64).map(|k| { let gcs_core::brep::geom::Curve::BSpline(b) = c else { return 0. };
                let [lo,hi] = b.domain(); torus.implicit(c.point(lo+(hi-lo)*k as f64/64.)).abs() }).fold(0.,f64::max);
            assert!(off <= 0.25*tol,"given {g}: a chord {off:e} off the torus");
        }
    }
    let complex = arrangement_given(&block,&[&ends[0],&ends[1],&tubes[0],&tubes[1]],&given,tol)
        .unwrap_or_else(|e| panic!("the tubes with the torus at both ends: {e}"));
    assert!(complex.cells().unwrap().len() >= 2);
}

/// **A whole-touched meridian's touching half is a sheet of its own, bounded by the tubes' seams on
/// it** (#942 T1d-ring, `rings::Strip`): the ring along a spline crosses each contact curve over its
/// top and bottom meridians, and there `s''` changes sign without the curve's sheet folding —
/// touching up to the meridian on one side, the sliver out to a fold on the other. So each tube is
/// entered touching up to its seam on the meridian and dipped across the sliver alone, and the
/// meridian's own touching half, swept, is fitted between the two tubes' seams: its sides are those
/// seams within the fits, its middle the canal's farthest point — the groove's floor, 1.5 below the
/// path, and the top 2.5 above it — and what it meets of each tube is given by construction, the
/// seam and the sliver's far seam, each the very curve the tube's faces are bounded by.
#[test]
fn a_whole_touched_meridians_touching_half_is_a_sheet_between_the_tubes_seams() {
    let (_,tool,path,tol) = along_a_spline();
    let ring = Ring::read(&tool).unwrap();
    let fit = gcs_core::brep::swept::rings::fitted(&ring,&path,[0.,1.],0.1*tol).unwrap();
    assert_eq!(fit.strips.len(),2);
    let mut heights: Vec<f64> = Vec::new();
    for strip in &fit.strips {
        for (e,&(i,c)) in strip.sides.iter().enumerate() {
            // (each tube touching on one side of its seam, dipped on the other)
            let pieces = fit.sides[i].len();
            let faces = [c.seam,(c.seam+pieces-1)%pieces].map(|k| fit.sides[i][k]);
            assert!(faces.contains(&gcs_core::brep::swept::rings::Side::Touching) && faces.contains(&gcs_core::brep::swept::rings::Side::Fold),"{faces:?}");
            let off = (0..=64).map(|j| { let t = j as f64/64.;
                gcs_core::brep::util::space::distance(strip.net.point(e as f64,t),fit.nets[i].point(c.seam as f64,t)) }).fold(0.,f64::max);
            assert!(off <= 0.25*tol,"side {e}: {off:e} off its tube's seam");
        }
        // (on the meridian's circle — the plane `x = 0` of the ring's pose, a tube's radius from its
        // centre 1.5 below or above the ring's — and on its far side from the ring's centre)
        for t in [0.,0.37,1.] {
            let middle = path.pose(t).unwrap().inverse().point(strip.net.point(0.5,t));
            let centre = if middle[2] < 0.5 { -1. } else { 2. };
            heights.push(middle[2]);
            assert!(middle[0].abs() <= 0.1*tol && (middle[1].hypot(middle[2]-centre)-0.5).abs() <= 0.1*tol,"{middle:?}");
            assert!((middle[2]-centre)*(centre-0.5) > 0.,"{middle:?}");
        }
    }
    heights.sort_by(f64::total_cmp);
    assert!(heights[2] < -1. && heights[3] > 2.,"{heights:?}");
}

/// **The half turn's stretches are sewn across its cut into one tube a contact curve** (#942 F3,
/// class B on the sheets a sweep makes): brep-bench's `tumbling-torus-half-turn`, a
/// torus of tube 0.16 turned 0.375 a radian about one line and back a radian about another over
/// half a turn, its contact changing shape where a fold pair is born, at 1.4766, and its roll cut
/// there. Each stretch is fitted a billionth of the roll clear of the cut, and its row there lies
/// along the next stretch's sheet: one row, by construction, the cut's. Sewn so, its four pieces
/// are two tubes, each row edge bounding a face of either stretch — where, the two rows told apart
/// by their parameters' values (6.3e-9 of the roll apart), nothing was sewn, and the row lying
/// along the next sheet was searched for where it met it and not proved.
#[test]
fn the_half_turns_stretches_are_sewn_across_its_cut_into_one_tube_a_contact_curve() {
    use gcs_core::brep::swept::rings::{changes,entered_at,fitted_over,posed};
    let (tool,path,[a,b],tol) = half_turn();
    let ring = Ring::read(&tool).unwrap();
    let (scale,cuts) = changes(&ring,&path,[a,b]).unwrap();
    assert_eq!(cuts.len(),1,"the contact changes shape once: {cuts:?}");
    assert!((cuts[0]-1.4766).abs() < 1e-4,"{cuts:?}");
    // the stretches as the road fits them, a billionth of the roll clear of the cut
    let eps = 1e-9*(b-a);
    let stretches = [[a,cuts[0]-eps],[cuts[0]+eps,b]];
    let fits: Vec<_> = stretches.iter().map(|&sp| fitted_over(&ring,&path,[a,b],sp,scale,0.1*tol).unwrap())
        .collect();
    assert!(fits.iter().all(|f| f.seeds.is_empty()),"no narrow pair's leaves cross");
    let start = posed(&tool,&ring,&path,&fits[0],a).unwrap();
    let end = posed(&tool,&ring,&path,&fits[1],b).unwrap();
    let poses = [path.pose(a).unwrap(),path.pose(b).unwrap()];
    let mut pieces = Vec::new();
    for (k,fit) in fits.iter().enumerate() {
        let at = [Some((&start,poses[0])),Some((&end,poses[1]))];
        pieces.extend(entered_at(fit,&ring,at,[k == 0,k == 1],&|_,_| false,tol).unwrap().0);
    }
    assert_eq!(pieces.len(),4,"two tubes a stretch");
    // and each curve a tube is cut along at an end pose carries, as its pcurve, the places on the
    // face it passes, each at the curve's own parameter there, where that holds the curve's bar (a
    // quarter of the tolerance): the end pose's cut beside the fold pair's birth among them, which
    // inverted onto the leaf landed on the other fold, 2.2e-4 off
    let (mut chords,mut worst) = (0,0f64);
    for piece in &pieces {
        for f in &piece.faces {
            for c in f.loops.iter().flatten() {
                let e = &piece.edges[c.edge as usize];
                let gcs_core::brep::topo::Pcurve::Curve(_) = &c.pcurve else { continue };
                chords += 1;
                for j in 0..=64 {
                    let t = e.t[0]+(e.t[1]-e.t[0])*j as f64/64.;
                    let on = f.surface.point(c.pcurve.at(t,e,&f.surface,&piece.vertices));
                    worst = worst.max(gcs_core::space::distance(on,e.point(t,&piece.vertices)));
                }
            }
        }
    }
    assert!(chords >= 3,"{chords} chords given the places they pass as their pcurves");
    assert!(worst <= 0.25*tol,"a chord's pcurve stands {worst:e} off it, its bar {:e}",0.25*tol);
    // sewn along the cut's row, the one there is
    let row = 0.5*(fits[0].roll[1]+fits[1].roll[0]);
    let across = |uv: [f64;2]| ((uv[1]-row).abs() <= 1e-6*(b-a)).then_some(0);
    let sewn = gcs_core::brep::sew::sewn_open(&pieces,tol,&across).unwrap();
    assert_eq!(gcs_core::brep::sew::parts(&sewn).len(),2,"one tube a contact curve");
    let uses = sewn.uses();
    for (i,e) in sewn.edges.iter().enumerate() {
        let (fi,li,ci) = uses[i][0];
        let f = &sewn.faces[fi];
        let uv = f.loops[li][ci].pcurve.at(0.5*(e.t[0]+e.t[1]),e,&f.surface,&sewn.vertices);
        if across(uv).is_none() { continue }
        assert_eq!(uses[i].len(),2,"row edge {i} bounds {} faces",uses[i].len());
    }
}

/// **A trace of no length bounds nothing** (#942 F6): brep-bench's
/// `tumbling-torus-near-orthogonal`, a torus of tube 0.08 tumbled a whole turn about two lines a
/// degree from square, its tube entered where it crosses the torus at each end pose. One trace of a
/// face's meeting with the tool there is two points on the face's rim 2.4e-13 apart, where a
/// crossing of the face ends: the tool touching the face at a point, not crossing it. Taken for a
/// chord, its two ends and the crossing's stood at one place on the rim and the face was refused
/// ("two of a ring's sheet's crossings of the tool end at one place on a face's rim"); a trace all of
/// whose points stand at one place is left out, and the tubes are entered.
#[test]
fn a_trace_of_no_length_bounds_nothing() {
    use gcs_core::brep::swept::rings::{entered,fitted,posed};
    use gcs_core::brep::trajectory::Screw;
    let tool = gcs_core::brep::build::torus([2.02,0.,1.],[0.,0.,1.],1.,0.08).unwrap();
    let (sl,cl) = 1f64.to_radians().sin_cos();
    let (sa,ca) = 89f64.to_radians().sin_cos();
    let own = Trajectory::Twist(Screw {origin:[2.,0.,0.],axis:[0.,0.,1.],ratio:1.01,advance:0.});
    let frame = Trajectory::Twist(Screw {origin:[0.,0.,0.],axis:[ca,sa*cl,sa*sl],ratio:1.,advance:0.});
    let path = Trajectory::Relative {source:Box::new(own),observer:Box::new(frame)};
    let roll = [-std::f64::consts::PI,std::f64::consts::PI];
    let tol = 0.08/50_000.;
    let ring = Ring::read(&tool).unwrap();
    let fit = fitted(&ring,&path,roll,0.1*tol).unwrap();
    let ends = roll.map(|t| posed(&tool,&ring,&path,&fit,t).unwrap());
    let poses = roll.map(|t| path.pose(t).unwrap());
    let (tubes,_) = entered(&fit,&ring,[&ends[0],&ends[1]],poses,tol).unwrap_or_else(|e| panic!("its tubes not entered: {e:?}"));
    assert_eq!(tubes.len(),2,"a tube for each contact curve");
}

/// **A double curve crossing its tube's seam bounds a tail beside it, not the whole tube** (#942
/// F6): brep-bench's `tumbling-torus-thin`, a torus of tube 0.015 turned a quarter of a radian about
/// one line and back a radian about another, its roll cut where a fold pair is born and where it
/// vanishes and again at each swallowtail point. In the stretch between, one leaf's preimage of the
/// pair's double curve crosses the seam where the tube's faces close round, read from 2.99 to 0.15
/// of its three faces. Read so, wrapped, the curve above which lies the swallowtail's tail ran from
/// one end of the tube to the other: all but a sliver beside its start row was taken for tail and
/// dropped, the stretch met the next at no row, and the tube was sewn into two candidates where it
/// is one — its trim searched three more pairs for the same refusal. Unwrapped along it, as the
/// seams it is cut at already were, the tubes are sewn one a contact curve.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 20 s: the thin tumbling torus entered and sewn")]
fn a_double_curve_crossing_its_tubes_seam_bounds_a_tail_beside_it_not_the_whole_tube() {
    use gcs_core::brep::swept::closed::{RingEntered,ring_entered};
    use gcs_core::brep::trajectory::Screw;
    let tool = gcs_core::brep::build::torus([3.,0.,2.],[0.,0.,1.],1.,0.015).unwrap();
    let (sl,cl) = 45f64.to_radians().sin_cos();
    let own = Trajectory::Twist(Screw {origin:[2.,0.,0.],axis:[0.,0.,1.],ratio:0.25,advance:0.});
    let frame = Trajectory::Twist(Screw {origin:[0.,0.5f64.sqrt()*cl,0.5f64.sqrt()*sl],axis:[1.,0.,0.],
        ratio:1.,advance:0.});
    let path = Trajectory::Relative {source:Box::new(own),observer:Box::new(frame)};
    let roll = [-75f64.to_radians(),75f64.to_radians()];
    let RingEntered {sheets,cut,..} = ring_entered(&tool,&path,roll,0.015/50_000.)
        .unwrap_or_else(|e| panic!("not entered: {e:?}"));
    assert!(cut,"the roll is cut where the contact changes shape");
    assert_eq!(sheets.len(),2,"one tube a contact curve: {:?}",sheets.iter().map(|s| s.faces.len()).collect::<Vec<_>>());
}

/// brep-bench's `tumbling-torus-full-turn`: a torus of tube 0.16 turned 0.375 a radian about one line
/// and back a radian about another over a whole turn, and its tolerance.
fn full_turn() -> (gcs_core::brep::topo::Brep,Trajectory,[f64;2],f64) {
    use gcs_core::brep::trajectory::Screw;
    let tool = gcs_core::brep::build::torus([3.,0.,2.],[0.,0.,1.],1.,0.16).unwrap();
    let (sl,cl) = 30f64.to_radians().sin_cos();
    let (sa,ca) = 20f64.to_radians().sin_cos();
    let own = Trajectory::Twist(Screw {origin:[2.,0.,0.],axis:[0.,0.,1.],ratio:0.375,advance:0.});
    let frame = Trajectory::Twist(Screw {origin:[0.,0.7071*cl,0.7071*sl],axis:[ca,sa*cl,sa*sl],ratio:1.,
        advance:0.});
    let path = Trajectory::Relative {source:Box::new(own),observer:Box::new(frame)};
    (tool,path,[-std::f64::consts::PI,std::f64::consts::PI],0.16/50_000.)
}

/// **The full turn's tube in the middle of the roll is holed where the end torus crosses it** (#942,
/// the slow tier): the end pose's torus passes through the tube some way into the roll, a closed
/// trace inside one of its faces that crosses no rim, read through the place the tube's crossing
/// with itself ends on it (`depth::passed_back`'s partner, covered by the end pose: a seed of
/// `rings::traces`); the piece about it kept with it as a hole, the trace's inside and what the tube
/// late in the roll covers beside it dropped. Each hole of each tube's face has an edge on the end
/// torus, and every edge of it lies on the end torus within the tolerance or on the sweep's boundary
/// (the tube's crossing with itself: held by no instant deeper than the tolerance, and by one to it).
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 45 s: the full turn entered")]
fn the_full_turns_tube_is_holed_where_the_end_torus_crosses_it() {
    use gcs_core::brep::swept::closed::{RingEntered,ring_entered};
    let (tool,path,roll,tol) = full_turn();
    let RingEntered {sheets,..} = ring_entered(&tool,&path,roll,tol).unwrap_or_else(|e| panic!("not entered: {e:?}"));
    let ring = Ring::read(&tool).unwrap();
    let end = path.pose(roll[1]).unwrap().inverse();
    let mut holes = 0;
    for b in &sheets {
        for f in b.faces.iter().filter(|f| f.loops.len() > 1) {
            for l in &f.loops[1..] {
                holes += 1;
                let mut on_torus = 0;
                for c in l {
                    let e = &b.edges[c.edge as usize];
                    let xs: Vec<[f64;3]> = (0..=8).map(|k| e.point(e.t[0]+(e.t[1]-e.t[0])*k as f64/8.,&b.vertices)).collect();
                    if xs.iter().all(|&x| ring.signed(end.point(x)).0.abs() <= tol) { on_torus += 1; continue }
                    for &x in &xs[1..8] {
                        let d = held(&ring,&path,roll,x,roll[0],0.);
                        assert!(d.abs() <= tol,"a hole's edge off the end torus on the sweep's boundary at {x:?}: held {d:e}");
                    }
                }
                assert!(on_torus > 0,"a hole bounded by the end torus");
            }
        }
    }
    assert!(holes > 0,"a tube's face holed by the end torus");
}

/// **brep-bench's `tumbling-torus-full-turn` is built closed** (#942, the slow tier): its trim reads
/// every piece of the tori at its end poses one way all over — the start torus's rim laid as each
/// edge the tube holds it as; its last stretch's tail between a double curve and the tube's seam
/// dropped, and the curve's end where it meets the end pose one point on both leaves — and the two
/// tori's meeting is given where it bounds the sweep (`depth::ends_met`); the start rim cut where
/// the tube late in the roll passes back over it (`boolean`'s `given_crossings`); the tube's crossings
/// with itself cut in (`depth::passed_back`), and the end torus's closed trace across the tube in the
/// middle of the roll a hole in its face; the double curve beside its swallowtail point at the cut
/// 2.2318 joined to it in chords straight in each leaf's parameters, and each leaf's drawing of it,
/// merged into the other's by the arrangement, keeping its own pcurve carried onto the other
/// (`boolean`'s `carried_pcurve`) — inverted, it landed on the leaf past the fold, 1.3e-5 off. Its
/// volume 23.6738854, the peer kernel cadrs's 23.6739; held to the sweep's membership. And each face
/// is the boundary by the depth: a place well inside it (the grid's place in its parameters farthest
/// from its loops) is covered by no instant but its own deeper than the tolerance (`depth::cover`,
/// its own instant the sheet's roll, or the end pose's whose torus it lies on).
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 60 s: the full turn built and held to membership")]
fn the_full_turn_is_built_closed() {
    use gcs_core::brep::swept::depth::{Reading,cover};
    let (tool,path,roll,tol) = full_turn();
    let built = gcs_core::brep::swept::closed::ring(&tool,&path,roll,0.16/250.,tol).unwrap_or_else(|u| panic!("{u:?}"));
    built.check(tol).unwrap();
    let v = gcs_core::brep::props::volume(&built);
    assert!((v-23.6738854384).abs() < 1e-7,"{v}");
    crate::swept_closed::agrees_over(&built,&tool,&path,roll);
    let ring = Ring::read(&tool).unwrap();
    let located = gcs_core::brep::query::Located::new(&built,tol);
    let poses = roll.map(|t| path.pose(t).unwrap().inverse());
    for (fi,f) in built.faces.iter().enumerate() {
        let edges: Vec<[f64;2]> = f.loops.iter().flatten().flat_map(|c| built.uv_ends(f,c)).collect();
        let [lo,hi] = located.parameter_box(fi);
        let n = 64;
        let uv = (0..=n*n).map(|k| [lo[0]+(hi[0]-lo[0])*(k%(n+1)) as f64/n as f64,lo[1]+(hi[1]-lo[1])*(k/(n+1)) as f64/n as f64])
            .filter(|&uv| located.holds_parameters(fi,uv))
            .max_by(|p,q| { let d = |x: [f64;2]| edges.iter().map(|e| (e[0]-x[0]).hypot(e[1]-x[1])).fold(f64::INFINITY,f64::min); d(*p).total_cmp(&d(*q)) })
            .unwrap_or_else(|| panic!("face {fi}: no place inside it"));
        let x = f.surface.point(uv);
        // (a tube's sheet's parameters its roll; a torus the pose whose torus it lies on)
        let own = if f.name == "sheet" { uv[1] } else if ring.signed(poses[0].point(x)).0.abs() <= tol { roll[0] } else { roll[1] };
        match cover(&ring,&path,roll,x,own,tol,tol).unwrap() {
            Reading::Far(c) => assert!(c.depth >= -tol,"face {fi} ({}) covered at {x:?} by the instant {}: {:e} deep",f.name,c.at,-c.depth),
            Reading::Alone => {}
            r => panic!("face {fi} ({}) held by its own instant's neighbours at {x:?}: {r:?}",f.name),
        }
    }
}

/// **The full turn is built wherever its end poses' seams stand** (#942; the slow tier): each end
/// pose's torus turned about its own axis — the same solid — so its seam meridian stands at an angle
/// of its own choosing (the seam vertex's, round the axis from `Frame::about`'s `x`), and the rest
/// built as the ring road builds it (`closed::ring_from`): each checked at the tolerance, its volume
/// the built one's within the tolerance's worth (the solid's area times the tolerance).
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, some minutes: the full turn built at many seams")]
fn the_full_turn_is_built_wherever_its_end_poses_seams_stand() {
    use gcs_core::brep::geom::{Frame,Rigid,Surface};
    use gcs_core::brep::swept::closed::{RingEntered,ring_entered,ring_from};
    let (tool,path,roll,tol) = full_turn();
    let once = ring_entered(&tool,&path,roll,tol).unwrap();
    let mut failed = Vec::new();
    for end in 0..2 {
        for k in 0..8 {
            let seam = -std::f64::consts::PI+std::f64::consts::TAU*(k as f64+0.5)/8.;
            let mut entered = RingEntered {ends:once.ends.clone(),sheets:once.sheets.clone(),given:once.given.clone(),cut:once.cut};
            let pose = &entered.ends[end];
            let Surface::Torus(f,..) = &pose.faces[0].surface else { panic!("an end pose a torus") };
            let about = Frame::about(f.o,f.z);
            let l = about.local(pose.vertices[0].p);
            entered.ends[end] = pose.moved(&Rigid::turn(f.o,f.z,seam-l[1].atan2(l[0])));
            match ring_from(&tool,&path,roll,0.16/250.,tol,entered) {
                Ok(built) => {
                    let v = gcs_core::brep::props::volume(&built);
                    let worth = tol*built.faces.iter().map(|f| gcs_core::brep::props::area(&built,f)).sum::<f64>();
                    if let Err(e) = built.check(tol) { failed.push(format!("end {end} seam {seam:.3}: {e:?}")) }
                    else if (v-23.6738854384).abs() > worth { failed.push(format!("end {end} seam {seam:.3}: volume {v}")) }
                }
                Err(u) => failed.push(format!("end {end} seam {seam:.3}: {u:?}")),
            }
        }
    }
    assert!(failed.is_empty(),"{}",failed.join("\n"));
}

/// **The full turn's start and end tori meet on the boundary along two short arcs, given** (#942
/// T1d-tori, `depth::ends_met`; the slow tier): over a whole turn the end pose stands beside the
/// start and the two tori meet, mostly where the instants between cover one or both — there no cut
/// is needed — but along two arcs, each some 0.02 long between a corner on a tube's seam with one
/// pose and a corner on the other pose's rim, both tori are uncovered but by one another: the arc
/// the trim's search missed ("a meeting no edge crosses"). Read apart from the road by `cover`:
/// each given arc lies on both tori and, at its middle, neither is covered by any other instant by
/// more than the tolerance.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 15 s: the full turn entered")]
fn the_full_turns_end_poses_meet_uncovered_along_two_short_arcs_given() {
    use gcs_core::brep::swept::closed::{RingEntered,ring_entered};
    use gcs_core::brep::swept::depth::{Reading,cover};
    let (tool,path,roll,tol) = full_turn();
    let RingEntered {ends,given,..} = ring_entered(&tool,&path,roll,tol).unwrap_or_else(|e| panic!("not entered: {e:?}"));
    let ring = Ring::read(&tool).unwrap();
    let met: Vec<&gcs_core::brep::boolean::Given> = given.iter()
        .filter(|g| g.surfaces[0] == ends[0].faces[0].surface && g.surfaces[1] == ends[1].faces[0].surface).collect();
    assert_eq!(met.len(),1,"one meeting of the two poses given");
    assert_eq!(met[0].curves.len(),2,"two arcs, one beside each place the tori's core circles cross");
    let poses = roll.map(|t| path.pose(t).unwrap());
    for c in &met[0].curves {
        let gcs_core::brep::geom::Curve::BSpline(b) = c else { panic!("an arc is a spline: {}",c.kind()) };
        let [t0,t1] = b.domain();
        let pts: Vec<[f64;3]> = (0..=16).map(|k| c.point(t0+(t1-t0)*k as f64/16.)).collect();
        let long: f64 = pts.windows(2).map(|w| gcs_core::brep::util::space::distance(w[0],w[1])).sum();
        assert!(long > 0.01 && long < 0.05,"an arc some 0.02 long: {long}");
        for &x in &pts {
            for q in &poses { assert!(ring.signed(q.inverse().point(x)).0.abs() <= tol,"the arc lies on both tori at {x:?}") }
        }
        let mid = pts[8];
        // (a point of the fitted arc stands off either torus by as much as its fit: read so)
        for &own in &roll {
            match cover(&ring,&path,roll,mid,own,tol,tol).unwrap() {
                Reading::Far(cv) => assert!(cv.depth >= -tol,"the pose at {own} uncovered at the arc's middle: {cv:?}"),
                Reading::Alone => {}
                r => panic!("the pose at {own} uncovered at the arc's middle: {r:?}"),
            }
        }
    }
}

/// The least signed distance from the ring posed at an instant of `roll` of the point `p`, over
/// the instants at least `apart` from `t` — the ring read at `4096` of them and the least of those
/// settled by golden sections: below nought where some instant holds `p`.
fn held(ring: &Ring,path: &Trajectory,roll: [f64;2],p: [f64;3],t: f64,apart: f64) -> f64 {
    let d = |tau: f64| ring.signed(path.pose(tau).unwrap().inverse().point(p)).0;
    let n = 4096;
    let step = (roll[1]-roll[0])/n as f64;
    let (mut best,mut at) = (f64::INFINITY,roll[0]);
    for k in 0..=n {
        let tau = roll[0]+step*k as f64;
        if (tau-t).abs() >= apart { let v = d(tau); if v < best { (best,at) = (v,tau) } }
    }
    let (mut lo,mut hi) = ((at-step).max(roll[0]),(at+step).min(roll[1]));
    let g = 0.5*(5f64.sqrt()-1.);
    for _ in 0..80 {
        let (x1,x2) = (hi-g*(hi-lo),lo+g*(hi-lo));
        if d(x1) < d(x2) { hi = x2 } else { lo = x1 }
    }
    best.min(d(0.5*(lo+hi)))
}

/// **The half turn's double curve is built from the swallowtail point beside its fold pair's birth**
/// (#942 T1d-tori): the pair is born at 1.4766, where the contact's least `s''` reaches nought, and
/// there the fold is tangent to the instant's own contact, no kernel of the sheet's: an ordinary
/// point of a cuspidal edge. Its double curve begins where the left fold runs along the kernel, 0.0073
/// of the roll later, and its two preimages run from there, one up to the roll's end and one back
/// across the cut into the stretch before the birth — where no stretch's net reaches it, and its rows
/// found no pair. Built outward from that point, and held to what the sweep is apart from the road:
/// every point of it is held by the ring at another instant only on its boundary (a double point),
/// and the leaf a little above it in the roll is inside the sweep — the tail — and a little below
/// it is not.
#[test]
fn the_half_turns_double_curve_is_built_from_the_swallowtail_point_beside_its_birth() {
    use gcs_core::brep::swept::rings::{branch_at,changes,double_curve_from_apex,fitted_over};
    let (tool,path,[a,b],tol) = half_turn();
    let ring = Ring::read(&tool).unwrap();
    let (scale,cuts) = changes(&ring,&path,[a,b]).unwrap();
    let eps = 1e-9*(b-a);
    let fits: Vec<_> = [[a,cuts[0]-eps],[cuts[0]+eps,b]].iter().map(|&sp| fitted_over(&ring,&path,[a,b],sp,scale,0.1*tol).unwrap())
        .collect();
    assert!(fits.iter().all(|f| f.seeds.is_empty()),"its rows find no pair");
    assert!(fits[0].apexes.is_empty(),"no swallowtail point before the birth: {:?}",fits[0].apexes);
    let [apex] = fits[1].apexes[..] else { panic!("one swallowtail point after the birth: {:?}",fits[1].apexes) };
    let (th0,t0,_) = apex.at;
    assert!(t0-cuts[0] > 0.007 && t0-cuts[0] < 0.0076,"the point {:e} of the roll after the birth",t0-cuts[0]);
    // the kernel's way there, `dt/dθ`, −1/2.06 as the fold's slope reaches it
    assert!((apex.way+0.485).abs() < 2e-3,"the kernel's way {}",apex.way);
    let curve = double_curve_from_apex(&ring,&path,[a,b],apex,tol).unwrap();
    let mark = curve.iter().position(|p| p.0.is_nan()).unwrap();
    let pts = &curve[..mark];
    let back: Vec<_> = pts.windows(2).enumerate().filter(|(_,w)| w[1].0 <= w[0].0).map(|(i,w)| (i,w[0],w[1])).collect();
    assert!(back.is_empty(),"a graph over its first preimage's angle: of {} it turns back at {back:?}",pts.len());
    assert!(pts.iter().any(|p| (p.0-th0).abs() <= 1e-12 && (p.1-t0).abs() <= 1e-12),"the point itself on the curve");
    let (lo,hi) = pts.iter().fold((f64::INFINITY,f64::NEG_INFINITY),|(l,h),p| (l.min(p.1),h.max(p.1)));
    assert!((hi-b).abs() <= 1e-12,"one preimage reaches the roll's end: {hi}");
    assert!(lo < cuts[0]-0.01,"the other runs back across the cut: {lo}");
    // a double point at every place along it, and the tail above it: read on the leaf (the contact's
    // `−` branch) apart from where the two preimages' instants meet, beside the point itself
    let leaf = |th: f64,t: f64| branch_at(&ring,&path,[a,b],t,false,&[th]).unwrap()[0].1;
    let mut read = 0;
    for p in pts.iter().step_by(8).filter(|p| (p.1-t0).abs() > 0.01) {
        let on = held(&ring,&path,[a,b],leaf(p.0,p.1),p.1,2e-3);
        assert!(on.abs() <= tol,"at {p:?} the ring at another instant stands {on:e} off it");
        let delta = 2e-3;
        if p.1+delta > b || p.1-delta < a { continue }
        let (above,below) = (held(&ring,&path,[a,b],leaf(p.0,p.1+delta),p.1+delta,2e-3),held(&ring,&path,[a,b],leaf(p.0,p.1-delta),p.1-delta,2e-3));
        assert!(above < -tol && below > 0.,"at {p:?}: above it {above:e}, below it {below:e}");
        read += 1;
    }
    assert!(read >= 8,"{read} places read");
}

/// **brep-bench's `tumbling-torus-half-turn` is built closed** (#942 T1d-tori), as the bench calls
/// it: the bar a 250th of the tube, the tolerance a 50 000th. Its fold pair is born at 1.4766 and its
/// double curve begins at the swallowtail point 0.0073 of the roll later, running back below it
/// across the birth's row; parted at the birth, that row's contact stood within the tolerance of the
/// double curve over 8.6e-4 of it (0.2e-6 to 3.03e-6, the tolerance 3.2e-6), the arrangement cut the
/// row's edge at the point, and the trim was refused ("face 8 loop breaks in parameters", edges used
/// four to six times). Parted at the swallowtail point instead, the pair read as not yet born before
/// it (`rings::fitted_unborn`), the row crosses the double curve only where it ends, and the solid
/// is built, checked and judged: its volume 12.2377154, where cadrs's is 12.2377, held to
/// membership. (A Monte Carlo count of the definition read 12.198 ± 0.021.)
#[test]
fn the_half_turn_is_built_closed() {
    let (tool,path,roll,tol) = half_turn();
    let built = gcs_core::brep::swept::closed::ring(&tool,&path,roll,0.16/250.,tol).unwrap_or_else(|u| panic!("{u:?}"));
    built.check(tol).unwrap();
    let v = gcs_core::brep::props::volume(&built);
    assert!((v-12.2377154238).abs() < 1e-7,"{v}");
    crate::swept_closed::agrees_over(&built,&tool,&path,roll);
}

/// **A double curve ending on a tube's crossing with the tool is one vertex with it** (#942
/// T1d-tori): the thin tumbling torus's double curve, where its tube passes back through itself,
/// ends at the end pose on the tube's rim, which touches the tool there, and on the leaf it passes
/// through it ends where that leaf crosses the tool — a chord ending on another inside the face's
/// square. That end is the tube's own place, by construction: put on the crossing's polyline
/// instead, it stood 1e-6 (3.5 tolerances) from the rim's corner it is, two vertices of the tube on
/// the tool, the touching rim a slit in the tool's face and the trim refused where it read the
/// face's piece both ways. No two of the tubes' vertices on the tool where it stops stand a few
/// tolerances apart, within ten of one another and not one place.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 20 s: the thin tumbling torus entered and sewn")]
fn a_double_curve_ending_on_a_tubes_crossing_with_the_tool_is_one_vertex_with_it() {
    use gcs_core::brep::swept::closed::{RingEntered,ring_entered};
    use gcs_core::brep::trajectory::Screw;
    use gcs_core::brep::util::space::distance;
    let tool = gcs_core::brep::build::torus([3.,0.,2.],[0.,0.,1.],1.,0.015).unwrap();
    let (sl,cl) = 45f64.to_radians().sin_cos();
    let own = Trajectory::Twist(Screw {origin:[2.,0.,0.],axis:[0.,0.,1.],ratio:0.25,advance:0.});
    let frame = Trajectory::Twist(Screw {origin:[0.,0.5f64.sqrt()*cl,0.5f64.sqrt()*sl],axis:[1.,0.,0.],
        ratio:1.,advance:0.});
    let path = Trajectory::Relative {source:Box::new(own),observer:Box::new(frame)};
    let roll = [-75f64.to_radians(),75f64.to_radians()];
    let tol = 0.015/50_000.;
    let RingEntered {ends,sheets,..} = ring_entered(&tool,&path,roll,tol).unwrap_or_else(|e| panic!("not entered: {e:?}"));
    let stop = &ends[1].faces[0].surface;
    let on: Vec<[f64;3]> = sheets.iter().flat_map(|s| s.vertices.iter().map(|v| v.p)).filter(|&p| stop.implicit(p).abs() <= tol).collect();
    assert!(on.len() > 4,"the tubes' vertices on the tool where it stops: {}",on.len());
    for (i,&p) in on.iter().enumerate() { for &q in &on[i+1..] {
        let d = distance(p,q);
        assert!(d <= tol || d > 10.*tol,"two of the tubes' vertices on the tool {d:e} apart: {p:?} and {q:?}");
    } }
}

/// **The full turn's tube passes back over its own start, read off the depth** (#942 T1d-tori,
/// `depth::passed_back`; the slow tier): over a whole turn the tube late in the roll sweeps across
/// the tube where it began — no fold pair's crossing, so no double curve the road traces — and the
/// curve where it does parts what it covers of the early tube from what stays on the boundary. It
/// leaves the start row at the start rim's crossing with the late tube's trace on the start torus,
/// `(2.5497, 3.1166, −0.1932)`, and ends where the late tube's covering instant reaches the roll's
/// end, on the end pose's trace. Read from the start row's change of cover: a run whose own end on
/// the start row is that point, its partner on the late tube's sheet, and each of its points the
/// partner's point in space — two sheets at one place.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 40 s: the full turn fitted and its crossings read")]
fn the_full_turns_tube_passing_back_over_its_start_is_read_from_the_start_row() {
    use gcs_core::brep::swept::closed::ring_fitted;
    use gcs_core::brep::swept::depth::{label_at,passed_back,sheet};
    use gcs_core::space::distance;
    let (tool,path,roll,tol) = full_turn();
    let (ring,_,_,curves,_) = ring_fitted(&tool,&path,roll,tol).unwrap_or_else(|e| panic!("not fitted: {e:?}"));
    let back = passed_back(&ring,&path,roll,&curves,tol).unwrap_or_else(|e| panic!("not read: {e:?}"));
    assert!(back.len()%2 == 0 && !back.is_empty(),"each run with its partner: {}",back.len());
    let at = |q: (f64,f64,f64)| sheet(&ring,&path,roll,q.0,q.1,label_at(&ring,&path,q.0,q.1,q.2).unwrap()).unwrap().0;
    let rim = [2.549694,3.116605,-0.193242];
    let run = back.chunks(2).find(|r| r[0].iter().any(|&q| q.1 == roll[0] && distance(at(q),rim) <= 1e-5))
        .unwrap_or_else(|| panic!("no run through the start rim's crossing: {:?}",back.chunks(2).map(|r| (r[0][0],r[0][r[0].len()-1])).collect::<Vec<_>>()));
    let (own,partner) = (&run[0],&run[1]);
    assert_eq!(own.len(),partner.len());
    assert!(partner.iter().all(|q| q.1 > 3.0 && q.1 <= roll[1]),"the partner late in the roll: {partner:?}");
    assert!(partner.iter().any(|q| q.1 == roll[1]),"its corner on the end pose's row: {partner:?}");
    // (each point the partner's in space: the two sheets cross there, to the tolerance the seam is
    // held to and the partner's root is read within)
    let apart = own.iter().zip(partner).map(|(&p,&q)| distance(at(p),at(q))).fold(0.,f64::max);
    assert!(apart <= 4.*tol,"the run off its partner by {apart:e}");
}

/// **A closed chord inside a face's square, with a curve ending on it, is a hole in the piece about
/// it** (#942, the full turn: the end torus's trace across the tube in the middle of the roll, and
/// the tube's crossing with itself ending on it; `rings::loops_of_square`). A circle laid inside the
/// square, a chord across it between two of its points and a chord cutting off a corner: the corner's
/// triangle, the square about the circle holding it as one hole (its places the circle's, walked
/// clockwise), and the circle's two halves — their areas the circle's between them, nothing lost.
#[test]
fn a_closed_chord_inside_a_faces_square_is_a_hole_in_the_piece_about_it() {
    use gcs_core::brep::swept::rings::loops_of_square;
    let (c,r,n) = ([0.5,0.5],0.2,400);
    let at = |k: usize| { let a = std::f64::consts::FRAC_PI_2+std::f64::consts::TAU*k as f64/n as f64; [c[0]+r*a.cos(),c[1]+r*a.sin()] };
    let circle: Vec<[f64;2]> = (0..=n).map(|k| at(k%n)).collect();
    // (the circle's points a quarter and three quarters round: (0.3, 0.5) and (0.7, 0.5))
    let across: Vec<[f64;2]> = (0..=20).map(|k| [at(n/4)[0]+(at(3*n/4)[0]-at(n/4)[0])*k as f64/20.,0.5]).collect();
    let corner = vec![[0.,0.1],[0.05,0.05],[0.1,0.]];
    let pieces = loops_of_square(&[(1,circle),(2,across),(0,corner)]).unwrap();
    let area = |p: &[[f64;2]]| 0.5*(0..p.len()).map(|i| { let (a,b) = (p[i],p[(i+1)%p.len()]); a[0]*b[1]-b[0]*a[1] }).sum::<f64>();
    assert_eq!(pieces.len(),4,"the corner, the square about the circle, the circle's halves: {:?}",
        pieces.iter().map(|p| p.iter().map(|l| area(l)).collect::<Vec<_>>()).collect::<Vec<_>>());
    let holed: Vec<&Vec<Vec<[f64;2]>>> = pieces.iter().filter(|p| p.len() > 1).collect();
    assert_eq!(holed.len(),1,"one piece holds a hole");
    assert_eq!(holed[0].len(),2,"one hole in it");
    let disc = std::f64::consts::PI*r*r;
    let hole = &holed[0][1];
    // (a polygon of 400 sides inscribed in the circle: its area within a ten-thousandth of the disc's)
    assert!((area(hole)+disc).abs() <= 1e-4*disc,"the hole the circle, walked clockwise: {}",area(hole));
    assert!(hole.iter().all(|p| ((p[0]-c[0]).hypot(p[1]-c[1])-r).abs() <= 1e-12),"the hole's places the circle's");
    assert!((area(&holed[0][0])-(1.-0.005)).abs() <= 1e-12,"the square about it less the corner: {}",area(&holed[0][0]));
    let inner: f64 = pieces.iter().filter(|p| p.len() == 1 && p[0].iter().all(|q| (q[0]-c[0]).hypot(q[1]-c[1]) <= r+1e-12))
        .map(|p| area(&p[0])).sum();
    assert!((inner-(-area(hole))).abs() <= 1e-12,"the circle's halves fill the hole: {inner} against {}",-area(hole));
}

/// brep-bench's `tumbling-torus-thin`: a torus of tube 0.015 turned a quarter of a radian about the
/// vertical line through (2, 0, 0) and back a radian about the line through (0, h cos 45°, h sin 45°)
/// along `x`, `h` √½, over ±75°, at the bench's tolerance, a 50 000th of the tube.
fn thin() -> (Brep,Trajectory,[f64;2],f64) {
    use gcs_core::brep::trajectory::Screw;
    let tool = gcs_core::brep::build::torus([3.,0.,2.],[0.,0.,1.],1.,0.015).unwrap();
    let (sl,cl) = 45f64.to_radians().sin_cos();
    let own = Trajectory::Twist(Screw {origin:[2.,0.,0.],axis:[0.,0.,1.],ratio:0.25,advance:0.});
    let frame = Trajectory::Twist(Screw {origin:[0.,0.5f64.sqrt()*cl,0.5f64.sqrt()*sl],axis:[1.,0.,0.],
        ratio:1.,advance:0.});
    let path = Trajectory::Relative {source:Box::new(own),observer:Box::new(frame)};
    (tool,path,[-75f64.to_radians(),75f64.to_radians()],0.015/50_000.)
}

/// **A bridge's end gives way to a double curve within the arrangement's band of it** (#942
/// T1d-tori, `rings::give_way`): the thin tumbling torus's first double curve's first preimage
/// starts on the fold at its swallowtail point (`s''` nought), crosses the bridge's end — where `s''`
/// is the depth of the pair's least — 0.036 of the roll above it and stands at three times that
/// depth where the roll ends; where it crosses, the sheet's rates all but one way, it runs within
/// 8.8e-7 of the end in space (three tolerances) over 9.9e-4 of its length, inside the band
/// (`tolerant::ON_CURVE`, eight) within which the arrangement takes two curves for one, and the
/// check refused the strip between them laid as one edge. So that branch is the bridge's start over
/// both stretches it spans, from its point to the roll's end, and the second curve's branch rising
/// from its point (crossing the end 0.03 above it) the end of the second pair's bridge over the
/// last; the branches clear of every end (the first curve's other preimage, at 3.6 to 18 times the
/// depth) give none. The half turn's give none: its double curve stands clear of its bridge's ends.
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 40 s: the thin tumbling torus fitted and given way")]
fn a_bridges_end_gives_way_to_a_double_curve_within_the_arrangements_band_of_it() {
    use gcs_core::brep::swept::closed::ring_fitted;
    let (tool,path,roll,tol) = thin();
    let (_,_,fits,curves,_) = ring_fitted(&tool,&path,roll,tol).unwrap();
    assert_eq!((fits.len(),curves.len()),(3,2),"three stretches, two double curves");
    let given: Vec<Vec<(usize,usize)>> = fits.iter().map(|f| f.ways.iter().map(|w| (w.of.0,w.side)).collect()).collect();
    assert_eq!(given,vec![vec![],vec![(0,0)],vec![(0,0),(1,1)]],"the branches given way, stretch by stretch");
    // each way's branch spans its stretch, and starts on the stretch's first row where it starts at
    // its swallowtail point
    for f in &fits { for w in &f.ways {
        let (lo,hi) = w.branch.iter().fold((f64::INFINITY,f64::NEG_INFINITY),|(l,h),p| (l.min(p.1),h.max(p.1)));
        assert!(lo <= f.roll[0] && hi >= f.roll[1],"a branch over {:?} given way over {:?}",[lo,hi],f.roll);
    } }
    let (tool,path,roll,tol) = half_turn();
    let (_,_,fits,_,_) = ring_fitted(&tool,&path,roll,tol).unwrap();
    assert!(fits.iter().all(|f| f.ways.is_empty()),"the half turn gives no way");
}

/// **brep-bench's `tumbling-torus-thin` is built closed** (#942 T1d-tori), as the bench calls it.
/// Its bridges' ends given way to the double curves they grazed (`rings::give_way`), each sheet's
/// seams first (`rings::seams_first`), the solid is built, checked at the call's tolerance and
/// judged: 28 faces, held to membership, its mesh within 1e-4 enclosing 0.6962402 (within 1e-5,
/// 0.6962961; cadrs's volume 0.69630). Read point by point, the solid and the sweep's definition agree
/// at all 20 000 places of its box (`ring_profile thin 75 agree 20000 2e-5`). So it is built too with
/// the observer's origin moved by one ulp and by 1e-6 either way (`ring_profile thin 75 nudge ulp`),
/// its mesh's volume within 1.2e-6 of this one's. (`props::volume` reads it 0.17% high, 0.6975:
/// beside each swallowtail point the lens's edge and a short piece of a row merged into the seam
/// given way are read by inversion onto the fold, and the line integral there misses still.)
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 3 min: the thin tumbling torus built closed")]
fn the_thin_tumbling_torus_is_built_closed() {
    let (tool,path,roll,tol) = thin();
    let built = gcs_core::brep::swept::closed::ring(&tool,&path,roll,0.015/250.,tol).unwrap_or_else(|u| panic!("{u:?}"));
    built.check(tol).unwrap();
    assert_eq!(built.faces.len(),28);
    let m = gcs_core::brep::mesh::mesh(&built,1e-4,0.1).unwrap();
    let v: f64 = m.tris.iter().map(|t| {
        let [p,q,r] = t.map(|i| m.pts[i as usize]);
        gcs_core::brep::util::space::dot(p,gcs_core::brep::util::space::cross(q,r))/6.
    }).sum();
    assert!((v-0.6962402).abs() < 1e-6,"its mesh within 1e-4 encloses {v}");
    assert!((v-0.69630).abs() < 1e-4,"{v} against cadrs's 0.69630");
    crate::swept_closed::agrees_over(&built,&tool,&path,roll);
}
