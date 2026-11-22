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
    assert_eq!(tubes.len(),2,"a tube for each contact curve");
    for (k,tube) in tubes.iter().enumerate() {
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
    use gcs_core::brep::trajectory::Screw;
    let tool = gcs_core::brep::build::torus([3.,0.,2.],[0.,0.,1.],1.,0.16).unwrap();
    let (sl,cl) = 30f64.to_radians().sin_cos();
    let (sa,ca) = 20f64.to_radians().sin_cos();
    let own = Trajectory::Twist(Screw {origin:[2.,0.,0.],axis:[0.,0.,1.],ratio:0.375,advance:0.});
    let frame = Trajectory::Twist(Screw {origin:[0.,0.7071*cl,0.7071*sl],axis:[ca,sa*cl,sa*sl],ratio:1.,
        advance:0.});
    let path = Trajectory::Relative {source:Box::new(own),observer:Box::new(frame)};
    let [a,b] = [-90f64.to_radians(),90f64.to_radians()];
    let tol = 0.16/50_000.;
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
        pieces.extend(entered_at(fit,&ring,at,[k == 0,k == 1],&|_| false,tol).unwrap().0);
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

/// **The full turn gets past its start and end tori** (#942 T1d-tori, the slow tier):
/// brep-bench's `tumbling-torus-full-turn`, a torus of tube 0.16 turned 0.375 a radian about one
/// line and back a radian about another over a whole turn. Its trim reads every piece of the tori
/// at its end poses one way all over — the start torus's rim laid as each edge the tube holds it as
/// (refused before, a piece of the start torus the boundary on one side of the rim and off it on
/// the other); its last stretch's tail between a double curve and the tube's seam dropped, and the
/// curve's end where it meets the end pose one point on both leaves (refused before, a piece of the
/// end torus read both ways round a chord hung open) — and it is refused only where the two tori's
/// meeting, found by search, misses an arc between the points they touch at (the end pose the start
/// turned 0.75π about a line parallel to the torus's axis).
#[test]
#[cfg_attr(not(feature = "slow"), ignore = "slow tier, about 40 s: the full turn built to its refusal")]
fn the_full_turns_tori_are_read_one_way_all_over_and_it_is_refused_where_they_meet() {
    use gcs_core::brep::swept::closed::ring;
    use gcs_core::brep::trajectory::Screw;
    let tool = gcs_core::brep::build::torus([3.,0.,2.],[0.,0.,1.],1.,0.16).unwrap();
    let (sl,cl) = 30f64.to_radians().sin_cos();
    let (sa,ca) = 20f64.to_radians().sin_cos();
    let own = Trajectory::Twist(Screw {origin:[2.,0.,0.],axis:[0.,0.,1.],ratio:0.375,advance:0.});
    let frame = Trajectory::Twist(Screw {origin:[0.,0.7071*cl,0.7071*sl],axis:[ca,sa*cl,sa*sl],ratio:1.,
        advance:0.});
    let path = Trajectory::Relative {source:Box::new(own),observer:Box::new(frame)};
    let roll = [-std::f64::consts::PI,std::f64::consts::PI];
    let why = format!("{:?}",ring(&tool,&path,roll,0.16/250.,0.16/50_000.).expect_err("refused where the tori meet"));
    assert!(why.contains("torus piece kept beside it (candidate and face (0, 0))") && why.contains("face (1, 0), a torus")
        && why.contains("met by search"),"{why}");
}
