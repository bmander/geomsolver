//! A station's contact curve and the sheet of them (`solid::contact_trace`), on the torus rolled
//! about a skew axis through a post: the admitted small fixture, whose sections are circles a
//! test can sample exactly, so the tracer is checked without a kernel and in seconds.
use fixtures::{tools::{skew_post,torus},motions::{Observer,cradle_roll}};
use gcs_core::solid::{SweepContacts,contact_trace::{Band,Extent,Sample,Station,Tracer,TraceError,STEP_TIME}};
use std::f64::consts::{PI,TAU};

type V = [f64;3];

fn sweep() -> SweepContacts {
    let source = format!("{}{}construction solid removal(tool, under: turn, from: -75deg, to: 75deg)\n{}",torus(2.),cradle_roll(0.25,Observer::Skew),skew_post(4.));
    let e = fixtures::read(&source);
    SweepContacts::read(&e.sketch,fixtures::solid(&e,"removal"),1e-10).unwrap()
}

/// The post the removal cuts: radius 0.4 about the vertical line x = 4, z in [0.5, 2].
fn in_post(p: &V) -> bool { (p[0]-4.).hypot(p[1]) < 0.4 && p[2] > 0.5 && p[2] < 2. }

/// The torus's section by the meridian half-plane at `angle` about its own axis (the vertical
/// line x = 3): a circle of radius 0.5 about (3, 0, 2) plus 1 along the half-plane, walked from
/// its outermost point by arc length.
fn station(angle: f64) -> Station<'static> {
    let side = [angle.cos(),angle.sin(),0.];
    let centre = [3.+side[0],side[1],2.];
    Station {length:PI,window:[0.,PI],sample:Box::new(move |s: f64| {
        let phi = s/0.5;
        let normal: V = std::array::from_fn(|k| phi.cos()*side[k]+if k == 2 { phi.sin() } else { 0. });
        Ok(Sample {position:std::array::from_fn(|k| centre[k]+0.5*normal[k]),normal})
    })}
}

#[test]
fn a_traced_contact_curve_is_one_root_carried_out_of_the_blank() {
    let sweep = sweep();
    let inside = |points: &[V]| Ok(points.iter().map(in_post).collect());
    let tracer = Tracer {sweep:&sweep,scale:1.,inside:&inside,debug:false};
    let (mut traced,mut missed) = (0,0);
    for k in 0..48 {
        let station = station(TAU*(k as f64+0.5)/48.);
        let t = match tracer.trace(&station,0.5,Extent::Blank) {
            Ok(t) => t,
            Err(TraceError::Missed) => { missed += 1; continue }
            Err(e) => panic!("station {k}: {e}"),
        };
        traced += 1;
        let curve = &t.curve;
        assert!(t.anchor < curve.len() && in_post(&curve[t.anchor].found.position),"the anchor is in the blank");
        assert!(!in_post(&curve[0].found.position) && !in_post(&curve.last().unwrap().found.position),
            "both ends leave the blank");
        for w in curve.windows(2) {
            assert!(w[1].tau > w[0].tau,"unfolded length increases along the curve");
            if w[0].found.branch == w[1].found.branch {
                assert!((w[1].found.time-w[0].found.time).abs() < STEP_TIME,"one root, no leap: {w:?}");
            }
        }
        // Every point is a root of its own sample's contact equation, as the tracer found it.
        for p in curve.iter().step_by(7) {
            let roots = tracer.contacts((station.sample)(p.s).unwrap(),tracer.wide()).unwrap();
            assert!(roots.iter().any(|f| f.branch == p.found.branch && (f.time-p.found.time).abs() < 1e-9
                && gcs_core::space::distance(f.position,p.found.position) < 1e-9),"{p:?} against {roots:?}");
        }
        // Resampled at its own lengths, the curve gives its own points back.
        let at: Vec<f64> = curve.iter().map(|p| p.tau).collect();
        for (p,f) in curve.iter().zip(tracer.along(&station,curve,&at).unwrap()) {
            assert!(gcs_core::space::distance(p.found.position,f.position) < 1e-6,"{p:?} against {f:?}");
        }
    }
    eprintln!("{traced} stations traced, {missed} miss the post");
    assert!(traced > 0 && missed > 0,"some stations reach the post and some do not");
}

#[test]
fn a_sheet_of_contact_curves_is_one_chart() {
    let sweep = sweep();
    let inside = |points: &[V]| Ok(points.iter().map(in_post).collect());
    let tracer = Tracer {sweep:&sweep,scale:1.,inside:&inside,debug:false};
    // The band the stations reach, coarsely: those whose curve anchors in the post.
    let hits: Vec<f64> = (0..96).map(|k| TAU*(k as f64+0.5)/96.)
        .filter(|&a| tracer.trace(&station(a),0.5,Extent::Blank).is_ok()).collect();
    let band = Band {stations:[hits[0],*hits.last().unwrap()],radius:1.};
    let sheet = tracer.sheet(&|a| Ok(station(a)),band,0.5,(band.stations[1]-band.stations[0])*0.15).unwrap();
    eprintln!("sheet {}x{}, {} withheld",sheet.rows,sheet.columns,sheet.withheld.len());
    assert_eq!(sheet.points.len(),sheet.rows*sheet.columns);
    assert!(!sheet.withheld.is_empty());
    assert!(sheet.chart_fault(&inside).unwrap().is_none());
}

/// A sample with no direction has no contact time: a degenerate reading, which a scan passes
/// over, never a failure of the construction.
#[test]
fn a_sample_without_a_normal_is_degenerate() {
    let sweep = sweep();
    let inside = |points: &[V]| Ok(points.iter().map(in_post).collect());
    let tracer = Tracer {sweep:&sweep,scale:1.,inside:&inside,debug:false};
    let e = tracer.contacts(Sample {position:[4.,0.,2.],normal:[0.;3]},tracer.wide()).unwrap_err();
    assert!(matches!(e,TraceError::Degenerate(_)),"{e:?}");
    assert_eq!(e.to_string(),"degenerate normal");
}
