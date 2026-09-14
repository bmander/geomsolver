//! Production source charts, with the cylinder formulas used only as a judge.
use super::{harness,creases,tools,motions};
use gcs_core::{interval::{Interval as I,minimum},solid::{SweepContacts,sweep_source::{Chart,Source,SourcePoint},
    swept_boundary::{self as sb,arrangement::{self as a,Pair,Options,TraceOptions,Error}}}};
fn sweep() -> SweepContacts {
    let e = harness::read(&creases::tumbling_cylinder());
    SweepContacts::read(&e.sketch,harness::solid(&e,"swept"),1e-10).unwrap()
}
fn walls(s: &SweepContacts) -> Pair<'_> {
    Pair::new(s,[Chart::Endpoint {face:1,end:sb::End::From},Chart::Endpoint {face:1,end:sb::End::To}],
        [[0.,1.];4],1,Options::default()).unwrap()
}
fn sampling() -> TraceOptions { TraceOptions {sagitta:0.001,parameter_step:0.1,max_depth:16,max_solves:4096} }
fn partition(trace: &a::Trace,range: [f64;2]) {
    let mut spans: Vec<_> = trace.segments.iter().map(|s| [trace.points[s[0]].coordinate,trace.points[s[2]].coordinate])
        .chain(trace.unresolved.iter().map(|u| u.range)).collect();
    spans.sort_by(|a,b| a[0].total_cmp(&b[0]));
    assert_eq!(spans.first().unwrap()[0],range[0]); assert_eq!(spans.last().unwrap()[1],range[1]);
    for w in spans.windows(2) { assert_eq!(w[0][1],w[1][0],"lost or duplicated parameter interval"); }
}

#[test]
fn endpoint_wall_trim_retains_both_native_consumers() {
    let s = sweep(); let pair = walls(&s); let h = s.domain()[1];
    let trace = pair.trace([0.05,0.45],[[0.5,0.05];2],sampling()).unwrap();
    assert!(trace.unresolved.is_empty(),"{:?}",trace.unresolved);
    assert!(trace.segments.len() > 8); partition(&trace,[0.05,0.45]);
    for p in &trace.points {
        assert!(harness::distance(p.positions[0],p.positions[1]) < 1e-11);
        let source = s.faces()[1].at(0.5,p.coordinate).unwrap().position;
        let expected = [source[0],source[1]/h.cos(),0.];
        for q in p.positions { assert!(harness::distance(q,expected) < 1e-9,"{p:?}"); }
        for k in 0..2 {
            assert_eq!(p.source[k].source,Source::Face(1));
            assert_eq!(s.source_at(p.source[k],p.roll[k]).unwrap().position,p.positions[k]);
        }
    }
    eprintln!("endpoint wall trim segments={} solves={} sampled_error={}",trace.segments.len(),trace.evaluations,trace.sampled_error);
}

#[test]
fn trim_is_stable_under_seed_consumer_and_roll_changes() {
    for h in [29.9,30.,30.1] {
        let source = format!("{}{}{}",tools::CYLINDER,motions::TUMBLE,motions::swept("turn",-h,h));
        let e = harness::read(&source); let s = SweepContacts::read(&e.sketch,harness::solid(&e,"swept"),1e-10).unwrap();
        let pair = walls(&s);
        let reversed = Pair::new(&s,[pair.charts()[1],pair.charts()[0]],pair.bounds(),3,Options::default()).unwrap();
        for v in [0.07,0.2,0.37,0.43] {
            let a = pair.at(v,[[0.3,v],[0.7,v]]).unwrap();
            let b = pair.at(v,[[0.7,v+0.01],[0.3,v-0.01]]).unwrap();
            let c = reversed.at(v,[a.parameters[1],a.parameters[0]]).unwrap();
            assert!(harness::distance(a.positions[0],b.positions[0]) < 1e-10);
            assert!(harness::distance(a.positions[0],c.positions[1]) < 1e-10);
        }
    }
}

#[test]
fn trim_refuses_coincident_carriers_and_keeps_unresolved_spans() {
    let s = sweep(); let pair = walls(&s);
    for options in [TraceOptions {max_solves:0,..sampling()},TraceOptions {max_solves:4,..sampling()},
        TraceOptions {max_solves:30,..sampling()},
        TraceOptions {max_depth:0,..sampling()}] {
        let trace = pair.trace([0.05,0.45],[[0.5,0.05];2],options).unwrap();
        assert!(!trace.unresolved.is_empty()); assert!(trace.evaluations <= options.max_solves);
        partition(&trace,[0.05,0.45]);
        if options.max_solves == 30 { assert!(!trace.segments.is_empty()); }
    }
    assert!(pair.at(2.,[[0.5;2];2]).is_err());
    let shallow = Pair::new(&s,pair.charts(),pair.bounds(),1,
        Options {minimum_sine:1.,..Options::default()}).unwrap();
    assert!(matches!(shallow.at(0.25,[[0.5,0.25];2]),Err(Error::NonTransverse)));
    // Same carrier at the same time has a 2D overlap, not an isolated trim curve.
    let chart = Chart::Endpoint {face:1,end:sb::End::From};
    let same = Pair::new(&s,[chart;2],[[0.,1.];4],1,Options::default()).unwrap();
    assert!(matches!(same.at(0.25,[[0.5,0.25];2]),Err(Error::Solve(gcs_core::envelope::Error::SingularIntersection)) | Err(Error::NonTransverse)));
    // Distinct parameter bands of one carried rim ALSO overlap in 2D. They
    // lie on the same sphere, although their generators and roll values differ.
    let mut seed = [[0.03,0.],[0.97,0.]];
    for p in &mut seed {
        let q = s.source_at(SourcePoint {source:Source::Edge(0),parameters:[p[0],0.]},0.).unwrap().faces[0].position;
        p[1] = (q[1]/q[2]).atan();
    }
    let h = s.domain()[1];
    let rim = Pair::new(&s,[Chart::Edge(0);2],[[0.01,0.1],[-h,h],[0.9,0.99],[-h,h]],0,Options::default()).unwrap();
    let result = rim.at(seed[0][0],seed);
    assert!(matches!(result,Err(Error::Solve(gcs_core::envelope::Error::SingularIntersection)) | Err(Error::NonTransverse)),"{result:?}");
}

#[test]
fn edge_contact_events_are_isolated_over_the_whole_roll_interval() {
    let s = sweep(); let roll = I::new(s.domain()[0],s.domain()[1]).unwrap();
    let options = a::edge::Options {parameter_width:1e-12,max_steps:32};
    for edge in 0..s.edges().len() { for face in s.edges()[edge].faces {
        let root = a::edge::isolate(&s,edge,face,I::new(0.49,0.51).unwrap(),roll,options).unwrap();
        assert!(root.resolved(),"{root:?}"); assert!(root.enclosure().contains(0.5));
        assert!(!root.derivative().contains(0.));
        let ends = root.endpoint_values().map(|x| x.bounds());
        assert!((ends[0][1] < 0. && ends[1][0] > 0.) || (ends[0][0] > 0. && ends[1][1] < 0.));
        for t in [s.domain()[0],0.,s.domain()[1]] {
            let [lo,hi] = root.enclosure().bounds(); let u = 0.5*(lo+hi);
            let q = s.source_at(SourcePoint {source:Source::Edge(edge),parameters:[u,0.]},t).unwrap();
            for consumer in &q.faces {
                assert!(harness::distance(consumer.position,q.faces[0].position) < 1e-12);
            }
            let consumer = q.faces.iter().find(|p| p.face == face).unwrap();
            let [u,v] = consumer.parameters;
            let c = gcs_core::envelope::contact(s.faces()[face].at(u,v).unwrap(),s.motion().at(t).unwrap()).unwrap();
            assert!(c.normal_velocity.abs() < 1e-10);
        }
    } }
}

#[test]
fn edge_event_limits_do_not_manufacture_a_trim_vertex() {
    let s = sweep(); let roll = I::new(s.domain()[0],s.domain()[1]).unwrap();
    let face = s.edges()[0].faces[0]; let domain = I::new(0.49,0.51).unwrap();
    let options = a::edge::Options {parameter_width:1e-12,max_steps:0};
    let root = a::edge::isolate(&s,0,face,domain,roll,options).unwrap();
    assert!(!root.resolved()); assert_eq!(root.enclosure(),domain); assert_eq!(root.steps(),0);
    assert!(matches!(a::edge::isolate(&s,0,face,I::new(0.,0.01).unwrap(),roll,options),Err(a::edge::Error::Unbracketed)));
    assert!(a::edge::isolate(&s,0,face,I::new(0.1,0.4).unwrap(),roll,options).is_err());
    assert!(a::edge::isolate(&s,0,face,domain,I::new(-2.,2.).unwrap(),options).is_err());
}

#[test]
fn global_hiding_requires_an_actual_strict_interior_witness() {
    let s = sweep();
    // This wall point is in local contact at the start pose (source z=0).
    // It is nevertheless strictly covered by the tool at an intermediate time.
    let (_,_,q) = Chart::Endpoint {face:1,end:sb::End::From}.at(&s,[0.5,0.25],1e-9).unwrap();
    let point = q.position.map(|x| I::new(x-1e-5,x+1e-5).unwrap());
    let report = a::hiding(&s,point,minimum::Options {value_tolerance:1e-9,max_evaluations:1000}).unwrap();
    let witness = report.witness.expect("hidden contact must retain its covering pose");
    assert!(witness.value().bounds()[1] < 0.);
    let pose = s.motion().bounds(I::point(witness.roll()).unwrap()).unwrap();
    assert_eq!(s.source_material().bounds(pose.inverse_point(witness.point()).unwrap()).unwrap(),witness.value());
    // The same hidden center cannot license discarding a box that also contains
    // exterior material. The x=5 side is outside this cylinder at every roll.
    let mut wide = point; wide[0] = I::new(2.,5.).unwrap();
    assert!(a::hiding(&s,wide,minimum::Options {value_tolerance:1e-9,max_evaluations:1000}).unwrap().witness.is_none());
    // A source point on the rotation axis stays on the boundary. No strict
    // witness is found, and there is no API that promotes this to visibility.
    let point = [4.,0.,0.].map(|x| I::point(x).unwrap());
    assert!(a::hiding(&s,point,minimum::Options {value_tolerance:1e-9,max_evaluations:1000}).unwrap().witness.is_none());
    // A tiny budget at the endpoint-wall intersection preserves uncertainty.
    let point = [3.,-1./s.domain()[1].cos(),0.].map(|x| I::point(x).unwrap());
    let limited = a::hiding(&s,point,minimum::Options {value_tolerance:1e-12,max_evaluations:4}).unwrap();
    assert!(limited.witness.is_none()); assert_eq!(limited.minimum.status,minimum::Status::BudgetExhausted);
    assert!(limited.minimum.value.contains(0.));
}
