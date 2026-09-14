//! Common-carrier domain coverage, independently judged in a radial chart.
use super::{harness,creases};
use gcs_core::solid::{SweepContacts,swept_boundary::arrangement::carrier::{
    Correspondence,Band,Options,union::{Union,Options as UnionOptions}}};
fn options() -> Options { Options {max_depth:16,max_cells:4096} }
fn union_options() -> UnionOptions { UnionOptions {max_events:4096,max_cells:4096} }
fn read() -> SweepContacts {
    let e = harness::read(&creases::tumbling_cylinder());
    SweepContacts::read(&e.sketch,harness::solid(&e,"swept"),1e-10).unwrap()
}
fn bands(s:&SweepContacts) -> [Band;2] {
    [Band {edge:0,domain:[[0.01,0.1],s.domain()]},
        Band {edge:0,domain:[[0.9,0.99],s.domain()]}]
}
#[test]
fn original_two_bands_form_an_inspectable_union() {
    let s = read(); let c = Correspondence::new(&s,&bands(&s),options()).unwrap();
    let u = Union::build(&c,union_options()).unwrap();
    eprintln!("cells={} edges={} vertices={} loops={} area={}",u.cells().len(),u.edges().len(),u.vertices().len(),u.boundaries().len(),u.chart_area());
    export(&c,&u);
    assert_eq!(u.boundaries().len(),1);
    assert!(u.cells().iter().any(|c| c.consumers.len() == 2));
    assert!(u.cells().iter().any(|c| c.consumers.len() == 1));
    for (i,cell) in u.cells().iter().enumerate() {
        let maps = u.consumers_at(i,[0.5,0.5],1e-9).unwrap();
        assert_eq!(maps.len(),cell.consumers.len());
        for m in &maps { assert!(harness::distance(m.evaluation.position,maps[0].evaluation.position) < 1e-9); }
    }
}
fn area(u:&Union<'_,'_>,i:usize) -> f64 {
    let a = u.coordinates(i,[0.,0.]).unwrap(); let b = u.coordinates(i,[1.,0.]).unwrap();
    let c = u.coordinates(i,[1.,1.]).unwrap(); let d = u.coordinates(i,[0.,1.]).unwrap();
    (b[0]-a[0])*0.5*(c[1]-b[1]+d[1]-a[1])
}
fn judge(c:&Correspondence<'_>,u:&Union<'_,'_>) {
    // Independent nominal-cylinder projection, not the arrangement's line/cut data.
    // A grid detects omitted and multiply emitted regions as well as lost consumers.
    let tau = std::f64::consts::TAU;
    for iw in 0..81 { for iy in 0..93 {
        let w = (iw as f64+0.371)/81.*0.7;
        let y = -1.2+(iy as f64+0.213)/93.*2.4;
        let expected:Vec<_> = c.regions().iter().enumerate().filter(|(_,r)| {
            let d = r.domain(); let mut ends = d[0].map(|t| (tau*t).sin().abs());
            ends.sort_by(f64::total_cmp);
            let q = -(tau*0.5*(d[0][0]+d[0][1])).sin().signum()*w;
            let mut ys = d[1].map(|r| q*r.cos()+r.sin()); ys.sort_by(f64::total_cmp);
            ends[0] < w && w < ends[1] && ys[0] < y && y < ys[1]
        }).map(|(i,_)| i).collect();
        let actual:Vec<_> = u.cells().iter().enumerate().filter(|(i,_)| {
            let a = u.coordinates(*i,[0.,0.]).unwrap(); let b = u.coordinates(*i,[1.,0.]).unwrap();
            if !(a[0] < w && w < b[0]) { return false; }
            let t = (w-a[0])/(b[0]-a[0]);
            let lo = u.coordinates(*i,[t,0.]).unwrap()[1]; let hi = u.coordinates(*i,[t,1.]).unwrap()[1];
            lo < y && y < hi
        }).collect();
        assert_eq!(actual.len(),usize::from(!expected.is_empty()),"coverage at {w},{y}: {expected:?}");
        if let Some((_,cell)) = actual.first() { assert_eq!(cell.consumers,expected,"consumers at {w},{y}"); }
    } }
    for (r,region) in c.regions().iter().enumerate() {
        let d = region.domain(); let mut ends = d[0].map(|t| (tau*t).sin().abs()); ends.sort_by(f64::total_cmp);
        let sign = -(tau*0.5*(d[0][0]+d[0][1])).sin().signum();
        let [lo,hi] = d[1];
        let exact = (sign*0.5*(hi.cos()-lo.cos())*(ends[1]*ends[1]-ends[0]*ends[0])
            +(hi.sin()-lo.sin())*(ends[1]-ends[0])).abs();
        let covered:f64 = u.cells().iter().enumerate().filter(|(_,c)| c.consumers.contains(&r)).map(|(i,_)| area(u,i)).sum();
        assert!((covered-exact).abs() < 2e-14,"region {r}: {covered} vs {exact}");
    }
    for (id,e) in u.edges().iter().enumerate() {
        assert!((1..=2).contains(&e.uses.len()));
        if e.uses.len() == 2 { assert_ne!(e.uses[0].forward,e.uses[1].forward); }
        for t in [0.,0.37,1.] {
            let maps = u.edge_at(id,t,1e-9).unwrap();
            for m in &maps { assert!(harness::distance(m.evaluation.position,maps[0].evaluation.position) < 2e-9); }
        }
    }
    for boundary in u.boundaries() {
        let ends:Vec<_> = boundary.edges.iter().map(|&(id,forward)| {
            let e = &u.edges()[id]; assert_eq!(e.uses.len(),1);
            if forward {e.vertices} else {[e.vertices[1],e.vertices[0]]}
        }).collect();
        for i in 0..ends.len() { assert_eq!(ends[i][1],ends[(i+1)%ends.len()][0]); }
    }
}
#[test]
fn overlap_area_and_consumer_coverage_match_the_analytic_rim() {
    let s = read(); let c = Correspondence::new(&s,&bands(&s),options()).unwrap();
    let u = Union::build(&c,union_options()).unwrap(); judge(&c,&u);
    let a = (std::f64::consts::TAU*0.01).sin(); let b = (std::f64::consts::TAU*0.1).sin();
    let (sine,cosine) = s.domain()[1].sin_cos(); let cross = sine/cosine;
    let expected = cosine*(cross*cross-a*a)+2.*sine*(cross-a)+4.*sine*(b-cross);
    assert!((u.chart_area()-expected).abs() < 2e-14);
    // The original periodic-side endpoints differ in the represented geometry.
    // At least one retained cell is thinner than a binary64 spatial welding tolerance.
    assert!(u.cells().iter().enumerate().any(|(i,_)| {
        let a = u.coordinates(i,[0.,0.]).unwrap()[0]; let b = u.coordinates(i,[1.,0.]).unwrap()[0];
        b-a > 0. && b-a < 1e-14
    }));
}
#[test]
fn input_order_and_inverse_seeds_do_not_change_the_union() {
    let s = read(); let b = bands(&s);
    let c = Correspondence::new(&s,&b,options()).unwrap();
    let r = Correspondence::new(&s,&[b[1],b[0]],options()).unwrap();
    let u = Union::build(&c,union_options()).unwrap(); let v = Union::build(&r,union_options()).unwrap();
    assert_eq!(u.vertices().iter().map(|v| v.coordinates).collect::<Vec<_>>(),v.vertices().iter().map(|v| v.coordinates).collect::<Vec<_>>());
    assert_eq!(u.edges().iter().map(|e| (e.curve,e.vertices)).collect::<Vec<_>>(),v.edges().iter().map(|e| (e.curve,e.vertices)).collect::<Vec<_>>());
    let (mut solved,mut unresolved) = (0,0);
    for (i,cell) in u.cells().iter().enumerate() {
        let mut expected:Vec<_> = cell.consumers.iter().map(|&r| 1-r).collect(); expected.sort();
        assert_eq!(expected,v.cells()[i].consumers);
        for (consumer,m) in cell.consumers.iter().zip(u.consumers_at(i,[0.5,0.5],1e-9).unwrap()) {
            let d = c.regions()[*consumer].domain();
            for fraction in [0.,0.2,0.5,0.8,1.] {
                let seed = d.map(|b| b[0]*(1.-fraction)+b[1]*fraction);
                match c.inverse(*consumer,m.coordinates,seed,1e-9) {
                    Ok(other) => {
                        solved += 1;
                        assert!(harness::distance(m.evaluation.position,other.evaluation.position) < 2e-9);
                    }
                    // A bounded local solve can fail from a distant corner seed.
                    // It cannot remove a cell or a consumer from this analytic union.
                    Err(gcs_core::solid::swept_boundary::arrangement::carrier::Error::Solve(
                        gcs_core::envelope::Error::NotConverged)) => { unresolved += 1; }
                    other => panic!("unexpected inverse result: {other:?}"),
                }
            }
        }
    }
    assert!(solved >= u.cells().len());
    eprintln!("inverse seed trials: {solved} solved, {unresolved} explicitly unresolved");
    let rebuilt = Union::build(&c,union_options()).unwrap();
    assert_eq!(u.cells().iter().map(|c| &c.consumers).collect::<Vec<_>>(),rebuilt.cells().iter().map(|c| &c.consumers).collect::<Vec<_>>());
    assert_eq!(u.chart_area(),rebuilt.chart_area());
}
#[test]
fn disjoint_nested_and_repeated_domains_keep_their_own_consumers() {
    let s = read(); let base = bands(&s)[0];
    let cases = [
        // Disjoint radial intervals; then strict containment; then duplicate domains.
        (vec![Band {domain:[[0.01,0.03],s.domain()],..base},Band {domain:[[0.06,0.1],s.domain()],..base}],2,1),
        (vec![base,Band {domain:[[0.03,0.07],[-0.2,0.2]],..base}],1,2),
        (vec![base,base],1,2),
        // Adjacent pieces repeat a complete source boundary, which must cancel.
        (vec![Band {domain:[[0.01,0.05],s.domain()],..base},Band {domain:[[0.05,0.1],s.domain()],..base}],1,1),
    ];
    for (bands,loops,multiplicity) in cases {
        let c = Correspondence::new(&s,&bands,options()).unwrap(); let u = Union::build(&c,union_options()).unwrap();
        assert_eq!(u.boundaries().len(),loops); assert_eq!(u.cells().iter().map(|c| c.consumers.len()).max(),Some(multiplicity));
        judge(&c,&u);
    }
}
#[test]
fn unresolved_domains_and_resource_limits_refuse_a_complete_union() {
    use gcs_core::solid::swept_boundary::arrangement::carrier::union::Error;
    let s = read(); let c = Correspondence::new(&s,&bands(&s),Options {max_depth:0,max_cells:1}).unwrap();
    assert!(!c.unresolved().is_empty());
    assert!(matches!(Union::build(&c,union_options()),Err(Error::CorrespondenceUnresolved)));
    // The full native cover, including unresolved boxes, remains on the input.
    assert_eq!(c.bands().len(),2); assert_eq!(c.unresolved()[0].domain,bands(&s)[1].domain);
    let c = Correspondence::new(&s,&bands(&s),options()).unwrap();
    for opts in [UnionOptions {max_events:1,max_cells:4096},UnionOptions {max_events:4096,max_cells:1}] {
        assert!(matches!(Union::build(&c,opts),Err(Error::Budget)));
    }
    let singular = [Band {edge:0,domain:[[0.,0.05],s.domain()]}];
    let c = Correspondence::new(&s,&singular,options()).unwrap();
    assert!(c.unresolved().iter().any(|r| r.domain[0][0] == 0.));
    assert!(matches!(Union::build(&c,union_options()),Err(Error::CorrespondenceUnresolved)));
    let mut b = bands(&s); b[1] = Band {edge:1,domain:b[0].domain};
    let c = Correspondence::new(&s,&b,options()).unwrap();
    assert!(matches!(Union::build(&c,union_options()),Err(Error::UnsupportedChart)));
}
#[test]
fn source_dimensions_and_motion_phase_are_read_from_the_snapshot() {
    use super::{tools,motions};
    for (radius,height,rate,phase) in [(0.8,1.2,1.,0.),(1.,1.,-1.,13.),(1.,1.,0.5,-9.)] {
        let tool = tools::CYLINDER.replace("x: 4,",&format!("x: {},",3.+radius))
            .replace("y: -1)",&format!("y: {})",-height)).replace("y: 1)",&format!("y: {height})"));
        let motion = motions::TUMBLE.replace("motion turn(about: tumbler)",
            &format!("motion turn(about: tumbler, ratio: {rate}, phase: {phase}deg)"));
        let e = harness::read(&format!("{tool}{motion}{}",motions::swept("turn",-30.,30.)));
        let s = SweepContacts::read(&e.sketch,harness::solid(&e,"swept"),1e-10).unwrap();
        let c = Correspondence::new(&s,&bands(&s),options()).unwrap();
        let u = Union::build(&c,union_options()).unwrap();
        for i in 0..u.cells().len() {
            for uv in [[0.23,0.31],[0.5,0.5],[0.73,0.81]] {
                let maps = u.consumers_at(i,uv,1e-9).unwrap();
                for m in &maps {
                    let p = m.evaluation.position;
                    assert!(((p[0]-3.).powi(2)+p[1]*p[1]+p[2]*p[2]-radius*radius-height*height).abs() < 1e-8);
                    assert!(harness::distance(p,maps[0].evaluation.position) < 2e-9);
                }
            }
        }
    }
}

fn export(c:&Correspondence<'_>,u:&Union<'_,'_>) {
    let Some(path) = std::env::var_os("SOLVENT_CARRIER_EXPORT") else { return; };
    let path = std::path::PathBuf::from(path); std::fs::create_dir_all(&path).unwrap();
    let mut table = String::from("kind\tid\tdata\n");
    for (i,v) in u.vertices().iter().enumerate() { table.push_str(&format!("vertex\t{i}\t{:?} enclosures={:?}\n",v.coordinates,v.enclosure)); }
    for (i,e) in u.edges().iter().enumerate() { table.push_str(&format!("edge\t{i}\t{:?} vertices={:?} uses={:?}\n",e.curve,e.vertices,e.uses)); }
    for (i,cell) in u.cells().iter().enumerate() {
        table.push_str(&format!("cell\t{i}\tconsumers={:?} edges={:?}\n",cell.consumers,cell.edges));
        for (&region,m) in cell.consumers.iter().zip(u.consumers_at(i,[0.5,0.5],1e-9).unwrap()) {
            table.push_str(&format!("map\t{i}:{region}\tband={} native={:?} position={:?}\n",c.regions()[region].band(),m.native,m.evaluation.position));
        }
    }
    for (i,b) in u.boundaries().iter().enumerate() { table.push_str(&format!("boundary\t{i}\t{:?}\n",b.edges)); }
    std::fs::write(path.join("union.tsv"),table).unwrap();
    // This drawing is a chart diagnostic. Lines become curved arcs in 3D.
    let point = |p:[f64;2]| format!("{:.8},{:.8}",80.+1000.*p[0],330.-240.*p[1]);
    let mut svg = String::from(r##"<svg xmlns="http://www.w3.org/2000/svg" width="800" height="680" viewBox="0 0 800 680">
<rect width="800" height="680" fill="white"/>
<g font-family="sans-serif" fill="#18232e"><text x="45" y="35" font-size="22">Carried-rim domain union</text>
<text x="45" y="60" font-size="14">Two native bands · one common chart · seven cells · one boundary loop</text>
<text x="45" y="630" font-size="14">Blue: band 0 only   Orange: band 1 only   Purple: both consumers</text>
<text x="45" y="655" font-size="13">Thin periodic-end cells remain distinct; this drawing cannot resolve their width.</text></g>
<path d="M80 85 V590 H725" fill="none" stroke="#607080"/>
<g font-family="sans-serif" font-size="14" fill="#18232e"><text x="610" y="612">w = |source y|</text><text x="20" y="95">posed y</text></g>
"##);
    for (i,cell) in u.cells().iter().enumerate() {
        let color = match cell.consumers.as_slice() { [0] => "#70b7e5",[1] => "#edb471",_ => "#b096d4" };
        let points = [[0.,0.],[1.,0.],[1.,1.],[0.,1.]].map(|uv| point(u.coordinates(i,uv).unwrap())).join(" ");
        svg.push_str(&format!("<polygon points=\"{points}\" fill=\"{color}\" stroke=\"#ffffff\" stroke-width=\"1\"><title>Cell {i}, consumers {:?}</title></polygon>\n",cell.consumers));
    }
    for b in u.boundaries() { for &(id,_) in &b.edges {
        let e = &u.edges()[id]; let points = e.vertices.map(|v| point(u.vertices()[v].coordinates)).join(" ");
        svg.push_str(&format!("<polyline points=\"{points}\" fill=\"none\" stroke=\"#18232e\" stroke-width=\"2\"><title>Shared edge {id}</title></polyline>\n"));
    } }
    svg.push_str("</svg>\n"); std::fs::write(path.join("union.svg"),svg).unwrap();
}

#[test]
fn point_touching_domains_keep_a_singular_junction_unresolved() {
    use gcs_core::solid::swept_boundary::arrangement::carrier::union::Error;
    let s = read();
    let b = [Band {edge:0,domain:[[0.01,0.05],[-0.2,0.]]},
        Band {edge:0,domain:[[0.05,0.1],[0.,0.2]]}];
    let c = Correspondence::new(&s,&b,options()).unwrap();
    assert!(c.unresolved().is_empty()); assert_eq!(c.regions().len(),2);
    assert!(matches!(Union::build(&c,union_options()),Err(Error::Topology)));
}
