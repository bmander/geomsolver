//! A station's contact curve and the sheet of them (`solid::contact_trace`), on the torus rolled
//! about a skew axis through a post: the admitted small fixture, whose sections are circles a
//! test can sample exactly, so the tracer is checked without a kernel and in seconds.
use fixtures::{tools::{skew_post,torus},motions::{Observer,cradle_roll}};
use gcs_core::solid::{SweepContacts,contact_trace::{Band,Extent,Grid,Rows,Sample,Station,Tracer,TraceError,Withheld,STEP_TIME,charted,marked}};
use std::f64::consts::{PI,TAU};

type V = [f64;3];

fn sweep() -> SweepContacts {
    let source = format!("{}{}construction removal := solid(tool, under: turn, from: -75deg, to: 75deg)\n{}",torus(2.),cradle_roll(0.25,Observer::Skew),skew_post(4.));
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
    let band = Band {stations:[hits[0],*hits.last().unwrap()],radius:1.,limit:std::f64::consts::TAU};
    for placement in [Rows::Walk,Rows::Length] {
        let sheet = tracer.sheet(&|a| Ok(station(a)),band,0.5,(band.stations[1]-band.stations[0])*0.15,placement).unwrap();
        eprintln!("{placement:?} rows: sheet {}x{}, {} withheld",sheet.rows,sheet.columns,sheet.withheld.len());
        assert_eq!(sheet.points.len(),sheet.rows*sheet.columns);
        assert!(!sheet.withheld.is_empty());
        assert!(sheet.chart_fault(&inside).unwrap().is_none());
        // The margin too is one chart: no column's time leaps between rows, in the blank or out.
        for c in 0..sheet.columns { for r in 1..sheet.rows {
            assert!((sheet.times[r*sheet.columns+c]-sheet.times[(r-1)*sheet.columns+c]).abs() < STEP_TIME);
        } }
        assert_eq!(sheet.withheld.len() % (sheet.rows-1),0);
        // Length rows are even in space down every column.
        if placement == Rows::Length { for c in 0..sheet.columns {
            let steps: Vec<f64> = (1..sheet.rows).map(|r| distance(sheet.points[r*sheet.columns+c],sheet.points[(r-1)*sheet.columns+c])).collect();
            let (least,most) = steps.iter().fold((f64::INFINITY,0_f64),|(l,m),s| (l.min(*s),m.max(*s)));
            assert!(most < 1.5*least,"column {c}: steps {least} to {most}");
        } }
    }
}

fn distance(a: V,b: V) -> f64 { (0..3).map(|k| (a[k]-b[k]).powi(2)).sum::<f64>().sqrt() }

/// The configured gear's sheet failed its fit at the root fillet, in the blank, because one
/// column in the margin leapt a turn round into a far corner's fan on its last row: the fit's
/// chord-length parameters are averaged over the columns, so that row moved every row's
/// (docs/native-hypoid-plan.md). A sheet's rows stop before such a leap outside the
/// blank; a row holding a contact in the blank is never trimmed.
#[test]
fn a_sheet_keeps_its_rows_only_while_the_margin_is_one_chart() {
    let steady = |n: usize| (0..n).map(|r| 0.01*r as f64).collect::<Vec<f64>>();
    let mut times = vec![steady(8),steady(8),steady(8)];
    let mut within = vec![vec![false;8];3];
    within[1][2] = true; within[1][4] = true;
    assert_eq!(charted(&times,&within),[0,7]);
    // A leap into the last row of one column, as the gear's last station took: that row goes.
    times[2][7] = 3.2;
    assert_eq!(charted(&times,&within),[0,6]);
    // One two rows out, before the first row in the blank: those rows go too.
    times[0][0] = -2.;
    assert_eq!(charted(&times,&within),[1,6]);
    times[0][1] = -2.;
    assert_eq!(charted(&times,&within),[2,6]);
    // A leap between rows in the blank stays for the chart contract to refuse.
    times[1][3] = 1.5;
    assert_eq!(charted(&times,&within),[2,6]);
    // A sheet with no contact in the blank keeps every row.
    assert_eq!(charted(&times,&vec![vec![false;8];3]),[0,7]);
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

/// A sheet held to a tolerance withholds the middles of its
/// cells' sides as well as their centres, and is refined where they miss. A contact in the middle
/// of a column's step between two rows marks those rows, one in the middle of a row's step marks
/// those columns, and a centre whose sides are both clear marks both; the grid splits what is
/// marked at the withheld coordinate, and grades so no interval is more than twice its neighbour.
#[test]
fn a_sheet_is_refined_where_its_withheld_contacts_miss() {
    // Rows 0..=4 and columns 0..=3: sites in half steps.
    let sites = [[1,0],[1,2],[3,1],[5,5],[7,3],[2,5]];
    let over = [true,false,false,true,false,false];
    // A side down column 0 between rows 0 and 1 marks row interval 0; the centre (5,5) of cell
    // (2,2), whose sides were clear, marks row interval 2 and column interval 2.
    assert_eq!(marked(&sites,&over,5,4),(vec![true,false,true,false],vec![false,false,true]));
    // A side across row 1 between columns 2 and 3 marks column interval 2 alone, and a centre in a
    // row a side has marked adds nothing.
    let over = [true,false,false,false,false,true];
    assert_eq!(marked(&sites,&over,5,4),(vec![true,false,false,false],vec![false,false,true]));
    let sites = [[1,0],[1,1]];
    assert_eq!(marked(&sites,&[true,true],2,2),(vec![true],vec![false]));

    let even = |n: usize| -> (Vec<f64>,Vec<f64>) { ((0..n).map(|i| i as f64).collect(),(0..n-1).map(|i| i as f64+0.5).collect()) };
    let ((rows,row_mids),(columns,column_mids)) = (even(5),even(4));
    let grid = Grid {rows,row_mids,columns,column_mids};
    // The withheld coordinate becomes a node, and each half gets its own middle.
    let once = grid.refined(&[false,true,false,false],&[false,false,false]);
    assert_eq!(once.rows,vec![0.,1.,1.5,2.,3.,4.]);
    assert_eq!(once.row_mids,vec![0.5,1.25,1.75,2.5,3.5]);
    assert_eq!((once.columns.clone(),once.column_mids.clone()),(grid.columns.clone(),grid.column_mids.clone()));
    // Split again, the quarter intervals would sit beside whole ones: the neighbours split too.
    let twice = once.refined(&[false,true,false,false,false],&[false;3]);
    assert_eq!(twice.rows,vec![0.,0.5,1.,1.25,1.5,2.,3.,4.]);
    for w in twice.rows.windows(3) { assert!((w[2]-w[1]) <= 2.*(w[1]-w[0])+1e-12 && (w[1]-w[0]) <= 2.*(w[2]-w[1])+1e-12,"{w:?}"); }
    assert_eq!(twice.row_mids.len(),twice.rows.len()-1);
    // Nothing marked, nothing moves.
    assert_eq!(grid.refined(&[false;4],&[false;3]),grid);
}

/// The torus rolled through the post, laid out once and read on its first grid and a refined one:
/// the sides' withheld contacts are where their sites say, a refined grid's new nodes are exactly
/// the contacts the coarser grid withheld there, and every node of the coarser grid is kept.
#[test]
fn a_refined_sheet_reads_its_new_nodes_where_the_coarse_one_withheld_them() {
    let sweep = sweep();
    let inside = |points: &[V]| Ok(points.iter().map(in_post).collect());
    let tracer = Tracer {sweep:&sweep,scale:1.,inside:&inside,debug:false};
    let hits: Vec<f64> = (0..96).map(|k| TAU*(k as f64+0.5)/96.)
        .filter(|&a| tracer.trace(&station(a),0.5,Extent::Blank).is_ok()).collect();
    let band = Band {stations:[hits[0],*hits.last().unwrap()],radius:1.,limit:std::f64::consts::TAU};
    let at = |a: f64| Ok(station(a));
    let layout = tracer.layout(&at,band,0.5,(band.stations[1]-band.stations[0])*0.15,Rows::Walk).unwrap();
    let centres = layout.sheet(&layout.grid,Withheld::Centres).unwrap();
    let coarse = layout.sheet(&layout.grid,Withheld::Sides).unwrap();
    let (rows,columns) = (coarse.rows,coarse.columns);
    // The centres are the centres-only sheet's, in its order.
    let at_centres: Vec<V> = coarse.sites.iter().zip(&coarse.withheld).filter(|(s,_)| s[0]%2 == 1 && s[1]%2 == 1).map(|(_,p)| *p).collect();
    assert_eq!(at_centres,centres.withheld);
    assert!(centres.sites.iter().all(|s| s[0]%2 == 1 && s[1]%2 == 1));
    assert_eq!(coarse.points,centres.points);
    let count = |odd_row: bool,odd_column: bool| coarse.sites.iter().filter(|s| (s[0]%2 == 1) == odd_row && (s[1]%2 == 1) == odd_column).count();
    assert_eq!(count(true,false),(rows-1)*columns);
    assert!(count(false,true) > 0 && count(false,true) % rows == 0 && count(true,true) % (rows-1) == 0);
    // A node station's side contact lies on its column between the nodes it is withheld between.
    for (site,p) in coarse.sites.iter().zip(&coarse.withheld).filter(|(s,_)| s[0]%2 == 1 && s[1]%2 == 0) {
        let (r,c) = (site[0]/2,site[1]/2);
        let (a,b) = (coarse.points[r*columns+c],coarse.points[(r+1)*columns+c]);
        assert!(distance(*p,a) <= distance(a,b)+1e-9 && distance(*p,b) <= distance(a,b)+1e-9);
        assert!(coarse.spacing(*site) <= distance(a,b)+1e-12);
    }
    // Refine one row interval and one column interval.
    let (r,c) = (rows/2,columns/2);
    let mut marks = (vec![false;rows-1],vec![false;columns-1]);
    marks.0[r] = true; marks.1[c] = true;
    let grid = layout.grid.refined(&marks.0,&marks.1);
    let fine = layout.sheet(&grid,Withheld::Sides).unwrap();
    assert_eq!((fine.rows,fine.columns),(rows+1,columns+1));
    let withheld_at = |site: [usize;2]| coarse.sites.iter().position(|s| *s == site).map(|i| coarse.withheld[i]);
    // The new row: the coarse sheet's side contacts down every column (and its centres between).
    for k in 0..=columns {
        let new = fine.points[(r+1)*fine.columns+if k <= c { k } else { k+1 }];
        if let Some(p) = withheld_at([2*r+1,2*k]) { assert_eq!(p,new); }
    }
    // The new column: the coarse sheet's mid station, at the rows its side contacts were at.
    for k in 0..rows {
        let new = fine.points[(if k <= r { k } else { k+1 })*fine.columns+c+1];
        if let Some(p) = withheld_at([2*k,2*c+1]) { assert_eq!(p,new); }
    }
    // Every coarse node is still a node.
    for i in 0..rows { for j in 0..columns {
        let (fi,fj) = (if i <= r { i } else { i+1 },if j <= c { j } else { j+1 });
        assert_eq!(coarse.points[i*columns+j],fine.points[fi*fine.columns+fj]);
    } }
    assert!(fine.chart_fault(&inside).unwrap().is_none());
}
