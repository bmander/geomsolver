//! A screw cut's sheet: the tool's characteristic (`characteristic`) carried along the
//! screw, S(s, t) = M(t)·c(s), its normal the tool's there carried with it. Every node and every
//! withheld point is exact: a point of the characteristic is a root of its ring in closed form, and
//! a pose of the screw is a formula. Nothing is traced. Rows run along the characteristic by
//! length in space, and columns along the roll over the stretch that carries it past the blank.
//! The grid is fitted, judged and refined where it misses (`sheet::settle`), as a traced sheet is.
//! A turn's and a slide's sheets are built the same way (#684): a turn's section is a half-plane
//! through its axis, so its sheet is that section revolved, and a slide's lies square to it, so
//! its sheet is that section extruded, its columns spread across the blank's heights. A turn
//! carried round a whole turn or more sweeps a solid of revolution (#684 N): its columns run once
//! round, the last pose the first, and the sheet is cut into faces round the turn that close on
//! each other, so no face of it has a seam for the split to meet.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use super::sheet::{Fitted,SweptCut,settle};
use super::Say;
use crate::sweep::characteristic::Characteristic;
use crate::sweep::trace::{Grid,Inside,Sheet,UNNAMED,Withheld};
use crate::sweep::refusal::{AtStage,ExportRefusal,Stage,Tolerance};
use crate::util::space::distance;

type V = [f64;3];

/// How far the first grid's columns turn the screw at most (radians), and how many rows it lays
/// along the characteristic.
const COLUMN_TURN: f64 = 10./180.*std::f64::consts::PI;
const FIRST_ROWS: usize = 24;
/// How many first columns' widths the roll's stretch is widened past where the characteristic
/// leaves the blank's heights.
const COLUMN_MARGIN: f64 = 2.;
/// How many faces a whole turn's sheet is cut into round the turn: two at least, so that it
/// closes on an edge and not a seam, and few enough that no face turns more than a quarter turn.
const AROUND: usize = 4;
/// How many columns a slide's first grid lays across the blank's heights: its sheet is straight
/// along them, so the fit needs few.
const SLIDE_COLUMNS: f64 = 8.;

/// The sheet of a screw cut whose characteristic `found` walked (in model units, as `cut.scale`
/// says millimetres a unit), the blank between `heights` along the screw's axis where known.
pub fn helical_sheet(cut: &SweptCut,found: &Characteristic,heights: Option<[f64;2]>,inside: Inside,near: &(dyn Fn(V) -> f64+Sync),
    tolerance: Option<Tolerance>,say: &Say) -> Result<Fitted,ExportRefusal> {
    let Laid {station,fans,grid,sheet} = laid(cut,found,heights,inside,say)?;
    let (scale,name,family) = (cut.scale,&cut.name,&cut.family);
    let sides = |grid: &Grid| lay(found,family,scale,station,&fans,grid,Withheld::Sides);
    settle(name,scale,sheet,&grid,&sides,inside,near,tolerance,"rows along the characteristic",say)?.and_then(|(mut fitted,kind)| {
        if fans.is_empty() && !found.whole { return Ok(fitted) }
        fitted.faces = fitted.named(kind,&|_| String::new(),found.whole.then_some(AROUND)).at(Stage::Fit)?;
        (say.stage)(&format!("`{name}`: the sheet cut into {} faces{}{}",fitted.faces.faces.len(),
            if fans.is_empty() { "" } else { " where its characteristic runs round a convex corner's fan" },
            if found.whole { " and round the turn, closing on itself" } else { "" }));
        Ok(fitted)
    })
}

/// A screw cut's sheet laid out: the station its section stands at, where the section runs onto
/// and off a convex corner's fan, its first grid (rows along the section, columns along the roll
/// over the stretch that carries it past the blank), and that grid's sheet, its edges clear of the
/// blank.
struct Laid { station: f64,fans: Vec<f64>,grid: Grid,sheet: Sheet }

/// The layout `helical_sheet` fits and `carried_sheet` carries.
fn laid(cut: &SweptCut,found: &Characteristic,heights: Option<[f64;2]>,inside: Inside,say: &Say) -> Result<Laid,ExportRefusal> {
    let (scale,name,screw,family) = (cut.scale,&cut.name,found.screw,&cut.family);
    // rows along the sheet's section at the reach's first point's station (`Screw::station`: its
    // height, or a turn's angle): the characteristic's points each carried along its path there,
    // `on_section`. A column is that section carried by the motion: S(s, t) = M(t)·σ(s), the same
    // surface as M(t)·c(s).
    let [first,last] = found.reach;
    let station = screw.station(found.nodes[first].position);
    let (section,normals): (Vec<V>,Vec<V>) = (0..found.nodes.len()).map(|w| found.on_section(w as f64,station))
        .collect::<Result<Vec<_>,_>>().at(Stage::Reach)?.into_iter().unzip();
    // σ(s) is carried for M(t) at roll time t+τ(s): the columns run over the stretch that carries
    // the section past the blank's heights (`Screw::passing`: a turn's, the whole roll, as a turn
    // keeps every height), widened, within what the roll carries every row to. A turn's delays
    // are read along the walk, each the one nearest the one before, as the section is one curve.
    // A whole turn's run once round from the roll's start, as its contacts were read
    // (`SweepContacts::domain`): every pose of it is a pose of that one turn.
    let roll = cut.limits;
    let (window,step) = if found.whole { ([roll[0],roll[0]+screw.period()],COLUMN_TURN/screw.ratio.abs()) } else {
        let mut delay: Vec<f64> = Vec::with_capacity(found.nodes.len());
        for node in &found.nodes { delay.push(screw.time_to_station(node.position,station,delay.last().copied().unwrap_or(0.))); }
        let (least,most) = delay.iter().fold((f64::INFINITY,f64::NEG_INFINITY),|(a,b),&d| (a.min(d),b.max(d)));
        let carried = [roll[0]-least,roll[1]-most];
        let base = family.pose_at(0.).at(Stage::Reach)?.point(section[first]);
        let passing = heights.and_then(|band| screw.passing(base,band)).unwrap_or([f64::NEG_INFINITY,f64::INFINITY]);
        let window = [passing[0].max(carried[0]),passing[1].min(carried[1])];
        let step = if screw.is_slide() { (window[1]-window[0]).max(0.)/SLIDE_COLUMNS } else { COLUMN_TURN/screw.ratio.abs() };
        ([(window[0]-COLUMN_MARGIN*step).max(carried[0]),(window[1]+COLUMN_MARGIN*step).min(carried[1])],step)
    };
    if !(window[0] < window[1]) {
        return Err(ExportRefusal::at(Stage::Reach,format!("`{name}`: the roll does not carry the characteristic past the blank")))
    }
    (say.stage)(&format!("`{name}`: its characteristic reaches the blank over {} of its points, carried over {:.1} degrees of the roll",
        last-first+1,(window[1]-window[0]).to_degrees()));
    (say.mark)(Stage::Reach);
    // rows by length along the section, each turn of its normal counted as a length of the
    // section's own size (its length over a half turn): a convex corner's fan turns the section
    // sharply over a stretch far shorter than a row, and is laid as many rows as its turn asks.
    // (Counted on the fan alone, the thread's flanks are laid too thinly for the split: a pcurve
    // a micrometre off.)
    let size = crate::util::space::polyline_length(&section)/std::f64::consts::PI;
    let turned = |a: V,b: V| crate::util::space::dot(a,b).clamp(-1.,1.).dacos();
    let mut lengths = vec![0.];
    for w in 1..section.len() { lengths.push(lengths[w-1]+distance(section[w-1],section[w])+size*turned(normals[w-1],normals[w])); }
    let total = *lengths.last().expect("a walk has nodes");
    let at_length = |l: f64| -> f64 {
        let w = lengths.partition_point(|&x| x < l).clamp(1,lengths.len()-1);
        let span = lengths[w]-lengths[w-1];
        (w-1) as f64+if span > 0. { ((l-lengths[w-1])/span).clamp(0.,1.) } else { 0. }
    };
    let mut rows: Vec<f64> = (0..FIRST_ROWS).map(|r| at_length(total*r as f64/(FIRST_ROWS-1) as f64)).collect();
    // a row where the characteristic runs onto a convex corner's fan and one where it runs off: the
    // sheet is cut there into faces (`Fitted::named`), the fan's own a band between its flanks',
    // so a point of either flank is never read on the other across the fan's sharp turn
    let fans: Vec<f64> = found.fan_ends().into_iter().filter(|&s| s > 0. && s < (section.len()-1) as f64).collect();
    for &s in &fans {
        let w = rows.partition_point(|&r| r < s);
        if rows.get(w) == Some(&s) { continue }
        // the nearer row within moved onto it (the rows stay in order), else one added
        let near = [w.wrapping_sub(1),w].into_iter().filter(|&k| k > 0 && k+1 < rows.len() && !fans.contains(&rows[k]))
            .min_by(|&a,&b| (rows[a]-s).abs().total_cmp(&(rows[b]-s).abs()));
        match near { Some(k) => rows[k] = s,None => rows.insert(w,s) }
    }
    let columns_n = (((window[1]-window[0])/step).ceil() as usize).max(4)+1;
    let columns: Vec<f64> = (0..columns_n).map(|c| window[0]+(window[1]-window[0])*c as f64/(columns_n-1) as f64).collect();
    let mids = |x: &[f64]| x.windows(2).map(|w| 0.5*(w[0]+w[1])).collect::<Vec<_>>();
    let grid = Grid {row_mids:mids(&rows),rows,column_mids:mids(&columns),columns};
    let sheet = lay(found,family,scale,station,&fans,&grid,Withheld::Centres).at(Stage::Sheet)?;
    // its edges out of the blank: the reach's ends and the roll's stretch were chosen so (a whole
    // turn's first and last columns are one, closed, and lie where they will)
    let edges: Vec<V> = (0..sheet.rows*sheet.columns).filter(|k| { let (r,c) = (k/sheet.columns,k%sheet.columns);
        r == 0 || r+1 == sheet.rows || (!found.whole && (c == 0 || c+1 == sheet.columns)) }).map(|k| sheet.points[k]).collect();
    let within = inside(&edges).at(Stage::Sheet)?.iter().filter(|b| **b).count();
    if within > 0 {
        return Err(ExportRefusal::at(Stage::Sheet,format!("`{name}`: {within} points of the sheet's edge lie in the blank")))
    }
    (say.stage)(&format!("`{name}`: sheet {}x{}, its edges clear of the blank",sheet.rows,sheet.columns));
    (say.mark)(Stage::Sheet);
    Ok(Laid {station,fans,grid,sheet})
}

/// A screw cut's sheet laid on `grid` (millimetres): the section's points at the grid's rows (at
/// `station`) carried by the motion's pose at each column, their normals turned, the contacts at
/// the cells' centres withheld, and with `Sides` the middles of their sides too.
fn lay(found: &Characteristic,family: &crate::sweep::motion::Family,scale: f64,station: f64,fans: &[f64],grid: &Grid,withheld: Withheld)
    -> Result<Sheet,String> {
    // a point of the sheet, in millimetres: the section's point carried, and its normal turned
    let carried = |(p,n): (V,V),pose: &crate::sweep::motion::Pose| -> (V,V) { (pose.point(p).map(|x| x*scale),pose.vector(n)) };
    let rows: Vec<(V,V)> = crate::util::par::map(&grid.rows,|&s| found.on_section(s,station)).into_iter().collect::<Result<_,_>>()?;
    let row_mids: Vec<(V,V)> = crate::util::par::map(&grid.row_mids,|&s| found.on_section(s,station)).into_iter().collect::<Result<_,_>>()?;
    // one pose a column
    let poses = |times: &[f64]| times.iter().map(|&t| family.pose_at(t)).collect::<Result<Vec<_>,_>>();
    let (mut columns,column_mids) = (poses(&grid.columns)?,poses(&grid.column_mids)?);
    // a whole turn's last column is its first, to the bit, so the ring closes on one edge
    if found.whole { let first = columns[0]; *columns.last_mut().expect("columns") = first; }
    let (nr,nc) = (grid.rows.len(),grid.columns.len());
    let bands = std::iter::once(0).chain(fans.iter().filter_map(|s| grid.rows.iter().position(|r| r == s)))
        .map(|r| (r,UNNAMED)).collect();
    let mut sheet = Sheet {bands,points:Vec::with_capacity(nr*nc),normals:Vec::with_capacity(nr*nc),times:Vec::with_capacity(nr*nc),
        rows:nr,columns:nc,withheld:Vec::new(),withheld_normals:Vec::new(),sites:Vec::new()};
    for row in &rows { for (pose,&t) in columns.iter().zip(&grid.columns) {
        let (p,n) = carried(*row,pose);
        sheet.points.push(p); sheet.normals.push(n); sheet.times.push(t);
    } }
    let mut hold = |pn: (V,V),site: [usize;2]| { sheet.withheld.push(pn.0); sheet.withheld_normals.push(pn.1); sheet.sites.push(site) };
    // the cells' centres, and with sides the middles of the cells' sides each way
    for (c,pose) in column_mids.iter().enumerate() {
        for (r,row) in row_mids.iter().enumerate() { hold(carried(*row,pose),[2*r+1,2*c+1]); }
        if withheld == Withheld::Sides { for (r,row) in rows.iter().enumerate() { hold(carried(*row,pose),[2*r,2*c+1]); } }
    }
    if withheld == Withheld::Sides {
        for (c,pose) in columns.iter().enumerate() {
            for (r,row) in row_mids.iter().enumerate() { hold(carried(*row,pose),[2*r+1,2*c]); }
        }
    }
    Ok(sheet)
}


/// How near the section's fit keeps to its exact points (a share of its size), and the most points
/// it reads along one band.
const SECTION_FIT: f64 = 1e-10;
const SECTION_MOST: usize = 4096;

/// The sheet of a screw cut whose characteristic `found` walked, as `helical_sheet` lays it but
/// carried exactly: its section (at the layout's station, each band between a convex corner's fan
/// ends its own curve) fitted along its length to its exact points, a curve of one parameter, and
/// that curve carried by the motion itself — a slide's an extrusion, a turn's a revolution, a
/// screw's the curve screwed (`Blend`) — so nothing is fitted across the motion, where the grid's
/// columns stand ten degrees apart. A whole turn's sheet is cut round into faces closing on each
/// other, as `helical_sheet`'s is. Its error is the section's fit, measured at the points between
/// those it passes through.
pub fn carried_sheet(cut: &SweptCut,found: &Characteristic,heights: Option<[f64;2]>,inside: Inside,say: &Say)
    -> Result<Fitted,ExportRefusal> {
    use crate::geom::{Carry,Curve,Frame,Rigid,Surface};
    use crate::topo::{reverse_loop,Brep,Coedge,EdgeCurve,Face,Pcurve};
    use crate::util::space::{add,dot,norm,scale as times,sub,unit};
    use std::sync::Arc;
    let found = &found.seamless();
    let Laid {station,fans,grid,sheet} = laid(cut,found,heights,inside,say)?;
    let (mm,name,screw,family) = (cut.scale,&cut.name,found.screw,&cut.family);
    let (t0,t1) = (grid.columns[0],grid.columns[grid.columns.len()-1]);
    let pose = family.pose_at(t0).at(Stage::Fit)?;
    let at = |s: f64| -> Result<V,String> { Ok(pose.point(found.on_section(s,station)?.0).map(|x| x*mm)) };
    // the bands between the fans' ends, each its section fitted through more of its exact points
    // until the points between them lie on it
    let mut ends = vec![grid.rows[0]];
    ends.extend(fans.iter().copied());
    ends.push(grid.rows[grid.rows.len()-1]);
    let size = crate::util::space::polyline_length(&sheet.points.iter().step_by(sheet.columns).copied().collect::<Vec<_>>());
    let bar = SECTION_FIT*size.max(1.);
    let mut error: f64 = 0.;
    let mut curves: Vec<crate::nurbs::BSpline> = Vec::new();
    for w in ends.windows(2) {
        let mut s: Vec<f64> = grid.rows.iter().copied().filter(|&r| r >= w[0] && r <= w[1]).collect();
        let mut points: Vec<V> = crate::util::par::map(&s,|&s| at(s)).into_iter().collect::<Result<_,_>>().at(Stage::Fit)?;
        let curve = loop {
            let mut t = vec![0.];
            for k in 1..points.len() { t.push(t[k-1]+distance(points[k-1],points[k])); }
            let curve = crate::nurbs::interpolate(&points,&t,3).ok_or("the section could not be interpolated").at(Stage::Fit)?;
            // (a point between two that the walk cannot read — its ring without the root, where
            // the curve turns between rings — is left unread beyond the reach, where the sheet
            // only carries its edge out of the blank, and refuses within it)
            let mids: Vec<f64> = s.windows(2).map(|x| 0.5*(x[0]+x[1])).collect();
            let between: Vec<Result<V,String>> = crate::util::par::map(&mids,|&m| at(m));
            let mut off: Vec<f64> = Vec::with_capacity(mids.len());
            for (&m,q) in mids.iter().zip(&between) {
                match q {
                    Ok(q) => off.push(distance(curve.point(curve.nearest(*q)),*q)),
                    Err(e) if m > found.reach[0] as f64 && m < found.reach[1] as f64 => return Err(e.clone()).at(Stage::Fit),
                    Err(_) => off.push(0.),
                }
            }
            let worst = off.iter().copied().fold(0.,f64::max);
            if worst <= bar || points.len() > SECTION_MOST { error = error.max(worst); break curve }
            let (mut s2,mut p2) = (vec![s[0]],vec![points[0]]);
            for k in 0..mids.len() {
                if off[k] > bar { s2.push(mids[k]); p2.push(*between[k].as_ref().expect("a point read")); }
                s2.push(s[k+1]); p2.push(points[k+1]);
            }
            (s,points) = (s2,p2);
        };
        curves.push(curve);
    }
    (say.stage)(&format!("`{name}`: its section fitted in {} curves of {} poles, within {error:.1e} mm of its exact points; carried {}",
        curves.len(),curves.iter().map(|c| c.poles.len()).sum::<usize>(),if screw.is_slide() { "along the slide" }
        else if screw.is_turn() { "round the turn" } else { "along the screw" }));
    (say.mark)(Stage::Fit);
    // the carry, in millimetres, over the columns' stretch of the roll: a whole turn's round once,
    // in quarters
    let span = t1-t0;
    let (angle,advance) = (screw.ratio*span,screw.advance*span/std::f64::consts::TAU*mm);
    let (centre,axis) = (screw.origin.map(|x| x*mm),unit(screw.axis));
    let pieces = if found.whole { AROUND } else { 1 };
    let stations: Vec<f64> = (0..=pieces).map(|j| j as f64/pieces as f64).collect();
    // each band's surface, its (along, carry) parameters' map to its own, a section carried to a
    // fraction of the carry, and a rail (where band `k` begins, or the last ends) over a stretch
    let carried = |c: f64| -> Rigid {
        Rigid::turn(centre,axis,angle*c).then(&Rigid {r:Rigid::identity().r,t:times(axis,advance*c)})
    };
    let moved = |b: &crate::nurbs::BSpline,m: &Rigid| -> crate::nurbs::BSpline {
        let mut b = b.clone(); b.poles = b.poles.iter().map(|&p| m.point(p)).collect(); b
    };
    let kind = if screw.is_slide() { 0 } else if screw.is_turn() { 1 } else { 2 };
    let surfaces: Vec<Surface> = curves.iter().map(|c| {
        let curve = Arc::new(Curve::BSpline(Arc::new(c.clone())));
        match kind {
            0 => Surface::Extrusion(Frame::about(c.point(c.domain()[0]),times(axis,advance.signum())),curve),
            1 => {
                let p = c.point(c.domain()[0]);
                let foot = add(centre,times(axis,dot(sub(p,centre),axis)));
                Surface::Revolution(Frame::new(foot,times(axis,angle.signum()),sub(p,foot)),curve)
            }
            _ => Surface::Blend(Frame::about(centre,axis),Arc::new(crate::geom::Blend {a:(*curve).clone(),ta:c.domain(),b:(*curve).clone(),
                tb:c.domain(),carry:Carry::Screw {center:centre,axis,angle,advance},closed:false})),
        }
    }).collect();
    let uv = |i: usize,a: f64,c: f64| -> [f64;2] {
        let [d0,d1] = curves[i].domain();
        match kind { 0 => [a,c*advance.abs()],1 => [c*angle.abs(),a],_ => [(a-d0)/(d1-d0),c] }
    };
    let rail = |k: usize,c: [f64;2]| -> (Curve,[f64;2]) {
        let (i,a) = if k < curves.len() { (k,curves[k].domain()[0]) } else { (k-1,curves[k-1].domain()[1]) };
        let p = curves[i].point(a);
        match (&surfaces[i],kind) {
            (_,0) => (Curve::Line {p,d:times(axis,advance.signum())},[c[0]*advance.abs(),c[1]*advance.abs()]),
            (_,1) => {
                let foot = add(centre,times(axis,dot(sub(p,centre),axis)));
                (Curve::Circle(Frame::new(foot,times(axis,angle.signum()),sub(p,foot)),norm(sub(p,foot))),
                    [c[0]*angle.abs(),c[1]*angle.abs()])
            }
            (Surface::Blend(_,b),_) => (Curve::Iso(b.clone(),if k < curves.len() { 0. } else { 1. }),c),
            _ => unreachable!("a screw's band is a blend"),
        }
    };
    let corner = |k: usize,c: f64| -> V {
        let (i,a) = if k < curves.len() { (k,curves[k].domain()[0]) } else { (k-1,curves[k-1].domain()[1]) };
        carried(c).point(curves[i].point(a))
    };
    let mut b = Brep::default();
    let m = if found.whole { pieces } else { pieces+1 };
    let vertices: Vec<Vec<u32>> = (0..=curves.len()).map(|k| (0..m).map(|j| b.vertex(corner(k,stations[j]))).collect()).collect();
    let sections: Vec<Vec<u32>> = (0..curves.len()).map(|i| (0..m).map(|j| {
        let c = moved(&curves[i],&carried(stations[j]));
        b.edge(EdgeCurve::Curve(Curve::BSpline(Arc::new(c))),curves[i].domain(),[vertices[i][j],vertices[i+1][j]])
    }).collect()).collect();
    let rails: Vec<Vec<u32>> = (0..=curves.len()).map(|k| (0..pieces).map(|j| {
        let (curve,t) = rail(k,[stations[j],stations[j+1]]);
        b.edge(EdgeCurve::Curve(curve),t,[vertices[k][j],vertices[k][(j+1)%m]])
    }).collect()).collect();
    for i in 0..curves.len() { for j in 0..pieces {
        let ([a0,a1],[c0,c1]) = (curves[i].domain(),[stations[j],stations[j+1]]);
        let line = |x: [f64;2],y: [f64;2]| Pcurve::Line {a:x,b:y};
        let lp = vec![
            Coedge {edge:sections[i][j],reversed:false,pcurve:line(uv(i,a0,c0),uv(i,a1,c0))},
            Coedge {edge:rails[i+1][j],reversed:false,pcurve:line(uv(i,a1,c0),uv(i,a1,c1))},
            Coedge {edge:sections[i][(j+1)%m],reversed:true,pcurve:line(uv(i,a0,c1),uv(i,a1,c1))},
            Coedge {edge:rails[i][j],reversed:true,pcurve:line(uv(i,a0,c0),uv(i,a0,c1))},
        ];
        // counter-clockwise in the surface's own parameters: a revolution's run (carry, along)
        let lp = if kind == 1 { reverse_loop(lp) } else { lp };
        b.faces.push(Face {surface:surfaces[i].clone(),reversed:false,loops:vec![lp],name:String::new()});
    } }
    (say.stage)(&format!("`{name}`: the sheet carried exactly, {} faces",b.faces.len()));
    Ok(Fitted {face:b.clone(),sheet,error,faces:b})
}
