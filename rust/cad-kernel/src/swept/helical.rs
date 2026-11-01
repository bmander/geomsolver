//! A screw cut's sheet: the tool's characteristic (`constant_twist`) carried along the
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
//! each other, so no face of it has a seam for the split to meet. Where the characteristic runs
//! onto or off a flat face the motion keeps in its own plane (#684 O), the sheet has a sharp edge
//! along that point's path: a crease, fitted continuous across and smooth either side
//! (`nurbs::interpolate_creased`), the sheet cut into faces there, the flat's strip its plane.
//! So too where it passes a convex corner, a rim or a crease whose fan stands at one point, every
//! normal between its faces in contact there (#684 V): the sweep has a sharp edge along that
//! point's path, between the two faces' sheets, and the sheet is creased there the same way.
//! A characteristic that closes on itself within the blank, a tool's passed wholly through it
//! (#684 W), is begun and ended at a crease (`Characteristic::closed`): its last row is its first,
//! to the bit, and the sheet, a tube, is cut into faces at its creases that close on each other
//! along the characteristic, so only its columns' ends must leave the blank. A tool slid along its
//! own axis wholly through the blank touches along its ring of greatest radius all round (#684 X):
//! the characteristic is that ring, closed with no crease, and the sheet the cylinder it sweeps,
//! written exactly (`build::tube`), not fitted, in faces round it. Under a turn about an axis the
//! tool's own meets, the characteristic runs half round a ring in contact all round (#684 X), and
//! the sheet is laid over it as over any stretch: its section there an arc of the circle about where
//! the axes meet, the zone of the sphere the ring sweeps.
#[allow(unused_imports)]
use crate::util::fmath::Det;
use super::constant_twist::Characteristic;
use super::sheet::{AtStage,Fitted,Grid,Inside,Refused,Say,Sheet,Stage,Tolerance,UNNAMED,Withheld,settle};
use crate::trajectory::{Motion,Trajectory};
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
/// How far a fan's normal turns (radians) at most for its stretch of the sheet to stay in the faces
/// either side of it rather than be cut out as its own: a tenth of a radian, past a drafted
/// crease's few degrees and short of any sharp corner the class has built.
const SHALLOW_FAN: f64 = 0.1;

/// The sheet of a cut named `name` along `path` over `roll` (in model units, `scale` millimetres a
/// unit) whose characteristic `found` walked, the blank between `heights` along the screw's axis
/// where known.
pub fn helical_sheet(name: &str,scale: f64,path: &Trajectory,roll: [f64;2],found: &Characteristic,heights: Option<[f64;2]>,inside: Inside,
    near: &(dyn Fn(V) -> f64+Sync),tolerance: Option<Tolerance>,say: &Say) -> Result<Fitted,Refused> {
    let (screw,family) = (found.screw,path);
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
    // (the roll the caller reads them over): every pose of it is a pose of that one turn.
    let (window,step) = if found.whole { ([roll[0],roll[0]+screw.period()],COLUMN_TURN/screw.ratio.abs()) } else {
        let mut delay: Vec<f64> = Vec::with_capacity(found.nodes.len());
        for node in &found.nodes { delay.push(screw.time_to_station(node.position,station,delay.last().copied().unwrap_or(0.))); }
        let (least,most) = delay.iter().fold((f64::INFINITY,f64::NEG_INFINITY),|(a,b),&d| (a.min(d),b.max(d)));
        let carried = [roll[0]-least,roll[1]-most];
        let base = family.at(0.).at(Stage::Reach)?.point(section[first]);
        let passing = heights.and_then(|band| screw.passing(base,band)).unwrap_or([f64::NEG_INFINITY,f64::INFINITY]);
        let window = [passing[0].max(carried[0]),passing[1].min(carried[1])];
        let step = if screw.is_slide() { (window[1]-window[0]).max(0.)/SLIDE_COLUMNS } else { COLUMN_TURN/screw.ratio.abs() };
        ([(window[0]-COLUMN_MARGIN*step).max(carried[0]),(window[1]+COLUMN_MARGIN*step).min(carried[1])],step)
    };
    if !(window[0] < window[1]) {
        return Err(Refused::at(Stage::Reach,format!("`{name}`: the roll does not carry the characteristic past the blank")))
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
    // (where the walk runs onto or off a flat face the motion keeps in its own plane, or passes a
    // fan standing at one point, the normal turns at a sharp edge, the crease, and the step past it
    // counts no turn, #684 O, V)
    let creased: Vec<f64> = found.creases();
    let mut lengths = vec![0.];
    for w in 1..section.len() {
        let turn = if creased.contains(&((w-1) as f64)) { 0. } else { size*turned(normals[w-1],normals[w]) };
        lengths.push(lengths[w-1]+distance(section[w-1],section[w])+turn);
    }
    let total = *lengths.last().expect("a walk has nodes");
    let at_length = |l: f64| -> f64 {
        let w = lengths.partition_point(|&x| x < l).clamp(1,lengths.len()-1);
        let span = lengths[w]-lengths[w-1];
        (w-1) as f64+if span > 0. { ((l-lengths[w-1])/span).clamp(0.,1.) } else { 0. }
    };
    // (a closed characteristic passes the tool twice, out one side and back the other, and is laid
    // twice the rows, #684 W)
    let first_rows = if found.closed { 2*FIRST_ROWS } else { FIRST_ROWS };
    let mut rows: Vec<f64> = (0..first_rows).map(|r| at_length(total*r as f64/(first_rows-1) as f64)).collect();
    // a row where the characteristic runs onto a convex corner's fan and one where it runs off: the
    // sheet is cut there into faces (`Fitted::named`), the fan's own a band between its flanks',
    // so a point of either flank is never read on the other across the fan's sharp turn
    let fans: Vec<f64> = found.fan_ends().into_iter().filter(|&s| s > 0. && s < (section.len()-1) as f64).collect();
    // and a row at each crease, where it runs onto or off a flat face (#684 O) or passes a fan
    // standing at one point (#684 V)
    let creases: Vec<f64> = creased.iter().copied().filter(|&s| s > 0. && s < (section.len()-1) as f64).collect();
    let fans: Vec<f64> = { let mut marks = [fans,creases.clone()].concat(); marks.sort_by(f64::total_cmp); marks.dedup(); marks };
    for &s in &fans {
        let w = rows.partition_point(|&r| r < s);
        if rows.get(w) == Some(&s) { continue }
        // the nearer row within moved onto it (the rows stay in order), else one added
        let near = [w.wrapping_sub(1),w].into_iter().filter(|&k| k > 0 && k+1 < rows.len() && !fans.contains(&rows[k]))
            .min_by(|&a,&b| (rows[a]-s).abs().total_cmp(&(rows[b]-s).abs()));
        match near { Some(k) => rows[k] = s,None => rows.insert(w,s) }
    }
    // the fans cut out of the sheet as faces of their own: those turning a sharp corner's worth.
    // A shallow one, a drafted crease's few degrees, is a band of the sheet too narrow for the split
    // to meet a plane in (a pcurve 3e-5 off where it crossed a block's side), and the flanks either
    // side of it are near enough alike that neither is read on the other (#684 AG)
    let mut cut: Vec<f64> = fans.iter().copied().filter(|&s| creases.contains(&s) || found.fan_turn(s) > SHALLOW_FAN).collect();
    // a closed characteristic's sheet cut once at least besides where it closes, at its crease, so
    // that its faces close on each other's edges, never one on its own (#684 W): at its middle row
    if found.closed && cut.is_empty() { cut.push(rows[rows.len()/2]); }
    // each stretch between two creases (or an end) at least four rows, as the creased fit asks
    // (`nurbs::interpolate_creased`): the widest step by length halved until it is
    let breaks: Vec<f64> = std::iter::once(rows[0]).chain(creases.iter().copied()).chain(std::iter::once(rows[rows.len()-1])).collect();
    for b in breaks.windows(2) {
        let length = |s: f64| { let w = (s.floor() as usize).min(lengths.len()-2); lengths[w]+(lengths[w+1]-lengths[w])*(s-w as f64) };
        while rows.iter().filter(|&&r| r >= b[0] && r <= b[1]).count() < 4 {
            let within: Vec<usize> = (0..rows.len()-1).filter(|&k| rows[k] >= b[0] && rows[k+1] <= b[1]).collect();
            let Some(&k) = within.iter().max_by(|&&x,&&y| (length(rows[x+1])-length(rows[x])).total_cmp(&(length(rows[y+1])-length(rows[y]))))
                else { break };
            rows.insert(k+1,at_length(0.5*(length(rows[k])+length(rows[k+1]))));
        }
    }
    let columns_n = (((window[1]-window[0])/step).ceil() as usize).max(4)+1;
    let columns: Vec<f64> = (0..columns_n).map(|c| window[0]+(window[1]-window[0])*c as f64/(columns_n-1) as f64).collect();
    let mids = |x: &[f64]| x.windows(2).map(|w| 0.5*(w[0]+w[1])).collect::<Vec<_>>();
    let grid = Grid {row_mids:mids(&rows),rows,column_mids:mids(&columns),columns};
    // a point of the sheet, in millimetres: the section's point carried, and its normal turned
    let carried = |(p,n): (V,V),pose: &Motion| -> (V,V) { (pose.point(p).map(|x| x*scale),pose.vector(n)) };
    let lay = |grid: &Grid,withheld: Withheld| -> Result<Sheet,String> {
        let mut rows: Vec<(V,V)> = crate::util::par::map(&grid.rows,|&s| found.on_section(s,station)).into_iter().collect::<Result<_,_>>()?;
        // a closed characteristic's last row is its first, to the bit, so the tube closes on one edge
        if found.closed { let first = rows[0]; *rows.last_mut().expect("rows") = first; }
        let row_mids: Vec<(V,V)> = crate::util::par::map(&grid.row_mids,|&s| found.on_section(s,station)).into_iter().collect::<Result<_,_>>()?;
        // one pose a column
        let poses = |times: &[f64]| times.iter().map(|&t| family.at(t)).collect::<Result<Vec<_>,_>>();
        let (mut columns,column_mids) = (poses(&grid.columns)?,poses(&grid.column_mids)?);
        // a whole turn's last column is its first, to the bit, so the ring closes on one edge
        if found.whole { let first = columns[0]; *columns.last_mut().expect("columns") = first; }
        let (nr,nc) = (grid.rows.len(),grid.columns.len());
        let bands = std::iter::once(0).chain(cut.iter().filter_map(|s| grid.rows.iter().position(|r| r == s)))
            .map(|r| (r,UNNAMED)).collect();
        let crease_rows: Vec<usize> = creases.iter().filter_map(|s| grid.rows.iter().position(|r| r == s)).collect();
        let mut sheet = Sheet {bands,points:Vec::with_capacity(nr*nc),normals:Vec::with_capacity(nr*nc),times:Vec::with_capacity(nr*nc),
            rows:nr,columns:nc,withheld:Vec::new(),withheld_normals:Vec::new(),sites:Vec::new(),creases:crease_rows.clone()};
        for row in &rows { for (pose,&t) in columns.iter().zip(&grid.columns) {
            let (p,n) = carried(*row,pose);
            sheet.points.push(p); sheet.normals.push(n); sheet.times.push(t);
        } }
        let mut hold = |pn: (V,V),site: [usize;2]| { sheet.withheld.push(pn.0); sheet.withheld_normals.push(pn.1); sheet.sites.push(site) };
        // the cells' centres, and with sides the middles of the cells' sides each way
        for (c,pose) in column_mids.iter().enumerate() {
            for (r,row) in row_mids.iter().enumerate() { hold(carried(*row,pose),[2*r+1,2*c+1]); }
            // (none on a crease, where the sheet's normal is either side's, nor where a closed
            // characteristic's sheet closes, at its crease)
            let seam = |r: usize| found.closed && (r == 0 || r+1 == nr);
            if withheld == Withheld::Sides { for (r,row) in rows.iter().enumerate().filter(|&(r,_)| !crease_rows.contains(&r) && !seam(r)) {
                hold(carried(*row,pose),[2*r,2*c+1]);
            } }
        }
        if withheld == Withheld::Sides {
            for (c,pose) in columns.iter().enumerate() {
                for (r,row) in row_mids.iter().enumerate() { hold(carried(*row,pose),[2*r+1,2*c]); }
            }
        }
        Ok(sheet)
    };
    let sheet = lay(&grid,Withheld::Centres).at(Stage::Sheet)?;
    // its edges out of the blank: the reach's ends and the roll's stretch were chosen so (a whole
    // turn's first and last columns are one, closed, and lie where they will; so too a closed
    // characteristic's first and last rows, #684 W)
    let edges: Vec<V> = (0..sheet.rows*sheet.columns).filter(|k| { let (r,c) = (k/sheet.columns,k%sheet.columns);
        (!found.closed && (r == 0 || r+1 == sheet.rows)) || (!found.whole && (c == 0 || c+1 == sheet.columns)) })
        .map(|k| sheet.points[k]).collect();
    let within = inside(&edges).at(Stage::Sheet)?.iter().filter(|b| **b).count();
    if within > 0 {
        return Err(Refused::at(Stage::Sheet,format!("`{name}`: {within} points of the sheet's edge lie in the blank")))
    }
    (say.stage)(&format!("`{name}`: sheet {}x{}, its edges clear of the blank",sheet.rows,sheet.columns));
    (say.mark)(Stage::Sheet);
    // a ring all round slid along the tool's own axis (#684 X): the sheet is the cylinder it sweeps,
    // written exactly from the first column's height to the last's, cut into faces at its quarters
    if let Some((centre,axis,radius)) = found.cylinder() {
        let [a,b] = window.map(|t| family.at(t).map(|pose| pose.point(centre).map(|x| x*scale)));
        let (a,b) = (a.at(Stage::Fit)?,b.at(Stage::Fit)?);
        let tube = crate::build::tube(a,axis,radius*scale,crate::util::space::dot(crate::util::space::sub(b,a),axis),AROUND).at(Stage::Fit)?;
        (say.stage)(&format!("`{name}`: the sheet is the cylinder of radius {:.4} mm the ring of greatest radius sweeps, written exactly, \
            in {AROUND} faces round it",radius*scale));
        (say.mark)(Stage::Fit);
        return Ok(Fitted {face:tube.clone(),sheet,error:0.,faces:tube})
    }
    let sides = |grid: &Grid| lay(grid,Withheld::Sides);
    settle(name,scale,sheet,&grid,&sides,inside,near,tolerance,"rows along the characteristic",say)?.and_then(|(mut fitted,kind)| {
        if cut.is_empty() && !found.whole && !found.closed { return Ok(fitted) }
        fitted.faces = fitted.named(kind,&|_| String::new(),found.whole.then_some(AROUND),found.closed).at(Stage::Fit)?;
        (say.stage)(&format!("`{name}`: the sheet cut into {} faces{}{}",fitted.faces.faces.len(),
            if creases.is_empty() { if cut.is_empty() { "" } else { " where its characteristic runs round a convex corner's fan" } }
                else if found.sharp().is_empty() { " where its characteristic runs onto or off a flat face, creased there" }
                else if found.flats().is_empty() { " where its characteristic passes a fan standing at one point, creased there" }
                else { " where its characteristic runs onto or off a flat face or passes a fan standing at one point, creased there" },
            if found.whole { " and round the turn, closing on itself" } else if found.closed { ", closing on itself along its characteristic" }
                else { "" }));
        Ok(fitted)
    })
}
