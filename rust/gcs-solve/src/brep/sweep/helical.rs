//! A screw cut's sheet: the tool's characteristic (`solid::constant_twist`) carried along the
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
use crate::fmath::Det;
use super::sheet::{Fitted,SweptCut,settle};
use super::Say;
use crate::solid::constant_twist::Characteristic;
use crate::solid::contact_trace::{Grid,Inside,Sheet,UNNAMED,Withheld};
use crate::solid::export::{AtStage,ExportRefusal,Stage,Tolerance};
use crate::space::distance;

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

/// The sheet of a screw cut whose characteristic `found` walked (in model units, as `cut.scale`
/// says millimetres a unit), the blank between `heights` along the screw's axis where known.
pub fn helical_sheet(cut: &SweptCut,found: &Characteristic,heights: Option<[f64;2]>,inside: Inside,near: &(dyn Fn(V) -> f64+Sync),
    tolerance: Option<Tolerance>,say: &Say) -> Result<Fitted,ExportRefusal> {
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
    let size = crate::space::polyline_length(&section)/std::f64::consts::PI;
    let turned = |a: V,b: V| crate::space::dot(a,b).clamp(-1.,1.).dacos();
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
    // the fans cut out of the sheet as faces of their own: those turning a sharp corner's worth.
    // A shallow one, a drafted crease's few degrees, is a band of the sheet too narrow for the split
    // to meet a plane in (a pcurve 3e-5 off where it crossed a block's side), and the flanks either
    // side of it are near enough alike that neither is read on the other (#684 AG)
    let cut: Vec<f64> = fans.iter().copied().filter(|&s| found.fan_turn(s) > SHALLOW_FAN).collect();
    let columns_n = (((window[1]-window[0])/step).ceil() as usize).max(4)+1;
    let columns: Vec<f64> = (0..columns_n).map(|c| window[0]+(window[1]-window[0])*c as f64/(columns_n-1) as f64).collect();
    let mids = |x: &[f64]| x.windows(2).map(|w| 0.5*(w[0]+w[1])).collect::<Vec<_>>();
    let grid = Grid {row_mids:mids(&rows),rows,column_mids:mids(&columns),columns};
    // a point of the sheet, in millimetres: the section's point carried, and its normal turned
    let carried = |(p,n): (V,V),pose: &crate::motion::Pose| -> (V,V) { (pose.point(p).map(|x| x*scale),pose.vector(n)) };
    let lay = |grid: &Grid,withheld: Withheld| -> Result<Sheet,String> {
        let rows: Vec<(V,V)> = crate::par::map(&grid.rows,|&s| found.on_section(s,station)).into_iter().collect::<Result<_,_>>()?;
        let row_mids: Vec<(V,V)> = crate::par::map(&grid.row_mids,|&s| found.on_section(s,station)).into_iter().collect::<Result<_,_>>()?;
        // one pose a column
        let poses = |times: &[f64]| times.iter().map(|&t| family.pose_at(t)).collect::<Result<Vec<_>,_>>();
        let (mut columns,column_mids) = (poses(&grid.columns)?,poses(&grid.column_mids)?);
        // a whole turn's last column is its first, to the bit, so the ring closes on one edge
        if found.whole { let first = columns[0]; *columns.last_mut().expect("columns") = first; }
        let (nr,nc) = (grid.rows.len(),grid.columns.len());
        let bands = std::iter::once(0).chain(cut.iter().filter_map(|s| grid.rows.iter().position(|r| r == s)))
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
    };
    let sheet = lay(&grid,Withheld::Centres).at(Stage::Sheet)?;
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
    let sides = |grid: &Grid| lay(grid,Withheld::Sides);
    settle(name,scale,sheet,&grid,&sides,inside,near,tolerance,"rows along the characteristic",say)?.and_then(|(mut fitted,kind)| {
        if cut.is_empty() && !found.whole { return Ok(fitted) }
        fitted.faces = fitted.named(kind,&|_| String::new(),found.whole.then_some(AROUND)).at(Stage::Fit)?;
        (say.stage)(&format!("`{name}`: the sheet cut into {} faces{}{}",fitted.faces.faces.len(),
            if cut.is_empty() { "" } else { " where its characteristic runs round a convex corner's fan" },
            if found.whole { " and round the turn, closing on itself" } else { "" }));
        Ok(fitted)
    })
}
