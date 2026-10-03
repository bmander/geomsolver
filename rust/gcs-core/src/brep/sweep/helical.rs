//! A screw cut's sheet: the tool's characteristic (`solid::constant_twist`) carried along the
//! screw, S(s, t) = M(t)·c(s), its normal the tool's there carried with it. Every node and every
//! withheld point is exact: a point of the characteristic is a root of its ring in closed form, and
//! a pose of the screw is a formula. Nothing is traced. Rows run along the characteristic by
//! length in space, and columns along the roll over the stretch that carries it past the blank.
//! The grid is fitted, judged and refined where it misses (`sheet::settle`), as a traced sheet is.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::sheet::{Fitted,SweptCut,settle};
use super::Say;
use crate::solid::constant_twist::Characteristic;
use crate::solid::contact_trace::{Grid,Inside,Sheet,Withheld};
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

/// The sheet of a screw cut whose characteristic `found` walked (in model units, as `cut.scale`
/// says millimetres a unit), the blank between `heights` along the screw's axis where known.
pub fn helical_sheet(cut: &SweptCut,found: &Characteristic,heights: Option<[f64;2]>,inside: Inside,near: &(dyn Fn(V) -> f64+Sync),
    tolerance: Option<Tolerance>,say: &Say) -> Result<Fitted,ExportRefusal> {
    let (scale,name,screw,family) = (cut.scale,&cut.name,found.screw,&cut.family);
    // rows along the sheet's section square to the axis at the reach's first point's height: the
    // characteristic's points each carried along its path there, `on_section`. A column is that
    // section carried by the motion: S(s, t) = M(t)·σ(s), the same surface as M(t)·c(s).
    let [first,last] = found.reach;
    let height = screw.height(found.nodes[first].position);
    let section: Vec<V> = (0..found.nodes.len()).map(|w| found.on_section(w as f64,height).map(|(p,_)| p))
        .collect::<Result<_,_>>().at(Stage::Reach)?;
    // σ(s) is carried for M(t) at roll time t+τ(s): the columns run over the stretch that carries
    // the section past the blank's heights, widened, within what the roll carries every row to
    let roll = cut.limits;
    let delay: Vec<f64> = (0..found.nodes.len()).map(|w| screw.time_to(found.nodes[w].position,height)).collect();
    let (least,most) = delay.iter().fold((f64::INFINITY,f64::NEG_INFINITY),|(a,b),&d| (a.min(d),b.max(d)));
    let base = family.pose_at(0.).at(Stage::Reach)?.point(section[first]);
    let window = match heights {
        Some([lo,hi]) => { let (x,y) = (screw.time_to(base,lo),screw.time_to(base,hi)); [x.min(y),x.max(y)] }
        None => [roll[0]-least,roll[1]-most],
    };
    let step = COLUMN_TURN/screw.ratio.abs();
    let window = [(window[0]-COLUMN_MARGIN*step).max(roll[0]-least),(window[1]+COLUMN_MARGIN*step).min(roll[1]-most)];
    if !(window[0] < window[1]) {
        return Err(ExportRefusal::at(Stage::Reach,format!("`{name}`: the roll does not carry the characteristic past the blank")))
    }
    (say.stage)(&format!("`{name}`: its characteristic reaches the blank over {} of its points, carried over {:.1} degrees of the roll",
        last-first+1,(window[1]-window[0]).to_degrees()));
    (say.mark)(Stage::Reach);
    // rows by length along the section
    let mut lengths = vec![0.];
    for w in 1..section.len() { lengths.push(lengths[w-1]+distance(section[w-1],section[w])); }
    let total = *lengths.last().expect("a walk has nodes");
    let at_length = |l: f64| -> f64 {
        let w = lengths.partition_point(|&x| x < l).clamp(1,lengths.len()-1);
        let span = lengths[w]-lengths[w-1];
        (w-1) as f64+if span > 0. { ((l-lengths[w-1])/span).clamp(0.,1.) } else { 0. }
    };
    let rows: Vec<f64> = (0..FIRST_ROWS).map(|r| at_length(total*r as f64/(FIRST_ROWS-1) as f64)).collect();
    let columns_n = (((window[1]-window[0])/step).ceil() as usize).max(4)+1;
    let columns: Vec<f64> = (0..columns_n).map(|c| window[0]+(window[1]-window[0])*c as f64/(columns_n-1) as f64).collect();
    let mids = |x: &[f64]| x.windows(2).map(|w| 0.5*(w[0]+w[1])).collect::<Vec<_>>();
    let grid = Grid {row_mids:mids(&rows),rows,column_mids:mids(&columns),columns};
    // a point of the sheet, in millimetres: the section's point carried, and its normal turned
    let carried = |(p,n): (V,V),pose: &crate::motion::Pose| -> (V,V) { (pose.point(p).map(|x| x*scale),pose.vector(n)) };
    let lay = |grid: &Grid,withheld: Withheld| -> Result<Sheet,String> {
        let rows: Vec<(V,V)> = crate::par::map(&grid.rows,|&s| found.on_section(s,height)).into_iter().collect::<Result<_,_>>()?;
        let row_mids: Vec<(V,V)> = crate::par::map(&grid.row_mids,|&s| found.on_section(s,height)).into_iter().collect::<Result<_,_>>()?;
        // one pose a column
        let poses = |times: &[f64]| times.iter().map(|&t| family.pose_at(t)).collect::<Result<Vec<_>,_>>();
        let (columns,column_mids) = (poses(&grid.columns)?,poses(&grid.column_mids)?);
        let (nr,nc) = (grid.rows.len(),grid.columns.len());
        let mut sheet = Sheet {points:Vec::with_capacity(nr*nc),normals:Vec::with_capacity(nr*nc),times:Vec::with_capacity(nr*nc),
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
    // its edges out of the blank: the reach's ends and the roll's stretch were chosen so
    let edges: Vec<V> = (0..sheet.rows*sheet.columns).filter(|k| { let (r,c) = (k/sheet.columns,k%sheet.columns);
        r == 0 || r+1 == sheet.rows || c == 0 || c+1 == sheet.columns }).map(|k| sheet.points[k]).collect();
    let within = inside(&edges).at(Stage::Sheet)?.iter().filter(|b| **b).count();
    if within > 0 {
        return Err(ExportRefusal::at(Stage::Sheet,format!("`{name}`: {within} points of the sheet's edge lie in the blank")))
    }
    (say.stage)(&format!("`{name}`: sheet {}x{}, its edges clear of the blank",sheet.rows,sheet.columns));
    (say.mark)(Stage::Sheet);
    let sides = |grid: &Grid| lay(grid,Withheld::Sides);
    settle(name,scale,sheet,&grid,&sides,inside,near,tolerance,"rows along the characteristic",say)?
}
