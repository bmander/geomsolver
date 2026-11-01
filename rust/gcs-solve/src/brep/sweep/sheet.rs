//! A swept cut's sheet: its contacts traced at the cutter's stations over the band its declared
//! roll reaches the blank in (`brep::swept::trace`), laid out on a grid widened until its edges
//! leave the blank, interpolated as a B-spline surface (`nurbs::interpolate_net`), and judged at the
//! contacts withheld from the fit — held to a tolerance, refined where it misses until it fits.
#[allow(unused_imports)]
use crate::fmath::Det;
use crate::brep::swept::cutter::Cutter;
use super::Say;
use crate::brep::swept::sheet::{FIT_DISTANCE,carry_out,settle};
use crate::model::{Sketch,SolidDef};
use crate::motion::Family;
use crate::brep::swept::trace::{Band,Columns,Grid,Inside,Layout,Rows,Sheet,Station,TraceError,Tracer,Withheld};
use crate::solid::export::{AtStage,ExportRefusal,Stage,Tolerance};
use crate::solid::{cad,SweepContacts};
/// A sheet fitted, judged and cut into faces is the kernel's (`brep::swept::sheet`, #721 F2b); what
/// is here is how a generating sweep's contacts are traced for it.
pub use crate::brep::swept::sheet::{Fitted,MOST_COLUMNS,MOST_REFINEMENTS,MOST_ROWS,feet_near,surface_grid};

type V = [f64;3];

/// A swept cut as its sheet is built: what the sketch says of it, read where the sketch is, so
/// that the sheets can be built side by side.
pub struct SweptCut {
    pub name: String,
    pub scale: f64,
    /// The cutter's recipe, the motion and the declared roll's limits.
    pub recipe: crate::json::Json,
    pub family: Family,
    pub limits: [f64;2],
    /// The contacts, whose failure to read is the reach's.
    pub contacts: Result<SweepContacts,String>,
    /// The route to each primitive of the cutter (`document::operand_paths`), by the key its
    /// faces are named under: a cut face is named by the cutter's face that made it (#212).
    pub routes: std::collections::BTreeMap<String,String>,
}

impl SweptCut {
    pub fn read(sk: &Sketch,swept: usize) -> Result<SweptCut,ExportRefusal> {
        let name = sk.solids[swept].name.clone();
        let scale = cad::millimetres(sk);
        let SolidDef::Swept {source,motion,..} = &sk.solids[swept].def else {
            return Err(ExportRefusal::at(Stage::Clearance,format!("`{name}` is not a continuous sweep")))
        };
        let recipe = cad::recipe(sk,*source as usize).at(Stage::Clearance)?;
        let family = Family::read(sk,*motion as usize).at(Stage::Clearance)?;
        let limits = crate::solid::roll::interval(sk,swept).at(Stage::Clearance)?;
        let contacts = SweepContacts::read(sk,swept,cad::AXIS_TOLERANCE);
        // a member of a union tool's faces are named by their routes in the union (#684 C)
        let SolidDef::Swept {source:tool,..} = sk.solids[crate::solid::distributed::union_of(sk,swept)].def else {
            unreachable!("a member's union is a sweep")
        };
        let routes = crate::solid::operand_paths(sk,tool as usize);
        Ok(SweptCut {name,scale,recipe,family,limits,contacts,routes})
    }
}

/// The sheet of one swept cut against the blank `inside` describes: its contacts reach the blank
/// over a band of the cutter's stations (`Cutter::reach`), its grid is laid out by each row
/// placement in turn and widened until its edges leave the blank, and the first placement whose
/// fit holds is the sheet; held to a tolerance, a fit that misses is refined where it misses.
/// (The roll's clearance of the blank at both limits is admission's row E1, asked before this.)
pub fn swept_sheet(cut: &SweptCut,inside: Inside,near: &(dyn Fn(V) -> f64+Sync),tolerance: Option<Tolerance>,say: &Say)
    -> Result<Fitted,ExportRefusal> {
    let cutter = Cutter::read(&cut.recipe).at(Stage::Reach)?;
    traced_sheet(cut,&cutter,inside,near,tolerance,say)
}

/// The sheet of a prism whose cap reaches the blank (#684 F): `swept_sheet`'s, its stations the
/// prism's sections through its extrusion at distances along the direction admission's row T3
/// chose, each its chord carried along the extrusion, so the walk runs from a side over a cap's
/// rim and across the cap.
pub fn capped_sheet(cut: &SweptCut,capped: &crate::solid::admission::Capped,inside: Inside,near: &(dyn Fn(V) -> f64+Sync),
    tolerance: Option<Tolerance>,say: &Say) -> Result<Fitted,ExportRefusal> {
    let middle = 0.5*(capped.band[0]+capped.band[1]);
    let cutter = Cutter::read(&cut.recipe).and_then(|c| c.across(capped.across,middle)).at(Stage::Reach)?;
    traced_sheet(cut,&cutter,inside,near,tolerance,say)
}

/// `swept_sheet` of the cutter's stations.
fn traced_sheet(cut: &SweptCut,cutter: &Cutter,inside: Inside,near: &(dyn Fn(V) -> f64+Sync),tolerance: Option<Tolerance>,say: &Say)
    -> Result<Fitted,ExportRefusal> {
    let (scale,name) = (cut.scale,&cut.name);
    let chart = cut.contacts.as_ref().map_err(Clone::clone).and_then(SweepContacts::chart).at(Stage::Reach)?;
    let tracer = Tracer {chart,scale,inside,debug:false};
    let started = crate::clock::Instant::now();
    let reach = cutter.reach(&tracer).at(Stage::Reach)?;
    let width = reach.stations[1]-reach.stations[0];
    let band = if cutter.revolved() { format!("{:.1} degrees",width.to_degrees()) } else { format!("{width:.2} mm") };
    (say.stage)(&format!("`{name}`: contacts reach the blank over {band} of stations and {} profile faces ({:?}: \
        sections {:.1} s, contacts {:.1} s, {} blank queries {:.1} s)",reach.faces,
        started.elapsed(),reach.spent[0],reach.spent[1],reach.queried,reach.spent[2]));
    (say.mark)(Stage::Reach);
    // a station: the section loop carrying the reach's walk, and the window between its anchors
    let station_at = |angle: f64| -> Result<Station,TraceError> {
        let loops = cutter.profile(angle)?;
        let profile = loops.into_iter().find(|l| l.index_of(reach.start.face).is_some() && l.index_of(reach.end.face).is_some())
            .ok_or("no section loop carries the profile at a station of the band")?;
        let window = Cutter::range(&profile,&reach,0.)?;
        let cutter = &cutter;
        let starts = profile.starts();
        Ok(Station {length:profile.augmented_length(),window,sample:Box::new(move |s| cutter.sample(&profile,s)),starts})
    };
    let band = Band {stations:reach.stations,radius:reach.radius,limit:cutter.station_limit()};
    let columns: std::cell::RefCell<Option<((f64,f64),Columns)>> = std::cell::RefCell::new(None);
    let widened = |placement: Rows| -> Result<(Layout,Sheet),String> {
        let (mut margin,mut station_margin) = (1.,(reach.stations[1]-reach.stations[0])*0.15);
        for _ in 0..6 {
            let layout = {
                let mut traced = columns.borrow_mut();
                if !traced.as_ref().is_some_and(|(at,_)| *at == (margin,station_margin)) {
                    *traced = Some(((margin,station_margin),tracer.columns(&station_at,band,margin,station_margin)?));
                }
                traced.as_ref().expect("traced").1.layout(placement)?
            };
            let sheet = layout.sheet(&layout.grid,Withheld::Centres)?;
            let edges = [(0..sheet.columns).map(|c| sheet.points[c]).collect::<Vec<_>>(),
                (0..sheet.columns).map(|c| sheet.points[(sheet.rows-1)*sheet.columns+c]).collect(),
                (0..sheet.rows).map(|r| sheet.points[r*sheet.columns]).collect(),
                (0..sheet.rows).map(|r| sheet.points[r*sheet.columns+sheet.columns-1]).collect()];
            let within: Vec<usize> = edges.iter().map(|e| inside(e).map(|v| v.iter().filter(|b| **b).count())).collect::<Result<_,_>>()?;
            if within.iter().any(|&n| n > 0) {
                // an edge point in the blank at a time within the declared roll is where the sheet is
                // too small; one at a time outside it is the sheet extended past the roll, which
                // widening only carries further, and that needs a sheet trimmed at the roll's limits
                let declared = chart.domain();
                let (mut late,mut early) = (None,0);
                for r in 0..sheet.rows { for c in 0..sheet.columns {
                    if !(r == 0 || r+1 == sheet.rows || c == 0 || c+1 == sheet.columns) { continue }
                    let k = r*sheet.columns+c;
                    if !inside(&[sheet.points[k]])?[0] { continue }
                    let t = sheet.times[k];
                    if t < declared[0] || t > declared[1] { late.get_or_insert((t,sheet.points[k])); } else { early += 1; }
                } }
                if early == 0 { if let Some((t,p)) = late {
                    return Err(format!("`{name}`: the sheet, extended past the declared roll, comes back into the blank at {:?} \
                        (time {:.1} degrees, the roll being {:.1} to {:.1}); it needs trimming at the roll's limits",
                        p.map(|x| (x*1e4).round()/1e4),t.to_degrees(),declared[0].to_degrees(),declared[1].to_degrees()))
                } }
                (say.stage)(&format!("`{name}`: the sheet's edge is in the blank (first row {}, last row {}, first column {}, last column {} \
                    points); widening its margins",within[0],within[1],within[2],within[3]));
                margin *= 1.6; station_margin *= 1.6; continue
            }
            if let Some(fault) = sheet.chart_fault(inside)? { return Err(format!("`{name}`: the sheet is not one regular chart: {fault}")) }
            (say.stage)(&format!("`{name}`: sheet {}x{}, one chart",sheet.rows,sheet.columns));
            return Ok((layout,sheet))
        }
        Err(format!("`{name}`: the candidate sheet cannot be widened out of the blank"))
    };
    let mut refused: Option<ExportRefusal> = None;
    for placement in [Rows::Walk,Rows::Length] {
        let by = match placement { Rows::Walk => "walk length",Rows::Length => "length in space" };
        if let Some(refusal) = &refused { (say.stage)(&format!("{}; placing the sheet's rows by {by} instead",refusal.message)); }
        let (layout,first) = widened(placement).at(Stage::Sheet)?;
        (say.mark)(Stage::Sheet);
        let lay = |grid: &Grid| layout.sheet(grid,Withheld::Sides).map_err(String::from);
        let settled = say.kernel(|say| settle(name,scale,first,&layout.grid,&lay,inside,near,tolerance,&format!("rows by {by}"),say))?;
        match settled.map_err(ExportRefusal::from) {
            Ok((mut fitted,kind)) => {
                // each band named by the cutter's face that made it, by its path in the cutter
                let face_name = |tag: u32| -> String {
                    // (`UNNAMED` carries the fan's bit, so names nothing)
                    let Some((key,face)) = cutter.face_of(tag) else { return String::new() };
                    crate::solid::faces::FacePath::new(cut.routes.get(&key).map_or("",String::as_str),&face).text()
                };
                fitted.faces = fitted.named(kind,&face_name,None,false).at(Stage::Fit)?;
                say.kernel(|say| carry_out(name,&mut fitted,inside,near,tolerance.map_or(FIT_DISTANCE,|t| t.fit()),say)).at(Stage::Sheet)?;
                (say.stage)(&format!("`{name}`: the sheet's faces, by the cutter's: {}",
                    fitted.faces.faces.iter().map(|f| if f.name.is_empty() { "(unnamed)" } else { f.name.as_str() }).collect::<Vec<_>>().join(", ")));
                return Ok(fitted)
            }
            Err(refusal) => refused = Some(refusal),
        }
    }
    Err(refused.unwrap_or_else(|| ExportRefusal::at(Stage::Sheet,format!("`{name}`: no row placement was offered"))))
}

