//! A swept cut's sheet: its contacts traced at the cutter's stations over the band its declared
//! roll reaches the blank in (`solid::contact_trace`), laid out on a grid widened until its edges
//! leave the blank, interpolated as a B-spline surface (`nurbs::interpolate_net`), and judged at the
//! contacts withheld from the fit — held to a tolerance, refined where it misses until it fits.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::cutter::Cutter;
use super::Say;
use crate::brep::geom::Surface;
use crate::brep::nurbs::{interpolate_net,Parametrization};
use crate::brep::topo::Brep;
use crate::model::{Sketch,SolidDef};
use crate::motion::Family;
use crate::solid::contact_trace::{Band,Columns,Grid,Inside,Layout,Rows,Sheet,Station,TraceError,Tracer,Withheld,marked};
use crate::solid::export::{AtStage,ExportRefusal,Stage,Tolerance};
use crate::solid::{cad,SweepContacts};
use crate::space::{cross,distance,dot,norm};

type V = [f64;3];

/// The fit contract's bars without a stated tolerance: how far the fitted sheet may pass from a
/// withheld contact, and how far its normal may turn from the contact's. Gross: they separate a
/// fit that follows its contacts from one that does not.
const FIT_DISTANCE: f64 = 0.25;
const FIT_TURN: f64 = 20.;
/// How far the sheet's own normal may turn between points a quarter of a cell apart in the blank:
/// past a right angle it has folded back on itself, a pleat between the withheld contacts.
const FOLD_TURN: f64 = 90.;
/// How near its contact a foot found by a local search must be to be taken without a global one.
const LOCAL_TRUST: f64 = 0.005;
/// How many times a sheet's grid is refined toward its tolerance, and the most nodes it may have
/// each way (a kernel interpolates at most 512): past either it is refused. The native
/// construction (`gcs-cli`'s `sweep_boundary`) refines its sheets by the same bars.
pub const MOST_REFINEMENTS: usize = 4;
pub const MOST_ROWS: usize = 480;
pub const MOST_COLUMNS: usize = 400;
/// How many points each span of a fitted sheet's edge between two of its nodes is read at, for
/// whether the edge is in the blank; and the lengths a side in the blank is carried on by, in
/// its end span's lengths, the shortest first.
const EDGE_READS: usize = 8;
const CARRIED: [f64;5] = [0.25,0.5,1.,2.,4.];

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

/// A fitted sheet: the surface as a one-face B-rep, its grid, and its withheld error; and the
/// sheet as the faces the cutter's faces made (#212), each a stretch of the surface between the
/// rows where one face of the cutter meets the next (`Sheet::bands`), named by that face's path
/// in the cutter (`outer_crown.flank`) — one face, unnamed, where the stations name none.
pub struct Fitted { pub face: Brep,pub sheet: Sheet,pub error: f64,pub faces: Brep }

impl Fitted {
    /// A fit whose faces are its one face.
    pub fn whole(face: Brep,sheet: Sheet,error: f64) -> Fitted { let faces = face.clone(); Fitted {face,sheet,error,faces} }

    /// The sheet cut where the cutter's faces meet (`Sheet::bands`), each band named by `name`
    /// (`UNNAMED`'s empty): segments of the fitted net between the rows' parameters as `kind`
    /// spread them, which are the fit's.
    pub(super) fn named(&self,kind: Parametrization,name: &dyn Fn(u32) -> String) -> Result<Brep,String> {
        let bands = &self.sheet.bands;
        let Some(Surface::BSpline(_,net)) = self.face.faces.first().map(|f| &f.surface) else { return Ok(self.face.clone()) };
        if bands.len() < 2 {
            let mut faces = self.face.clone();
            faces.faces[0].name = bands.first().map_or(String::new(),|b| name(b.1));
            return Ok(faces)
        }
        let (tu,_) = crate::brep::nurbs::net_parameters(&self.sheet.points,self.sheet.rows,self.sheet.columns,kind)
            .ok_or("the sheet's rows have no parameters")?;
        let [[a,z],v] = net.domain();
        let at = |r: usize| a+(z-a)*tu[r];
        let nets = bands.iter().enumerate().map(|(i,&(start,tag))| {
            let (lo,hi) = (if i == 0 { a } else { at(start) },bands.get(i+1).map_or(z,|b| at(b.0)));
            (net.segment([lo,hi],v),name(tag))
        }).collect();
        crate::brep::build::sheets(nets)
    }
}

/// The sides of a sheet face `build::sheet` made (its edges in order: the start of `v`, the end of
/// `u`, the end of `v`, the start of `u`), each as the parameter it ends (0 `u`, 1 `v`) and whether
/// at the end of it.
const SIDES: [(usize,bool);4] = [(1,false),(0,true),(1,true),(0,false)];

/// The points of a sheet face's edges in the blank, with the side each is on (`SIDES`): each edge
/// read at `EDGE_READS` points a span of the most `nodes` an edge of the grid has.
fn edge_in_blank(face: &Brep,nodes: usize,inside: Inside) -> Result<Vec<(usize,V)>,String> {
    let n = EDGE_READS*nodes.max(2);
    let points: Vec<(usize,V)> = face.edges.iter().enumerate().flat_map(|(side,e)| (0..=n).map(move |k| {
        (side,e.point(e.t[0]+(e.t[1]-e.t[0])*k as f64/n as f64,&face.vertices))
    })).collect();
    let within = inside(&points.iter().map(|(_,p)| *p).collect::<Vec<_>>())?;
    Ok(points.into_iter().zip(within).filter(|(_,w)| *w).map(|(p,_)| p).collect())
}

/// A fitted sheet whose edge is in the blank carried out of it by its own surface. The grid's edge
/// nodes leave the blank, but where a corner of the cutter grazes the blank at some stations and
/// at others is a fan the rows stop before (`contact_trace::charted`), the edge between them stays
/// in it, and a sheet whose edge is in the blank divides nothing there. Each side in the blank is
/// continued past it by its end span's polynomial (`Net::extended`), the shortest of `CARRIED`
/// that leaves the blank, where everything the continuation puts in the blank is within
/// `allowance` of the blank's surface: what it bounds there in place of the corner's own sweep.
/// Otherwise the sheet is left as it is.
fn carry_out(name: &str,fitted: &mut Fitted,inside: Inside,near: &(dyn Fn(V) -> f64+Sync),allowance: f64,say: &Say)
    -> Result<(),String> {
    let nodes = fitted.sheet.rows.max(fitted.sheet.columns);
    let within = edge_in_blank(&fitted.face,nodes,inside)?;
    if within.is_empty() { return Ok(()) }
    let at = |p: V| p.map(|x| (x*1e4).round()/1e4);
    let depth = within.iter().map(|(_,p)| -near(*p)).fold(0_f64,f64::max);
    let sides: std::collections::BTreeSet<usize> = within.iter().map(|(side,_)| *side).collect();
    let Some(Surface::BSpline(_,net)) = fitted.face.faces.first().map(|f| &f.surface) else { return Ok(()) };
    // each side's end span, the length it is carried by a multiple of
    let span = |k: usize,high: bool| -> f64 {
        let (knots,p) = if k == 0 { (&net.uknots,net.du) } else { (&net.vknots,net.dv) };
        let count = if k == 0 { net.poles.len() } else { net.poles[0].len() };
        let [a,z] = [knots[p],knots[count]];
        if high { z-knots[..count].iter().copied().filter(|&x| x < z).fold(a,f64::max) }
        else { knots[p+1..].iter().copied().filter(|&x| x > a).fold(z,f64::min)-a }
    };
    let carried = |net: &crate::brep::nurbs::Net,multiple: f64| -> Option<crate::brep::nurbs::Net> {
        sides.iter().try_fold(net.clone(),|n,&side| { let (k,high) = SIDES[side]; n.extended(k,high,multiple*span(k,high)) })
    };
    for multiple in CARRIED {
        let Some(wider) = carried(net,multiple) else { return Ok(()) };
        let face = crate::brep::build::sheet(wider.clone())?;
        if !edge_in_blank(&face,nodes,inside)?.is_empty() { continue }
        // the strips added, read for how deep they are in the blank
        let [[u0,u1],[v0,v1]] = net.domain();
        let [[w0,w1],[x0,x1]] = wider.domain();
        let n = EDGE_READS*nodes.max(2);
        let strip: Vec<V> = sides.iter().flat_map(|&side| {
            let (k,high) = SIDES[side];
            let (lo,hi) = match (k,high) { (0,true) => (u1,w1),(0,false) => (w0,u0),(_,true) => (v1,x1),_ => (x0,v0) };
            let wider = &wider;
            (0..=EDGE_READS).flat_map(move |i| (0..=n).map(move |j| {
                let (across,along) = (lo+(hi-lo)*i as f64/EDGE_READS as f64,j as f64/n as f64);
                if k == 0 { wider.point(across,x0+(x1-x0)*along) } else { wider.point(w0+(w1-w0)*along,across) }
            }))
        }).collect();
        let deepest = inside(&strip)?.into_iter().zip(&strip).filter(|(w,_)| *w).map(|(_,p)| -near(*p)).fold(depth,f64::max);
        if deepest > allowance {
            (say.stage)(&format!("`{name}`: the fitted sheet's edge is in the blank (at {:?}), and carrying it out puts the \
                sheet {deepest:.4} mm into it, against {allowance} mm; left as it is",at(within[0].1)));
            return Ok(())
        }
        // the faces the cutter's faces made: a side across the bands (`v`) carried on in each, one along
        // them (`u`) in the first or the last
        let count = fitted.faces.faces.len();
        let nets: Vec<(crate::brep::nurbs::Net,String)> = fitted.faces.faces.iter().enumerate().map(|(i,f)| {
            let Surface::BSpline(_,band) = &f.surface else { return Err("a sheet's face is not a B-spline".to_string()) };
            let band = sides.iter().try_fold((**band).clone(),|b,&side| {
                let (k,high) = SIDES[side];
                if k == 0 && (if high { i+1 != count } else { i != 0 }) { return Some(b) }
                b.extended(k,high,multiple*span(k,high))
            }).ok_or("a sheet's face is rational")?;
            Ok((band,f.name.clone()))
        }).collect::<Result<_,String>>()?;
        fitted.faces = if count < 2 {
            let mut faces = crate::brep::build::sheet(nets[0].0.clone())?;
            faces.faces[0].name = nets[0].1.clone();
            faces
        } else { crate::brep::build::sheets(nets)? };
        fitted.face = face;
        (say.stage)(&format!("`{name}`: the fitted sheet's edge is in the blank by up to {depth:.4} mm (at {:?}), where a corner of \
            the cutter grazes it; carried on past it by its own surface, {deepest:.4} mm into the blank at most",at(within[0].1)));
        return Ok(())
    }
    (say.stage)(&format!("`{name}`: the fitted sheet's edge is in the blank (at {:?}), and carrying it on does not take it out; \
        left as it is",at(within[0].1)));
    Ok(())
}

/// A sheet face's points and outward normals on an `nu × nv` grid of its parameters, row-major.
pub fn surface_grid(face: &Brep,nu: usize,nv: usize) -> Vec<(V,V)> {
    let f = &face.faces[0];
    let [[a,z],[c,d]] = domain(face);
    (0..nu*nv).map(|k| {
        let (i,j) = (k/nv,k%nv);
        let uv = [a+(z-a)*i as f64/(nu-1) as f64,c+(d-c)*j as f64/(nv-1) as f64];
        let (p,su,sv) = f.surface.d1(uv);
        let n = cross(su,sv);
        let l = norm(n).max(1e-300);
        (p,n.map(|x| x/l*if f.reversed { -1. } else { 1. }))
    }).collect()
}

fn domain(face: &Brep) -> [[f64;2];2] {
    match &face.faces[0].surface { Surface::BSpline(_,n) => n.domain(),_ => [[0.,1.],[0.,1.]] }
}

/// For each point, its foot on the sheet face: the unoriented normal there and the distance to
/// it, or none. Searched locally from a guess (fractions of the face's parameters, NaN for none),
/// globally where the local search does not improve on the guess or lands farther than `trust`.
pub fn feet_near(face: &Brep,points: &[V],guesses: &[[f64;2]],trust: f64) -> Vec<Option<(V,f64)>> {
    let s = &face.faces[0].surface;
    let [[a,z],[c,d]] = domain(face);
    let gap = |uv: [f64;2],p: V| distance(s.point(uv),p);
    crate::par::indices(points.len(),|k| {
        let (p,g) = (points[k],guesses[k]);
        let start = (g[0].is_finite() && g[1].is_finite()).then(|| [a+(z-a)*g[0],c+(d-c)*g[1]]);
        let local = start.and_then(|uv| s.foot_from(p,uv).filter(|&f| gap(f,p) <= gap(uv,p)));
        let foot = match local {
            Some(uv) if gap(uv,p) <= trust => uv,
            _ => { let global = s.inverse(p); match local { Some(uv) if gap(uv,p) < gap(global,p) => uv,_ => global } }
        };
        let (_,su,sv) = s.d1(foot);
        let n = cross(su,sv);
        let l = norm(n);
        (l > 0.).then(|| (n.map(|x| x/l),gap(foot,p)))
    })
}

/// For each point, the fractions of the parameters of the nearest point of an `nu × nv` surface
/// grid, found in the cells about it; NaN where none lies there.
fn nearest_on(grid: &[(V,V)],nu: usize,nv: usize,points: &[V]) -> Vec<[f64;2]> {
    let mut steps: Vec<f64> = (0..nu*nv).filter(|k| k%nv+1 < nv && k/nv+1 < nu)
        .flat_map(|k| [distance(grid[k].0,grid[k+1].0),distance(grid[k].0,grid[k+nv].0)]).collect();
    steps.sort_by(f64::total_cmp);
    let mut cells = crate::space::Grid::new(steps.get(steps.len()*19/20).copied().unwrap_or(1.).max(1e-9));
    for (k,(p,_)) in grid.iter().enumerate() { cells.insert(*p,k as u32); }
    points.iter().map(|&p| {
        let mut best = (f64::INFINITY,usize::MAX);
        cells.around(p,|k| { let e = distance(grid[k as usize].0,p); if e < best.0 { best = (e,k as usize); } });
        if best.1 == usize::MAX { [f64::NAN;2] } else { [(best.1/nv) as f64/(nu-1) as f64,(best.1%nv) as f64/(nv-1) as f64] }
    }).collect()
}

/// A sheet grid interpolated as a face.
fn fit(sheet: &Sheet,kind: Parametrization) -> Result<Brep,String> {
    let net = interpolate_net(&sheet.points,sheet.rows,sheet.columns,kind).ok_or("the sheet grid could not be interpolated")?;
    crate::brep::build::sheet(net)
}

/// A fitted sheet's verdict at its withheld contacts: it fits, or, held to a tolerance, it misses
/// at the contacts marked, which a finer grid may mend.
enum Judged { Fits(Brep,f64,Parametrization),Misses {over: Vec<bool>,refusal: ExportRefusal} }

/// The fit contract (as the native host's): at every withheld contact near the blank the face
/// passes near the true contact and its normal agrees with the contact's, and nowhere in the blank
/// does its own normal turn back between points a quarter of a cell apart. Without a tolerance the
/// bars are gross, the sheet fitted by centripetal parameters (this kernel's interpolation, its
/// knots averaged from the parameters, overshoots the pinion's unevenly spaced rows by chord length:
/// it folded); held to one, chord-length and centripetal fits are both read and the better kept.
fn judged(name: &str,sheet: &Sheet,scale: f64,near: &(dyn Fn(V) -> f64+Sync),tolerance: Option<Tolerance>,say: &Say)
    -> Result<Judged,ExportRefusal> {
    let at = |p: V| p.map(|x| (x*1e3).round()/1e3);
    let (nu,nv) = (4*(sheet.rows-1)+1,4*(sheet.columns-1)+1);
    let folds = |grid: Vec<(V,V)>| -> Result<Vec<(V,V)>,ExportRefusal> {
        let (mut fold,mut folded) = (0_f64,[0.;3]);
        for i in 0..nu { for j in 0..nv {
            let (p,n) = grid[i*nv+j];
            let beside = [(i+1 < nu).then(|| grid[(i+1)*nv+j]),(j+1 < nv).then(|| grid[i*nv+j+1])];
            for (q,m) in beside.into_iter().flatten() {
                let angle = dot(n,m).clamp(-1.,1.).dacos().to_degrees();
                if angle > fold && (near(p) < 0. || near(q) < 0.) { fold = angle; folded = p; }
            }
        } }
        if fold > FOLD_TURN {
            return Err(ExportRefusal {stage:Stage::Withheld,condition:None,witness:Some(folded.map(|x| x/scale)),
                message:format!("`{name}`: the fitted sheet folds in the blank: its normal turns {fold:.1} degrees between points a \
                quarter of a cell apart (at {:?}), against {FOLD_TURN} degrees",at(folded))})
        }
        Ok(grid)
    };
    let Some(tolerance) = tolerance else {
        // by centripetal parameters, and where that fit leaves its contacts or folds, by chord length
        // (the two fail on different sheets: centripetal where rows are evenly spaced and a corner is
        // sharp, chord length where they are not)
        let gross = |kind: Parametrization| -> Result<(Brep,f64,Parametrization),ExportRefusal> {
        let face = fit(sheet,kind).at(Stage::Fit)?;
        (say.stage)(&format!("`{name}`: fitted the sheet ({kind:?}); measuring {} withheld contacts against it",sheet.withheld.len()));
        (say.mark)(Stage::Fit);
        let held: Vec<(V,V)> = sheet.withheld.iter().zip(&sheet.withheld_normals).filter(|(p,_)| near(**p) < 0.5).map(|(p,n)| (*p,*n)).collect();
        let points: Vec<V> = held.iter().map(|(p,_)| *p).collect();
        let grid = surface_grid(&face,nu,nv);
        let guesses = nearest_on(&grid,nu,nv,&points);
        let (mut error,mut turn,mut worst,mut turned) = (0_f64,0_f64,[0.;3],[0.;3]);
        for ((p,n),found) in held.iter().zip(feet_near(&face,&points,&guesses,LOCAL_TRUST)) {
            let Some((m,gap)) = found else { error = f64::INFINITY; worst = *p; continue };
            let angle = dot(m,*n).abs().min(1.).dacos().to_degrees();
            if gap > error { error = gap; worst = *p; }
            if angle > turn { turn = angle; turned = *p; }
        }
        (say.stage)(&format!("`{name}`: fitted sheet within {error:.2e} mm of the {} withheld contacts at the blank, normals within {turn:.2} degrees",
            points.len()));
        if error > FIT_DISTANCE || turn > FIT_TURN {
            let witness = if error > FIT_DISTANCE { worst } else { turned };
            return Err(ExportRefusal {stage:Stage::Withheld,condition:None,witness:Some(witness.map(|x| x/scale)),
                message:format!("`{name}`: the fitted sheet leaves its contacts: {error:.3} mm (at {:?}) and {turn:.1} degrees (at {:?}), \
                against {FIT_DISTANCE} mm and {FIT_TURN} degrees",at(worst),at(turned))})
        }
        folds(grid)?;
        Ok((face,error,kind))
        };
        let (face,error,kind) = match gross(Parametrization::Centripetal) {
            Ok(fitted) => fitted,
            Err(first) => match gross(Parametrization::ChordLength) { Ok(fitted) => fitted,Err(_) => return Err(first) },
        };
        (say.mark)(Stage::Withheld);
        return Ok(Judged::Fits(face,error,kind))
    };
    let held: Vec<usize> = (0..sheet.withheld.len()).filter(|&i| near(sheet.withheld[i]) < 0.5).collect();
    let points: Vec<V> = held.iter().map(|&i| sheet.withheld[i]).collect();
    let bar = tolerance.fit();
    struct Candidate { face: Brep,kind: Parametrization,far: Vec<bool>,turned: Vec<bool>,error: f64,worst: V,turn: (f64,f64,V),distant: usize,bent: usize }
    let fitted = |kind: Parametrization| -> Result<Candidate,ExportRefusal> {
        let face = fit(sheet,kind).at(Stage::Fit)?;
        let grid = folds(surface_grid(&face,nu,nv))?;
        let guesses = nearest_on(&grid,nu,nv,&points);
        let feet = feet_near(&face,&points,&guesses,bar);
        let mut c = Candidate {face,kind,far:vec![false;sheet.withheld.len()],turned:vec![false;sheet.withheld.len()],error:0.,worst:[0.;3],
            turn:(0.,f64::INFINITY,[0.;3]),distant:0,bent:0};
        for (&i,found) in held.iter().zip(feet) {
            let p = sheet.withheld[i];
            let (gap,angle) = match found { Some((m,gap)) => (gap,dot(m,sheet.withheld_normals[i]).abs().min(1.).dacos().to_degrees()),
                None => (f64::INFINITY,180.) };
            let limit = tolerance.turn(gap,sheet.spacing(sheet.sites[i]));
            if gap > c.error { c.error = gap; c.worst = p; }
            if angle-limit > c.turn.0-c.turn.1 { c.turn = (angle,limit,p); }
            c.far[i] = gap > bar; c.turned[i] = angle > limit;
        }
        (c.distant,c.bent) = (c.far.iter().filter(|m| **m).count(),c.turned.iter().filter(|m| **m).count());
        (say.stage)(&format!("`{name}`: fitted sheet {}x{} ({kind:?}) within {:.2} µm of the {} withheld contacts at the blank (bar {:.2} µm), \
            normals within their bars by {:.2} degrees at worst ({:.2} against {:.2}); {} miss by distance, {} by normal",
            sheet.rows,sheet.columns,c.error*1e3,points.len(),bar*1e3,c.turn.1-c.turn.0,c.turn.0,c.turn.1,c.distant,c.bent));
        Ok(c)
    };
    let chosen = match (fitted(Parametrization::Centripetal),fitted(Parametrization::ChordLength)) {
        (Err(fold),Err(_)) => return Err(fold),
        (Ok(c),Err(_)) | (Err(_),Ok(c)) => c,
        (Ok(a),Ok(b)) => if (b.distant,b.bent,b.error) < (a.distant,a.bent,a.error) { b } else { a },
    };
    (say.mark)(Stage::Fit);
    let Candidate {face,far,turned,error,worst,turn,distant,bent,kind,..} = chosen;
    if distant+bent > 0 {
        let witness = if distant > 0 { worst } else { turn.2 };
        return Ok(Judged::Misses {over:if distant > 0 { far } else { turned },refusal:ExportRefusal {stage:Stage::Withheld,
            condition:None,witness:Some(witness.map(|x| x/scale)),
            message:format!("`{name}`: the fitted sheet misses {distant} of its {} withheld contacts by distance, {:.2} µm from one \
            (at {:?}) against {:.2} µm, half the {} µm tolerance, and {bent} by normal, one {:.2} degrees off (at {:?}) against its \
            {:.2}",points.len(),error*1e3,at(worst),bar*1e3,tolerance.millimetres*1e3,turn.0,at(turn.2),turn.1)}})
    }
    (say.stage)(&format!("`{name}`: the {kind:?} fit holds"));
    (say.mark)(Stage::Withheld);
    Ok(Judged::Fits(face,error,kind))
}

/// The sheet of one swept cut against the blank `inside` describes: its contacts reach the blank
/// over a band of the cutter's stations (`Cutter::reach`), its grid is laid out by each row
/// placement in turn and widened until its edges leave the blank, and the first placement whose
/// fit holds is the sheet; held to a tolerance, a fit that misses is refined where it misses.
/// (The roll's clearance of the blank at both limits is admission's row E1, asked before this.)
pub fn swept_sheet(cut: &SweptCut,inside: Inside,near: &(dyn Fn(V) -> f64+Sync),tolerance: Option<Tolerance>,say: &Say)
    -> Result<Fitted,ExportRefusal> {
    let (scale,name) = (cut.scale,&cut.name);
    let cutter = Cutter::read(&cut.recipe).at(Stage::Reach)?;
    let sweep = cut.contacts.as_ref().map_err(Clone::clone).at(Stage::Reach)?;
    let tracer = Tracer {sweep,scale,inside,debug:false};
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
                let declared = sweep.domain();
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
        match settle(name,scale,first,&layout.grid,&lay,inside,near,tolerance,&format!("rows by {by}"),say)? {
            Ok((mut fitted,kind)) => {
                // each band named by the cutter's face that made it, by its path in the cutter
                let face_name = |tag: u32| -> String {
                    // (`UNNAMED` carries the fan's bit, so names nothing)
                    let Some((key,face)) = cutter.face_of(tag) else { return String::new() };
                    crate::solid::faces::FacePath::new(cut.routes.get(&key).map_or("",String::as_str),&face).text()
                };
                fitted.faces = fitted.named(kind,&face_name).at(Stage::Fit)?;
                carry_out(name,&mut fitted,inside,near,tolerance.map_or(FIT_DISTANCE,|t| t.fit()),say).at(Stage::Sheet)?;
                (say.stage)(&format!("`{name}`: the sheet's faces, by the cutter's: {}",
                    fitted.faces.faces.iter().map(|f| if f.name.is_empty() { "(unnamed)" } else { f.name.as_str() }).collect::<Vec<_>>().join(", ")));
                return Ok(fitted)
            }
            Err(refusal) => refused = Some(refusal),
        }
    }
    Err(refused.unwrap_or_else(|| ExportRefusal::at(Stage::Sheet,format!("`{name}`: no row placement was offered"))))
}

/// A sheet laid out on a grid fitted and judged: `first` (centres withheld) as it is without a
/// tolerance, else `lay` on `grid` (sides withheld) refined where it misses until it fits or the
/// refinements run out. A fit that misses its contacts or folds is the inner refusal (another
/// layout may fit), anything else the outer. `how` says how the grid was laid, for the refusal.
#[allow(clippy::too_many_arguments)]
pub(super) fn settle(name: &str,scale: f64,first: Sheet,grid: &Grid,lay: &(dyn Fn(&Grid) -> Result<Sheet,String>+Sync),inside: Inside,
    near: &(dyn Fn(V) -> f64+Sync),tolerance: Option<Tolerance>,how: &str,say: &Say)
    -> Result<Result<(Fitted,Parametrization),ExportRefusal>,ExportRefusal> {
    if tolerance.is_none() {
        return match judged(name,&first,scale,near,None,say) {
            Ok(Judged::Fits(face,error,kind)) => Ok(Ok((Fitted::whole(face,first,error),kind))),
            Ok(Judged::Misses {refusal,..}) => Ok(Err(refusal)),
            Err(refusal) if refusal.stage == Stage::Withheld => Ok(Err(refusal)),
            Err(refusal) => Err(refusal),
        }
    }
    let mut grid = grid.clone();
    for round in 0.. {
        let sheet = lay(&grid).at(Stage::Sheet)?;
        (say.stage)(&format!("`{name}`: sheet {}x{} and {} withheld contacts read",sheet.rows,sheet.columns,sheet.withheld.len()));
        if let Some(fault) = sheet.chart_fault(inside).at(Stage::Sheet)? {
            return Err(ExportRefusal::at(Stage::Sheet,format!("`{name}`: the refined sheet is not one regular chart: {fault}")))
        }
        let (over,mut refusal) = match judged(name,&sheet,scale,near,tolerance,say) {
            Ok(Judged::Fits(face,error,kind)) => return Ok(Ok((Fitted::whole(face,sheet,error),kind))),
            Ok(Judged::Misses {over,refusal}) => (over,refusal),
            Err(refusal) if refusal.stage == Stage::Withheld => return Ok(Err(refusal)),
            Err(refusal) => return Err(refusal),
        };
        let (rows,columns) = marked(&sheet.sites,&over,sheet.rows,sheet.columns);
        let next = grid.refined(&rows,&columns);
        if round >= MOST_REFINEMENTS || next.rows.len() > MOST_ROWS || next.columns.len() > MOST_COLUMNS {
            refusal.message = format!("{} with {how}, after {round} refinements to {}x{} nodes (at most {MOST_REFINEMENTS} \
                refinements and {MOST_ROWS}x{MOST_COLUMNS} nodes)",refusal.message,sheet.rows,sheet.columns);
            return Ok(Err(refusal))
        }
        (say.stage)(&format!("`{name}`: refining the sheet where it misses: {} of {} row intervals and {} of {} column intervals \
            split, {}x{} nodes",rows.iter().filter(|m| **m).count(),rows.len(),columns.iter().filter(|m| **m).count(),
            columns.len(),next.rows.len(),next.columns.len()));
        grid = next;
    }
    unreachable!("the refinements end")
}
