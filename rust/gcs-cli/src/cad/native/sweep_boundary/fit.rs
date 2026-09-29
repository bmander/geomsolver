//! The fit contract of a swept solid's sheet: the grid interpolated as a native face and judged
//! at its withheld contacts, by the gross bars or by a stated tolerance.
use super::*;

/// The fit contract's bounds without a stated tolerance: how far the fitted sheet may pass from a
/// withheld contact, and how far its normal may turn from the contact's. Gross, not the accuracy
/// budget: they separate a fit that follows its contacts from one that does not. A tolerance
/// replaces them (`Tolerance::fit`, `Tolerance::turn`).
const FIT_DISTANCE: f64 = 0.25;
const FIT_TURN: f64 = 20.;
/// How far the fitted face's own normal may turn between points a quarter of a cell apart in the
/// blank: past a right angle it has folded back on itself, a pleat between the withheld contacts.
const FOLD_TURN: f64 = 90.;
/// How near its contact a foot found by a local search from a guess must be to be taken as the
/// nearest foot without a global search (mm): the tolerance a fabrication export's fit is held to.
const LOCAL_TRUST: f64 = 0.005;
/// OCCT's interpolation parametrizations, by the number `Session::fit_sheet_with` takes.
const PARAMETRIZATIONS: [&str;3] = ["even","chord-length","centripetal"];

/// A fitted sheet's verdict at its withheld contacts: it fits (the face and its withheld error),
/// or, held to a tolerance, it misses at the contacts marked, which a finer grid may mend.
pub(super) enum Judged { Fits(c_int,f64),Misses {over: Vec<bool>,refusal: ExportRefusal} }

/// The fit contract of a sheet grid fitted as a native face, judged where the face bounds the cut:
/// in the blank, or within half a millimetre of it (`near`). At every withheld contact the face
/// passes near the true contact there and its normal agrees with the contact's; and nowhere in the
/// blank does its own normal turn back between points a quarter of a cell apart, a pleat the
/// withheld contacts (one a cell) can straddle. Without a tolerance the bars are gross (0.25 mm,
/// 20°); held to one, the distance bar is its fit share and the normal bar follows from it, the gap
/// and the node spacing at each contact (`Tolerance::turn`), and a miss is a verdict the caller may
/// refine.
pub(super) fn judged(session: &Session,name: &str,sheet: &Sheet,scale: f64,near: &(dyn Fn([f64;3]) -> f64+Sync),tolerance: Option<Tolerance>)
    -> Result<Judged,ExportRefusal> {
    let at = |p: [f64;3]| p.map(|x| (x*1e3).round()/1e3);
    let (nu,nv) = (4*(sheet.rows-1)+1,4*(sheet.columns-1)+1);
    let surface = |face: c_int| session.surface_grid(face,nu,nv).at(Stage::Withheld);
    let folds = |grid: Vec<([f64;3],[f64;3])>| -> Result<Vec<([f64;3],[f64;3])>,ExportRefusal> {
        let (mut fold,mut folded) = (0_f64,[0.;3]);
        for i in 0..nu { for j in 0..nv {
            let (p,n) = grid[i*nv+j];
            let beside = [(i+1 < nu).then(|| grid[(i+1)*nv+j]),(j+1 < nv).then(|| grid[i*nv+j+1])];
            for (q,m) in beside.into_iter().flatten() {
                let angle = dot(n,m).clamp(-1.,1.).acos().to_degrees();
                if angle > fold && (near(p) < 0. || near(q) < 0.) { fold = angle; folded = p; }
            }
        } }
        if fold > FOLD_TURN {
            return Err(ExportRefusal {stage:Stage::Withheld,condition:None,witness:Some(folded.map(|x| x/scale)),
                message:format!("`{name}`: the fitted sheet folds in the blank: its normal turns {fold:.1} degrees between points a \
                quarter of a cell apart (at {:?}), against {FOLD_TURN} degrees",at(folded))});
        }
        Ok(grid)
    };
    let Some(tolerance) = tolerance else {
        let face = session.fit_sheet(&sheet.points,sheet.rows,sheet.columns).at(Stage::Fit)?;
        stage(&format!("`{name}`: fitted the sheet; measuring {} withheld contacts against it",sheet.withheld.len()));
        mark(Stage::Fit);
        let held: Vec<([f64;3],[f64;3])> = sheet.withheld.iter().zip(&sheet.withheld_normals)
            .filter(|(p,_)| near(**p) < 0.5).map(|(p,n)| (*p,*n)).collect();
        let points: Vec<[f64;3]> = held.iter().map(|(p,_)| *p).collect();
        // Each foot searched from the nearest point of the fold check's grid, as a sheet held to a
        // tolerance is (below), and taken only within `LOCAL_TRUST` of its contact: a local foot
        // farther off may be beside a nearer one where the sheet passes close to itself (the
        // pinion's read 83 degrees off there against 8 at its nearest), so it is searched globally.
        let grid = surface(face)?;
        let guesses = nearest_on(&grid,nu,nv,&points);
        let (mut error,mut turn,mut worst,mut turned) = (0_f64,0_f64,[0.;3],[0.;3]);
        for ((p,n),found) in held.iter().zip(session.surface_feet_near(face,&points,&guesses,LOCAL_TRUST).at(Stage::Withheld)?) {
            let Some((m,gap)) = found else { error = f64::INFINITY; worst = *p; continue };
            let angle = dot(m,*n).abs().min(1.).acos().to_degrees();
            if gap > error { error = gap; worst = *p; }
            if angle > turn { turn = angle; turned = *p; }
        }
        stage(&format!("`{name}`: fitted sheet within {error:.2e} mm of the {} withheld contacts at the blank, normals within {turn:.2} degrees",
            points.len()));
        if error > FIT_DISTANCE || turn > FIT_TURN {
            let witness = if error > FIT_DISTANCE { worst } else { turned };
            return Err(ExportRefusal {stage:Stage::Withheld,condition:None,witness:Some(witness.map(|x| x/scale)),
                message:format!("`{name}`: the fitted sheet leaves its contacts: {error:.3} mm (at {:?}) and {turn:.1} degrees (at {:?}), \
                against {FIT_DISTANCE} mm and {FIT_TURN} degrees",at(worst),at(turned))});
        }
        folds(grid)?;
        mark(Stage::Withheld);
        return Ok(Judged::Fits(face,error));
    };
    let held: Vec<usize> = (0..sheet.withheld.len()).filter(|&i| near(sheet.withheld[i]) < 0.5).collect();
    let points: Vec<[f64;3]> = held.iter().map(|&i| sheet.withheld[i]).collect();
    let bar = tolerance.fit();
    // One interpolation of the grid, judged: a fold refuses it outright (refining would not unfold
    // it; another row placement may), otherwise every withheld contact is read against it.
    let fitted = |parametrization: c_int| -> Result<Candidate,ExportRefusal> {
        let clock = std::time::Instant::now();
        let face = session.fit_sheet_with(&sheet.points,sheet.rows,sheet.columns,parametrization).at(Stage::Fit)?;
        let grid = folds(surface(face)?)?;
        // Each contact's foot is searched from the nearest point of the fold check's grid, a quarter
        // of a cell apart: a local search where a global one samples the whole face, and a local
        // extremum is never nearer than the nearest foot, so a gap it reads is never too small. A
        // foot farther than the bar is searched globally, so a miss is never a local search's.
        let guesses = nearest_on(&grid,nu,nv,&points);
        let feet = session.surface_feet_near(face,&points,&guesses,bar).at(Stage::Withheld)?;
        // Distance first: a sheet is refined where it misses its contacts by distance, then where its
        // normals turn past their bars once every distance holds. A normal's bar shrinks as its
        // contact's gap nears the tolerance, so a contact missing by distance misses by normal too,
        // and would mark the other direction for a miss the first already names.
        let mut c = Candidate {face,parametrization,far:vec![false;sheet.withheld.len()],turned:vec![false;sheet.withheld.len()],
            error:0.,worst:[0.;3],turn:(0.,f64::INFINITY,[0.;3]),distant:0,bent:0,spent:clock.elapsed()};
        for (&i,found) in held.iter().zip(feet) {
            let p = sheet.withheld[i];
            let (gap,angle) = match found { Some((m,gap)) => (gap,dot(m,sheet.withheld_normals[i]).abs().min(1.).acos().to_degrees()),
                None => (f64::INFINITY,180.) };
            let limit = tolerance.turn(gap,sheet.spacing(sheet.sites[i]));
            if gap > c.error { c.error = gap; c.worst = p; }
            if angle-limit > c.turn.0-c.turn.1 { c.turn = (angle,limit,p); }
            c.far[i] = gap > bar; c.turned[i] = angle > limit;
            if (c.far[i] || c.turned[i]) && tracing() {
                eprintln!("  parametrization {parametrization}: miss at site {:?}: {:.2} µm, {angle:.2} against {limit:.2} degrees, \
                    spacing {:.4} mm, blank {:.3}, at {:?}",sheet.sites[i],gap*1e3,sheet.spacing(sheet.sites[i]),near(p),at(p));
            }
        }
        (c.distant,c.bent) = (c.far.iter().filter(|m| **m).count(),c.turned.iter().filter(|m| **m).count());
        stage(&format!("`{name}`: fitted sheet {}x{} ({}) within {:.2} µm of the {} withheld contacts at the blank (bar {:.2} µm), \
            normals within their bars by {:.2} degrees at worst ({:.2} against {:.2}); {} miss by distance, {} by normal ({:?})",
            sheet.rows,sheet.columns,PARAMETRIZATIONS[parametrization as usize],c.error*1e3,points.len(),
            bar*1e3,c.turn.1-c.turn.0,c.turn.0,c.turn.1,c.distant,c.bent,c.spent));
        Ok(c)
    };
    // Both parametrizations interpolate the same exact contacts, and the withheld contacts say which
    // follows them: chord-length parameters (a step's share of its column's length, averaged over
    // the columns) stay consistent under refinement, since halving a step halves its share, but where
    // a sheet's contacts crowd on one face and stretch round the next (the pinion's flank into its
    // fillet) a column's own parameters depart from their average and the fit creased there, 31°
    // between points a sixteenth of a cell apart; centripetal ones (the square root of each step)
    // temper that departure, the pinion's first fit coming 7.6 µm from its contacts where chord
    // length came 16.5 with no crease past 5°, but give a halved step √2 of its share, so a locally
    // refined grid's parameters no longer match its spacing (the swept torus's fit went from 0.11 to
    // 3.1 µm on three rows added). Neither holds everywhere; the better fit is the sheet.
    // the two fitted side by side, what each says said in that order
    let chosen = match beside(|| fitted(2),|| fitted(1),|_| false) {
        (Err(fold),Err(_)) => return Err(fold),
        (Ok(c),Err(_)) | (Err(_),Ok(c)) => c,
        (Ok(a),Ok(b)) => if (b.distant,b.bent,b.error) < (a.distant,a.bent,a.error) { b } else { a },
    };
    mark(Stage::Fit);
    let Candidate {face,far,turned,error,worst,turn,distant,bent,..} = chosen;
    if distant+bent > 0 {
        let witness = if distant > 0 { worst } else { turn.2 };
        return Ok(Judged::Misses {over:if distant > 0 { far } else { turned },refusal:ExportRefusal {stage:Stage::Withheld,
            condition:None,witness:Some(witness.map(|x| x/scale)),
            message:format!("`{name}`: the fitted sheet misses {distant} of its {} withheld contacts by distance, {:.2} µm from one \
            (at {:?}) against {:.2} µm, half the {} µm tolerance, and {bent} by normal, one {:.2} degrees off (at {:?}) against its \
            {:.2}",points.len(),error*1e3,at(worst),bar*1e3,tolerance.millimetres*1e3,turn.0,at(turn.2),turn.1)}});
    }
    stage(&format!("`{name}`: the {} fit holds",PARAMETRIZATIONS[chosen.parametrization as usize]));
    mark(Stage::Withheld);
    Ok(Judged::Fits(face,error))
}

/// One interpolation of a sheet held to a tolerance and its reading at the withheld contacts: which
/// miss by distance and which by normal, the worst gap and where, and the normal furthest past its
/// own bar (turn, bar, where).
struct Candidate {
    face: c_int,parametrization: c_int,far: Vec<bool>,turned: Vec<bool>,error: f64,worst: [f64;3],
    turn: (f64,f64,[f64;3]),distant: usize,bent: usize,spent: std::time::Duration,
}

/// For each point, the parameters (fractions of the UV box) of the nearest point of an `nu` x `nv`
/// grid of a face's surface, found in the cells about it of a grid about as wide as the surface
/// grid's steps; NaN where no surface grid point lies in those cells.
fn nearest_on(grid: &[([f64;3],[f64;3])],nu: usize,nv: usize,points: &[[f64;3]]) -> Vec<[f64;2]> {
    let mut steps: Vec<f64> = (0..nu*nv).filter(|k| k%nv+1 < nv && k/nv+1 < nu)
        .flat_map(|k| [distance(grid[k].0,grid[k+1].0),distance(grid[k].0,grid[k+nv].0)]).collect();
    steps.sort_by(f64::total_cmp);
    let mut cells = gcs_core::space::Grid::new(steps.get(steps.len()*19/20).copied().unwrap_or(1.).max(1e-9));
    for (k,(p,_)) in grid.iter().enumerate() { cells.insert(*p,k as u32); }
    points.iter().map(|&p| {
        let mut best = (f64::INFINITY,usize::MAX);
        cells.around(p,|k| {
            let e = distance(grid[k as usize].0,p);
            if e < best.0 { best = (e,k as usize); }
        });
        if best.1 == usize::MAX { [f64::NAN;2] } else { [(best.1/nv) as f64/(nu-1) as f64,(best.1%nv) as f64/(nv-1) as f64] }
    }).collect()
}
