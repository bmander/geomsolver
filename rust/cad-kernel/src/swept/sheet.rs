//! **A swept cut's sheet fitted** (#721 F2b): its contacts laid out on a grid (`Sheet`, `Grid`),
//! interpolated as a B-spline surface (`nurbs::interpolate_creased`) and judged at the contacts
//! withheld from the fit — held to a tolerance (`Tolerance`), refined where it misses until it fits
//! (`settle`), and cut into the faces of the tool that made it (`Fitted::named`). How the contacts
//! were found is the caller's: traced by the solver for the generating class, the characteristic
//! carried for a constant twist (`helical`). A refusal says the stage it was met at (`Refused`).
#[allow(unused_imports)]
use crate::util::fmath::Det;
use crate::geom::Surface;
use crate::nurbs::{interpolate_creased,Parametrization};
use crate::topo::Brep;
use crate::util::space::{cross,distance,dot,norm};

type V = [f64;3];

/// The stages a sheet is built through, in order: its contacts reach the blank, its grid laid with
/// its edges clear of the blank, fitted, and judged at the contacts withheld from the fit.
#[derive(Clone,Copy,Debug,PartialEq,Eq,PartialOrd,Ord)]
pub enum Stage { Reach,Sheet,Fit,Withheld }

/// Why a sheet was not built: the stage it was refused at, a point where (millimetres over the
/// model's scale, as the caller reads it), and the words a person reads.
#[derive(Clone,Debug)]
pub struct Refused { pub stage: Stage,pub witness: Option<V>,pub message: String }

impl Refused {
    pub fn at(stage: Stage,message: impl Into<String>) -> Refused { Refused {stage,witness:None,message:message.into()} }
}

/// A result's error as a refusal at a stage.
pub trait AtStage<T> {
    fn at(self,stage: Stage) -> Result<T,Refused>;
}

impl<T,E: Into<String>> AtStage<T> for Result<T,E> {
    fn at(self,stage: Stage) -> Result<T,Refused> { self.map_err(|e| Refused::at(stage,e)) }
}

/// How a sheet's build speaks: a line of progress, and the stage just passed.
pub struct Say<'a> { pub stage: &'a (dyn Fn(&str)+Sync),pub mark: &'a (dyn Fn(Stage)+Sync) }

/// Whether points lie inside the blank, in native millimetres.
pub type Inside<'a> = &'a (dyn Fn(&[V]) -> Result<Vec<bool>,String>+Sync);

/// A sheet band's face where the stations do not say (#212).
pub const UNNAMED: u32 = u32::MAX;

/// A candidate sheet: contact positions on a row-major grid with the outward
/// normal of the cutter at each contact, in native millimetres.
#[derive(Debug)]
pub struct Sheet {
    /// Where each of the cutter's faces begins down the rows, and the face (`Station::starts`'
    /// tag, `UNNAMED` where the stations say none): the first row of each band of rows one face
    /// made, the first at row 0, a band's last row the next one's first — every column crossing
    /// from one face to the next at that row (#212).
    pub bands: Vec<(usize,u32)>,
    pub points: Vec<V>,
    pub normals: Vec<V>,
    /// Each point's contact time.
    pub times: Vec<f64>,
    pub rows: usize,
    pub columns: usize,
    /// Contacts withheld from the fit (`Withheld`), with their normals.
    pub withheld: Vec<V>,
    pub withheld_normals: Vec<V>,
    /// Where each withheld contact is, in half rows and half columns: `[2r+1, 2c+1]` the centre
    /// of the cell after node `(r, c)`, `[2r+1, 2c]` the middle of column `c`'s step from row `r`,
    /// `[2r, 2c+1]` the middle of row `r`'s step from column `c`.
    pub sites: Vec<[usize;2]>,
    /// The rows along which the sheet is creased, between two bands (`bands`): a sharp edge of the
    /// sweep, where a flat face the motion keeps in its own plane meets the face beside it (#684 O).
    /// The fit is continuous across one, smooth either side (`nurbs::interpolate_creased`).
    pub creases: Vec<usize>,
}

impl Sheet {
    /// The chart contract: where the grid lies in the blank it samples one regular chart of the
    /// envelope. Down each column, the contact time does not leap (a root a turn away, or a
    /// root elsewhere taken for this one); the root itself may change, at a fold where the two
    /// merge, which the traced column walks through. Position is no test: near a fold a contact
    /// runs fast along the sweep, which is steep, not broken; whether the fit can follow it is
    /// the fit contract's question. Steps wholly
    /// outside the blank are the margin that carries the sheet's edge out, and bound nothing;
    /// a fault there that disturbs the fit inside is the fit contract's to find. The first
    /// violation, where it is.
    pub fn chart_fault(&self,inside: Inside) -> Result<Option<String>,String> {
        let within = inside(&self.points)?;
        let mut faults = Vec::new();
        for c in 0..self.columns {
            let at = |r: usize| r*self.columns+c;
            for r in 1..self.rows {
                if !within[at(r)] && !within[at(r-1)] { continue; }
                let p = self.points[at(r)].map(|x| (x*1e3).round()/1e3);
                if (self.times[at(r)]-self.times[at(r-1)]).abs() > 1. {
                    faults.push(format!("the contact time leaps from {:.3} to {:.3} between rows {} and {r} of column {c} at {p:?}",
                        self.times[at(r-1)],self.times[at(r)],r-1));
                }
            }
        }
        Ok(faults.first().map(|f| format!("{f} ({} such steps in the blank, over {} columns)",faults.len(),self.columns)))
    }

    /// The shortest side, in space, of the cells a withheld contact's site touches: the spacing
    /// of the nodes the fit's error there is spread between.
    pub fn spacing(&self,site: [usize;2]) -> f64 {
        let cells = |i: usize,n: usize| -> Vec<usize> {
            if i%2 == 1 { vec![i/2] } else { [(i/2).checked_sub(1),(i/2+1 < n).then_some(i/2)].into_iter().flatten().collect() }
        };
        let at = |r: usize,c: usize| self.points[r*self.columns+c];
        let mut least = f64::INFINITY;
        for r in cells(site[0],self.rows) { for c in cells(site[1],self.columns) {
            for d in [distance(at(r,c),at(r+1,c)),distance(at(r,c+1),at(r+1,c+1)),distance(at(r,c),at(r,c+1)),distance(at(r+1,c),at(r+1,c+1))] {
                least = least.min(d);
            }
        } }
        least
    }
}

/// Where a sheet's nodes lie and where its withheld contacts: the row coordinates (unfolded walk
/// lengths for `Rows::Walk`, fractions of each column's length between the sheet's ends for
/// `Rows::Length`) and the column station angles, each ascending, with the coordinate a withheld
/// contact takes in each interval between them.
#[derive(Clone,Debug,PartialEq)]
pub struct Grid { pub rows: Vec<f64>,pub row_mids: Vec<f64>,pub columns: Vec<f64>,pub column_mids: Vec<f64> }

/// How far one interval may be longer than its neighbour after a refinement: an interpolating
/// spline through nodes whose spacing jumps by more rings in the long interval, and the
/// refinement would chase its own ringing.
const GRADING: f64 = 2.;

impl Grid {
    /// The grid with each marked interval (row intervals, then column intervals) split at its
    /// withheld coordinate, which becomes a node (the contact withheld there, a node's), with a
    /// withheld coordinate in the middle of each half; and further intervals split until none is
    /// more than `GRADING` times as long as its neighbour.
    pub fn refined(&self,rows: &[bool],columns: &[bool]) -> Grid {
        let (r,rm) = split(&self.rows,&self.row_mids,rows);
        let (c,cm) = split(&self.columns,&self.column_mids,columns);
        Grid {rows:r,row_mids:rm,columns:c,column_mids:cm}
    }
}

/// One direction of `Grid::refined`.
fn split(nodes: &[f64],mids: &[f64],marked: &[bool]) -> (Vec<f64>,Vec<f64>) {
    let mut marked = marked.to_vec();
    let width = |i: usize,m: &[bool]| (nodes[i+1]-nodes[i]).abs()*if m[i] { 0.5 } else { 1. };
    loop {
        let mut changed = false;
        for i in 0..marked.len() {
            if marked[i] { continue }
            let w = width(i,&marked);
            let over = [i.checked_sub(1),(i+1 < marked.len()).then_some(i+1)].into_iter().flatten()
                .any(|j| w > GRADING*width(j,&marked)*(1.+1e-9));
            if over { marked[i] = true; changed = true; }
        }
        if !changed { break }
    }
    let (mut n,mut m) = (vec![nodes[0]],Vec::new());
    for i in 0..mids.len() {
        if marked[i] {
            m.push(0.5*(nodes[i]+mids[i])); n.push(mids[i]);
            m.push(0.5*(mids[i]+nodes[i+1]));
        } else { m.push(mids[i]); }
        n.push(nodes[i+1]);
    }
    (n,m)
}

/// Which intervals of a sheet's grid to refine, from its withheld contacts (`Sheet::sites`) and
/// which of them miss the fit's bar: a contact at the middle of a column's step between two rows
/// says the rows there are too far apart, one at the middle of a row's step between two columns
/// that the columns are; one at a cell's centre whose sides said neither, that both are (a twist
/// neither side's contact sees). Row intervals, then column intervals.
pub fn marked(sites: &[[usize;2]],over: &[bool],rows: usize,columns: usize) -> (Vec<bool>,Vec<bool>) {
    let (mut r,mut c) = (vec![false;rows.saturating_sub(1)],vec![false;columns.saturating_sub(1)]);
    for (s,_) in sites.iter().zip(over).filter(|(_,o)| **o) {
        match (s[0]%2,s[1]%2) { (1,0) => r[s[0]/2] = true, (0,1) => c[s[1]/2] = true, _ => {} }
    }
    let (sides_r,sides_c) = (r.clone(),c.clone());
    for (s,_) in sites.iter().zip(over).filter(|(_,o)| **o) {
        let (i,j) = (s[0]/2,s[1]/2);
        if s[0]%2 == 1 && s[1]%2 == 1 && !sides_r[i] && !sides_c[j] { r[i] = true; c[j] = true; }
    }
    (r,c)
}

/// Which contacts a sheet withholds from its fit: one at the centre of every cell, or also one
/// at the middle of each side of every cell (down each column between its rows, and across each
/// row between its columns), which say which way a cell is under-sampled.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub enum Withheld { Centres,Sides }

/// The fit contract's bars without a stated tolerance: how far the fitted sheet may pass from a
/// withheld contact, and how far its normal may turn from the contact's. Gross: they separate a
/// fit that follows its contacts from one that does not.
pub const FIT_DISTANCE: f64 = 0.25;
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

/// A native export's stated tolerance: how far the written surface may lie from the exact one,
/// in native millimetres (a physical length, whatever unit the document is written in), and
/// every bar that follows from it. Without one an export keeps its gross bars (a fit within
/// 0.25 mm and 20°, the mesher's 0.01 mm deflection, the field probed 0.1 mm off), which separate
/// a fit that follows its contacts from one that does not and are no fabrication claim.
#[derive(Clone,Copy,Debug,PartialEq)]
pub struct Tolerance { pub millimetres: f64 }

impl Tolerance {
    /// The fabrication tolerance an export is held to when none is named: 10 µm.
    pub const FABRICATION: Tolerance = Tolerance {millimetres:0.01};
    /// The share of the tolerance a fitted sheet may use; the STL's chordal sag has the rest, so
    /// the sheet (the STEP's surface) plus the sag (the STL's) stays within the tolerance.
    pub const FIT_SHARE: f64 = 0.5;
    /// The gross bar a fitted sheet's normal is never allowed past, in degrees.
    pub const MOST_TURN: f64 = 20.;
    /// The least distance the field is probed at off a mesh: four times the largest vertex and
    /// edge tolerance the kernel has left on a united member (4.8e-4 mm), inside which a probe
    /// could stand in the kernel's own tolerance band rather than on one side of the surface.
    pub const LEAST_PROBE: f64 = 0.002;

    pub fn new(millimetres: f64) -> Result<Tolerance,String> {
        if millimetres.is_finite() && millimetres > 0. { Ok(Tolerance {millimetres}) }
        else { Err(format!("an export tolerance must be a positive length, not {millimetres} mm")) }
    }

    /// How far a fitted sheet may pass from any withheld contact.
    pub fn fit(&self) -> f64 { self.millimetres*Self::FIT_SHARE }

    /// How far a fitted sheet's normal may turn from a withheld contact's, in degrees, where the
    /// sheet passes `gap` from the contact and its nodes there are `spacing` apart (the shortest
    /// side of the cells beside it). Withheld contacts lie on a grid of half cells, so no point of
    /// the sheet is further than a quarter of a cell from one along either direction; a sheet
    /// turned by `θ` there departs from the exact surface by about `gap + tan θ · spacing / 4`
    /// within that reach, and `θ` may not carry it past the whole tolerance, the STEP's claim:
    /// `tan θ ≤ 4 (tolerance − gap) / spacing`. A pleat between contacts (the gear's first sheet,
    /// turned 83° within 60 µm) fails it; a fit rounding a narrow strip where the exact normal
    /// turns fast (a convex corner's fan, a small fillet), which is near in position and turned by
    /// a few degrees, does not. Never past `MOST_TURN`, whatever the spacing.
    pub fn turn(&self,gap: f64,spacing: f64) -> f64 {
        let room = (self.millimetres-gap).max(0.);
        (4.*room/spacing.max(1e-12)).datan().to_degrees().min(Self::MOST_TURN)
    }

    /// The STL mesher's absolute chordal deflection: the share of the tolerance the sheet does
    /// not use.
    pub fn deflection(&self) -> f64 { self.millimetres*(1.-Self::FIT_SHARE) }

    /// The field-agreement probe, in millimetres: how far off each triangle a probe stands, the
    /// band a triangle's centroid must read within to withdraw a one-sided disagreement, and the
    /// value tolerance a probe's sign is read to. A mesh within the tolerance of the exact surface
    /// leaves a probe twice the tolerance off at least the tolerance clear of it on its own side,
    /// read to half the tolerance; a triangle within the tolerance of the surface is one lying
    /// on it. Never nearer than `LEAST_PROBE`.
    pub fn probe(&self) -> [f64;3] {
        let offset = (2.*self.millimetres).max(Self::LEAST_PROBE);
        [offset,offset/2.,offset/4.]
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
    /// spread them, which are the fit's. `around`, where it is some: the sheet's last column is
    /// its first, as a whole turn's is, and each band is cut into that many faces along the
    /// columns, closing on itself (`build::grid_sheet`, #684 N). `closed`: the sheet's last row is
    /// its first, as a sheet along a characteristic that closes on itself is, and its last band
    /// closes on its first's edges (#684 W).
    pub fn named(&self,kind: Parametrization,name: &dyn Fn(u32) -> String,around: Option<usize>,closed: bool) -> Result<Brep,String> {
        let bands = &self.sheet.bands;
        let Some(Surface::BSpline(_,net)) = self.face.faces.first().map(|f| &f.surface) else { return Ok(self.face.clone()) };
        if bands.len() < 2 && around.is_none() && !closed {
            let mut faces = self.face.clone();
            faces.faces[0].name = bands.first().map_or(String::new(),|b| name(b.1));
            return Ok(faces)
        }
        let [[a,z],[c,d]] = net.domain();
        let mut us = vec![a];
        if bands.len() > 1 {
            let (tu,_) = crate::nurbs::net_parameters(&self.sheet.points,self.sheet.rows,self.sheet.columns,kind)
                .ok_or("the sheet's rows have no parameters")?;
            us.extend(bands[1..].iter().map(|b| a+(z-a)*tu[b.0]));
        }
        us.push(z);
        let pieces = around.unwrap_or(1);
        let vs: Vec<f64> = (0..=pieces).map(|k| c+(d-c)*k as f64/pieces as f64).collect();
        let names: Vec<String> = bands.iter().map(|b| name(b.1)).collect();
        crate::build::grid_sheet(net,&us,&vs,&names,[closed,around.is_some()])
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
pub fn carry_out(name: &str,fitted: &mut Fitted,inside: Inside,near: &(dyn Fn(V) -> f64+Sync),allowance: f64,say: &Say)
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
    let carried = |net: &crate::nurbs::Net,multiple: f64| -> Option<crate::nurbs::Net> {
        sides.iter().try_fold(net.clone(),|n,&side| { let (k,high) = SIDES[side]; n.extended(k,high,multiple*span(k,high)) })
    };
    for multiple in CARRIED {
        let Some(wider) = carried(net,multiple) else { return Ok(()) };
        let face = crate::build::sheet(wider.clone())?;
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
        let nets: Vec<(crate::nurbs::Net,String)> = fitted.faces.faces.iter().enumerate().map(|(i,f)| {
            let Surface::BSpline(_,band) = &f.surface else { return Err("a sheet's face is not a B-spline".to_string()) };
            let band = sides.iter().try_fold((**band).clone(),|b,&side| {
                let (k,high) = SIDES[side];
                if k == 0 && (if high { i+1 != count } else { i != 0 }) { return Some(b) }
                b.extended(k,high,multiple*span(k,high))
            }).ok_or("a sheet's face is rational")?;
            Ok((band,f.name.clone()))
        }).collect::<Result<_,String>>()?;
        fitted.faces = if count < 2 {
            let mut faces = crate::build::sheet(nets[0].0.clone())?;
            faces.faces[0].name = nets[0].1.clone();
            faces
        } else { crate::build::sheets(nets)? };
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
    crate::util::par::indices(points.len(),|k| {
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
    let mut cells = crate::util::space::Grid::new(steps.get(steps.len()*19/20).copied().unwrap_or(1.).max(1e-9));
    for (k,(p,_)) in grid.iter().enumerate() { cells.insert(*p,k as u32); }
    points.iter().map(|&p| {
        let mut best = (f64::INFINITY,usize::MAX);
        cells.around(p,|k| { let e = distance(grid[k as usize].0,p); if e < best.0 { best = (e,k as usize); } });
        if best.1 == usize::MAX { [f64::NAN;2] } else { [(best.1/nv) as f64/(nu-1) as f64,(best.1%nv) as f64/(nv-1) as f64] }
    }).collect()
}

/// A sheet grid interpolated as a face.
fn fit(sheet: &Sheet,kind: Parametrization) -> Result<Brep,String> {
    // creased where the sweep has a sharp edge (#684 O), smooth either side
    let net = interpolate_creased(&sheet.points,sheet.rows,sheet.columns,kind,&sheet.creases)
        .ok_or("the sheet grid could not be interpolated")?;
    crate::build::sheet(net)
}

/// A fitted sheet's verdict at its withheld contacts: it fits, or, held to a tolerance, it misses
/// at the contacts marked, which a finer grid may mend.
enum Judged { Fits(Brep,f64,Parametrization),Misses {over: Vec<bool>,refusal: Refused} }

/// The fit contract (as the native host's): at every withheld contact near the blank the face
/// passes near the true contact and its normal agrees with the contact's, and nowhere in the blank
/// does its own normal turn back between points a quarter of a cell apart. Without a tolerance the
/// bars are gross, the sheet fitted by centripetal parameters (this kernel's interpolation, its
/// knots averaged from the parameters, overshoots the pinion's unevenly spaced rows by chord length:
/// it folded); held to one, chord-length and centripetal fits are both read and the better kept.
fn judged(name: &str,sheet: &Sheet,scale: f64,near: &(dyn Fn(V) -> f64+Sync),tolerance: Option<Tolerance>,say: &Say)
    -> Result<Judged,Refused> {
    let at = |p: V| p.map(|x| (x*1e3).round()/1e3);
    let (nu,nv) = (4*(sheet.rows-1)+1,4*(sheet.columns-1)+1);
    // a crease of the fit, where its normal turns at a sharp edge of the sweep (#684 O), is no fold:
    // the readings either side of one are not compared
    let creased = |face: &Brep,i: usize| -> bool {
        let Some(Surface::BSpline(_,net)) = face.faces.first().map(|f| &f.surface) else { return false };
        let [[a,z],_] = net.domain();
        let (u0,u1) = (a+(z-a)*i as f64/(nu-1) as f64,a+(z-a)*(i+1) as f64/(nu-1) as f64);
        crate::nurbs::distinct(&net.uknots).into_iter().any(|(k,m)| m >= net.du && k >= u0 && k <= u1 && k > a && k < z)
    };
    let folds = |face: &Brep,grid: Vec<(V,V)>| -> Result<Vec<(V,V)>,Refused> {
        let (mut fold,mut folded) = (0_f64,[0.;3]);
        for i in 0..nu {
            let across = !sheet.creases.is_empty() && i+1 < nu && creased(face,i);
            for j in 0..nv {
                let (p,n) = grid[i*nv+j];
                let beside = [(i+1 < nu && !across).then(|| grid[(i+1)*nv+j]),(j+1 < nv).then(|| grid[i*nv+j+1])];
                for (q,m) in beside.into_iter().flatten() {
                    let angle = dot(n,m).clamp(-1.,1.).dacos().to_degrees();
                    if angle > fold && (near(p) < 0. || near(q) < 0.) { fold = angle; folded = p; }
                }
            }
        }
        if fold > FOLD_TURN {
            return Err(Refused {stage:Stage::Withheld,witness:Some(folded.map(|x| x/scale)),
                message:format!("`{name}`: the fitted sheet folds in the blank: its normal turns {fold:.1} degrees between points a \
                quarter of a cell apart (at {:?}), against {FOLD_TURN} degrees",at(folded))})
        }
        Ok(grid)
    };
    let Some(tolerance) = tolerance else {
        // by centripetal parameters, and where that fit leaves its contacts or folds, by chord length
        // (the two fail on different sheets: centripetal where rows are evenly spaced and a corner is
        // sharp, chord length where they are not)
        let gross = |kind: Parametrization| -> Result<(Brep,f64,Parametrization),Refused> {
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
            return Err(Refused {stage:Stage::Withheld,witness:Some(witness.map(|x| x/scale)),
                message:format!("`{name}`: the fitted sheet leaves its contacts: {error:.3} mm (at {:?}) and {turn:.1} degrees (at {:?}), \
                against {FIT_DISTANCE} mm and {FIT_TURN} degrees",at(worst),at(turned))})
        }
        folds(&face,grid)?;
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
    let fitted = |kind: Parametrization| -> Result<Candidate,Refused> {
        let face = fit(sheet,kind).at(Stage::Fit)?;
        let grid = folds(&face,surface_grid(&face,nu,nv))?;
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
        return Ok(Judged::Misses {over:if distant > 0 { far } else { turned },refusal:Refused {stage:Stage::Withheld,
            witness:Some(witness.map(|x| x/scale)),
            message:format!("`{name}`: the fitted sheet misses {distant} of its {} withheld contacts by distance, {:.2} µm from one \
            (at {:?}) against {:.2} µm, half the {} µm tolerance, and {bent} by normal, one {:.2} degrees off (at {:?}) against its \
            {:.2}",points.len(),error*1e3,at(worst),bar*1e3,tolerance.millimetres*1e3,turn.0,at(turn.2),turn.1)}})
    }
    (say.stage)(&format!("`{name}`: the {kind:?} fit holds"));
    (say.mark)(Stage::Withheld);
    Ok(Judged::Fits(face,error,kind))
}

/// A sheet laid out on a grid fitted and judged: `first` (centres withheld) as it is without a
/// tolerance, else `lay` on `grid` (sides withheld) refined where it misses until it fits or the
/// refinements run out. A fit that misses its contacts or folds is the inner refusal (another
/// layout may fit), anything else the outer. `how` says how the grid was laid, for the refusal.
#[allow(clippy::too_many_arguments)]
pub fn settle(name: &str,scale: f64,first: Sheet,grid: &Grid,lay: &(dyn Fn(&Grid) -> Result<Sheet,String>+Sync),inside: Inside,
    near: &(dyn Fn(V) -> f64+Sync),tolerance: Option<Tolerance>,how: &str,say: &Say)
    -> Result<Result<(Fitted,Parametrization),Refused>,Refused> {
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
            return Err(Refused::at(Stage::Sheet,format!("`{name}`: the refined sheet is not one regular chart: {fault}")))
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
