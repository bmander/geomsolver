//! A body cut by sweeps of the planar subtraction class (`solid::planar_subtraction`, #606) built by
//! this kernel: in each slab of the blank, the region the tool's section sweeps
//! (`envelope::swept_region`) carried along the plane's normal through the slab — a hair past the
//! blank where the slab ends at its face — and taken from the blank, at every placement of every
//! sweep. A loop of the region is a profile's: each kept stretch a cubic B-spline fitted through
//! its exact points, each step along the box a line, its corners the stretches' shared ends; a
//! loop counter-clockwise is an outer loop, one clockwise a hole in the one about it. A drafted
//! slab (#607) is lofted between its stations, each stretch a B-spline surface through the
//! stretch's points at every station. Nothing is traced, split or judged cell by cell.
#[allow(unused_imports)]
use crate::fmath::Det;
use super::{Prepared,Say};
use crate::brep::boolean::Op;
use crate::brep::topo::Brep;
use crate::envelope::swept_region::{Piece,SweptRegion};
use crate::json::{object,Json};
use crate::model::Sketch;
use crate::solid::admission::Class;
use crate::solid::cad::{self,StaticRecipe};
use crate::solid::export::{AtStage,ExportRefusal,Stage,Tolerance};

type P = [f64;2];

/// Whether every sweep of the body was admitted to the planar subtraction class.
pub fn all(prepared: &Prepared) -> bool { prepared.classes.iter().all(|c| matches!(c,Class::Subtraction(_))) }

/// A stretch's points at fractions of it, from a region's loop's piece.
fn sampler<'a>(region: &'a SweptRegion,l: usize,k: usize) -> impl Fn(f64,f64,usize) -> Vec<(f64,f64)>+'a {
    move |lo: f64,hi: f64,m: usize| (0..=m).map(|j| {
        let p = region.point(l,k,lo+(hi-lo)*j as f64/m as f64).unwrap_or([f64::NAN;2]);
        (p[0],p[1])
    }).collect()
}

/// The region's loops as profiles (millimetres): each counter-clockwise loop with the clockwise
/// loops within it as its holes.
fn profiles(region: &SweptRegion,scale: f64,fit: f64,name: &str) -> Result<Vec<Json>,ExportRefusal> {
    let plane = region.setting.plane;
    let lift = |p: (f64,f64)| plane.world([p.0,p.1],0.).map(|x| x*scale);
    let vec = |v: [f64;3]| Json::Arr(v.iter().map(|&x| x.into()).collect());
    let point = |p: P| vec(lift((p[0],p[1])));
    let edges = |l: usize| -> Result<Json,ExportRefusal> {
        let lp = &region.loops[l];
        let n = lp.pieces.len();
        // a loop of one stretch (a whole ring) is split at its middle: an edge has two ends
        let halves: Vec<(usize,[f64;2])> = if n == 1 { vec![(0,[0.,0.5]),(0,[0.5,1.])] } else { (0..n).map(|k| (k,[0.,1.])).collect() };
        Ok(Json::Arr(halves.into_iter().map(|(k,[f0,f1])| {
            let (Some(a),Some(b)) = (region.point(l,k,f0),region.point(l,k,f1)) else {
                return Err(ExportRefusal::at(Stage::Reach,format!("`{name}`: its swept region's boundary cannot be read")))
            };
            match lp.pieces[k] {
                Piece::Side {..} => Ok(object([("kind","line".into()),("start",point(a)),("end",point(b))])),
                Piece::Run {..} => {
                    let (poles,knots,err) = cad::fit_sampled(&sampler(region,l,k),(f0,f1),Some(((a[0],a[1]),(b[0],b[1]))),fit)
                        .ok_or_else(|| ExportRefusal::at(Stage::Reach,format!("`{name}`: a stretch of its swept region's boundary could \
                            not be fitted within {:.1e} mm",fit*scale)))?;
                    Ok(cad::bspline_edge(&poles,&knots,err*scale,&lift))
                }
            }
        }).collect::<Result<Vec<_>,_>>()?))
    };
    let mut out = Vec::new();
    for (l,holes) in region.nesting(8) {
        let loops = std::iter::once(l).chain(holes).map(edges).collect::<Result<Vec<_>,_>>()?;
        out.push(object([("loops",Json::Arr(loops)),("origin",vec(plane.o.map(|x| x*scale))),("normal",vec(plane.n))]));
    }
    Ok(out)
}

/// The body: its blank less each subtraction sweep's swept region carried through each slab, at
/// each placement.
pub fn cut(sk: &Sketch,body: usize,recipe: &StaticRecipe,prepared: &Prepared,tolerance: Option<Tolerance>,say: &Say) -> Result<Brep,ExportRefusal> {
    let scale = prepared.field.scale();
    // as a curve in a static profile is fitted, within the tolerance's share for the fit when finer
    let fit = tolerance.map_or(cad::FIT_MM,|t| t.fit().min(cad::FIT_MM))/scale;
    let mut solid = prepared.blank.clone();
    for (k,&swept) in prepared.distinct.iter().enumerate() {
        let name = &sk.solids[swept].name;
        let Class::Subtraction(found) = &prepared.classes[k] else {
            return Err(ExportRefusal::at(Stage::Admission,format!("`{name}` is not a planar subtraction sweep")))
        };
        let clock = crate::clock::Instant::now();
        let mut cutters: Vec<Brep> = Vec::new();
        for slab in &found.slabs {
            match slab.stations.as_slice() {
                [(_,region)] => for profile in profiles(region,scale,fit,name)? {
                    let p = crate::brep::build::Profile::from_json(&profile).at(Stage::Reach)?;
                    cutters.push(crate::brep::build::prism(&p,slab.reach[0]*scale,slab.reach[1]*scale).at(Stage::Reach)?);
                },
                stations => cutters.extend(lofted(stations,scale,fit,name)?),
            }
        }
        (say.stage)(&format!("`{name}`: its swept region fitted in {} slabs, {} loops, and carried through the blank ({:?})",
            found.slabs.len(),cutters.len(),clock.elapsed()));
        (say.mark)(Stage::Reach);
        for c in recipe.sweeps.iter().filter(|c| c.swept == swept) {
            for cutter in &cutters {
                let placed = cutter.moved(&super::sector::placed(c.pose,scale));
                solid = crate::brep::boolean::combined(&solid,&placed,Op::Cut,0.)
                    .map_err(|e| ExportRefusal::at(Stage::Split,format!("`{}`: less `{name}`'s swept region: {e}",sk.solids[body].name)))?;
            }
        }
    }
    (say.mark)(Stage::Split);
    Ok(solid)
}

/// A drafted slab's cutters (#607): its stations' regions, alike loop for loop and piece for piece,
/// lofted through every other station — each piece a B-spline surface, cubic along the piece through
/// its points at the same fractions at every station, cubic across them — and held at the stations
/// between, within the fit's bar, at its quarter points and middle. A loop counter-clockwise is a
/// cutter's outer loop, the clockwise loops within it its holes, as a prism's are.
fn lofted(stations: &[(f64,SweptRegion)],scale: f64,fit: f64,name: &str) -> Result<Vec<Brep>,ExportRefusal> {
    use crate::brep::nurbs::{BSpline,Net};
    let refused = |why: String| ExportRefusal::at(Stage::Reach,format!("`{name}`: {why}"));
    let (base,first) = (&stations[0].1,stations[0].0);
    let plane = base.setting.plane;
    // each station's loop matching each of the first's, and how far round it starts: admission
    // read them alike (`planar_subtraction::Subtraction::check`)
    let matched: Vec<Vec<(usize,usize)>> = stations.iter().map(|(z,region)| region.matching(base)
        .ok_or_else(|| refused(format!("its drafted tool's swept region changes its shape between heights {first:.4} and {z:.4}"))))
        .collect::<Result<_,_>>()?;
    // the lofted stations, every other one; the ones between hold the loft to the bar
    let (lofted,held): (Vec<usize>,Vec<usize>) = (0..stations.len()).partition(|j| j%2 == 0);
    if lofted.len() < 4 { return Err(refused("a drafted slab is lofted through four stations at least".into())) }
    let (z0,z1) = (stations[lofted[0]].0,stations[*lofted.last().expect("stations")].0);
    let along = |z: f64| (z-z0)/(z1-z0);
    let point = |j: usize,l: usize,k: usize,f: f64| -> Option<P> {
        let (m,r) = matched[j][l];
        let region = &stations[j].1;
        let n = region.loops[m].pieces.len();
        region.point(m,(k+r)%n,f)
    };
    let interpolate = |pts: &[(f64,f64)],at: &[f64]| crate::curve::interpolating_ctrl_at(pts,at.to_vec())
        .ok_or_else(|| refused("its drafted tool's swept region could not be interpolated".into()));
    let vs: Vec<f64> = lofted.iter().map(|&j| along(stations[j].0)).collect();
    // across the stations each pole's column is interpolated, its heights the stations'
    let heights: Vec<(f64,f64)> = lofted.iter().map(|&j| (stations[j].0,0.)).collect();
    let (hs,vknots,_) = interpolate(&heights,&vs)?;
    let net_of = |l: usize,k: usize| -> Result<Net,ExportRefusal> {
        let mut samples = 8;
        loop {
            let fractions: Vec<f64> = (0..=samples).map(|i| i as f64/samples as f64).collect();
            // along the piece at each lofted station, the same fractions: the same knots
            let mut ctrl: Vec<Vec<(f64,f64)>> = Vec::new();
            let mut uknots = Vec::new();
            let mut worst: f64 = 0.;
            for &j in &lofted {
                let pts: Vec<(f64,f64)> = fractions.iter().map(|&f| point(j,l,k,f).map(|p| (p[0],p[1]))
                    .ok_or_else(|| refused("its drafted tool's swept region cannot be read".into()))).collect::<Result<_,_>>()?;
                let (c,knots,t) = interpolate(&pts,&fractions)?;
                let curve = BSpline::new(crate::curve::DEGREE,knots.clone(),c.iter().map(|&(x,y)| [x,y,0.]).collect()).map_err(refused)?;
                for i in 0..samples {
                    let f = 0.5*(fractions[i]+fractions[i+1]);
                    let Some(e) = point(j,l,k,f) else { continue };
                    let q = curve.point(0.5*(t[i]+t[i+1]));
                    worst = worst.max((q[0]-e[0]).dhypot(q[1]-e[1]));
                }
                ctrl.push(c); uknots = knots;
            }
            if worst > fit && samples < 1024 { samples *= 2; continue }
            let mut poles: Vec<Vec<[f64;3]>> = Vec::new();
            for i in 0..ctrl[0].len() {
                let column: Vec<(f64,f64)> = ctrl.iter().map(|c| c[i]).collect();
                let (c,_,_) = interpolate(&column,&vs)?;
                poles.push(c.iter().zip(&hs).map(|(&(x,y),&(h,_))| plane.world([x,y],h).map(|x| x*scale)).collect());
            }
            return Ok(Net {du:crate::curve::DEGREE,dv:crate::curve::DEGREE,uknots,vknots:vknots.clone(),poles,weights:None})
        }
    };
    let pieces: Vec<(usize,usize)> = (0..base.loops.len()).flat_map(|l| (0..base.loops[l].pieces.len()).map(move |k| (l,k))).collect();
    let mut built = crate::par::map(&pieces,|&(l,k)| net_of(l,k)).into_iter();
    let mut nets: Vec<Vec<Net>> = Vec::new();
    for l in 0..base.loops.len() {
        nets.push(built.by_ref().take(base.loops[l].pieces.len()).collect::<Result<_,_>>()?);
    }
    // the stations between, read off the loft: a piece's quarter points and middle
    for &j in &held {
        let v = along(stations[j].0);
        for (l,ring) in nets.iter().enumerate() {
            for (k,net) in ring.iter().enumerate() {
                let [[u0,u1],_] = net.domain();
                for f in [0.25,0.5,0.75] {
                    let (Some(e),q) = (point(j,l,k,f),net.point(u0+(u1-u0)*f,v)) else { continue };
                    let q = plane.page(q.map(|x| x/scale));
                    let off = (q[0]-e[0]).dhypot(q[1]-e[1]);
                    if off > fit.max(1e-9) {
                        return Err(refused(format!("its drafted tool's swept region bends between its stations: lofted, it passes {:.2e} mm \
                            from the region at height {:.4}",off*scale,stations[j].0)))
                    }
                }
            }
        }
    }
    // a cutter of each outer loop and the holes within it
    let mut out = Vec::new();
    for (l,holes) in base.nesting(8) {
        let rings = std::iter::once(l).chain(holes).map(|h| nets[h].clone()).collect();
        out.push(crate::brep::build::lofted(plane.o.map(|x| x*scale),plane.n,[z0*scale,z1*scale],rings).map_err(refused)?);
    }
    Ok(out)
}
