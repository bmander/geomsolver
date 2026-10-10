//! A body cut by sweeps of the planar generating class (`solid::planar_class`, #65) built by this
//! kernel: what a pocketed prism turned through a whole period leaves of the blank is the blank
//! within the pocket's inner envelope carried along the prisms. The envelope is fitted a cubic
//! B-spline a piece, its corners the pieces' shared ends, extruded through the blank and kept in
//! common with it, at every placement of every sweep. Nothing is traced or split: the envelope's
//! own contacts, solved exactly (`InnerEnvelope::point`), are what the splines are fitted through.
use super::{Prepared,Say};
use crate::brep::boolean::Op;
use crate::brep::topo::Brep;
use crate::json::{object,Json};
use crate::model::Sketch;
use crate::solid::admission::Class;
use crate::solid::cad::{self,StaticRecipe};
use crate::solid::export::{AtStage,ExportRefusal,Stage,Tolerance};

/// Whether every sweep of the body was admitted to the planar class.
pub fn all(prepared: &Prepared) -> bool { prepared.classes.iter().all(|c| matches!(c,Class::Planar(_))) }

/// The body: its blank kept in common with each planar sweep's envelope prism, at each placement.
pub fn cut(sk: &Sketch,body: usize,recipe: &StaticRecipe,prepared: &Prepared,tolerance: Option<Tolerance>,say: &Say) -> Result<Brep,ExportRefusal> {
    let scale = prepared.field.scale();
    // as a curve in a static profile is fitted, within the tolerance's share for the fit when finer
    let fit = tolerance.map_or(cad::FIT_MM,|t| t.fit().min(cad::FIT_MM))/scale;
    let mut solid = prepared.blank.clone();
    for (k,&swept) in prepared.distinct.iter().enumerate() {
        let Class::Planar(found) = &prepared.classes[k] else {
            return Err(ExportRefusal::at(Stage::Admission,format!("`{}` is not a planar sweep",sk.solids[swept].name)))
        };
        let clock = crate::clock::Instant::now();
        let env = &found.envelope;
        let n = env.normal();
        let lift = |p: (f64,f64)| env.world([p.0,p.1]).map(|x| x*scale);
        // one B-spline a piece, from its corner to the next
        let edges = (0..env.pieces.len()).map(|piece| {
            let ends = [env.point(piece,0.),env.point(piece,1.)];
            let (Some(a),Some(b)) = (ends[0],ends[1]) else {
                return Err(ExportRefusal::at(Stage::Reach,format!("`{}`: the envelope's piece {piece} has no ends",sk.solids[swept].name)))
            };
            let sweep = |lo: f64,hi: f64,m: usize| -> Vec<(f64,f64)> { (0..=m).map(|j| {
                let p = env.point(piece,lo+(hi-lo)*j as f64/m as f64).unwrap_or([f64::NAN;2]);
                (p[0],p[1])
            }).collect() };
            let (poles,knots,err) = cad::fit_sampled(&sweep,(0.,1.),Some(((a[0],a[1]),(b[0],b[1]))),fit)
                .ok_or_else(|| ExportRefusal::at(Stage::Reach,format!("`{}`: the envelope's piece {piece} could not be fitted within \
                    {:.1e} mm",sk.solids[swept].name,fit*scale)))?;
            Ok(cad::bspline_edge(&poles,&knots,err*scale,&lift))
        }).collect::<Result<Vec<_>,_>>()?;
        let origin = env.world([0.,0.]);
        let vec = |v: [f64;3]| Json::Arr(v.iter().map(|&x| x.into()).collect());
        let profile = object([("loops",Json::Arr(vec![Json::Arr(edges)])),("origin",vec(origin.map(|x| x*scale))),("normal",vec(n))]);
        // through the blank, a hair past it each way
        let at = crate::space::dot(origin,n);
        let [h0,h1] = found.heights;
        let pad = 1e-3*(h1-h0)+1e-6;
        let prism = crate::brep::build::prism(&crate::brep::build::Profile::from_json(&profile).at(Stage::Reach)?,
            (h0-pad-at)*scale,(h1+pad-at)*scale).at(Stage::Reach)?;
        (say.stage)(&format!("`{}`: its inner envelope fitted, {} pieces, and carried through the blank ({:?})",sk.solids[swept].name,
            env.pieces.len(),clock.elapsed()));
        (say.mark)(Stage::Reach);
        for c in recipe.sweeps.iter().filter(|c| c.swept == swept) {
            let placed = prism.moved(&super::sector::placed(c.pose,scale));
            solid = crate::brep::recipe::combined(&solid,&placed,Op::Common,0.)
                .map_err(|e| ExportRefusal::at(Stage::Split,format!("`{}`: kept within `{}`'s envelope: {e}",sk.solids[body].name,sk.solids[swept].name)))?;
        }
    }
    (say.mark)(Stage::Split);
    Ok(solid)
}
