//! **A swept cut's faces, named by the cutter's faces that made them** (#212): a body cut by a
//! generating sweep is built with its sheet cut where the cutter's faces meet, each stretch named
//! by the face that made it (`brep::sweep::sheet::Fitted::faces`), and a boolean never renames —
//! so the built body's faces read as sets by name (`BodyFaces`), its blank's by the region's terms.
//! Held here against the definitions alone (`face_paths`, `face_source`): each named face of the
//! built members of the bevel pair, one tooth space each, lies on the analytic surface its name
//! says — a cut face on its cutter face's envelope and on no other's, a blank face on its term's
//! surface of revolution.
use gcs_core::envelope::{GeneratedEnvelope,IntersectionOptions};
use gcs_core::program::Elaborated;
use gcs_core::solid::faces::{face_paths,face_source,BodyFaces,FaceSource};
use gcs_core::roots;
use gcs_core::space::{across,distance,dot,sub};

type V = [f64;3];

/// The bevel pair at 24 and 48 teeth of module 2, one tooth space each.
fn pair() -> Elaborated {
    fixtures::gear::read(&fixtures::gear::one_space(&fixtures::gear::at([24,48],2.,0.,0.,35.)))
}

/// A point of face `fi` well inside it (model units), and the face's normal there: its surface at
/// the middle of its outer loop's parameters.
fn inside(faces: &BodyFaces,fi: usize) -> (V,V) {
    let (b,mm,origin) = faces.brep();
    let f = &b.faces[fi];
    let uses = &f.loops[0];
    let mut uv = [0.;2];
    let mut n = 0.;
    for c in uses {
        let e = &b.edges[c.edge as usize];
        for k in 0..8 {
            let t = e.t[0]+(e.t[1]-e.t[0])*(k as f64+0.5)/8.;
            let q = c.pcurve.at(t,e,&f.surface,&b.vertices);
            uv = [uv[0]+q[0],uv[1]+q[1]];
            n += 1.;
        }
    }
    let q = f.surface.point([uv[0]/n,uv[1]/n]);
    let p = std::array::from_fn(|k| q[k]/mm+origin[k]);
    (p,faces.normal(fi,p))
}

/// An envelope's contacts over a grid of its source (13 × 97 points, each roll where the normal
/// velocity changes sign over 64 steps): each one's parameters and place, where a search for
/// the envelope's point near a place starts.
fn contacts(envelope: &GeneratedEnvelope) -> Vec<([f64;3],V)> {
    let [u,v,roll] = envelope.domain();
    let ts: Vec<f64> = (0..=64).map(|k| roll[0]+(roll[1]-roll[0])*k as f64/64.).collect();
    let mut out = Vec::new();
    for i in 0..=12 { for j in 0..=96 {
        let (a,b) = (u[0]+(u[1]-u[0])*i as f64/12.,v[0]+(v[1]-v[0])*j as f64/96.);
        let nv = |t: f64| envelope.evaluate([a,b,t]).map_or(f64::NAN,|c| c.normal_velocity);
        let at: Vec<f64> = ts.iter().map(|&t| nv(t)).collect();
        for k in 0..64 {
            if !(at[k]*at[k+1] <= 0.) { continue }
            let (lo,_) = roots::bisect(ts[k],ts[k+1],|a,b| 0.5*(a+b),|a,b| b-a > (ts[1]-ts[0])*1e-12,
                |m| at[k]*nv(m) > 0.);
            if let Ok(c) = envelope.evaluate([a,b,lo]) { out.push(([a,b,lo],c.position)); }
        }
    } }
    out
}

/// How far `p` is from the envelope along the normal `n` there: the envelope's point on the line
/// through `p` along `n`, sought from the nearest of its `contacts`.
fn off_envelope(envelope: &GeneratedEnvelope,contacts: &[([f64;3],V)],p: V,n: V) -> f64 {
    let Some(&(seed,q)) = contacts.iter().min_by(|a,b| distance(a.1,p).total_cmp(&distance(b.1,p))) else { return f64::INFINITY };
    let (e1,e2) = across(n);
    let [u,v,roll] = envelope.domain();
    match envelope.intersect(|_,c| { let d = sub(c.position,p); [dot(d,e1),dot(d,e2)] },seed,IntersectionOptions {
        bounds:[u,v,roll],parameter_scale:[1.;3],residual_tolerance:[1e-9,1e-9,1e-9],max_iterations:100}) {
        Ok(found) => distance(found.contact.position,p),
        Err(_) => distance(q,p),
    }
}

#[test]
fn a_swept_bodys_faces_lie_on_what_their_names_say() {
    let e = pair();
    let sk = &e.sketch;
    for (member,cut) in [("pinion",["inner","inner_round","tip","outer_round","outer"]),
        ("gear",["outer_crown.inner","outer_crown.inner_round","outer_crown.tip","neighbor.outer_round","neighbor.outer"])] {
        let body = fixtures::solid(&e,&format!("pair.{member}.body"));
        let faces = BodyFaces::read(sk,body).unwrap_or_else(|why| panic!("{member}: {why}"));
        let named: Vec<String> = faces.paths().iter().map(|p| p.text()).collect();
        // every face the build named, and the definitions name it too
        let structural: Vec<String> = face_paths(sk,body).iter().map(|p| p.text()).collect();
        for face in cut.iter().map(|f| format!("indexed[0].{f}")).chain(["tip","heel","toe","back"].map(|f| format!("material.{f}"))) {
            assert!(named.contains(&face),"{member}: no `{face}` among {named:?}");
            assert!(structural.contains(&face),"{member}: `{face}` is not among the definitions' {structural:?}");
        }
        let sources: Vec<(String,FaceSource)> = faces.paths().iter().map(|p| (p.text(),face_source(sk,body,p)
            .unwrap_or_else(|why| panic!("{member}: `{}`: {why}",p.text())))).collect();
        let envelopes: Vec<(String,GeneratedEnvelope,Vec<([f64;3],V)>)> = sources.iter().filter_map(|(name,s)| match s {
            FaceSource::Swept {surface,motion,roll,..} => {
                let envelope = GeneratedEnvelope::new(name.clone(),surface.clone(),motion.clone(),*roll).unwrap();
                let contacts = contacts(&envelope);
                Some((name.clone(),envelope,contacts))
            }
            FaceSource::Revolved {..} => None,
        }).collect();
        assert_eq!(envelopes.len(),5,"{member}: {named:?}");
        for (fi,(name,source)) in sources.iter().enumerate() {
            let (p,n) = inside(&faces,fi);
            match source {
                // a blank face on its term's surface, exactly
                FaceSource::Revolved {surface,..} => {
                    let on = surface.projector().unwrap().project(p).unwrap();
                    assert!(on.incidence_error < 1e-9,"{member}: `{name}` is {} off its term",on.incidence_error);
                }
                // a cut face on its cutter face's envelope, within the sheet's fit, and well off
                // every other face's of the cutter
                FaceSource::Swept {pose,..} => {
                    assert!(pose.point([1.,2.,3.]).iter().zip([1.,2.,3.]).all(|(a,b)| (a-b).abs() < 1e-12),
                        "{member}: the first tooth space is cut where its sweep is");
                    for (other,envelope,contacts) in &envelopes {
                        let d = off_envelope(envelope,contacts,p,n);
                        println!("{member}: `{name}` is {d:.2e} mm off `{other}`'s envelope");
                        if other == name { assert!(d < 0.03,"{member}: `{name}` is {d} mm off its envelope"); }
                        else { assert!(d > 0.1,"{member}: `{name}` lies on `{other}`'s envelope ({d} mm)"); }
                    }
                }
            }
        }
    }
}
