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
use gcs_core::space::{cross,dot,norm,scale,sub};

type V = [f64;3];

/// The bevel pair at 24 and 48 teeth of module 2, one tooth space each.
fn pair() -> Elaborated {
    let configuration = fixtures::gear::configuration([24,48],2.,0.,0.,35.);
    fixtures::read_beside(&fixtures::gear::source(),&fixtures::gear::project(),&mut |name,text| {
        if name == "configuration" { configuration.clone() }
        else if name == "members" { fixtures::gear::one_space(&text) }
        else { text }
    })
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

/// How far `p` is from the envelope along the normal `n` there: the envelope's point on the line
/// through `p` along `n`, sought from the nearest of its contacts over a grid of its source.
fn off_envelope(envelope: &GeneratedEnvelope,p: V,n: V) -> f64 {
    let [u,v,roll] = envelope.domain();
    let mut best: Option<([f64;3],f64)> = None;
    for i in 0..=12 { for j in 0..=96 {
        let (a,b) = (u[0]+(u[1]-u[0])*i as f64/12.,v[0]+(v[1]-v[0])*j as f64/96.);
        let nv = |t: f64| envelope.evaluate([a,b,t]).map_or(f64::NAN,|c| c.normal_velocity);
        let ts: Vec<f64> = (0..=64).map(|k| roll[0]+(roll[1]-roll[0])*k as f64/64.).collect();
        for w in ts.windows(2) {
            let (mut lo,mut hi) = (w[0],w[1]);
            if !(nv(lo)*nv(hi) <= 0.) { continue }
            for _ in 0..40 { let m = 0.5*(lo+hi); if nv(lo)*nv(m) <= 0. { hi = m } else { lo = m } }
            let Ok(c) = envelope.evaluate([a,b,lo]) else { continue };
            let d = norm(sub(c.position,p));
            if best.is_none_or(|(_,e)| d < e) { best = Some(([a,b,lo],d)); }
        }
    } }
    let Some((seed,coarse)) = best else { return f64::INFINITY };
    let e1 = { let a = if n[0].abs() < 0.9 { [1.,0.,0.] } else { [0.,1.,0.] }; let c = cross(n,a); scale(c,1./norm(c)) };
    let e2 = cross(n,e1);
    match envelope.intersect(|_,c| { let d = sub(c.position,p); [dot(d,e1),dot(d,e2)] },seed,IntersectionOptions {
        bounds:[u,v,roll],parameter_scale:[1.;3],residual_tolerance:[1e-9,1e-9,1e-9],max_iterations:100}) {
        Ok(found) => norm(sub(found.contact.position,p)),
        Err(_) => coarse,
    }
}

#[test]
fn a_swept_bodys_faces_lie_on_what_their_names_say() {
    let e = pair();
    let sk = &e.sketch;
    for (member,cut) in [("pinion",["inner","inner_round","tip","outer_round","outer"]),
        ("gear",["outer_crown.inner","outer_crown.inner_round","outer_crown.close1","neighbor.outer_round","neighbor.outer"])] {
        let body = fixtures::solid(&e,&format!("pair.{member}.body"));
        let faces = BodyFaces::read(sk,body).unwrap_or_else(|why| panic!("{member}: {why}"));
        let named: Vec<String> = faces.paths().iter().map(|p| p.text()).collect();
        // every face the build named, and the definitions name it too
        let structural: Vec<String> = face_paths(sk,body).iter().map(|p| p.text()).collect();
        for face in cut.iter().map(|f| format!("indexed.{f}")).chain(["tip","heel","toe","back"].map(|f| format!("material.{f}"))) {
            assert!(named.contains(&face),"{member}: no `{face}` among {named:?}");
            assert!(structural.contains(&face),"{member}: `{face}` is not among the definitions' {structural:?}");
        }
        let sources: Vec<(String,FaceSource)> = faces.paths().iter().map(|p| (p.text(),face_source(sk,body,p)
            .unwrap_or_else(|why| panic!("{member}: `{}`: {why}",p.text())))).collect();
        let envelopes: Vec<(String,GeneratedEnvelope)> = sources.iter().filter_map(|(name,s)| match s {
            FaceSource::Swept {surface,motion,roll,..} =>
                Some((name.clone(),GeneratedEnvelope::new(name.clone(),surface.clone(),motion.clone(),*roll).unwrap())),
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
                    for (other,envelope) in &envelopes {
                        let d = off_envelope(envelope,p,n);
                        println!("{member}: `{name}` is {d:.2e} mm off `{other}`'s envelope");
                        if other == name { assert!(d < 0.03,"{member}: `{name}` is {d} mm off its envelope"); }
                        else { assert!(d > 0.1,"{member}: `{name}` lies on `{other}`'s envelope ({d} mm)"); }
                    }
                }
            }
        }
    }
}
