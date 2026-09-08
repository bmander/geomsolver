//! Spatial face loops are built after their generated surfaces and finite edges.
use super::{resolve::{follow_building,Resolver},Code,Diag,Made,SourceMap};
use crate::{ir::{Kid,Operation,Statement},model::{EntKind,EntRef,FaceE,FaceSupport,Sketch},syntax::StmtId};
use std::collections::BTreeSet;

pub(super) fn faces(sk: &mut Sketch,res: &mut Resolver,map: &mut SourceMap,
    body: &[&Statement],skip: &BTreeSet<StmtId>,diags: &mut Vec<Diag>) {
    for st in body {
        let Operation::Decl(d) = &st.kind else { continue; };
        if d.kind != EntKind::Face || skip.contains(&st.id)
            || d.children.get(2).is_none_or(Vec::is_empty) { continue; }
        let build = || -> Result<FaceE,String> {
            if d.closed { return Err("spatial faces close through shared vertices, without `-> close`".into()); }
            if d.children.get(1).is_some_and(|g| !g.is_empty()) {
                return Err("spatial faces currently take one boundary loop".into());
            }
            let resolve = |k: &Kid| -> Result<EntRef,String> {
                let Kid::Ref(r) = k else { return Err("a spatial face names existing edges and support".into()); };
                let e = res.lookup(r).ok_or_else(|| format!("no such entity: `{}`",r.root.text))?;
                let e = follow_building(sk,res,e,r)?;
                if e.i() >= sk.count(e.kind) { return Err(format!("`{}` could not be built",r.root.text)); }
                Ok(e)
            };
            let [on] = d.children[2].as_slice() else { return Err("a face needs exactly one `on:` support".into()); };
            let edges = d.children[0].iter().map(resolve).collect::<Result<Vec<_>,_>>()?;
            let edge_names = d.children[0].iter().map(|k| {
                let Kid::Ref(r) = k else { unreachable!() };
                match r.path.last() {
                    Some(crate::syntax::Seg::Field(n)) => n.text.clone(),
                    _ => r.root.text.rsplit('.').next().unwrap_or(&r.root.text).to_string(),
                }
            }).collect();
            let face = FaceE {edges,edge_names,holes:vec![],support:FaceSupport::Surface(resolve(on)?),
                name:d.name.key().text.clone(),class:d.class.clone()};
            crate::spatial_face::boundary(sk,&face)?;
            Ok(face)
        };
        match build() {
            Ok(face) => {
                let e = EntRef::face(sk.faces.len());
                res.of.insert(face.name.clone(),e);
                map.bind(&face.name,e,d.name.named()); map.record(st,Made::Ent(e));
                sk.faces.push(face);
            }
            Err(message) => {
                diags.push(Diag {code:Code::E080,span:st.span,stmt:Some(st.id),message});
                res.of.remove(&d.name.key().text);
            }
        }
    }
}
