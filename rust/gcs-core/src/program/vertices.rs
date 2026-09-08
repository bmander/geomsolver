//! Spatial corners depend on named seams and add no planar solver coordinates.
use super::{resolve::{follow_building,Resolver},Code,Diag,Made,SourceMap};
use crate::{ir::{Kid,Operation,Statement},model::{EntKind,EntRef,Sketch,VertexE},syntax::StmtId};
use std::collections::BTreeSet;

pub(super) fn vertices(sk: &mut Sketch,res: &mut Resolver,map: &mut SourceMap,
    body: &[&Statement],skip: &BTreeSet<StmtId>,diags: &mut Vec<Diag>) {
    for st in body {
        let Operation::Decl(d) = &st.kind else { continue; };
        if d.kind != EntKind::Vertex || skip.contains(&st.id) { continue; }
        let build = || -> Result<VertexE,String> {
            let operand = |i| -> Result<u32,String> {
                let [Kid::Ref(r)] = d.children.get(i).map(Vec::as_slice).unwrap_or_default() else {
                    return Err("a vertex names two existing seams".into());
                };
                let e = res.lookup(r).ok_or_else(|| format!("no such entity: `{}`",r.root.text))?;
                let e = follow_building(sk,res,e,r)?;
                if e.kind != EntKind::Seam || e.i() >= sk.seams.len() {
                    return Err("a vertex needs a valid seam".into());
                }
                Ok(e.idx)
            };
            let first = operand(0)?; let second = operand(1)?;
            crate::vertex::validate(sk,[first,second])?;
            Ok(VertexE {first,second,name:d.name.key().text.clone(),class:d.class.clone()})
        };
        match build() {
            Ok(v) => {
                let e = EntRef::new(EntKind::Vertex,sk.vertices.len());
                res.of.insert(v.name.clone(),e);
                map.bind(&v.name,e,d.name.named()); map.record(st,Made::Ent(e));
                sk.vertices.push(v);
            }
            Err(message) => {
                diags.push(Diag {code:Code::E080,span:st.span,stmt:Some(st.id),message});
                res.of.remove(&d.name.key().text);
            }
        }
    }
}
