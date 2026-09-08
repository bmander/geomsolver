//! Finite spatial edge declarations depend on seams, vertices and a slicing direction.
use super::{resolve::{follow_building,Resolver},Code,Diag,Made,SourceMap};
use crate::{ir::{Kid,Operation,Statement},model::{EdgeE,EntKind,EntRef,Sketch},syntax::StmtId};
use std::collections::BTreeSet;

pub(super) fn edges(sk: &mut Sketch,res: &mut Resolver,map: &mut SourceMap,
    body: &[&Statement],skip: &BTreeSet<StmtId>,diags: &mut Vec<Diag>) {
    for st in body {
        let Operation::Decl(d) = &st.kind else { continue; };
        if d.kind != EntKind::Edge || skip.contains(&st.id) { continue; }
        let build = || -> Result<EdgeE,String> {
            let operand = |i,kind| -> Result<u32,String> {
                let [Kid::Ref(r)] = d.children.get(i).map(Vec::as_slice).unwrap_or_default() else {
                    return Err("an edge names a seam, two vertices and an along line".into());
                };
                let e = res.lookup(r).ok_or_else(|| format!("no such entity: `{}`",r.root.text))?;
                let e = follow_building(sk,res,e,r)?;
                if e.kind != kind || e.i() >= sk.count(kind) {
                    return Err(format!("edge operand {} needs a valid {}",i+1,kind.as_str()));
                }
                Ok(e.idx)
            };
            let edge = EdgeE {seam:operand(0,EntKind::Seam)?,start:operand(1,EntKind::Vertex)?,
                end:operand(2,EntKind::Vertex)?,along:operand(3,EntKind::Line)?,
                name:d.name.key().text.clone(),class:d.class.clone()};
            crate::edge::validate(sk,&edge)?;
            Ok(edge)
        };
        match build() {
            Ok(edge) => {
                let e = EntRef::new(EntKind::Edge,sk.edges.len());
                res.of.insert(edge.name.clone(),e);
                map.bind(&edge.name,e,d.name.named()); map.record(st,Made::Ent(e));
                sk.edges.push(edge);
            }
            Err(message) => {
                diags.push(Diag {code:Code::E080,span:st.span,stmt:Some(st.id),message});
                res.of.remove(&d.name.key().text);
            }
        }
    }
}
