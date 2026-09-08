//! Name a generating junction, an envelope/boundary seam, or two-surface intersection.
use super::{resolve::{follow_building,Resolver},Code,Diag,Made,SourceMap};
use crate::{ir::{Kid,Operation,Statement},model::{EntKind,EntRef,SeamE,Sketch},syntax::StmtId};
use std::collections::BTreeSet;

pub(super) fn seams(sk: &mut Sketch,res: &mut Resolver,map: &mut SourceMap,
    body: &[&Statement],skip: &BTreeSet<StmtId>,diags: &mut Vec<Diag>) {
    for st in body {
        let Operation::Decl(d) = &st.kind else { continue; };
        if d.kind != EntKind::Seam || skip.contains(&st.id) { continue; }
        let build = || -> Result<SeamE,String> {
            let operand = |i| -> Result<EntRef,String> {
                let [Kid::Ref(r)] = d.children.get(i).map(Vec::as_slice).unwrap_or_default() else {
                    return Err("a seam names two generating faces, a generating face and surface, or two surfaces".into());
                };
                let e = res.lookup(r).ok_or_else(|| format!("no such entity: `{}`",r.root.text))?;
                let e = follow_building(sk,res,e,r)?;
                if e.i() >= sk.count(e.kind) { return Err(format!("`{}` could not be built",r.root.text)); }
                Ok(e)
            };
            let first = operand(0)?; let second = operand(1)?;
            crate::seam::validate(sk,[first,second])?;
            Ok(SeamE {first,second,name:d.name.key().text.clone(),class:d.class.clone()})
        };
        match build() {
            Ok(s) => {
                let e = EntRef::new(EntKind::Seam,sk.seams.len());
                res.of.insert(s.name.clone(),e);
                map.bind(&s.name,e,d.name.named()); map.record(st,Made::Ent(e));
                sk.seams.push(s);
            }
            Err(message) => {
                diags.push(Diag {code:Code::E080,span:st.span,stmt:Some(st.id),message});
                res.of.remove(&d.name.key().text);
            }
        }
    }
}
