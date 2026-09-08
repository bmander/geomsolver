//! Spatial trims declare material sides, without adding any solver coordinates.
use super::{resolve::{follow_building,Resolver},Code,Diag,Made,SourceMap};
use crate::{ir::{Kid,Operation,Statement},model::{EntKind,EntRef,PatchE,Sketch,SolidDef},
    syntax::StmtId};
use std::collections::BTreeSet;

pub(super) fn patches(sk: &mut Sketch,res: &mut Resolver,map: &mut SourceMap,
    body: &[&Statement],skip: &BTreeSet<StmtId>,diags: &mut Vec<Diag>) {
    for st in body {
        let Operation::Decl(d) = &st.kind else { continue; };
        if d.kind != EntKind::Patch || skip.contains(&st.id) { continue; }
        let build = || -> Result<PatchE,String> {
            let resolve = |kid: &Kid| -> Result<EntRef,String> {
                let Kid::Ref(r) = kid else { return Err("a patch names existing geometry".into()); };
                let e = res.lookup(r).ok_or_else(|| format!("no such entity: `{}`",r.root.text))?;
                let e = follow_building(sk,res,e,r)?;
                if e.i() >= sk.count(e.kind) { return Err(format!("`{}` could not be built",r.root.text)); }
                Ok(e)
            };
            let [source] = d.children.first().map(Vec::as_slice).unwrap_or_default() else {
                return Err("a patch needs exactly one source surface or envelope".into());
            };
            let source = resolve(source)?;
            if !matches!(source.kind,EntKind::Surface | EntKind::Envelope) {
                return Err("a patch source must be a surface or envelope".into());
            }
            let group = |i: usize| -> Result<Vec<u32>,String> {
                d.children.get(i).into_iter().flatten().map(|k| {
                    let e = resolve(k)?;
                    if e.kind != EntKind::Solid { return Err("a patch trim names a solid".into()); }
                    let SolidDef::Revolve {ref sweep,..} = sk.solids[e.i()].def else {
                        return Err("analytic trims currently require an unmodified full revolution".into());
                    };
                    if !sweep.value.is_finite() || sweep.value < std::f64::consts::TAU {
                        return Err("analytic trims currently require a full revolution".into());
                    }
                    Ok(e.idx)
                }).collect()
            };
            let inside = group(1)?; let outside = group(2)?;
            if inside.is_empty() && outside.is_empty() {
                return Err("a patch needs at least one inside or outside trim".into());
            }
            Ok(PatchE {source,inside,outside,name:d.name.key().text.clone(),class:d.class.clone()})
        };
        match build() {
            Ok(p) => {
                let e = EntRef::new(EntKind::Patch,sk.patches.len());
                res.of.insert(p.name.clone(),e);
                map.bind(&p.name,e,d.name.named()); map.record(st,Made::Ent(e));
                sk.patches.push(p);
            }
            Err(message) => {
                diags.push(Diag {code:Code::E080,span:st.span,stmt:Some(st.id),message});
                res.of.remove(&d.name.key().text);
            }
        }
    }
}
