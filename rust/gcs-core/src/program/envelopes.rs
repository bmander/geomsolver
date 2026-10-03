//! Bind implicit envelopes after their source surfaces and motions exist.
use super::{resolve::{follow_building,Resolver},Code,Diag,Made,SourceMap};
use crate::{ir::{Kid,Operation,Statement},model::{EntKind,EntRef,EnvelopeE,Sketch},
    syntax::{Arg,StmtId}};
use std::collections::BTreeSet;

pub(super) fn envelopes(sk: &mut Sketch, res: &mut Resolver, map: &mut SourceMap,
    body: &[&Statement], skip: &BTreeSet<StmtId>, diags: &mut Vec<Diag>) {
    for st in body {
        let Operation::Decl(d) = &st.kind else { continue; };
        if d.kind != EntKind::Envelope || skip.contains(&st.id) { continue; }
        let build = || -> Result<EnvelopeE,String> {
            let operand = |i: usize, kind: EntKind| -> Result<u32,String> {
                let group = d.children.get(i).ok_or("an envelope needs a surface and motion")?;
                let [Kid::Ref(r)] = group.as_slice() else {
                    return Err("an envelope names a surface and motion".into());
                };
                let e = res.lookup(r).ok_or_else(|| format!("no such entity: `{}`",r.root.text))?;
                let e = follow_building(sk,res,e,r)?;
                if e.kind != kind || e.i() >= sk.count(kind) {
                    return Err(format!("an envelope needs a valid {}",kind.as_str()));
                }
                Ok(e.idx)
            };
            let angle = |a: &Arg| super::bound_angle(sk,a,"envelope bound").map(f64::to_radians);
            let e = d.angular_span.as_ref().ok_or("an envelope needs angular bounds")?;
            let roll = [angle(&e.from)?,angle(&e.to)?];
            if roll[0] >= roll[1] { return Err("an envelope needs increasing roll bounds".into()); }
            Ok(EnvelopeE {surface:operand(0,EntKind::Surface)?,motion:operand(1,EntKind::Motion)?,
                roll,name:d.name.key().text.clone(),class:d.class.clone()})
        };
        match build() {
            Ok(value) => {
                let e = EntRef::new(EntKind::Envelope,sk.envelopes.len());
                res.of.insert(value.name.clone(),e);
                map.bind(&value.name,e,d.name.named());
                map.record(st,Made::Ent(e));
                sk.envelopes.push(value);
            }
            Err(message) => {
                diags.push(Diag {code:Code::E080,span:st.span,stmt:Some(st.id),message});
                res.of.remove(&d.name.key().text);
            }
        }
    }
}
