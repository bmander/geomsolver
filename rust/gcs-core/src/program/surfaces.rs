//! Bind analytic surfaces to existing solid boundaries. No new coordinates or solver rows.
use super::{resolve::{follow_building,Resolver},Code,Diag,Made,SourceMap};
use crate::{ir::{Kid,Operation,Statement},model::{EntKind,EntRef,Sketch,SolidDef,SurfaceE},
    syntax::{Arg,StmtId}};
use std::collections::BTreeSet;

pub(super) fn surfaces(sk: &mut Sketch, res: &mut Resolver, map: &mut SourceMap,
    body: &[&Statement], skip: &BTreeSet<StmtId>, diags: &mut Vec<Diag>) {
    for st in body {
        let Operation::Decl(d) = &st.kind else { continue };
        if d.kind != EntKind::Surface || skip.contains(&st.id) { continue; }
        let build = || -> Result<(usize,EntRef,Option<[f64;2]>),String> {
            let operand = |i: usize| -> Result<EntRef,String> {
                let group = d.children.get(i).ok_or("a surface needs a solid and its profile edge")?;
                let [Kid::Ref(r)] = group.as_slice() else {
                    return Err("a surface names a solid and its profile edge".into());
                };
                let e = res.lookup(r)
                    .ok_or_else(|| format!("no such entity: `{}`",r.root.text))?;
                let e = follow_building(sk,res,e,r)?;
                if e.i() >= sk.count(e.kind) { return Err(format!("`{}` could not be built",r.root.text)); }
                Ok(e)
            };
            let solid = operand(0)?;
            let edge = operand(1)?;
            if solid.kind != EntKind::Solid {
                return Err("a surface's first argument is a solid".into());
            }
            let SolidDef::Revolve {face,ref sweep,..} = sk.solids[solid.i()].def else {
                return Err("analytic surface references currently require an unmodified revolution".into());
            };
            if !sk.faces[face as usize].boundaries().any(|(edges,_)| edges.contains(&edge)) {
                return Err("the edge is not a boundary of this solid's profile".into());
            }
            if !matches!(edge.kind,EntKind::Line | EntKind::Arc | EntKind::Circle) {
                return Err("an exact revolved surface requires a line, arc or circle".into());
            }
            let span = d.angular_span.as_ref().map(|s| -> Result<[f64;2],String> {
                let angle = |a: &Arg| super::bound_angle(sk,a,"surface bound").map(f64::to_radians);
                let bounds = [angle(&s.from)?,angle(&s.to)?];
                if !bounds.iter().all(|x| x.is_finite()) || bounds[0] < 0. || bounds[0] >= bounds[1]
                    || bounds[1] > sweep.value.min(std::f64::consts::TAU) {
                    return Err("a surface needs increasing angular bounds within its source sweep".into());
                }
                Ok(bounds)
            }).transpose()?;
            Ok((solid.i(),edge,span))
        };
        let name = &d.name.key().text;
        match build() {
            Ok((solid,edge,span)) => {
                let e = EntRef::new(EntKind::Surface,sk.surfaces.len());
                sk.surfaces.push(SurfaceE {solid:solid as u32,edge,span,name:name.clone(),class:d.class.clone()});
                res.of.insert(name.clone(),e);
                map.bind(name,e,d.name.named());
                map.record(st,Made::Ent(e));
            }
            Err(message) => {
                diags.push(Diag {code:Code::E080,span:st.span,stmt:Some(st.id),message});
                res.of.remove(name);
            }
        }
    }
}
