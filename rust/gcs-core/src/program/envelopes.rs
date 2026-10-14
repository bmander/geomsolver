//! Bind envelopes in space: what the solve reads of each one's source, before any relation is
//! stated (`held_envelopes`, #283), and a revolution's named side once the solids are built
//! (`envelopes`).
use super::{resolve::{follow_building,Resolver},Code,Diag,Made,SourceMap};
use crate::{constraints::{Arg as CArg,CKind,Constraint,SpecKind},
    ir::{Decl,Kid,Operation,Statement},model::{EntKind,EntRef,EnvelopeE,Held,SetKind,SetShape,Sketch},
    syntax::{Arg,StmtId}};
use std::collections::{BTreeMap,BTreeSet};

/// The roll an envelope's declaration states, in radians.
fn roll(sk: &Sketch, d: &Decl) -> Result<[f64;2],String> {
    let angle = |a: &Arg| super::bound_angle(sk,a,"envelope bound").map(f64::to_radians);
    let e = d.angular_span.as_ref().ok_or("an envelope needs angular bounds")?;
    let roll = [angle(&e.from)?,angle(&e.to)?];
    if roll[0] >= roll[1] { return Err("an envelope needs increasing roll bounds".into()); }
    Ok(roll)
}

/// Each envelope in space, built with the drawing — after the motions, before any relation, so
/// a contact may name it (#283): its motion, its roll, and what the solve reads its source as —
/// a set's surface (its operand the hidden point the flattener applied the set to,
/// `{envelope}#probe`, whose shape the set's body states there: `set_shape`), or a revolution's
/// named side, read off the declarations since the solids are not built yet (its axis and its
/// edge).  An envelope whose source the solve cannot read is built all the same, saying why
/// (`Held::Not`), so it is evaluated after the solve as before and only a contact naming it is
/// refused.  Returns the statements built: `envelopes` binds their surfaces.
pub(super) fn held_envelopes(sk: &mut Sketch, res: &mut Resolver, map: &mut SourceMap,
    body: &[&Statement], skip: &BTreeSet<StmtId>) -> BTreeSet<StmtId> {
    let decls: BTreeMap<&str,&Decl> = body.iter().filter_map(|st| match &st.kind {
        Operation::Decl(d) => Some((d.name.key().text.as_str(),&**d)),
        _ => None,
    }).collect();
    let mut made = BTreeSet::new();
    for st in body {
        let Operation::Decl(d) = &st.kind else { continue };
        if d.kind != EntKind::Envelope || skip.contains(&st.id) { continue; }
        let operand = |i: usize| -> Option<&crate::syntax::Ref> {
            match d.children.get(i)?.as_slice() {
                [Kid::Ref(r)] => Some(r),
                _ => None,
            }
        };
        let motion = operand(1).and_then(|r| res.lookup(r))
            .filter(|e| e.kind == EntKind::Motion && e.i() < sk.motions.len());
        // a declaration that would not be built after the solve either is left to say so there —
        // its name, until then, naming nothing a relation could hold to
        let (Some(motion),Some(source),Ok(roll)) = (motion,operand(0),roll(sk,d)) else {
            res.of.remove(&d.name.key().text);
            continue;
        };
        let held = match res.lookup(source) {
            // a set applied to its hidden point: the point held, as a region's probe is
            Some(p) if p.kind == EntKind::Point && crate::flatten::is_probe(&source.root.text) => {
                for q in sk.point_all_params(p.i()) {
                    sk.params[q as usize].fixed = true;
                }
                Held::Set {probe:p.idx,shape:None}
            }
            _ => revolution(sk,res,&decls,source).unwrap_or_else(Held::Not),
        };
        let name = d.name.key().text.clone();
        let e = EntRef::new(EntKind::Envelope,sk.envelopes.len());
        sk.envelopes.push(EnvelopeE {surface:None,motion:motion.idx,roll,name:name.clone(),
            class:d.class.clone(),held});
        res.of.insert(name.clone(),e);
        map.bind(&name,e,d.name.named());
        map.record(st,Made::Ent(e));
        made.insert(st.id);
    }
    made
}

/// A revolution's named side as the solve reads it, off the declarations: `surface(rev, e)` over
/// `rev := solid(face, about: ax)` is `e` turned about `ax`.  Why not, where it is not one.
fn revolution(sk: &Sketch, res: &Resolver, decls: &BTreeMap<&str,&Decl>, source: &crate::syntax::Ref)
    -> Result<Held,String> {
    let not = "the solve reads the envelope of a cone or a cylinder set, or of a named side of a \
               revolution (`surface(rev, e)`)";
    let sd = decls.get(source.root.text.as_str()).filter(|x| x.kind == EntKind::Surface && source.path.is_empty())
        .ok_or(not)?;
    let (Some([Kid::Ref(solid)]),Some([Kid::Ref(edge)])) =
        (sd.children.first().map(Vec::as_slice),sd.children.get(1).map(Vec::as_slice)) else { return Err(not.into()) };
    let rev = decls.get(solid.root.text.as_str()).filter(|x| x.kind == EntKind::Solid && solid.path.is_empty())
        .ok_or(not)?;
    let Some(crate::syntax::Sweep::Revolve {axis,..}) = &rev.sweep else {
        return Err("the solve reads a named side of a revolution, not of another solid".into());
    };
    let read = |r: &crate::syntax::Ref| -> Result<EntRef,String> {
        let e = res.lookup(r).ok_or_else(|| format!("no such entity: `{}`",r.root.text))?;
        let e = follow_building(sk,res,e,r)?;
        if e.i() >= sk.count(e.kind) { return Err(format!("`{}` could not be built",r.root.text)); }
        Ok(e)
    };
    let (axis,edge) = (read(axis)?,read(edge)?);
    if !matches!(axis.kind,EntKind::Line | EntKind::Axis) {
        return Err("a revolution turns about a line or an axis".into());
    }
    if !matches!(edge.kind,EntKind::Line | EntKind::Arc | EntKind::Circle) {
        return Err("the solve reads a revolution's side turned from a line, an arc or a circle".into());
    }
    Ok(Held::Revolution {axis,edge})
}

/// The envelope whose hidden point a constraint reads, where it reads one: an operand, or a
/// line's end — what a set's body states at its envelope's point (#283).
pub(super) fn probe_envelope(sk: &Sketch, c: &Constraint) -> Option<usize> {
    let probes: Vec<(usize,usize)> = sk.envelopes.iter().enumerate().filter_map(|(i,e)| match e.held {
        Held::Set {probe,..} => Some((probe as usize,i)),
        _ => None,
    }).collect();
    if probes.is_empty() { return None; }
    super::regions::points_read(sk,c)
        .find_map(|p| probes.iter().find(|(q,_)| *q == p).map(|&(_,i)| i))
}

/// What a set's body states at envelope `e`'s point, read as the surface it is (#283) — by the
/// constraint's kind, as a region's terms are: the angle from a line seen from its start a cone,
/// a distance from a line a cylinder.  Never a row.
pub(super) fn set_shape(sk: &mut Sketch, c: &Constraint, e: usize) -> Result<(),String> {
    let Held::Set {probe,shape:None} = sk.envelopes[e].held else {
        return Err("an envelope's set states one surface at its point".into());
    };
    if c.bound.is_some() {
        return Err("an envelope's set is a surface its point is on, not a side of one".into());
    }
    let kind = crate::syntax::snake(c.kind.name());
    let ent = |i: usize| c.args[i].ent();
    // a direction its axis does not run along, where the place's turn round it starts
    let reference = |sk: &Sketch, axis: usize| {
        let l = &sk.lines[axis];
        let d = crate::space::sub(sk.world_point(l.p2 as usize),sk.world_point(l.p1 as usize));
        let k = (0..3).min_by(|&i,&j| d[i].abs().total_cmp(&d[j].abs())).unwrap_or(0);
        std::array::from_fn(|i| if i == k { 1.0 } else { 0.0 })
    };
    let number = |a: &CArg, s: SpecKind| super::regions::term_number(a,sk,s).ok_or_else(|| {
        let text = match a { CArg::Expr(e) => format!(" — `{}` is neither",e.text), _ => String::new() };
        format!("an envelope's set's number is stated, or a number times one unknown of the drawing \
                 and a number{text}")
    });
    let (kind,axis,spec) = match c.kind {
        CKind::Angle3 => {
            if ent(0).kind != EntKind::Line || ent(1).kind != EntKind::Line {
                return Err("a cone an envelope reads is about a line, seen from its start".into());
            }
            (SetKind::Cone,super::regions::cone_axis(sk,c,probe as usize)?,SpecKind::Angle)
        }
        CKind::PointLine3 => (SetKind::Cylinder,ent(1),SpecKind::Length),
        _ => return Err(format!("the solve reads the envelope of a cone's or a cylinder's surface, and \
                                 this set states `{kind}` at its point")),
    };
    let shape = SetShape {kind,axis:axis.idx,number:number(&c.args[2],spec)?,stood:c.args[2].num(),
        reference:reference(sk,axis.i())};
    // an unknown the number reads is a column of every contact with the envelope, so it is one
    // now, whether or not a row of the drawing reads it too (`expr::evaluate` keeps it bound)
    if let Some((name,seed)) = shape.unknown(&sk.declared) {
        crate::expr::bind_free(sk,&name,seed);
    }
    sk.envelopes[e].held = Held::Set {probe,shape:Some(shape)};
    Ok(())
}

/// Bind each envelope's revolution's named side once the solids and their surfaces are built:
/// the side of an envelope `held_envelopes` built (`held`), or a new envelope evaluated after the
/// solve, as every envelope in space was before the solve could see one.
pub(super) fn envelopes(sk: &mut Sketch, res: &mut Resolver, map: &mut SourceMap,
    body: &[&Statement], skip: &BTreeSet<StmtId>, held: &BTreeSet<StmtId>, diags: &mut Vec<Diag>) {
    for st in body {
        let Operation::Decl(d) = &st.kind else { continue; };
        if d.kind != EntKind::Envelope || skip.contains(&st.id) { continue; }
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
        let name = d.name.key().text.clone();
        if held.contains(&st.id) {
            let Some(e) = res.of.get(&name).copied().filter(|e| e.kind == EntKind::Envelope) else { continue };
            // a set's envelope has no side to bind
            if matches!(sk.envelopes[e.i()].held,Held::Set {..}) { continue; }
            match operand(0,EntKind::Surface) {
                Ok(s) => sk.envelopes[e.i()].surface = Some(s),
                Err(message) => diags.push(Diag::on(Code::E080, st, message)),
            }
            continue;
        }
        let build = || -> Result<EnvelopeE,String> {
            Ok(EnvelopeE {surface:Some(operand(0,EntKind::Surface)?),motion:operand(1,EntKind::Motion)?,
                roll:roll(sk,d)?,name:name.clone(),class:d.class.clone(),
                held:Held::Not("its source and motion were not read before the relations".into())})
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
                diags.push(Diag::on(Code::E080, st, message));
                res.of.remove(&name);
            }
        }
    }
}
