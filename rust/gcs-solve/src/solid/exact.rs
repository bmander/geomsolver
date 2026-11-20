//! A static solid read through this kernel's B-rep (rung 3 of docs/rust-kernel-plan.md): its CAD
//! recipe built node by node, every face named by the path the faceted kernel gives it —
//! `bore.wall`, a placed copy's `placed.bore.wall` — so a body's surviving faces, its provenance
//! and its round features read the same whichever kernel evaluated it. A boolean never renames:
//! a face keeps the name of the operand face it is a piece of.
use super::*;
use crate::brep::geom;
use crate::brep::topo::Brep;
use crate::brep::recipe::{name_under, named_parts, Recipe};
use std::collections::BTreeSet;

/// A static solid's exact B-rep, in millimetres (`mm` of them a model unit, `cad::millimetres`)
/// about `origin` (model units, the point the facet term is evaluated about too), each face named
/// by its document path.
#[derive(Clone, Debug)]
pub struct Exact {
    pub brep: Brep,
    pub mm: f64,
    pub origin: [f64; 3],
    /// The faces whose facets lead, as the facet term's do — a prism's caps and a sweep's — so an
    /// edge between two faces, named by the face that leads, is named as it was.
    pub leading: BTreeSet<String>,
    /// Each face's name in its two halves, beside `brep.faces`: the primitive it is a piece of,
    /// keyed as `document::operand_primitives` keys it, and the face's own name in it (#188);
    /// empty for a swept solid supplied whole.
    pub named: Vec<(String, String)>,
}

impl Exact {
    /// Its B-rep moved by `by`, each face named in its two halves again (`NAMED`, `named`'s
    /// inverse): what a build that names faces by their halves takes as an operand built already
    /// (#760: a shared leaf's joined names read as no primitive's, and a round over them as none).
    pub(crate) fn halved(&self, by: &geom::Rigid) -> Brep {
        let mut brep = self.brep.moved(by);
        for (f, (of, face)) in brep.faces.iter_mut().zip(&self.named) {
            f.name = name_under(of, face);
        }
        brep
    }

    /// The same B-rep under names `re` rewrites (#566): its faces', their halves' and the leading
    /// ones'.
    pub(crate) fn renamed(&self, re: &super::evaluated::Renaming<'_>) -> Exact {
        self.with(self.brep.clone(), self.origin, re)
    }

    /// **Another instance's solid of the same binding** (#566): this B-rep moved by `motion` and
    /// renamed from the instance `from` to `to`, about its origin moved — nothing built again.
    /// A motion that reflects (a mirrored instance's, #578) reflects the B-rep (`Brep::reflected`),
    /// or says why it cannot.
    pub(crate) fn placed(&self, motion: &Rigid, from: &str, to: &str) -> Result<Exact, String> {
        // the B-rep is about its origin, in millimetres: the origin takes the motion's
        // translation, and the B-rep its turn alone
        let turn = geom::Rigid { r: motion.r, t: [0.0; 3] };
        let brep = match motion.improper() {
            true => self.brep.reflected(&turn)?,
            false => self.brep.moved(&turn),
        };
        Ok(self.with(brep, motion.point(self.origin), &super::evaluated::Renaming { from, to }))
    }

    /// `brep` about `origin`, under this one's names rewritten by `re`.
    fn with(&self, mut brep: Brep, origin: [f64; 3], re: &super::evaluated::Renaming<'_>) -> Exact {
        for f in &mut brep.faces {
            f.name = re.name(&f.name);
        }
        Exact {
            brep,
            mm: self.mm,
            origin,
            leading: self.leading.iter().map(|f| re.name(f)).collect(),
            named: self.named.iter().map(|(of, face)| (re.name(of), face.clone())).collect(),
        }
    }
}

/// Solid `si`'s exact B-rep, or why this kernel does not build it (no length unit, a sweep, a
/// recipe node it refuses) — a caller then evaluates the facet term instead.
pub(crate) fn build(sk: &Sketch, si: usize) -> Result<Exact, String> {
    let mm = cad::millimetres(sk);
    let origin = frame_origin(sk, si, REPORT_UNIT);
    let recipe = cad::recipe_about(sk, si, origin.map(|x| x * mm))?;
    let (mut built, leading) = nodes(sk, si, &recipe, origin, None)?;
    let brep = built.remove(&recipe.root).ok_or("recipe: no root")?;
    Ok(named(brep, mm, origin, leading))
}

/// **Solid `si`'s written terms, built where it is** (#961): the nodes its root body's stock,
/// unions, cuts and commons need — what reads its faces (a round's fill and cut) left unbuilt —
/// each as `build` builds it about `si`'s origin (millimetres), by the node's id: what a reading
/// of the body's terms against one another classifies, with no Boolean of the body made.  Where
/// a needed node reads others no walk here lists, every node but the root is built.
pub(crate) fn operands(sk: &Sketch, si: usize) -> Result<(Recipe, BTreeMap<i64, Brep>), String> {
    use crate::brep::recipe::Node;
    let mm = cad::millimetres(sk);
    let origin = frame_origin(sk, si, REPORT_UNIT);
    let recipe = cad::recipe_of_terms(sk, si, origin.map(|x| x * mm))?;
    let of: BTreeMap<i64, &Node> = recipe.nodes.iter().map(|n| (n.id, &n.node)).collect();
    let mut wanted: Option<BTreeSet<i64>> = match of.get(&recipe.root) {
        Some(Node::Body(b)) => Some(std::iter::once(b.stock)
            .chain(b.on.iter().chain(&b.cut).chain(&b.bound).copied()).collect()),
        _ => None,
    };
    // every node those read, all the way down
    let mut pending: Vec<i64> = wanted.iter().flatten().copied().collect();
    while let (Some(id), Some(w)) = (pending.pop(), wanted.as_mut()) {
        // a body is built as its own (`nodes`), reading nothing here
        match of.get(&id).filter(|n| !matches!(n, Node::Body(_))).map(|n| reads(n)) {
            Some(Some(more)) => pending.extend(more.into_iter().filter(|m| w.insert(*m))),
            Some(None) => wanted = None,
            None => {}
        }
    }
    let wanted =
        wanted.unwrap_or_else(|| of.keys().copied().filter(|&id| id != recipe.root).collect());
    let (built, _) = nodes(sk, si, &recipe, origin, Some(&wanted))?;
    Ok((recipe, built))
}

/// The nodes node `n` reads as it is built, where this walk knows them: none for one of a
/// profile, points or a family, its sources for a placement, a half-space or a prism through
/// everything, its terms for a body; `None` for a lowered region, whose pieces read the body.
fn reads(n: &crate::brep::recipe::Node) -> Option<Vec<i64>> {
    use crate::brep::recipe::Node;
    let all = |pieces: &mut dyn Iterator<Item = &Node>| -> Option<Vec<i64>> {
        let mut out = Vec::new();
        for p in pieces {
            out.extend(reads(p)?);
        }
        Some(out)
    };
    Some(match n {
        Node::Prism { .. } | Node::Revolve { .. } | Node::Hull { .. } | Node::Sewn { .. }
        | Node::Canal { .. } | Node::Family(_) | Node::Carried { .. } => Vec::new(),
        Node::Placed { source, .. } => vec![*source],
        Node::Through { uses, .. } => uses.iter().flat_map(|u| u.sources.iter().copied()).collect(),
        Node::Half { sources, .. } => sources.clone(),
        Node::Body(b) => std::iter::once(&b.stock).chain(b.sets().into_iter().flat_map(|(_, l)| l))
            .copied().collect(),
        Node::Grown { body, pieces } => {
            let mut out = all(&mut pieces.iter())?;
            out.push(*body);
            out
        }
        Node::Region(kept) => all(&mut kept.iter().map(|k| &k.node))?,
        Node::Glued(pieces) => all(&mut pieces.iter())?,
        Node::Lowered(_) => return None,
    })
}

/// `recipe`'s nodes built about `origin` (model units) for solid `si` — those of `only` where it
/// says (the root then not among them), else every one — and the faces whose facets lead.
fn nodes(sk: &Sketch, si: usize, recipe: &Recipe, origin: [f64; 3],
         only: Option<&BTreeSet<i64>>)
    -> Result<(BTreeMap<i64, Brep>, BTreeSet<String>), String> {
    let mm = cad::millimetres(sk);
    // the rounding of coordinates read where the solid stands, before the shift: a few units in
    // the last place of the largest
    let floor = 16.0 * f64::EPSILON * origin.iter().fold(0.0f64, |m, x| m.max(x.abs())) * mm;
    let mut built = BTreeMap::new();
    let mut leading = BTreeSet::new();
    // each node as what a body built of it says calls it
    let names: BTreeMap<i64, String> = recipe.nodes.iter().map(|n| (n.id, n.called())).collect();
    for n in &recipe.nodes {
        let id = n.id;
        if only.is_some_and(|w| !w.contains(&id) || id == recipe.root) {
            continue;
        }
        // an operand of one binding with others is its representative's B-rep, built once and
        // placed (#566), moved into this one's place; its faces are named already.  One of this
        // solid's own instance is read where this one is built, its preimage (#578); a mirrored
        // one of another instance is read as it is published, reflected
        let bound = |i: usize| sk.solids[i].binding.as_ref();
        let own = bound(id as usize).zip(bound(si)).is_some_and(|(a, b)| a.at == b.at);
        let mirrored = bound(id as usize).is_some_and(|b| b.mirror.is_some());
        // and where only some are asked for (`operands`), a body among them is its own build,
        // remembered (`Sketch::exact_solid`): what reads a term of a body reads it once
        let body = only.is_some() && matches!(n.node, crate::brep::recipe::Node::Body(_));
        let leaf = if id as usize == si {
            None
        } else if own {
            (sk.shared(id as usize) || body).then(|| sk.exact_preimage(id as usize))
        } else {
            (sk.shared(id as usize) || mirrored || body).then(|| sk.exact_solid(id as usize))
        };
        if let Some(Ok(x)) = leaf {
            let t = std::array::from_fn(|k| (x.origin[k] - origin[k]) * mm);
            built.insert(id, x.halved(&geom::Rigid { t, ..geom::Rigid::identity() }));
            leading.extend(x.leading.iter().cloned());
            continue;
        }
        let mut b = n.node.built(&built, &names, floor)?;
        let sol = &sk.solids[id as usize];
        match &sol.def {
            SolidDef::Placed { source, .. } => {
                let paths = operand_paths(sk, *source as usize);
                for f in &mut b.faces {
                    let (of, face) = named_parts(&f.name);
                    let path = paths.get(of).map(String::as_str).unwrap_or(of);
                    let placed = super::document::placed_name(&sol.name, path);
                    if leading.contains(&format!("{of}.{face}")) {
                        leading.insert(format!("{placed}.{face}"));
                    }
                    f.name = name_under(&placed, face);
                }
            }
            // a body's faces are its operands', named as they were — where its node is a body's (a
            // shell's cavity is one construction standing for the solids it is made of, `shelled`)
            SolidDef::Body { .. } if matches!(n.node, crate::brep::recipe::Node::Body(_)) => {}
            _ => {
                // a turn's ends do not lead
                let caps = if matches!(sol.def, SolidDef::Revolve { .. }) { &[][..] } else { sol.caps() };
                for f in &mut b.faces {
                    if caps.contains(&f.name.as_str()) {
                        leading.insert(format!("{}.{}", sol.name, f.name));
                    }
                    f.name = name_under(&sol.name, &f.name);
                }
            }
        }
        built.insert(id, b);
    }
    Ok((built, leading))
}

/// A B-rep whose faces a build named in two halves (`brep::recipe::NAMED`: a recipe's, a swept body's), as an
/// `Exact`: each face's halves in `named`, and its name the two joined by a dot.
pub(crate) fn named(mut brep: Brep, mm: f64, origin: [f64; 3], leading: BTreeSet<String>) -> Exact {
    let mut named = Vec::with_capacity(brep.faces.len());
    for f in &mut brep.faces {
        let (of, face) = named_parts(&f.name);
        named.push((of.to_string(), face.to_string()));
        f.name = joined(&f.name);
    }
    Exact { brep, mm, origin, leading, named }
}

/// A face's name as a build named it in two halves, joined by a dot: the kernel's
/// (`brep::recipe::joined`).
pub(crate) use crate::brep::recipe::joined;

/// Static solid `si`'s B-rep as an export built it where it stands (millimetres,
/// `brep::export::exact`), as an `Exact` about the solid's origin, as `build` stands one (#592).
pub(crate) fn about_origin(sk: &Sketch, si: usize, world: &Brep) -> Exact {
    let mm = cad::millimetres(sk);
    let origin = frame_origin(sk, si, REPORT_UNIT);
    let t = origin.map(|x| -x * mm);
    named(world.moved(&geom::Rigid { t, ..geom::Rigid::identity() }), mm, origin, BTreeSet::new())
}

/// Swept body `si`'s exact B-rep as this kernel builds it (`brep::export::exact`, at the gross
/// bars), about the origin, each face named as `build` names a static solid's (#212): the blank's
/// by its primitive, a cut's by the placement that cuts it and the cutter's face that made it.
pub(crate) fn swept(sk: &Sketch, si: usize) -> Result<Exact, String> {
    let mm = cad::millimetres(sk);
    if let Some(x) = sk.supplied_exact_solid(si).filter(|x| !x.named.is_empty()) {
        return Ok((*x).clone());
    }
    let say = crate::brep::sweep::Say { stage: &|_: &str| {}, mark: &|_| {} };
    let built = crate::brep::export::exact(sk, si, None, None, &say).map_err(|r| r.message)?;
    Ok(named(built.solid, mm, [0.0; 3], BTreeSet::new()))
}
