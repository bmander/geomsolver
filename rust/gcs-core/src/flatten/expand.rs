//! Source-order statement expansion, repetition, seed settling and emission.

use super::*;
use super::apply::Pass;
use super::sets::Made;
use std::ops::ControlFlow;

/// A chain a call gives one of its `path` formals: the argument's index, the argument as
/// written, and each link as a reference the call's scope reads, with its absolute name.
type ChainArgument = (usize, Ref, Vec<(Ref, String)>);

impl<'a> Walk<'a> {
    /// The view an enclosing instance is drawn `in`, put on one declaration its expansion
    /// makes (§6.7).  A curve is left alone — it is its expressions — and a declaration that
    /// already says which plane (a clause of its own, a block in the component) rides on the
    /// instance's frame as well (#383): `rewrite_decl` puts it on its path, where it resolves.
    /// A frame declared in the component takes the instance's as where it rides.
    pub(super) fn stamp_scope_plane(&mut self, d: &mut Decl, scope: &Scope) {
        d.annotations.roles = d.annotations.roles.union(scope.stamps.roles);
        // the instance's classes, over the declaration's own: what the assembly says of an
        // instance is the later and stronger word, so a phantom's centreline is a phantom's
        for c in &scope.stamps.class.0 {
            if !d.class.has(c) {
                d.class.0.push(c.clone());
            }
        }
        let Some(p) = &scope.stamps.plane else { return };
        if !d.kind.bears_points() && d.kind != crate::model::EntKind::Frame {
            return; // a curve is its expressions
        }
        d.membership.join(&p.plane, crate::syntax::Source::Instance);
    }

    /// Walk one body, in source order.
    pub(super) fn body(
        &mut self,
        body: &[Stmt],
        scope: &Scope,
        vals: &mut Vals,
        path: &[PathStep],
        depth: usize,
    ) {
        if depth > MAX_DEPTH {
            if let Some(st) = body.first() {
                self.err(Code::E103, st.span, format!("nested more than {MAX_DEPTH} deep"));
            }
            return;
        }
        let prefix = scope.prefix().to_string();
        for st in body {
            let StmtKind::Group(g) = &st.kind else { continue };
            let name = &g.name.text;
            let conflicts = body.iter().any(|other| match &other.kind {
                StmtKind::Decl(d) => &d.name.key().text == name,
                StmtKind::Param(p) => &p.name.text == name,
                _ => false,
            }) || vals.contains_key(name) || scope.groups.contains(name)
                || self.aliases.iter().any(|(key, _, _)| key == &format!("{prefix}{name}"));
            if conflicts {
                self.err(Code::E001, g.name.span, format!("`{name}` is declared twice"));
            }
        }
        // a named use's label is a name of the body like any other (§9.9, [0.149])
        for (k, st) in body.iter().enumerate() {
            let StmtKind::Relation(r) = &st.kind else { continue };
            let Some(label) = &r.name else { continue };
            let name = &label.text;
            let conflicts = body.iter().enumerate().any(|(j, other)| match &other.kind {
                StmtKind::Decl(d) => &d.name.key().text == name,
                StmtKind::Param(p) => &p.name.text == name,
                StmtKind::Group(g) => &g.name.text == name,
                StmtKind::Instance(i) => &i.name.text == name,
                StmtKind::Set(s) => &s.name.text == name,
                StmtKind::Relation(o) => j < k && o.name.as_ref().is_some_and(|n| &n.text == name),
                _ => false,
            }) || vals.contains_key(name);
            if conflicts {
                self.err(Code::E001, label.span, format!("`{name}` is declared twice"));
            }
        }
        for st in body {
            if let StmtKind::Instance(inst) = &st.kind {
                let name = format!("{prefix}{}", inst.name.text);
                self.names.insert(name.clone());
                if inst.annotations.private { self.private_names.insert(name.clone(), scope.owner.clone()); }
                self.group_names.insert(name);
            }
        }
        // the root's numbers are the file's: the params of every module the file `use`s come
        // first, so the file's own may read them under their modules' paths
        if depth == 0 {
            let uses: Vec<String> = self.prog.uses.iter().map(|u| u.name.clone()).collect();
            for (k, v) in self.used_params(&uses) {
                vals.or_insert(k, v);
            }
            // and those it imports bare, by their own names (§14.4 [0.48])
            for (k, v) in self.imported_params(None) {
                vals.or_insert(k, v);
            }
        }
        // every `param` of the body, whatever line it stands on — a body is a set (P2)
        self.params(body, vals, scope);
        // and every instance's numbers, by its name (`ball.r`), over the body's own
        self.instance_numbers(body, vals, scope);
        // Remember ambient numbers so accidental capture gets an explicit diagnostic.
        // and with them the numbers in force are complete for every statement of the body:
        // the enclosing ones the caller passed and the body's own — so that is the table each
        // statement is emitted with, which is what an index (`p[n - 1]`) is read against.  The
        // root's scope arrived with an empty one, and a top-level index could read a literal
        // and not a `param` (#45.2).
        let mine: Vec<String> = body.iter().filter_map(|st| match &st.kind {
            StmtKind::Group(g) => Some(g.name.text.clone()),
            _ => None,
        }).collect();
        let groups = if mine.is_empty() {
            scope.groups.clone()
        } else {
            std::rc::Rc::new(scope.groups.iter().cloned().chain(mine).collect())
        };
        let scope = &Scope { vals: std::rc::Rc::new(vals.clone()), groups, ..scope.clone() };
        if depth == 0 {
            self.file_vals = scope.vals.clone();
        }
        // what a frame with a value draws in its block, read where the value puts it (#398 B)
        let lowered = self.valued_blocks(body);
        let body: &[Stmt] = lowered.as_deref().unwrap_or(body);
        for st in body {
            // a set's body walked again for a tangency states its relations' derivatives and
            // makes nothing a second time, and one walked for what it makes states nothing (§6.21)
            let relation = matches!(st.kind, StmtKind::Relation(_));
            match scope.pred.pass {
                Pass::Along(_) if !relation => continue,
                Pass::Made
                    if relation
                        || matches!(st.kind, StmtKind::ClaimOver(_) | StmtKind::Minimize(_) | StmtKind::Measured(_)) =>
                {
                    continue
                }
                _ => {}
            }
            if self.emitted() >= MAX_FLAT {
                self.err(
                    Code::E103,
                    st.span,
                    format!("more than {MAX_FLAT} statements once expanded"),
                );
                return;
            }
            match &st.kind {
                StmtKind::Chain(chain) => self.walk_chain(chain, st, scope, path, &prefix),
                // a reading at an instant, every way it may be written out (#391 S2)
                StmtKind::Decl(d) if d.carried.is_some() => {
                    self.walk_reading_at(d, st, scope, vals, path, depth, &prefix)
                }
                StmtKind::Decl(d) => {
                    // a face family's station is drawn, hidden, before the solid it makes
                    if let Some(crate::syntax::Sweep::Family(f)) = &d.sweep {
                        if self.walk_family(f, st, scope, vals, path, depth).is_break() {
                            return;
                        }
                    }
                    self.walk_decl(d, st, scope, vals, path, &prefix)
                }
                StmtKind::Param(_) | StmtKind::Group(_) => {} // worked out above, before the walk
                StmtKind::Instance(inst) => {
                    if self.walk_instance(inst, st, scope, vals, path, depth).is_break() {
                        return;
                    }
                }
                StmtKind::Block(b) => self.walk_block(b, st, scope, vals, path, depth),
                // a constraint: its dimension is written in the component's own parameters, which
                // do not exist in the flat document, so they are worked out here
                // — and one written with a defined word is that word's body (§9.9)
                StmtKind::Relation(rel) => self.walk_relation(rel, st, scope, vals, path, depth),
                // a set written in place: nothing drawn, its uses expanded once every name is
                // known (§6.21)
                StmtKind::Set(set) => {
                    let abs = format!("{prefix}{}", set.name.text);
                    self.names.insert(abs.clone());
                    self.set_made(abs, &set.lit, scope, depth, Made { st, path, outer: scope });
                }
                // an energy: each integrand with the scope's numbers written in (#121)
                StmtKind::Minimize(m) => self.walk_minimize(m, st, scope, vals, path),
                // a bound on measurements: the scope's numbers written in, what it measures
                // resolved with the references (`rewrite`) (#183 F3, #190)
                StmtKind::Measured(m) => {
                    let mut m2 = m.clone();
                    for text in m2.lhs.numbers_mut().into_iter().chain(m2.rhs.numbers_mut()) {
                        let held = &self.held.names;
                        *text = super::values::substitute_in(text, vals, self.units, held);
                    }
                    self.emit(StmtKind::Measured(m2), st, scope, path);
                }
                // a load case (#510 P4): the scope's numbers written in, what it names resolved
                // with the references (`rewrite`), and nothing drawn
                StmtKind::Scenario(sc) => {
                    let mut sc2 = sc.clone();
                    for text in sc2.numbers_mut() {
                        let held = &self.held.names;
                        *text = super::values::substitute_in(text, vals, self.units, held);
                    }
                    self.emit(StmtKind::Scenario(sc2), st, scope, path);
                }
                // a claim over an interval: its numbers and its relations' worked out as above
                StmtKind::ClaimOver(c) => self.walk_claim_over(c, st, scope, vals, path, depth),
                // a gauge or an orientation: kept as written, resolved later
                other => self.emit(other.clone(), st, scope, path),
            }
        }
    }

    /// **A reading at an instant** (#391 S2), `n := x at τ`: walked as each of what it may be — a
    /// frame carried, a solid placed, a point held — every statement tagged with which, and
    /// recorded with what it reads, for `settle_readings` to keep the one `x` is.
    #[allow(clippy::too_many_arguments)]
    fn walk_reading_at(
        &mut self,
        d: &Decl,
        st: &Stmt,
        scope: &Scope,
        vals: &Vals,
        path: &[PathStep],
        depth: usize,
        prefix: &str,
    ) {
        let c = d.carried.as_ref().expect("a reading at an instant");
        let k = self.readings.len();
        self.readings.push((c.source.clone(), scope.clone()));
        for (s, read) in super::carried::ways(st, d) {
            let sc = Scope { reading: Some((k, read)), ..scope.clone() };
            match &s.kind {
                StmtKind::Decl(x) => self.walk_decl(x, &s, &sc, vals, path, prefix),
                // a set's body walked for what it makes states nothing
                StmtKind::Relation(_) if matches!(scope.pred.pass, Pass::Made) => {}
                StmtKind::Relation(r) => self.walk_relation(r, &s, &sc, vals, path, depth),
                _ => unreachable!("a reading is written out as declarations and relations"),
            }
        }
    }

    /// **What each reading at an instant is** (#391 S2), once every name is known: the kind of
    /// what its source names — a solid, a point, or anything else a frame (a datum the standard
    /// library draws after the document is one) — and where the source is itself a reading,
    /// what that one turned out to be.  The other ways it was written out are dropped.
    pub(super) fn settle_readings(&mut self) {
        use super::carried::Read;
        if self.readings.is_empty() {
            return;
        }
        let alias = self.alias_table();
        let mut read: Vec<Option<Read>> = vec![None; self.readings.len()];
        // a source that is a reading waits for that one: round by round, until a round settles
        // nothing more
        loop {
            let mut settled = false;
            for k in 0..read.len() {
                if read[k].is_some() {
                    continue;
                }
                let (source, sc) = &self.readings[k];
                read[k] = match self.look(&alias).raw(source, sc) {
                    Some((abs, rest)) if rest.is_empty() => self.read_of(&abs, &read),
                    _ => Some(Read::Frame),
                };
                settled |= read[k].is_some();
            }
            if !settled {
                break;
            }
        }
        // a reading of itself, or of a reading of itself, reads nothing: refused, and written out
        // no way at all
        let cyclic: Vec<Ref> = self.readings.iter().zip(&read)
            .filter(|(_, r)| r.is_none()).map(|((source, _), _)| source.clone()).collect();
        for source in cyclic {
            self.err(Code::E041, source.span, format!("`{}` is read at an instant of itself",
                                                       ref_text(&source)));
        }
        self.out.retain(|(_, _, sc)| match sc.reading {
            Some((k, way)) => read[k] == Some(way),
            None => true,
        });
        for (_, _, sc) in &mut self.out {
            sc.reading = None;
        }
        self.readings.clear();
    }

    /// What a reading of the declaration named `abs` is: what its kind makes it, or, where that
    /// declaration is one way of writing out a reading, what that reading turned out to be —
    /// `None` while it is not yet known.  A name nothing in the walk declares is a frame's.
    fn read_of(&self, abs: &str, read: &[Option<super::carried::Read>])
        -> Option<super::carried::Read> {
        for (st, _, sc) in &self.out {
            let StmtKind::Decl(d) = &st.kind else { continue };
            if d.name.key().text != abs {
                continue;
            }
            match sc.reading {
                None => return Some(super::carried::Read::of(d.kind)),
                Some((j, way)) => match read[j] {
                    Some(r) if r == way => return Some(r),
                    Some(_) => {}
                    None => return None,
                },
            }
        }
        Some(super::carried::Read::Frame)
    }

    /// A named chain: its name made absolute under the scope, and stamped as its instances say.
    fn walk_chain(
        &mut self,
        chain: &crate::syntax::NamedChain,
        st: &Stmt,
        scope: &Scope,
        path: &[PathStep],
        prefix: &str,
    ) {
        let mut chain = chain.clone();
        let abs = format!("{prefix}{}", chain.name.key().text);
        self.names.insert(abs.clone());
        if chain.annotations.private { self.private_names.insert(abs.clone(), scope.owner.clone()); }
        chain.annotations.roles = chain.annotations.roles.union(scope.stamps.roles);
        chain.name = chain.name.prefixed(abs, scope.block.copies);
        if scope.stamps.anonymous {
            chain.name = crate::syntax::DeclName::Key(chain.name.key().clone());
        }
        self.emit(StmtKind::Chain(chain), st, scope, path);
    }

    /// A declaration: its name made absolute, its numbers worked out in the scope, and the
    /// twins, the probe and the set its region makes beside it.
    pub(super) fn walk_decl(
        &mut self,
        d: &Decl,
        st: &Stmt,
        scope: &Scope,
        vals: &Vals,
        path: &[PathStep],
        prefix: &str,
    ) {
        let abs = format!("{prefix}{}", d.name.key().text);
        self.names.insert(abs.clone());
        if d.kind == crate::model::EntKind::Solid {
            self.solid_names.insert(abs.clone());
        }
        if d.annotations.private { self.private_names.insert(abs.clone(), scope.owner.clone()); }
        // a spline through a chain's corners waits for every chain (#243 L7, `through`)
        if d.through.is_some() {
            self.aside.fits.push(super::through::Fit {
                d: d.clone(),
                st: st.clone(),
                scope: scope.clone(),
                path: path.to_vec(),
                prefix: prefix.to_string(),
            });
            return;
        }
        let mut d2 = d.clone();
        d2.name = d.name.prefixed(abs.clone(), scope.block.copies);
        if scope.stamps.anonymous {
            d2.name = crate::syntax::DeclName::Key(d2.name.key().clone());
        }
        // a computed point is made of expressions over the formals, which is a
        // thing a curve can be and a drawing cannot: nothing on the sheet holds a
        // point to a formula (§6.5)
        if let Some([(x, xs), (y, ys)]) = &d.computed {
            if self.sym.is_none() {
                let n = &d.name.key().text;
                self.err(
                    Code::E103,
                    d.name.span(),
                    format!(
                        "`{n}` is a computed point, so its component is drawn only \
                         as a curve: `e := trace(Component(…).{n} over u in (a, b))`"
                    ),
                );
                return;
            }
            d2.computed = Some([
                (self.subst_sym(x, vals, scope), *xs),
                (self.subst_sym(y, vals, scope), *ys),
            ]);
            self.emit(StmtKind::Decl(d2), st, scope, path);
            return;
        }
        if let Some(c) = d2.curve.as_mut() {
            // a point traced over a declared parameter with no interval of its own runs over the
            // parameter's (#391 S4); a component sees none
            if c.domain.0.is_empty() {
                let swept = &c.swept.text;
                let Some(p) = self.parameters.get(swept).filter(|_| scope.prefix().is_empty())
                else {
                    self.err(Code::E103, c.swept.span, format!("`{swept}` is no parameter the \
                        document declares: a curve says the interval its formal runs over, `in (a, \
                        b)`, or runs over a declared `param {swept}: … over (a, b)`"));
                    return;
                };
                c.domain = (crate::syntax::num(p.from), crate::syntax::num(p.to));
            }
            // the interval is written over the parameters in scope, like a number
            let second = c.second.as_mut().map(|(_, d)| d);
            let ends = [&mut c.domain.0, &mut c.domain.1].into_iter()
                .chain(second.into_iter().flat_map(|d| [&mut d.0, &mut d.1]));
            for t in ends {
                match value_of(t, vals, self.units) {
                    Ok(v) => *t = crate::syntax::num(v),
                    Err(e) => self.err(Code::E103, st.span, format!("`{t}`: {e}")),
                }
            }
            // an instance written in place is bound like any other and never
            // drawn: the curve is the only thing made of it (§6.5)
            if let CurveTarget::Anon(inst, point) = &c.target {
                if let Some((_, _, _, key)) =
                    self.bind_instance(inst, scope, vals, false)
                {
                    c.of = Some(crate::syntax::CurveOf {
                        instance: key,
                        point: ref_text(point),
                    });
                }
            }
        }
        // a face family: its interval worked out, its running number named, and the face it
        // sweeps read under the instance `walk_family` drew (#243 L1)
        if let Some(crate::syntax::Sweep::Family(f)) = d2.sweep.as_mut() {
            let key = format!("{}{}", scope.prefix(), f.instance.name.text);
            // refused where the instance was walked: no solid
            let Some(swept) = self.held.formals.get(&key).cloned() else { return };
            let held = format!("{key}.{swept}");
            // over a chain, each leg's running number and the face it sweeps (#243 L4)
            let legs = self.held.chained.get(&key).cloned().unwrap_or_default();
            if !self.held.names.contains_key(&held) && legs.is_empty() {
                return;
            }
            // the declared parameter's interval (#398 B), which `walk_family` read
            if let Some(r) = self.parameters.get(&f.swept.text) {
                f.domain = (crate::syntax::num(r.from), crate::syntax::num(r.to));
            }
            let instances: Vec<String> = if legs.is_empty() {
                vec![f.instance.name.text.clone()]
            } else {
                legs.iter().map(|(_, k)| format!("{}_{k}", f.instance.name.text)).collect()
            };
            f.held = Some(legs.first().map_or(held, |(h, _)| h.clone()));
            f.legs = legs;
            let faces = instances.into_iter().map(|text| {
                let mut face = f.face.clone();
                face.path.insert(0, crate::syntax::Seg::Field(face.root.clone()));
                face.root = Name { text, span: face.span };
                crate::syntax::Kid::Ref(face)
            }).collect();
            if let Some(slot) = d2.children.first_mut() {
                *slot = faces;
            }
        }
        self.declared_interval(&mut d2, scope);
        self.settle_seeds(&mut d2, vals, scope, st.span);
        // and a solid's sweep, which is written in the same little language
        if let Some(sw) = d2.sweep.as_mut() {
            for a in sw.args_mut() {
                self.settle_arg(a, vals, scope);
            }
        }
        if let Some(e) = d2.angular_span.as_mut() {
            self.settle_arg(&mut e.from, vals, scope);
            self.settle_arg(&mut e.to, vals, scope);
            for a in e.ease.iter_mut().flat_map(|x| x.args_mut()) {
                self.settle_arg(a, vals, scope);
            }
        }
        self.stamp_scope_plane(&mut d2, scope);
        // a region solid's set, read once: a copy's solid of a set that is one already a copy's
        // solid is that solid (#415), and any other makes its region of it
        let set = self.region_set(&d2, scope);
        if self.same_solid(&d2, &abs, set.as_ref(), scope) {
            return;
        }
        let twins = self.twins_of(&d2, &d.name.key().text);
        let set = set.map(|(set, _, _)| set);
        let region: Vec<_> = match self.region_of(&mut d2, &d.name.key().text, set) {
            Some(r) => vec![r],
            None => self.envelope_of(&mut d2, &d.name.key().text, scope),
        };
        self.emit(StmtKind::Decl(d2), st, scope, path);
        for twin in twins {
            self.emit(StmtKind::Decl(twin), st, scope, path);
        }
        // a region solid's probe, and its set applied to it; an envelope's or a meshing's
        for (probe, inside) in region {
            self.emit(StmtKind::Decl(probe), st, scope, path);
            self.emit(StmtKind::Relation(inside), st, scope, path);
        }
    }

    /// **A reading over a declared parameter reads its interval** (#391 S2): `over t`, with no
    /// `in (a, b)`, names a `param t: Angle over (a, b)` of the document, and its ends are that
    /// declaration's — the roll is an angle, so a parameter of another type is refused (a sweep's
    /// by the elaborator, since a face of the drawing swept over one is a face family, #398 B), as
    /// is a name the document declares no parameter by (a component's body sees none).
    fn declared_interval(&mut self, d: &mut Decl, scope: &Scope) {
        let (over, from, to, swept) = match (&mut d.sweep, &mut d.angular_span) {
            (Some(crate::syntax::Sweep::Swept { over, from, to, .. }), _) => {
                (&*over, from, to, true)
            }
            (_, Some(crate::syntax::AngularSpan { over: Some(over), from, to, .. })) => {
                (&*over, from, to, false)
            }
            _ => return,
        };
        if !over.declared {
            return;
        }
        let name = &over.name;
        let declared = (scope.module.is_none() && scope.prefix().is_empty())
            .then(|| self.parameters.get(&name.text)).flatten();
        let Some(running) = declared else {
            let m = format!("`{}` names no parameter the document declares: a reading over the \
                             roll states its interval, `over {} in (a, b)`, or runs over a \
                             `param {}: Angle over (a, b)`", name.text, name.text, name.text);
            self.err(Code::E103, name.span, m);
            return;
        };
        // a face of the drawing may be swept over a parameter of any kind (#398 B), a face
        // family: the elaborator, which knows what the tool is, refuses a solid swept so
        let angle = running.dim == crate::units::Dim::ANGLE;
        if !angle && !swept {
            let m = format!("`{}` is no angle: a reading over the roll runs over an angle",
                            name.text);
            self.err(Code::E103, name.span, m);
            return;
        }
        let unit = if angle { "deg" } else { "" };
        let end = |v: f64| crate::syntax::Arg::Dim {
            text: format!("{}{unit}", crate::syntax::num(v)), span: name.span };
        (*from, *to) = (end(running.from), end(running.to));
    }

    /// **A face family's station** (#243 L1): the instance written in place, drawn under a key
    /// the source cannot write (`#f12.`) with the running formal left unbound, so it is the free
    /// value `bind` makes of one (`#f12.s`) — and that value *held* (`Held`): the drawn station
    /// stands at the middle of the interval, the family solves it again at every other.  The
    /// interval is a declared parameter's (#398 B), `over s` naming `param s: Scalar over (a, b)`,
    /// and the formal that runs is the one the call gives it (`Tube(g, s: u)` over `u`), or else
    /// the one named as it.
    fn walk_family(
        &mut self,
        f: &crate::syntax::FamilySpec,
        st: &Stmt,
        scope: &Scope,
        vals: &Vals,
        path: &[PathStep],
        depth: usize,
    ) -> ControlFlow<()> {
        let inst = &f.instance;
        let comp = match self.prog.resolve_component(&inst.component.text, scope.module) {
            Ok(i) => &self.prog.components[i],
            Err(m) => {
                self.err(Code::E103, inst.component.span, m);
                return ControlFlow::Continue(());
            }
        };
        let Some(running) = self.family_parameter(f, scope) else {
            return ControlFlow::Continue(());
        };
        let (param, cname) = (&f.swept.text, &inst.component.text);
        // the formal the call gives the parameter, or the one named as it
        let given = inst.args.iter().position(|a| match &a.value {
            crate::syntax::InstVal::Ref(r) => r.path.is_empty() && r.root.text == *param,
            crate::syntax::InstVal::Expr(t) => t.trim() == param.as_str(),
            _ => false,
        });
        let swept = match given {
            Some(i) => match &inst.args[i].label {
                Some(l) => l.text.clone(),
                None => comp.formals.get(i).map(|fo| fo.name.text.clone()).unwrap_or_default(),
            },
            None => param.clone(),
        };
        let Some(formal) = comp.formals.iter().find(|fo| fo.name.text == swept) else {
            self.err(Code::E103, f.swept.span, format!("`{cname}` has no formal `{swept}` to run \
                over `{param}`: give it one, `{cname}(…, s: {param})`"));
            return ControlFlow::Continue(());
        };
        if !matches!(formal.ty, Ty::Scalar | Ty::Length | Ty::Angle) {
            let m = format!("a face family runs a number, and `{swept}` is not one");
            self.err(Code::E103, f.swept.span, m);
            return ControlFlow::Continue(());
        }
        if formal.ty.dim() != running.dim {
            let m = format!("`{param}` is a {}, and `{cname}`'s `{swept}` a {}: a face family runs \
                             its formal over a parameter of its kind", running.dim.name(),
                            formal.ty.dim().name());
            self.err(Code::E103, f.swept.span, m);
            return ControlFlow::Continue(());
        }
        // what else the call gives the formal: it runs, so the call leaves it to the parameter
        let bound = inst.args.iter().enumerate().any(|(i, a)| Some(i) != given
            && a.label.as_ref().map_or_else(|| comp.formals.get(i).is_some_and(|fo|
                fo.name.text == swept), |l| l.text == swept));
        if bound {
            let m = format!("`{swept}` runs over `{param}`, so the call gives it nothing else");
            self.err(Code::E103, f.swept.span, m);
            return ControlFlow::Continue(());
        }
        let mut inst = inst.clone();
        if let Some(i) = given {
            // the arguments after it keep their formals, labelled, once it is taken off the call
            for (k, a) in inst.args.iter_mut().enumerate().skip(i + 1) {
                if a.label.is_none() {
                    a.label = comp.formals.get(k).map(|fo| Name { text: fo.name.text.clone(),
                                                                  span: a.span });
                }
            }
            inst.args.remove(i);
        }
        let (a, b) = (running.from, running.to);
        let key = format!("{}{}", scope.prefix(), inst.name.text);
        self.held.formals.insert(key, swept.clone());
        // a path given a named chain: one leg per link the interval reaches (#243 L4), every
        // chain the call gives run in step (L7)
        let chains = self.chain_arguments(comp, &inst, scope);
        if !chains.is_empty() {
            return self.walk_legs(f, &swept, &inst, chains, (a, b), formal.ty, st, scope, vals,
                                  path, depth);
        }
        let held = format!("{}{}.{swept}", scope.prefix(), inst.name.text);
        self.held.names.insert(held, (formal.ty.dim(), 0.5 * (a + b)));
        self.walk_instance(&inst, st, scope, vals, path, depth)
    }

    /// **The parameter a face family runs over** (#398 B): `over s` names a `param s: … over (a,
    /// b)` the document declares — a component's body sees none, so a family stands where its
    /// parameter is declared.  Said and `None` where it names none.
    pub(super) fn family_parameter(&mut self, f: &crate::syntax::FamilySpec, scope: &Scope)
        -> Option<crate::model::Running> {
        let name = &f.swept;
        let declared = (scope.module.is_none() && scope.prefix().is_empty())
            .then(|| self.parameters.get(&name.text).cloned()).flatten();
        if declared.is_none() {
            let m = format!("`{0}` names no parameter the document declares: a face family runs \
                             over one, `param {0}: Scalar over (a, b)`, and stands where it is \
                             declared", name.text);
            self.err(Code::E103, name.span, m);
        }
        declared
    }

    /// **A face family over a chain** (#243 L4, finding 6): the path formal given a named open
    /// chain, the station runs over it link by link — link `k` while `s` is in `(k, k + 1)` — as
    /// one family.  Each link the interval reaches is a **leg**: the instance drawn under its own
    /// key (`#f12_k`), the formal given that link, its own running number held at the middle of
    /// its part of the interval, and the family's formal read as `k` past it (`bind`), so a law
    /// in `s` reads the chain's `s` on every link; a contact pinned to `s` on the link reads the
    /// leg's own number, the fraction of that link's run (`settle_relation`).
    ///
    /// **Several chains run in step** (#243 L7): every `path` formal given a named open chain is
    /// given its link `k` on leg `k` — a guide and the rails of a loft through several sections,
    /// each a link a pair of neighbouring sections — so the chains must have as many links.
    #[allow(clippy::too_many_arguments)]
    fn walk_legs(
        &mut self,
        f: &crate::syntax::FamilySpec,
        swept: &str,
        inst: &crate::syntax::Instance,
        chains: Vec<ChainArgument>,
        (a, b): (f64, f64),
        ty: Ty,
        st: &Stmt,
        scope: &Scope,
        vals: &Vals,
        path: &[PathStep],
        depth: usize,
    ) -> ControlFlow<()> {
        let n = chains[0].2.len();
        if let Some((_, r, other)) = chains.iter().find(|(_, _, l)| l.len() != n) {
            let (first, m) = (ref_text(&chains[0].1), other.len());
            let m = format!("a face family runs the chains it is given in step, link by link, and \
                             `{}` has {m} link{} where `{first}` has {n}", ref_text(r),
                            if m == 1 { "" } else { "s" });
            self.err(Code::E080, r.span, m);
            return ControlFlow::Continue(());
        }
        if ty != Ty::Scalar {
            let m = format!("a face family over a chain runs `{swept}` link by link, from 0 to \
                             the chain's {n}: it is a `Scalar`");
            self.err(Code::E103, f.swept.span, m);
            return ControlFlow::Continue(());
        }
        if a < 0.0 || b > n as f64 {
            let m = format!("the chain has {n} link{}, so a family over it runs within (0, {n}): \
                             link `k` while `{swept}` is in (k, k + 1)",
                            if n == 1 { "" } else { "s" });
            self.err(Code::E080, st.span, m);
            return ControlFlow::Continue(());
        }
        let key = format!("{}{}", scope.prefix(), inst.name.text);
        let mut legs = Vec::new();
        for k in 0..n {
            let (lo, hi) = (a.max(k as f64), b.min(k as f64 + 1.0));
            if hi - lo <= 1e-12 * (b - a) {
                continue;
            }
            let mut leg = inst.clone();
            leg.name.text = format!("{}_{k}", inst.name.text);
            let mut abs = Vec::with_capacity(chains.len());
            for (at, _, links) in &chains {
                let (link, full) = links[k].clone();
                leg.args[*at].value = crate::syntax::InstVal::Ref(link);
                abs.push(full);
            }
            let held = format!("{}{}.{swept}", scope.prefix(), leg.name.text);
            self.held.names.insert(held.clone(), (ty.dim(), 0.5 * (lo + hi) - k as f64));
            self.held.legs.insert(held.clone(), (k as f64, abs));
            legs.push((held, k as f64));
            self.walk_instance(&leg, st, scope, vals, path, depth)?;
        }
        self.held.chained.insert(key, legs);
        ControlFlow::Continue(())
    }

    /// The named chains a call gives its `path` formals (#243 L4, L7): each argument's index, the
    /// argument as written, and each link as a reference the call's scope reads with its absolute
    /// name.  A closed chain is no path a family can run along, its ends meeting; said here, and
    /// then no chain is run.
    fn chain_arguments(
        &mut self,
        comp: &Component,
        inst: &crate::syntax::Instance,
        scope: &Scope,
    ) -> Vec<ChainArgument> {
        let mut out = Vec::new();
        for fo in comp.formals.iter().filter(|fo| fo.ty == Ty::Path) {
            match self.chain_argument(comp, inst, scope, fo) {
                Ok(Some(c)) => out.push(c),
                Ok(None) => {}
                Err(()) => return Vec::new(),
            }
        }
        out
    }

    /// The named chain a call gives the `path` formal `fo`, where it gives one, or `Err` where it
    /// is refused (said here).
    fn chain_argument(
        &mut self,
        comp: &Component,
        inst: &crate::syntax::Instance,
        scope: &Scope,
        fo: &crate::syntax::Formal,
    ) -> Result<Option<ChainArgument>, ()> {
        let Some(at) = super::bindings::arg_index(comp, inst, fo) else { return Ok(None) };
        let crate::syntax::InstVal::Ref(r) = &inst.args[at].value else { return Ok(None) };
        let alias = self.alias_table();
        let Some((chain, sc)) = self.chain_named(&alias, r, scope) else { return Ok(None) };
        if chain.closed {
            let m = format!("`{}` is a closed chain, and a face family along it would meet \
                             itself: run it over an open one", ref_text(r));
            self.err(Code::E080, r.span, m);
            return Err(());
        }
        let mut links = Vec::new();
        for l in &chain.links {
            let Some((full, local)) = self.link_read(&alias, l, &sc, scope) else {
                let m = format!("`{}`'s link `{}` is not one the call can name", ref_text(r),
                                ref_text(l));
                self.err(Code::E103, r.span, m);
                return Err(());
            };
            links.push((Ref::dotted(&local, r.span), full));
        }
        Ok(Some((at, r.clone(), links)))
    }

    /// The named chain `r` names in `scope`, as the walk has made it so far, and the scope it was
    /// written in.
    pub(super) fn chain_named(
        &self,
        alias: &dyn super::resolve::Aliases,
        r: &Ref,
        scope: &Scope,
    ) -> Option<(crate::syntax::NamedChain, Scope)> {
        let (abs, rest) = self.look(alias).find(r, scope)?;
        if !rest.is_empty() {
            return None;
        }
        self.out.iter().find_map(|(st, _, sc)| match &st.kind {
            StmtKind::Chain(c) if c.name.key().text == abs => Some((c.clone(), sc.clone())),
            _ => None,
        })
    }

    /// A chain's link `l`, written in `sc`: its absolute name, and the name `scope` reads it by.
    pub(super) fn link_read(
        &self,
        alias: &BTreeMap<String, String>,
        l: &Ref,
        sc: &Scope,
        scope: &Scope,
    ) -> Option<(String, String)> {
        let (t, rest) = self.look(alias).find(l, sc)?;
        let full = std::iter::once(t).chain(rest).collect::<Vec<_>>().join(".");
        let local = scope.prefixes.iter()
            .find_map(|p| full.strip_prefix(p.as_str()).filter(|n| !n.is_empty()))?.to_string();
        Some((full, local))
    }

    /// An instance: its arguments bound and its component's body walked under its prefix — or
    /// the walk stopped, past the instances' cap.
    fn walk_instance(
        &mut self,
        inst: &crate::syntax::Instance,
        st: &Stmt,
        scope: &Scope,
        vals: &Vals,
        path: &[PathStep],
        depth: usize,
    ) -> ControlFlow<()> {
        // a body of nothing but calls emits no statement, so the statements' cap
        // never stops a recursion branching at every level: the instances have one
        if self.instances.len() >= MAX_FLAT {
            self.once(Code::E103, st.span, format!("more than {MAX_FLAT} instances once expanded"));
            return ControlFlow::Break(());
        }
        let Some((comp, sub_vals, sides, key)) =
            self.bind_instance(inst, scope, vals, true)
        else {
            return ControlFlow::Continue(());
        };
        // the instance's own `in`, or the one already in force around it — both at once
        // nest (#383): the one around it is ridden on, as are the instance's further blocks
        let mut rides = scope.stamps.rides.clone();
        let here = |plane: &Ref| InPlane { owner: scope.owner.clone(), plane: plane.clone(),
                                           prefixes: scope.prefixes.clone(), closed: scope.closed };
        let in_plane = match (inst.membership.plane(), &scope.stamps.plane) {
            // written here, in this scope: it resolves against these prefixes
            (Some(p), outer) => {
                rides.extend(outer.iter().cloned());
                rides.extend(inst.membership.rides().iter().map(here));
                Some(here(p))
            }
            (None, q) => q.clone(),
        };
        // a symmetric family's instance given what one before it was given, in another order it
        // allows, is that one (#415): a name for it, and nothing made again
        let stamps = Stamps {
            roles: scope.stamps.roles.union(inst.annotations.roles),
            plane: in_plane,
            rides,
            class: {
                let mut c = scope.stamps.class.clone();
                c.0.extend(inst.class.0.iter().cloned());
                c
            },
            anonymous: scope.stamps.anonymous || inst.name.text.starts_with('#'),
        };
        if self.same_instance(comp, &key, scope, &sub_vals, &sides, &stamps) {
            return ControlFlow::Continue(());
        }
        // the call's numbers over the constants of the file the component is written in (#461),
        // its other numbers out of reach as the caller's are
        let (constants, unknowns) = self.file_numbers(comp.module);
        let forbidden = Outside { around: scope.vals.clone(), file: self.file_vals.clone(), unknowns };
        let mut sub_vals = Vals::over_with(constants, sub_vals);
        let given = std::rc::Rc::new(sub_vals.clone());
        let sc = Scope {
            owner: key.clone(),
            access: scope.access.clone(),
            typed: scope.typed.clone(),
            prefixes: vec![key],
            closed: true,
            forbidden,
            groups: std::rc::Rc::new(comp.formals.iter().filter(|f| matches!(f.ty, Ty::Group | Ty::Set)).map(|f| f.name.text.clone()).collect()),
            vals: given.clone(),
            // the sides this instance was given, and no others: a component reads a
            // side by the name of its own formal, as it reads every other argument
            sides: std::rc::Rc::new(sides),
            // and its body's calls name components from the file it was written in
            module: comp.module,
            block: InBlock {
                cyc: None,
                // a ring's copy reaches into the instances written in it
                ring: scope.block.ring.clone(),
                copies: scope.block.copies,
            },
            stamps,
            pred: PredWalk { pass: Pass::Itself, written: None, held: false },
            reading: None,
        };
        // a component reached again while it is still being expanded recurses
        // (§8), ended by a `when` over its numbers: given the numbers it was given
        // there, every choice comes out the same and nothing ends it, so that, and
        // nesting past the cap, is E003, said once at the call closing the cycle
        let same = self.instantiating.iter()
            .rposition(|(c, given)| std::ptr::eq(*c, comp) && given.own() == sub_vals.own());
        let deep = depth + 1 >= MAX_DEPTH
            && self.instantiating.iter().any(|(c, _)| std::ptr::eq(*c, comp));
        if same.is_some() || deep {
            let name = |c: &Component| c.name.as_ref().map_or(String::new(), |n| n.text.clone());
            let message = match same {
                None => format!("`{}` instantiates itself more than {MAX_DEPTH} deep: \
                                 its `when` does not end it soon enough", name(comp)),
                Some(k) => {
                    let mut through: Vec<String> = Vec::new();
                    for (c, _) in &self.instantiating[k + 1..] {
                        let n = format!("`{}`", name(c));
                        if !std::ptr::eq(*c, comp) && !through.contains(&n) {
                            through.push(n);
                        }
                    }
                    let through = match through.is_empty() {
                        true => String::new(),
                        false => format!(" through {}", through.join(", ")),
                    };
                    format!("`{}` instantiates itself{through} with the numbers it was \
                             given, so nothing ends it: a `when` over a number each \
                             call changes does", name(comp))
                }
            };
            self.once(Code::E003, st.span, message);
            return ControlFlow::Continue(());
        }
        let mut instance_path = path.to_vec();
        instance_path.push(PathStep::Instance(st.id));
        // the last bound is this one: nothing since `bind_instance` pushed another
        if let Some(info) = self.instances.last_mut().filter(|i| i.prefix == sc.owner) {
            info.path = instance_path.clone();
        }
        // an instance of a family of sets is the set, its formals bound (§6.21) —
        // and what its body makes without reading its point is the instance's,
        // made once here (F3)
        let own = match &comp.set {
            Some(lit) => {
                let name = sc.prefix().trim_end_matches('.').to_string();
                let (own, lit) = super::sets::instance_part(lit);
                let made = Made { st, path, outer: scope };
                self.set_made(name, &lit, &sc, depth + 1, made);
                own
            }
            None => Vec::new(),
        };
        self.instantiating.push((comp, given));
        // a point formal drawn in a plane and left out is the instance's own point there, made
        // by the call (§5 [0.95]) — in its formal's plane, whatever plane the instance is in
        let owned = self.owned_points(comp, inst, st, vals);
        if !owned.is_empty() {
            let mut at_call = sc.clone();
            at_call.stamps.plane = None;
            self.body(&owned, &at_call, &mut sub_vals, path, depth + 1);
        }
        self.body(&comp.body, &sc, &mut sub_vals, &instance_path, depth + 1);
        if !own.is_empty() {
            self.body(&own, &sc, &mut sub_vals, &instance_path, depth + 1);
        }
        self.instantiating.pop();
        ControlFlow::Continue(())
    }

    /// A block: a `when` made or not, one over a chain set aside until the chain is found, or
    /// its copies.
    fn walk_block(
        &mut self,
        b: &Block,
        st: &Stmt,
        scope: &Scope,
        vals: &Vals,
        path: &[PathStep],
        depth: usize,
    ) {
        // made once in this scope, or not at all (#210): no copy, no prefix, so what
        // it declares is named as if written beside it, and the numbers it defines
        // are its own
        if b.kind == BlockKind::When {
            let names = Asked::new(self, scope);
            match super::values::condition(&b.count, vals, scope, self.units, &names) {
                Ok(true) => {
                    // a frame of its own over the numbers around it, which it shares (#405)
                    let mut sub = Vals::over(scope.vals.clone());
                    self.body(&b.body, scope, &mut sub, path, depth + 1);
                }
                Ok(false) => {}
                Err((code, message)) => self.once(code, b.head, message),
            }
            return;
        }
        if b.over.is_some() {
            // how many is the chain's, and the chain may be a name no walk has
            // reached yet (a forward reference, or one inside an instance written
            // further down): expanded once every name is known, where the block
            // itself stands in the output until then
            self.aside.blocks.push(Pending {
                st: st.clone(),
                scope: scope.clone(),
                path: path.to_vec(),
                depth,
            });
            self.out.push((st.clone(), path.to_vec(), scope.clone()));
            return;
        }
        // a count is a whole number: a computed one (`360deg / 15deg`) may carry
        // the arithmetic's last bit, but 2.5 is not rounded to anything (#115)
        let n = match value_of(&b.count, vals, self.units) {
            Ok(v) if v.is_finite() && v >= 0.0
                && (v - v.round()).abs() <= 1e-9 * v.max(1.0) => v.round() as usize,
            Ok(v) => {
                self.err(
                    Code::E103,
                    b.span,
                    format!("`{}` is {v}, which is not a count", b.count),
                );
                return;
            }
            Err(e) => {
                self.err(Code::E103, b.span, format!("`{}`: {e}", b.count));
                return;
            }
        };
        self.copies(b, st, scope, path, depth, n, &[]);
    }

    /// A relation: a defined word's body applied, or its numbers worked out in the scope and
    /// stated — as its derivative where a tangency walks the body.
    fn walk_relation(
        &mut self,
        rel: &crate::syntax::Relation,
        st: &Stmt,
        scope: &Scope,
        vals: &Vals,
        path: &[PathStep],
        depth: usize,
    ) {
        // a step is the ordinates and levels it is short for, stated where it stands (#330)
        if self.walk_step(rel, st, scope, vals, path) {
            return;
        }
        // a word a file defines is its body, applied where the statement stands
        if self.apply_word(st, rel, path, scope, depth) {
            return;
        }
        // only a defined word's body makes anything a name could stand for
        if let Some(n) = &rel.name {
            let word = rel.form.written().map_or("this relation", |w| w.word.text.as_str());
            self.err(Code::E001, n.span, format!(
                "`{}` names what a word's body makes, and `{word}` makes nothing: name a use of a \
                 word a file defines (`rim := a round(r: 3mm) b`), or write the relation bare",
                n.text));
            return;
        }
        let Some(mut r2) = self.settle_relation(rel, vals, scope) else { return };
        if let Some(twin) = scope.twin() {
            // a claim, a gauge and a bound state no row, so they have no
            // derivative
            // — `inside` or `outside` a plane among the bounds, lowered later
            let (gauge, bound) = match r2.form.written() {
                Some(w) => (crate::constraints::gauge_op(&w.word.text).is_some(),
                            w.states_bound()),
                None => (false, false),
            };
            if r2.claim || gauge || bound {
                return;
            }
            r2.along = Some(twin.clone());
        }
        self.emit(StmtKind::Relation(r2), st, scope, path);
    }

    /// An energy: each integrand with the scope's numbers written in (#121).
    fn walk_minimize(
        &mut self,
        m: &crate::syntax::Minimize,
        st: &Stmt,
        scope: &Scope,
        vals: &Vals,
        path: &[PathStep],
    ) {
        let mut m2 = m.clone();
        let mut ok = true;
        for t in &mut m2.terms {
            match super::values::settle_integrand(t, vals, self.units) {
                Ok(text) => t.body = text,
                Err(e) => {
                    self.err(Code::E101, t.body_span, e);
                    ok = false;
                }
            }
        }
        if ok {
            self.emit(StmtKind::Minimize(m2), st, scope, path);
        }
    }

    /// A claim over an interval: its numbers and its relations' worked out as a lone relation's.
    fn walk_claim_over(
        &mut self,
        c: &crate::syntax::ClaimOver,
        st: &Stmt,
        scope: &Scope,
        vals: &Vals,
        path: &[PathStep],
        depth: usize,
    ) {
        let mut c2 = c.clone();
        for a in [&mut c2.from, &mut c2.to] { self.settle_arg(a, vals, scope); }
        let mut body = Vec::with_capacity(c.body.len());
        for inner in &c.body {
            let StmtKind::Relation(rel) = &inner.kind else {
                body.push(inner.clone());
                continue;
            };
            // a word's relations are claimed as the statement is
            let from = self.out.len();
            if self.apply_word(inner, rel, path, scope, depth) {
                for (made, _, _) in self.out.drain(from..).collect::<Vec<_>>() {
                    match made.kind {
                        StmtKind::Relation(_) => body.push(made),
                        _ => self.once(Code::E103, inner.span, "a word in a swept claim \
                            states relations, and declares nothing"),
                    }
                }
                continue;
            }
            let Some(r2) = self.settle_relation(rel, vals, scope) else { continue };
            body.push(Stmt { kind: StmtKind::Relation(r2), ..inner.clone() });
        }
        c2.body = body;
        self.emit(StmtKind::ClaimOver(c2), st, scope, path);
    }

    /// A relation's numbers worked out against the parameters in scope, in either of its
    /// representations, and the enclosing instances' classes stamped over its own.
    pub(super) fn settle_relation(&mut self, rel: &crate::syntax::Relation, vals: &Vals, scope: &Scope)
        -> Option<crate::syntax::Relation> {
        let mut r2 = rel.clone();
        self.held.on_leg = self.leg_named(&r2, scope);
        if let Some(w) = r2.form.written_mut() {
            for a in w.args.iter_mut() {
                match a {
                    crate::syntax::OpArg::Slot { arg, .. } => self.settle_arg(arg, vals, scope),
                    crate::syntax::OpArg::Vector { parts, .. } => {
                        for arg in parts {
                            self.settle_arg(arg, vals, scope)
                        }
                    }
                    crate::syntax::OpArg::Named(_, arg) => self.settle_arg(arg, vals, scope),
                    crate::syntax::OpArg::Dim(text, span) => {
                        self.settle_dim(text, *span, vals, scope)
                    }
                    crate::syntax::OpArg::Bound { hi, grow, .. } => {
                        for (text, span) in hi.iter_mut().chain(grow.iter_mut()) {
                            self.settle_dim(text, *span, vals, scope)
                        }
                    }
                    crate::syntax::OpArg::Ent(_) => {}
                }
            }
        }
        for a in r2.form.canonical_args_mut().iter_mut().flatten() {
            self.settle_arg(a, vals, scope);
        }
        self.held.on_leg = None;
        // **a rate is read along a formal its body runs over** (#339 D2, D3): kept, the formal
        // named as the running number it is given; not a row at a drawn instance's home, so not
        // stated there
        if let Some(w) = r2.form.written_mut() {
            let mut stated = true;
            for rate in w.rates.iter_mut() {
                match self.rate_over(&rate.over.text, vals) {
                    // a name no formal is in scope is the slot of the contact holding the point
                    // (#365), which the elaborator finds
                    Ok(Some(text)) if text.is_empty() => rate.slot = true,
                    Ok(Some(text)) => rate.over.text = text,
                    Ok(None) => stated = false,
                    Err((code, m)) => {
                        self.err(code, rate.over.span, m);
                        stated = false;
                    }
                }
            }
            if !stated {
                return None;
            }
        }
        // the enclosing instances' classes, over the statement's own: a ghosted part's
        // dimensions are the ghost's (`display: geometry`) as its lines are
        for c in &scope.stamps.class.0 {
            if !r2.class.has(c) {
                r2.class.0.push(c.clone());
            }
        }
        Some(r2)
    }

    /// The running number `rate(x over s)`'s formal `s` is given (#339 D2, D3): `None` where it
    /// is given a number, the home of a drawn instance.  A face family's running number, or a
    /// multiple of it, is its own; where a component is expanded to be traced (`sym`) a formal
    /// handed on as itself (`Drag(…, u: u)`) is the formal it was handed, which the trace runs
    /// over.  A name nothing in scope is, outside a trace, is the slot of the contact holding the
    /// rate's point (#365): `Some("")`, found by the elaborator, which refuses it where no contact
    /// owns one.  Anything else is no number running, and refused.
    fn rate_over(&self, s: &str, vals: &Vals)
        -> Result<Option<String>, (Code, String)> {
        let Some(a) = vals.get(s) else {
            if self.sym.is_none() && self.held.names.get(s).is_none() {
                return Ok(Some(String::new()));
            }
            return Err((Code::E040, format!("`{s}` is no formal here: a rate runs over a formal \
                the body is given its running number in, `rate(x over s)`")));
        };
        match &a.free {
            None => Ok(None),
            Some(u) if self.held.names.contains_key(u) && a.m != 0.0 => Ok(Some(u.clone())),
            Some(u) if self.sym.is_some() && a.m == 1.0 && a.c == 0.0 => Ok(Some(u.clone())),
            Some(u) if self.sym.is_some() => Err((Code::E103, format!("`{s}` is given an \
                expression in `{u}`, and a rate is read along a formal handed on as itself: give \
                the component `{s}: {u}`"))),
            Some(_) => Err((Code::E040, format!("`{s}` is an unknown of the drawing: a rate runs \
                along a face family's running number, `sweep(I(…).f over s)`, or a \
                traced body's formal"))),
        }
    }

    /// The leg of a family over a chain whose link relation `r`, written in a leg's station,
    /// names among its operands (#243 L4) — a link of any chain the leg runs (L7): that leg's
    /// running number and the link's index.
    fn leg_named(&self, r: &crate::syntax::Relation, scope: &Scope) -> Option<(String, f64)> {
        if self.held.legs.is_empty() {
            return None;
        }
        let w = r.form.written()?;
        let prefix = scope.prefix();
        let (held, (k, links)) = self.held.legs.iter().find(|(h, _)| {
            prefix.starts_with(&crate::model::station_prefix(h))
        })?;
        let alias = self.alias_table();
        w.ops.iter().any(|op| self.look(&alias).find(op, scope).is_some_and(|(t, rest)| {
            links.contains(&std::iter::once(t).chain(rest).collect::<Vec<_>>().join("."))
        })).then(|| (held.clone(), *k))
    }

    /// A seed written as an expression is worked out here, against the parameters in scope,
    /// and is a number from now on.
    fn settle_seeds(
        &mut self,
        d: &mut Decl,
        vals: &Vals,
        scope: &Scope,
        span: Span,
    ) {
        for (at, m) in self.seed_texts(d, vals, scope, span) {
            self.err(Code::E103, at, m);
        }
    }

    /// `settle_seeds`, the mistakes it finds handed back to be said.
    fn seed_texts(&self, d: &mut Decl, vals: &Vals, scope: &Scope, span: Span) -> Vec<(Span, String)> {
        // An unbound numeric formal is a solver unknown, but a hint still needs a
        // starting value. Read it only in hints — where the unknown's own seed says it starts
        // (`lean: hint(20deg)`), else at zero; constraints keep the symbolic value. Traced
        // components retain their parameterized seeds.
        let seeded;
        let vals: &dyn Table = if self.sym.is_none() && vals.any_unknown() {
            seeded = Seeded { vals, unknowns: &self.unknowns, held: &self.held.names };
            &seeded
        } else {
            vals
        };
        let mut faults = Vec::new();
        // a rational spline's weights: numbers over this scope's values, dimensionless
        for w in d.weights.iter_mut().flatten() {
            let Some(t) = w.text.take() else { continue };
            let t = self.subst_sym(&t, vals, scope);
            match super::values::value_aff(&t, vals, self.units) {
                Ok(a) if a.free.is_none() && a.dim.is_scalar() => w.value = a.c,
                Ok(_) => faults.push((w.span, format!("a weight is a plain number, and `{t}` is not"))),
                Err(e) => faults.push((w.span, format!("`{t}`: {e}"))),
            }
        }
        // one seed's text settled into `seed`, or kept, or a mistake to say
        let mut settle = |text: &mut Option<String>, seed: &mut f64| {
            let Some(t) = text.take() else { return };
            match self.settle_seed(t, text, vals, scope) {
                Ok(Some(v)) => *seed = v,
                Ok(None) => {}
                Err(m) => faults.push((span, m)),
            }
        };
        for (text, seed) in d.seed_text.iter_mut().zip(d.seed.iter_mut()) {
            settle(text, seed);
        }
        // a seed in a child slot — `line l(hint((w / 2, 0)), …)` — is the same clause one
        // level down, and is written over the same parameters
        for kid in d.children.iter_mut().flatten() {
            let crate::syntax::Kid::Hint(ks) = kid else { continue };
            for (text, seed) in ks.text.iter_mut().zip(ks.v.iter_mut()) {
                settle(text, seed);
            }
            // how far a direction read off geometry is turned: a number over the parameters in
            // scope, read by the build as a place's `turn:` is
            if let Some((b, _)) = ks.dir.as_mut().and_then(|d| d.turn.as_mut()) {
                *b = if self.sym.is_some() {
                    self.subst_sym(b, vals, scope)
                } else {
                    super::values::substitute_in(b, vals, self.units, &self.held.names)
                };
            }
        }
        for (b, _) in d.seed_at.iter_mut().flat_map(|a| a.texts_mut()) {
            // a place's numbers are texts over the parameters in scope, read later — by the
            // trace compile off its variable table, or by the build off the geometry's seeds —
            // so the numbers in force are written in and the rest is left for that reader
            *b = if self.sym.is_some() {
                self.subst_sym(b, vals, scope)
            } else {
                super::values::substitute_in(b, vals, self.units, &self.held.names)
            };
        }
        faults
    }

    /// One seed's text, settled over the parameters in scope: the number it comes to, or —
    /// where it cannot come to one here — the text kept in `keep` for a later reader.
    ///
    /// Two readers keep a text.  A trace block's compile reads a variable of the curve or a
    /// formal's coordinate off its own table.  And on the sheet a seed may **read geometry**
    /// (`hint((k.center.x + k.r, pin.y))`): the value read is that scalar's own *seed*,
    /// which is a fact about the built drawing and not about this scope, so `program::build`
    /// works it out once every declaration has one (§6.4).  Any other failure is the error it
    /// always was.
    fn settle_seed(
        &self,
        t: String,
        keep: &mut Option<String>,
        vals: &dyn Table,
        scope: &Scope,
    ) -> Result<Option<f64>, String> {
        let t = self.subst_sym(&t, vals, scope);
        match value_of(&t, vals, self.units) {
            Ok(v) => Ok(Some(v)),
            Err(_) if self.sym.is_some() => {
                *keep = Some(t);
                Ok(None)
            }
            // the numbers in force written in, so what the build reads is geometry and nothing
            // of this scope's
            Err(_) if reads_geometry(&t, self.units) => {
                *keep = Some(super::values::substitute_in(&t, vals, self.units, &self.held.names));
                Ok(None)
            }
            Err(e) => Err(format!("`{t}`: {e}")),
        }
    }

    /// The copies of one block: `n` of them, each its body under the block's prefix, with the
    /// trailing joint stated between neighbours.  Where the block runs over a chain, `edges`
    /// holds each copy's edge — the link as the chain wrote it and the scope it was written in —
    /// and copy `k` gets an alias `<block>#<id>.<k>.<var>` for it, which is how `e` is found
    /// through the copy's prefix like any name, and passed on to an instance as any alias is.
    #[allow(clippy::too_many_arguments)]
    pub(super) fn copies(
        &mut self,
        b: &Block,
        st: &Stmt,
        scope: &Scope,
        path: &[PathStep],
        depth: usize,
        n: usize,
        edges: &[(&Name, &[Link])],
    ) {
        // a block's index and its edge binder are names of each copy, and one over a name the
        // enclosing scope already has would read two ways in one body (E002): said once per block
        let binders = b.binder.iter().chain(edges.iter().map(|(v, _)| *v));
        for name in binders {
            let taken = scope.vals.contains_key(&name.text)
                || scope.groups.contains(&name.text)
                || self.names.contains(&format!("{}{}", scope.prefix(), name.text));
            if taken && !self.diagnostics.iter().any(|d| d.code == Code::E002 && d.span == name.span) {
                self.err(
                    Code::E002,
                    name.span,
                    format!("`{}` is already a name here, and a block's binder may not shadow it", name.text),
                );
            }
        }
        if n == 0 {
            return;
        }
        if n > MAX_FLAT {
            self.err(Code::E103, b.span, format!("{n} copies is more than {MAX_FLAT}"));
            return;
        }
        let block_prefix = keys::block(scope.prefix(), st.id.0);
        // a ring: copies the representative's turns.  One inside another is refused (E022) and
        // expanded as the cycle it is besides, so what it makes is still there to be named; and
        // its index, read anywhere in its body, varies nothing (E015)
        let ring = match (b.kind, &b.about) {
            (BlockKind::Ring, Some(_)) if scope.block.ring.is_some() => {
                self.once(Code::E022, st.span, "a `ring` inside a `ring` is not supported: turn the \
                    inner one's copies about a centre the outer ring does not move, outside it");
                None
            }
            (BlockKind::Ring, Some(about)) => {
                let i = b.binder.as_ref().map_or("", |n| n.text.as_str());
                for &s in &b.index_reads {
                    self.once(Code::E015, s, format!("`{i}` is a `ring`'s index: every copy is the \
                        first turned, so there is nothing for it to vary — reach a neighbour by \
                        `next` or `prev`"));
                }
                let written = (block_prefix.clone(), about.clone(), scope.clone(), n, st.span);
                self.rings.written.push(written);
                Some(block_prefix.clone())
            }
            _ => None,
        };
        let mut ranges: Vec<(usize, usize)> = Vec::new();
        for k in 0..n {
            // each copy a frame of its own — its binder, then what its body defines — over the
            // numbers around the block, shared by every copy and never copied (#405)
            let mut sub = Vals::over(scope.vals.clone());
            if let Some(i) = &b.binder {
                sub.insert(i.text.clone(), Aff::num(k as f64));
            }
            for (var, links) in edges {
                let key = format!("{block_prefix}{k}.{}", var.text);
                match &links[k] {
                    Link::Edge(r, sc) => self.aliases.push((key, r.clone(), sc.clone())),
                    // a face of corners: its run from corner `k` to the next, drawn once under the
                    // face whatever walks it — the face's own geometry, read where the face is,
                    // hidden as the face's own closing lines are
                    Link::Run { from, to, scope: sc, key: run } => {
                        if self.names.insert(run.clone()) {
                            let at = Span::at(var.span.lo as usize);
                            let name = Name { text: run.clone(), span: at };
                            let hidden = crate::style::Classes::one("closure");
                            let line = Decl::line(crate::syntax::DeclName::Key(name), from.clone(),
                                                  to.clone(), hidden);
                            self.emit(StmtKind::Decl(line), st, sc, path);
                        }
                        let root = Scope::absolute(sc.access.clone());
                        self.aliases.push((key, Ref::dotted(run, var.span), root));
                    }
                }
            }
            let sc = Scope {
                owner: scope.owner.clone(),
                access: scope.access.clone(),
                typed: scope.typed.clone(),
                prefixes: std::iter::once(format!("{block_prefix}{k}."))
                    .chain(scope.prefixes.iter().cloned())
                    .collect(),
                // `next` and `prev` mean something only where the copies close
                closed: scope.closed,
                forbidden: scope.forbidden.clone(),
                groups: scope.groups.clone(),
                vals: std::rc::Rc::new(sub.clone()),
                sides: scope.sides.clone(),
                module: scope.module,
                block: InBlock {
                    cyc: b.kind.wraps().then(|| Cyc { prefix: block_prefix.clone(), k, n }),
                    ring: match &ring {
                        Some(prefix) => Some(super::Ring { prefix: prefix.clone(), turned: k > 0 }),
                        None => scope.block.ring.clone(),
                    },
                    // the prefix just built is the block's id, so every declaration
                    // below is a copy, however deep and through however many instances
                    copies: true,
                },
                stamps: scope.stamps.clone(),
                pred: PredWalk {
                    pass: scope.pred.pass.clone(),
                    written: None,
                    held: scope.pred.held,
                },
                reading: None,
            };
            let mut p2 = path.to_vec();
            p2.push(PathStep::Copy { block: st.id, index: k as u32 });
            let from = self.out.len();
            self.body(&b.body, &sc, &mut sub, &p2, depth + 1);
            // the trailing joint's relations, stated between this copy and the
            // next: every copy for a cycle (the wrap seals the loop),
            // all but the last for a repeat, whose final corner is simply not
            // stated (issue #38).  The joint is the *block's* statement, so it
            // gets the `cyc` a repeat's own body does not — a wrapping kind's
            // scope already carries it.
            if let Some(j) = &b.joint {
                if b.kind.wraps() {
                    self.body(&j.stmts, &sc, &mut sub, &p2, depth + 1);
                } else if k + 1 < n {
                    let sc2 = Scope {
                        block: InBlock {
                            cyc: Some(Cyc { prefix: block_prefix.clone(), k, n }),
                            ..sc.block.clone()
                        },
                        ..sc.clone()
                    };
                    self.body(&j.stmts, &sc2, &mut sub, &p2, depth + 1);
                }
                ranges.push((from, self.out.len()));
            }
        }
        if let Some(j) = &b.joint {
            self.weld(j, b.kind, &block_prefix, n, &ranges);
        }
    }

    /// How many statements the walk has made, counting those held aside while a deferred block
    /// is expanded into a vector of its own (`expand_pending`).
    pub(super) fn emitted(&self) -> usize {
        self.out.len() + self.aside.held
    }

    /// An expanded statement keeps the id of the statement it came from.
    ///
    /// It is **the same statement** — a `cycle` of thirty makes thirty things from one line of
    /// source, and the line is what a caret lands on, what a splice edits and what a span points
    /// at.  What tells the thirty apart is the `path`, which is already on every `Site`.
    ///
    /// Minting a fresh id per copy would make each look like a statement of its own, and every
    /// consumer that turns an id back into source — `Elaborated::retext`, `adopt`, `edit::remove`
    /// — would find nothing there.  The multiplicity that a fresh id used to hide is exactly what
    /// `commit_seeds` needs to see: an id reached more than once has no single pose to record.
    pub(super) fn emit(&mut self, kind: StmtKind, st: &Stmt, scope: &Scope, path: &[PathStep]) {
        let stmt = Stmt { id: st.id, kind, span: st.span, chained: st.chained };
        self.out.push((stmt, path.to_vec(), scope.clone()));
    }
}

impl Walk<'_> {
    /// A point drawn in further planes (`point in P, G`, §6.7) is drawn in each by a **twin**: a
    /// point of its own there, keyed `{point}@{k}` and called by the point's name, seeded where
    /// the point is (`hint(at: point)`, read in space and seen in the twin's plane).  The
    /// elaborator ties each to the point in space (`Expansion::twins`).  `local` is the point's
    /// name as its scope reads it.
    fn twins_of(&mut self, d: &Decl, local: &str) -> Vec<Decl> {
        let point = d.name.key().text.clone();
        let at = Span::at(d.name.span().lo as usize);
        d.membership.also().iter().enumerate().map(|(k, plane)| {
            self.twin(&point, format!("{point}@{}", k + 1), at, Ref::new(local), plane.clone())
        }).collect()
    }

    /// The twin of `point` keyed `key`, drawn in `plane` and seeded at `seed` (the point as the
    /// scope it is declared in reads it), its name an empty span at `at` — recorded, so the
    /// elaborator ties it (`Expansion::twins`).
    fn twin(&mut self, point: &str, key: String, at: Span, seed: Ref, plane: Ref) -> Decl {
        self.names.insert(key.clone());
        self.twins.push((point.to_string(), key.clone()));
        let name = crate::syntax::DeclName::Key(Name { text: key, span: at });
        let mut twin = Decl::point(name, [0.0; 2], Some(crate::syntax::AtRef::at(seed)));
        twin.membership = crate::syntax::Membership::written_at(plane, at);
        twin
    }

    /// **A point drawn in a plane said to be on another is drawn in that one too** (§6.7,
    /// §6.21, #211): `gear.M coincident q`, where `gear.M` is drawn in `p` and `q` is a plane,
    /// is `gear.M` drawn in `q` by a twin, as `in p, q` would draw it — the statement made the
    /// twin's declaration in place of a row, so what is drawn in `q` reads the point there.  It
    /// is said here, once every declaration is walked, because the memberships the elaborator
    /// reads first must know the twin; a point that is drawn in no plane yet is left to
    /// `planes::incidences`, which draws it in the plane, and one already drawn in it to its row.
    /// The point is said first: `q coincident gear.M` puts the plane through the point, a row (a
    /// plane dropped through a point, `edit::add_datum`, may lie on its plane).  A claim, a
    /// derivative, a set's application and a ring's statement say a row as written.
    pub(super) fn drawn_incidences(&mut self, alias: &BTreeMap<String, String>) {
        use crate::model::EntKind;
        if self.sym.is_some() {
            return; // a traced body is solved for its point, never drawn
        }
        let units = self.units;
        // a plane or a point is never a face's name past a solid
        let solids = BTreeSet::new();
        let at = |r: &Ref, sc: &Scope, names: &BTreeSet<String>| {
            Lookup { names, solids: &solids, alias, units }.find(r, sc)
                .and_then(|(abs, rest)| rest.is_empty().then_some(abs))
        };
        // every declaration walked, by its absolute name, and the standard datums, walked after
        // the document's own (`finish`) but planes and a point it may name like any other
        let mut decls: Vec<(String, &Decl, Scope)> =
            self.out.iter().filter_map(|(st, _, sc)| match &st.kind {
                StmtKind::Decl(d) => Some((d.name.key().text.clone(), d, sc.clone())),
                _ => None,
            }).collect();
        if let Some((comp, scope)) = self.datums() {
            decls.extend(comp.body.iter().filter_map(|st| match &st.kind {
                StmtKind::Decl(d) => Some((format!("std.{}", d.name.key().text), d, scope.clone())),
                _ => None,
            }));
        }
        let planes: BTreeSet<String> = decls.iter()
            .filter(|(_, d, _)| d.kind == EntKind::Frame)
            .map(|(key, ..)| key.clone())
            .collect();
        // each point its declaration draws in a plane, with every plane it is drawn in, first
        // first.  One its instance's `in` puts in a view stands in space to the component that
        // declares it, which says where it stands whatever view the instance is put in: its
        // statements read as a point in space's (`planes::incidences`)
        let mut drawn: BTreeMap<String, Vec<String>> = BTreeMap::new();
        for (key, d, sc) in &decls {
            use crate::syntax::Source;
            if d.kind != EntKind::Point || d.membership.source() == Source::Instance {
                continue;
            }
            let Some(first) = d.membership.plane().and_then(|r| at(r, sc, &self.names)) else {
                continue;
            };
            let also = d.membership.also().iter().filter_map(|r| at(r, sc, &self.names));
            drawn.insert(key.clone(), std::iter::once(first).chain(also).collect());
        }
        for i in 0..self.out.len() {
            let (st, _, sc) = &self.out[i];
            let StmtKind::Relation(r) = &st.kind else { continue };
            if r.claim || r.initially || r.reads_rate() || r.along.is_some() || r.space
                || sc.block.ring.is_some() {
                continue;
            }
            let Some(w) = r.form.written() else { continue };
            let [rp, rq] = &w.ops[..] else { continue };
            if w.word.text != "coincident" || !w.args.is_empty() {
                continue;
            }
            // the point first: `P coincident p` puts the plane through the point, by a row
            let (Some(point), Some(plane)) = (at(rp, sc, &self.names), at(rq, sc, &self.names))
            else {
                continue;
            };
            let (rp, rq) = (rp.clone(), rq.clone());
            let Some(images) = drawn.get_mut(&point) else { continue };
            if !planes.contains(&plane) || images.contains(&plane) {
                continue;
            }
            // past the twins `in` made, and any a name already holds
            let mut k = images.len();
            while self.names.contains(&format!("{point}@{k}")) {
                k += 1;
            }
            images.push(plane);
            let span = Span::at(rp.span.lo as usize);
            let twin = self.twin(&point, format!("{point}@{k}"), span, rp, rq);
            self.out[i].0.kind = StmtKind::Decl(twin);
        }
    }
}

impl Walk<'_> {
    /// **A region as a solid** (§6.21): `blank := solid(R)`, its one operand a set, is the points
    /// inside it — the solid made `Sweep::Region` over a hidden point keyed `{solid}#probe`
    /// (held, in space), and `probe inside R` stated beside it, so the set's body is applied to
    /// the probe as any use applies it and the bounds it puts there are the solid's terms
    /// (`constrain`).
    /// `None` for any other declaration.
    /// `set` is the solid's set (`region_set`).
    fn region_of(&mut self, d: &mut Decl, local: &str, set: Option<Ref>)
        -> Option<(Decl, crate::syntax::Relation)> {
        use crate::syntax::Sweep;
        let set = set?;
        let at = Span::at(d.name.span().lo as usize);
        let key = keys::probe(&d.name.key().text);
        self.names.insert(key.clone());
        self.sets.probes.push(key.clone());
        let probe = keys::probe(local);
        d.children.clear();
        d.sweep = Some(Sweep::Region { probe: Ref::new(&probe) });
        Some((super::sets::probe(key, at), applied(&probe, set, "inside", at)))
    }

    /// **A set's envelope** (#283): `flank := envelope(S, under: m, …)`, its first operand a set,
    /// is the envelope of `S`'s surface — the operand made a hidden point keyed `{flank}#probe`
    /// (held, in space) and `probe coincident S` stated beside it, so the set's body is applied to
    /// the probe as any use applies it, and what it states there is the surface the envelope reads
    /// (a cone's angle, a cylinder's distance: `program::envelopes`), never a row.  A meshing's
    /// flank that is a set is read the same way, its two keyed `{contact}#first#probe` and
    /// `{contact}#second#probe`.  Empty for any other declaration.
    fn envelope_of(&mut self, d: &mut Decl, local: &str, sc: &Scope)
        -> Vec<(Decl, crate::syntax::Relation)> {
        let slots: &[(usize, &str)] = match d.kind {
            crate::model::EntKind::Envelope => &[(0, "")],
            crate::model::EntKind::Meshing => &[(0, "#first"), (1, "#second")],
            _ => return Vec::new(),
        };
        let mut made = Vec::new();
        for &(i, which) in slots {
            let Some(set) = self.set_operand(d, i, sc) else { continue };
            let at = Span::at(d.name.span().lo as usize);
            let key = keys::probe(&format!("{}{which}", d.name.key().text));
            self.names.insert(key.clone());
            let probe = keys::probe(&format!("{local}{which}"));
            d.children[i] = vec![Kid::Ref(Ref::new(&probe))];
            made.push((super::sets::probe(key, at), applied(&probe, set, "coincident", at)));
        }
        made
    }

    /// The set a region solid (`taken := solid(run)`) is of, where it is one (`set_named`).
    fn region_set(&mut self, d: &Decl, sc: &Scope) -> Option<(Ref, String, bool)> {
        use crate::syntax::Sweep;
        if d.kind != crate::model::EntKind::Solid || !matches!(d.sweep, Some(Sweep::Body)) {
            return None;
        }
        self.set_named(d, 0, sc)
    }

    /// The set a declaration's `i`th operand names, where it names one.
    fn set_operand(&mut self, d: &Decl, i: usize, sc: &Scope) -> Option<Ref> {
        self.set_named(d, i, sc).map(|(set, _, _)| set)
    }

    /// `set_operand`, with the set's absolute name and whether the name was found as an instance
    /// that is one made before (`same`, #415), the absolute name then that one's.
    pub(super) fn set_named(&mut self, d: &Decl, i: usize, sc: &Scope)
        -> Option<(Ref, String, bool)> {
        let [Kid::Ref(set)] = d.children.get(i).map(Vec::as_slice).unwrap_or(&[]) else {
            return None;
        };
        // a plain name declared in scope, as a family's instance is (`taken := solid(run)`), is
        // read off the names without the alias table, which a region asked for per solid would
        // build afresh each time (`Lookup::raw`'s order: each prefix, then the bare name) — and
        // so is one that is an instance made before
        let plain = set.path.is_empty() && !matches!(set.root.text.as_str(), "next" | "prev");
        let candidates = sc.prefixes.iter().map(|p| format!("{p}{}", set.root.text))
            .chain((!sc.closed).then(|| set.root.text.clone()));
        let first = candidates.filter(|_| plain).find(|abs| {
            self.names.contains(abs) || self.same.sets.contains_key(abs)
                || self.aliases.iter().any(|(k, _, _)| k == abs)
        });
        let (found, through) = match first {
            Some(abs) if self.names.contains(&abs) => (Some((abs, Vec::new())), false),
            Some(abs) if self.same.sets.contains_key(&abs) => {
                (Some((self.same.sets[&abs].clone(), Vec::new())), true)
            }
            _ => {
                let alias = self.alias_table();
                (self.look(&alias).raw(set, sc), false)
            }
        };
        let Some((abs, rest)) = found else { return None };
        if !rest.is_empty() || !self.sets.made.contains_key(&abs) {
            return None;
        }
        Some((set.clone(), abs, through))
    }
}

/// What a `when` asks of names that are not numbers (#267), answered by the walk as it stands
/// where the block is met: the aliases made so far — a face a copy runs over among them — read
/// only for the names a condition asks of (`resolve::Lazy`).
struct Asked<'w, 'a> {
    walk: &'w Walk<'a>,
    scope: &'w Scope,
    alias: super::resolve::Lazy<'w, 'a>,
}

/// The most aliases a walk may have made for `Asked` to hold each answer to the walk's whole table
/// under debug assertions: the documents of the suite, but not a body rounded all over, which the
/// table per question would make quadratic again.
const CHECKED: usize = 512;

impl<'w, 'a> Asked<'w, 'a> {
    fn new(walk: &'w Walk<'a>, scope: &'w Scope) -> Self {
        Asked { walk, scope, alias: super::resolve::Lazy::new(walk) }
    }

    /// `ask` of `name`, through the aliases read lazily.
    fn ask<T: PartialEq + std::fmt::Debug>(&self, name: &str,
        ask: impl Fn(&Ref, &dyn super::resolve::Aliases) -> T) -> T {
        let r = Ref::dotted(name, Span::default());
        let found = ask(&r, &self.alias);
        if cfg!(debug_assertions) && self.walk.aliases.len() <= CHECKED {
            assert_eq!(found, ask(&r, &self.walk.alias_table()), "`{name}`");
        }
        found
    }
}

impl super::values::Names for Asked<'_, '_> {
    fn stands(&self, name: &str) -> Option<String> {
        self.ask(name, |r, alias| self.walk.stands_for(r, self.scope, alias))
    }

    fn faces(&self, name: &str) -> Option<Vec<String>> {
        self.ask(name, |r, alias| self.walk.faces_named(r, self.scope, alias))
    }

    fn running(&self, name: &str) -> bool {
        self.walk.held.names.contains_key(name)
    }
}

/// `probe word set`, stated by the walk at `at`: how a probe has its set applied to it.
fn applied(probe: &str, set: Ref, word: &str, at: Span) -> crate::syntax::Relation {
    let written = crate::syntax::Written {
        word: Name { text: word.to_string(), span: set.span },
        fixity: crate::constraints::Fixity::Infix,
        ops: vec![Ref::new(probe), set],
        args: Vec::new(),
        span: at,
        rates: Vec::new(),
    };
    crate::syntax::Relation::of(crate::syntax::RelationForm::Written(written))
}

/// The numbers in force as a seed reads them (`settle_seeds`): a value affine in an unknown of
/// the drawing at where that unknown starts — its own seed, or where a face family's station
/// stands, else zero — and each running number where its station stands.  Asked name by name
/// and never built: a table of them per declaration is a copy of the root's per declaration,
/// quadratic in the assembly as soon as one number in it is free (#405).
struct Seeded<'v> {
    vals: &'v Vals,
    unknowns: &'v BTreeMap<String, crate::model::Declared>,
    held: &'v BTreeMap<String, (crate::units::Dim, f64)>,
}

impl Seeded<'_> {
    fn seeded(&self, value: &Aff) -> Aff {
        let start = |n: &String| {
            self.unknowns.get(n).and_then(|d| d.seed).or_else(|| self.held.get(n).map(|&(_, v)| v))
        };
        let seed = value.free.as_ref().and_then(start);
        Aff::of_dim(value.at(seed.unwrap_or(0.0)), value.dim)
    }
}

impl expr::Env for Seeded<'_> {
    fn value(&self, name: &str) -> Option<Aff> {
        match self.held.get(name) {
            Some(&(dim, v)) => Some(Aff::of_dim(v, dim)),
            None => self.vals.get(name).map(|v| self.seeded(v)),
        }
    }
}

impl Table for Seeded<'_> {
    fn has(&self, name: &str) -> bool {
        self.held.contains_key(name) || self.vals.contains_key(name)
    }

    /// Every value here is a number.
    fn binds(&self, _unknown: &str) -> bool {
        false
    }

    fn under(&self, prefix: &str) -> BTreeMap<String, Aff> {
        let mut out: BTreeMap<String, Aff> =
            self.vals.under(prefix).into_iter().map(|(k, v)| (k, self.seeded(&v))).collect();
        let held = super::table::prefixed(self.held, prefix);
        out.extend(held.map(|(k, &(dim, v))| (k.clone(), Aff::of_dim(v, dim))));
        out
    }
}
