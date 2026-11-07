//! **Parts** (#461, #562, `docs/parts-plan.md` §3, §6): every braced definition is a function of
//! its arguments and of the frame its instance stands in, and the elaborator holds it to that —
//! a *part* is any drawn instance of one.  Each is read as the classifier reads one
//! (`boundary::split`): what it made and the rows its body states, its inputs — every other
//! column those rows read, the frame it stands in among them — held.  Then, over the structure of
//! those rows alone, as the diagnosis reads a document's (Dulmage–Mendelsohn):
//!
//! * **PT2, its own unknowns are its inputs' to determine.**  A maximum matching of its rows on
//!   its own unknowns covers every unknown.  One left over is **E108** at the definition, naming
//!   what it moves: the parent would have to place it by reaching inside, and a definition's
//!   freedom is a formal instead, which the parent binds or leaves unknown (FR2).
//! * **PT3, no fact about its operands alone.**  A row reading none of its own unknowns, or one
//!   past what they take (the matching's over-determined rows), is **E109** at the row: a verb
//!   fused to a noun, which belongs where the operands are given, or in a relation word (FR1).
//!   A numeric formal the call left unbound was given by nothing, so a row may determine it: it
//!   is a column here, and one no row reaches is the freedom exported to the parent.
//!
//! The structure, and not the numbers: deterministic and decided before any solve.  A claim, a
//! claim block's witness and a bound state no row and are not counted.  Nothing here changes a
//! row: a part is flattened and solved as any instance is.  Each fault is said once for its
//! definition, against the first instance it was found in.
//!
//! **The diagnosis, read through the parts** (§6, `read`): what the flat solve's diagnosis says,
//! said again in a part's terms where a part is in it, from the flat solve alone.
//!
//! * **D1, an over-constraint through a part's outputs is the parent's.**  A conflict holding a
//!   part's rows and a parent's row over what the part makes names the parent's row as the one to
//!   remove: the part's rows are why, never the culprit, since they place what it makes from what
//!   it is given.
//! * **D2, a part that does not hold is named, with its inputs.**  Where the solve stopped with a
//!   part's rows unmet, the innermost such part is named with its inputs where they stood.
//! * **D3, a part singular at its inputs.**  Its rows place every unknown it makes structurally
//!   (PT2), and at the solved pose they may still not: that is said, with the rank.
//!
//! **PT5, its root is stated** (`roots`, asked by `solventc --lint`): a part whose rows, from
//! scattered starts, come to another pose its stated `ccw`/`cw` admit leaves which it takes to
//! its seeds — **W207**, advisory, since a root not found is not shown absent.

use crate::boundary::{columns, count, list, split, World};
use crate::constraints::{CKind, Constraint, SpecKind};
use crate::diagnose::Diagnosis;
use crate::graph::dulmage_mendelsohn;
use crate::json::{self, Json};
use crate::model::{entity_name, EntKind, EntRef, Sketch};
use crate::program::{Code, Diag, Elaborated, Instanced, Site};
use crate::syntax::StmtId;
use crate::system::System;
use std::collections::{BTreeMap, BTreeSet};

/// How many of the unknowns left free, or of the rows, a message names before it counts the rest.
const NAMED: usize = 6;

/// One drawn instance of a definition, read: its rows and unknowns set apart from the document.
struct Read<'e> {
    inst: &'e Instanced,
    /// Its own unknowns: what it made, the numbers no entity owns that its rows read, and the free
    /// variables under its name that are no formal of its own (an inner instance's, unbound).
    own: BTreeSet<u32>,
    /// Its acting rows, by constraint id.
    rows: BTreeSet<u32>,
    /// What else its rows read: what it is given.
    inputs: BTreeSet<u32>,
    /// Its numeric formals the call left unbound, the free variables under its name that are
    /// formals of its own: nothing gave them, so its rows may determine them, and a freedom left
    /// in them is exported to the parent (FR2), not left free.
    unbound: BTreeSet<u32>,
}

/// Every drawn instance of a braced definition in `sk`, read, with the reading of the document they
/// share — or `None` where the document draws none.
fn instances<'e>(e: &'e Elaborated, sk: &Sketch) -> Option<(World, Vec<Read<'e>>)> {
    let prog = &e.program;
    let mut drawn = drawn(e).peekable();
    drawn.peek()?;
    let world = World::of(sk, &e.map);
    let by_id: BTreeMap<u32, &Constraint> = sk.constraints.iter().map(|c| (c.id, c)).collect();
    let zero = sk.zero;
    let read = drawn.map(|inst| {
        let comp = &prog.components[inst.comp];
        let s = split(&e.map, sk, &world, inst);
        let rows: BTreeSet<u32> = s.kept.iter().copied()
            .filter(|id| by_id.get(id).is_some_and(|c| c.acts()))
            .collect();
        let formals: BTreeSet<String> =
            comp.formals.iter().map(|f| format!("{}{}", inst.prefix, f.name.text)).collect();
        let mut own = s.own;
        let mut unbound = BTreeSet::new();
        for (k, &p) in sk.free_vars.range(inst.prefix.clone()..)
            .take_while(|(k, _)| k.starts_with(&inst.prefix)) {
            match formals.contains(k) {
                true => unbound.insert(p),
                false => own.insert(p),
            };
        }
        let inputs = rows.iter().filter_map(|id| by_id.get(id))
            .flat_map(|c| columns(sk, c))
            .filter(|p| !own.contains(p) && Some(*p) != zero)
            .collect();
        Read { inst, own, rows, inputs, unbound }
    }).collect();
    Some((world, read))
}

/// Every drawn instance of a braced definition in `e`: its parts.  A development (#645) is a
/// figure the core lays, its three freedoms the parent's to place.
fn drawn(e: &Elaborated) -> impl Iterator<Item = &Instanced> {
    let prog = &e.program;
    e.map.instances.iter().filter(move |i| i.drawn && prog.components[i.comp].braced()
        && crate::flatten::developable::development_body(&prog.components[i.comp]).is_none())
}

/// What the document calls entity `r`.
fn ent_name(e: &Elaborated, r: EntRef) -> String {
    e.map.name_of(r).cloned().unwrap_or_else(|| entity_name(r))
}

/// The entity that owns parameter `p`, by the document's name, or the free variable it is.
fn owner_name(e: &Elaborated, sk: &Sketch, world: &World, p: u32) -> Option<String> {
    if let Some(&r) = world.owner.get(&p) {
        return Some(ent_name(e, r));
    }
    sk.free_vars.iter().find(|(_, &q)| q == p).map(|(k, _)| k.clone())
}

/// `items` as a list, the first `NAMED` of them and how many more.
fn listed(items: Vec<String>) -> String {
    list(&items, NAMED)
}

/// What is wrong with each part the document instantiates, as diagnostics (PT2, PT3).  A stage
/// of the run (#851): said to whoever listens.
pub fn check(e: &Elaborated) -> Vec<Diag> {
    let checking = || crate::progress::Work::check(drawn(e).count());
    crate::progress::during(checking, || check_parts(e))
}

/// `check`, unsaid.
fn check_parts(e: &Elaborated) -> Vec<Diag> {
    let prog = &e.program;
    let sk = &e.sketch;
    let map = &e.map;
    let Some((world, read)) = instances(e, sk) else { return Vec::new() };
    // the document's structure, compiled once: each part's is its rows' over its own columns
    let sys = System::new(sk);
    let (all, all_c) = sys.structure();
    let mut rows_of: BTreeMap<u32, Vec<usize>> = BTreeMap::new();
    for (k, &id) in all_c.iter().enumerate() {
        rows_of.entry(id).or_default().push(k);
    }
    let mut out: Vec<Diag> = Vec::new();
    // each fault once a definition: E108 at its name, E109 at a statement or, sited nowhere, by
    // its text
    let mut free_said: BTreeSet<usize> = BTreeSet::new();
    let mut said: BTreeSet<(usize, Result<StmtId, String>)> = BTreeSet::new();
    // a frame's own datums, which a body reads as `std.origin`, are bound under no name it says:
    // read off the keys once, where a fault names one
    let datums: std::cell::OnceCell<BTreeMap<EntRef, String>> = std::cell::OnceCell::new();
    // innermost first (#819): a fault inside a part is said there, and the rows its matching
    // leaves over are no rows of the part around it, which would count them again and say its
    // own rows past what its unknowns take for what is the inner part's to say
    let mut order: Vec<&Read> = read.iter().collect();
    order.sort_by_key(|r| std::cmp::Reverse(r.inst.path.0.len()));
    let mut spent: BTreeSet<usize> = BTreeSet::new();
    for r in order {
        let inst = r.inst;
        let comp = &prog.components[inst.comp];
        let word = prog.component_id(inst.comp);
        let at = comp.name.as_ref().map(|n| n.span).unwrap_or(comp.span);
        let instance = inst.prefix.trim_end_matches('.');
        // its rows over its own columns, numbered first, and the formals the call left unbound
        let col_set = |of: &BTreeSet<u32>| -> BTreeSet<usize> {
            of.iter().filter_map(|&p| usize::try_from(sys.col_of[p as usize]).ok()).collect()
        };
        let own_cols = col_set(&r.own);
        let mut cols: Vec<usize> = own_cols.iter().copied().collect();
        let n_own = cols.len();
        cols.extend(col_set(&r.unbound).difference(&own_cols));
        let local: BTreeMap<usize, usize> = cols.iter().enumerate().map(|(k, &c)| (c, k)).collect();
        // each row by its index in the document's structure, which names its constraint
        let row_k: Vec<usize> = r.rows.iter()
            .flat_map(|id| rows_of.get(id).into_iter().flatten().copied())
            .filter(|k| !spent.contains(k))
            .collect();
        let adj: Vec<Vec<usize>> = row_k.iter()
            .map(|&k| all[k].iter().filter_map(|c| local.get(c).copied()).collect())
            .collect();
        // PT3 is judged over its own unknowns and the formals the call left unbound, which its
        // rows may determine or leave to the parent, and PT2 over its own alone: the same matching
        // where the call left none
        let wide = dulmage_mendelsohn(&adj, cols.len());
        let unmatched: Vec<usize> = if n_own == cols.len() {
            (0..n_own).filter(|&c| wide.mate_col[c] < 0).collect()
        } else {
            let narrow: Vec<Vec<usize>> =
                adj.iter().map(|a| a.iter().copied().filter(|&c| c < n_own).collect()).collect();
            let dm = dulmage_mendelsohn(&narrow, n_own);
            (0..n_own).filter(|&c| dm.mate_col[c] < 0).collect()
        };
        // the rows it leaves over, spent for the parts around it, and those PT3 says: of no own
        // unknown, or past what they take
        let over: BTreeSet<usize> = wide.over_rows.into_iter().collect();
        let mut fused: Vec<u32> = Vec::new();
        for (k, cols) in adj.iter().enumerate() {
            if wide.mate_row[k] < 0 {
                spent.insert(row_k[k]);
            }
            let id = all_c[row_k[k]];
            if (cols.is_empty() || over.contains(&k)) && !fused.contains(&id) {
                fused.push(id);
            }
        }

        // PT2: an own unknown no row reaches, the matching's unmatched columns
        let mut free: Vec<String> = Vec::new();
        let mut left = 0usize;
        for col in unmatched {
            left += 1;
            let what = owner_name(e, sk, &world, sys.free[cols[col]] as u32)
                .map_or("a number its rows read".to_string(), |n| format!("`{n}`"));
            if !free.contains(&what) {
                free.push(what);
            }
        }
        if left > 0 && free_said.insert(inst.comp) {
            out.push(Diag::new(Code::E108, at, None, format!(
                "`{word}` leaves {left} of its own unknowns free, moving {} (in `{instance}`): a \
                 definition is a function of what it is given — state what places them inside \
                 it, or make the freedom a formal the parent binds or leaves unknown",
                listed(free))));
        }

        // PT3, each said once for its definition, at the statement it is: a statement's row at
        // the statement, in the names the definition's body reads (`normal.u`, not
        // `x.normal.u`), and a row the language built for a declaration (an axis's unit row, a
        // point's lift) at that declaration, where no statement written is among them
        let named = |x: EntRef| map.name_of(x)
            .map(|n| n.strip_prefix(inst.prefix.as_str()).unwrap_or(n).to_string())
            .or_else(|| datums.get_or_init(|| map.keys()
                .filter_map(|(k, x)| {
                    crate::flatten::frame_datum(k).map(|(_, d)| (x, format!("std.{d}")))
                })
                .collect()).get(&x).cloned());
        let text = |c: &Constraint| crate::io::describe_with(c, &named);
        let past = |verb: &str, text: &str, beside: &str| format!(
            "`{word}` {verb} `{text}` past what its own unknowns take (in `{instance}`){beside}: a \
             definition's rows determine what it makes, and one more says something of what it \
             is given");
        let fused: Vec<&Constraint> =
            fused.into_iter().filter_map(|id| sk.constraint(id)).collect();
        let mut written: Vec<(&Constraint, &Site)> = Vec::new();
        let (mut built, mut lifts): (Vec<&Constraint>, Vec<&Constraint>) = (Vec::new(), Vec::new());
        for c in fused {
            match map.site_of_constraint(c.id) {
                Some(site) => written.push((c, site)),
                None if matches!(c.kind, CKind::Lift | CKind::ChartLift) => lifts.push(c),
                None => built.push(c),
            }
        }
        // each declaration a row the language built is the row of, by what the row places, once
        // a declaration: its own entity (`normal`, for its axis's unit row) and its site.  A
        // point's lift, which only stands it in its plane, names its point where nothing else
        // the language built is among them
        if built.is_empty() {
            built = lifts;
        }
        let mut decls: Vec<(EntRef, &Site)> = Vec::new();
        let mut loose: Vec<&Constraint> = Vec::new();
        for c in built {
            let site = crate::boundary::subject(c).into_iter().chain(c.entities())
                .find_map(|x| map.site_of(x).filter(|s| inst.contains(&s.path)));
            match site.and_then(|s| map.ents_made_by(s.stmt).next().map(|own| (own, s))) {
                Some((own, s)) if !decls.iter().any(|(_, d)| d.stmt == s.stmt) => {
                    decls.push((own, s))
                }
                Some(_) => {}
                None => loose.push(c),
            }
        }
        let name = |x: EntRef| named(x).unwrap_or_else(|| entity_name(x));
        if !written.is_empty() {
            // beside what those build
            let beside = match &decls[..] {
                [] => String::new(),
                [(x, _)] => format!(", beside what `{}`'s declaration builds", name(*x)),
                more => format!(", beside what the declarations of {} build",
                    listed(more.iter().map(|(x, _)| format!("`{}`", name(*x))).collect())),
            };
            let mine = |p: &u32| r.own.contains(p) || r.unbound.contains(p);
            for (c, site) in written {
                if !said.insert((inst.comp, Ok(site.stmt))) {
                    continue;
                }
                let message = match columns(sk, c).iter().any(mine) {
                    false => format!(
                        "`{word}` states `{}`, a fact about what it is given and not about \
                         anything it makes (in `{instance}`): state it where those are given, or \
                         as a relation word", text(c)),
                    true => past("states", &text(c), &beside),
                };
                out.push(Diag::new(Code::E109, site.span, Some(site.stmt), message));
            }
            continue;
        }
        // no statement among them: the declaration builds what places it twice
        for (x, site) in decls {
            if !said.insert((inst.comp, Ok(site.stmt))) {
                continue;
            }
            let how = match x.kind {
                EntKind::Frame => " — a plane over axes of its own, `plane(u: hint(dir: l))`, \
                    each held to what it runs along, `l parallel(sense: same) P.u`",
                _ => "",
            };
            out.push(Diag::new(Code::E109, site.span, Some(site.stmt), format!(
                "`{word}` builds `{}` over what places it more than once (in `{instance}`): \
                 what it is built over places it already, so building it says something of what \
                 it is given — that its operands meet, or lie square.  Build it over what places \
                 it once{how}", name(x))));
        }
        for c in loose {
            let text = text(c);
            if said.insert((inst.comp, Err(text.clone()))) {
                out.push(Diag::new(Code::E109, at, None, past("builds", &text, "")));
            }
        }
    }
    out
}

/// What the diagnosis says of a drawing's parts (D1–D3), in the core's words.
#[derive(Clone, Debug, Default)]
pub struct Reading {
    /// How many part instances the drawing has, and of how many definitions.
    pub instances: usize,
    pub definitions: usize,
    /// D1: each parent's row that over-constrains a part, said as the one to remove.
    pub over: Vec<String>,
    /// D2: each innermost part whose rows the solve left unmet, with its inputs where they stood.
    pub unmet: Vec<String>,
    /// D3: each part its rows do not place at its inputs as they stand, with the rank.
    pub singular: Vec<String>,
}

/// The diagnosis `d` of the solved sketch `sk`, read through `e`'s parts.  `settled` says whether
/// the solve settled: a part is judged singular only at a pose that is one.  A stage of the run
/// (#851), and each part's rows ranked an object in it, by its definition: said to whoever
/// listens, so a reading that takes long says where.
pub fn read(e: &Elaborated, sk: &Sketch, d: &Diagnosis, settled: bool) -> Reading {
    let reading = || crate::progress::Work::parts(drawn(e).count());
    crate::progress::during(reading, || read_parts(e, sk, d, settled))
}

/// `read`, unsaid.
fn read_parts(e: &Elaborated, sk: &Sketch, d: &Diagnosis, settled: bool) -> Reading {
    let prog = &e.program;
    let Some((world, read)) = instances(e, sk) else { return Reading::default() };
    let mut out = Reading { instances: read.len(), ..Reading::default() };
    out.definitions = read.iter().map(|r| r.inst.comp).collect::<BTreeSet<_>>().len();
    let name_of = |r: EntRef| e.map.name_of(r).cloned();
    let text = |id: u32| {
        sk.constraint(id).map(|c| format!("`{}`", crate::io::describe_with(c, &name_of)))
    };
    let label = |r: &Read| format!("part `{}` (`{}`)", r.inst.prefix.trim_end_matches('.'),
        prog.component_id(r.inst.comp));
    let inputs = |r: &Read| -> Vec<String> {
        let mut names: Vec<String> = Vec::new();
        for &p in &r.inputs {
            if let Some(n) = owner_name(e, sk, &world, p).map(|n| format!("`{n}`")) {
                if !names.contains(&n) {
                    names.push(n);
                }
            }
        }
        names
    };

    // D1: a conflict holding a part's rows and a row of the parent's over what it makes
    let conflict: Vec<u32> = d.conflicts.clone().unwrap_or_else(|| d.over.clone());
    let mut explained: BTreeSet<&str> = BTreeSet::new();
    for r in &read {
        let inside: Vec<u32> = conflict.iter().copied().filter(|id| r.rows.contains(id)).collect();
        if inside.is_empty() {
            continue;
        }
        let through: Vec<u32> = conflict.iter().copied()
            .filter(|id| !r.rows.contains(id))
            .filter(|&id| sk.constraint(id)
                .is_some_and(|c| columns(sk, c).iter().any(|p| r.own.contains(p))))
            .collect();
        for id in through {
            let Some(row) = text(id) else { continue };
            explained.insert(r.inst.prefix.as_str());
            out.over.push(format!(
                "{row} over-constrains {}: what it makes is placed by what it is given ({}), \
                 so remove this, not {}", label(r), listed(inputs(r)),
                listed(inside.iter().filter_map(|&k| text(k)).collect())));
        }
    }

    // D2: the innermost parts whose rows the solve left unmet, where no row of a parent's
    // over-constraining them is the reason already said
    let violated: BTreeSet<u32> = d.violated.iter().copied().collect();
    let unmet: Vec<&Read> = read.iter()
        .filter(|r| r.rows.iter().any(|id| violated.contains(id)))
        .filter(|r| !explained.contains(r.inst.prefix.as_str()))
        .collect();
    for r in &unmet {
        let nested = unmet.iter().any(|o| o.inst.path.0.len() > r.inst.path.0.len()
            && r.inst.contains(&o.inst.path));
        if nested {
            continue;
        }
        let rows: Vec<String> =
            r.rows.iter().filter(|id| violated.contains(id)).filter_map(|&id| text(id)).collect();
        let mut at: Vec<String> = Vec::new();
        let mut seen: BTreeSet<EntRef> = BTreeSet::new();
        for &p in &r.inputs {
            let Some(&ent) = world.owner.get(&p) else { continue };
            if !seen.insert(ent) {
                continue;
            }
            let n = ent_name(e, ent);
            let v = |x: f64| crate::expr::reading(SpecKind::Scalar, x);
            at.push(match ent.kind {
                EntKind::Point => {
                    let q = sk.world_point(ent.i());
                    format!("`{n}` at ({}, {}, {})", v(q[0]), v(q[1]), v(q[2]))
                }
                _ => format!("`{n}`"),
            });
        }
        out.unmet.push(format!("{} does not hold where the solve stopped: {}; what it is given \
            there: {}", label(r), listed(rows), listed(at)));
    }

    // D3: a part its rows do not place at its inputs as they stand — the document's Jacobian at
    // the pose, conditioned once and kept sparse, and each part's rows over its own columns
    // ranked from it part by part (#851): a dense rank of a part is the cube of everything it
    // makes, and a part that places forty others makes all of theirs
    if settled {
        let mut sys = System::new(sk);
        let z = sys.z0(sk);
        let hard = sys.hard_rows();
        let jac = sys.conditioned_sparse(&z);
        let mut rows_of: BTreeMap<u32, Vec<usize>> = BTreeMap::new();
        for &row in &hard {
            let (b, i) = sys.instance_of(row);
            rows_of.entry(sys.blocks[b].cids[i]).or_default().push(row);
        }
        for r in &read {
            let cols: Vec<usize> = r.own.iter()
                .filter_map(|&p| usize::try_from(sys.col_of[p as usize]).ok())
                .collect::<BTreeSet<usize>>().into_iter().collect();
            let rows: Vec<usize> =
                r.rows.iter().flat_map(|id| rows_of.get(id).into_iter().flatten().copied()).collect();
            let n = cols.len();
            if n == 0 || rows.is_empty() {
                continue;
            }
            let ranking = || crate::progress::Work::rank(&prog.component_id(r.inst.comp));
            let rank = crate::progress::during(ranking,
                || crate::diagnose::rank_by_parts(&jac, &rows, &cols));
            if rank < n {
                out.singular.push(format!("{} is singular at what it is given: its rows place \
                    {rank} of its {n} own unknowns here, so a small change to its inputs may move \
                    it far or find no pose", label(r)));
            }
        }
    }
    out
}

/// `read`'s lines, as `solventc` prints them.
pub fn text(r: &Reading) -> Vec<String> {
    if r.instances == 0 {
        return Vec::new();
    }
    let mut out = vec![format!("parts: {} of {}", count(r.instances, "instance"),
        count(r.definitions, "definition"))];
    out.extend(r.over.iter().map(|l| format!("part conflict: {l}")));
    out.extend(r.unmet.iter().map(|l| format!("part unmet: {l}")));
    out.extend(r.singular.iter().map(|l| format!("part singular: {l}")));
    out
}

/// `read` as JSON, under `--json`'s `"parts"`.
pub fn json(r: &Reading) -> Json {
    let lines = |v: &[String]| Json::Arr(v.iter().map(|s| Json::Str(s.clone())).collect());
    json::object([
        ("instances", Json::Int(r.instances as i64)),
        ("definitions", Json::Int(r.definitions as i64)),
        ("over", lines(&r.over)),
        ("unmet", lines(&r.unmet)),
        ("singular", lines(&r.singular)),
    ])
}

/// **PT5: which root a part takes is stated** (W207, advisory): each part the document draws,
/// read through its first instance by the classifier's root search (`boundary::classify`), whose
/// scattered starts found another pose its stated choices admit.  Read at the solved pose, so
/// asked of a solved sketch, and only by `--lint`: a search is a heuristic and costs probe solves.
pub fn roots(e: &Elaborated, solved: &Sketch) -> Vec<Diag> {
    let prog = &e.program;
    if !prog.components.iter().any(|c| c.braced()) {
        return Vec::new();
    }
    let mut out = Vec::new();
    for c in crate::boundary::classify(e, solved) {
        let comp = &prog.components[c.comp];
        if !comp.braced() || c.class != crate::boundary::Class::LocallyDetermined {
            continue;
        }
        let Some(a) = &c.analysis else { continue };
        let Some(r) = &a.roots else { continue };
        let at = comp.name.as_ref().map(|n| n.span).unwrap_or(comp.span);
        out.push(Diag::new(Code::W207, at, None, format!(
            "`{}` leaves which root it takes to its seeds: from {} scattered starts its rows \
             came to {} poses its stated choices admit (in `{}`) — state the one it means, with \
             `ccw`/`cw`, a `sense:` or a bound", c.name, r.starts, r.admitted, a.instance)));
    }
    crate::modules::localize(prog, &mut out);
    out
}
