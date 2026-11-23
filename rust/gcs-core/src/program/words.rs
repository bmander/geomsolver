//! **Relation words and imports, asked of the text** (§9.9, §14.4 [0.48]): every definition in
//! every file, used or not, and every name a `use` imports bare.  What a word *means* where it is
//! written is the flattener's (`flatten::words`); what is wrong with how it or an import is
//! written is said here, once, where it is written.

use super::{Code, Diag};
use crate::syntax::{Program, StmtKind};
use crate::syntax::Span;
use std::collections::{BTreeMap, BTreeSet};

/// The names a file defines at its top level — its components, relation words, interfaces,
/// values, groups, instances and declarations — which an import may not shadow.
fn own_names(p: &Program, file: Option<usize>) -> BTreeSet<&str> {
    let root = match file {
        None => &p.root().body,
        Some(k) => &p.modules[k].root.body,
    };
    let components = p.components.iter().filter(|c| c.module == file).filter_map(|c| c.name.as_ref());
    let words = p.words.iter().filter(|d| d.module == file).map(|d| &d.word);
    let interfaces = p.interfaces.iter().filter(|f| f.module == file).map(|f| &f.name);
    let stmts = root.iter().filter_map(|st| st.kind.bound_name());
    components.chain(words).chain(interfaces).chain(stmts).map(|n| n.text.as_str()).collect()
}

/// What module `k` defines as `name` for a file to import: `Some(true)` a relation word,
/// `Some(false)` a component, an interface or a top-level value or group (a module's own drawing is not
/// exported, §14.4), `None` nothing.
pub(crate) fn defines(p: &Program, k: usize, name: &str) -> Option<bool> {
    if p.words.iter().any(|d| d.module == Some(k) && d.word.text == name) {
        return Some(true);
    }
    let other = p.component_in(Some(k), name).is_some()
        || p.interface_in(Some(k), name).is_some()
        || p.modules[k].root.body.iter().any(|st| match &st.kind {
            StmtKind::Param(d) => d.name.text == name,
            StmtKind::Group(g) => g.name.text == name,
            _ => false,
        });
    other.then_some(false)
}

pub(super) fn check(p: &Program, diags: &mut Vec<Diag>) {
    let mut said: Vec<Diag> = Vec::new();
    let mut say = |code: Code, span, message: String| {
        said.push(Diag::new(code, span, None, message));
    };
    // every definition: what is wrong with it as written, and one defined twice in one file
    let mut first: BTreeMap<(Option<usize>, &str, bool), Span> = BTreeMap::new();
    // each file's numbers, read once for every word it defines
    let mut numbers: BTreeMap<Option<usize>, BTreeSet<String>> = BTreeMap::new();
    for d in &p.words {
        let key = (d.module, d.word.text.as_str(), d.fixity == crate::constraints::Fixity::Infix);
        if let Some(&was) = first.get(&key) {
            let again = super::defined_again(p, &d.word, was);
            say(again.code, again.span, again.message);
            continue;
        }
        first.insert(key, d.word.span);
        let numbers = numbers.entry(d.module)
            .or_insert_with(|| crate::flatten::file_number_names(p, d.module));
        for (code, span, message) in crate::flatten::word_faults(d, numbers) {
            say(code, span, message);
        }
    }
    given(p, &mut say);
    // every import, in every file
    let files = std::iter::once(None).chain((0..p.modules.len()).map(Some));
    for file in files {
        let uses = p.use_stmts(file);
        if uses.iter().all(|u| u.names.is_empty()) {
            continue;
        }
        let own = own_names(p, file);
        let mut from: BTreeMap<&str, &str> = BTreeMap::new();
        for u in uses {
            let Some(k) = p.module_named(&u.name) else { continue };
            for n in &u.names {
                let name = n.text.as_str();
                let Some(word) = defines(p, k, name) else {
                    say(Code::E101, n.span, match crate::library::home(&u.name, name) {
                        // `use std (horizontal)`, from before the library's modules
                        Some(home) => format!("`{}` defines no `{name}`: `{home}` does, \
                                               `use {home} ({name})`", u.name),
                        None => format!("`{}` defines no `{name}`", u.name),
                    });
                    continue;
                };
                // a relation word's collisions are its definition's to say (`word_faults`, of
                // its fixity: the library's infix `horizontal` is no prefix word of the core's)
                let builtin = crate::expr::builtin(name).is_some()
                    || crate::constraints::is_operator(name)
                    || crate::syntax::reserved_word(name);
                if builtin && !word {
                    say(Code::E071, n.span, format!(
                        "`{name}` is a word of the language's own, so `{}`'s is read by its \
                         path, `{}.{name}`",
                        u.name, u.name
                    ));
                    continue;
                }
                match from.get(name) {
                    Some(&m) if m == u.name => {
                        say(Code::E071, n.span, format!("`{name}` is imported twice"));
                        continue;
                    }
                    Some(&m) => {
                        say(Code::E071, n.span, format!(
                            "`{name}` is imported from `{m}` and from `{}`: read one by its path",
                            u.name
                        ));
                        continue;
                    }
                    None => {}
                }
                from.insert(name, u.name.as_str());
                if own.contains(name) {
                    say(Code::E071, n.span, format!(
                        "`{name}` is defined in this file, and imported from `{}`: read the \
                         module's by its path, `{}.{name}`",
                        u.name, u.name
                    ));
                }
            }
        }
    }
    // a module's are shown at the `use` that brought it in, as every diagnostic inside one is
    crate::modules::localize(p, &mut said);
    said.sort_by_key(|d| d.span.lo);
    diags.append(&mut said);
}

/// **The given modules, asked of the text** (#993 stage 2): a file the kernel refuses said once,
/// at the `use` that brought it in (E072), the module's name in front of the kernel's words — or
/// of how many solids it gives, a given body being one; and a file using one whose path it
/// defines itself at its top level (E071), which would read two things by one name.
fn given(p: &Program, say: &mut impl FnMut(Code, Span, String)) {
    // a program that uses no given module asks nothing more: no file's `use`s walked, no file's
    // own names read
    if p.modules.iter().all(|m| m.given.is_none()) {
        return;
    }
    for m in &p.modules {
        let Some(g) = &m.given else { continue };
        if let Err(why) = &g.body {
            say(Code::E072, m.via, format!("`{}`: {why}", m.name));
        }
    }
    let files = std::iter::once(None).chain((0..p.modules.len()).map(Some));
    for file in files {
        let own = own_names(p, file);
        for u in p.use_stmts(file) {
            let given = p.module_named(&u.name).is_some_and(|k| p.modules[k].given.is_some());
            let head = u.name.split('.').next().unwrap_or_default();
            if given && own.contains(head) {
                say(Code::E071, u.span, format!(
                    "`{head}` is defined in this file, and `use {}` names the body its file \
                     gives: one name, two things", u.name));
            }
        }
    }
}
