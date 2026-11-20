//! Writing a word of the lexicon (`lexicon.rs`): the statement its filled slots make, a name
//! minted for what it declares, the `use` it needs, appended and held to the elaborator.

use super::*;
use crate::lexicon::{self, Entry, Role};
use std::collections::BTreeMap;

/// The word `r` names and its slots' texts, the name it declares — given, or minted in its
/// kind's alphabet (`l0`), or else its word's (`region0`) — among them.
fn filled(prog: &Program, r: &WordRequest)
    -> Result<(Entry, BTreeMap<String, String>, Option<String>), String> {
    let e = lexicon::entry(Some(prog), &r.id).ok_or_else(|| format!("no word `{}`", r.id))?;
    let mut args = r.args.clone();
    let Some(slot) = e.slots.iter().find(|s| s.role == Role::Name) else {
        return Ok((e, args, None));
    };
    let given = args.get(&slot.key).map(|s| s.trim()).filter(|s| !s.is_empty());
    let stem = match e.makes.as_deref().and_then(EntKind::parse) {
        Some(kind) => syntax::kind_initial(kind).to_string(),
        None => lexicon::name_stem(&e),
    };
    let name = chosen(&mut taken_names(prog), given, &stem)?;
    args.insert(slot.key.clone(), name.clone());
    Ok((e, args, Some(name)))
}

/// **The statement a word would write**, as it would be written — its name minted — for a front
/// end to show while the slots are being filled.  Refused for a slot still empty.
pub fn word_text(prog: &Program, r: &WordRequest) -> Result<String, String> {
    let (e, args, _) = filled(prog, r)?;
    lexicon::statement(&e, &args, r.plane.as_deref())
}

/// **Write a word**: the statement its slots make, appended where `append_at` says, the module
/// it is read from brought in by a `use` (naming a relation word, which is only ever bare), and
/// the whole held to the elaborator — refused, in its words, where what was written is an error.
/// `names` is the name the statement gives, where it gives one.
pub fn write_word(prog: &Program, r: &WordRequest) -> Edit {
    let made = || -> Result<Edit, String> {
        let (e, args, name) = filled(prog, r)?;
        let stmt = lexicon::statement(&e, &args, r.plane.as_deref())?;
        let (at, lead) = append_at(prog);
        let splice = Splice { at, with: format!("{lead}{stmt}") };
        let names = name.into_iter().collect();
        match &e.uses {
            Some((module, word)) => {
                let words: Vec<&str> = word.iter().map(String::as_str).collect();
                checked_using(prog, splice, module, &words, names)
            }
            None => {
                let (text, landed) = splice_landing(prog.text(), vec![splice]);
                let ours = landed[0].ok_or("the end of the document is no place in it")?;
                checked(prog, text, ours, names)
            }
        }
    };
    Edit::refused_or(prog, made())
}
