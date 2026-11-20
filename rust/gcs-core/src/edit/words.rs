//! Writing a word of the lexicon (`lexicon.rs`): the statement its filled slots make, a name
//! minted for what it declares, the `use` it needs, appended and held to the elaborator.

use super::*;
use crate::json::Json;
use crate::lexicon::{self, Entry, Role};
use std::collections::BTreeMap;

/// A word and what fills its slots, as a front end sends it: `{id, args: {key: text}, plane?}` —
/// each slot's text by its key (a slot of several, its names joined by commas), and the plane a
/// drawn declaration is drawn in.
#[derive(Clone, Debug, Default)]
pub struct WordRequest {
    pub id: String,
    pub args: BTreeMap<String, String>,
    pub plane: Option<String>,
}

impl WordRequest {
    pub fn from_json(v: &Json) -> Result<WordRequest, String> {
        let id = v.get("id").map(|j| j.as_str().to_string()).filter(|s| !s.is_empty())
            .ok_or("a word to write names its `id`")?;
        let mut args = BTreeMap::new();
        if let Some(Json::Obj(kv)) = v.get("args") {
            for (k, j) in kv {
                let text = match j {
                    Json::Str(s) => s.clone(),
                    Json::Arr(xs) => xs.iter().map(|x| x.as_str().to_string())
                        .collect::<Vec<_>>().join(", "),
                    Json::Null => continue,
                    other => other.dump(None),
                };
                args.insert(k.clone(), text);
            }
        }
        let plane = v.get("plane").map(|j| j.as_str().to_string()).filter(|s| !s.is_empty());
        Ok(WordRequest { id, args, plane })
    }
}

/// A name nothing has taken, `stem` and the least number that makes it free.
fn fresh(prog: &Program, stem: &str) -> String {
    let taken = taken_names(prog);
    (0..).map(|i| format!("{stem}{i}")).find(|n| !taken.contains(n))
        .expect("an unbounded sequence always holds a fresh name")
}

/// The word `r` names and its slots' texts, the name it declares minted where none is given.
fn filled(prog: &Program, r: &WordRequest) -> Result<(Entry, BTreeMap<String, String>, Option<String>), String> {
    let e = lexicon::entry(Some(prog), &r.id).ok_or_else(|| format!("no word `{}`", r.id))?;
    let mut args = r.args.clone();
    let mut name = None;
    if let Some(slot) = e.slots.iter().find(|s| s.role == Role::Name) {
        let given = args.get(&slot.key).map(|s| s.trim().to_string()).filter(|s| !s.is_empty());
        let n = match given {
            Some(n) if !syntax::is_name(&n) => return Err(format!("`{n}` is not a name")),
            Some(n) if taken_names(prog).contains(&n) => {
                return Err(format!("`{n}` is already a name in this document"));
            }
            Some(n) => n,
            None => match e.makes.as_deref().and_then(EntKind::parse) {
                Some(kind) => mint(prog, kind),
                None => fresh(prog, &lexicon::name_stem(&e)),
            },
        };
        args.insert(slot.key.clone(), n.clone());
        name = Some(n);
    }
    Ok((e, args, name))
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
        let mut splices = vec![Splice { at, with: format!("{lead}{stmt}") }];
        if let Some((module, word)) = &e.uses {
            let words: Vec<&str> = word.iter().map(String::as_str).collect();
            splices.push(imported(prog, module, &words));
        }
        let (text, landed) = splice_landing(prog.text(), splices);
        let regions: Vec<Span> = landed.into_iter()
            .map(|l| l.ok_or("the edit is no place in the source"))
            .collect::<Result<_, _>>()?;
        checked_in(prog, text, &regions, name.into_iter().collect())
    };
    Edit::refused_or(prog, made())
}
