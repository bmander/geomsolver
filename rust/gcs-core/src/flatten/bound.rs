//! What a closed instance is a function of (#566): its definition, its arguments and the datum of
//! its frame its body is drawn in — a key a solid it makes carries into the sketch, where solids
//! of one key are one evaluation placed by each one's frame (`model::Binding`).

use super::*;

/// A closed instance's binding: the key every instance of the same definition given the same
/// numbers and drawn in the same datum of its frame shares, and the frame it stands in
/// (absolute, `None` for the world).
#[derive(Clone, Debug, PartialEq)]
pub struct Bound {
    pub key: String,
    pub frame: Option<String>,
}

impl<'a> Walk<'a> {
    /// Each drawn instance's binding, where it is closed: a definition (not a family of sets)
    /// whose formals are numbers, or groups carrying numbers alone, each given a constant, and
    /// whose call stands in a frame drawn in that frame or one of its datums.  Every other
    /// instance is its own binding, and evaluated as today.
    pub(super) fn bind_closed(&mut self, alias: &BTreeMap<String, String>) {
        // where each body is read: the scope of the first statement it emits
        let mut bodies: BTreeMap<&str, &Scope> = BTreeMap::new();
        for (_, _, sc) in &self.out {
            if sc.closed {
                bodies.entry(sc.owner.as_str()).or_insert(sc);
            }
        }
        let look = self.look(alias);
        // a body this pass did not walk (the datums' pass after the document's) keeps what the
        // pass that walked it decided
        let bound: Vec<Option<Option<Bound>>> = self.instances.iter().map(|info| {
            let sc = bodies.get(info.prefix.as_str())?;
            Some(self.closed(info, alias).then(|| {
                let stands = sc.stands.as_deref()?;
                let frame = look.stood_in(stands, sc);
                let datum = drawn_in(&look, stands, sc, frame.as_deref())?;
                let mut key = format!("{}\u{1f}{datum}", info.comp);
                for (name, v) in &info.values {
                    key.push_str(&format!("\u{1f}{name}={:016x}", v.c.to_bits()));
                }
                Some(Bound { key, frame })
            }).flatten())
        }).collect();
        for (info, b) in self.instances.iter_mut().zip(bound) {
            if let Some(b) = b {
                info.binding = b;
            }
        }
    }

    /// Whether `info`'s definition is a function of numbers alone, each given a constant.
    fn closed(&self, info: &InstanceInfo, alias: &BTreeMap<String, String>) -> bool {
        let comp = &self.prog.components[info.comp];
        let carries = |formal: &str| {
            let key = format!("{}{formal}", info.prefix);
            alias.range(key.clone()..).take_while(|(k, _)| k.starts_with(&key))
                .any(|(k, _)| k == &key || k[key.len()..].starts_with('.'))
        };
        info.drawn && !info.path.is_empty() && comp.set.is_none()
            && info.values.values().all(|v| v.free.is_none())
            && comp.formals.iter().all(|f| match f.ty {
                Ty::Ent(_) | Ty::Path | Ty::Set => false,
                Ty::Group => !carries(&f.name.text),
                _ => true,
            })
    }
}

/// What the body standing at `stands` is drawn in, as a datum of `frame`: `page` where no `in`
/// stands around it, `frame` for the frame itself, the datum's name for one of its planes (the
/// plane its origin is drawn in, `o.plane`, being its front) — `None` for any other plane,
/// which the body would read beside its frame.
fn drawn_in(look: &Lookup<'_>, stands: &[InPlane], sc: &Scope, frame: Option<&str>)
    -> Option<String> {
    let Some(ip) = stands.last() else { return Some("page".into()) };
    let at = Scope { stands: None, ..Scope::written_at(ip, sc) };
    let (abs, rest) = look.raw(&ip.plane, &at)?;
    let made = frame.map_or_else(|| "std.".to_string(), keys::frame_datums);
    let datum = abs.strip_prefix(made.as_str());
    match rest.as_slice() {
        [] if Some(abs.as_str()) == frame => Some("frame".into()),
        [] => datum.map(str::to_string),
        [p] if p == "plane" && datum == Some("origin") => Some("front".into()),
        _ => None,
    }
}
