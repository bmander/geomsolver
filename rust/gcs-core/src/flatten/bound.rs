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
        // a body this pass did not walk (the datums' pass after the document's) keeps what the
        // pass that walked it decided
        let look = self.look(alias);
        let bound: Vec<(usize, Option<Bound>)> = self.instances.iter().enumerate()
            .filter_map(|(n, info)| {
                let sc = bodies.get(info.prefix.as_str())?;
                Some((n, self.closed(info, alias).then(|| bound(&look, info, sc)).flatten()))
            })
            .collect();
        for (n, b) in bound {
            self.instances[n].binding = b;
        }
    }

    /// Whether `info`'s definition is a function of numbers alone, each given a constant.
    fn closed(&self, info: &InstanceInfo, alias: &BTreeMap<String, String>) -> bool {
        let comp = &self.prog.components[info.comp];
        let carries = |formal: &str| {
            let key = format!("{}{formal}", info.prefix);
            let mut under = table::prefixed(alias, &key);
            under.any(|(k, _)| resolve::under_member(k, &key))
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

/// Closed instance `info`'s binding, its body read at `sc`: `None` where its call is drawn in a
/// plane its frame does not make.
fn bound(look: &Lookup<'_>, info: &InstanceInfo, sc: &Scope) -> Option<Bound> {
    use std::fmt::Write;
    let stands = sc.stands.as_deref()?;
    let frame = look.stood_in(stands, sc);
    let mut key = format!("{}\u{1f}{}", info.comp, drawn_in(look, stands, sc, frame.as_deref())?);
    for (name, v) in &info.values {
        let _ = write!(key, "\u{1f}{name}={:016x}", v.c.to_bits());
    }
    Some(Bound { key, frame })
}

/// What the body standing at `stands` is drawn in, as a datum of `frame`: `page` where no `in`
/// stands around it, `frame` for the frame itself, and a datum's path for one of its planes or a
/// field past one (`origin.plane`) — `None` for any other plane, which the body would read beside
/// its frame.
fn drawn_in(look: &Lookup<'_>, stands: &[InPlane], sc: &Scope, frame: Option<&str>)
    -> Option<String> {
    let Some(ip) = stands.last() else { return Some("page".into()) };
    let at = Scope { stands: None, ..Scope::written_at(ip, sc) };
    let (abs, rest) = look.raw(&ip.plane, &at)?;
    if rest.is_empty() && Some(abs.as_str()) == frame {
        return Some("frame".into());
    }
    let made = frame.map_or_else(|| "std.".to_string(), keys::frame_datums);
    abs.strip_prefix(made.as_str()).map(|datum| resolve::dotted(datum.to_string(), rest))
}
