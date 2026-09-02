//! The module library compiled into the core.
//!
//! The core takes text and has no filesystem, so in the browser a `use` can only resolve against
//! what is compiled in — and that is this table: the modules the shipped examples are written
//! over, named as `use` names them (`engine.crank` is `rust/examples/engine/crank.sv`).  The CLI
//! looks beside the document first and comes here second, so a file of the same name wins there.

/// The library, `(name, text)`.
pub const MODULES: &[(&str, &str)] = &[
    ("std", include_str!("../../lib/std.sv")),
    ("engine.dims", include_str!("../../examples/engine/dims.sv")),
    ("engine.parts", include_str!("../../examples/engine/parts.sv")),
    ("engine.valvetrain", include_str!("../../examples/engine/valvetrain.sv")),
    ("engine.end_view", include_str!("../../examples/engine/end_view.sv")),
    ("engine.side_view", include_str!("../../examples/engine/side_view.sv")),
    ("engine.top_view", include_str!("../../examples/engine/top_view.sv")),
];

/// A module's text by name, or `None` where the library has none.
pub fn module(name: &str) -> Option<&'static str> {
    MODULES.iter().find(|(n, _)| *n == name).map(|(_, t)| *t)
}

/// The resolver `modules::link` takes, over the library alone.
pub fn resolve(name: &str) -> Option<String> {
    module(name).map(str::to_string)
}

/// Parse a document and link it against the library — what every host without a filesystem does.
pub fn parse_linked(src: &str) -> (crate::syntax::Program, Vec<crate::syntax::SynErr>, Vec<crate::program::Diag>) {
    let (mut prog, errs) = crate::syntax::parse(src);
    let diags = crate::modules::link(&mut prog, &mut resolve);
    (prog, errs, diags)
}
