//! The module library compiled into the core.
//!
//! The core takes text and has no filesystem, so in the browser a `use` can only resolve against
//! what is compiled in — and that is this table: the modules the shipped examples are written
//! over, named as `use` names them (`engine.crank` is `rust/examples/engine/crank.sv`).  The CLI
//! looks beside the document first and comes here second, so a file of the same name wins there.

/// The library, `(name, text)`.
pub const MODULES: &[(&str, &str)] = &[
    ("std", include_str!("../../lib/std.sv")),
    ("hardware", include_str!("../../lib/hardware.sv")),
    ("engine.dims", include_str!("../../examples/engine/dims.sv")),
    ("engine.parts", include_str!("../../examples/engine/parts.sv")),
    ("engine.valvetrain", include_str!("../../examples/engine/valvetrain.sv")),
    ("engine.block", include_str!("../../examples/engine/block.sv")),
    ("engine.head", include_str!("../../examples/engine/head.sv")),
    ("engine.conrod", include_str!("../../examples/engine/conrod.sv")),
    ("engine.crankshaft", include_str!("../../examples/engine/crankshaft.sv")),
    ("engine.end_view", include_str!("../../examples/engine/end_view.sv")),
    ("engine.side_view", include_str!("../../examples/engine/side_view.sv")),
    ("vtwin.dims", include_str!("../../examples/vtwin/dims.sv")),
    ("vtwin.parts", include_str!("../../examples/vtwin/parts.sv")),
    ("vtwin.frame", include_str!("../../examples/vtwin/frame.sv")),
    ("vtwin.crank", include_str!("../../examples/vtwin/crank.sv")),
    ("vtwin.cylinder", include_str!("../../examples/vtwin/cylinder.sv")),
    ("vtwin.piston", include_str!("../../examples/vtwin/piston.sv")),
    ("vtwin.disc", include_str!("../../examples/vtwin/disc.sv")),
    ("vtwin.flywheel", include_str!("../../examples/vtwin/flywheel.sv")),
    ("vtwin.throttle", include_str!("../../examples/vtwin/throttle.sv")),
    ("vtwin.bank", include_str!("../../examples/vtwin/bank.sv")),
    ("vtwin.side_view", include_str!("../../examples/vtwin/side_view.sv")),
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
