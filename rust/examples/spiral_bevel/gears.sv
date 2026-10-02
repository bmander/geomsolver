// The pair, both members as ordinary solids: `pair.pinion.body` and `pair.gear.body`.
// `solventc --step` or `--stl` exports either; see README.md.
unit mm
use std
use design
use members

pair := members.HypoidPair(std.front, design.hypoid_design)
