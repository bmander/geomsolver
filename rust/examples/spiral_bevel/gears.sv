// Both members as ordinary solids, laid out step by step (layout.sv, members.sv).
// `solventc --stl` or `--step` exports either body; see README.md.
unit mm
use std
use design
use members

pair := HypoidPair(std.front, hypoid_design)
