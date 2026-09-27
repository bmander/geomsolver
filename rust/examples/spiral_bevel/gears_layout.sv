// Both members as ordinary solids, laid out step by step (layout.sv, members.sv):
// the pair gears.sv makes, from geometric constructions. The normal module is left
// to the layout, which measures it off the tooth trace.
unit mm
use std
use design
use members

pair: HypoidPair(std.front, hypoid_design)
