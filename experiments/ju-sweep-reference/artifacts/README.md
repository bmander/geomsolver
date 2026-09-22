# STL inspection pairs

Each example directory contains:

- `tool-at-start.stl`: a parametric mesh of the YAML tool at time zero.
- `swept-solid.stl`: the reference's final sweep mesh converted to binary STL.

The pair shares coordinates and scale, so both can be loaded into the same scene.
Coordinates retain the examples' unitless model units; no millimeter conversion
or normalization was applied.

`simple-stock` is a radius-0.2 sphere translated from x = −0.5 to +0.5.
`letter_L-stock` is a radius-0.04 sphere on the supplied curved, overlapping path.
`flipping_torus-stock` uses major radius 0.2, minor radius 0.05, translation and
a full rotation. The local torus axis is x.

`prism-tumble` and `triangular-prism-tumble` are actual sharp-edged OBJ tools
swept by the reference CLI through a full z rotation and translation.
`analytic-prism-tumble` uses an exact box-distance callback with the same motion.
The rectangular tool measures 0.3 × 0.18 × 0.12 in model units. Both input paths
retain sharp tool edges. The prism STL files are about 19–22 MB each.

These are the original sweep triangles, including measured defects in the curved
path, torus, and prism examples. See [the report](../../../docs/ju-sweep-reference-experiment.md)
for the audit results. The tool meshes are independent visualizations of the
analytic input bodies; they are not outputs of the sweep algorithm.
