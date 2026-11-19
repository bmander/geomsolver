# Hydrostatics · where a body is made, and a face's measures

What a hull displaces, where its buoyancy acts and how stiff its waterplane is, stated in the
document beside the geometry (#953). A body kept within the water is the hull in common with a
half-space, `immersed := solid({ p | p in hull; p in water })`, built exactly; its waterplane is
its face `immersed.water`. Four words measure it, each a row of the drawing stated and judged
claimed:

- `g centroid B` — the point where a solid's volume (or a face's area) is centred: the centre of
  buoyancy of the immersed body, the centre of flotation of its waterplane;
- `g centerofmass B` — where a body's mass is centred, `B` a solid or an instance, every body
  under it weighed by its own density;
- `area(A) F` and `moment(I, about: l) F` — a face's area and its second moment about a line, the
  waterplane's `I` for BM = I / ∇.

The library's **box barge** (`box_barge.sv`, in the examples menu) finds its waterline from its
displacement and states BM. Here:

- `cylinder.sv` — a log floating at a draft: curved faces, against the circular segment's closed
  forms, each to the quadratic mesh's error the report gives beside it.
- `heeled_wedge.sv` — a V-wedge heeled 10° about its apex: an inclined waterline, exact.
- `centre_of_mass.sv` — a steel and an aluminium block as two components of one instance and as
  one component of both: one centre of mass.
- `punt.sv` — a hull of panels: two developable sides and four drawn faces, a closed shell, and the
  solid it encloses (`p in shell.side`); its displacement at a stated waterline claimed.
