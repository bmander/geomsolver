# Phase 3c: correspondence between carried-rim charts

This implements the bounded correspondence experiment in the
[Phase 3 subphase plan](swept-boundary-phase-three-plan.md). It starts with source edges and
native parameter boxes; domain union, global event discovery and exposure remain 3d–3g.
The ordinary production cylinder mesh is unchanged.

## Output and supported geometry

`solid::swept_boundary::arrangement::carrier::Correspondence` borrows an immutable
`SweepContacts` snapshot and retains each supplied `Band { edge, domain }` separately.
Its outputs are a common spherical carrier, regular native regions with projection charts
and derivative evidence, and the unresolved remainder of every requested band. Multiple bands
may name the same edge; neither source parameters nor consumer identities are collapsed.

The initial exact relation supports a circular fixed-u edge at a line-meridian endpoint of a
revolved face, carried by one nonstationary, nonscrew rotation. Source and motion axes must be
exactly coordinate-aligned and orthogonal. Their axis lines must intersect. Radius, height,
axis location, rotation direction, rate and phase are read from the snapshot, not a cylinder
fixture table. The two incident faces must describe the same parameterized edge structurally.
General orientations, relative motions, round-meridian endpoints and other edge charts refuse
this relation. They are not evidence of different carriers; they need a broader relation reader.

The reader also supports multiple such source edges when it can establish the same carrier.
Sharing a carrier does not establish that the requested domains overlap, are eligible edge
sweeps, or are globally visible.

## Why the carrier identity is analytic

Let `s` be the circle's source axis, `m` the distinct rotation axis and `k` the remaining
coordinate. The circle has centre `C` and radial vector `A` perpendicular to `s`. Its other
radial vector is `B = s cross A`, with the source-axis sign retained. These vectors have
identical length and are perpendicular by construction. Thus its source curve is
`C + A cos(theta) + B sin(theta)`, with the original angular chart.

An intersection of the two axis lines exists in this supported configuration exactly when
`C[k] == motion_origin[k]`. Its point `O` equals `C` except that its `s` coordinate is the
motion origin's `s` coordinate. Consequently every point of the source circle satisfies

```
|point - O|^2 = |A|^2 + (C[s] - O[s])^2.
```

Rotation about the motion axis fixes `O` and preserves this equation for every roll. This is
an identity of the mathematical geometry represented by the stored snapshot, not a sphere fit
to samples and not a bound on the original constraint solve's error.

Circle extraction and the centre offset require exactly representable additions/subtractions.
`interval::exact_difference` uses the TwoDiff residual to refuse inexact operations, overflow
and nonfinite input. Carrier comparison retains the unordered absolute terms whose squares
sum to the radius squared. Equal centres and equal term lists are a sufficient exact identity
test; comparing rounded squared radii would not be. Different term decompositions that happen
to produce the same mathematical radius intentionally remain unestablished.

The nominal cylinder bands produce centre `(3,0,0)` and squared-radius terms `(0,1,1)`.
Moving the motion axis even slightly out of the circle-axis plane must fail the exact
intersection condition. A tilted axis must fail the coordinate-axis reader. Neither case is
snapped onto the nominal carrier.

## Regular maps and their inverse branches

A region's common chart retains the motion-axis coordinate and one transverse Cartesian
coordinate. The omitted coordinate has a fixed sign relative to the sphere centre. The
sphere equation then determines that coordinate uniquely on the selected hemisphere.

For a native rectangle `(edge t, roll)`, interval source bounds enclose the axial derivative
`dx/dt`. Interval motion/source bounds enclose the omitted coordinate and therefore the
other derivative `dy/droll = +/- rate * (omitted - centre)`. Both must exclude zero.
The axial coordinate is independent of roll. These two monotonicities establish global
injectivity on the rectangle, not merely nonzero rank at a sampled point. The retained
orientation is that of the native-to-projection map; it is not an outward material normal.

The map uses whichever transverse projection can be established over the cell. It subdivides
when a cell crosses an angular chart boundary, a source fold or a pole. Depth, floating-point
resolution, bound errors and a total cell budget retain unresolved native rectangles. Completed
and unresolved rectangles cover the supplied bands with disjoint interiors. Projection charts
avoid an atan2 longitude seam; their hemisphere identities remain explicit. Source coordinates
never wrap at `t=0/1`, and no equality based on binary64 TAU is assumed there. Complete periodic
handoffs remain part of 3e.

`forward` returns the original source evaluation, both incident face coordinates and the common
projection coordinates. `inverse` uses the existing bounded DogLeg adapter, constrained to the
chosen native region, and checks all three original position equations. `transfer` reprojects
through the destination chart, checks its hemisphere and compares the two original evaluations.
Different generating times and edge parameters are preserved rather than averaged.

Point inversion is numerical, with an explicit position-agreement tolerance. The sphere relation
and interval regularity support its chosen branch, but a successful point solve is not an
interval enclosure of the inverse point. A failed solve is not proof that a target is outside
the curved image of a region. That distinction is necessary for 3d's domain operations.

## Regression coverage and validation

The focused `sweep_mesh::carrier` suite covers:

- The two original overlapping bands, retaining both inverse maps and both incident faces.
- Independent nominal sphere and inverse-parameter checks, multiple inverse seeds, reversed
  band order, negative/nonunit motion rate and nonzero phase.
- Native-cover partitioning across folds and projection-chart changes, including zero and
  limited work budgets and a circle passing through a carrier pole.
- A tiny skew-axis offset, a tilted axis, unsupported source/motion families and invalid input.
- Common-carrier domains on opposite hemispheres and outside one another's roll limits.

The arithmetic unit regression checks that a rounded difference cannot establish identity,
including subnormal residuals and overflow. Final validation:

- `cargo test --manifest-path rust/Cargo.toml -p gcs-core --test core sweep_mesh::carrier -- --nocapture`:
  **7 passed**, zero failed.
- `cargo test --manifest-path rust/Cargo.toml -p gcs-core --lib interval::exact_tests`:
  **1 passed**, zero failed.
- `cargo test --manifest-path rust/Cargo.toml -p gcs-core --test core`:
  **1,264 passed, zero failed, 100 ignored** (153.66 seconds).
- `cargo check --manifest-path rust/Cargo.toml -p gcs-cli --features occt,manifold`: passed.
- The ignored `sweep_mesh::status::phase_zero_status` exporter passed (114.87 seconds), using
  `SOLVENT_EXPORT=/private/tmp/swept-boundary-phase3c-exports`. All seven STLs and `status.tsv`
  are byte-identical to the `exports/` members of the unchanged Phase 1a replay archive,
  SHA-256 `914b94111651a4efcaa3249614fd2731e3c87ef9c24ba8abd509af1461e7fb7c`.
  The dumbbell retains the same construction refusal. These remain diagnostic candidates.
- `git diff --check`, whitespace checks on new files and local documentation links: passed.

The original two-band case returns two regular regions with opposite native-to-chart
orientations, with no unresolved correspondence region. Inversion tests use position agreement
`1e-10`; the nominal sphere and source-parameter formulas are independent test-only judges.
The full-turn and pole controls intentionally retain unresolved singular-limit regions rather
than assert regular continuation there. This completes the bounded 3c exit, not the cylinder
construction or acceptance gates.

## Next step

3d must construct the union of the mapped rim domains, including their boundaries, coverage
multiplicity and consumer maps. The current regular projection regions and explicit unresolved
cover are its inputs. Common-carrier identity and numerical transfer alone do not perform that
union, infer trim incidence or select globally exposed material.
