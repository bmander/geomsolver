# Finite faces and indexed surface accuracy

The earlier [fillet](FILLETS.md), [flank](FLANKS.md) and [closure](SUPPORTS.md) bounds concern
complete support surfaces extracted from the tooth-space STEP files. This audit checks
that the finite CAD faces cover those parameter domains, then transfers the support bounds
to the actual indexed pinion and gear STEP files.

## Exact finite parameter coverage

`export_face_domains.py` extracts each tooth-space face's ordered wires and straight 2D
parameter curves. It compares the actual support's entire coefficient definition with the
previously bounded extraction and requires exactly one reference per face. The adapter
checks that ordered wire traversal visits every edge. All twenty tooth-space faces are
accounted for, and all use the full `[0,1] × [0,1]` parameter rectangle.

`check_face_domains.py` uses exact rational arithmetic on the exported binary64 line
origins, directions and finite parameter ranges. Each segment must lie on one square side.
The segments must form one directed closed cycle, with no holes. On each side, sorted
intervals must partition `[0,1]` exactly once: no missing endpoints, overlaps or gaps.
The support's knot domain must be the same square. This permits subdivided sides without
assuming that every rectangle always has four topological edges.

The same check passes for **all 360 indexed B-spline faces**. Six of their rectangles have
one side split into two edges; the split endpoints agree exactly. There are no shortened
or missing parameter strips. This proves finite parameter coverage for the extracted
surface/parameter-curve representation. It does not independently bound the separate 3D
edge curves against both incident surfaces, or establish global material exposure.

## Transfer through indexing and CAD operations

`export_indexed_surfaces.py` inventories every face of both actual indexed STEP solids.
Each B-spline is associated with a candidate tooth-space support and tooth index using an
interior control point. That sample only selects a candidate: the independent checker
compares **every control point** and requires identical degrees, knots and weights.
The references must equal the earlier bounded coefficient records, and all source, tool,
coefficient and indexed STEP hashes must agree.

There are exactly five retained supports at every tooth index: four generated faces and
the root closure. No duplicate index/support pair is accepted. The pinion has 120 such
faces; the gear has 240. The remaining faces are recorded separately as analytical sphere
and cone surfaces; they are not silently counted as verified by this B-spline audit.

For a common polynomial B-spline basis `N`,

```text
Sindexed(u,v) - R*Sreference(u,v) = sum N_ij(u,v) * (Pindexed_ij - R*Preference_ij).
```

The basis is a nonnegative partition of unity on the checked domain. Thus the largest
rotated pole-error norm bounds the entire surface difference. `R` is the mathematical
rotation by `2*pi*index/teeth`, not the backend's rounded transform. An independent rational
arctangent enclosure supplies `pi = 4*atan(1)`; interval sine/cosine and outward arithmetic
bound the rotated poles. This includes differences introduced by indexing, Boolean
construction and export as observed in the extracted binary64 coefficients. It does not
bound decimal STEP-to-reader conversion against the mathematical decimal file values.

## Results

[Recorded results](indexed-surface-results.json) bind all artifacts, the verifier code and
the earlier nominal support bounds. The complete audit took 27.9 seconds including input
checks and exact finite-domain verification.

| Member | Indexed B-spline faces | Maximum rotation-transfer bound (mm) | Combined nominal bound (mm) |
|---|---:|---:|---:|
| Pinion | 120 | 1.887e-11 | 0.000997078 |
| Gear | 240 | 2.248e-11 | 0.000973153 |

The combined bound adds each face's transfer bound to the earlier bound for its corresponding
support. Rigid rotation preserves the nominal correspondence distance; the root cone is
invariant under the indexing rotation. Every combined bound remains below 0.001 mm. For
root closures this remains a distance to the nominal conical support, with no claim that
its parameterization matches a particular finite region on that cone.

Independent controls check exact quarter turns, a known clamped-corner displacement, basis
changes, invalid indices, gaps, overlaps, holes, reversed/disconnected wires, and tiny
out-of-domain edges. An actual indexed coefficient export with one corner pole displaced
by 0.01 mm is rejected by the transfer checker. As with the earlier controls, this tests
the mathematical audit and does not certify the native STEP reader.

## Reproduction

```sh
/private/tmp/solvent-occt-env/bin/python experiments/cad-backend/export_face_domains.py /private/tmp/solvent-cad-face-domains.json /private/tmp/solvent-cad-closure-coefficients.json /private/tmp/solvent-cad-fillet-coefficients.json /private/tmp/solvent-cad-flank-coefficients.json
python3 experiments/cad-backend/check_face_domains.py /private/tmp/solvent-cad-face-domains.json /private/tmp/solvent-cad-face-domain-audit.json
/private/tmp/solvent-occt-env/bin/python experiments/cad-backend/export_indexed_surfaces.py /private/tmp/solvent-cad-sections.json /private/tmp/solvent-cad-one-cut-checked /private/tmp/solvent-cad-full-pinion/report.json /private/tmp/solvent-cad-full-gear/report.json /private/tmp/solvent-cad-indexed-coefficients.json
python3 experiments/cad-backend/check_indexed_surfaces.py /private/tmp/solvent-cad-indexed-coefficients.json /private/tmp/solvent-cad-face-domains.json /private/tmp/solvent-cad-indexed-surface-audit.json
python3 -m unittest discover -s experiments/cad-backend -p 'test_*.py'
```

Analytical blank-face accuracy and trims, separate 3D edge consistency, original source and
assembly-frame error, STEP-reader conversion, global swept-material exposure and continuous
mating remain acceptance tasks. These results extend the nominal surface evidence to the
indexed candidates; they do not establish a production-validated pair by themselves.
