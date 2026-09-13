# Frozen Phase 1 counterexamples

These are deliberately bad historical candidates from `a996167` plus Phase 0a.
Do not update them to match a future successful construction.

- `merge-before.mesh`: clipped source triangle 2701, true source patch 10.
- `merge-after.mesh`: the same triangle after aliasing vertex 4052 to 2046.
  The move is below the 0.02 snap tolerance but reverses the material sides.
- `zip-proposal.mesh`: applied small-fan proposal 7 in zip round 0; its nominal
  inside is exterior and its nominal outside material.

The complete stage snapshots, exact decisions, options, source parameters, field
witnesses and code overlay are in `docs/fixtures/cylinder-phase-one/replay.tar.gz`.
`docs/swept-boundary-phase-one.md` records the derivation and evidence limits.
