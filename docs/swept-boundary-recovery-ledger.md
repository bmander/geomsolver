# Swept-boundary construction recovery ledger

Phase 0a separates acceptance safety, validator calibration and successful construction.
This ledger accounts for all thirteen tests deferred in Phase 0. The barriers below are the
Phase 0 observations, not requirements that failures persist. New measurements belong in the
status matrix. No historical construction target is declared recovered by a reference mesh
passing validation.

Restore each success case through the current accepted-boundary API, with spatial evidence
and the relevant independent geometry checks. The default refusal budget may be insufficient;
record a justified bounded acceptance budget explicitly. Whole-cylinder and general-sweep
completion additionally require the embedding and encoded-export gates in the
[architecture plan](swept-boundary-architecture-plan.md).

## Deferred tests

Test names below are under `sweep_mesh::`. “Restore” means reinstate that test with the current
acceptance contract; “replace” identifies a historical heuristic that is not itself a goal.

| Order | Test | Recorded barrier | Required positive recovery |
|---|---|---|---|
| 1 | `closed::a_sphere_turned_about_its_own_centre_sweeps_only_itself` | Spatial audit refusal | Restore using the generated candidate; compare with the independent sphere control. |
| 1 | `closed::a_sphere_translated_along_the_spindle_sweeps_a_capsule` | Spatial audit refusal | Restore the 10-unit generated sweep, with independent capsule geometry and both coverage directions. The 2-unit calibration fixture alone does not restore this case. |
| 1 | `closed::a_negative_advance_sweeps_the_other_way` | Spatial audit refusal | Restore the negative 10-unit sweep, including extent and orientation checks. |
| 2 | `cases::a_box_translated_along_x_sweeps_a_longer_box` | Two unresolved centroid checks | Restore strict surface evidence and the exact prismatic volume check. |
| 2 | `closed::a_triangular_prism_translated_along_x_sweeps_exactly` | Twenty unresolved centroid checks | Restore supported grazing/cap boundaries and the independent exact volume check. |
| 2 | `closed::a_cylinder_plunged_along_its_axis_sweeps_a_longer_cylinder` | Disconnected topology | Restore all intended surface as one supported cylinder, without deleting components to pass topology. |
| 3 | `creases::no_loop_the_field_calls_a_hole_is_left_unfilled` | Loop projection is not patch certification | Replace with accepted complete-cylinder geometry and the independently judged shared-boundary replay. A `Spanned` fan is not the acceptance target. |
| 3–4 | `creases::seeds_moved_below_every_tolerance_leave_the_mesh_as_it_was` | Instability and unresolved acceptance across five cases | Replace exact triangulation equality with accepted geometric/topological agreement within declared error under seed and sheet-order perturbations; apply as each case recovers. Preserve export byte parity for behavior-preserving refactors separately. |
| 4 | `cases::a_turning_prism_closes_along_its_creases` | Open topology and reversed checks | Restore conforming shared boundaries, strict evidence and the independent volume check. |
| 4 | `cases::a_box_turned_about_its_face_centre_closes_along_its_creases` | Seventeen unresolved centroid checks | Restore the generated candidate with supported grazing and crease geometry. |
| 4 | `cases::a_lens_turned_about_the_spindle_closes_along_its_crease` | Non-manifold vertex plus failed/unresolved checks | Restore valid vertex links and supported curved crease geometry. |
| 4 | `closed::a_sphere_turned_about_the_spindle_sweeps_a_torus_segment_with_spherical_ends` | Spatial audit refusal | Restore the generated torus segment against independent tube/end geometry. |
| 4 | `closed::a_cylinder_turned_about_the_spindle_sweeps_a_ring_sector_with_round_ends` | Open topology | Restore the ring-sector boundary with independently checked round ends. |

The whole-turn box and dumbbell remain additional generality targets; their tests were already
deferred before Phase 0 and are not part of this count of thirteen. The cylinder's original
complete-acceptance target likewise remains separate from its deferred loop-fill heuristic.

## Current separation of evidence

- `evidence.rs` and `audit_report.rs` use deliberately defective inputs to enforce refusal,
  report completeness, and budget accounting. These remain permanent negative controls.
- `calibration.rs` validates independent sphere/capsule meshes against static and swept fields.
  It establishes validator capability, not recovery of the candidate generator.
- `status.rs` exercises reporting and acceptance invariants on generated candidates. Its
  export instrument records exact current counts; those counts are not locked into ordinary
  tests as required failures. A newly accepted case still needs its positive recovery checks
  above before this ledger can mark construction recovered.

All thirteen historical bodies remain deferred at this stage. Recovery order follows the
mechanism being implemented; record any deliberate reorder and its reason here.
