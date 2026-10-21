# Analysis: materials, and what a solid is made of

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch
`model/material.rs`, a `material(…)` group (`flatten::values::material`), a solid's `of`
(`program::solids::build`), or a material's JSON (`io::material_json`) or report
(`report::materials_json`). An issue number (#n) or a spec version ([0.n], see the
[language changelog](../solvent-changelog.md)) says where a rule came from.

This is the contract file of #510, design intent as structural performance. It holds what has
landed, P3 so far. Load scenarios, the analyses and the requirements judged under them will be
added here as their stages land.

- **A material is a model value** (#510 P3, [0.164]): `model::Material`, a name and the
  properties stated of it (`model::Property`: `E`, `nu`, `rho`, `yield`).  `Sketch::materials`
  holds the document's materials by absolute name (`steel`, `p.al` inside an instance), and
  `SolidE::material` names the one a solid is made of.  Nothing reads either yet.  The solve,
  the diagnosis and every evaluated solid are unchanged by a material.  `tests/materials.rs` is
  the gate.
- **A material is held in SI** (`Units::to_si`): pascals, and kilograms per cubic metre.  The
  same text states the same material in any document unit, inch or millimetre
  (`a_material_is_the_same_in_any_unit`).  The stored value is the value worked out (`7850kg / 1m^3` in millimetres comes back a hair under 7850).  Only a report
  rounds, `Property::text` to twelve figures.
- **`steel := material(E: …)` is a group with a material head** (`syntax::GroupDecl::material`).
  Its members are worked out as any group's, so `steel.E` reads as a number with its dimension.
  A material is handed to a component as a `group` formal.  `flatten::values::material` reads the
  members once they are worked out and checks each with `Property::read`.  A material whose
  properties all pass is stated in `Expansion::materials`.  One that fails any check is still
  a name a solid's `of` may give, but nothing more is said of that solid.  Every complaint is
  E103 at the property, or E001 for a property given twice:
  - a label that is no property;
  - a value of another dimension;
  - a bare number for a dimensioned property;
  - a value out of range;
  - a value that reads an unknown of the drawing;
  - a material stating nothing (E103 at the name).

  **A bare number is refused, not read in the document's unit.**  In a millimetre document the
  unit of pressure is a kilogram per millimetre-second², which no one means by `200000`.
  `Property::admits` holds the ranges: `E`, `rho` and `yield` are positive, and `nu` lies in
  (-1, 0.5).  The JSON reader holds the same ranges.
- **`of` names a material** (`syntax::Decl::material`, a trailer of `solid(…)`, reserved like
  `in`).  The flattener resolves it and judges it in `flatten::resolve::rewrite`, where it
  knows whether the name resolved.  `build_solid` only attaches a material that was stated.
  Each outcome:
  - a name that resolves to something that is no material is E103 (`` `rim` is no material``);
  - a material that was refused says nothing more;
  - a name of nothing is E101 alone, and one read through a name already said to be nothing
    says nothing.

  A group read whole is "handed on" for the W206 lint only inside an argument of a call
  (`program::drift`), so `of m` reads its group formal.

  The material belongs to the solid the document named.  When `Sketch::as_body` turns a cut
  solid into a body, the body keeps the material and the sweep moved to its stock carries none.
  A body's layers below its top carry none either.
- **A material is document state.**  `io::to_json` writes `materials`, but only when there is
  one: `[{name, E?, nu?, rho?, yield?}]` in SI.  `io::from_json` reads it back through
  `io::material_from_json`, which refuses a property no material has and a value no property
  takes.  The JSON holds no solids, so a solid's `of` is the source's to carry.  A rebuild
  (`graft`) copies the materials, and a paste brings those the document lacks.  `to_program`
  lifts each one as `material(…)`, written by `Property::source`, under a name a program can
  declare.  The printer writes `material(…)` and `of` back as written.
- **The surface:** `report::materials_json` gives each material's record with its `text` and the
  `solids` made of it.  `gcs_materials_json` exposes it over the ABI and `mesh.ts`'s `materials`
  exposes it to the page.  `report::material_text` is the CLI's line, `material steel: E 200GPa,
  …, of disc`.  `gcs-ffi/tests/materials.rs` checks the ABI and its JSON round trip.
