# Solvent Drawing (.svd)

Solvent models (`.sv`) specify geometry, constraints, solver hints, and geometric assertions.
Solvent Drawing files (`.svd`) describe presentations of solved models. Drawing edits cannot
change a model's solution. A component remains a scope containing statements; it has no
designated body or implicit geometric output.

```
model part from "vtwin_piston.sv"
use "technical.svd"

sheet piston {
  size A4
  scale 2
  view front(part.pis.body) from front at (60mm, 150mm)
  view right(part.pis.body) from right at (140mm, 150mm)
  view top(part.pis.body) from top at (60mm, 65mm)
  dimensions in front
  label "Piston" at (20mm, 280mm)
}
```

The imported `technical.svd` can contain reusable styles:

```
style .hidden { dash: 4 3; color: #888888 }
style .section { width: 2 }
style .dimension { color: #000000 }
```

Run `build/solventc rust/examples/vtwin_piston.svd --output piston.svg`. A drawing with several
sheets requires `--sheet NAME`. SVG width and height are physical paper dimensions. The
browser's **File → Open drawing folder…** loads a local folder containing the drawing and
its model files, with subdirectories preserved. The project file picker on the right lists
both `.svd` and `.sv` files. Selecting a drawing shows its paper preview in the main workspace,
with sheet selection and SVG export; selecting a model shows the normal editable geometry view.
The source panel edits the selected file. Edits stay with each file when switching, and drawings
render from the updated model sources. **File → Save** downloads the selected source file.
In the drawing preview, scroll to zoom around the pointer and drag to pan. The **−**, **+**,
and **Fit sheet** buttons also control the view. With the preview focused, arrow keys pan,
**+ / −** zoom, and **0** fits the sheet. Each drawing sheet remembers its view while switching
files or rendering edits; exports retain the authored paper size and layout.

**File → Open examples…**, the default startup example, and `?example=NAME` URLs open the
example's main `.svd`, with its `.sv` assembly and imported modules available in the same picker.
Example sources are included in the static web build;
the local demo server supplies fresh files when available. Parameterized URLs such as
`?example=rect_fillets:80:40:5` use the same drawing with the generated model parameters.

## Files and dependencies

`model ALIAS from "PATH"` imports the root model in a `.sv` file. Model `use` statements still
resolve through the existing module system. Model aliases are local to the drawing, and a
drawing can import multiple models. Repeated imports of the same canonical file share one
solved snapshot. Unsolved or invalid models produce a drawing diagnostic.

`use "PATH"` imports styles, recursively. Such an import may contain other style imports but
cannot contain models or sheets. Imports resolve relative to their importing file. Cycles,
excessive nesting, missing files, and invalid source produce errors. Imported rules precede
the importing file's rules, in import order; sheet rules follow document rules.

The core performs no file I/O. `drawing::compile` takes a host loader returning a canonical
source identity and text. `drawing::render` instead takes already-solved models and source
maps, so an interactive caller can render several drawings without solving again. The native
and WebAssembly ABI accept a bundle of source texts and use the same compiler and SVG writer.

## Sheets and coordinates

`sheet NAME { ... }` declares a sheet. Names must be unique within the document.

* `size A4`, `size A3`, `size Letter`, or `size (WIDTH, HEIGHT)` specifies paper dimensions.
  Default: A4 portrait. Swap width and height for landscape.
* `scale NUMBER` specifies paper length / model length. Default: 1; each view may override it.
* Page lengths accept `mm`, `cm`, and `in`; bare page lengths mean millimetres.
* Page coordinates start at the upper left; x grows right and y grows down.
* `at (X, Y)` places the projected coordinate origin. It does not automatically center or fit
  the geometry. This makes aligned projections predictable.
* A model's declared units are converted to paper millimetres. For legacy unitless models,
  one model unit is treated as one millimetre for drawing scale.
* Stroke widths, dash lengths, and lettering use CSS pixels (96 per inch), independently of
  model scale. Sizes and scales must be finite and positive.

A model plane supplies geometric orientation and origin only. Its old 2D sheet placement is
not copied into a solid view. Page position and scale come exclusively from the `.svd` file.

## Views and sections

```
view front(m.body) from front at (60mm, 150mm) scale 2
view auxiliary(m.body) from m.datum at (150mm, 150mm)
section cutaway(m.body) from front cut m.midplane at (60mm, 240mm)
sketch construction(m.linkage) at (60mm, 100mm)
```

A solid view names an explicit solid, including an intermediate or cutter when desired.
It does not turn a component into a body. `from` accepts `front`, `back`, `right`, `left`,
`top`, `bottom`, or a model plane path; omitted, it means `front`. Projection uses Solvent's
world axes: the front plane has horizontal X and vertical Z. A section additionally names
a cutting plane and must be viewed parallel to it. Views retain source face paths, distinguish
hidden edges, and suppress tessellation seams through the existing renderer.

A `sketch` presents the model's existing 2D coordinates. Its target can be a model alias, a
component/member scope, or an individual entity. It has no arbitrary 3D viewing direction.
Points and plane glyphs are hidden by default; styles can show them. A bare `.sv` still gets an
automatic editor preview without an authored drawing. It does not automatically annotate
model constraints. The editor can inspect all constraint values through an explicit option;
editing a dimension temporarily shows that dimension alone. Opening a model resets this
inspection overlay. Authored annotations are the drawing's `dimension`, `measure`, and
`dimensions` requests.

## Dimensions and annotations

```
dimensions in front
dimension m.width in construction at (0, 12)
measure distance(m.a, m.b) in front offset 6mm
label "Section A–A" at (20mm, 270mm)
```

`dimensions in VIEW` selects automatic annotations: overall projected extents and surviving
round-feature diameters for a solid, or constraint dimensions for a sketch. Solid measurements
refer to the target solid, including when its view is a section. Annotation layout reuses the
core callout engine. Individual constraints are hidden until selected; `dimension` selects a
named dimension such as `a distance(width = 60mm) b`. Named constraint annotations currently
belong in sketch views, where their original dimension frame is meaningful. Their optional
`at (t, r)` uses that frame: model units for linear offsets, degrees for angular offsets.

`measure distance(A, B) in VIEW` reads two named points without adding a constraint, assertion,
or variable to the model. Entity fields and indexed members work, e.g. `m.bar.p1` and
`m.peg[2].p`. The value is the physical distance in model units. `offset` is a paper distance
from the measured line, default 6mm. A foreshortened or collapsed projection is refused rather
than labeled as a true-length dimension. Measurements must reference the view's model.

Missing names are errors, including references to deleted dimensions. Source line numbers,
statement offsets, and internal entity indices are never drawing references. A model's
constraint value remains in `.sv`; its displayed annotation and placement live in `.svd`.

## Styling

```
style .hidden { dash: 4 3; color: #888888 }
style .point { display: inline }
style m.linkage.axis { color: #999999; width: 0.5 }
style m.linkage { display: none }
```

Class selectors refer to renderer-provided roles, such as `.visible`, `.hidden`, `.section`,
`.dimension`, `.reference`, `.point`, and `.closure`. Geometry selectors name an entity or a
component/member scope. Explicit geometry selectors must resolve; a misspelling is an error.
Later rules override properties they state. Geometry-specific rules override implicit roles.

Properties are `color` (#RGB, #RRGGBB, or #RRGGBBAA), `width`, `dash`, and `display`
(`none`, `inline`, or `geometry`). A semicolon separates properties; an empty `dash:` means a
solid stroke. Comments start with `//`; strings support escaped quotes, backslashes, and `\n`.
Unknown statements, properties, invalid values, and duplicate model/sheet/view names are errors.

## Migrating existing source

`.sv` rejects `view`, `section`, `dimensions`, `style`, `class` clauses, and callout placements.
Move output requests and styling to a drawing. Keep model planes needed by geometry, while
replacing page-spacing constructions with drawing positions. The six `vtwin_*.svd` part sheets
demonstrate this separation; their models no longer instantiate `ThreeViews` for page layout.

Model serialization and editor reconciliation omit styles, classes, and callout placements.
The explicit `syntax::parse_legacy` reader remains for historical renderer/migration fixtures;
it is not used to accept `.sv` source in the CLI or browser. Low-level rendering APIs retain
their presentation structures as adapters, independently of the model language.
