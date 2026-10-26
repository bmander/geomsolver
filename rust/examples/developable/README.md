# Developable panels

Panels cut flat and bent to shape: a plywood boat's strakes, duct adapters, rectangle-to-round
transitions, any sheet rolled between two edges. Only a **developable** surface comes off flat
without stretching — a ruled one with **one tangent plane along each straight ruling** — and both
the panel and its pattern are words of `std.sv`, written as what they are:

```sv
panel := std.Developable(a, b)                             // the set: what the panel is
skin := { q | q distance(<= 2mm, side: outside) panel }    // the sheet, 2 thick: its band
sheet := solid(skin)
flat := std.Development(panel, s, d: 1mm) in plane          // its neutral layer, laid in a plane
```

The core recognises those bodies by what they say — a hand-written copy of `Developable`'s is
built the same — and builds them exactly: the rulings where one plane touches both rails, the band
as a faceted solid, the pattern by integrating the isometry the development's body states.
Issue #645; [docs/contract/developables.md](../../../docs/contract/developables.md) is the
contract.

Read them in this order:

| File | What it shows |
|---|---|
| `frustum.sv` | **A cone, the closed form.** Two coaxial circles, r 50 and r 30, 100 apart: every ruling a generator, the pattern an annular sector of radii 254.951 and 152.971 through 70.60°. The sheet is the panel's band 2 thick on its outside; its neutral layer, 1 out, is what is laid flat, and it lies on the sector of radii R + 5d. |
| `offset_adapter.sv` | **An oblique cone.** Two parallel rims whose axes do not line up: the plane touching one at an angle touches the other at the same angle, so every ruling pairs equal angles and runs through the apex. |
| `tilted_adapter.sv` | **Which root.** The upper rim tilted 25°: the rulings are solved, not paired, and of the two planes through a tangent that touch the other rim, `sense: same` takes the one the other rim runs through the way this one does — not the crossing root, a bow-tie that cuts through itself. |
| `cut_cylinder.sv` | **A plane curve in space.** A round duct mitred by a tilted plane: `std.Ellipse` drawn in that plane, read as a path in space, and the panel between it and the base circle is the cylinder. Laid flat it is the mitre template, a strip whose top edge is a cosine. |
| `square_to_round.sv` | **Corners fan.** A 200 × 120 rectangle (a closed chain) to a Ø100 round 150 above: four planar triangles, and four fans about the corners, where the plane turns between the corner's two sides, supporting it. The band is seamed at the middle of a long side, and its pattern is the sheet-metal transition's. |
| `dory_panel.sv` | **Ends fan, and the flat placed by relations.** A dory's side strake between two splines in space, chine and sheer. The sheer runs past the chine at the stem and the transom, so the chine's end points fan. The neutral layer is laid on a 2440 × 1220 sheet of plywood drawn in its own plane, placed by two distances and a perpendicular; `dory_panel.svd` is the cut file. |

What the words are:

- **`std.Developable(a, b)`** — a set: a point on the ruling between `m` on `a` and `n` on `b`,
  where one plane `T` touches `a` at `m` and `b` at `n`, `b` running through `T` the way `a` does
  (`sense: same`). At a corner, or a rail's end, `T` *supports* the rail: it lies between the two
  one-sided tangents, so the rulings fan there. Two closed rails make a band, which must come
  round; one closed and one open is refused.
- **Sides** — the panel's `outside` is the side `T`'s normal points to, `a′ × (n − m)`: the order
  of the rails. Given the other way round, the sides swap, and so does which face of the pattern
  is up.
- **The band** — `{ q | q distance(<= d, side: outside) S }` is the band from the panel along its
  normal, square at its edges: past a sheet's edge there is no foot. Its faces are `inside`,
  `outside`, `a`, `b`, and the end rulings' `start` and `end` (none on a band that closes).
  `std.Offset(S, d:)` is its layer.
- **`std.Development(S, s, d)`** — the layer `d` out along `S`'s normal, laid in a plane with its
  outside up, from its home ruling, the one through `s` on `S.a`. It makes `flat.a` and
  `flat.b` (curves of the plane), the home ruling's ends `flat.f` and `flat.g`, and the end
  rulings `flat.start` and `flat.end` — a band's `flat.seam` and `flat.seam_end`. It is a figure
  with **three freedoms**, placed by relations like any drawn figure.
- **Refused, said once at the use (E080)** — rulings that fold (one rail would run backward, as
  where the panel turns flat along a ruling and two families of touching planes cross), a ruling
  tangent to a rail, no root with `sense: same`, a band that does not come round, one rail closed
  and one open.

`solventc` prints, for a panel's band, its volume, faces and bounds; for a flat, each curve's
length, the pattern's area and its bounds; and a refusal in the core's words where the panel
cannot be built. Each file's test is in `rust/gcs-core/tests/developable.rs`, held to the closed
forms in its comments.
