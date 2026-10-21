// A first-stage high-pressure turbine rotor blade: a cooled, twisted airfoil on a platform, a
// shank carrying the seal wings, and a three-lobe fir-tree root broached along the engine axis.
//
// The frame is the engine's: `std.x` runs downstream along the engine's axis, `std.z` outward
// along the blade's stacking axis and `std.y` round the rotor, the hub's flowpath at z = 0 on the
// stacking axis, `r_hub` from the engine's axis.
//
// The airfoil is designed as a blade designer designs one: a section at the hub, the mean and
// the tip, each given by its chord, its stagger and its metal angles, its wedge angles and its
// edge radii, and every section between them the same construction given those numbers
// blended through the three (a quadratic in the span, `Station`). Each section is the
// classical circular-arc turbine profile (`Section`): a leading-edge circle and a trailing-edge
// circle a chord apart at the stagger; the suction side leaving the leading edge at the inlet
// angle plus half the leading wedge and reaching the trailing edge at the exit angle less half
// the trailing wedge, two arcs (the aft one, the uncovered turning, of a stated radius); the
// pressure side one concave arc leaving the leading edge at the inlet angle less half the wedge
// and touching the trailing-edge circle. Every joint is smooth. The design sections are stacked
// on their centroids, so the radial pull on the airfoil hardly bends it: `mid` is where the
// chord's middle stands from the stacking axis, the centroid's offset worked out from the
// section's five arcs (`tests/turbine_blade.rs` holds the three to it).
//
// Its metal angles turn the flow 105° at the hub and 93° at the tip; it twists 10° and tapers
// from a 30 mm chord to 28 over a 40 mm span: one of 76 blades on a 280 mm hub, a solidity of
// 1.3 there.
//
// Below the flowpath the platform is a sector of the hub's annulus one blade pitch wide (its
// slash faces radial planes), and the seal wings ("angel wings") are sectors too, each standing
// off the shank fore and aft. The shank and the fir tree are one section, broached along the
// engine's axis: each lobe a 40° bearing face, a land and a steeper under-flank into the next,
// narrower neck, every corner broken by a round. Six cooling holes are drilled radially, the
// length of the blade, on an arc that keeps about a millimetre of wall at every span and at the
// root's narrowest neck. Not drawn: the fillet where the airfoil meets the platform, since a
// round between a loft's wall and a plane is not built yet.
//
// Edit the design table (`hub`, `mean`, `tip`), `span`, `r_hub`, `blades`, the root's numbers
// or the holes.
unit mm
use std (coords, round, turns)

// ---- the stage ----
r_hub := 280mm               // the hub's flowpath at the stacking axis, from the engine's axis
blades := 76
pitch_angle := 360deg / blades
span := 40mm
embed := 1mm                 // the airfoil runs this far into the platform

// ---- the airfoil's design table: three sections, at the hub, the mean and the tip ----
// chord: the true chord, leading-edge circle to trailing-edge circle; stagger: the chord's
// angle from the engine's axis; inlet, exit: the metal angles at the edges, from the axis;
// le_wedge, te_wedge: the angles the two sides make there; r_le, r_te: the edges' radii;
// r_aft: the suction side's aft arc; mid_u, mid_v: the chord's middle from the stacking axis.
hub := {chord: 30mm, stagger: -18deg, inlet: 40deg, exit: -65deg, le_wedge: 40deg,
        te_wedge: 8deg, r_le: 1.6mm, r_te: 0.6mm, r_aft: 40mm, mid_u: 0.772mm, mid_v: -5.807mm}
mean := {chord: 29mm, stagger: -22.5deg, inlet: 33deg, exit: -66.5deg, le_wedge: 37deg,
         te_wedge: 7deg, r_le: 1.4mm, r_te: 0.55mm, r_aft: 42mm, mid_u: 0.386mm,
         mid_v: -5.406mm}
tip := {chord: 28mm, stagger: -28deg, inlet: 25deg, exit: -68deg, le_wedge: 34deg,
        te_wedge: 6deg, r_le: 1.2mm, r_te: 0.5mm, r_aft: 45mm, mid_u: 0.082mm, mid_v: -5.014mm}

// ---- the root ----
platform_t := 3mm            // the platform's thickness under the flowpath
fore := 18mm                 // the platform's reach upstream and downstream of the stacking axis
aft := 17mm
root_len := 28mm             // the root's length along the engine's axis, centred
neck := 5.4mm                // half the shank's width, and the first neck's
shank_to := 14mm             // the first neck, below the flowpath
lobe := 2mm                  // how far each lobe stands out from its neck
contact := 40deg             // the bearing face's slope from the platform's plane
land := 0.6mm
lobe_pitch := 4.6mm
taper := 0.6mm               // each neck narrower than the last, each side
rise := lobe * tan(contact)
fall := lobe_pitch - rise - land
// the section below draws three lobes: the root's foot, its depth and half its width there
root_depth := shank_to + 3 * lobe_pitch
foot := neck - 3 * taper
edge_r := 0.4mm              // the round breaking every corner of the root

wing_z := 8.5mm              // the seal wings: their underside below the flowpath,
wing_t := 1.4mm              // their thickness,
wing_out := 4mm              // and how far they stand off the shank

// ---- the cooling holes ----
holes := 6
hole_d := 1.2mm
hole_arc := 33.5mm           // the arc they are drilled on, from the first hole to the last

// One section of the airfoil, drawn in `n` about the stacking axis at its origin. Its numbers may
// run with the span; its seeds are read off `near`, the design-table section the family is
// seeded at, since a seed reads no number that runs.
component Section(n: plane, chord: Length, stagger: Angle, inlet: Angle, exit: Angle,
                  le_wedge: Angle, te_wedge: Angle, r_le: Length, r_te: Length, r_aft: Length,
                  mid_u: Length, mid_v: Length, near: group) {
  // where the sides run at the edges, `near`'s: the suction side's and the pressure side's
  // leaving the leading edge, and reaching the trailing edge
  ss_in := near.inlet + near.le_wedge / 2
  ps_in := near.inlet - near.le_wedge / 2
  ss_out := near.exit - near.te_wedge / 2
  ps_out := near.exit + near.te_wedge / 2
  hc := near.chord / 2
  in n {
    // the engine's axis through the section, which every angle is read from
    ahead := point
    ahead level(v) n
    ahead distance(10mm, along: u) n
    private construction axial := line(n.origin, ahead)

    // the edges' circles, a chord apart at the stagger, the chord's middle at `mid`
    mid := point
    mid coords(du: mid_u, dv: mid_v) n
    cle := point hint((near.mid_u - hc * cos(near.stagger), near.mid_v - hc * sin(near.stagger)))
    cte := point hint((near.mid_u + hc * cos(near.stagger), near.mid_v + hc * sin(near.stagger)))
    construction chord_line := line(cle, cte)
    mid midpoint chord_line
    axial angle(stagger) chord_line
    length(chord - r_le - r_te) chord_line

    // where the sides leave the edges' circles: each radius square to its side's direction
    ss_le := point hint((cle.x - near.r_le * sin(ss_in), cle.y + near.r_le * cos(ss_in)))
    ps_le := point hint((cle.x + near.r_le * sin(ps_in), cle.y - near.r_le * cos(ps_in)))
    ss_te := point hint((cte.x - near.r_te * sin(ss_out), cte.y + near.r_te * cos(ss_out)))
    ps_te := point hint((cte.x + near.r_te * sin(ps_out), cte.y - near.r_te * cos(ps_out)))
    private construction r_ss_le := line(cle, ss_le)
    private construction r_ps_le := line(cle, ps_le)
    private construction r_ss_te := line(cte, ss_te)
    axial angle(inlet + le_wedge / 2 + 90deg) r_ss_le
    axial angle(inlet - le_wedge / 2 - 90deg) r_ps_le
    axial angle(exit - te_wedge / 2 + 90deg) r_ss_te

    // the sides' centres, each on the far side of its side from the edge's centre, and the
    // suction side's two arcs meeting at `crest`, near its highest point
    c_fwd := point hint((ss_le.x + 0.45 * near.chord * sin(ss_in),
                         ss_le.y - 0.45 * near.chord * cos(ss_in)))
    c_aft := point hint((ss_te.x + near.r_aft * sin(ss_out), ss_te.y - near.r_aft * cos(ss_out)))
    c_ps := point hint((ps_le.x + 0.75 * near.chord * sin(ps_in),
                        ps_le.y - 0.75 * near.chord * cos(ps_in)))
    crest := point hint((near.mid_u - hc / 2 * sin(near.stagger) + 0.4 * hc * cos(near.stagger),
                         near.mid_v + hc / 2 * cos(near.stagger) + 0.4 * hc * sin(near.stagger)))

    // the profile, counter-clockwise from the trailing edge: each arc counter-clockwise from
    // its start but the pressure side's, which the walk takes from its end
    le := arc(center: cle, start: ss_le, end: ps_le) hint(r: near.r_le)
    pressure := arc(center: c_ps, start: ps_te, end: ps_le) hint(r: 1.5 * hc)
    te := arc(center: cte, start: ps_te, end: ss_te) hint(r: near.r_te)
    suction_aft := arc(center: c_aft, start: ss_te, end: crest) hint(r: near.r_aft)
    suction_fwd := arc(center: c_fwd, start: crest, end: ss_le) hint(r: 0.9 * hc)
    profile := face(pressure, te, suction_aft, suction_fwd, le)
    radius(r_le) le
    radius(r_te) te
    radius(r_aft) suction_aft

    // every joint smooth: it lies on the line through the two centres
    c_fwd coincident r_ss_le
    c_ps coincident r_ps_le
    c_aft coincident r_ss_te
    private construction te_ps := line(cte, ps_te)
    c_ps coincident te_ps
    private construction crest_line := line(c_aft, crest)
    c_fwd coincident crest_line
  }
}

// The section at span fraction `s` along the stacking axis `g`, square to it, every number of
// the design table blended through the hub's, the mean's and the tip's.
component Station(g: line, up: axis, hub: group, mean: group, tip: group, s: Scalar) {
  st := std.OnLine(g, up: up, s: s)
  wh := (1 - s) * (1 - 2 * s)
  wm := 4 * s * (1 - s)
  wt := s * (2 * s - 1)
  sec := Section(st.n, chord: wh * hub.chord + wm * mean.chord + wt * tip.chord,
                 stagger: wh * hub.stagger + wm * mean.stagger + wt * tip.stagger,
                 inlet: wh * hub.inlet + wm * mean.inlet + wt * tip.inlet,
                 exit: wh * hub.exit + wm * mean.exit + wt * tip.exit,
                 le_wedge: wh * hub.le_wedge + wm * mean.le_wedge + wt * tip.le_wedge,
                 te_wedge: wh * hub.te_wedge + wm * mean.te_wedge + wt * tip.te_wedge,
                 r_le: wh * hub.r_le + wm * mean.r_le + wt * tip.r_le,
                 r_te: wh * hub.r_te + wm * mean.r_te + wt * tip.r_te,
                 r_aft: wh * hub.r_aft + wm * mean.r_aft + wt * tip.r_aft,
                 mid_u: wh * hub.mid_u + wm * mean.mid_u + wt * tip.mid_u,
                 mid_v: wh * hub.mid_v + wm * mean.mid_v + wt * tip.mid_v, near: mean)
}

// ---- the airfoil ----
// the stacking axis, hub to tip; the family runs on past the hub into the platform
foot0 := point
tip0 := point
fix((0mm, 0mm, 0mm)) foot0
fix((0mm, 0mm, span)) tip0
construction centerline stack := line(foot0, tip0)
param s: Scalar over (-embed / span, 1)
airfoil := sweep(Station(stack, std.y, hub, mean, tip).sec.profile over s)

// ---- the platform and the seal wings: sectors of the annulus about the engine's axis ----
in std.front {
  // the engine's axis, `r_hub` below the flowpath
  e0 := point
  e1 := point
  fix((-30mm, -r_hub)) e0
  fix((30mm, -r_hub)) e1
  construction centerline engine := line(e0, e1)

  // the platform's section through the stacking axis, each side one number
  plat := step(u: aft, v: -platform_t) (underside := line) -> step(v: 0mm) (trailing := line) ->
          step(u: -fore) (flowpath := line) -> step(v: -platform_t) (leading := line) -> close

  // a wing fore and aft of the shank, each standing `wing_out` off it
  fwd_wing := step(u: -root_len / 2 + embed, v: -wing_z) (fw_under := line) ->
              step(v: -wing_z + wing_t) (fw_root := line) ->
              step(u: -root_len / 2 - wing_out) (fw_top := line) ->
              step(v: -wing_z) (fw_lip := line) -> close
  aft_wing := step(u: root_len / 2 + wing_out, v: -wing_z) (aw_under := line) ->
              step(v: -wing_z + wing_t) (aw_lip := line) ->
              step(u: root_len / 2 - embed) (aw_top := line) ->
              step(v: -wing_z) (aw_root := line) -> close
}
index := frame
index turns(1) engine
// each swept one pitch from the front plane, then turned back half a pitch to stand centred
platform_sector := solid(plat, about: engine, sweep: pitch_angle)
wing_f_sector := solid(fwd_wing, about: engine, sweep: pitch_angle)
wing_a_sector := solid(aft_wing, about: engine, sweep: pitch_angle)
platform := platform_sector at(under: index) -pitch_angle / 2
wing_f := wing_f_sector at(under: index) -pitch_angle / 2
wing_a := wing_a_sector at(under: index) -pitch_angle / 2

// ---- the shank and the fir tree: one section across the engine's axis, broached along it ----
in std.side {
  fir := step(u: neck) (crown := line) -> step(v: -shank_to) (shank_r := line) ->
         step(du: lobe, dv: -rise) (bear_r0 := line) -> step(dv: -land) (land_r0 := line) ->
         step(du: -lobe - taper, dv: -fall) (under_r0 := line) ->
         step(du: lobe, dv: -rise) (bear_r1 := line) -> step(dv: -land) (land_r1 := line) ->
         step(du: -lobe - taper, dv: -fall) (under_r1 := line) ->
         step(du: lobe, dv: -rise) (bear_r2 := line) -> step(dv: -land) (land_r2 := line) ->
         step(du: -lobe - taper, dv: -fall) (under_r2 := line) ->
         step(u: -foot) (base := line) ->
         step(du: -lobe - taper, dv: fall) (under_l2 := line) -> step(dv: land) (land_l2 := line) ->
         step(du: lobe, dv: rise) (bear_l2 := line) ->
         step(du: -lobe - taper, dv: fall) (under_l1 := line) -> step(dv: land) (land_l1 := line) ->
         step(du: lobe, dv: rise) (bear_l1 := line) ->
         step(du: -lobe - taper, dv: fall) (under_l0 := line) -> step(dv: land) (land_l0 := line) ->
         step(du: lobe, dv: rise) (bear_l0 := line) ->
         step(v: -platform_t + embed) (shank_l := line) -> close
}
root := solid(fir, from: -root_len / 2, to: root_len / 2)
// every corner of the section broken, both sides, top to bottom: each bearing face's edges and
// each neck
root.shank_r round(r: edge_r) root.bear_r0
root.bear_r0 round(r: edge_r) root.land_r0
root.land_r0 round(r: edge_r) root.under_r0
root.under_r0 round(r: edge_r) root.bear_r1
root.bear_r1 round(r: edge_r) root.land_r1
root.land_r1 round(r: edge_r) root.under_r1
root.under_r1 round(r: edge_r) root.bear_r2
root.bear_r2 round(r: edge_r) root.land_r2
root.land_r2 round(r: edge_r) root.under_r2
root.under_r2 round(r: edge_r) root.base
root.shank_l round(r: edge_r) root.bear_l0
root.bear_l0 round(r: edge_r) root.land_l0
root.land_l0 round(r: edge_r) root.under_l0
root.under_l0 round(r: edge_r) root.bear_l1
root.bear_l1 round(r: edge_r) root.land_l1
root.land_l1 round(r: edge_r) root.under_l1
root.under_l1 round(r: edge_r) root.bear_l2
root.bear_l2 round(r: edge_r) root.land_l2
root.land_l2 round(r: edge_r) root.under_l2
root.under_l2 round(r: edge_r) root.base

// ---- the blade ----
blade := solid(root)
platform union blade
wing_f union blade
wing_a union blade
airfoil union blade

// ---- the cooling holes, drilled the length of the blade ----
in std.top {
  first := point
  last := point
  fix((-9mm, 2mm)) first
  fix((6mm, -2mm)) last
  bend := point hint((-9.9, -31.5))
  construction drill_line := arc(center: bend, start: last, end: first)
  radius(hole_arc) drill_line
  repeat holes as i {
    c := point
    c coincident(t == i / (holes - 1)) drill_line
    bore := circle(center: c) hint(r: hole_d / 2)
    radius(hole_d / 2) bore
    drill := solid(face(bore), from: -root_depth - 1mm, to: span + 1mm)
    drill cut blade
  }
}
