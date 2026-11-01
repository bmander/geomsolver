# Changelog

What changes in each release of `cad-kernel`. Versions follow [semver](https://semver.org/):
until 1.0 a minor release may break the public API and a patch release never does.

Write each change under **Unreleased** as it lands. `scripts/release-cad-kernel.sh prepare X.Y.Z`
(in geomsolver) turns that section into the release's, and the release's notes are that section.

## Unreleased

- The generating class's rows asked of one placement (`swept::admission`): M2, E1–E4 and T1–T3 for
  a tool under a relative rotation (`check`), T2, E1 and S1–S4 under a constant twist
  (`check_screw`), sampled (`Sampling`), returning what they found (`Found`, `Capped`) and the
  blank's reads (`Reads`); `constant_twist::Row` gains the generating class's rows.
- The planar classes' readings: a closed curve's inner envelope under a planar motion
  (`swept::planar`, its curve a `Section` and its material a `Region`, read in a `View`) and the
  region a planar tool's section sweeps (`swept::region`), each refused by a typed row with a
  witness. A planar profile edge (`plane2::Edge`: a line, an arc, a conic) and its readers.
- A trajectory's pose alone (`Trajectory::pose`, `Pose`) and an interval bound on the speed it
  carries a point or a box back at (`Trajectory::inverse_point_speed_bound`, `_over`), read through
  a pace's value and enclosure (`Pace::value`, `Pace::enclose`) and a path's pose and stretch
  (`Path::pose`, `Path::stretch`, `Path::speed_over`, `Stretch`).
- A ball's tube along a trajectory (`swept::canal`): in closed form along a constant twist, a face
  family fitted to its disc along any other, refused by rows typed with what they saw where it
  would meet itself. The rotation-minimising frame's step (`trajectory::reflected`) and a face
  family's tolerance, longest piece and Lagrange weights (`family::EXACT`, `LONGEST_PIECE`,
  `weights`).
- Generating sweeps' contact times (`swept::generating`): a tool point's contact chart along a
  trajectory that turns it, its form read once (`Form`: a sinusoid in the roll under a turn or a
  turn seen from a turn, a line under a slide seen from a turn), its roots
  (`NormalVelocity::roots`), the contact times of a point or a surface over the roll (`Chart`), and
  a trajectory with no chart refused by what it is (`Uncharted`).
- A cutter sectioned at its stations (`swept::cutter`) and its stations' contact curves traced into
  a candidate sheet (`swept::trace`).
- Sweeps of a constant twist (`swept`): a tool read as its faces (`swept::tool::Tool`, faces of
  revolution in `swept::surface`) carried by a screw, a turn or a slide, the characteristic walked
  through a blank with its rows typed (`swept::constant_twist`), and its sheet fitted and judged to
  a tolerance (`swept::sheet`, `swept::helical`).
- A ring's contact under a motion (`contact::RingFrame`) and its roots on a chart of the ring
  (`contact::Sinusoid::roots_over`), and a moving surface point (`contact::SurfacePoint`,
  `contact::contact`).

## [0.1.0] - 2026-11-01

The first release: the kernel as geomsolver builds and tests it.

- Geometry: planes, cylinders, cones, spheres, tori, extrusions and revolutions of curves,
  rational and polynomial B-spline curves and surfaces, traced intersection curves.
- Topology: oriented edge uses with p-curves, seams, degenerate poles, a validity check.
- Construction: spheres, cones and tori by their numbers; prisms (drafted too), revolutions and
  canals of line, arc and spline profiles.
- Booleans: union, cut and common, closed-form intersections where one exists, else traced; a
  meeting the kernel cannot resolve is refused.
- Measures: exact volume and area.
- Meshing: constrained Delaunay refined to a measured sag.
- Output: STEP AP214 with p-curves and seam curves, and STL.
- Errors carry a `Kind`.
- Deterministic elementary functions (`util::fmath`): the same bits on every target.
