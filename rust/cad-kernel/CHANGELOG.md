# Changelog

What changes in each release of `cad-kernel`. Versions follow [semver](https://semver.org/):
until 1.0 a minor release may break the public API and a patch release never does.

Write each change under **Unreleased** as it lands. `scripts/release-cad-kernel.sh prepare X.Y.Z`
(in geomsolver) turns that section into the release's, and the release's notes are that section.

## Unreleased

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
