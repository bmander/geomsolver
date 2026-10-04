/** Browser entry points are files or directories; solver cases also retain their individual keys. */
import { cases, type Case } from '../core/examples.js';

export type ExampleTarget = { kind: 'file'; path: string }
  | { kind: 'directory'; path: string; entry: string };
export interface Example extends Case { target: ExampleTarget }
const parts = ['cylinder', 'plate', 'piston', 'disc', 'flywheel', 'throttle'];

/** Projects the core's case library does not hold, read like the V-twin from the demo server or
 *  the packaged sources: their modules are theirs, not the library's. */
const projects: Example[] = [{
  label: 'Hypoid gear pair · swept solids', key: 'spiral_bevel',
  description: 'A 24-tooth pinion and a 48-tooth gear on square shafts 25 mm apart, laid out step '
    + 'by step from their pitch cones through the mean point (layout.sv). Each is its blank less '
    + 'a generating crown rolled through it at every tooth. Edit the tooth counts, module, the '
    + 'offset between the shafts or the spiral angle in configuration.sv; the solids refine in '
    + 'the background as you watch.',
  target: { kind: 'directory', path: 'spiral_bevel', entry: 'gears.sv' },
}, {
  label: 'Twist drill · flutes ground under a screw', key: 'twist_drill',
  description: 'A 10 mm two-flute drill: each flute is what a grinding wheel carried along a screw '
    + 'about the drill sweeps through the stock, so its section is the wheel\'s characteristic '
    + 'carried along the helix, not the wheel\'s own profile. A wider wheel grinds the body '
    + 'clearance behind each margin, two cones grind the 118° point and the shank is added after. '
    + 'Edit the helix, the wheels or the point in configuration.sv; drill.svd is the drawing.',
  target: { kind: 'directory', path: 'twist_drill', entry: 'drill.sv' },
}, {
  label: 'Pin wheel, generated · swept solid', key: 'lantern_generation',
  description: 'A pinion rolls against a wheel blank and its one pin cuts a tooth space. The roll\'s '
    + 'ratio is measured off the drawing after the solve (the wheel\'s pitch radius over the '
    + 'pinion\'s), so edit either radius and the cut is re-timed and refines as you watch.',
  target: { kind: 'file', path: 'lantern_generation.sv' },
}, {
  label: 'Skew axes · views solved in space', key: 'skew_axes',
  description: 'Two shafts that do not meet, one drawn in the front view and one in a side view whose '
    + 'fold nobody states: the shaft angle and the offset between them are relations in space, and '
    + 'the solve answers for the fold (DOF 0): orbit (right-drag) to see the side view folded '
    + 'under the front one; set `shaft_angle` to 60deg and it tilts to 30°, or change `offset` and '
    + 'the pinion shaft moves along the common perpendicular.',
  target: { kind: 'file', path: 'skew_axes.sv' },
}, {
  label: 'Hypoid pitch cones · spatial layout', key: 'hypoid_pitch_cones',
  description: 'A hypoid pair laid out through its mean point: square shafts 20 mm apart, the pinion\'s '
    + 'view solved, and the two pitch cones named and stated to touch at M (DOF 0). Orbit '
    + '(right-drag) to see them kiss on the pitch plane; set `E` to 0mm for a straight bevel, or '
    + 'edit the tooth counts. The primer\'s §2.12 builds the same pair without naming the cones, each '
    + 'axial view folded along its pitch generator.',
  target: { kind: 'file', path: 'hypoid_pitch_cones.sv' },
}, {
  label: 'Sphere, cone and cylinder · spatial relations', key: 'sphere_cone_cylinder',
  description: 'A line tangent to a shaft, a point on a ball and two cones touching at a point, each '
    + 'relation read in space between a front and a side view. One freedom is left, the point on the '
    + 'ball: drag it round the circle the side view cuts, and orbit (right-drag) to see the '
    + 'surfaces; edit the first cone\'s half-angle and the second reopens to keep touching.',
  target: { kind: 'file', path: 'sphere_cone_cylinder.sv' },
}, {
  label: 'Dimpled ring · a closed intersection', key: 'solid_dimpled_ring',
  description: 'A ball pressed into a torus off to one side. The two meet in one small loop that '
    + 'crosses neither solid\'s seam, so the kernel finds it by searching the faces. Orbit '
    + '(right-drag) to see the dimple; raise `sunk` to deepen it, or set it to 0mm and the bare touch '
    + 'is refused rather than built.',
  target: { kind: 'file', path: 'solid_dimpled_ring.sv' },
}];

export function exampleCases(): Example[] {
  return cases().filter((c) => !parts.some((p) => c.key === `vtwin_${p}`)).map((c): Example =>
    c.key === 'vtwin' ? { ...c, label: 'V-twin air engine',
      description: 'Assembly and six part drawings, with their editable models and shared components.',
      target: { kind: 'directory', path: 'vtwin', entry: 'assembly.svd' } }
    : { ...c, target: { kind: 'file', path: `${c.key}.svd` } }).concat(projects);
}

/** Keep old part links useful, and accept explicit source paths as file examples. */
export function exampleTarget(key: string): { key: string; target: ExampleTarget; file?: string } {
  const part = parts.find((p) => key === `vtwin_${p}`);
  if (part) return { ...exampleTarget('vtwin'), file: `vtwin/${part}.svd` };
  const stem = key.split(':')[0];
  const known = exampleCases().find((c) => c.key === stem);
  if (known) return { key, target: known.target };
  if (/^(?:[\w-]+\/)*[\w-]+\.svd?$/.test(key)) {
    return { key, target: { kind: 'file', path: key } };
  }
  throw new Error(`unknown example: ${key}`);
}
