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
    + 'by step from their pitch cones through the mean point, in one file (hypoid.sv) whose words '
    + 'choose every root, with no point seeded. Each is its blank less a generating crown rolled '
    + 'through it at every tooth. Edit the tooth counts, module, the offset between the shafts or '
    + 'the spiral angle at its head; the solids refine in the background as you watch.',
  target: { kind: 'directory', path: 'spiral_bevel', entry: 'hypoid.sv' },
}, {
  label: 'Hypoid drive · the pair imported, to print', key: 'hypoid_drive',
  description: 'The hypoid pair above, imported whole (`use spiral_bevel.hypoid`) and restated '
    + 'nowhere, made a hand-cranked drive to 3D print: a frame inside the pair carrying a plain '
    + 'sleeve along each shaft, each member a part with a spoked hub seated on its blank\'s back '
    + 'cone, splined shafts pressed and glued into the hubs and the cranks, and each crank '
    + 'counterweighted so it spins without wobbling, its handle loose on its pin. Every part '
    + 'stands on the pair\'s own axes; seventeen claims check the fits and the clearances, the '
    + 'cranks through a whole turn.',
  target: { kind: 'directory', path: 'hypoid_drive', entry: 'drive.sv' },
}, {
  label: 'Twist drill · flutes ground under a screw', key: 'twist_drill',
  description: 'A 10 mm two-flute drill: each flute is what a grinding wheel carried along a screw '
    + 'about the drill sweeps through the stock, so its section is the wheel\'s characteristic '
    + 'carried along the helix, not the wheel\'s own profile. A wider wheel grinds the body '
    + 'clearance behind each margin, two cones grind the 118° point and the shank is added after. '
    + 'Edit the helix, the wheels or the point in configuration.sv; drill.svd is the drawing.',
  target: { kind: 'directory', path: 'twist_drill', entry: 'drill.sv' },
}, {
  label: 'Wankel rotor and housing · one planetary motion', key: 'wankel',
  description: 'A rotary engine from one motion and three numbers: the rotor\'s centre rides the '
    + 'eccentric while it turns a third as fast. The housing\'s bore is an apex\'s envelope, its '
    + 'path (the epitrochoid); the rotor is its blank less the housing turned about it over a '
    + 'whole period, a flank the bore\'s envelope under that turn (drawn as `flank`). A rotor of '
    + 'three arcs is claimed clear of the housing at every pose. Edit R, e and W in configuration.sv; '
    + 'wankel.svd is the drawing.',
  target: { kind: 'directory', path: 'wankel', entry: 'wankel.sv' },
}, {
  label: 'E-bike hub motor, 1 hp · an assembly of parts', key: 'hub_motor',
  description: 'A direct-drive, outer-rotor motor for a 26" rear wheel: 746 W at 255 rpm from 48 V. '
    + 'It has 46 magnets over 51 slots, and one slot, one magnet and one hall sensor are drawn and '
    + 'turned round in rings. The copper is the coils\' band less the lamination. Axle, carrier, '
    + 'covers and bearings are profiles turned in a half-section, and twelve claims check the fits, '
    + 'the 0.6 mm air gap among them. The sizing is worked in motor/dims.sv; hub_motor.svd is the '
    + 'drawing and the README says what each part is.',
  target: { kind: 'directory', path: 'hub_motor', entry: 'hub_motor.svd' },
}, {
  label: 'Pin wheel, generated · swept solid', key: 'lantern_generation',
  description: 'A pinion rolls against a wheel blank and its one pin cuts a tooth space. The roll\'s '
    + 'ratio is measured off the drawing after the solve (the wheel\'s pitch radius over the '
    + 'pinion\'s), so edit either radius and the cut is re-timed and refines as you watch.',
  target: { kind: 'file', path: 'lantern_generation.sv' },
}, {
  label: 'Penrose tiling · components calling each other', key: 'penrose',
  description: 'Penrose\'s P3 by Robinson triangles: an acute half rhomb cuts into an acute and an '
    + 'obtuse one, an obtuse into two obtuse and an acute, each a golden ratio smaller, and two '
    + 'components each call the other until a `when` over the count ends them. A sun of ten '
    + 'triangles three levels down is 130 half rhombs, every point placed by distances alone. '
    + 'Edit `depth` or `l`.',
  target: { kind: 'file', path: 'penrose.sv' },
}, {
  label: 'Generation by motion · fourteen examples', key: 'generation',
  description: 'Curves and solids nobody draws: what a tool leaves behind as it moves, or the path a '
    + 'mechanism traces, each a curve the solve can constrain. A Jansen leg whose toe rod solves to '
    + 'stand on level ground, the same leg bending at a stated radius where it stands '
    + '(osculating_stride.sv), a dwell linkage fitted by curvature, a heart cam and cams cut from '
    + 'their followers\' laws (with dwells, a swinging arm, a flat face, a base circle solved '
    + 'against the cam\'s curvature, a plate as a solid), conjugate cams, '
    + 'clock gearing, a spur mesh cut twice over, a rack-cut spur gear exported exactly, and the '
    + 'same gear whose pressure angle a gauge in space solves (gauged_gear.sv). The README says '
    + 'what each shows.',
  target: { kind: 'directory', path: 'generation', entry: 'stride.sv' },
}, {
  label: 'Mechanisms · followed over a parameter', key: 'mechanisms',
  description: 'Linkages driven by a number that runs: the crank\'s angle declared as '
    + '`param t: Angle over (0deg, 360deg)`, every relation held for every value of it, and the '
    + 'drawing followed round by continuation on the assembly mode it is drawn in. A four-bar\'s '
    + 'coupler curve over a whole turn, a slider-crank straight through its dead centres, a triple '
    + 'rocker stopped where its motion turns back, a four-bar stopped at its change point, '
    + 'where two branches cross, and a cam whose roller rides the profile cut from its law. The '
    + 'README says what each shows.',
  target: { kind: 'directory', path: 'mechanisms', entry: 'four_bar.sv' },
}, {
  label: 'Rounds and chamfers · written in Solvent', key: 'rounds',
  description: 'A block with one rim rounded and the other chamfered, a pocket rounded inside, a '
    + 'boss filleted where it stands, and a disc whose round and chamfer are built exactly. None '
    + 'is a kernel feature: `round` and `chamfer` are words of the standard library over faces '
    + 'read as sets, and by_hand.sv writes its own round in twenty lines. regions.sv shows solids '
    + 'that are sets, each built exactly: an egg by its foci, a bicylinder, a Voronoi cell. Then the fillet stack\'s parts: a cast pad, a turned '
    + 'knob, a rail, a pipe tee\'s crotch, a drilled pipe\'s rim, tangent chains round a slot and '
    + 'a pocket, rounds that run out onto a face or into a post, the seams of two rods made one, '
    + 'a D-shaped shaft rounded all over, a groove rounded all round, a slab drilled twice and '
    + 'half a frustum rounded all over, a tee\'s crotch chamfered, a boss rounded across a '
    + 'bullnose\'s seam, chamfers spilled over a seam onto a rim\'s round and a chamfer\'s '
    + 'edges rounded by a round that names it, all built '
    + 'exactly too. '
    + 'The README says what each shows; the surfaces refine in the background as you watch.',
  target: { kind: 'directory', path: 'rounds', entry: 'block.sv' },
}, {
  label: 'Lofts and sweeps · written in Solvent', key: 'lofts',
  description: 'Fourteen solids swept by a face of a component as one of its numbers runs: a pipe '
    + 'round a U-bend, a twisted duct and an auger, a taper and a swell, a thread, a flight and a '
    + 'spring, a ribbon wound in a frame that does not twist, a bottle through three sections ruled and smooth, squares on rails with no guide, '
    + 'two sketches drawn apart lofted corner to corner (and a square to a triangle, and to a '
    + 'point), a blade held to two rails and a profile to three, and a square to a circle. None '
    + 'is a kernel feature: each station is the writer\'s component over the standard library\'s '
    + 'words, every wall is built exactly, and every volume is a closed form the file gives. The '
    + 'README says what each shows; the solids refine in the background as you watch.',
  target: { kind: 'directory', path: 'lofts', entry: 'pipe.sv' },
}, {
  label: 'Rates · curves integrated in Solvent', key: 'rates',
  description: 'Five curves stated by how their points move rather than where they are: a point '
    + 'dragged by a string (the tractrix), a railway transition whose curvature grows with its '
    + 'length (the clothoid) eased into its bend, a hanging chain shot to its far end beside the '
    + 'free curve that hangs the same, a dog chasing a hare until it catches it, and a filament '
    + 'wound on a cone — and five fronts moving square to themselves, surfaces marched by the '
    + 'method of lines: an ellipse\'s moving inward until it folds, geodesic circles on a sphere '
    + 'closing on its antipode and on a cone, a Wankel bore, the trochoid\'s parallel curve, '
    + 'refused past its waist, and light spreading through Maxwell\'s fish-eye lens at a speed '
    + 'read where it is, refocused at the far rim. Each body states a rate, `rate(p over s)`, and '
    + 'is integrated from where it is drawn; each is a closed form its file gives. The README says '
    + 'what each shows.',
  target: { kind: 'directory', path: 'rates', entry: 'tractrix.sv' },
}, {
  label: 'Developable panels · sheets cut flat and bent', key: 'developable',
  description: 'Panels a plane touches along each straight ruling, so they come off a flat sheet '
    + 'without stretching: a conical reducer, an offset and a tilted duct adapter, a mitred duct, '
    + 'a rectangle to a round (four planar triangles and four fans about the corners), an offset '
    + 'square-to-round whose four triangles all differ, and a whole dory, its bottom and both '
    + 'sides, the sides fanning at the stem and the transom, laid out on two sheets of plywood. '
    + 'Each panel is `std.panels.Developable(a, b)`, its sheet a band of it (`distance(<= t, '
    + 'side: outside)`), and its pattern `std.panels.Development`, laid in a plane and placed by '
    + 'relations; each is a closed form its file gives. dory.svd and offset_transition.svd are cut files; '
    + 'the README says what each shows.',
  target: { kind: 'directory', path: 'developable', entry: 'frustum.sv' },
}, {
  label: 'Paths in space · guides that leave the plane', key: 'paths',
  description: 'Seven solids swept along guides drawn in space: a square wire wound on a helix '
    + 'written by its ordinates, a ribbon in Frenet\'s frame facing its helix\'s axis beside one '
    + 'in the frame that does not twist, a pipe bent in two planes carrying a bar\'s frame to its '
    + 'end cap, a line routed through space with no plane written, an S-bend whose right turn is an '
    + 'arc walked from its end, a cable through clip points between two connectors, and a flange '
    + 'square to a leaning stub. Each guide is a curve of the drawing — a traced curve, a spline '
    + 'over points in space, a chain of lines and arcs across planes — and each volume is a closed '
    + 'form its file gives. The README says what each shows.',
  target: { kind: 'directory', path: 'paths', entry: 'coil.sv' },
}, {
  label: 'Connecting rod, forged · rounded in Solvent', key: 'conrod',
  description: 'Two eyes joined by a tapering I-section shank: the shank filled into both eyes, a '
    + 'pocket each side of the web rounded inside, the bores chamfered and the outside\'s edges '
    + 'rounded, each one statement of the standard library\'s `round` and `chamfer` over the faces '
    + 'the rod\'s solids gave it, the sketch only the sharp outline. Built exactly. The comments '
    + 'say which edges are left sharp, and why: rounds of separate statements meeting where each '
    + 'reads the rod before the other. Edit any number and apply; rod.svd is the drawing.',
  target: { kind: 'directory', path: 'conrod', entry: 'rod.sv' },
}, {
  label: 'Sundials · hour lines from the sun', key: 'sundial',
  description: 'A horizontal dial and a polar one, their hour lines found rather than tabled: the '
    + 'equinox sun is a ring of 24 hourly places about the style (sun.sv), and each hour line is '
    + 'where the plane through the style and that hour\'s sun meets the dial. The horizontal dial '
    + 'comes out at tan θ = sin(latitude) · tan(h) and marks midsummer\'s first and last light; '
    + 'the polar dial, the one for the equator, at height · tan(h). Edit `latitude`, `hours` or '
    + '`height`, and orbit (right-drag) to see the sun\'s ring square to the style.',
  target: { kind: 'directory', path: 'sundial', entry: 'horizontal.svd' },
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
    + 'edit the tooth counts. The test fixture of the same name builds the pair without naming the '
    + 'cones, each axial view standing on its pitch generator.',
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
