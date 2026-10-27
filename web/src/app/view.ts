/* The sketch view: the object the whole front end holds, and the state every part of it reads
 * — the camera, the selection, the tool, the compiled plan and the last diagnosis.  What is
 * *done* to it lives in the modules beside this one: `paint` strokes it, `gesture` drives it
 * from the pointer, `tools` draws into it, `solids` makes solids and datums of it, `extent`
 * sizes what they made, `dimension` writes a number on it and `edit` changes the document.
 * They take the view as their first argument, so this file stays the state and
 * the seams — the history, the solve, and the two one-line questions everything else asks:
 * where a world place is on the canvas (`camera`) and what is under the cursor (the core).
 * Neither answer is worked out here: the camera is the front end's only linear algebra and the
 * geometry is all the core's, which is what keeps the two apart.
 *
 * **The view is the workspace**: one scene in space, every sketch standing on its own plane and
 * solids drawn under it by three.js (`box3d.ts`).  Each plane's page reaches the screen through
 * one affine map (`core/workspace.ts`, composed by `camera.ts`), so the painter, the tools and the
 * gestures read and write page coordinates as they always did — through the camera of the view
 * they are working in — and what is under the pointer is asked of the core where the eye sees it.
 *
 * Every mutation funnels through `afterEdit`, which re-solves (when auto-solve is on),
 * re-diagnoses and notifies the shell exactly once. */
import { Box3D } from './box3d.js';
import { objects } from '../core/mesh.js';
import { ObjectBuilds, type BuildRow, type PoolWorker } from './object-builds.js';
import { pageStore, type SurfaceStore } from './surface-store.js';
import { coreIdentity } from '../core/wasm.js';
import { ClaimJudging, type Judging } from './claim-judging.js';
import { ScenarioReading, type Drawn, type Reading } from './scenario-view.js';
import type { ScenarioMark } from '../core/scenario.js';
import * as dim from '../core/callout.js';
import { Constraint } from '../core/constraints.js';
import { PlanResult, PlanSolver, asSolveResult } from '../core/decompose.js';
import { Diagnosis, diagnose } from '../core/diagnose.js';
import * as io from '../core/io.js';
import { Param, Plane, Point, Primitive, Sketch, toWire } from '../core/model.js';
import * as params from '../core/parameters.js';
import type { March, Parameter } from '../core/parameters.js';
import { Document, Edit, fromSketch } from '../core/program.js';
import type { BodyWord, Region, Scenario, ScenarioAnswer } from '../core/program.js';
import { Method, SolveResult, System } from '../core/system.js';
import type { Item } from '../core/overview.js';
import { Motion, WitnessReport, analyze } from '../core/witness.js';
import { Camera, ViewCam } from './camera.js';
import * as edit from './edit.js';
import * as extent from './extent.js';
import * as hidden from './hidden.js';
import { setCut } from '../core/section.js';
import type { Frame3 } from '../core/section.js';
import type { Row } from '../core/outline.js';
import type { Handle, LiveExtent } from './extent.js';
import * as dimension from './dimension.js';
import { abandonGesture, bindEvents, takeHit } from './gesture.js';
import type { Gesture } from './gesture.js';
import type { DimAlt, LiveDim } from './dimension.js';
import { paint } from './paint.js';
import * as solids from './solids.js';
import * as tools from './tools.js';
import * as underlay from './underlay.js';
import type { Bitmap, Underlay } from './underlay.js';
import {
  NOWHERE, PAGE, boundsSeen, calloutSeen, lookOf, maps, nearestSeen, ofView, panesSeen, pickSeen,
  pickEdgeSeen, pickSolidSeen, placeOf, spacePoints, underSeen, workspace,
} from '../core/workspace.js';
import type { View, Workspace } from '../core/workspace.js';

/** **How the workspace is sectioned**: every object cut by the plane named `plane` — one of the
 *  document's, or a standard plane it does not have yet — moved `offset` (the drawing's unit)
 *  along that plane's normal, the side the normal points to taken away, or the other side where
 *  `flip`. */
export interface Sectioning {
  plane: string;
  flip: boolean;
  offset: number;
}

/** **What of an object is picked**: which object (its name, and its index in the current sketch)
 *  and the part of it — a face, named where it was made; an edge, by the two faces meeting
 *  there; or, with neither, the object whole, a body. */
export interface SolidPick {
  name: string;
  index: number;
  /** The face picked, or '' for an edge or the body whole. */
  face: string;
  /** The edge picked: the two faces meeting there, by the paths its mesh names them by. */
  edge?: [string, string];
  /** Where in space the click met the face or the edge, for a sketch on it to start at. */
  at?: [number, number, number];
}

/** Which part of an object a pick is. */
export function partOf(p: SolidPick): 'body' | 'face' | 'edge' {
  return p.edge ? 'edge' : p.face ? 'face' : 'body';
}

/** Whether two picks are of the same part of the same object. */
export function samePick(a: SolidPick, b: SolidPick): boolean {
  return a.index === b.index && a.face === b.face
    && (a.edge ?? []).join('\n') === (b.edge ?? []).join('\n');
}

/** The object a pick is of, picked whole. */
export function bodyOf(p: SolidPick): SolidPick {
  return { name: p.name, index: p.index, face: '' };
}

/** Each object the picks are of once, whole, in the order its first part was picked. */
export function objectsPicked(picks: readonly SolidPick[]): SolidPick[] {
  const bodies: SolidPick[] = [];
  for (const s of picks) {
    if (!bodies.some((b) => b.index === s.index)) bodies.push(bodyOf(s));
  }
  return bodies;
}

/** `list` with `x` toggled in it (`same` saying which are one), or — not adding — `x` alone:
 *  what a click and a shift-click do to a selection. */
export function toggled<T>(list: readonly T[], x: T, add: boolean,
                           same: (a: T, b: T) => boolean = (a, b) => a === b): T[] {
  if (!add) return [x];
  return list.some((y) => same(y, x)) ? list.filter((y) => !same(y, x)) : [...list, x];
}

/** What a pick is called — its part, and the part's name: `face` `block.top`, `edge`
 *  `block.near · block.north`, `body` `block` — a part named where it was made, on another
 *  object, saying which (`stock.top on body`). */
export function pickName(p: SolidPick): { kind: string; name: string } {
  const on = (path: string) => path.split('.')[0] === p.name ? '' : ` on ${p.name}`;
  const kind = partOf(p);
  return {
    kind,
    name: p.edge ? `${p.edge[0]} · ${p.edge[1]}${on(p.edge[0])}`
      : p.face ? `${p.face}${on(p.face)}` : p.name,
  };
}

/** What a hit is called in a list to choose among: what kind of thing it is, and its name — the
 *  source's, for something drawn — and whether it is behind the material. */
export function hitName(doc: Document, hit: Hit): { kind: string; name: string; hidden: boolean } {
  if (hit.kind === 'entity') {
    return { kind: toWire(hit.ent.kind), name: doc.nameOf(hit.ent) ?? '(unnamed)', hidden: false };
  }
  return { ...pickName(hit.pick), hidden: !!hit.hidden };
}

/** **One thing under the pointer** that a click could take: something drawn, or a part of an
 *  object — `hidden` where it is an edge behind the material, listed only to be chosen. */
export type Hit =
  | { kind: 'entity'; ent: Primitive }
  | { kind: 'solid'; pick: SolidPick; hidden?: boolean };

/* What is being sized, and its arrow, belong to `extent`; the view holds them. */
export type { Handle, LiveExtent } from './extent.js';

/* A dimension being written belongs to `dimension`, but it is the view a caller holds, so the
 * two types are published from here as well. */
export type { DimAlt, LiveDim } from './dimension.js';

export const PICK_PX = 8;
/** The same tolerance for a finger: a fingertip covers some 7–10 mm of glass, and the thing it
 *  means is anywhere under it — so a tap reaches as far as half the 44-pixel target a touch
 *  screen's guidelines ask of a control.  Nearest still wins within it. */
export const TOUCH_PICK_PX = 22;

/** The planes every workspace offers, whether or not the document has them yet — a CAD part's
 *  origin planes.  `std.front` is the page itself seen in space, so geometry drawn on it with no
 *  `use std` stays on the page; the other two bring the library in the first time they are drawn
 *  on (`ensurePlane`). */
export const STANDARD_PLANES = ['std.front', 'std.side', 'std.top'] as const;

/** A document of nothing but `use std`, read once: where the standard planes are for a document
 *  that does not have them yet, so the eye can be turned to one before it is drawn on. */
let stdDoc: Document | null = null;
function standardDocument(): Document {
  return stdDoc ??= Document.read('use std\n');
}

/** The eye a flat drawing opens with — the front, square on — and the three-quarter view a
 *  document with solids or geometry off the front plane opens with. */
const FRONT = { az: -Math.PI / 2, el: 0 };
const THREE_QUARTER = { az: -Math.PI / 3, el: Math.PI / 6 };
/** How long the eye takes to swing square on to a chosen plane. */
const LOOK_MS = 260;

/** How long after an edit, with the page otherwise quiet, its parameters are followed where the
 *  page cannot say when it is idle (#412). */
const IDLE_FOLLOW_MS = 200;

const ANIM_DT = 0.03;        // seconds per animation tick
const ANIM_PERIOD = 2.0;     // seconds spent on each degree of freedom

interface Animation {
  modes: Motion[];
  /** The sketch it started on: the animation borrows that sketch's values and puts them back,
   *  and neither the tick nor the restore may touch a sketch that replaced it. */
  sketch: Sketch;
  x0: Float64Array;
  free: Int32Array;
  amp: number;
  labels: string[];
  t: number;
  showing: number;
}

/** A place a click asked for, and the Point it came from if it came from one. */
export interface Place {
  at: [number, number];
  on: Point | null;
}

export type Tool =
  'select' | 'point' | 'line' | 'rect' | 'circle' | 'arc' | 'arc3' | 'spline' | 'splinefit'
  | 'extrude' | 'revolve' | 'union' | 'cut' | 'common' | 'fillet' | 'chamfer';

/** Whether a tool makes a solid of the closed region a click is inside (Extrude, Revolve): it
 *  washes the region under the pointer, and draws no snap. */
export function sweepsRegion(tool: Tool): boolean {
  return tool === 'extrude' || tool === 'revolve';
}

/** The body word a Boolean tool writes (Union, Cut, Intersect — #162 F4), or null for any other
 *  tool.  Intersect is the body rule's `common`. */
export function combines(tool: Tool): BodyWord | null {
  return tool === 'union' || tool === 'cut' || tool === 'common' ? tool : null;
}

/** The library word an edge tool writes along a solid's edges (Fillet, Chamfer), or null for any
 *  other tool. */
export function roundsEdges(tool: Tool): EdgeWord | null {
  return tool === 'fillet' ? 'round' : tool === 'chamfer' ? 'chamfer' : null;
}

/** `std.rounds`' two words over a solid's edges: rounded by a ball, or cut by a flat. */
export type EdgeWord = 'round' | 'chamfer';

/** What the Fillet or Chamfer tool has picked of one object: whole faces (every edge of each) and
 *  edges (the two faces meeting there), by the paths its mesh names them by, in click order — or
 *  the body `whole`, every edge of it. */
export interface Rounding {
  solid: number;
  name: string;
  faces: string[];
  edges: [string, string][];
  whole: boolean;
}

/** **What a click with a tool down picks** — the one statement of it, read by the hover, a
 *  ⌘-click's list and the selection a tool is taken up over: anything with the select tool, an
 *  object's edges and faces (and a body whole) with an edge tool, bodies with a Boolean, and
 *  nothing with a tool that draws. */
export function picksAs(tool: Tool): 'all' | 'parts' | 'bodies' | null {
  return tool === 'select' ? 'all' : roundsEdges(tool) ? 'parts' : combines(tool) ? 'bodies'
    : null;
}

/** A Boolean about to be written: the body it is written on, its word and the tools it joins. */
export interface Combining {
  target: SolidPick;
  word: BodyWord;
  tools: SolidPick[];
}


export class SketchView {
  /** **The document.**  The source somebody wrote, the drawing it elaborates to, and where each
   *  part of the drawing was written.  Every edit is an edit of this text; the drawing is what
   *  the text came to, and is replaced whole whenever the text changes structurally. */
  doc: Document;
  /** Where the drawing sits on the canvas — and the whole of the front end's linear algebra:
   *  every world/screen conversion in `app/` goes through it (see `camera.ts`). */
  readonly cam = new Camera();
  tool: Tool = 'select';
  method: Method = 'dogleg';
  autoSolve = true;
  usePlan = false;
  colorByState = true;
  /** Paint the dimensioned constraints on the drawing as callouts.  View state, like the
   *  colouring: a document change leaves it as it was.  The model canvas's callouts are the
   *  editor's, and separate from what a `.svd` asks to be annotated on paper. */
  showDimensions = true;
  /** **Where the eye stands**, in radians: a bearing about the vertical and an elevation above
   *  the horizon.  The whole of how the workspace is looked at — the three.js camera is set from
   *  it (`box3d.ts`) and the core projects each plane for it (`core/workspace.ts`).  `az = -π/2,
   *  el = 0` is the front seen square on, where a flat drawing opens.  View state: never saved,
   *  exported, solved or undone, and a document change leaves it where it was. */
  orbit = { ...FRONT };
  /** A picture to trace over, in world coordinates — see `underlay.ts`.  Handled like anything
   *  else on the canvas (clicked, dragged, deleted) but **not document state**: it is scenery,
   *  and only its frame answers a press until it is selected, which is what lets the drawing be
   *  made straight through it. */
  underlay: Underlay | null = null;

  /** What is selected.  Written through a setter on purpose: selecting geometry is the other
   *  half of the picture's exclusivity, and there are eleven sites that assign this.  Enforced
   *  here, paste, a rubber band and a press on an entity all inherit the rule; enforced at each
   *  of them, it is a rule stated nowhere they can see and the failure is silent — a Delete
   *  that takes the photograph instead of what was just pasted. */
  get selected(): Primitive[] { return this._selected; }
  set selected(prims: Primitive[]) {
    this._selected = prims;
    // assigning the drawing's selection, even to nothing, lets the solids go: a press on a
    // callout or on the constraint list selects that instead
    this._solids = [];
    if (prims.length) this.dropImage();
    // picking a view is choosing where to draw: a plane selected on its own becomes the current
    // one, and stays so past the selection — the point drawn next is in it
    if (prims.length === 1 && prims[0] instanceof Plane) {
      this.plane = prims[0];
      this.openSpace = false;
    }
  }
  private _selected: Primitive[] = [];
  /** **The outline's rows hidden** (#435), by the key each is carried by (`hidden.rowKey`): view
   *  state, as the orbit is — never saved, solved or undone, carried across an edit by path and
   *  let go by a load.  What they made is `hiddenEnts` and `hiddenSolids`, set on every drawing
   *  the view is handed (`hidden.applyHidden`), each time bumping `hiddenEpoch`. */
  hiddenRows = new Set<string>();
  hiddenEnts = new Set<Primitive>();
  hiddenSolids = new Set<number>();
  hiddenEpoch = 0;
  /** **The workspace sectioned** (`core/section.ts`, `Sectioning`).  View state, as the orbit
   *  is: never saved, solved or undone, set on every drawing the view is handed
   *  (`applySection`), carried across an edit by the plane's name, let go when the name no longer
   *  reaches a plane and by a load.  `sectionEpoch` counts each change, so the box re-cuts. */
  section: Sectioning | null = null;
  sectionEpoch = 0;
  /** The solids selected, each by the face the click landed on — the third selection, exclusive
   *  with the other two for the picture's reason: a solid is not a `Primitive`, nothing
   *  constrains one beside a point, and holding both would leave Delete ambiguous. */
  get selectedSolids(): readonly SolidPick[] { return this._solids; }
  private _solids: SolidPick[] = [];
  /** **The current plane**: the view every point a tool mints is drawn in, or null for the
   *  page.  A proxy of the current sketch, carried across a re-elaboration by name like the
   *  selection, and let go when the name no longer resolves — a deleted plane, or a load. */
  plane: Plane | null = null;
  /** A standard plane chosen to draw on that the document does not have yet — it says no
   *  `use std` — named until the first press of a tool brings it in (`ensurePlane`). */
  pendingPlane: string | null = null;
  /** **Drawing in space** — the chooser's None: no plane is current, and every point a tool mints
   *  stands in space, in no view, where the eye sees the click (`Sketch.pointSeen`).  View state,
   *  like the current plane: kept across an edit and an undo, let go by a load. */
  openSpace = false;
  highlight: Primitive[] = [];
  pending: Point[] = [];
  /** Where the fit tool has been told the curve must pass, before there is a curve.  Places
   *  rather than Points, so the tool leaves nothing in the sketch if it is abandoned; one that
   *  came from a Point becomes a constraint once there is a curve to name. */
  pendingFit: Place[] = [];
  diagnosis: Diagnosis | null = null;
  /** What each load scenario says of the drawing as it stands, read again after every solve: a
   *  requirement refuted is a word of the footer's, as a claim about solids is.  Read in a
   *  worker with the picture (`scenarioRead`), so it lags the solve by the reading. */
  scenarios: Scenario[] = [];
  /** What each mass and volume the drawing states comes to (#510, `mass(10kg) arm`), read with
   *  the scenarios: a claim refuted is a word of the footer's too. */
  measures: ScenarioAnswer[] = [];
  /** **The scenario drawn over the workspace** (#510), by name — '' for none: its supports and
   *  loads (`scenarioMarks`, read at once on every edit) and how the drawing moves under it
   *  (`picture`, read in a worker).  View state, as the plane chosen is: it writes nothing,
   *  solves nothing, is kept across an edit by its name and let go by a load. */
  scenario = '';
  /** Where the scenario drawn falls on the drawing: a mark for each of its lines. */
  scenarioMarks: ScenarioMark[] = [];
  /** How the drawing moves under the scenario drawn, once the worker has read it; null before,
   *  for none, and for a drawing that has moved since. */
  picture: Drawn | null = null;
  /** The reading as it stands: whether one is out, and why it failed. */
  scenarioReading: Reading | null = null;
  /** The factor the picture is drawn at, as the card's slider sets it; null for the picture's
   *  own (`Drawn.scale`). */
  scenarioScale: number | null = null;
  /** Whether the picture swings from rest to its factor and back, and whether the mesh that was
   *  solved is drawn over it. */
  scenarioAnimate = false;
  scenarioMesh = false;
  /** Bumped whenever the marks or the picture change, so the box rebuilds them only then. */
  scenarioEpoch = 0;
  /** The constraint the shell has the focus on, so its callout can say so. */
  litConstraint: Constraint | null = null;
  lastResult: SolveResult | null = null;
  lastPlan: PlanResult | null = null;

  /** **The instant the drawing is shown at** (#398 L): a parameter it declares and where along it
   *  — `at` null at home, where it is drawn.  Presentation, as a wobble is: what is drawn moves
   *  and the document does not.  An edit is made at home and the drawing stood back here after it
   *  (`afterEdit`); no seed is written from an instant (the core refuses).  View state: kept
   *  across an edit by the parameter's name, let go by a load, never saved. */
  instant: { name: string; at: number | null } | null = null;
  /** How the drawing runs over each parameter it declares, as the core followed it since the
   *  last edit — the timeline's reach and its singular instants — or null where it has not been
   *  followed since (#412): an edit returns without following anything, and the march is taken
   *  when it is needed (`marches`) or once the page is idle after the edit. */
  private followed: March[] | null = null;
  /** The idle follow an edit asked for, while it is pending. */
  private idleFollow: { cancel: () => void } | null = null;

  onChanged: () => void = () => {};
  /** The instant shown moved, or the parameters and their marches did (an edit, a load). */
  onTimeline: () => void = () => {};
  /** The active tool changed (including when Escape backs out of one). */
  onTool: (tool: Tool) => void = () => {};
  /** Orbit mode was put up or down — see `setOrbitMode`. */
  onOrbitMode: (on: boolean) => void = () => {};
  /** A pointer interaction changed the canvas selection — so the canvas now has the focus,
   *  and whatever else was focused (a constraint row) no longer does. */
  onSelect: () => void = () => {};
  /** The source changed — a structural edit, a seed writeback, an undo.  What the panel is
   *  wired to, and **never** `onDragFrame`: a drag writes the source once, when it is let go. */
  onProgram: () => void = () => {};
  /** A different document was opened, so source navigation starts at its main file. */
  onLoad: () => void = () => {};
  /** Fired when what the outline hides changes, so its eyes follow. */
  onHidden: () => void = () => {};
  /** Fired when the section is turned on or off, moved or flipped, so its bar follows. */
  onSection: () => void = () => {};
  /** Per gesture frame: the sketch's structure and the constraint list are unchanged, so
   *  only the status line needs updating (a drag's numbers, a band's selection count). */
  onDragFrame: () => void = () => {};
  onStatus: (msg: string) => void = () => {};
  /** The objects being built in the background (`ObjectBuilds`), as each job moves: the build
   *  bar follows (#672). */
  onBuilds: (rows: BuildRow[]) => void = () => {};
  /** The claims about solids being judged in the background, as the judging moves; null once the
   *  drawing has none. */
  onClaims: (judging: Judging | null) => void = () => {};
  /** The load scenarios and the drawing's measures were read again, after a solve (`scenarios`,
   *  `measures`). */
  onScenarios: (scenarios: Scenario[], measures: ScenarioAnswer[]) => void = () => {};
  /** The scenario drawn, its marks or its picture changed: the chooser and its card follow. */
  onScenarioView: () => void = () => {};
  /** A dimension callout was clicked: the shell puts the focus on that constraint. */
  onPickConstraint: (c: Constraint) => void = () => {};
  /** What is held changed, and nothing else did — the line that names it is the whole of the
   *  consequence.  Narrower than `onChanged`, which re-reads every constraint and every
   *  expression out of the core and re-renders the program overlay: fading the traced picture
   *  is a keypress that auto-repeats, and it moves one number in one string. */
  onPicked: () => void = () => {};
  /** And double-clicked: the shell opens its value for editing. */
  onEditConstraint: (c: Constraint) => void = () => {};
  /** A dimension is being written, and this is where its number is on screen — the shell puts
   *  an editor there.  Called again whenever the callout moves or turns into another kind, and
   *  with nulls when there is no longer one being written. */
  onDimension: (live: LiveDim | null, at: [number, number] | null) => void = () => {};

  /** The dimension being written, if any — see `startDimension`. */
  liveDim: LiveDim | null = null;
  /** An extrusion is being sized, and this is where its number is on screen — the shell puts an
   *  editor there; nulls when there is none. */
  onExtent: (live: LiveExtent | null, at: [number, number] | null) => void = () => {};
  /** **Orbit mode**: a press with the primary button — one finger, on a phone — turns the
   *  workspace about rather than reaching the tool or the selection, so the view can be turned
   *  with the one gesture a touch screen has.  View state, toggled from the workspace's corner;
   *  a pinch still zooms and pans.  Written through `setOrbitMode`. */
  orbitMode = false;
  /** The extrusion being sized, if any — see `extrudeAt`. */
  liveExtent: LiveExtent | null = null;
  /** The closed region under the pointer while the Extrude tool is down: what a click there
   *  would extrude, washed on the drawing, or why it could not be. */
  hoverRegion: Region | { refused: string } | null = null;
  /** The region the Revolve tool's first click found, waiting for the line or axis to turn it
   *  about. */
  pendingRevolve: Region | null = null;
  /** **What the Fillet or Chamfer tool would write over** — the selection's faces and edges of
   *  one object, a body picked whole for every face of it — or null.  The tool's clicks are the
   *  selection's, so what was selected before it was taken up is where it starts. */
  get rounding(): Rounding | null { return solids.roundingOf(this); }
  /** **What a click would take, under the pointer** — streamed as it moves (`gesture::hover`) and
   *  lit where it is drawn, the drawing's in the highlight's ink and an object's part in the box —
   *  or a row of the list a ⌘-click opened, while the pointer is on it.  View state. */
  hovered: Hit | null = null;
  /** **Several things are under a ⌘-click**: the shell lists them at `at` (canvas coordinates) to
   *  choose among, each row lit as it is pointed at (`hoverHit`), the one chosen taken as a click
   *  on it would be (`takeHit`, with `add` the click's shift). */
  onChoose: (hits: Hit[], at: [number, number], add: boolean) => void = () => {};
  /** **A right-click that did not turn the workspace**: the shell opens a menu at `at` (canvas
   *  coordinates) on what it was on (`contextAt`), or on nothing — Hide, and Show all while
   *  anything is hidden. */
  onContextMenu: (hit: Hit | null, at: [number, number]) => void = () => {};
  /** The size last written by the Fillet or Chamfer tool, or typed and left, by its word: where
   *  the next starts.  View state, never saved. */
  roundSize: Record<EdgeWord, string> = { round: '', chamfer: '' };
  /** Edges or faces are picked for rounding, and this is where its number is asked on screen, and
   *  what it shows — the shell puts an editor there; nulls when there is none. */
  onRound: (r: Rounding | null, at: [number, number] | null, size: string) => void = () => {};
  /** Where the click that last picked for the Fillet or Chamfer tool was, on the canvas. */
  roundClick: [number, number] | null = null;


  /* -- the view's own workings.  What is not `private` here is read by the modules beside
   * this file — they are as much the view as this class is, and TypeScript has no way to say
   * "private to these five files".  Nothing outside `app/` touches them. */

  ctx: CanvasRenderingContext2D;
  /** Where the pointer last was, in canvas coordinates. */
  cursor: [number, number] = [0, 0];
  /** Both histories hold **program text**: `undoStack` the states an edit moved away from,
   *  `redoStack` the ones an undo moved away from.  Text, so undo restores what somebody wrote
   *  — and so a caller that snapshots before it knows whether the edit will happen must
   *  snapshot `source`, never a serialised sketch: `Document.read` does not throw on a program
   *  error, so the wrong kind of string comes back as an empty drawing rather than a refusal. */
  private undoStack: string[] = [];
  private redoStack: string[] = [];
  private planSolver: PlanSolver | null = null;
  private planKey = '';
  /** The topology `lastSystem` was compiled from, so a stale one is not diagnosed against. */
  private systemKey = '';
  private lastSystem: System | null = null;
  private witness: WitnessReport | null = null;
  private witnessFor: Diagnosis | null = null;
  anim: Animation | null = null;
  private animTimer = 0;
  /** The last copy, as a sketch of its own.  A clipboard is a document, which is what lets one
   *  outlive the selection — and the sketch — it came from. */
  clipboard: Sketch | null = null;
  /** How many times the clipboard has been pasted since it was filled, so pastes cascade
   *  instead of landing on each other. */
  pastes = 0;
  /** The one gesture in progress, if any — pan, point drag, radius drag or rubber band.
   *  One field rather than four means a new gesture cannot be forgotten in `setSketch`, and
   *  the pointer handlers stay two lines each. */
  gesture: Gesture | null = null;
  /** The pointer that owns `gesture`; a second one is ignored until it lets go. */
  gesturePointer: number | null = null;
  /** Whether the last pointer over the canvas was a finger — what the pick tolerance is sized
   *  to (`pickPx`).  Set by every press and move, so a pen or a mouse taking over takes it back. */
  finger = false;
  /** `underParams` as a set, rebuilt when the diagnosis it came from is replaced. */
  movable: { owner: Diagnosis; set: Set<Param> } | null = null;
  /** A gesture moved geometry, so the null space no longer describes the pose on screen. */
  staleDiagnosis = false;
  private frame = 0;
  /** The eye swinging square on to a plane: its timer, so a second choice replaces the first. */
  private looking = 0;
  /** Every object, built exactly by a pool of workers and drawn as it arrives (#671): the page's
   *  own thread builds none.  Without workers (node, a test's view) the sketch builds its own. */
  builds = new ObjectBuilds((error) => this.objectsArrived(error), (r) => this.onBuilds(r), undefined,
                            pageStore(coreIdentity()));
  /** The claims about solids, judged in a worker: the diagnosis here leaves them out. */
  readonly claims = new ClaimJudging((j) => this.onClaims(j));
  /** The load scenarios' answers and the picture of the one drawn, read in a worker. */
  readonly scenarioRead = new ScenarioReading((r) => this.scenarioArrived(r));
  /** The animation frame the picture's swing is running on, while it is. */
  private swinging = 0;
  /** A source sync is running: `swap` re-enters `afterEdit`, and the second pass has nothing
   *  left to write.  One flag rather than a subtle argument about termination. */
  private syncing = false;

  constructor(readonly canvas: HTMLCanvasElement, doc: Document,
              readonly boxCanvas: HTMLCanvasElement | null = null) {
    this.doc = doc;
    this.box3d = new Box3D(boxCanvas);
    this.ctx = canvas.getContext('2d')!;
    this.drawOnFront();
    bindEvents(this);
  }

  /** There is no page: a document nothing has been chosen on is drawn on the front, brought in by
   *  the first press of a tool where the document does not say `use std` yet (`ensurePlane`). */
  private drawOnFront(): void {
    if (this.plane || this.pendingPlane || this.openSpace) return;
    const front = this.doc.entity('std.front');
    if (front instanceof Plane) this.plane = front;
    else this.pendingPlane = 'std.front';
  }

  /** The drawing the source came to.  Read everywhere and written nowhere: a new drawing is a
   *  new elaboration, which is `setProgram`. */
  get sketch(): Sketch { return this.doc.sketch; }

  /** The source.  This is the document — what Save writes, what undo remembers, and what the
   *  panel shows. */
  get source(): string { return this.doc.text; }

  // -- coordinates ---------------------------------------------------------

  /* Two cameras.  `cam` is the eye's: a similarity over the picture plane the core projects the
   * workspace onto, which zooms and pans.  A **view's** camera is that composed with the view's
   * own map (`ViewCam`), and is what turns page coordinates into pixels — so `w2s` and `s2w` read
   * whichever view is being worked in: the one the painter is drawing (`inView`), else the
   * current plane, which is where a tool puts what it makes. */

  w2s(x: number, y: number): [number, number] { return this.viewCam().w2s(x, y); }

  s2w(sx: number, sy: number): [number, number] { return this.viewCam().s2w(sx, sy); }

  /** A page length in screen pixels, in the view being worked in. */
  len(w: number): number { return this.viewCam().len(w); }

  /** The eye length of one screen pixel — what the core sizes annotation and pick tolerances
   *  through, all of which it measures on the eye's picture plane. */
  get unit(): number { return this.cam.unit; }

  /** A screen length as the eye length it stands for. */
  world(px: number): number { return this.cam.world(px); }

  get width(): number { return this.canvas.clientWidth; }
  get height(): number { return this.canvas.clientHeight; }

  /** What of the workspace does not depend on the eye: the view each thing stands in, each view's
   *  place and the eye square on to it.  Asked once an edit — `afterEdit` forgets it, and a new
   *  drawing is a new sketch. */
  workspace(): Workspace {
    const c = this.wsCache;
    if (c && c.sketch === this.sketch) return c.ws;
    const ws = workspace(this.sketch);
    this.wsCache = { sketch: this.sketch, ws };
    return ws;
  }
  private wsCache: { sketch: Sketch; ws: Workspace } | null = null;

  /** Every view's camera, the page first: the core's map of it for the orbit, composed with the
   *  eye's camera.  Built once a frame — `draw` forgets them, since a drag can move a plane's own
   *  points and so its map — and asked of per point. */
  private cams(): ViewCam[] {
    const { az, el } = this.orbit;
    const k = this.cam;
    const c = this.camCache;
    if (c && c.sketch === this.sketch && c.az === az && c.el === el && c.scale === k.scale
        && c.x === k.originX && c.y === k.originY) return c.cams;
    const cams = maps(this.sketch, az, el).map((m) => k.through(m));
    const space = spacePoints(this.sketch, az, el);
    const eye = k.through([1, 0, 0, 0, 1, 0]);
    this.camCache = { sketch: this.sketch, az, el, scale: k.scale, x: k.originX, y: k.originY, cams,
                      space, eye };
    return cams;
  }
  private camCache: { sketch: Sketch; az: number; el: number; scale: number; x: number; y: number;
                      cams: ViewCam[]; space: ([number, number] | null)[]; eye: ViewCam } | null = null;

  /** The view the painter is drawing in, while it is — see `inView`. */
  private drawing: ViewCam | null = null;

  /** The plane being drawn on: the current plane's view, else the page's — and none at all while
   *  drawing in space, where no view's callouts are the ones being drawn. */
  get activeView(): View {
    if (this.openSpace) return NOWHERE;
    return this.plane ? this.plane.index : PAGE;
  }

  /** The view a tool reads its clicks off: the plane being drawn on. */
  get toolView(): View {
    return this.activeView;
  }

  /** A view's camera, or null for a figure that stands in no one view. */
  camOf(view: View): ViewCam | null {
    return ofView(this.cams(), view);
  }

  /** The camera of the view being worked in: the painter's, else the tool's — the eye's own while
   *  drawing in space, whose clicks are read on its picture plane. */
  viewCam(): ViewCam {
    return this.drawing ?? this.toolCam();
  }

  /** The camera a tool reads its clicks through: the current plane's, or the eye's own while
   *  drawing in space. */
  toolCam(): ViewCam {
    if (this.openSpace) {
      this.cams();
      return this.camCache!.eye;
    }
    return this.camOf(this.toolView) ?? this.cams()[0];
  }

  /** The view a point stands in. */
  viewOf(p: Point): View {
    return this.workspace().views.point[p.index] ?? PAGE;
  }

  /** The view an entity is drawn in — a point's own, a figure's, or `NOWHERE` for one whose points
   *  stand in views apart in space (a projector between two views is drawn end to end instead). */
  viewOfEntity(e: Primitive): View {
    const table = (this.workspace().views as Record<string, View[] | undefined>)[e.kind];
    return table?.[e.index] ?? PAGE;
  }

  /** Where a point is seen, whichever view it stands in — a point in space where the core says the
   *  eye sees it, through the eye's own camera. */
  seen(p: Point): [number, number] {
    this.cams();
    const at = this.camCache!.space[p.index];
    if (at) return this.camCache!.eye.w2s(...at);
    return (this.camOf(this.viewOf(p)) ?? this.viewCam()).w2s(...p.xy);
  }

  /** Where a canvas point is on a view's page — where the eye's ray through it meets that plane. */
  s2wIn(view: View, sx: number, sy: number): [number, number] {
    return (this.camOf(view) ?? this.viewCam()).s2w(sx, sy);
  }

  /** **Where a click puts something new**, on the view being worked in: `s2w` to the digits the
   *  zoom justifies (`io.placed`), so a seed the gesture writes says where the pointer was and
   *  no more — `hint((12.3, 40.5))` at six pixels a unit, not seventeen digits of where a pixel
   *  fell. */
  place(sx: number, sy: number): [number, number] { return this.viewCam().place(sx, sy); }

  /** `place`, on a view's page — how a drag reads where the pointer has carried a point. */
  placeIn(view: View, sx: number, sy: number): [number, number] {
    return (this.camOf(view) ?? this.viewCam()).place(sx, sy);
  }

  /** A length a gesture measured on a view's page (a radius, a side), to the digits the zoom
   *  justifies there. */
  placeLength(w: number, view: View = this.toolView): number {
    return io.placed(w, (this.camOf(view) ?? this.viewCam()).pixel);
  }

  /** Run `fn` in a view: every `w2s`, `s2w` and `len` inside it is that view's.  Nothing runs for
   *  a figure that stands in no one view, which has nowhere to be drawn. */
  inView<T>(view: View, fn: () => T): T | undefined {
    const cam = this.camOf(view);
    return cam ? this.withCam(cam, fn) : undefined;
  }

  /** Run `fn` through a view's camera already in hand. */
  withCam<T>(cam: ViewCam, fn: () => T): T {
    const outer = this.drawing;
    this.drawing = cam;
    try {
      return fn();
    } finally {
      this.drawing = outer;
    }
  }

  /** The eye's own camera: the picture plane the core projects the workspace onto, on screen. */
  eyeCam(): ViewCam {
    this.cams();
    return this.camCache!.eye;
  }

  /** Whether a point stands in space, in no view: seen where it is, dragged where it is seen. */
  inSpace(p: Point): boolean {
    this.cams();
    return this.camCache!.space[p.index] != null;
  }

  /** A canvas point on the eye's picture plane, which is where the core asks what is there. */
  eye(sx: number, sy: number): [number, number] {
    return this.cam.s2w(sx, sy);
  }

  /** **Show the solid's surfaces in the box**, not only its edges.
   *
   *  View state, `underlay`'s rule and the orbit's: a drawing is the same drawing whether or not
   *  you are looking at it filled in, so this is never saved, exported, solved or undone.
   *
   *  It is on by default, and it was not while the box was strokes on a flat canvas: there a
   *  surface cost the boundary of every solid and a painter's order that is only very nearly
   *  right.  A depth buffer settles the order per pixel and a mesh is uploaded once, so the
   *  object may as well be shown as an object.  Off is still worth having — a wireframe is how
   *  you see the far side of a part — and is what the toggle means. */
  showSolid = true;

  /** **The workspace's three.js renderer.**  Built once and kept, because a WebGL context is a scarce
   *  thing a browser hands out and dropping one per toggle would eventually get none. */
  readonly box3d: Box3D;

  setShowSolid(on: boolean): void {
    if (this.showSolid === on) return;
    this.showSolid = on;
    this.draw();               // the flat scene does not carry it: the box's own build does
  }

  setOrbitMode(on: boolean): void {
    if (this.orbitMode === on) return;
    this.orbitMode = on;
    this.onOrbitMode(on);
    // a press no longer reaches the tool, so the Extrude tool's wash promises nothing
    if (this.hoverRegion || this.hovered) {
      this.hoverRegion = null;
      this.hovered = null;
      this.draw();
    }
  }

  /** The entity a scene item is drawn from, where one is — the one decode of the wire's
   *  `kind`/`index` pair, so the painter and the picks read it one way. */
  entityOf(it: Omit<Item, 'pts' | 'shade'>): Primitive | undefined {
    return it.kind !== undefined && it.index !== undefined
      ? this.doc.entityOf({ kind: it.kind, index: it.index }) : undefined;
  }

  /** The view a scene item belongs to, as a `Plane` — the other decode of the wire, for the pane
   *  `box3d` bolds as the plane being drawn on. */
  planeOf(it: Omit<Item, 'pts' | 'shade'>): Plane | null {
    const p = it.plane !== undefined ? this.doc.entityOf({ kind: 'frame', index: it.plane }) : null;
    return p instanceof Plane ? p : null;
  }

  /** End everything in flight, uncommitted: a gesture, a wobble animation, a dimension being
   *  carried, a tool's half-collected clicks, and the remembered workspace.  Called before the
   *  drawing is replaced (`swap`) and when a project's paper is shown (`pauseEditing`), so the
   *  list of things that can be mid-way is written once — a tool's
   *  pending points are proxies that die with the sketch, and a curve fit finished after a load
   *  would hand the new sketch the old one's points. */
  private settle(carry = false): void {
    this.stopAnimation();             // first: it restores into the sketch it started on
    abandonGesture(this);             // dropped, not ended: `end` would commit into what follows
    if (this.liveDim) this.endDimension(false);
    this.pending = [];
    this.pendingFit = [];
    this.hoverRegion = null;
    this.pendingRevolve = null;
    // what an edge tool picked was of the drawing being left: indices and paths of its objects
    this.endRound();
    // an extrusion being sized is named, not a proxy: an edit carries it (`swap`), a new document
    // ends it
    if (!carry) this.endExtent();
  }

  /** Frame everything the workspace shows — figures and solids, as the eye now sees them. */
  fit(): void {
    const { az, el } = this.orbit;
    // a first guess at the scale from the page, so the extent below is tessellated at a zoom near
    // the one it will be shown at rather than whatever the last document was left at
    if (this.sketch.points.length) {
      this.cam.fitTo([...this.sketch.drawnBounds()] as [number, number, number, number],
                     this.width, this.height);
    }
    const b = boundsSeen(this.sketch, this.unit, az, el);
    if (b) this.cam.fitTo(b, this.width, this.height);
    this.fitted = this.eyeNow();
    this.draw();
  }

  /** The eye as `fit` left it, while an opening's objects are still being built: once they are, the
   *  frame is taken again if nobody has moved the eye since — the objects were not there to frame. */
  private fitted: string | null = null;

  /** Where the eye is: its orbit, scale and origin. */
  private eyeNow(): string {
    const { az, el } = this.orbit;
    return [az, el, this.cam.scale, this.cam.originX, this.cam.originY].join(' ');
  }

  // -- sketch mutation -----------------------------------------------------

  /** Show a different document.  Loading one is not a step along the history — there is no
   *  future to return to any more — so it drops the redo stack; stepping uses `swap`.
   *
   *  A program that will not elaborate leaves the drawing alone and says so: a half-written
   *  source is a thing somebody is in the middle of typing, not a reason to lose their work. */
  setProgram(text: string, fit = true): boolean {
    return this.reread(text, fit, false);
  }

  /** Project navigation starts a separate edit history; undo must never cross file boundaries. */
  openProjectFile(text: string): boolean {
    const chosen = this.scenario;
    this.scenario = '';
    if (!this.another(() => this.reread(text, true, false))) {
      this.scenario = chosen;
      return false;
    }
    this.undoStack = [];
    return true;
  }

  /** Finish transient interactions before showing a project's paper preview. */
  pauseEditing(): void { this.settle(); }

  /** **A new document**, and the whole of what that means.  Every way of getting one — File ▸
   *  New, Open, a test case — comes through here, so the intention is spelled once: the
   *  outgoing document goes on the undo stack as one step (so a load is undoable, and ⌘Z after
   *  it cannot land on an older state of a drawing that is gone), whatever was in flight is
   *  settled, the selection is not carried (it belonged to the other drawing), the box is left
   *  if the new drawing has no view to show, and the camera is refitted.  `setProgram` is the
   *  *edit* of the same shape — the program panel replacing the text — which keeps the history
   *  its own way and is not this. */
  load(text: string, fit = true): boolean {
    this.pushUndo();
    // another document states its own scenarios: none is drawn until one is chosen
    const chosen = this.scenario;
    this.scenario = '';
    if (!this.another(() => this.reread(text, fit, false))) {
      this.scenario = chosen;
      this.dropUndo();
      return false;
    }
    // another document is drawn on its front until told, as a view opens
    if (this.openSpace) {
      this.openSpace = false;
      this.drawOnFront();
    }
    this.onLoad();
    return true;
  }

  /** Read another document: nothing of it is hidden, the outgoing rows not being its own — and
   *  they are kept where the read is refused, the drawing staying as it was. */
  private another(read: () => boolean): boolean {
    const held = this.hiddenRows;
    const cut = this.section;
    this.hiddenRows = new Set();
    this.section = null;
    if (read()) {
      if (cut) {
        this.sectionEpoch++;
        this.onSection();
      }
      return true;
    }
    this.hiddenRows = held;
    this.section = cut;
    return false;
  }

  /** File ▸ New: a fresh sheet that says `use std`, so the first thing drawn is drawn on the
   *  front and has `std.origin` to be placed against. */
  newDocument(): void {
    this.load('use std\n');
  }

  /** Read a source afresh and make it the document — the one place `Document.read` is called, so
   *  a program that will not elaborate is refused in exactly one way.  `carry` says whether the
   *  selection is the same drawing's (an edit) or another's (a load). */
  private reread(text: string, fit: boolean, carry: boolean): boolean {
    let next: Document;
    try {
      next = Document.read(text);
    } catch {
      this.onStatus('the program could not be read');
      return false;
    }
    this.redoStack = [];
    this.swap(next, fit, carry);
    return true;
  }

  /** Adopt a sketch built some other way — an example, a JSON file, a fresh sheet — by lifting
   *  it into the program it is written as.  **The one migration seam**: past it, everything is
   *  a document.  The sketch is consumed. */
  setSketch(sk: Sketch, fit = true): void {
    let text: string;
    try {
      text = fromSketch(sk);
    } finally {
      sk.dispose();
    }
    this.load(text, fit);
  }

  /** Adopt an elaboration already in hand — the seam every structural edit goes through, so
   *  there is exactly one place where the drawing is replaced. */
  private swap(next: Document, fit: boolean, carry = false): void {
    this.settle(carry);               // before the swap: nothing in flight may reach the new sketch
    const held = carry ? this.namesOf(this.selected) : [];
    const heldSolids = carry ? this._solids : [];
    const heldPlane = carry && this.plane ? this.doc.nameOf(this.plane) : undefined;
    const space = this.openSpace;
    const old = this.doc;
    this.doc = next;
    // the outgoing elaboration owns a core sketch, and a wasm heap only grows
    if (old !== next) old.dispose();
    // the current plane crosses by name too, and only if the name still reaches a plane: an
    // edit keeps it, deleting it or loading another document lets it go
    // the selection first: its setter arms the current plane when a lone plane is picked, and
    // what the swap carries is the answer — otherwise a plane still selected but deliberately
    // *not* current (`choosePlane`) would be re-armed by the rebind
    this.selected = carry ? this.rebind(held) : [];
    // a solid crosses by name too; the face path is the source's words and needs no rebinding
    if (heldSolids.length) {
      const solids = new Map(this.doc.solids().map((s) => [s.name, s.index]));
      this._solids = heldSolids.flatMap((s) => {
        const index = solids.get(s.name);
        return index === undefined ? [] : [{ ...s, index }];
      });
    }
    // what the outline hid, by path, on the drawing just taken (a load emptied it first)
    hidden.carryHidden(this);
    // the section by its plane's name, on the drawing just taken (a load let it go first)
    this.applySection();
    if (this.liveExtent && !extent.liveHandle(this)) this.endExtent();
    const again = heldPlane ? this.doc.entity(heldPlane) : undefined;
    this.plane = again instanceof Plane ? again : null;
    // drawing in space is no plane to lose, and outlasts a lone plane the selection re-armed
    this.openSpace = space;
    this.drawOnFront();
    // a standard plane chosen before the document had it: brought in by the edit just taken
    if (this.pendingPlane) {
      const wanted = this.doc.entity(this.pendingPlane);
      if (wanted instanceof Plane) {
        this.plane = wanted;
        this.pendingPlane = null;
      } else if (!carry) {
        this.pendingPlane = null;   // another document: it chooses its own
      }
    }
    // a new document is looked at the way it is drawn: a flat one square on, one with solids or
    // with geometry off the front plane from three quarters
    if (fit && !carry) this.orbit = this.homeOrbit();
    // another document runs over its own parameters, from its start
    if (!carry) this.instant = null;
    this.highlight = [];
    this.hovered = null;            // a proxy dies with its sketch, and the next move asks again
    this.litConstraint = null;
    this.pastes = 0;              // a fresh sheet: the next paste starts its cascade over
    this.releasePlan();
    this.afterEdit();
    this.onProgram();
    // after the solve: loading a case can move the geometry a long way
    if (fit) this.fit();
  }

  /* -- a selection across a re-elaboration --------------------------------
   *
   * A proxy is interned on `(kind, index)` in one `Sketch`, so it dies with the elaboration that
   * made it.  A **name** does not: it is what the source calls the thing, and the source is what
   * survives an edit.  So a selection crosses by name, and the lookup is the source map's — the
   * front end does no indexing of its own. */

  namesOf(ps: Primitive[]): string[] {
    return ps.map((p) => this.doc.nameOf(p)).filter((n): n is string => !!n);
  }

  private rebind(names: string[]): Primitive[] {
    return names.map((n) => this.doc.entity(n)).filter((p): p is Primitive => !!p);
  }

  /** Apply an edit the core computed.  `structural` re-elaborates and carries the selection
   *  across by name; `numeric` and `none` leave the drawing standing, because the core has said
   *  the topology cannot have moved and a compiled plan is still good. */
  apply(e: Edit, what?: string): boolean {
    if (e.refused) {
      this.onStatus(e.refused);
      return false;
    }
    if (e.kind === 'none') return false;
    this.pushUndo();
    if (!this.take(e.text, e.kind === 'numeric')) {
      this.dropUndo();
      return false;
    }
    if (what) this.onStatus(what);
    return true;
  }

  /** Take an elaboration already read as the document — for a caller that had to read it to
   *  know whether to take it at all (a dimension's text that does not elaborate is refused and
   *  left in its editor).  The selection crosses by name, as after any structural edit. */
  takeDocument(next: Document): void {
    this.redoStack = [];
    this.swap(next, false, true);
  }

  /** Elaborate the source again, as it stands — the selection crossing by name. */
  rereadSource(): boolean {
    return this.reread(this.source, false, true);
  }

  /** Take a new source as the document.
   *
   *  `numeric` is the core's word that the topology cannot have moved, so the drawing stands and
   *  the compiled plan and the selection with it — which is what keeps editing a dimension
   *  instant.  When the core would rather be re-elaborated it says so by refusing `retext`, and
   *  re-reading is always correct and only slower. */
  private take(text: string, numeric: boolean): boolean {
    if (numeric && this.doc.retext(text)) {
      this.redoStack = [];
      this.afterEdit();
      this.onProgram();
      return true;
    }
    return this.reread(text, false, true);
  }

  /** Bring the source back into step with a drawing a gesture changed.
   *
   *  A tool draws by mutating the elaborated sketch — that is how it gets to snap and solve with
   *  the pointer still down — so this is where the document catches up: a splice appending what
   *  was drawn, taking out what was deleted, and committing the seeds.  Everything somebody wrote
   *  is left alone, which is the difference between a document and a print-out of the drawing.
   *
   *  Safe mid-tool: `reconcile` extends the elaboration rather than replacing it, so a chaining
   *  tool's half-built polyline keeps the proxies it is holding.  Held off only while a *drag* is
   *  live (`syncSeeds` is the seam for that, once, at release) and while a dimension is being
   *  carried, which is a thing being said rather than a thing yet done. */
  syncSource(): void {
    if (this.syncing || this.anim || this.gesture || this.liveDim) return;
    this.syncing = true;
    try {
      const e = this.doc.reconcile();
      if (e.refused) return this.onStatus(e.refused);
      if (e.kind !== 'none') this.onProgram();
    } finally {
      this.syncing = false;
    }
  }

  /** Put where the drawing *is* back into the seeds it came from.  Run at the end of a gesture,
   *  never per frame: during a drag the text is stale, and that is correct — a drag is one edit,
   *  at the moment it is let go. */
  syncSeeds(): void {
    if (this.anim) return;      // a wobble is not where the drawing is; freezing it in would lie
    const e = this.doc.commitSeeds();
    if (e.kind === 'none') return;
    if (this.doc.retext(e.text)) return this.onProgram();
    // the core would rather be re-elaborated.  Re-read rather than drop it: dropping would leave
    // the source quietly describing where the drawing *was*, which is the one failure this whole
    // design exists to prevent.  No solve here — the drag has already done it, and `endGesture`
    // is about to tell the shell.
    this.reread(e.text, false, true);
  }


  /** Remember a state to come back to — the current one, or an earlier one a caller took a
   *  snapshot of before it knew whether the edit would come to anything.  A state is program
   *  text, so undo is exact: it restores what somebody wrote, comments and all. */
  pushUndo(state: string = this.source): void {
    this.undoStack.push(state);
    if (this.undoStack.length > 100) this.undoStack.shift();
    this.redoStack = [];              // a fresh edit is a new branch: the old future is gone
  }

  /** The edit that snapshot was taken for came to nothing, so take the snapshot back —
   *  for an edit that cannot know whether it will happen until it has tried. */
  dropUndo(): void {
    this.undoStack.pop();
  }

  undo(): void { this.step(this.undoStack, this.redoStack, 'undo'); }

  redo(): void { this.step(this.redoStack, this.undoStack, 'redo'); }

  /** One step along the history.  The state being left goes onto the other stack, so the two
   *  are mirror images and the pair walks the same line in both directions. */
  private step(from: string[], to: string[], what: string): void {
    const s = from.pop();
    if (!s) return this.onStatus(`nothing to ${what}`);
    let next: Document;
    try {
      next = Document.read(s);
    } catch {
      return this.onStatus(`could not ${what}`);
    }
    to.push(this.source);
    this.swap(next, false);
    this.onStatus(what);
  }

  /** Free the compiled plan.  A live point drag holds the core's pointer to it, so the gesture
   *  goes first — freeing a plan out from under one would leave the drag stepping freed memory,
   *  and on wasm that aborts rather than raising.  Every caller gets this, so no caller has to
   *  remember the order. */
  releasePlan(): void {
    if (this.planSolver && this.gesture) abandonGesture(this);
    if (this.lastSystem === this.planSolver?.system) this.lastSystem = null;
    this.planSolver?.dispose();
    this.planSolver = null;
    this.planKey = '';
  }

  /** The plan solver owns its System; anything else we compiled here is ours to free. */
  private releaseSystem(): void {
    if (this.lastSystem && this.lastSystem !== this.planSolver?.system) this.lastSystem.dispose();
    this.lastSystem = null;
  }

  /** The decomposition plan, compiled once per topology and replayed for dimension edits and
   *  drags. */
  plan(): PlanSolver {
    const key = this.sketch.topologyKey();
    if (!this.planSolver || key !== this.planKey) {
      this.releasePlan();
      this.planSolver = new PlanSolver(this.sketch, true);
      this.planKey = key;
    }
    return this.planSolver;
  }

  /** One solve by the selected path; keeps the compiled System for the diagnosis that follows. */
  solveOnce(): SolveResult {
    this.releaseSystem();
    this.systemKey = this.sketch.topologyKey();
    if (this.usePlan && this.sketch.constraints.length) {
      const ps = this.plan();
      this.lastPlan = ps.solve(1e-9, true, this.method);     // reads and records sketch.branches
      this.lastSystem = ps.system;                           // borrowed, not ours to dispose
      return asSolveResult(this.lastPlan);
    }
    this.lastPlan = null;
    const sys = new System(this.sketch);
    this.lastSystem = sys;
    return sys.solve({ method: this.method });
  }

  solveNow(): SolveResult {
    this.lastResult = this.solveOnce();
    this.onChanged();
    this.draw();
    return this.lastResult;
  }

  private rediagnose(system: System | null, settled: boolean): void {
    // the claims about solids are the worker's (`claims`): judged here they held the thread
    // for seconds an edit, and nothing else the diagnosis says reads them; and a pose no solve
    // settled at is judged for nothing (#449)
    this.diagnosis = this.sketch.constraints.length
      ? diagnose(this.sketch, { system, solidClaims: false, settled }) : null;
    this.staleDiagnosis = false;
  }

  /** Every mutation ends here.  A failed solve leaves the last good geometry on screen: the
   *  failure is reported by the diagnosis, not by exploded geometry (which would also mislead
   *  the conflict search). */
  afterEdit(): SolveResult | null {
    this.wsCache = null;
    // A dimension still being laid down is being *said*, not solved, and the drawing holds
    // still under the number while somebody decides where to put it and which one it is: no
    // solve, and no re-diagnosis either — nothing changes colour, no banner appears and
    // disappears under the pointer, and the constraint list does not rebuild on every kind the
    // number passes through.  The click that plants it is when all of it happens, once, and
    // when what it came to is reported — see `dimension.placeDimension`.
    if (this.liveDim?.placing) {
      this.draw();
      return this.lastResult;
    }
    // an edit is made where the drawing is drawn: home, whatever instant it is shown at
    params.home(this.sketch);
    // whether the pose diagnosed is this edit's answer: a failed solve's pose is put back to the
    // last good geometry, which this drawing does not hold
    let settled = true;
    if (this.autoSolve) {
      const xBefore = this.sketch.getX();
      this.lastResult = this.solveOnce();
      if (!this.lastResult.success) this.sketch.setX(xBefore);
      settled = this.lastResult.settled;
    }
    // with auto-solve off nothing has recompiled since the edit, so the System we still hold was
    // built from a sketch that no longer exists: diagnosing against it names dead constraints
    const fresh = this.systemKey === this.sketch.topologyKey();
    if (!fresh) this.releaseSystem();
    this.rediagnose(fresh ? this.lastSystem : null, settled);
    this.syncSource();        // the drawing changed, so the document has to say what it now says
    this.readBehind();
    this.followLater();
    this.onChanged();
    this.draw();
    return this.lastResult;
  }

  /** **Read the drawing again where it stands, off the page's thread**: its objects' exact
   *  builds, the solid claims' verdicts and the load scenarios' answers with the picture of the
   *  one drawn.  At the end of an edit and at the release of a drag, which moves the drawing
   *  without one: a build of what the drawing still reads is asked of no worker again. */
  readBehind(): void {
    this.builds.start(this.doc);
    this.claims.start(this.doc);
    this.readScenarios();
  }

  /** Build this view's objects in a pool of workers that `spawn` makes, remembering builds in
   *  `store` (a test's, in place of the page's): the outgoing pool's workers let go. */
  usePool(spawn: (() => PoolWorker) | null, store: SurfaceStore | null = null, size?: number): ObjectBuilds {
    this.builds.dispose();
    this.builds = new ObjectBuilds((error) => this.objectsArrived(error), (r) => this.onBuilds(r), spawn, store, size);
    return this.builds;
  }

  /** An object's build arrived from the pool: the same sketch now shows more, so what was drawn
   *  from it is drawn again — and, the last of an opening's, framed where the eye was left as the
   *  opening framed it. */
  private objectsArrived(error?: string): void {
    this.box3d.invalidate();
    if (error) this.onStatus(error);
    this.onChanged();
    const left = this.builds.rows.some((r) => r.state === 'waiting' || r.state === 'building');
    if (!left && this.fitted !== null) {
      const owed = this.fitted === this.eyeNow();
      this.fitted = null;
      if (owed) { this.fit(); this.fitted = null; return; }
    }
    this.draw();
  }

  // -- Stage 4: witness analysis and DOF animation --------------------------

  /** Witness analysis of the current sketch, cached until the next edit.  The structural
   *  diagnosis is already in hand, so this only adds the Jacobian work. */
  witnessReport(): WitnessReport | null {
    const d = this.diagnosis;
    if (!d) return null;
    if (this.witnessFor !== d) {
      this.witnessFor = d;
      this.witness = analyze(this.sketch);
    }
    return this.witness;
  }

  /** Animate the remaining internal DOFs (each null-space mode in turn); false if none. */
  startAnimation(): boolean {
    this.stopAnimation();             // a second click would otherwise leak the running interval
    const rep = this.witnessReport();
    const modes = rep ? rep.motions.filter((m) => !m.rigid) : [];
    if (!modes.length) return false;
    this.anim = {
      modes,
      sketch: this.sketch,
      x0: this.sketch.getX(),
      free: this.sketch.freeIndices(),
      amp: 0.06 * this.sketch.extent(),
      labels: modes.map((m) => m.moving.slice(0, 8).map((p) => p.name).join(', ')),
      t: 0,
      showing: -1,
    };
    this.animTimer = window.setInterval(() => this.animTick(), ANIM_DT * 1000);
    return true;
  }

  stopAnimation(): void {
    if (!this.anim) return;
    clearInterval(this.animTimer);
    this.animTimer = 0;
    const { x0, sketch } = this.anim;
    this.anim = null;
    sketch.setX(x0);                  // the sketch it started on, whatever is on screen now
    this.draw();
  }

  // -- the timeline (#398 L) -------------------------------------------------

  /** The parameters the drawing declares, each with the interval it runs over. */
  parameters(): Parameter[] {
    return params.parameters(this.sketch);
  }

  /** **How the drawing runs over each parameter it declares**, followed now if it has not been
   *  since the last edit: asked by what needs a march (an instant shown, the timeline scrubbed,
   *  run or dragged, a point dragged along it).  Empty with no parameters. */
  get marches(): March[] {
    if (!this.followed) this.follow();
    return this.followed!;
  }

  /** The march along `name`, followed now if it has not been since the last edit. */
  marchOf(name: string): March | undefined {
    return this.marches.find((m) => m.name === name);
  }

  /** The march along `name` if it has been followed since the last edit, and nothing is followed
   *  to answer: what the timeline shows, the interval without its reach until it is. */
  followedMarch(name: string): March | undefined {
    return this.followed?.find((m) => m.name === name);
  }

  /** After an edit, at home.  Shown at an instant, the drawing is stood back there at once, as
   *  near it as the march now reaches — which needs the march.  At home nothing is followed
   *  until it is needed or the page is idle (#412): the timeline shows the interval meanwhile. */
  private followLater(): void {
    this.cancelFollow();
    this.followed = null;
    if (this.instant?.at != null) return this.follow();
    if (this.parameters().length === 0) this.followed = [];
    this.onTimeline();
    if (this.followed) return;
    const doc = this.doc;
    const run = () => {
      this.idleFollow = null;
      // a document since let go, or replaced, is not this one's to follow
      if (this.doc === doc && doc.live && !this.followed) this.follow();
    };
    // a page says when it is idle; node, where the tests run, does not
    if (typeof requestIdleCallback === 'function') {
      const id = requestIdleCallback(run);
      this.idleFollow = { cancel: () => cancelIdleCallback(id) };
    } else {
      const id = setTimeout(run, IDLE_FOLLOW_MS);
      this.idleFollow = { cancel: () => clearTimeout(id) };
    }
  }

  /** Let the idle follow an edit asked for go: the march was taken, or another edit came. */
  private cancelFollow(): void {
    this.idleFollow?.cancel();
    this.idleFollow = null;
  }

  /** **Each parameter followed**, now: the timeline's reach, and the drawing stood back at the
   *  instant it was shown at, as near it as the march now reaches. */
  follow(): void {
    this.cancelFollow();
    this.followed = this.parameters().map((p) => params.march(this.sketch, p.name));
    const shown = this.instant;
    if (shown && !this.marchOf(shown.name)) this.instant = null;
    else if (shown?.at != null) this.showInstant(shown.name, shown.at, false);
    this.onTimeline();
  }

  /** **Show the drawing at `at` of `name`**, on the branch its march carries — null for home.
   *  Clamped to what the march reached, so a timeline dragged past a singular instant stands
   *  there.  Whether it could be; where not, it is left at home and the status line says why. */
  showInstant(name: string, at: number | null, say = true): boolean {
    const m = this.marchOf(name);
    if (!m) return false;
    this.stopAnimation();             // it restores into the pose it started from
    params.home(this.sketch);
    let why: string | null = null;
    if (at !== null && m.followed) {
      at = Math.min(Math.max(at, m.from), m.reached);
      why = params.stand(this.sketch, name, at);
    } else if (at !== null) {
      why = m.text;
    }
    this.instant = { name, at: why ? null : at };
    if (why && say) this.onStatus(why);
    if (say) this.onTimeline();
    this.draw();
    return !why;
  }

  /** Where a drag along the march put the drawing (`gesture`'s scrub): the instant shown. */
  scrubbedTo(name: string, at: number): void {
    this.instant = { name, at };
    this.onTimeline();
  }

  /** A value of `name` as a reader reads it, in the core's words: `129.838deg`. */
  reading(name: string, v: number): string {
    return params.reading(this.sketch, name, v);
  }

  private animTick(): void {
    const a = this.anim;
    if (!a || a.sketch !== this.sketch) return;
    a.t += ANIM_DT;
    const k = Math.floor(a.t / ANIM_PERIOD) % a.modes.length;
    const phase = Math.sin((2 * Math.PI * (a.t % ANIM_PERIOD)) / ANIM_PERIOD);
    const x = Float64Array.from(a.x0);
    const v = a.modes[k].velocity;
    for (let i = 0; i < a.free.length; i++) x[a.free[i]] += a.amp * phase * v[i];
    this.sketch.setX(x);
    if (k !== a.showing) {
      a.showing = k;
      this.onStatus(`DOF ${k + 1}/${a.modes.length}: ${a.labels[k]}`);
    }
    this.draw();
  }

  stateOf(e: Primitive): string {
    return this.diagnosis?.entityState.get(e) ?? 'well';
  }
  // -- hit testing ---------------------------------------------------------

  /* The core's answers, asked where the eye sees things: the camera turns the click into a place
   * on the picture plane and the pixel tolerance into a length there, and every figure is measured
   * standing on its own plane.  Views that lie on top of one another on the page are nowhere near
   * one another in space, which is why none of this is asked of page coordinates any more. */

  /** How near, in pixels, a press must be to what it picks: a finger's reach or a cursor's. */
  get pickPx(): number { return this.finger ? TOUCH_PICK_PX : PICK_PX; }

  pickPoint(sx: number, sy: number, tol = this.pickPx): Point | null {
    const { az, el } = this.orbit;
    const { point, dist } = nearestSeen(this.sketch, az, el, ...this.eye(sx, sy));
    return point && dist < this.world(tol) ? point : null;
  }

  pick(sx: number, sy: number): Primitive | null {
    const { az, el } = this.orbit;
    return pickSeen(this.sketch, this.unit, az, el, ...this.eye(sx, sy), this.world(this.pickPx));
  }

  /** The object face under the canvas point, nearest the eye, and the solid it is a face of. */
  solidAt(sx: number, sy: number): SolidPick | null {
    const { az, el } = this.orbit;
    const hit = pickSolidSeen(this.sketch, az, el, ...this.eye(sx, sy));
    return hit && this.solidPick(hit.solid, { face: hit.face, at: hit.at });
  }

  /** A part of object `index` picked — the object named as the source names it — or null for an
   *  object with no name. */
  solidPick(index: number,
            part: Omit<SolidPick, 'name' | 'index'> = { face: '' }): SolidPick | null {
    if (this.solidNames?.doc !== this.doc) {
      const names = new Map(this.doc.solids().map((s) => [s.index, s.name]));
      this.solidNames = { doc: this.doc, names };
    }
    const name = this.solidNames.names.get(index);
    return name ? { name, index, ...part } : null;
  }
  /** The objects' names by index, read once per document: the hover asks every move. */
  private solidNames: { doc: Document; names: Map<number, string> } | null = null;

  /** Select a part of an object — a body, a face, an edge — or with `add` toggle it in the
   *  selection.  It lets the drawing's selection and the picture go, as selecting either of them
   *  lets the solids go. */
  pickSolid(pick: SolidPick, add = false): void {
    this.pickSolids(toggled(this._solids, pick, add, samePick));
  }

  /** Select these parts of objects, in this order. */
  pickSolids(picks: SolidPick[]): void {
    this._solids = picks;
    this._selected = [];
    this.dropImage();
  }

  /** The edge of an object the eye sees under the canvas point, within a pick's reach. */
  edgeAt(sx: number, sy: number): SolidPick | null {
    const { az, el } = this.orbit;
    const hit = pickEdgeSeen(this.sketch, az, el, ...this.eye(sx, sy), this.world(this.pickPx));
    return hit && this.solidPick(hit.solid, { face: '', edge: hit.faces, at: hit.at });
  }

  /** **Everything under the canvas point** a click with the tool down could take: what is drawn,
   *  then the objects' edges and faces nearest the eye first — an edge behind the material too,
   *  `hidden` — then the objects whole.  An edge tool takes only objects' parts, and a Boolean
   *  only objects. */
  hitsAt(sx: number, sy: number): Hit[] {
    const { az, el } = this.orbit;
    const takes = picksAs(this.tool);
    return underSeen(this.sketch, this.unit, az, el, ...this.eye(sx, sy),
                     this.world(this.pickPx)).flatMap((u): Hit[] => {
      if (u.kind === 'drawn') return takes === 'all' ? [{ kind: 'entity', ent: u.ent }] : [];
      if (takes === 'bodies' && u.kind !== 'object') return [];
      const pick = this.solidPick(u.solid,
        u.kind === 'edge' ? { face: '', edge: u.faces, at: u.at }
        : u.kind === 'face' ? { face: u.face, at: u.at } : { face: '' });
      return pick ? [{ kind: 'solid', pick, hidden: u.kind === 'edge' && !u.seen }] : [];
    });
  }

  /** **What a right-click is on**, as a press finds it (`gesture::whatIsAt`): what is drawn there,
   *  else the edge or face of an object the eye sees, else a pane's plane — or nothing. */
  contextAt(sx: number, sy: number): Hit | null {
    const ent = this.pick(sx, sy);
    if (ent && !(ent instanceof Plane)) return { kind: 'entity', ent };
    const pick = this.edgeAt(sx, sy) ?? this.solidAt(sx, sy);
    if (pick) return { kind: 'solid', pick };
    return ent ? { kind: 'entity', ent } : null;
  }

  /** Light a hit as the pointer is on it — a row of the ⌘-click's list — or nothing. */
  hoverHit(hit: Hit | null): void {
    this.hovered = hit;
    this.draw();
  }

  /** **The model outline** (#435): every instance the document makes and its members, as the
   *  core arranges them — read once per text, since a splice moves its spans and a tool's append
   *  adds rows without a new drawing. */
  outline(): Row[] {
    const c = this.outlineCache;
    if (c?.doc === this.doc && c.text === this.doc.text) return c.rows;
    const rows = this.doc.outline();
    this.outlineCache = { doc: this.doc, text: this.doc.text, rows };
    return rows;
  }
  private outlineCache: { doc: Document; text: string; rows: Row[] } | null = null;

  /** **Isolate** (`I`): everything hidden but the selection and the rows that made it. */
  isolate(): boolean { return hidden.isolate(this); }
  /** **Show all** (`⇧I`). */
  showAll(): void { hidden.showAll(this); }

  /** What is lit under the pointer, of the drawing's own: `highlight` and the hovered entity. */
  get lit(): Primitive[] {
    const h = this.hovered;
    return h?.kind === 'entity' && !this.highlight.includes(h.ent)
      ? [...this.highlight, h.ent] : this.highlight;
  }

  /** The plane whose pane is under the canvas point, nearest the eye, as the chooser names it — a
   *  pane is the plane, so double-clicking one is choosing it.  Of panes standing in one place
   *  (`std.up` on `std.front`), the one the chooser offers. */
  paneAt(sx: number, sy: number): string | null {
    const { az, el } = this.orbit;
    const offered = this.planeChoices();
    for (const pl of panesSeen(this.sketch, az, el, ...this.eye(sx, sy))) {
      const name = this.doc.nameOf(pl);
      if (name && offered.includes(name)) return name;
    }
    return null;
  }

  // -- a load scenario, drawn (#510) -----------------------------------------

  /** The document's load scenarios, by name, in the order written. */
  scenarioNames(): { name: string; extends: string | null }[] {
    return this.doc.scenarioNames();
  }

  /** **Draw scenario `name` over the workspace** ('' for none): its marks at once, and its
   *  picture once the worker has read it.  Writes nothing and solves nothing. */
  chooseScenario(name: string): void {
    if (name === this.scenario) return;
    this.scenario = name;
    this.scenarioScale = null;
    this.readScenarios();
    this.draw();
  }

  /** The factor the picture is drawn at: the card's, else the picture's own. */
  get scenarioFactor(): number {
    return this.scenarioScale ?? this.picture?.scale ?? 1;
  }

  /** Swing the picture from rest to its factor and back, or stop. */
  setScenarioAnimate(on: boolean): void {
    this.scenarioAnimate = on;
    cancelAnimationFrame(this.swinging);
    this.swinging = 0;
    const tick = (): void => {
      if (!this.scenarioAnimate || !this.picture) { this.swinging = 0; return; }
      this.draw();
      this.swinging = requestAnimationFrame(tick);
    };
    if (on) this.swinging = requestAnimationFrame(tick);
    this.draw();
  }

  /** Read the scenario drawn again: its marks here, at once, and every scenario's answers with
   *  its picture in the worker.  A name the document no longer states is let go. */
  private readScenarios(): void {
    const names = this.doc.scenarioNames();
    if (this.scenario && !names.some((n) => n.name === this.scenario)) this.scenario = '';
    const k = names.findIndex((n) => n.name === this.scenario);
    this.scenarioMarks = k >= 0 ? this.doc.scenarioMarks(k) : [];
    this.scenarioEpoch += 1;
    // a job of another drawing or choice tells at once that its picture is gone (a pose this
    // edit may have left) until the worker says again; the same one leaves the picture standing
    this.scenarioRead.start(this.doc, this.scenario);
    this.onScenarioView();
  }

  /** The worker's reading arrived, or moved: the answers to the footer, the picture to the box. */
  private scenarioArrived(r: Reading | null): void {
    this.scenarioReading = r;
    const picture = r && r.drawn === this.scenario ? r.picture : null;
    if (picture !== this.picture) {
      this.picture = picture;
      this.scenarioEpoch += 1;
    }
    // the scenarios' answers are a final reading's: a coarser level's, while a finer one is
    // read, are the reading's own (`scenarioReading.scenarios`), for the card to show
    if (!r || (r.scenarios && !r.reading)) {
      this.scenarios = r?.scenarios ?? [];
      this.measures = r?.measures ?? [];
      this.onScenarios(this.scenarios, this.measures);
    }
    if (this.scenarioAnimate && this.picture && !this.swinging) this.setScenarioAnimate(true);
    this.onScenarioView();
    this.draw();
  }

  // -- painting ------------------------------------------------------------

  draw(): void {
    this.camCache = null;     // a plane's own points may have moved, and its map with them
    if (this.frame) return;
    this.frame = requestAnimationFrame(() => { this.frame = 0; paint(this); });
  }

  resize(): void {
    const dpr = window.devicePixelRatio || 1;
    this.canvas.width = Math.round(this.width * dpr);
    this.canvas.height = Math.round(this.height * dpr);
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    // Coalesce resize with the solve and fit that follow it during startup. Painting
    // synchronously here projects the unsolved solid, only to discard it immediately.
    this.draw();
  }

  /** The dimension whose callout is under the cursor, if any.  Callouts are painted over the
   *  geometry and are what a click on them means, so they are picked before it.  The core does
   *  the test against the same layout it drew — including what a point in reach outranks — so
   *  what is picked is what is on screen. */
  pickCallout(sx: number, sy: number): Constraint | null {
    if (!this.showDimensions) return null;
    const { az, el } = this.orbit;
    const hit = calloutSeen(this.sketch, this.unit, az, el, ...this.eye(sx, sy), this.pickPx);
    const c = hit && (this.sketch.constraintById(hit.id) ?? null);
    // only a callout that is drawn answers a press
    return c && (c === this.litConstraint || this.showsCallouts(hit.view)) ? c : null;
  }

  /** Whether a view's callouts are shown: those of the plane being drawn on, as a CAD tool shows
   *  the dimensions of the sketch being edited — and of any view standing in the same place, as
   *  the page and `std.front` do.  The rest of the workspace's dimensions would be a thicket from
   *  any one eye; the dimension being written or focused is shown wherever it is. */
  showsCallouts(view: View): boolean {
    const ws = this.workspace();
    return view !== NOWHERE && placeOf(ws, view) === placeOf(ws, this.activeView);
  }

  /** The view a dimension's callout is laid out in — where a press on it is read, and where it
   *  is dragged to. */
  calloutView(c: Constraint): View {
    const k = dim.callouts(this.sketch, this.unit, [c.id]).items[0];
    return k ? k.view : NOWHERE;
  }

  /* -- what the shell asks the view to do ---------------------------------
   *
   * The work is next door; these are the names the rest of the app knows it by, so a caller
   * holds one object and never has to know which module a verb lives in. */

  setTool(tool: Tool): void {
    tools.setTool(this, tool);
    // a Boolean tool starts from the bodies already selected: the first is the target
    solids.tellCombine(this);
    // an edge tool says what it wants
    this.tellRound();
  }

  /* The picture traced over, if there is one.  It is view state and not document state, so
   * none of this goes near `afterEdit`, the undo stack or the source — a repaint is the whole
   * of the consequence. */

  /** Put a picture in the middle of the view, replacing any that was there.  It arrives
   *  selected, so the handles that place it are there to be used at once. */
  traceImage(image: Bitmap, name: string, url: string | null = null): void {
    underlay.release(this.underlay);
    this.underlay = underlay.place(this, image, name, url);
    // select is where a picture is handled, and it arrives selected — so the tool comes with
    // it rather than being set by whichever caller happened to ask.  A drawing tool would
    // leave it `picked` with nothing on the canvas willing to answer for it.
    this.setTool('select');
    this.pickImage();
    this.onStatus(`tracing ${name} — drag it to place it, drag a corner to size and turn it, `
                  + 'Delete to remove it');
    this.onChanged();
    this.draw();
  }

  /** Select the picture.  The two selections are exclusive: a photograph is not a `Primitive`
   *  and cannot be constrained or dimensioned beside one, so holding both would only leave
   *  Delete ambiguous. */
  pickImage(): void {
    if (!this.underlay) return;
    this.underlay.picked = true;
    this.selected = [];
  }

  /** And the other way: anything that selects geometry lets the picture go. */
  dropImage(): void {
    if (this.underlay) this.underlay.picked = false;
  }

  /** Take it away again.  The sentence is here and not at the callers: the menu item and the
   *  Delete key remove the same picture, and a removal reported two ways (or, as it was, one
   *  way and silently) is two removals as far as anyone reading the status line is concerned. */
  removeImage(): void {
    const gone = this.underlay?.name;
    underlay.release(this.underlay);
    this.underlay = null;
    if (gone) this.onStatus(`removed ${gone}`);
    this.onChanged();
    this.draw();
  }

  /** Fade it, or bring it back — `by` is added to the opacity and the result is kept on 0…1.
   *  What it came to is read off the status line, which is where what is picked is named; it is
   *  not also toasted, since two places saying one number is two places to keep in step. */
  fadeImage(by: number): void {
    const u = this.underlay;
    if (!u) return;
    u.opacity = Math.min(1, Math.max(0, u.opacity + by));
    this.onPicked();
    this.draw();
  }
  cancelTool(): void { tools.cancelTool(this); }
  /** Space: every selection let go — the drawing's, the solids', the picture — the tool still
   *  down.  An edge tool's number, asked over what it had picked, is put away with them. */
  selectNone(): void {
    this.selected = [];
    this.dropImage();
    if (this.roundClick) {
      this.endRound();
      this.tellRound();
    }
    this.onSelect();
    this.onChanged();
    this.draw();
  }
  finishCurve(): void { tools.finishCurve(this); }
  finishSplineFit(): void { tools.finishSplineFit(this); }

  /* The solid tools (`solids.ts`) and the sizing of what they make (`extent.ts`). */

  regionAt(sx: number, sy: number): Region | { refused: string } | null {
    return solids.regionAt(this, sx, sy);
  }
  extrudeAt(sx: number, sy: number, cut = false, through = false): void {
    solids.extrudeAt(this, sx, sy, cut, through);
  }
  revolveAt(sx: number, sy: number, cut = false): void { solids.revolveAt(this, sx, sy, cut); }
  combineAt(sx: number, sy: number): void { solids.combineAt(this, sx, sy); }
  get combining(): Combining | null { return solids.combining(this); }
  finishCombine(): void { solids.finishCombine(this); }
  roundAt(sx: number, sy: number): void { solids.roundAt(this, sx, sy); }
  /** A hit taken as a click on it would be — a row chosen from a ⌘-click's list. */
  takeHit(hit: Hit, add = false): void { takeHit(this, hit, add); }
  finishRound(size?: string): string | null { return solids.finishRound(this, size); }
  endRound(): void { solids.endRound(this); }
  tellRound(): void { solids.tellRound(this); }
  turnsAbout(e: Primitive | null | undefined): string | null { return solids.turnsAbout(this, e); }
  addDatum(kind: 'plane' | 'axis', name?: string): boolean {
    return solids.addDatum(this, kind, name);
  }
  sketchOnFace(pick: SolidPick): boolean { return solids.sketchOnFace(this, pick); }

  extentArrow(): Handle | null { return extent.extentArrow(this); }
  sizeExtent(sx: number, sy: number, both = false): void {
    extent.sizeExtent(this, sx, sy, both);
  }
  commitExtent(text?: string): string | null { return extent.commitExtent(this, text); }
  endExtent(): void { extent.endExtent(this); }
  /** **The planes a sketch can be drawn on**, by name, for the workspace's chooser: the three
   *  standard ones first, whether or not the document has them yet, then every other plane the
   *  document names (`std.up` is the front plane turned, so it is not offered twice). */
  planeChoices(): string[] {
    const ws = this.workspace();
    const names = new Set<string>(STANDARD_PLANES);
    // where the standard planes the chooser offers stand: a standard datum standing there too is
    // one of them turned (`std.up` is the front), and is not offered twice
    const offered = new Set<View>([PAGE]);
    for (const n of STANDARD_PLANES) {
      const pl = this.doc.entity(n);
      if (pl instanceof Plane) offered.add(placeOf(ws, pl.index));
    }
    for (const pl of this.sketch.planes) {
      // a frame declared bare draws in space, and is no place to draw flat (#383)
      if (!pl.planar) continue;
      const n = this.doc.nameOf(pl);
      if (!n || (n.startsWith('std.') && !names.has(n) && offered.has(placeOf(ws, pl.index)))) continue;
      names.add(n);
    }
    return [...names];
  }

  /** The name of the plane being drawn on, as the chooser shows it — `''` while drawing in
   *  space, the chooser's None. */
  get planeName(): string {
    if (this.openSpace) return '';
    if (this.pendingPlane) return this.pendingPlane;
    return (this.plane && this.doc.nameOf(this.plane)) || 'std.front';
  }

  /** **Section the workspace**, or stop: `next` names its plane, which side it keeps and how far
   *  along its normal it is moved.  The core is told (`applySection`) and the box re-cuts. */
  setSection(next: Sectioning | null): void {
    this.section = next;
    this.sectionEpoch++;
    this.applySection();
    this.onSection();
    this.draw();
  }

  /** Section by the plane being drawn on (the front where none is), or stop sectioning. */
  toggleSection(): void {
    if (this.section) return this.setSection(null);
    const plane = this.planeName || 'std.front';
    this.setSection({ plane, flip: false, offset: 0 });
    this.onStatus(`section on ${plane}`);
  }

  /** The frame of the section's plane, where it stands now: a plane of the document, or a
   *  standard one the document does not have yet, read from the library's.  Null where the
   *  section's plane is no plane. */
  sectionFrame(): Frame3 | null {
    if (!this.section) return null;
    const name = this.section.plane;
    const found = this.doc.entity(name);
    if (found instanceof Plane) return found.frame3;
    if (!(STANDARD_PLANES as readonly string[]).includes(name)) return null;
    const std = standardDocument().entity(name);
    return std instanceof Plane ? std.frame3 : null;
  }

  /** **Tell the drawing where it is sectioned**: the cut set on the core's sketch, so the box
   *  draws and the picks pass by what it took away.  A section whose plane no longer resolves
   *  is let go. */
  applySection(): void {
    const frame = this.sectionFrame();
    if (this.section && !frame) {
      this.section = null;
      this.sectionEpoch++;
      this.onSection();
    }
    setCut(this.sketch, frame && this.section && { frame, ...this.section });
  }

  /** **Sketch on a plane**: make it the one the next thing is drawn in, and turn the eye square
   *  on to it unless `look` is false (a double-click on its pane leaves the eye where it is).  A
   *  standard plane the document does not have yet is remembered by name and brought in by the
   *  first press of a tool (`ensurePlane`), so choosing one writes nothing. */
  choosePlane(name: string, look = true): void {
    const found = this.doc.entity(name);
    if (found instanceof Plane) {
      // **choosing a hidden plane shows it**
      hidden.reveal(this, found);
      this.plane = found;
      this.pendingPlane = null;
    } else if ((STANDARD_PLANES as readonly string[]).includes(name)) {
      this.plane = null;
      this.pendingPlane = name;
    } else {
      return this.onStatus(`there is no plane ${name}`);
    }
    this.openSpace = false;
    // the plane is where the next thing goes, not a thing picked: a selected plane would open the
    // constraints window over the drawing about to be made
    if (this.selected.some((e) => e instanceof Plane)) {
      this.selected = this.selected.filter((e) => !(e instanceof Plane));
    }
    if (look) this.lookAt(this.lookFor(name));
    this.onStatus(`drawing on ${name}`);
    this.onChanged();
  }

  /** **Draw in space** — the chooser's None: no plane is current, and what a tool makes next stands
   *  in space where the eye sees it, nought deep (`workspace::unseen`); its constraints say the
   *  rest.  The eye stays where it is — there is no plane to face — and nothing is written. */
  drawInSpace(): void {
    this.plane = null;
    this.pendingPlane = null;
    this.openSpace = true;
    if (this.selected.some((e) => e instanceof Plane)) {
      this.selected = this.selected.filter((e) => !(e instanceof Plane));
    }
    this.onStatus('drawing in space');
    this.onChanged();
  }

  /** The eye square on to a named plane: this document's own where it has the plane, else the
   *  standard library's, read once from a document of nothing but `use std`. */
  private lookFor(name: string): { az: number; el: number } {
    const here = this.doc.entity(name);
    if (here instanceof Plane) return lookOf(this.workspace(), here.index) ?? { ...this.orbit };
    if (name === 'std.front') return lookOf(this.workspace(), PAGE) ?? { ...this.orbit };
    const std = standardDocument();
    const pl = std.entity(name);
    return (pl instanceof Plane && lookOf(workspace(std.sketch), pl.index)) || { ...this.orbit };
  }

  /** Swing the eye to `to`, the short way round, and redraw as it goes.  The camera's own
   *  similarity is left where it is, so what was in the middle of the screen stays there. */
  lookAt(to: { az: number; el: number }): void {
    // where there is no clock to animate by (a test's view), the eye is simply there
    const clock = typeof window !== 'undefined' && typeof window.setInterval === 'function';
    if (clock) window.clearInterval(this.looking);
    const from = { ...this.orbit };
    let daz = to.az - from.az;
    daz -= 2 * Math.PI * Math.round(daz / (2 * Math.PI));
    if (!clock) {
      this.orbit = { az: from.az + daz, el: to.el };
      this.draw();
      return;
    }
    const t0 = performance.now();
    const step = (): void => {
      const t = Math.min(1, (performance.now() - t0) / LOOK_MS);
      const k = t * t * (3 - 2 * t);
      this.orbit = { az: from.az + daz * k, el: from.el + (to.el - from.el) * k };
      this.draw();
      if (t >= 1) {
        window.clearInterval(this.looking);
        this.looking = 0;
      }
    };
    this.looking = window.setInterval(step, 16);
    step();
  }

  /** Bring a chosen standard plane into the document before anything is drawn on it: the
   *  document gains `use std` (one undoable edit) and so the plane, which becomes current.
   *  True when the plane is ready to be drawn on. */
  ensurePlane(): boolean {
    // drawing on a hidden plane shows it, as choosing it does
    if (this.plane) hidden.reveal(this, this.plane);
    if (!this.pendingPlane) return true;
    const name = this.pendingPlane;
    if (!this.apply(this.doc.addUse('std'), `use std, for ${name}`)) return false;
    const pl = this.doc.entity(name);
    if (!(pl instanceof Plane)) {
      this.onStatus(`${name} is not in the standard library`);
      return false;
    }
    this.plane = pl;
    this.pendingPlane = null;
    return true;
  }

  /** How a document opens: square on to the front when everything is drawn there, and from three
   *  quarters when it has a solid or anything stands on another plane. */
  private homeOrbit(): { az: number; el: number } {
    const ws = this.workspace();
    // anything standing elsewhere than the page's place — or across places, a projector — but a
    // plane's own origin, which stands wherever its plane does whether or not it is drawn on
    const origins = new Set(this.sketch.planes.flatMap((pl) => (pl.origin ? [pl.origin.index] : [])));
    const off = Object.entries(ws.views).some(([kind, vs]) =>
      (vs as View[]).some((v, i) => !(kind === 'point' && origins.has(i)) && placeOf(ws, v) !== PAGE));
    return off || objects(this.sketch).length ? { ...THREE_QUARTER } : { ...FRONT };
  }

  startDimension(targets: Constraint[], fresh: boolean, alt: DimAlt | null): boolean {
    return dimension.startDimension(this, targets, fresh, alt);
  }
  endDimension(commit: boolean): void { dimension.endDimension(this, commit); }
  rewriteDimension(text: string): string | null { return dimension.rewriteDimension(this, text); }

  addConstraints(...cs: Constraint[]): void { edit.addConstraints(this, ...cs); }
  removeConstraint(c: Constraint): void { edit.removeConstraint(this, c); }
  deleteSelected(): void { edit.deleteSelected(this); }
  makeComponent(name?: string): boolean { return edit.makeComponent(this, name); }
  toggleConstructionSelected(): void { edit.toggleConstructionSelected(this); }
  toggleFixSelected(): void { edit.toggleFixSelected(this); }
  copySelected(): number { return edit.copySelected(this); }
  cutSelected(): number { return edit.cutSelected(this); }
  pasteClipboard(): number { return edit.pasteClipboard(this); }
  resetCallouts(c?: Constraint | null): number { return edit.resetCallouts(this, c); }
}
