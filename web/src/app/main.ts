/* The sketcher shell: toolbars, the constraint list, the diagnosis banner and the dialogs.
 *
 *   tools     P point · L line · R rectangle · C circle · A arc (centre, start, end)
 *             3 arc through two ends and a point on it
 *   Esc       stop a DOF animation, then drop the tool's pending points, then put the tool
 *             down — Select is not a tool, it is what no tool being held looks like, so it
 *             is also where clicking the pressed toolbar button leaves you
 *   select    click (shift = multi), or drag a box over empty canvas to take everything
 *             that lies entirely inside it
 *   constrain I coincident · D dimension — length, radius, offset, ring, or a corner's angle
 *             H horizontal · V vertical · B parallel · ⇧L perpendicular · ⇧M midpoint
 *             E equal · T tangent · ⇧Q symmetric
 *   dimension every dimensioned constraint is called out on the drawing: click one to select
 *             it, drag it where you want it, double-click it to change its number.  Edit ▸
 *             Re-place dimensions undoes the arranging; Options turns the lot off
 *   editing   F fix/unfix · G construction · Del delete · Ctrl+Z undo · ⇧Ctrl+Z redo ·
 *             Ctrl+X/C/V cut, copy, paste the selection · wheel zoom · right-drag pan
 *   menus     File/Edit/Solution hold everything that is not a tool or a constraint; the
 *             solver's own switches are behind Solution ▸ Options
 *
 * Everything below is presentation; the solver, diagnosis, decomposition and root selection
 * all live in core/ and are shared with the test suite. */
import * as C from '../core/constraints.js';
import * as examples from '../core/examples.js';
import * as io from '../core/io.js';
import { Constraint, ENTITY_KINDS } from '../core/constraints.js';
import { applyAlternative, enumerateStep, isCurrent } from '../core/homotopy.js';
import {
  Arc, Circle, Line, Point, Primitive, Sketch, angleBetween, distanceBetween, expand,
  signedPointToLine,
} from '../core/model.js';
import { METHODS, Method } from '../core/system.js';
import { initCore } from '../core/wasm.js';
import { movingParams, witnessSummary } from '../core/witness.js';
import { SketchView, Tool } from './view.js';
import {
  MenuItem, ToolbarButton, addButton, addCheckbox, addLink, addMenu, addSelect, addSeparator,
  askChoice, askNumber, closeMenus, download, openFile, showReport, showSheet, stats, toast,
} from './ui.js';

const canvas = document.getElementById('canvas') as HTMLCanvasElement;
const menubar = document.getElementById('menubar') as HTMLElement;
const aboutBadge = document.getElementById('about') as HTMLButtonElement;
const aboutDag = document.getElementById('about-dag') as HTMLTemplateElement;
const barTools = document.getElementById('bar-tools') as HTMLElement;
const barConstraints = document.getElementById('bar-constraints') as HTMLElement;
const clist = document.getElementById('clist') as HTMLElement;
const elist = document.getElementById('elist') as HTMLElement;
const banner = document.getElementById('banner') as HTMLElement;
const bannerText = document.getElementById('banner-text') as HTMLElement;
const bannerSelect = document.getElementById('banner-select') as HTMLButtonElement;
const measureEl = document.getElementById('measure') as HTMLElement;
const footerEl = document.querySelector('footer') as HTMLElement;

await initCore();
(document.getElementById('loading') as HTMLElement).remove();

const view = new SketchView(canvas, examples.rectFillets());
let currentConstraint: Constraint | null = null;

/** Move the keyboard focus onto a constraint row, or off it with null.  Delete acts on
 *  whichever of the two selections holds the focus, so exactly one of `currentConstraint` and
 *  `view.selected` is ever populated — that is the whole reason deleting a constraint stopped
 *  taking the geometry with it, so every path that sets either one comes through here. */
function focusConstraint(c: Constraint | null, highlight?: Primitive[]): void {
  currentConstraint = c;
  view.litConstraint = c;             // so its callout on the drawing says so too
  view.highlight = highlight ?? (c ? expand(c.entities()) : []);
  if (c) view.selected = [];
}
let rows: Constraint[] = [];
let erows: Primitive[] = [];
let lastSelKey = '';
const toolButtons = new Map<Tool, HTMLButtonElement>();

/* -- toolbars ---------------------------------------------------------------- */

/** The buttons follow the view, so Escape backing out of a tool updates them too.  Select is
 *  not a button: it is what the toolbar looks like with nothing pressed, so clicking the
 *  pressed tool puts it down and lands you back there. */
view.onTool = (t) => {
  for (const [k, b] of toolButtons) b.setAttribute('aria-pressed', String(k === t));
};
for (const [label, tool, key] of [
  ['Point', 'point', 'p'], ['Line', 'line', 'l'], ['Rect', 'rect', 'r'],
  ['Circle', 'circle', 'c'], ['Arc', 'arc', 'a'], ['Arc 3-pt', 'arc3', '3'],
] as [string, Tool, string][]) {
  toolButtons.set(tool, addButton(barTools, {
    label, key, toggle: true, title: 'Click again to put the tool down and go back to selecting',
    onClick: () => view.setTool(view.tool === tool ? 'select' : tool),
  }));
}
view.setTool('select');
addSeparator(barTools);
/* Beside the tools, past the divider: what the geometry you drew is *for*, as against the
 * constraints that hold it.  A table like the constraints bar's, so the chip and the
 * accelerator stay one string — see ACTION_KEYS. */
const TOOL_BUTTONS: ToolbarButton[] = [
  { label: 'Construction', key: 'g', onClick: () => view.toggleConstructionSelected(),
    title: 'Draw the selected lines/circles/arcs dashed as reference geometry (they still constrain)' },
];
for (const b of TOOL_BUTTONS) addButton(barTools, b);

/* -- menu bar ------------------------------------------------------------------- */

/* Everything that is neither a tool nor a constraint.  Like the constraints bar these are
 * tables rather than calls, so the accelerator printed beside an item is the same string the
 * keyboard handler matches — see ACTION_KEYS. */
const MENUS: [string, (MenuItem | null)[]][] = [
  ['File', [
    { label: 'New', onClick: () => view.setSketch(newSketch()) },
    { label: 'Open…', onClick: () => void doOpen() },
    { label: 'Open test case…', onClick: () => void openCase() },
    null,
    { label: 'Save', onClick: () => download('sketch.json', io.dumps(view.sketch)) },
  ]],
  ['Edit', [
    { label: 'Undo', key: '⌘z', onClick: () => view.undo() },
    { label: 'Redo', key: '⇧⌘z', onClick: () => view.redo() },
    null,
    { label: 'Cut', key: '⌘x', onClick: () => report(view.cutSelected(), 'cut'),
      title: 'Take the selection out of the sketch and onto the clipboard' },
    { label: 'Copy', key: '⌘c', onClick: () => report(view.copySelected(), 'copied'),
      title: 'The selection, the points that define it, and every constraint that stays inside' },
    { label: 'Paste', key: '⌘v', onClick: () => report(view.pasteClipboard(), 'pasted'),
      title: 'A copy of the clipboard, nudged clear and selected, joined to nothing' },
    null,
    { label: 'Fit to screen', onClick: () => view.fit() },
    { label: 'Re-place dimensions', onClick: () => {
      // the focused dimension if there is one, otherwise every dimension on the drawing
      const n = view.resetCallouts(currentConstraint);
      toast(n ? `${n} dimension(s) put back` : 'no dimension has been moved');
    } },
  ]],
  ['Solution', [
    { label: 'Solve', onClick: () => { view.solveNow(); reportSolve(); } },
    { label: 'Diagnose…', onClick: () => void showDiagnosis() },
    null,
    { label: 'Animate DOF', onClick: () => {
      if (!view.startAnimation()) toast('no remaining internal DOF to animate');
    } },
    { label: 'Flip branch', onClick: () => flipBranch() },
    { label: 'Alternatives…', onClick: () => void alternatives() },
    null,
    { label: 'Options…', onClick: () => void options() },
  ]],
];
for (const [label, items] of MENUS) addMenu(menubar, label, items);
aboutBadge.addEventListener('click', () => void about());

/** What the wordmark on the bar opens.  The diagram lives in the page as a template, since a
 *  fixed drawing is markup rather than something to build an element at a time. */
function about(): Promise<void> {
  return showSheet('Geometric Constraint Solver', (body) => {
    const by = document.createElement('p');
    by.textContent = 'A project by Brandon Martin-Anderson';
    const where = document.createElement('p');
    addLink(where, 'GitHub →', 'https://github.com/bmander/geomsolver');
    body.append(by, aboutDag.content.cloneNode(true), where);
  });
}

/** Say what a clipboard command did.  Nothing to copy and nothing to paste are both ordinary
 *  outcomes, not failures, so they say so rather than going quiet. */
function report(n: number, did: string): void {
  toast(n ? `${did} ${n} ${n === 1 ? 'entity' : 'entities'}`
          : did === 'pasted' ? 'nothing on the clipboard' : 'select something first');
}

/** A fresh sheet, which is not quite empty.  With nothing fixed on it the first shape drawn
 *  is free to float — it satisfies its constraints just as well anywhere on the canvas, so a
 *  drag slides the whole thing rather than working against anything.  One fixed point at the
 *  origin gives the drawing somewhere to be. */
function newSketch(): Sketch {
  const sk = new Sketch();
  sk.point(0, 0, true);
  return sk;
}

/** The reference sketches, each with the one line that says what it is there to show.  Asked
 *  for the first time the menu item is picked, so booting costs nothing for a list most
 *  sessions never open. */
let cases: ReturnType<typeof examples.cases> | null = null;
async function openCase(): Promise<void> {
  cases ??= examples.cases();
  const i = await askChoice('Open test case', 'The sketches the solver is exercised on:',
                            cases.map((c) => `${c.label} — ${c.description}`));
  if (i === null) return;
  view.setSketch(examples.build(cases[i].key));
  toast(`${cases[i].label} — ${cases[i].description}`, 12000);
}

/** Everything that changes how the solve runs, gathered behind one item.  The controls are
 *  built from the view each time the sheet opens, so none of them can go stale, and each
 *  takes effect as it is switched. */
function options(): Promise<void> {
  return showSheet('Options', (body) => {
    const box = document.createElement('div');
    box.className = 'fields';
    addCheckbox(box, 'auto-solve', view.autoSolve, (v) => { view.autoSolve = v; },
                'Solve after every edit, rather than on demand');
    addCheckbox(box, 'plan', view.usePlan, (v) => { view.usePlan = v; view.solveNow(); },
                'Solve through the cached decomposition plan instead of one Newton system '
              + 'over the whole sketch');
    addCheckbox(box, 'colour by state', view.colorByState, (v) => { view.colorByState = v; view.draw(); },
                'Paint each entity by what diagnosis makes of it');
    addCheckbox(box, 'dimensions', view.showDimensions,
                (v) => { view.showDimensions = v; view.draw(); },
                'Call out every dimensioned constraint on the drawing — click one to select it, '
              + 'drag it where you want it, double-click to change its number');
    addSelect(box, 'method', [...METHODS], view.method, (m) => {
      view.method = m as Method;
      view.solveNow();
    }, 'The trust-region method the solver runs');
    body.append(box);
  });
}

/* constraints whose arguments are just entities:
 * (label, class, points, lines, circles/arcs, shortcut) */
type Simple = [string, C.ConstraintCtor, number, number, number, string?];
const SIMPLE: Simple[] = [
  ['Horizontal', C.Horizontal, 0, 1, 0, 'h'],
  ['Vertical', C.Vertical, 0, 1, 0, 'v'],
  ['Parallel', C.Parallel, 0, 2, 0, 'b'],
  ['Perpendicular', C.Perpendicular, 0, 2, 0, '⇧l'],
  ['Midpoint', C.Midpoint, 1, 1, 0, '⇧m'],
];
/* One "these touch" button.  Which incidence it means is the selection's business, not the
 * user's: two points meet, a point sits on a line, a point sits on a circle or arc. */
const INCIDENCE: Simple[] = [
  ['Coincident', C.Coincident, 2, 0, 0],
  ['On line', C.PointOnLine, 1, 1, 0],
  ['On circle', C.PointOnCircle, 1, 0, 1],
];
/* The constraints bar, in an order that interleaves the dimensioned constraints with the
 * entity-only ones.  `key` is both the chip printed on the button and the token the keyboard
 * handler matches — '⇧l' prints as ⇧L and fires on shift-L — so a button and its shortcut
 * cannot drift apart. */
const CONSTRAINT_BUTTONS: ToolbarButton[] = [
  { label: 'Coincident', key: 'i', onClick: () => cCoincident(),
    title: 'Two points meet · a point on a line · a point on a circle or arc' },
  { label: 'Dimension', key: 'd', onClick: () => void cDimension(),
    title: 'Put a number on the selection · a length, a radius, an offset, a ring '
         + '· two lines take their gap when parallel and their angle when not' },
  ...SIMPLE.map((c): ToolbarButton => ({ label: c[0], key: c[5], onClick: () => applySimple(c) })),
  { label: 'Equal', key: 'e', onClick: () => cEqual() },
  { label: 'Tangent', key: 't', onClick: () => cTangent() },
  { label: 'Symmetric', key: '⇧q', onClick: () => cSymmetric() },
  { label: 'Fix', key: 'f', onClick: () => view.toggleFixSelected() },
];
for (const b of CONSTRAINT_BUTTONS) addButton(barConstraints, b);

/* -- selection helpers -------------------------------------------------------- */

function sel(): { pts: Point[]; lines: Line[]; circles: (Circle | Arc)[] } {
  const s = view.selected;
  return {
    pts: s.filter((e): e is Point => e instanceof Point),
    lines: s.filter((e): e is Line => e instanceof Line),
    circles: s.filter((e): e is Circle | Arc => e instanceof Circle || e instanceof Arc),
  };
}

function need(ok: boolean, what: string): boolean {
  if (!ok) toast(`select ${what} first`);
  return ok;
}

/** Generic applier: checks the selection has the required counts and passes the entities in
 *  spec order.  Single-line constraints (Horizontal/Vertical) apply to every selected line. */
function applySimple([, cls, nPts, nLines, nCirc]: Simple): void {
  const { pts, lines, circles } = sel();
  const perLine = nPts === 0 && nLines === 1 && nCirc === 0;
  const ok = pts.length === nPts && circles.length === nCirc
    && (perLine ? lines.length >= 1 : lines.length === nLines);
  const what = ([[nPts, 'point(s)'], [nLines, 'line(s)'], [nCirc, 'circle(s)/arc(s)']] as const)
    .filter(([n]) => n).map(([n, w]) => `${n} ${w}`).join(', ');
  if (!need(ok, what)) return;
  const made = (perLine ? lines : [null]).map((ln) => {
    const args: unknown[] = [];
    let pi = 0, li = 0, ci = 0;
    for (const [, kind] of cls.spec) {
      if (kind === 'point') args.push(pts[pi++]);
      else if (kind === 'line') args.push(perLine ? ln : lines[li++]);
      else if (ENTITY_KINDS.has(kind)) args.push(circles[ci++]);
    }
    return new (cls as unknown as new (...a: unknown[]) => Constraint)(...args);
  });
  view.addConstraints(...made);
}

/** The single incidence button: read the selection and pick the constraint that fits it. */
function cCoincident(): void {
  const { pts, lines, circles } = sel();
  const hit = INCIDENCE.find(([, , nPts, nLines, nCirc]) =>
    pts.length === nPts && lines.length === nLines && circles.length === nCirc);
  if (!need(!!hit, 'two points, a point and a line, or a point and a circle/arc')) return;
  applySimple(hit as Simple);
}

/** The one dimension button: what it puts a number on is the selection's business.  Two points
 *  or a single line take a length, a point and a line a signed offset, a circle its radius and
 *  two of them the ring between them — and two lines take the gap between them when they are
 *  parallel, the angle at their corner when they are not. */
async function cDimension(): Promise<void> {
  let { pts } = sel();
  const { lines, circles } = sel();
  if (!pts.length && lines.length === 2) {
    const [a, b] = lines;
    const [ax, ay] = a.direction();
    const [bx, by] = b.direction();
    const s = Math.hypot(ax, ay) * Math.hypot(bx, by);
    if (!need(s > 0, 'lines with two distinct endpoints')) return;
    // "Parallel" here means the sketch makes them so, not that they merely look it: a solved
    // Parallel sits within a residual scaled to the sketch's extent, which on a short line is
    // a few ten-thousandths of a radian.  Anything looser is a corner, and what a drawing
    // dimensions on a corner is its angle.
    return Math.abs(ax * by - ay * bx) <= 1e-3 * s ? cParallelDistance(a, b) : cAngle(a, b);
  }
  if (pts.length === 1 && lines.length === 1) return cPointLineDistance(pts[0], lines[0]);
  // a dimension *between* two circles is the ring; on any other number of them it is the
  // radius they are each to have
  if (!pts.length && !lines.length && circles.length) {
    return circles.length === 2 ? cAnnularDistance(circles[0], circles[1]) : cRadius(circles);
  }
  if (!pts.length && lines.length === 1) pts = [lines[0].p1, lines[0].p2];
  if (!need(pts.length === 2, 'two points, one line, a point and a line, two lines, '
                            + 'or one or more circles/arcs')) return;
  const cur = Math.hypot(pts[0].x.value - pts[1].x.value, pts[0].y.value - pts[1].y.value);
  const v = await askNumber('Distance', 'Distance', cur);
  if (v !== null) view.addConstraints(new C.Distance(pts[0], pts[1], v));
}

/** Two parallel lines: dimension the gap between them.  It does not make them parallel — the
 *  caller has already established that they are, and sent the other case to Angle, because a
 *  "gap" between converging lines pins one endpoint's offset and reads as arbitrary. */
async function cParallelDistance(l1: Line, l2: Line): Promise<void> {
  const cur = signedPointToLine(l2.p1.x.value, l2.p1.y.value, l1);
  const v = await askNumber('Parallel distance', 'Gap (negative puts the second line on the other side)', cur);
  if (v !== null) view.addConstraints(new C.ParallelDistance(l1, l2, v));
}

/** A point and a line: dimension the point's perpendicular offset, signed so negating it moves
 *  the point across.  Measured to the infinite line — the foot may fall off the end of the
 *  segment, which is what a drawing means by "distance to this edge". */
async function cPointLineDistance(p: Point, line: Line): Promise<void> {
  const [dx, dy] = line.direction();
  if (!need(Math.hypot(dx, dy) > 0, 'a line with two distinct endpoints')) return;
  if (!need(p !== line.p1 && p !== line.p2, 'a point that is not an endpoint of the line')) return;
  const v = await askNumber('Point-line distance', 'Offset (negative puts the point on the other side)',
                            signedPointToLine(p.x.value, p.y.value, line));
  if (v !== null) view.addConstraints(new C.PointLineDistance(p, line, v));
}

/** Two circles or arcs: dimension the annulus between them.  Like the parallel gap it sizes
 *  the ring without centring it, so say so when the centres are not already together. */
async function cAnnularDistance(c1: Circle | Arc, c2: Circle | Arc): Promise<void> {
  const cur = Math.abs(c2.radius.value) - Math.abs(c1.radius.value);
  const v = await askNumber('Annular distance',
                            'Ring thickness (negative makes the first circle the outer one)', cur);
  if (v === null) return;
  view.addConstraints(new C.AnnularDistance(c1, c2, v));
  const [a, b] = [c1.center, c2.center];
  if (Math.hypot(a.x.value - b.x.value, a.y.value - b.y.value) > 1e-9) {
    toast('the ring is dimensioned, but these circles are not concentric — add Coincident on their centres');
  }
}

/** Two lines that meet at a corner: dimension the corner. */
async function cAngle(l1: Line, l2: Line): Promise<void> {
  const cur = (angleBetween(l1, l2) * 180) / Math.PI;   // the core's rule, in degrees
  const v = await askNumber('Angle', 'Angle from the first to the second line (degrees)', cur);
  if (v !== null) view.addConstraints(new C.Angle(l1, l2, (v * Math.PI) / 180));
}

/** An equality set: every selected line the same length, or every selected circle/arc the
 *  same radius.  n entities need n-1 constraints, all against the first — a star rather than
 *  a cycle, since closing the loop would make one equation redundant (which is exactly what
 *  `polygon_chain` does on purpose, to have a redundancy the graph cannot see). */
function cEqual(): void {
  const { lines, circles } = sel();
  if (lines.length >= 2 && !circles.length) {
    view.addConstraints(...lines.slice(1).map((l) => new C.EqualLength(lines[0], l)));
  } else if (circles.length >= 2 && !lines.length) {
    view.addConstraints(...circles.slice(1).map((c) => new C.EqualRadius(circles[0], c)));
  } else {
    need(false, 'two or more lines, or two or more circles/arcs (not a mix)');
  }
}

/** Two points mirrored across a line — pick the two points and the axis. */
function cSymmetric(): void {
  const { pts, lines } = sel();
  if (!need(pts.length === 2 && lines.length === 1, 'two points and a line (the mirror axis)')) return;
  view.addConstraints(new C.Symmetric(pts[0], pts[1], lines[0]));
}

function cTangent(): void {
  const { lines, circles } = sel();
  if (lines.length === 1 && circles.length === 1) {
    const ln = lines[0], cc = circles[0];
    if (cc instanceof Arc) {
      const ends = new Set([ln.p1, ln.p2]);
      if (ends.has(cc.start)) return view.addConstraints(new C.TangentArcLine(cc, ln, 'start'));
      if (ends.has(cc.end)) return view.addConstraints(new C.TangentArcLine(cc, ln, 'end'));
    }
    view.addConstraints(new C.TangentLineCircle(ln, cc));
  } else if (circles.length === 2 && !lines.length) {
    // the sense is left out: the core reads it off the geometry, the same rule everywhere
    view.addConstraints(new C.TangentCircleCircle(circles[0], circles[1]));
  } else {
    need(false, 'a line and a circle/arc, or two circles/arcs');
  }
}

/** One circle or arc takes its radius; several are all given the same one. */
async function cRadius(circles: (Circle | Arc)[]): Promise<void> {
  const v = await askNumber('Radius', 'Radius', Math.abs(circles[0].radius.value));
  if (v === null) return;
  view.addConstraints(...circles.map((cc) => new C.Radius(cc, v)));
}

/* -- Stage 5: root selection ---------------------------------------------------- */

/** A selected tangency row toggles its inside/outside flag; a selected point flips the
 *  closed-form constructions that place it (the other circle-circle intersection), recorded
 *  in the sketch's branches and replayed sticky. */
function flipBranch(): void {
  const c = currentConstraint;
  if (c && C.isType(c, 'TangentLineCircle')) {
    view.pushUndo();
    c.setValue('side', -Number(c.side));
    view.afterEdit();
    toast(`flipped the tangency side of ${io.describe(c, view.sketch)}`);
    return;
  }
  if (c && C.isType(c, 'TangentCircleCircle')) {
    view.pushUndo();
    const now = !c.external;
    c.setValue('external', now);
    view.afterEdit();
    toast(`flipped to ${now ? 'external' : 'internal'} tangency`);
    return;
  }
  const pts = view.selected.filter((e): e is Point => e instanceof Point);
  if (!pts.length) {
    toast('select a point (or a tangency row) to flip its solution branch');
    return;
  }
  const ps = view.plan();
  view.pushUndo();
  const n = pts.reduce((s, p) => s + ps.flip(p), 0);
  if (!n) {
    toast('no closed-form construction places the selected point(s)');
    return;
  }
  const r = ps.solve();
  view.afterEdit();
  toast(`flipped ${n} construction(s)${r.success ? '' : ' — the other root is not reachable here'}`);
}

/** Enumerate the real solutions of the construction that places the selected point (homotopy
 *  continuation on its merge system) and let the user pick one. */
async function alternatives(): Promise<void> {
  const pts = view.selected.filter((e): e is Point => e instanceof Point);
  if (pts.length !== 1) {
    toast('select exactly one point');
    return;
  }
  const ps = view.plan();
  const placing = ps.stepsPlacing(pts[0]);
  if (!placing.length) {
    toast('no construction places that point (under-constrained or not decomposable)');
    return;
  }
  const idx = placing[0][0];
  const alts = enumerateStep(ps, idx, { locate: pts[0] });
  if (alts.length < 2) {
    toast(`${alts.length} real solution(s) for this construction — nothing to choose`);
    return;
  }
  const labels = alts.map((a) => (isCurrent(a) ? '● current — ' : '')
    + (a.location ? `point at (${io.fmt(a.location[0], 3)}, ${io.fmt(a.location[1], 3)})`
                  : `distance ${io.fmt(a.distance, 3)}`));
  const pick = await askChoice('Alternative solutions',
    `${alts.length} real solutions of this construction:`, labels);
  if (pick === null || isCurrent(alts[pick])) return;
  view.pushUndo();
  applyAlternative(ps, idx, alts[pick]);
  const res = view.afterEdit();
  // a root of the isolated merge system is not always reachable through a whole-plan replay
  // (the leaves are re-derived from the new geometry, and the surrounding merges may pull it
  // back) — say so rather than leaving an unexplained conflict on screen
  toast(!res || res.success ? 'switched to the chosen solution'
    : 'that root is not reachable from here — the replay could not keep it (undo to go back)');
}

/* -- reports -------------------------------------------------------------------- */

function reportSolve(): void {
  const r = view.lastResult;
  if (r) toast(`${r.success ? 'solved' : 'NOT CONVERGED'} — max|r| = ${r.maxResidual.toExponential(1)} in ${(r.timeS * 1e3).toFixed(1)} ms`);
}

async function showDiagnosis(): Promise<void> {
  const d = view.diagnosis;
  if (!d) {
    await showReport('Diagnosis', 'No constraints.');
    return;
  }
  const sk = view.sketch;
  const ix = new io.Index(sk);
  const lines: string[] = [];
  const { summary } = await import('../core/diagnose.js');
  lines.push(summary(d), '');
  if (d.conflicts?.length) {
    lines.push('Conflict — remove one of:', ...d.conflicts.map((c) => `   ✗ ${io.describe(c, ix)}`), '');
  }
  if (d.over.length) {
    lines.push(`Structurally redundant block (${d.nRedundant} equation(s) too many):`,
               ...d.over.map((c) => `   • ${io.describe(c, ix)}`), '');
  }
  if (d.underParams.length) {
    const names = (['point', 'circle', 'arc'] as const)
      .flatMap((k) => sk.entities(k)).filter((e) => d.entityState.get(e) === 'under')
      .map((e) => ix.name(e)).sort();
    lines.push(`Under-constrained (${d.dof} DOF): ${names.join(', ')}`, '');
  }
  if (d.components.length > 1) {
    lines.push('Components: ' + d.components.map((c) => `${c.params.length} params / DOF ${c.dof}`).join(', '), '');
  }
  const big = d.rigidClusters.filter((c) => c.length >= 3);
  if (big.length) {
    lines.push('Rigid clusters (distance graph): '
      + big.map((c) => '{' + c.map((p) => ix.name(p)).sort().join(', ') + '}').join('; '), '');
  }
  const rep = view.witnessReport();
  if (rep) {
    lines.push(`Witness analysis: ${witnessSummary(rep)}`);
    for (const dep of rep.dependencies) {
      lines.push(`   ⟂ ${io.describe(dep.constraint, ix)} is implied by `
        + dep.impliedBy.slice(0, 6).map((c) => io.describe(c, ix)).join(', ')
        + (dep.theorem ? '  [theorem-type: invisible to structural analysis]' : ''));
    }
    const internal = rep.motions.filter((m) => !m.rigid);
    internal.slice(0, 8).forEach((m, i) => {
      lines.push(`   DOF ${i + 1}: ` + movingParams(m).slice(0, 10).map((p) => p.name).join(', '));
    });
    if (internal.length) lines.push('   (use “Animate DOF” to see them move)');
    lines.push('');
  }
  lines.push(...d.warnings);
  if (view.lastPlan) {
    lines.push('', `Decomposition: ${view.lastPlan.plan.summary}`
      + (view.lastPlan.fellBack ? ' — numeric fallback used' : ''));
  }
  await showReport('Diagnosis', lines.join('\n'));
}

async function doOpen(): Promise<void> {
  const text = await openFile();
  if (!text) return;
  try {
    view.setSketch(io.loads(text));
    toast('loaded');
  } catch (err) {
    toast(`could not load: ${(err as Error).message}`);
  }
}

/* -- constraint list and status ---------------------------------------------------- */

function rebuildRows(next: Constraint[]): void {
  const ix = new io.Index(view.sketch);
  clist.replaceChildren();
  next.forEach((c) => {
    const li = document.createElement('li');
    li.dataset.base = io.describe(c, ix);
    li.textContent = li.dataset.base;
    li.addEventListener('click', () => {
      focusConstraint(c);
      refresh();
      view.draw();
    });
    li.addEventListener('dblclick', () => void editValue(c));
    clist.append(li);
  });
  rows = next;
}

async function editValue(c: Constraint): Promise<void> {
  for (const [attr, kind] of c.spec) {
    if (kind !== 'length' && kind !== 'angle') continue;
    const deg = kind === 'angle';
    const rec = c as unknown as Record<string, number>;
    const label = `${c.typeName} ${attr}${deg ? ' (degrees)' : ''}`;
    const v = await askNumber(label, label, deg ? (rec[attr] * 180) / Math.PI : rec[attr]);
    if (v !== null) {
      view.pushUndo();
      rec[attr] = deg ? (v * Math.PI) / 180 : v;
      rows = [];                        // force the row text to rebuild
      view.afterEdit();
    }
    return;
  }
  toast(`${c.typeName} has no editable dimension`);
}

/** Same objects in the same order — the lists are rebuilt only when this fails, so rows keep
 *  their DOM nodes (and the user's scroll position) across ordinary edits and drags. */
function sameList<T>(a: readonly T[], b: readonly T[]): boolean {
  return a.length === b.length && a.every((c, i) => c === b[i]);
}

/** A component row's fixed part: its short name, its type, and the points that define it, so
 *  the list reads as the sketch's structure rather than as coordinates that churn on every
 *  drag.  Live values belong in the measurement readout, not here. */
function describeEntity(e: Primitive, ix: io.Index): string {
  const n = ix.name(e).padEnd(4);
  if (e instanceof Line) return `${n}line    ${ix.name(e.p1)}–${ix.name(e.p2)}`;
  if (e instanceof Arc) return `${n}arc     @${ix.name(e.center)} ${ix.name(e.start)}–${ix.name(e.end)}`;
  if (e instanceof Circle) return `${n}circle  @${ix.name(e.center)}`;
  return `${n}point`;
}

function rebuildERows(next: Primitive[]): void {
  const ix = new io.Index(view.sketch);
  elist.replaceChildren();
  next.forEach((e) => {
    const li = document.createElement('li');
    li.dataset.base = describeEntity(e, ix);
    li.addEventListener('click', (ev) => {
      if (ev.shiftKey) {
        const i = view.selected.indexOf(e);
        if (i >= 0) view.selected.splice(i, 1);
        else view.selected.push(e);
      } else {
        view.selected = [e];
      }
      focusConstraint(null);          // the canvas selection has the focus, so Del means geometry
      refresh();
      view.draw();
    });
    elist.append(li);
  });
  erows = next;
}

/** The component list, mirrored from the constraint list above it: a row is marked when the
 *  entity is selected, and softly marked when the focused constraint reaches it — so picking
 *  either list shows you what it touches in the other. */
function refreshERows(): void {
  const next = view.sketch.primitives();
  if (!sameList(next, erows)) rebuildERows(next);

  const sel = new Set(view.selected);
  const lit = new Set(view.highlight);
  erows.forEach((e, i) => {
    const li = elist.children[i] as HTMLElement;
    const fixed = e instanceof Point && e.isFixed;
    const constr = !(e instanceof Point) && e.construction;
    li.textContent = `${li.dataset.base}${fixed ? '  ·fixed' : ''}${constr ? '  ·constr' : ''}`;
    li.classList.toggle('sel', sel.has(e));
    li.classList.toggle('touches', !sel.has(e) && lit.has(e));
    li.classList.toggle('construction', constr);
  });

  // a highlight you cannot see is no highlight: bring the first selected row into view, but
  // only when the selection actually changed, so the list stays put while you scroll it
  const key = view.selected.map((e) => erows.indexOf(e)).join(',');
  if (key !== lastSelKey) {
    lastSelKey = key;
    const first = erows.indexOf(view.selected[0]);
    if (first >= 0) (elist.children[first] as HTMLElement).scrollIntoView({ block: 'nearest' });
  }
}

/** Rows and banner: everything that only changes when the sketch or the selection does. */
function refreshRows(): void {
  const sk = view.sketch;
  const next = sk.userConstraints();
  if (!sameList(next, rows)) rebuildRows(next);
  if (currentConstraint && !rows.includes(currentConstraint)) currentConstraint = null;

  const d = view.diagnosis;
  const selDirect = new Set(view.selected);
  const selAll = new Set(expand(view.selected));
  const culprits = new Set(d?.conflicts ?? []);
  const bad = new Set(d?.violated ?? []);          // culprits are handled first, below
  const over = new Set(d?.over ?? []);
  const implied = new Set(d?.implied ?? []);   // a theorem made it follow: a note, not a fault
  rows.forEach((c, i) => {
    const li = clist.children[i] as HTMLElement;
    const base = li.dataset.base ?? '';
    li.className = '';
    li.removeAttribute('title');
    if (culprits.has(c)) { li.textContent = `✗ ${base}`; li.classList.add('culprit'); }
    else if (bad.has(c)) { li.textContent = base; li.classList.add('violated'); }
    else if (over.has(c)) { li.textContent = `≈ ${base}`; li.classList.add('over'); }
    else if (implied.has(c)) {
      li.textContent = `≡ ${base}`;
      li.classList.add('implied');
      li.title = 'already implied by the other constraints — consistent, nothing to fix';
    }
    else li.textContent = base;
    const hit = selAll.size > 0 && (c.entities().some((e) => selAll.has(e))
      || expand(c.entities()).some((e) => selDirect.has(e)));
    li.classList.toggle('touches', hit);
    li.setAttribute('aria-current', String(c === currentConstraint));
  });
  refreshERows();
  refreshBanner();
}

/** Measurement readout: with exactly two entities picked, what separates them.
 *
 *  Distances come from `distanceBetween` in the model, so the readout and any constraint you
 *  then apply agree on what "distance" means — lines are infinite, arcs are their circle.
 *  Two lines also get their angle, which is the informative number when they are not
 *  parallel and the distance is 0 by definition. */
function refreshMeasure(): void {
  const [a, b] = view.selected;
  if (!b || view.selected.length !== 2) {
    measureEl.className = '';
    return;
  }
  const ix = new io.Index(view.sketch);
  const rows = [`<b>${ix.name(a)} → ${ix.name(b)}</b>`];
  rows.push(`distance  ${io.fmt(distanceBetween(a, b), 6)}`);
  if (a instanceof Point && b instanceof Point) {
    rows.push(`Δx ${io.fmt(b.x.value - a.x.value, 6)}   Δy ${io.fmt(b.y.value - a.y.value, 6)}`);
  } else if (a instanceof Line && b instanceof Line) {
    rows.push(`angle     ${io.fmt((angleBetween(a, b) * 180) / Math.PI, 4)}°`);
  }
  measureEl.innerHTML = rows.join('\n');
  measureEl.className = 'on';
}

/** The status line — the only thing that changes on every frame of a drag.  How much freedom
 *  is left is the one number worth watching while you work; anything that stops the drawing
 *  satisfying its constraints displaces it, since then no count on screen is describing the
 *  sketch you are looking at.  The detail lives in Solve and Diagnose. */
function refreshStatus(): void {
  const d = view.diagnosis;
  const r = view.lastResult;
  const conflict = d?.status === 'conflict';
  footerEl.classList.toggle('unsolved', conflict || (!!r && !r.success));
  // nothing is diagnosed until there is a constraint to diagnose, and until then the freedom
  // left is simply every free parameter — there are no equations for one to be spent on
  stats(conflict ? '⚠ CONFLICT'
      : r && !r.success ? `⚠ NOT CONVERGED  max|r|=${r.maxResidual.toExponential(1)}`
      : `DOF ${d ? d.dof : view.sketch.freeIndices().length}`);
  refreshMeasure();
}

function refresh(): void {
  refreshRows();
  refreshStatus();
}

function refreshBanner(): void {
  const d = view.diagnosis;
  banner.className = '';
  if (!d || (d.status !== 'conflict' && d.status !== 'over')) return;
  const ix = new io.Index(view.sketch);
  if (d.status === 'conflict') {
    bannerText.textContent = d.conflicts?.length
      ? `⚠ Conflicting constraints — remove one of: ${d.conflicts.map((c) => io.describe(c, ix)).join(', ')}`
      : `⚠ ${d.violated.length} constraint(s) cannot be satisfied`;
    banner.className = 'conflict';
  } else {
    bannerText.textContent = `⚠ ${d.nRedundant} redundant equation(s) (consistent, but over-constrained) among: `
      + d.over.map((c) => io.describe(c, ix)).join(', ');
    banner.className = 'over';
  }
  bannerSelect.hidden = !(d.conflicts?.length || d.status === 'over');
}

bannerSelect.addEventListener('click', () => {
  const d = view.diagnosis;
  if (!d) return;
  const culprits = d.conflicts?.length ? d.conflicts : d.over;
  // the banner names a *set*, so show all of it — but the focus stays on the one constraint
  // Delete would remove, rather than on the geometry the set happens to touch
  focusConstraint(culprits[0] ?? null, expand(new Set(culprits.flatMap((c) => c.entities()))));
  refresh();
  view.draw();
});

/* -- keyboard ------------------------------------------------------------------- */

const TOOL_KEYS: Record<string, Tool> = {
  p: 'point', l: 'line', r: 'rect', c: 'circle', a: 'arc', 3: 'arc3',
};
/** Every accelerator in the app, read off the buttons and menu items themselves so there is
 *  one list and not two.  The token is the chip the control prints, lowercased: '⇧l', '⌘z'. */
const ACTION_KEYS = new Map<string, () => void>(
  [...TOOL_BUTTONS, ...CONSTRAINT_BUTTONS, ...MENUS.flatMap(([, items]) => items)]
    .flatMap((b) => (b?.key ? [[b.key, b.onClick] as [string, () => void]] : [])));

window.addEventListener('keydown', (e) => {
  const t = e.target as HTMLElement | null;
  if (t && (t.tagName === 'INPUT' || t.tagName === 'SELECT' || t.tagName === 'TEXTAREA')) return;
  if (e.metaKey || e.ctrlKey || e.altKey) {
    // ⌘C and ⌘X belong to the page while there is text selected in it: taking those would stop
    // anyone copying a constraint out of the list
    const text = window.getSelection();
    const takingText = !!text && !text.isCollapsed && (e.key === 'c' || e.key === 'x');
    const cmd = e.altKey || takingText ? undefined
              : ACTION_KEYS.get(`${e.shiftKey ? '⇧' : ''}⌘${e.key.toLowerCase()}`);
    if (cmd) { e.preventDefault(); cmd(); }
    return;
  }
  const k = e.key.toLowerCase();
  if (k === 'escape') { if (!closeMenus()) view.cancelTool(); return; }
  if (k === 'delete' || k === 'backspace') {
    e.preventDefault();
    if (currentConstraint) {
      const c = currentConstraint;
      focusConstraint(null);
      view.removeConstraint(c);
      toast(`removed ${c.typeName}`);
    } else {
      view.deleteSelected();
    }
    return;
  }
  // shift is part of the token, so ⇧L is Perpendicular and never the Line tool
  const action = ACTION_KEYS.get(e.shiftKey ? `⇧${k}` : k);
  if (action) { action(); return; }
  if (!e.shiftKey && TOOL_KEYS[k]) view.setTool(TOOL_KEYS[k]);
});

/* -- boot ------------------------------------------------------------------------- */

view.onSelect = () => { if (currentConstraint) focusConstraint(null); };
/* A dimension on the drawing and its row in the list are the same constraint, so clicking
 * either does the same thing — and double-clicking either opens the same number. */
view.onPickConstraint = (c) => { focusConstraint(c); refresh(); view.draw(); };
view.onEditConstraint = (c) => void editValue(c);
view.onChanged = refresh;
view.onDragFrame = refreshStatus;
view.onStatus = toast;
new ResizeObserver(() => view.resize()).observe(canvas);
view.resize();
view.fit();
view.afterEdit();
