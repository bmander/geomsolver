/* The sketcher shell: toolbars, the constraint list, the diagnosis banner and the dialogs.
 *
 *   tools     P point · L line · R rectangle · C circle · A arc (centre, start, end)
 *             3 arc through two ends and a point on it
 *   Esc       stop a DOF animation, then drop the tool's pending points, then put the tool
 *             down — Select is not a tool, it is what no tool being held looks like, so it
 *             is also where clicking the pressed toolbar button leaves you
 *   select    click (shift = multi), or drag a box over empty canvas to take everything
 *             that lies entirely inside it
 *   constrain I coincident · D dimension — length, run, rise, radius, offset, ring, or a
 *             corner's angle
 *             H horizontal · V vertical · B parallel · ⇧L perpendicular · ⇧M midpoint
 *             E equal · T tangent · ⇧Q symmetric
 *   dimension D states one at once and opens its number where it will be read: move it where
 *             you want it and click to plant it, type, Enter — Esc takes the whole thing back.
 *             On two points, where you put it is *which* dimension it is: above or below them
 *             the run between them, out to either side the rise, across them their length.
 *             Every dimension is called out on the drawing: click one to select it, drag it
 *             where you want it, double-click it to change its number.  Edit ▸ Re-place
 *             dimensions undoes the arranging; Options turns the lot off.  A number may be an
 *             expression — `w = 80` names it, `h = w / 2` and `sin(h * 10)` use it — and the
 *             core evaluates them in dependency order (Solution ▸ Diagnose lists them)
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
  Arc, Circle, Line, Point, Primitive, Sketch, Spline, angleBetween, distanceBetween, expand,
  signedPointToLine,
} from '../core/model.js';
import { METHODS, Method } from '../core/system.js';
import { initCore } from '../core/wasm.js';
import { movingParams, witnessSummary } from '../core/witness.js';
import { expressions } from '../core/expr.js';
import { DimAlt, SketchView, Tool } from './view.js';
import {
  MenuItem, ToolbarButton, addButton, addCheckbox, addLink, addMenu, addSelect, addSeparator,
  askChoice, closeMenus, download, openFile, showReport, showSheet, stats,
  toast,
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

/** The sketch the page opens on: the case named in the URL — `?example=pythagoras`, or an
 *  `…/example/<slug>` path the server handed to the page — else the default.  The slug is a case
 *  key, arguments and all (`truss:50`); one nothing answers to is said and the default shown. */
function initialSketch(): Sketch {
  const url = new URL(location.href);
  const slug = url.searchParams.get('example') ?? /\/example\/([^/]+)\/?$/.exec(url.pathname)?.[1];
  if (slug) {
    try {
      return examples.build(decodeURIComponent(slug));
    } catch (err) {
      setTimeout(() => toast(`no example “${slug}”: ${(err as Error).message}`, 12000), 0);
    }
  }
  return examples.rectFillets();
}

const view = new SketchView(canvas, initialSketch());
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
  ['Spline', 'spline', 's'], ['Spline fit', 'splinefit', 'w'],
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
              + 'drag it where you want it, double-click to change its number.  Asking for a '
              + 'dimension turns them back on: the number is edited where it is drawn');
    addSelect(box, 'method', [...METHODS], view.method, (m) => {
      view.method = m as Method;
      view.solveNow();
    }, 'The trust-region method the solver runs');
    body.append(box);
  });
}

/* constraints whose arguments are just entities:
 * (label, class, points, lines, circles/arcs, shortcut) */
type Simple = [string, C.ConstraintCtor, number, number, number, string?, number?];
const SIMPLE: Simple[] = [
  ['Parallel', C.Parallel, 0, 2, 0, 'b'],
  ['Perpendicular', C.Perpendicular, 0, 2, 0, '⇧l'],
  ['Midpoint', C.Midpoint, 1, 1, 0, '⇧m'],
];
/* Level and plumb read the selection too.  A pair of points says exactly what a line through
 * them would — that the segment between them is level — and wanting that without drawing the
 * line is the common case: two corners of a shape that share no edge. */
const LEVEL: Simple[] = [
  ['Horizontal', C.Horizontal, 0, 1, 0],
  ['Horizontal', C.HorizontalPoints, 2, 0, 0],
  ['Vertical', C.Vertical, 0, 1, 0],
  ['Vertical', C.VerticalPoints, 2, 0, 0],
];

function cLevel(label: 'Horizontal' | 'Vertical'): void {
  const { pts, lines, circles } = sel();
  const hit = LEVEL.find(([l, , nPts, nLines, nCirc]) =>
    l === label && circles.length === nCirc && pts.length === nPts
    && (nLines === 1 ? lines.length >= 1 : lines.length === nLines));
  if (!need(!!hit, 'one or more lines, or two points')) return;
  applySimple(hit as Simple);
}

/* One "these touch" button.  Which incidence it means is the selection's business, not the
 * user's: two points meet, a point sits on a line, a point sits on a circle or arc. */
const INCIDENCE: Simple[] = [
  ['Coincident', C.Coincident, 2, 0, 0],
  ['On line', C.PointOnLine, 1, 1, 0],
  ['On circle', C.PointOnCircle, 1, 0, 1],
  // a curve is one more row, not a branch: `applySimple` fills the spec's slots by kind, and
  // the contact's hidden parameter is not an entity slot so it is left for the core to seed
  ['On curve', C.PointOnSpline, 1, 0, 0, undefined, 1],
];
/* The constraints bar, in an order that interleaves the dimensioned constraints with the
 * entity-only ones.  `key` is both the chip printed on the button and the token the keyboard
 * handler matches — '⇧l' prints as ⇧L and fires on shift-L — so a button and its shortcut
 * cannot drift apart. */
const CONSTRAINT_BUTTONS: ToolbarButton[] = [
  { label: 'Coincident', key: 'i', onClick: () => cCoincident(),
    title: 'Two points meet · a point on a line · a point on a circle, arc or curve' },
  { label: 'Dimension', key: 'd', onClick: () => cDimension(),
    title: 'Put a number on the selection, then place it and type · a length, a radius, an '
         + 'offset, a ring · on two points, above them is the run and beside them the rise '
         + '· two lines take their gap when parallel and their angle when not' },
  { label: 'Horizontal', key: 'h', onClick: () => cLevel('Horizontal'),
    title: 'Level: one or more lines, or a pair of points with no line between them' },
  { label: 'Vertical', key: 'v', onClick: () => cLevel('Vertical'),
    title: 'Plumb: one or more lines, or a pair of points with no line between them' },
  ...SIMPLE.map((c): ToolbarButton => ({ label: c[0], key: c[5], onClick: () => applySimple(c) })),
  { label: 'Equal', key: 'e', onClick: () => cEqual() },
  { label: 'Tangent', key: 't', onClick: () => cTangent(),
    title: 'A line or a circle tangent to a circle/arc · a line tangent to a curve '
         + '· a circle taking a curve\'s own radius where it touches' },
  { label: 'Symmetric', key: '⇧q', onClick: () => cSymmetric() },
  { label: 'Fix', key: 'f', onClick: () => view.toggleFixSelected() },
];
for (const b of CONSTRAINT_BUTTONS) addButton(barConstraints, b);

/* -- selection helpers -------------------------------------------------------- */

function sel(): {
  pts: Point[]; lines: Line[]; circles: (Circle | Arc)[]; splines: Spline[];
} {
  const s = view.selected;
  return {
    pts: s.filter((e): e is Point => e instanceof Point),
    lines: s.filter((e): e is Line => e instanceof Line),
    circles: s.filter((e): e is Circle | Arc => e instanceof Circle || e instanceof Arc),
    splines: s.filter((e): e is Spline => e instanceof Spline),
  };
}

function need(ok: boolean, what: string): boolean {
  if (!ok) toast(`select ${what} first`);
  return ok;
}

/** Generic applier: checks the selection has the required counts and passes the entities in
 *  spec order.  Single-line constraints (Horizontal/Vertical) apply to every selected line. */
function applySimple([, cls, nPts, nLines, nCirc, , nSpl = 0]: Simple): void {
  const { pts, lines, circles, splines } = sel();
  const perLine = nPts === 0 && nLines === 1 && nCirc === 0;
  const ok = pts.length === nPts && circles.length === nCirc && splines.length === nSpl
    && (perLine ? lines.length >= 1 : lines.length === nLines);
  const what = ([[nPts, 'point(s)'], [nLines, 'line(s)'], [nCirc, 'circle(s)/arc(s)'],
                 [nSpl, 'curve(s)']] as const)
    .filter(([n]) => n).map(([n, w]) => `${n} ${w}`).join(', ');
  if (!need(ok, what)) return;
  const made = (perLine ? lines : [null]).map((ln) => {
    const args: unknown[] = [];
    let pi = 0, li = 0, ci = 0, si = 0;
    for (const [, kind] of cls.spec) {
      if (kind === 'point') args.push(pts[pi++]);
      else if (kind === 'line') args.push(perLine ? ln : lines[li++]);
      else if (kind === 'spline') args.push(splines[si++]);
      else if (ENTITY_KINDS.has(kind)) args.push(circles[ci++]);
      // a `param` slot is not an entity: it is left out, and the core seeds it off the geometry
    }
    return new (cls as unknown as new (...a: unknown[]) => Constraint)(...args);
  });
  view.addConstraints(...made);
}

/** The single incidence button: read the selection and pick the constraint that fits it. */
function cCoincident(): void {
  const { pts, lines, circles, splines } = sel();
  const hit = INCIDENCE.find(([, , nPts, nLines, nCirc, , nSpl = 0]) =>
    pts.length === nPts && lines.length === nLines && circles.length === nCirc
    && splines.length === nSpl);
  if (!need(!!hit, 'two points, a point and a line, or a point and a circle/arc/curve')) return;
  applySimple(hit as Simple);
}

/** One of the three dimensions between two points, stated at what the sketch measures now.
 *  The pair is ordered so the number reads positive: a run or a rise is signed from the first
 *  point to the second, so which of the two comes first is what its sign says. */
function pairDim(kind: string, a: Point, b: Point): Constraint {
  // a run measures x and a rise measures y; a length has no axis of its own, and no sign
  const axis = kind === 'HorizontalDistance' ? 0 : kind === 'VerticalDistance' ? 1 : null;
  const [p, q] = axis !== null && b.xy[axis] < a.xy[axis] ? [b, a] : [a, b];
  const v = axis !== null ? q.xy[axis] - p.xy[axis]
                          : Math.hypot(q.x.value - p.x.value, q.y.value - p.y.value);
  return C.build(kind, [p, q, v]);
}

/** Put a dimension on the selection — the one path all six dimension buttons take.
 *
 *  It is stated at once, at what it measures now, and its number is opened on the drawing
 *  where it will stay: no box in the middle of the screen, and nothing to accept before the
 *  drawing shows what was asked for.  `alt` is what else the same selection could have meant,
 *  which is then settled by where the number is put.
 *
 *  A relation the selection *already states* is edited rather than stated again: a second
 *  Distance on the same pair is not a second fact about them, it is the same fact written
 *  twice, and the only thing that can come of adding it is a conflict.  So the ones that are
 *  there are opened for editing and only the ones that are not get added — the mixed case is
 *  real: dimensioning three circles when one of them already has a radius. */
function dimension(cs: Constraint[], alt: DimAlt | null = null): void {
  // a number is written where it is read, so there has to be somewhere to write it: asking for
  // a dimension with the callouts turned off turns them back on rather than refusing
  if (!view.showDimensions) {
    view.showDimensions = true;
    toast('dimensions turned back on — a number is edited on the drawing');
  }
  const found = cs.map((c) => C.stating(view.sketch, c));
  const targets = cs.map((c, i) => found[i] ?? c);
  const fresh = cs.filter((_, i) => !found[i]);
  // an alternative is a choice about a dimension being written; one already on the drawing is
  // being edited, and moving it about must not silently make it a different constraint
  view.startDimension(targets, fresh, fresh.length === cs.length ? alt : null);
}

/** The one dimension button: what it puts a number on is the selection's business.  Two points
 *  or a single line take a length — or the run or the rise between them, whichever the number
 *  is put where — a point and a line a signed offset, a circle its radius and two of them the
 *  ring between them, and two lines take the gap between them when they are parallel, the angle
 *  at their corner when they are not. */
function cDimension(): void {
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
  const [a, b] = pts;
  dimension([pairDim('Distance', a, b)], { a, b, make: (k: string) => pairDim(k, a, b) });
}

/** Two parallel lines: dimension the gap between them.  It does not make them parallel — the
 *  caller has already established that they are, and sent the other case to Angle, because a
 *  "gap" between converging lines pins one endpoint's offset and reads as arbitrary. */
function cParallelDistance(l1: Line, l2: Line): void {
  const cur = signedPointToLine(l2.p1.x.value, l2.p1.y.value, l1);
  dimension([new C.ParallelDistance(l1, l2, cur)]);
}

/** A point and a line: dimension the point's perpendicular offset, signed so negating it moves
 *  the point across.  Measured to the infinite line — the foot may fall off the end of the
 *  segment, which is what a drawing means by "distance to this edge". */
function cPointLineDistance(p: Point, line: Line): void {
  const [dx, dy] = line.direction();
  if (!need(Math.hypot(dx, dy) > 0, 'a line with two distinct endpoints')) return;
  if (!need(p !== line.p1 && p !== line.p2, 'a point that is not an endpoint of the line')) return;
  dimension([new C.PointLineDistance(p, line, signedPointToLine(p.x.value, p.y.value, line))]);
}

/** Two circles or arcs: dimension the annulus between them.  Like the parallel gap it sizes
 *  the ring without centring it, so say so when the centres are not already together. */
function cAnnularDistance(c1: Circle | Arc, c2: Circle | Arc): void {
  dimension([new C.AnnularDistance(c1, c2, Math.abs(c2.radius.value) - Math.abs(c1.radius.value))]);
  const [a, b] = [c1.center, c2.center];
  if (Math.hypot(a.x.value - b.x.value, a.y.value - b.y.value) > 1e-9) {
    toast('the ring is dimensioned, but these circles are not concentric — add Coincident on their centres');
  }
}

/** Two lines that meet at a corner: dimension the corner. */
function cAngle(l1: Line, l2: Line): void {
  // the constructor takes radians, as the kernels do; only what a person writes is in degrees
  dimension([new C.Angle(l1, l2, angleBetween(l1, l2))]);
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
  const { lines, circles, splines } = sel();
  if (lines.length === 1 && splines.length === 1 && !circles.length) {
    // one constraint owning one parameter, not two: split in half it would be a point on the
    // curve and a direction somewhere else on it
    view.addConstraints(new C.SplineTangentLine(splines[0], lines[0]));
    return;
  }
  if (circles.length === 1 && splines.length === 1 && !lines.length) {
    // a circle against a curve says more than a line does: not just the direction there but how
    // hard it turns, so the circle becomes the curve's own radius — its osculating circle
    view.addConstraints(new C.SplineCurvature(splines[0], circles[0]));
    return;
  }
  if (splines.length) {
    need(false, 'a curve and either a line or a circle');
    return;
  }
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
    need(false, 'a line and a circle/arc/curve, a curve and a circle, or two circles/arcs');
  }
}

/** One circle or arc takes its radius; several are opened together and all take the number
 *  that is typed, whichever of their callouts it is typed on. */
function cRadius(circles: (Circle | Arc)[]): void {
  dimension(circles.map((cc) => new C.Radius(cc, Math.abs(cc.radius.value))));
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
  const exprs = expressions(sk);
  if (exprs.length) {
    lines.push('Expressions (in evaluation order):');
    for (const it of exprs) {
      const c = sk.constraintById(it.id);
      const where = c ? `${c.typeName}.${it.attr}` : `#${it.id}.${it.attr}`;
      const reads = it.deps.length ? `  ← ${it.deps.join(', ')}` : '';
      lines.push(it.error
        ? `   ✗ ${it.text}   [${where}]  ${it.error} — last value ${io.fmt(it.value, 6)} stands`
        : `   ${it.name ? `${it.name} = ` : ''}${io.fmt(it.value, 6)}   [${where}: ${it.text}]${reads}`);
    }
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
    li.addEventListener('dblclick', () => editValue(c));
    clist.append(li);
  });
  rows = next;
}

/** A dimension's number as a person reads and writes it: what it was written as if it was
 *  written, else what the drawing says it is — the same rounding, so the editor that sits on a
 *  callout shows the callout's own number rather than a longer one that means the same.  In
 *  degrees for an angle, which is the unit the core takes text in either way.
 *
 *  Nothing is written back unless somebody types, so the rounding here costs no precision: a
 *  dimension nobody edits keeps the measurement it was stated at, to the last figure. */
function dimensionField(c: Constraint): { attr: string; text: string } | null {
  const [d] = c.dimensions();
  if (!d) return null;
  const [attr, kind] = d;
  const v = (c as unknown as Record<string, number>)[attr];
  return { attr, text: c.expr(attr) ?? io.fmt(kind === 'angle' ? (v * 180) / Math.PI : v, 4) };
}

/** Open a dimension's number for editing, on the drawing where it is drawn. */
function editValue(c: Constraint): void {
  if (!dimensionField(c)) return toast(`${c.typeName} has no editable dimension`);
  dimension([c]);              // already stated: nothing to add and nothing to place
}

/** Write `text` on every constraint being edited; false if the core would not have it, which
 *  leaves them all as they were.  A text that reads a name nothing defines *is* taken — the
 *  row says so until the name appears — since that is a document half-written, not a mistake. */
function setDimension(cs: Constraint[], text: string): boolean {
  try {
    let why: string | null = null;
    for (const c of cs) {
      const f = dimensionField(c);
      if (f) why = c.setDimension(f.attr, text) ?? why;
    }
    if (why) toast(`stored, but ${why}`);
    return true;
  } catch (err) {
    toast(`not a dimension: ${(err as Error).message}`);
    return false;
  }
}

/* -- the number on the drawing -------------------------------------------------- */

/* A dimension's number is edited where it is drawn.  One input, moved to whichever callout is
 * being written and filled in by the view, so the figure, where it sits, which of the three it
 * is and what it says are one gesture — and every part of it is on the drawing while it
 * happens, rather than behind a box in the middle of the screen. */
let dimBox: HTMLInputElement | null = null;
/** Whether anybody has typed in it: until they have, it keeps showing what the dimension
 *  measures, which changes under it as the callout is moved from one kind to another. */
let dimTyped = false;

function sizeDimBox(box: HTMLInputElement): void {
  box.style.width = `${Math.max(4, box.value.length + 1)}ch`;
}

function openDimBox(): HTMLInputElement {
  const box = document.createElement('input');
  box.type = 'text';
  box.className = 'dim';
  box.spellcheck = false;
  box.title = 'a number, or an expression — Enter to accept, Esc to take it back';
  dimTyped = false;
  box.addEventListener('input', () => { dimTyped = true; sizeDimBox(box); });
  box.addEventListener('keydown', (e) => {
    e.stopPropagation();
    if (e.key === 'Enter') { e.preventDefault(); finishDim(true, true); }
    if (e.key === 'Escape') { e.preventDefault(); finishDim(false); }
  });
  // clicking away accepts it, the way a cell in a sheet does.  The click that *plants* the
  // callout is not a click away: the canvas refuses the focus for exactly that reason
  box.addEventListener('blur', () => finishDim(true));
  (document.getElementById('canvas-wrap') as HTMLElement).append(box);
  dimBox = box;
  box.focus();
  return box;
}

function closeDimBox(): void {
  const box = dimBox;
  dimBox = null;                    // first, so the blur that removing it may fire does nothing
  box?.remove();
}

/** Done typing.  Accepted, the text goes on every constraint the dimension was opened on;
 *  refused, the whole thing comes back out.  A text the core will not have keeps the editor
 *  open when there is somebody still typing in it to correct it. */
function finishDim(commit: boolean, keepOpen = false): void {
  const live = view.liveDim;
  if (!live || !dimBox) return;
  // an untouched number is not written: the dimension keeps the measurement it was stated at,
  // which is exact, rather than the rounded one the editor showed
  const text = dimTyped ? dimBox.value.trim() : '';
  if (commit && text && !setDimension(live.targets, text)) {
    if (keepOpen) return;
    commit = false;
  }
  closeDimBox();
  rows = [];                        // the row text states the number, so it has to rebuild
  view.endDimension(commit);
}

view.onDimension = (live, at) => {
  if (!live) return closeDimBox();
  const box = dimBox ?? openDimBox();
  if (!dimTyped) {
    box.value = dimensionField(live.targets[0])?.text ?? '';
    sizeDimBox(box);
    box.select();
  }
  if (at) {
    box.style.left = `${at[0]}px`;
    box.style.top = `${at[1]}px`;
  }
};

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
  // a curve reads as its control polygon: that is what it is made of and what edits it
  if (e instanceof Spline) return `${n}spline  ${e.ctrl.map((p) => ix.name(p)).join('–')}`;
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
  // an expression that could not be computed: its constraint holds the number it last had
  const exprError = new Map<number, string>();
  for (const it of expressions(sk)) if (it.error) exprError.set(it.id, it.error);
  if (exprError.size || rows.some((c) => Object.keys(c.exprs).length)) {
    // an expression's number moves when a name it reads does, without the list changing — so
    // the rows' text is re-read while any expression is about
    const ix = new io.Index(sk);
    rows.forEach((c, i) => { (clist.children[i] as HTMLElement).dataset.base = io.describe(c, ix); });
  }
  rows.forEach((c, i) => {
    const li = clist.children[i] as HTMLElement;
    const base = li.dataset.base ?? '';
    li.className = '';
    li.removeAttribute('title');
    if (exprError.has(c.id)) {
      li.textContent = `ƒ ${base}`;
      li.classList.add('expr-error');
      li.title = `expression: ${exprError.get(c.id)} — the last number stands`;
    }
    else if (culprits.has(c)) { li.textContent = `✗ ${base}`; li.classList.add('culprit'); }
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
  p: 'point', l: 'line', r: 'rect', c: 'circle', a: 'arc', 3: 'arc3', s: 'spline',
  w: 'splinefit',
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
  // the curve tools collect as many points as the user wants, so they need a way to say "that
  // is the curve" — the only tools whose click count is not known in advance
  if (k === 'enter') { e.preventDefault(); view.finishCurve(); return; }
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
  // shift is part of the token, so ⇧L is Perpendicular and never the Line tool.  The key is
  // taken: an action that opens a dialog has focused its text field by the time the browser
  // would insert the character, and would find a stray letter in it
  const action = ACTION_KEYS.get(e.shiftKey ? `⇧${k}` : k);
  if (action) { e.preventDefault(); action(); return; }
  if (!e.shiftKey && TOOL_KEYS[k]) view.setTool(TOOL_KEYS[k]);
});

/* -- boot ------------------------------------------------------------------------- */

view.onSelect = () => { if (currentConstraint) focusConstraint(null); };
/* A dimension on the drawing and its row in the list are the same constraint, so clicking
 * either does the same thing — and double-clicking either opens the same number. */
view.onPickConstraint = (c) => { focusConstraint(c); refresh(); view.draw(); };
view.onEditConstraint = (c) => editValue(c);
view.onChanged = refresh;
view.onDragFrame = refreshStatus;
view.onStatus = toast;
new ResizeObserver(() => view.resize()).observe(canvas);
view.resize();
view.fit();
view.afterEdit();
