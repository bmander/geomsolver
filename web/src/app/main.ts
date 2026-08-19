/* The sketcher shell: toolbars, the constraint list, the diagnosis banner and the dialogs.
 *
 *   tools     S select · P point · L line · C circle · A arc · Esc cancel
 *   editing   F fix/unfix · Del delete · Ctrl+Z undo · wheel zoom · right-drag pan
 *
 * Everything below is presentation; the solver, diagnosis, decomposition and root selection
 * all live in core/ and are shared with the test suite. */
import * as C from '../core/constraints.js';
import * as examples from '../core/examples.js';
import * as io from '../core/io.js';
import { Constraint, ENTITY_KINDS } from '../core/constraints.js';
import { applyAlternative, enumerateStep, isCurrent } from '../core/homotopy.js';
import { Arc, Circle, Line, Point, Primitive, Sketch, expand } from '../core/model.js';
import { METHODS, Method } from '../core/system.js';
import { initCore } from '../core/wasm.js';
import { movingParams, witnessSummary } from '../core/witness.js';
import { SketchView, Tool } from './view.js';
import {
  addButton, addCheckbox, addSelect, addSeparator, askChoice, askNumber, download, openFile,
  showReport, stats, toast,
} from './ui.js';

const canvas = document.getElementById('canvas') as HTMLCanvasElement;
const barTools = document.getElementById('bar-tools') as HTMLElement;
const barConstraints = document.getElementById('bar-constraints') as HTMLElement;
const clist = document.getElementById('clist') as HTMLElement;
const banner = document.getElementById('banner') as HTMLElement;
const bannerText = document.getElementById('banner-text') as HTMLElement;
const bannerSelect = document.getElementById('banner-select') as HTMLButtonElement;

await initCore();
(document.getElementById('loading') as HTMLElement).remove();

const view = new SketchView(canvas, examples.rectFillets());
let currentConstraint: Constraint | null = null;
let rows: Constraint[] = [];
const toolButtons = new Map<Tool, HTMLButtonElement>();

/* -- toolbars ---------------------------------------------------------------- */

function setTool(t: Tool): void {
  view.setTool(t);
  for (const [k, b] of toolButtons) b.setAttribute('aria-pressed', String(k === t));
}

for (const [label, tool, key] of [
  ['Select', 'select', 's'], ['Point', 'point', 'p'], ['Line', 'line', 'l'],
  ['Circle', 'circle', 'c'], ['Arc', 'arc', 'a'],
] as [string, Tool, string][]) {
  toolButtons.set(tool, addButton(barTools, { label, key, toggle: true, onClick: () => setTool(tool) }));
}
setTool('select');
addButton(barTools, { label: 'Cancel', key: 'esc', onClick: () => view.cancelTool() });
addSeparator(barTools);

const caseBox = addSelect(barTools,
  [{ value: '', label: '— load a test case —' },
   ...examples.CASES.map(([name, , desc]) => ({ value: name, label: name, title: desc }))],
  (name) => {
    const hit = examples.CASES.find(([n]) => n === name);
    if (!hit) return;
    view.setSketch(hit[1]());
    toast(`${hit[0]} — ${hit[2]}`, 12000);
    caseBox.value = '';
  }, '230px');

addSeparator(barTools);
addButton(barTools, { label: 'New', onClick: () => view.setSketch(new Sketch()) });
addButton(barTools, { label: 'Open', onClick: () => void doOpen() });
addButton(barTools, { label: 'Save', onClick: () => download('sketch.json', io.dumps(view.sketch)) });
addButton(barTools, { label: 'Undo', key: '⌘Z', onClick: () => view.undo() });
addButton(barTools, { label: 'Fit', onClick: () => view.fit() });
addSeparator(barTools);
addButton(barTools, { label: 'Solve', onClick: () => { view.solveNow(); reportSolve(); } });
addButton(barTools, { label: 'Diagnose', onClick: () => void showDiagnosis() });
addButton(barTools, { label: 'Animate DOF', onClick: () => {
  if (!view.startAnimation()) toast('no remaining internal DOF to animate');
} });
addButton(barTools, { label: 'Flip branch', onClick: () => flipBranch() });
addButton(barTools, { label: 'Alternatives…', onClick: () => void alternatives() });
addSeparator(barTools);
addCheckbox(barTools, 'auto-solve', true, (v) => { view.autoSolve = v; });
addCheckbox(barTools, 'plan', false, (v) => { view.usePlan = v; view.solveNow(); });
addCheckbox(barTools, 'colour by state', true, (v) => { view.colorByState = v; view.draw(); });
addSelect(barTools, METHODS.map((m) => ({ value: m, label: m })), (m) => {
  view.method = m as Method;
  view.solveNow();
});

/* constraints whose arguments are just entities: (label, class, points, lines, circles/arcs) */
const SIMPLE: [string, C.ConstraintCtor, number, number, number][] = [
  ['Coincident', C.Coincident, 2, 0, 0],
  ['Horizontal', C.Horizontal, 0, 1, 0],
  ['Vertical', C.Vertical, 0, 1, 0],
  ['Parallel', C.Parallel, 0, 2, 0],
  ['Perpendicular', C.Perpendicular, 0, 2, 0],
  ['On line', C.PointOnLine, 1, 1, 0],
  ['Midpoint', C.Midpoint, 1, 1, 0],
  ['On circle', C.PointOnCircle, 1, 0, 1],
];
addButton(barConstraints, { label: 'Coincident', onClick: () => applySimple(SIMPLE[0]) });
addButton(barConstraints, { label: 'Distance', onClick: () => void cDistance() });
for (const s of SIMPLE.slice(1)) addButton(barConstraints, { label: s[0], onClick: () => applySimple(s) });
addButton(barConstraints, { label: 'Angle', onClick: () => void cAngle() });
addButton(barConstraints, { label: 'Equal', onClick: () => cEqual() });
addButton(barConstraints, { label: 'Tangent', onClick: () => cTangent() });
addButton(barConstraints, { label: 'Radius', onClick: () => void cRadius() });
addButton(barConstraints, { label: 'Fix', key: 'f', onClick: () => view.toggleFixSelected() });

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
function applySimple([label, cls, nPts, nLines, nCirc]: typeof SIMPLE[number]): void {
  const { pts, lines, circles } = sel();
  const perLine = nPts === 0 && nLines === 1 && nCirc === 0;
  const ok = pts.length === nPts && circles.length === nCirc
    && (perLine ? lines.length >= 1 : lines.length === nLines);
  const what = ([[nPts, 'point(s)'], [nLines, 'line(s)'], [nCirc, 'circle(s)/arc(s)']] as const)
    .filter(([n]) => n).map(([n, w]) => `${n} ${w}`).join(', ');
  if (!need(ok, what)) return;
  for (const ln of perLine ? lines : [null]) {
    const args: unknown[] = [];
    let pi = 0, li = 0, ci = 0;
    for (const [, kind] of cls.spec) {
      if (kind === 'point') args.push(pts[pi++]);
      else if (kind === 'line') args.push(perLine ? ln : lines[li++]);
      else if (ENTITY_KINDS.has(kind)) args.push(circles[ci++]);
    }
    view.addConstraint(new (cls as unknown as new (...a: unknown[]) => Constraint)(...args));
  }
  void label;
}

async function cDistance(): Promise<void> {
  let { pts } = sel();
  const { lines } = sel();
  if (pts.length === 0 && lines.length === 1) pts = [lines[0].p1, lines[0].p2];
  if (!need(pts.length === 2, 'two points (or one line)')) return;
  const cur = Math.hypot(pts[0].x.value - pts[1].x.value, pts[0].y.value - pts[1].y.value);
  const v = await askNumber('Distance', 'Distance', cur);
  if (v !== null) view.addConstraint(new C.Distance(pts[0], pts[1], v));
}

async function cAngle(): Promise<void> {
  const { lines } = sel();
  if (!need(lines.length === 2, 'two lines')) return;
  const [d1, d2] = [lines[0].direction(), lines[1].direction()];
  const cur = (Math.atan2(d1[0] * d2[1] - d1[1] * d2[0], d1[0] * d2[0] + d1[1] * d2[1]) * 180) / Math.PI;
  const v = await askNumber('Angle', 'Angle from the first to the second line (degrees)', cur);
  if (v !== null) view.addConstraint(new C.Angle(lines[0], lines[1], (v * Math.PI) / 180));
}

function cEqual(): void {
  const { lines, circles } = sel();
  if (lines.length === 2) view.addConstraint(new C.EqualLength(lines[0], lines[1]));
  else if (circles.length === 2) view.addConstraint(new C.EqualRadius(circles[0], circles[1]));
  else need(false, 'two lines or two circles/arcs');
}

function cTangent(): void {
  const { lines, circles } = sel();
  if (lines.length === 1 && circles.length === 1) {
    const ln = lines[0], cc = circles[0];
    if (cc instanceof Arc) {
      const ends = new Set([ln.p1, ln.p2]);
      if (ends.has(cc.start)) return view.addConstraint(new C.TangentArcLine(cc, ln, 'start'));
      if (ends.has(cc.end)) return view.addConstraint(new C.TangentArcLine(cc, ln, 'end'));
    }
    view.addConstraint(new C.TangentLineCircle(ln, cc));
  } else if (circles.length === 2 && !lines.length) {
    const [a, b] = circles;
    const d = Math.hypot(a.center.x.value - b.center.x.value, a.center.y.value - b.center.y.value);
    view.addConstraint(new C.TangentCircleCircle(a, b, d > Math.max(Math.abs(a.radius.value), Math.abs(b.radius.value))));
  } else {
    need(false, 'a line and a circle/arc, or two circles/arcs');
  }
}

async function cRadius(): Promise<void> {
  const { circles } = sel();
  if (!need(circles.length >= 1, 'circle(s)/arc(s)')) return;
  const v = await askNumber('Radius', 'Radius', Math.abs(circles[0].radius.value));
  if (v === null) return;
  for (const cc of circles) view.addConstraint(new C.Radius(cc, v));
}

/* -- Stage 5: root selection ---------------------------------------------------- */

/** A selected tangency row toggles its inside/outside flag; a selected point flips the
 *  closed-form constructions that place it (the other circle-circle intersection), recorded
 *  in the sketch's branches and replayed sticky. */
function flipBranch(): void {
  const c = currentConstraint;
  if (c instanceof C.TangentLineCircle) {
    view.pushUndo();
    c.side = -c.side;
    view.afterEdit();
    toast(`flipped the tangency side of ${io.describe(c, view.sketch)}`);
    return;
  }
  if (c instanceof C.TangentCircleCircle) {
    view.pushUndo();
    c.external = !c.external;
    view.afterEdit();
    toast(`flipped to ${c.external ? 'external' : 'internal'} tangency`);
    return;
  }
  const pts = view.selected.filter((e): e is Point => e instanceof Point);
  if (!pts.length) {
    toast('select a point (or a tangency row) to flip its solution branch');
    return;
  }
  const ps = view.plan();
  view.pushUndo();
  const n = pts.reduce((s, p) => s + ps.flip(ps.graph.P(p)), 0);
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
  const el = ps.graph.P(pts[0]);
  const placing = ps.plan.stepsPlacing(el);
  if (!placing.length) {
    toast('no construction places that point (under-constrained or not decomposable)');
    return;
  }
  const idx = placing[0][0];
  const alts = enumerateStep(ps.plan, idx, { locate: el });
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
  applyAlternative(ps.plan, idx, alts[pick]);
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
    lines.push('', `Decomposition: ${view.lastPlan.plan.summary()}`
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
      currentConstraint = c;
      view.highlight = expand(c.entities());
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

function sameRows(a: Constraint[], b: Constraint[]): boolean {
  return a.length === b.length && a.every((c, i) => c === b[i]);
}

function refresh(): void {
  const sk = view.sketch;
  const next = sk.userConstraints();
  if (!sameRows(next, rows)) rebuildRows(next);
  if (currentConstraint && !rows.includes(currentConstraint)) currentConstraint = null;

  const d = view.diagnosis;
  const selDirect = new Set(view.selected);
  const selAll = new Set(expand(view.selected));
  const culprits = new Set(d?.conflicts ?? []);
  const bad = new Set([...(d?.conflicts ?? []), ...(d?.violated ?? [])]);
  const over = new Set(d?.over ?? []);
  rows.forEach((c, i) => {
    const li = clist.children[i] as HTMLElement;
    const base = li.dataset.base ?? '';
    li.className = '';
    if (culprits.has(c)) { li.textContent = `✗ ${base}`; li.classList.add('culprit'); }
    else if (bad.has(c)) { li.textContent = base; li.classList.add('violated'); }
    else if (over.has(c)) { li.textContent = `≈ ${base}`; li.classList.add('over'); }
    else li.textContent = base;
    const hit = selAll.size > 0 && (c.entities().some((e) => selAll.has(e))
      || expand(c.entities()).some((e) => selDirect.has(e)));
    li.classList.toggle('touches', hit);
    li.setAttribute('aria-current', String(c === currentConstraint));
  });

  const r = view.lastResult;
  let msg = `points ${sk.points.length}  lines ${sk.lines.length}  circles ${sk.circles.length}  arcs ${sk.arcs.length}`
    + `   | params ${sk.params.length} (free ${sk.freeIndices().length})  equations ${sk.nResiduals()}`;
  if (d) {
    msg += `  DOF ${d.dof}`;
    if (d.nRedundant) msg += `  redundant ${d.nRedundant}`;
    if (d.status === 'conflict') msg += '  ⚠ CONFLICT';
    else if (d.numericRank !== null && d.numericRank < d.structuralRank) msg += '  ⚠ geometric dependency';
    else if (d.numericRank === null && d.warnings.length) msg += '  (structural only)';
  }
  msg += `   | selected ${view.selected.length}`;
  if (r) {
    msg += `   | ${r.success ? 'solved' : 'NOT CONVERGED'}  max|r|=${r.maxResidual.toExponential(1)}  `
      + `${(r.timeS * 1e3).toFixed(1)} ms  nfev=${r.nfev}  ${r.method}`;
  }
  if (view.lastPlan) {
    msg += `   | plan: ${view.lastPlan.plan.summary()}${view.lastPlan.fellBack ? ' (fell back)' : ''}`;
  }
  stats(msg);
  refreshBanner();
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
  view.selected = [...new Set(culprits.flatMap((c) => c.entities()))] as Primitive[];
  currentConstraint = culprits[0] ?? null;
  view.highlight = currentConstraint ? expand(currentConstraint.entities()) : [];
  refresh();
  view.draw();
});

/* -- keyboard ------------------------------------------------------------------- */

const TOOL_KEYS: Record<string, Tool> = { s: 'select', p: 'point', l: 'line', c: 'circle', a: 'arc' };

window.addEventListener('keydown', (e) => {
  const t = e.target as HTMLElement | null;
  if (t && (t.tagName === 'INPUT' || t.tagName === 'SELECT' || t.tagName === 'TEXTAREA')) return;
  if ((e.metaKey || e.ctrlKey) && e.key.toLowerCase() === 'z') { e.preventDefault(); view.undo(); return; }
  if (e.metaKey || e.ctrlKey || e.altKey) return;
  const k = e.key.toLowerCase();
  if (k === 'escape') { view.cancelTool(); return; }
  if (k === 'delete' || k === 'backspace') {
    e.preventDefault();
    if (currentConstraint && !view.selected.length) {
      const c = currentConstraint;
      currentConstraint = null;
      view.highlight = [];
      view.removeConstraint(c);
      toast(`removed ${c.typeName}`);
    } else {
      view.deleteSelected();
    }
    return;
  }
  if (k === 'f') { view.toggleFixSelected(); return; }
  if (TOOL_KEYS[k]) setTool(TOOL_KEYS[k]);
});

/* -- boot ------------------------------------------------------------------------- */

view.onChanged = refresh;
view.onStatus = toast;
new ResizeObserver(() => view.resize()).observe(canvas);
view.resize();
view.fit();
view.afterEdit();
toast('Rectangle with fillets — drag a point, or load another case from the toolbar', 12000);
