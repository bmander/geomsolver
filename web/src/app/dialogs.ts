/* What the menus open: the sheets, the reports and the two root-selection commands.  Each is
 * one item's whole behaviour, so the menu tables in `main` stay a list of names. */
import * as C from '../core/constraints.js';
import * as examples from '../core/examples.js';
import * as remote from './remote.js';
import * as io from '../core/io.js';
import { applyAlternative, enumerateStep, isCurrent } from '../core/homotopy.js';
import { Point } from '../core/model.js';
import { Attitude } from '../core/program.js';
import { METHODS, Method } from '../core/system.js';
import { witnessSummary } from '../core/witness.js';
import { expressions } from '../core/expr.js';
import { aboutDag, currentConstraint, view } from './shell.js';
import {
  addCheckbox, addLink, addNumber, addSelect, addText, askChoice, askFields, openFile,
  showReport, showSheet, toast,
} from './ui.js';

/** What the wordmark on the bar opens.  The diagram lives in the page as a template, since a
 *  fixed drawing is markup rather than something to build an element at a time. */
export function about(): Promise<void> {
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
export function report(n: number, did: string): void {
  toast(n ? `${did} ${n} ${n === 1 ? 'entity' : 'entities'}`
          : did === 'pasted' ? 'nothing on the clipboard' : 'select something first');
}

/** A fresh sheet, which is not quite empty.  With nothing fixed on it the first shape drawn
 *  is free to float — it satisfies its constraints just as well anywhere on the canvas, so a
 *  drag slides the whole thing rather than working against anything.  One fixed point at the
 *  origin gives the drawing somewhere to be. */
/** The reference sketches, each with the one line that says what it is there to show.  Asked
 *  for the first time the menu item is picked, so booting costs nothing for a list most
 *  sessions never open. */
let cases: ReturnType<typeof examples.cases> | null = null;
export async function openCase(): Promise<void> {
  cases ??= examples.cases();
  const i = await askChoice('Open test case', 'The sketches the solver is exercised on:',
                            cases.map((c) => `${c.label} — ${c.description}`));
  if (i === null) return;
  view.load(await remote.source(cases[i].key));
  toast(`${cases[i].label} — ${cases[i].description}`, 12000);
}

/** Everything that changes how the solve runs, gathered behind one item.  The controls are
 *  built from the view each time the sheet opens, so none of them can go stale, and each
 *  takes effect as it is switched. */
export function options(): Promise<void> {
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
    addCheckbox(box, 'overview', view.overview, (v) => { view.setOverview(v); },
                'Fold the views back into the glass box they were unfolded from, with the object '
              + 'reconstructed between them.  Drag to orbit, wheel to zoom; the drawing is '
              + 'read-only in there — a click lights an edge up and nothing edits');
    addCheckbox(box, 'show solid', view.showSolid, (v) => { view.setShowSolid(v); },
                'In the overview, fill the object\u2019s surfaces as well as drawing its edges. '
              + 'Only a document with a `solid` in it has surfaces to show; it costs the boundary '
              + 'of every one, which is why it is a choice');
    addSelect(box, 'method', [...METHODS], view.method, (m) => {
      view.method = m as Method;
      view.solveNow();
    }, 'The trust-region method the solver runs');
    body.append(box);
  });
}

/* -- views ---------------------------------------------------------------------- */

/** The two choices in the `from` list that are not a view of the document's. */
const PAGE = 'page';
const EXPLICIT = 'explicit u, v';

/** `"0.6, 0.8, 0"` as the three texts a basis is written from — texts, not numbers: the form
 *  parses nothing, and the core says whether they span a plane. */
function triple(s: string): [string, string, string] | null {
  const parts = s.split(',').map((x) => x.trim());
  return parts.length === 3 && parts.every((x) => x) ? [parts[0], parts[1], parts[2]] : null;
}

/** Ask what the next plane is — its name, and its attitude as the statement will spell it —
 *  then arm the plane tool with it: where it sits on the page is the two clicks' business.
 *  **No geometry here**: a fold is handed on as the degrees typed, a basis as the texts, and
 *  the core validates both where it elaborates them. */
export async function insertPlane(): Promise<void> {
  const views = view.sketch.planes.map((p) => view.doc.nameOf(p))
    .filter((n): n is string => !!n);
  let name = '', from = PAGE, fold = '0', u = '1, 0, 0', v = '0, 0, 1';
  const ok = await askFields('Insert plane', (body) => {
    addText(body, 'name', name, (s) => { name = s; },
            'What the statement calls the view — blank to have one minted');
    addSelect(body, 'from', [PAGE, ...views, EXPLICIT], from, (s) => { from = s; },
              'The page itself; a view folded from one already drawn; or a basis written out');
    addNumber(body, 'fold', 0, (s) => { fold = s; },
              'Degrees, about the fold line with the view it is from: 0 is the top view of a '
            + 'front, -90 the right view, third-angle');
    addText(body, 'u', u, (s) => { u = s; }, 'The page\'s x axis in space, for an explicit basis');
    addText(body, 'v', v, (s) => { v = s; }, 'The page\'s y axis in space, for an explicit basis');
  });
  if (!ok) return;
  let attitude: Attitude | null;
  if (from === PAGE) {
    attitude = null;
  } else if (from === EXPLICIT) {
    const uu = triple(u), vv = triple(v);
    if (!uu || !vv) {
      toast('u and v are each three numbers, separated by commas');
      return;
    }
    attitude = { u: uu, v: vv };
  } else {
    attitude = { from, fold: `${fold.trim() || '0'}deg` };
  }
  view.insertPlane({ name: name.trim() || undefined, attitude });
  toast('click where the view\'s origin goes, then where it points — or Enter to point it right');
}

/* -- Stage 5: root selection ---------------------------------------------------- */

/** A selected tangency row toggles its inside/outside flag; a selected point flips the
 *  closed-form constructions that place it (the other circle-circle intersection), recorded
 *  in the sketch's branches and replayed sticky. */
export function flipBranch(): void {
  if (!view.mayEdit()) return;
  const c = currentConstraint;
  if (c && C.isType(c, 'TangentLineCircle')) {
    view.pushUndo();
    // the side is a word now (`left`, `right`, §9.2) — the core's vocabulary, read off the
    // registry rather than restated here, so a flip is "the other one it takes"
    const words = C.wordsFor(c, 'side');
    c.setValue('side', words.find((w) => w !== c.side) ?? words[0]);
    view.afterEdit();
    toast(`flipped the tangency side of ${io.describe(c, view.doc)}`);
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
export async function alternatives(): Promise<void> {
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
  if (pick === null || isCurrent(alts[pick]) || !view.mayEdit()) return;
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

export function reportSolve(): void {
  const r = view.lastResult;
  if (r) toast(`${r.success ? 'solved' : 'NOT CONVERGED'} — max|r| = ${r.maxResidual.toExponential(1)} in ${(r.timeS * 1e3).toFixed(1)} ms`);
}

export async function showDiagnosis(): Promise<void> {
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
    lines.push('Conflict — remove one of:', ...d.conflicts.map((c) => `   ✗ ${io.describe(c, view.doc)}`), '');
  }
  if (d.over.length) {
    lines.push(`Structurally redundant block (${d.nRedundant} equation(s) too many):`,
               ...d.over.map((c) => `   • ${io.describe(c, view.doc)}`), '');
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
      lines.push(`   ⟂ ${io.describe(dep.constraint, view.doc)} is implied by `
        + dep.impliedBy.slice(0, 6).map((c) => io.describe(c, view.doc)).join(', ')
        + (dep.theorem ? '  [theorem-type: invisible to structural analysis]' : ''));
    }
    const internal = rep.motions.filter((m) => !m.rigid);
    internal.slice(0, 8).forEach((m, i) => {
      lines.push(`   DOF ${i + 1}: ` + m.moving.slice(0, 10).map((p) => p.name).join(', '));
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
      // a free name is an unknown the solver moves, so what it is worth is where it stands now
      const free = it.free.length ? `  (${it.free.join(', ')} free)` : '';
      lines.push(it.error
        ? `   ✗ ${it.text}   [${where}]  ${it.error} — last value ${io.fmt(it.value, 6)} stands`
        : `   ${it.name ? `${it.name} = ` : ''}${io.fmt(it.value, 6)}   [${where}: ${it.text}]${reads}${free}`);
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

export async function doOpen(): Promise<void> {
  const text = await openFile();
  if (!text) return;
  try {
    view.setSketch(io.loads(text));
    toast('loaded');
  } catch (err) {
    toast(`could not load: ${(err as Error).message}`);
  }
}
