/* What the menus open: the sheets, the reports and the two root-selection commands.  Each is
 * one item's whole behaviour, so the menu tables in `main` stay a list of names. */
import * as C from '../core/constraints.js';
import { exampleCases } from './example-catalog.js';
import * as remote from './remote.js';
import { githubProject, parseGithub } from './github.js';
import { loaded, loading, painted } from './loading.js';
import { held, openProject, openSide } from './program.js';
import * as io from '../core/io.js';
import { applyAlternative, enumerateStep, isCurrent } from '../core/homotopy.js';
import { Point } from '../core/model.js';
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
let cases: ReturnType<typeof exampleCases> | null = null;
export async function openCase(): Promise<void> {
  cases ??= exampleCases();
  const i = await askChoice('Open examples', 'Choose a file or directory to explore:',
    cases.map((c) => ({ title: c.label,
      description: `${c.target.path}${c.target.kind === 'directory' ? '/' : ''} — ${c.description}` })), true);
  if (i == null) return;
  await openExample(cases[i].key);
}

/** The latest opening, of an example or a GitHub project: one fetched after another was asked
 *  for is dropped, and the newer one owns the loading modal. */
let openingRequest = 0;
export async function openExample(key: string, navigation: 'push' | 'replace' | 'none' = 'push',
  file?: string): Promise<void> {
  const request = ++openingRequest;
  try {
    loading(`fetching ${key}…`);
    const bundle = await remote.drawing(key, file);
    if (request !== openingRequest) return;
    // the reading holds the thread, so the modal saying so is painted first
    loading(`reading ${bundle.source}…`);
    await painted();
    if (request !== openingRequest) return;
    const updateUrl = (selected: string | undefined, method: 'push' | 'replace') => {
      const url = new URL(location.href);
      url.searchParams.set('example', bundle.key);
      if (selected) url.searchParams.set('file', selected);
      else url.searchParams.delete('file');
      if (url.href !== location.href) history[method === 'push' ? 'pushState' : 'replaceState'](null, '', url);
    };
    await held(() => openProject(bundle, { directory: bundle.directory, entry: bundle.entry,
      onSelect: (path) => updateUrl(path, 'replace') }));
    if (navigation !== 'none') {
      updateUrl(file || bundle.key !== key ? bundle.source : undefined, navigation);
    }
  }
  catch (e) { loaded(); toast(`Could not open example: ${(e as Error).message}`); }
}

/** A GitHub repository, or a folder or a file in one, opened as the project: its files in the
 *  panel's tree, the address asked for in a form since the browser has no picker for it, and a
 *  token beside it for a private one, held in this page's memory and nowhere else. */
let githubAddress = '', githubToken = '';
export async function openGithub(): Promise<void> {
  if (!await askFields('Open GitHub project', (body) => {
    addText(body, 'address', githubAddress, (v) => { githubAddress = v; },
      'A repository (https://github.com/owner/repo, or owner/repo), a folder in it '
    + '(…/tree/branch/folder) or a file (…/blob/branch/file.svd)');
    const token = addText(body, 'token', githubToken, (v) => { githubToken = v.trim(); },
      'For a private repository: a GitHub token that can read its contents.  It is sent '
    + 'only to api.github.com, and kept only until the page is closed');
    token.type = 'password'; token.autocomplete = 'off';
    token.placeholder = 'optional, for a private repository';
  })) return;
  const address = parseGithub(githubAddress);
  if (!address) { toast('That is not a GitHub repository address'); return; }
  const request = ++openingRequest;
  try {
    loading(`fetching ${address.owner}/${address.repo}…`);
    const bundle = await githubProject(address, githubToken);
    if (request !== openingRequest) return;
    loading(`reading ${bundle.label}…`);
    await painted();
    if (!await held(() => openProject(bundle, { directory: bundle.directory }))) return;
    // what was asked for is the repository's files, so the panel turns to them
    clearExampleAddress();
    openSide('project');
    toast(`opened ${bundle.label}`);
  } catch (e) {
    if (request !== openingRequest) return;
    loaded();
    toast(`Could not open GitHub project: ${(e as Error).message}`);
  }
}

/** The address names an example only while that example is the document: another one (File ▸
 *  New, Open, a drawing folder, a GitHub project) takes `example` and `file` off it, in place —
 *  a step back to the bare address would open a blank sheet — and an example still on its way
 *  is dropped. */
export function leaveExample(): void {
  ++openingRequest;
  loaded();
  clearExampleAddress();
}
/** The address's example taken off, leaving the modal as it is: a GitHub project's own meshing
 *  may be what it is showing. */
function clearExampleAddress(): void {
  const url = new URL(location.href);
  url.searchParams.delete('example');
  url.searchParams.delete('file');
  if (url.href !== location.href) history.replaceState(null, '', url);
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
                (v) => { view.showDimensions = v; },
                'Call out every dimensioned constraint on the drawing — click one to select it, '
              + 'drag it where you want it, double-click to change its number.  Off, editing a '
              + 'dimension still shows that one.  A .svd asks for its own annotations on paper');
    addCheckbox(box, 'show solid', view.showSolid, (v) => { view.setShowSolid(v); },
                'Fill each object\u2019s surfaces as well as drawing its edges — off, the solid is '
              + 'a wireframe, which is how you see its far side');
    addCheckbox(box, 'tangent edges', view.showTangentEdges, (v) => { view.setShowTangentEdges(v); },
                'Ink, lighter than a sharp edge, where one face of an object runs smoothly into '
              + 'the next — a round\u2019s run-out into the flat it rounds');
    if (view.builds.remembers) {
      const forget = document.createElement('button');
      forget.textContent = 'Forget built objects';
      forget.title = 'Every object built here is kept in this browser, so opening a document again '
        + 'draws it at once: forget them all, and the next opening builds each again';
      forget.addEventListener('click', () => {
        void view.builds.forget().then(() => toast('forgot every built object'), (e) => toast(String(e)));
      });
      box.append(forget);
    }
    addSelect(box, 'method', [...METHODS], view.method, (m) => {
      view.method = m as Method;
      view.solveNow();
    }, 'The trust-region method the solver runs');
    body.append(box);
  });
}

/* -- views ---------------------------------------------------------------------- */

/** Ask what the next plane is called, then drop it as the Plane button does — constrained to the
 *  selection by the core's table.  **No geometry here**: the core says what the selection makes. */
export async function insertPlane(): Promise<void> {
  let name = '';
  const ok = await askFields('Insert plane', (body) => {
    addText(body, 'name', name, (s) => { name = s; },
            'What the statement calls the plane — blank to have one minted');
  });
  if (!ok) return;
  view.addDatum('plane', name.trim() || undefined);
}

/** Ask what the component is called, then package the selection as the Component button does.
 *  Whether the name will do is the core's to say: a taken one is refused in its words. */
export async function insertComponent(): Promise<void> {
  if (!view.selected.length) { toast('select what the component is to be made of'); return; }
  let name = '';
  const ok = await askFields('Make component', (body) => {
    addText(body, 'name', name, (s) => { name = s; },
            'What the definition is called — blank to have one minted (Part, Part2, …)');
  });
  if (!ok) return;
  view.makeComponent(name.trim() || undefined);
}

/* -- Stage 5: root selection ---------------------------------------------------- */

/** A selected tangency row toggles its inside/outside flag; a selected point flips the
 *  closed-form constructions that place it (the other circle-circle intersection), recorded
 *  in the sketch's branches and replayed sticky. */
export function flipBranch(): void {
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

export function reportSolve(): void {
  const r = view.lastResult;
  // a stop on the iteration limit is said in the core's words (#449)
  if (r) toast(`${r.settled ? 'solved' : r.success ? r.message : 'NOT CONVERGED'} — max|r| = ${r.maxResidual.toExponential(1)} in ${(r.timeS * 1e3).toFixed(1)} ms`);
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
    lines.push('Expressions:');
    for (const it of exprs) {
      const c = sk.constraintById(it.id);
      const where = c ? `${c.typeName}.${it.attr}` : `#${it.id}.${it.attr}`;
      const reads = it.deps.length ? `  ← ${it.deps.join(', ')}` : '';
      // a free name is an unknown the solver moves, so what it is worth is where it stands now
      const free = it.free.length ? `  (${it.free.join(', ')} free)` : '';
      lines.push(it.error
        ? `   ✗ ${it.text}   [${where}]  ${it.error} — last value ${io.fmt(it.value, 6)} stands`
        : `   ${io.fmt(it.value, 6)}   [${where}: ${it.text}]${reads}${free}`);
    }
    lines.push('');
  }
  // each load scenario's words and its requirements' answers, as `solventc` says them
  for (const s of view.scenarios) lines.push(...s.words, ...s.answers.map((a) => a.text), '');
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
