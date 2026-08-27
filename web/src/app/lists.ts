/* The constraints window, the banner and the status line: everything the shell says about the
 * sketch rather than draws of it.
 *
 * The window belongs to what is picked.  It lists what holds the selected component and
 * nothing else, and the status line says in words which component that is — the two are the
 * one answer to "what have I got hold of", one as a list and one as a line.  The list is
 * rebuilt only when its contents change, so rows keep their DOM nodes, and the scroll
 * position, across ordinary edits and drags. */
import * as io from '../core/io.js';
import { Constraint } from '../core/constraints.js';
import {
  Arc, Circle, Ellipse, Line, Point, Primitive, Spline, angleBetween, distanceBetween, expand,
} from '../core/model.js';
import { expressions } from '../core/expr.js';
import {
  banner, bannerSelect, bannerText, clearFocus, clist, componentEl, cpanel, cpanelTitle,
  currentConstraint, focusConstraint, footerEl, measureEl, view,
} from './shell.js';
import { dragWindow, stats } from './ui.js';

/** The constraints the window was last built from, and the components it is open on: the list
 *  is rebuilt only when what it holds actually changed, or when `invalidateRows` says a number
 *  in it moved under the same constraints. */
let rows: Constraint[] = [];
let subject: Primitive[] = [];
let stale = false;

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
    // a dimension's row and its callout on the drawing are the same constraint, so both
    // open its number through the one hook the shell wires for it
    li.addEventListener('dblclick', () => view.onEditConstraint(c));
    clist.append(li);
  });
  rows = next;
}
/** Same objects in the same order — the lists are rebuilt only when this fails, so rows keep
 *  their DOM nodes (and the user's scroll position) across ordinary edits and drags. */
function sameList<T>(a: readonly T[], b: readonly T[]): boolean {
  return a.length === b.length && a.every((c, i) => c === b[i]);
}

/** A component in words: its short name, its type, and the points that define it, so what the
 *  status line says is the sketch's structure rather than coordinates that churn on every
 *  drag.  Live values belong in the measurement readout, not here. */
function describeEntity(e: Primitive, ix: io.Index): string {
  const n = ix.name(e).padEnd(4);
  const tag = e instanceof Point ? (e.isFixed ? '  ·fixed' : '')
            : e.construction ? '  ·constr' : '';
  const body = e instanceof Line ? `line    ${ix.name(e.p1)}–${ix.name(e.p2)}`
    : e instanceof Arc ? `arc     @${ix.name(e.center)} ${ix.name(e.start)}–${ix.name(e.end)}`
    : e instanceof Circle ? `circle  @${ix.name(e.center)}`
    // a curve reads as its control polygon: that is what it is made of and what edits it
    : e instanceof Spline ? `spline  ${e.ctrl.map((p) => ix.name(p)).join('–')}`
    : e instanceof Ellipse ? `ellipse @${ix.name(e.center)} →${ix.name(e.major)}`
    : 'point';
  return `${n}${body}${tag}`;
}

/** What the window is open on: the selection while there is one, and otherwise whatever it was
 *  last opened on, minus anything that has since left the document.  Staying put is the whole
 *  point of the second clause — focusing a constraint empties the selection, so a row of the
 *  window would otherwise pull the window out from under the pointer that clicked it.
 *
 *  Nothing here infers where a focus came from; `openPanel` and `closePanel` are how the two
 *  other things that pick — a callout on the drawing, the banner's button, a press that hits
 *  nothing — say what they picked. */
function panelSubject(live: Set<Primitive>): Primitive[] {
  if (view.selected.length) return view.selected;
  return subject.filter((e) => live.has(e));
}

/** Every constraint that reaches the window's subject: one stated *on* it, and one stated on
 *  a point that defines it.  The same test the constraint rows were marked with when they were
 *  a list of the whole sketch. */
function holding(cs: readonly Constraint[], on: readonly Primitive[]): Constraint[] {
  if (!on.length) return [];
  const direct = new Set(on);
  const all = new Set(expand(on));
  return cs.filter((c) => c.entities().some((e) => all.has(e))
    || expand(c.entities()).some((e) => direct.has(e)));
}

/** Open the window on some geometry nothing has *selected* — the constraint behind a callout
 *  clicked on the drawing, the culprits the banner names.  What it holds is what there is to
 *  look at then, and there is no selection to say it. */
export function openPanel(on: readonly Primitive[]): void {
  subject = [...new Set(on)];
}

/** A press that picked nothing: the window has nothing to be open on, so it shuts. */
export function closePanel(): void {
  subject = [];
}

/** The window, the status line's component and the banner: everything that only changes when
 *  the sketch or the selection does. */
export function refreshRows(): void {
  const sk = view.sketch;
  const all = sk.userConstraints();
  // the focused constraint has left the document — a row of the window is not where that is
  // noticed any more, since the window holds only the ones reaching what is picked
  if (currentConstraint && !all.includes(currentConstraint)) clearFocus();
  subject = panelSubject(new Set(sk.primitives()));
  const next = holding(all, subject);
  if (stale || !sameList(next, rows)) { stale = false; rebuildRows(next); }

  const d = view.diagnosis;
  const culprits = new Set(d?.conflicts ?? []);
  const bad = new Set(d?.violated ?? []);          // culprits are handled first, below
  const over = new Set(d?.over ?? []);
  const implied = new Set(d?.implied ?? []);   // a theorem made it follow: a note, not a fault
  // a claim (§9.7) is judged, never solved for, so its verdict is its own and none of the sets
  // above ever holds one — which is why these are read separately rather than folded in
  const proved = new Set(d?.claimsTheorem ?? []);
  const refuted = new Set(d?.claimsViolated ?? []);
  const independent = new Set(d?.claimsConsuming ?? []);
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
    // a claim's verdict comes before the ordinary readings because none of them can apply to
    // one: it joins no conflict, no over-block and no implied set, so what is left to say about
    // it is exactly which of the three it landed in
    else if (proved.has(c)) {
      li.textContent = `⊢ ${base}`;
      li.classList.add('claim-proved');
      li.title = 'proved — the rest of the document entails it, and nothing needs fixing';
    }
    else if (refuted.has(c)) {
      li.textContent = `⊬ ${base}`;
      li.classList.add('claim-refuted');
      li.title = 'refuted — this drawing is a counterexample; the drawing is fine, the claim is wrong';
    }
    else if (independent.has(c)) {
      li.textContent = `? ${base}`;
      li.classList.add('claim-independent');
      li.title = 'independent — true here, but the document does not say so: stating it would cost a freedom';
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
    li.setAttribute('aria-current', String(c === currentConstraint));
  });
  refreshPanel();
  refreshBanner();
}

/** The window's heading and whether it is up at all, and the same answer in the status line:
 *  what is picked, said the way the old sidebar's row said it. */
function refreshPanel(): void {
  cpanel.hidden = subject.length === 0;
  if (!subject.length) {
    componentEl.textContent = '';
    return;
  }
  const ix = new io.Index(view.sketch);
  const names = subject.map((e) => ix.name(e)).join(' ');
  const n = rows.length;
  cpanelTitle.textContent = `${names} — ${n || 'no'} constraint${n === 1 ? '' : 's'}`;
  // one thing picked reads as its sidebar row did; several read as a list of names, because a
  // status line is one line and the descriptions do not fit across it
  componentEl.textContent = `${subject.length === 1 ? describeEntity(subject[0], ix)
    : `${subject.length} components: ${names}`}   |   `;
}
// where the window sits is the user's, not the layout's
dragWindow(cpanel, cpanelTitle);

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
export function refreshStatus(): void {
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

export function refresh(): void {
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
  const lit = expand(new Set(culprits.flatMap((c) => c.entities())));
  focusConstraint(culprits[0] ?? null, lit);
  openPanel(lit);
  refresh();
  view.draw();
});

/** The rows state their constraints' numbers, so a number that changed under them means the
 *  list has to be rebuilt even though it holds the same constraints in the same order.  A flag
 *  rather than an emptied list: `rows` is also how the window knows a focused constraint came
 *  from one of its own rows. */
export function invalidateRows(): void {
  stale = true;
}
