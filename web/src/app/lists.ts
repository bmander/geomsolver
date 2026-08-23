/* The sidebar, the banner and the status line: everything the shell says about the sketch
 * rather than draws of it.  The two lists mirror each other — picking a row in either marks
 * what it touches in the other — and they are rebuilt only when their contents change, so rows
 * keep their DOM nodes, and the scroll position, across ordinary edits and drags. */
import * as io from '../core/io.js';
import { Constraint } from '../core/constraints.js';
import {
  Arc, Circle, Line, Point, Primitive, Spline, angleBetween, distanceBetween, expand,
} from '../core/model.js';
import { expressions } from '../core/expr.js';
import {
  banner, bannerSelect, bannerText, clearFocus, clist, currentConstraint, elist, focusConstraint,
  footerEl, measureEl, view,
} from './shell.js';
import { stats } from './ui.js';

/** The constraints and the components the two lists were last built from, and which entities
 *  were selected when the component list last scrolled itself: a list is rebuilt only when
 *  what it holds actually changed. */
let rows: Constraint[] = [];
let erows: Primitive[] = [];
let lastSelKey = '';

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
export function refreshRows(): void {
  const sk = view.sketch;
  const next = sk.userConstraints();
  if (!sameList(next, rows)) rebuildRows(next);
  if (currentConstraint && !rows.includes(currentConstraint)) clearFocus();

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
  focusConstraint(culprits[0] ?? null, expand(new Set(culprits.flatMap((c) => c.entities()))));
  refresh();
  view.draw();
});

/** The rows state their constraints' numbers, so a number that changed under them means the
 *  list has to be rebuilt even though it holds the same constraints in the same order. */
export function invalidateRows(): void {
  rows = [];
}
