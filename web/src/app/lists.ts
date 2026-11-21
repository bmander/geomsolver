/* The banner and the status line: everything the shell says about the sketch rather than draws
 * of it.  The status line says in words what is picked — the answer to "what have I got hold
 * of". */
import * as io from '../core/io.js';
import {
  Arc, Axis, Circle, Curve, Line, Plane, Point, Primitive, Spline, Traced, angleBetween,
  distanceBetween, expand,
} from '../core/model.js';
import { unbuilt } from '../core/mesh.js';
import {
  banner, bannerSelect, bannerText, clearFocus, componentEl, currentConstraint, focusConstraint,
  footerEl, measureEl, view,
} from './shell.js';
import { stats } from './ui.js';
import { pickName } from './view.js';
import type { SolidPick } from './view.js';

/** A component in words: its short name, its type, and the points that define it, so what the
 *  status line says is the sketch's structure rather than coordinates that churn on every
 *  drag.  Live values belong in the measurement readout, not here. */
function describeEntity(e: Primitive, ix: io.Index): string {
  const n = ix.name(e).padEnd(4);
  // a point says which view it is drawn in, by the name the source calls the view
  const tag = e instanceof Point ? `${e.isFixed ? '  ·fixed' : ''}${inView(e)}`
            : e.hasClass('construction') ? '  ·constr' : '';
  const body = e instanceof Line ? `line    ${ix.name(e.p1)}–${ix.name(e.p2)}`
    : e instanceof Arc ? `arc     @${ix.name(e.center)} ${ix.name(e.start)}–${ix.name(e.end)}`
    : e instanceof Circle ? `circle  @${ix.name(e.center)}`
    // a curve reads as its control polygon: that is what it is made of and what edits it
    : e instanceof Spline ? `spline  ${e.ctrl.map((p) => ix.name(p)).join('–')}`
    : e instanceof Plane ? `plane   @${ix.name(e.origin)}`
    // a curve written in the language is its statement; the program panel says the rest
    : e instanceof Curve ? 'curve'
    : e instanceof Axis ? 'axis'
    // a spline over points in space, among others: a curve the core traces, in no view
    : e instanceof Traced ? 'curve in space'
    : 'point';
  return `${n}${body}${tag}`;
}

/** `  ·in top` for a point drawn in a view, and nothing for one on the page. */
function inView(p: Point): string {
  const pl = p.plane;
  return pl ? `  ·in ${planeName(pl)}` : '';
}

/** What the source calls a view — and, for one it calls nothing, the sketch's own label. */
function planeName(pl: Plane): string {
  return view.doc.nameOf(pl) ?? pl.name;
}

/** What only changes when the sketch or the selection does: the status line's component and
 *  the banner. */
function refreshPicked(): void {
  // the focused constraint has left the document
  const focus = currentConstraint;
  if (focus && !view.sketch.userConstraints().includes(focus)) clearFocus();
  refreshHeld();
  refreshBanner();
}

/** What is picked, in the status line: one thing as its description, several by name. */
export function refreshHeld(): void {
  const picked = view.selected;
  if (!picked.length) {
    componentEl.textContent = heldLine();
    return;
  }
  const ix = new io.Index(view.sketch);
  // a status line is one line, and several descriptions do not fit across it
  componentEl.textContent = `${picked.length === 1 ? describeEntity(picked[0], ix)
    : `${picked.length} components: ${picked.map((e) => ix.name(e)).join(' ')}`}   |   `;
}
/** The status line for a selection that is not the drawing's: the traced picture, or solids. */
function heldLine(): string {
  const u = view.underlay;
  if (u?.picked) return `image ${u.name} — ${Math.round(u.opacity * 100)}%   |   `;
  const solids = view.selectedSolids;
  const said = (s: SolidPick) => {
    const { kind, name } = pickName(s);
    return `${kind} ${name}`;
  };
  if (solids.length === 1) return `${said(solids[0])}   |   `;
  if (solids.length) return `${solids.length} picked: ${solids.map(said).join(', ')}   |   `;
  return '';
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
  // an axis or a face is on no sheet and has no distance to anything: the core says NaN
  const d = distanceBetween(a, b);
  if (Number.isFinite(d)) rows.push(`distance  ${io.fmt(d, 6)}`);
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
  const unsolved = d?.status === 'unsolved' || (!!r && !r.settled);
  footerEl.classList.toggle('unsolved', conflict || unsolved);
  // nothing is diagnosed until there is a constraint to diagnose, and until then the freedom
  // left is simply every free parameter — there are no equations for one to be spent on
  const state = conflict ? '⚠ CONFLICT'
      : unsolved ? `⚠ NOT CONVERGED${r ? `  max|r|=${r.maxResidual.toExponential(1)}` : ''}`
      : `DOF ${d ? d.dof : view.sketch.freeIndices().length}`;
  stats(state);
  refreshMeasure();
}

export function refresh(): void {
  refreshPicked();
  refreshStatus();
}

function refreshBanner(): void {
  const d = view.diagnosis;
  banner.className = '';
  if (!d || (d.status !== 'conflict' && d.status !== 'over')) {
    // an object that does not evaluate is left out of the box, and this is where the page says
    // why, in the core's words — after the constraints, which may be what put it there
    const gone = unbuilt(view.sketch);
    if (!gone.length) return;
    bannerText.textContent = `⚠ Not drawn — ${gone.map((u) => u.reason).join('; ')}`;
    banner.className = 'unbuilt';
    bannerSelect.hidden = true;
    return;
  }
  const ix = new io.Index(view.sketch);
  if (d.status === 'conflict') {
    // a requirement out of reach is said with why, in the core's words: nothing to remove
    bannerText.textContent = d.outOfReach.length
      ? `⚠ ${d.outOfReach.map((r) => `${io.describe(r.constraint, view.doc)}: ${r.text}`).join('; ')}`
      : d.conflicts?.length
      ? `⚠ Conflicting constraints — remove one of: ${d.conflicts.map((c) => io.describe(c, view.doc)).join(', ')}`
      : `⚠ ${d.violated.length} constraint(s) cannot be satisfied`;
    banner.className = 'conflict';
  } else {
    bannerText.textContent = `⚠ ${d.nRedundant} redundant equation(s) (consistent, but over-constrained) among: `
      + d.over.map((c) => io.describe(c, view.doc)).join(', ');
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
  refresh();
  view.draw();
});
