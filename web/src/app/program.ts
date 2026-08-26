/* The document, shown beside the drawing it makes.
 *
 * **This is not a view of the sketch — it is the source, and the drawing is what it came to.**  So
 * the panel shows `view.source` and never re-prints: the comments somebody wrote, the components
 * they factored out and the blank lines they left are the document as much as the numbers are, and
 * a re-print would put a lift of the drawing there instead, losing all of it on the first drag.
 *
 * The core owns the language entirely — reading, elaborating, editing and where every statement
 * sits.  This module is the panel: it puts the text on the page, keeps out of the way while
 * somebody is typing in it, and hands what they typed back.
 *
 * Two rules, both borrowed from modules that learned them the hard way.  `lists.ts` rebuilds its
 * rows only when the contents actually changed, so a caret and a scroll position survive an edit
 * made elsewhere; `dimbox.ts` will not overwrite a box somebody is in.  A panel that reprinted on
 * every solve would take the line out from under the cursor. */
import type { Diagnostic, SourceMap } from '../core/program.js';
import { currentConstraint, pdiags, ped, ppanel, ppanelState, psplit, view } from './shell.js';
import { toast } from './ui.js';

/** The box somebody types in.  `app/editor.ts` owns the two layers and the colouring; this module
 *  owns what the text *means* — which document it came from and what applying it does. */
const ptext = ped.box;

/** The text the panel last put there.  An identical reprint is one string compare and no write,
 *  which is what makes refreshing on every change affordable. */
let shown = '';
/** Somebody is editing: nothing overwrites the box until they are done with it. */
let typed = false;


export function programPanelOpen(): boolean {
  return !ppanel.hidden;
}

/** Show the program, or stop showing it.
 *
 *  **On by default.**  The source is not a remark about the drawing — it is what the drawing
 *  *is*, so the page opens on both, and closing it is the deliberate act.  The partition goes
 *  wherever the panel does: a handle for something that is not there resizes nothing. */
export function toggleProgramPanel(): void {
  ppanel.hidden = !ppanel.hidden;
  psplit.hidden = ppanel.hidden;
  if (!ppanel.hidden) {
    shown = '';                 // it has been away; whatever it held is stale
    refreshProgram();
  }
  view.resize();                // the canvas is a different width now
}

/** The partition between the drawing and the source.
 *
 *  The panel is on the right, so its width is the distance from the pointer to the row's right
 *  edge — measured against that edge rather than accumulated from where the drag began, so a
 *  pointer that runs past a limit and comes back picks the edge up where it left it instead of
 *  an offset away.  The canvas is watched by a `ResizeObserver` (`main.ts`), so nothing here
 *  has to tell the view it got narrower. */
function bindPartition(): void {
  const to = (clientX: number): void => {
    const row = ppanel.parentElement;
    if (row) setPanelWidth(row.getBoundingClientRect().right - clientX);
  };
  psplit.addEventListener('pointerdown', (e) => {
    e.preventDefault();                        // a drag on a separator is not a text selection
    psplit.setPointerCapture(e.pointerId);
    psplit.classList.add('dragging');
  });
  psplit.addEventListener('pointermove', (e) => {
    if (psplit.hasPointerCapture(e.pointerId)) to(e.clientX);
  });
  const done = (e: PointerEvent): void => {
    if (!psplit.hasPointerCapture(e.pointerId)) return;
    psplit.releasePointerCapture(e.pointerId);
    psplit.classList.remove('dragging');
  };
  psplit.addEventListener('pointerup', done);
  psplit.addEventListener('pointercancel', done);
  // and by keyboard, since it is focusable: a separator nobody can reach with the keyboard is a
  // control only half the people using it have
  psplit.addEventListener('keydown', (e) => {
    const step = e.key === 'ArrowLeft' ? 1 : e.key === 'ArrowRight' ? -1 : 0;
    if (!step) return;
    e.preventDefault();
    setPanelWidth(ppanel.getBoundingClientRect().width + step * (e.shiftKey ? 40 : 8));
  });
}

/** How wide the panel may be, as the stylesheet says.  Read rather than restated: the drag and
 *  the layout would otherwise be two rules about one limit, and the first edit to either would
 *  make them disagree. */
function widthBounds(room: number): [number, number] {
  const s = getComputedStyle(ppanel);
  // `width` comes back resolved to pixels; `min-width`/`max-width` do **not** — a percentage is
  // handed back as it was written, so `parseFloat('60%')` is sixty *pixels* unless it is asked
  // what of.  Left unasked, the panel clamped itself to a 60px maximum and the handle spent the
  // whole drag pinned against a limit the layout was quietly correcting.
  const px = (v: string, fallback: number): number => {
    const n = parseFloat(v);
    if (!Number.isFinite(n)) return fallback;
    return v.trimEnd().endsWith('%') ? (n / 100) * room : n;
  };
  return [px(s.minWidth, 240), px(s.maxWidth, room * 0.6)];
}

/** Put the partition somewhere, in whatever units keep it there.
 *
 *  A **percentage** of the row, not pixels: the panel opens at 30% of the window, and a width
 *  frozen in pixels the moment somebody nudged the handle would stop tracking a window that is
 *  then resized — the drawing and the source would drift apart on a laptop being plugged into a
 *  monitor.  Clamped here to what the stylesheet allows, so the handle never runs on past a
 *  limit the layout is quietly enforcing and leave the pointer somewhere the edge is not. */
function setPanelWidth(px: number): void {
  const room = ppanel.parentElement?.clientWidth ?? window.innerWidth;
  if (room <= 0) return;
  const [lo, hi] = widthBounds(room);
  const w = Math.min(Math.max(px, lo), Math.min(hi, room));
  ppanel.style.width = `${((w / room) * 100).toFixed(3)}%`;
}

/** Re-print, unless the panel is being typed in or already says this. */
export function refreshProgram(): void {
  if (ppanel.hidden) return;
  if (typed) {
    ppanel.classList.add('dirty');
    ppanelState.textContent = ' — edited, ⌘↵ to apply';
    return;
  }
  const text = view.source;
  if (text === shown) return;
  shown = text;
  ped.setText(text);
  ped.setLit(litSpan());        // the spans moved with the text; the mark follows them
  ppanel.classList.remove('dirty');
  ppanelState.textContent = '';
  showDiags(view.doc.diagnostics);
}

/** Apply what is in the box.  One undo entry, one solve, one diagnosis — like every other edit.
 *
 *  A program that will not read leaves the drawing exactly as it was and says why: half a
 *  statement is not an instruction to delete anything. */
export function applyProgram(): boolean {
  const text = ptext.value;
  const undo = view.source;
  if (!view.setProgram(text, false)) return false;
  showDiags(view.doc.diagnostics);
  if (!view.doc.ok) {
    // it drew what it could, and the errors are in the gutter beside the lines that caused them
    const first = view.doc.diagnostics.find((d) => d.severity === 'error');
    toast(first ? `line ${first.line}: ${first.message}` : 'the program has an error');
  } else {
    toast('program applied');
  }
  view.pushUndo(undo);
  typed = false;
  shown = text;
  ppanel.classList.remove('dirty');
  ppanelState.textContent = '';
  return view.doc.ok;
}

/** Put back what the drawing says, throwing away what was typed. */
export function revertProgram(): void {
  typed = false;
  shown = '';
  ppanel.classList.remove('dirty');
  refreshProgram();
}

/** Where every part of the drawing was written, in *this* elaboration.  Asked each time rather
 *  than remembered: a map belongs to the elaboration that made it, and this panel outlives many. */
function map(): SourceMap {
  return view.doc.map;
}

function showDiags(ds: Diagnostic[]): void {
  pdiags.replaceChildren();
  for (const d of ds) {
    const li = document.createElement('li');
    li.className = d.severity === 'error' ? 'error' : 'warn';
    li.textContent = `${d.code}  ${d.line}:${d.col}  ${d.message}`;
    li.addEventListener('click', () => {
      ptext.focus();
      ptext.setSelectionRange(d.lo, Math.max(d.hi, d.lo + 1));
    });
    pdiags.append(li);
  }
}

/** Light the statement that made what is picked.  `highlight`, never `selected`: setting the
 *  selection would fire `onSelect`, which is a press on the canvas and shuts the constraints
 *  window. */
/** Where the thing being looked at was written down — the focused constraint, or else the first
 *  selected element.  Null when neither is, and when what is picked came out of a component and
 *  so has no statement of its own in this document to point at. */
function litSpan(): [number, number] | null {
  const c = currentConstraint;
  if (c) {
    const f = map().constraints.find((x) => x.id === c.id);
    return f ? [f.lo, f.hi] : null;
  }
  const first = view.selected[0];
  if (!first) return null;
  const f = map().entities.find((x) => x.kind === first.ref[0] && x.index === first.ref[1]);
  return f ? [f.lo, f.hi] : null;
}

/** Point the panel at whatever is picked: mark the statement it was written as, and bring it on
 *  screen.  Called from both funnels — `view.onSelect` for the drawing, `hooks.focusChanged` for
 *  a constraint — so picking either way says the same thing here. */
export function showStatementFor(): void {
  if (ppanel.hidden || typed) return;
  const where = litSpan();
  ped.setLit(where);
  // the selection and the scroll are only for a box nobody is in: moving either under somebody
  // who is typing would take the line out from under their caret
  if (!where || document.activeElement === ptext) return;
  ptext.setSelectionRange(where[0], where[1]);
  ped.scrollToLine(shown.slice(0, where[0]).split('\n').length - 1);
}

/** Wire the box up.  Called once, from `main`, so this module is reached the way `lists` is and
 *  the view never has to import the shell. */
export function bindProgramPanel(): void {
  bindPartition();
  ptext.addEventListener('input', () => {
    typed = true;
    // colour what was just typed, not what the drawing came from: half a statement is still the
    // program somebody is looking at, and the core colours it as far as it goes
    ped.repaint();
    ppanel.classList.add('dirty');
    ppanelState.textContent = ' — edited, ⌘↵ to apply';
  });
  ptext.addEventListener('keydown', (e) => {
    // every accelerator in `main` is already yielded inside a TEXTAREA, but a handler that did
    // not stop here would still reach the window listeners below it
    e.stopPropagation();
    if (e.key === 'Enter' && (e.metaKey || e.ctrlKey)) {
      e.preventDefault();
      applyProgram();
    } else if (e.key === 'Escape') {
      e.preventDefault();
      // put the text back *before* blurring, so the blur handler finds nothing to apply
      revertProgram();
      ptext.blur();
    }
  });
  ptext.addEventListener('blur', () => {
    if (typed) applyProgram();
  });
  // a click in the text says which statement, and the drawing lights what it made
  ptext.addEventListener('click', () => {
    const off = ptext.selectionStart;
    // the innermost statement containing the caret: a statement inside a block is inside its
    // block's span, and the one that made something is the one a click there means
    let best: { name: string; lo: number; hi: number } | null = null;
    for (const x of map().entities) {
      if (off < x.lo || off >= x.hi) continue;
      if (!best || x.hi - x.lo < best.hi - best.lo) best = x;
    }
    const e = best && view.doc.entity(best.name);
    if (e) {
      view.highlight = [e];
      view.draw();
    }
  });
}
