/* The sketcher shell: toolbars, the constraint list, the diagnosis banner and the dialogs.
 *
 *   tools     P point · L line · R rectangle · C circle · A arc (centre, start, end)
 *             3 arc through two ends and a point on it
 *             O ellipse (centre, major end, then a click the rim passes through)
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
 *             The drawing holds still while you carry it: nothing is solved until it lands.
 *             On two points, where you put it is *which* dimension it is: above or below them
 *             the run between them, out to either side the rise, across them their length.
 *             Every dimension is called out on the drawing: click one to select it, drag it
 *             where you want it, double-click it to change its number.  Edit ▸ Re-place
 *             dimensions undoes the arranging; Options turns the lot off.  A number may be an
 *             expression — `w = 80` names it, `h = w / 2` and `sin(h * 10)` use it — and the
 *             core evaluates them in dependency order (Solution ▸ Diagnose lists them).  A name
 *             *nothing* defines is a free variable: `a` on two dimensions ties them to each
 *             other and leaves what they are worth to the solver
 *   editing   F fix/unfix · G construction · Del delete · Ctrl+Z undo · ⇧Ctrl+Z redo ·
 *             Ctrl+X/C/V cut, copy, paste the selection · wheel zoom · right-drag pan
 *   views     Insert ▸ Plane… asks what the view is and takes two clicks for where it sits;
 *             Insert ▸ Three views lays out front, top and right.  A plane selected on its own
 *             is the one being drawn in — every point a tool mints goes `in` it, and the
 *             status line says so — until Insert ▸ Draw on page.  J projects two points, one
 *             in each of two views, onto one point in space
 *   menus     File/Edit/Insert/Solution hold everything that is not a tool or a constraint;
 *             the solver's own switches are behind Solution ▸ Options
 *
 * Everything below is presentation; the solver, diagnosis, decomposition and root selection
 * all live in core/ and are shared with the test suite.  This file is the wiring: what is on
 * the bars, what the keys do, and what the view calls back into.  The rest of the shell is
 * next door — `shell` holds the page and the view, `commands` the constraints bar, `dialogs`
 * what the menus open, `lists` the constraints window and the status line, and `dimbox` the
 * number a dimension is edited in. */
import * as io from '../core/io.js';
import { CONSTRAINT_BUTTONS } from './commands.js';
import {
  about, alternatives, doOpen, flipBranch, insertPlane, newSketch, openCase, options, report,
  reportSolve, showDiagnosis,
} from './dialogs.js';
import { threeViews } from './tools.js';
import { editValue, onDimension } from './dimbox.js';
import { bindProgramPanel, refreshProgram, showStatementFor, toggleProgramPanel } from './program.js';
import { closePanel, openPanel, refresh, refreshPanel, refreshStatus } from './lists.js';
import {
  aboutBadge, barConstraints, barTools, canvas, currentConstraint, focusConstraint, hooks, menubar,
  view,
} from './shell.js';
import {
  MenuItem, ToolbarButton, addButton, addMenu, addSeparator, closeMenus, download, openImage,
  toast,
} from './ui.js';
import { Tool } from './view.js';

/* -- toolbars ---------------------------------------------------------------- */

const toolButtons = new Map<Tool, HTMLButtonElement>();

/** The buttons follow the view, so Escape backing out of a tool updates them too.  Select is
 *  not a button: it is what the toolbar looks like with nothing pressed, so clicking the
 *  pressed tool puts it down and lands you back there. */
view.onTool = (t) => {
  for (const [k, b] of toolButtons) b.setAttribute('aria-pressed', String(k === t));
};
for (const [label, tool, key] of [
  ['Point', 'point', 'p'], ['Line', 'line', 'l'], ['Rect', 'rect', 'r'],
  ['Circle', 'circle', 'c'], ['Arc', 'arc', 'a'], ['Arc 3-pt', 'arc3', '3'],
  ['Ellipse', 'ellipse', 'o'], ['Spline', 'spline', 's'], ['Spline fit', 'splinefit', 'w'],
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
for (const b of CONSTRAINT_BUTTONS) addButton(barConstraints, b);

/* -- menu bar ------------------------------------------------------------------- */

/** The drawing as an SVG file.
 *
 *  **The core draws it.**  The figures, the strokes a style sheet resolves to and the whole of
 *  the arithmetic are `gcs_core::svg`'s, which is what keeps this button and `solventc --output`
 *  from being two implementations of one picture.  The app contributes the one thing the core
 *  cannot know: how wide the page is.  The canvas's own width is the honest answer — the export
 *  is the size of the area the drawing was being looked at in — and everything drawn at a
 *  constant size follows from it, since a page width is what fixes `unit`.  A canvas with no
 *  width yet is `svg::render`'s business and not this button's — it floors the page itself, and
 *  a second floor here would be a second rule about the same number.
 *
 *  The *camera* is deliberately not consulted.  An export is of the drawing, not of the view:
 *  the core fits the whole figure to the page, so a file is the same whatever the pan and zoom
 *  happened to be when the button was pressed. */
function exportSvg(): void {
  download('sketch.svg', io.svg(view.sketch, view.width));
  toast('exported sketch.svg');
}

/** Put a picture behind the drawing to trace over.
 *
 *  It is **view state, not document state** — scenery, like the camera and the colouring, and
 *  the same in kind as the paper you would tape to a drawing board.  So it is not saved, not
 *  exported, not solved and not undone: undoing a line must not move your photograph, and the
 *  Solvent document stays what somebody wrote.  What survives tracing is the drawing. */
async function traceImage(): Promise<void> {
  const got = await openImage();
  if (!got) return;
  view.traceImage(got.image, got.name, got.url);
}

/** The traced picture's two keys, live only while it is the selected thing — the brackets a
 *  drawing program fades a reference layer with.  The gate is here and not in `fadeImage`,
 *  which fades whatever picture there is: it is about what the *accelerator* may do, so it
 *  belongs beside the accelerator, in the one table every key in the app is read off. */
const fade = (by: number) => () => { if (view.underlay?.picked) view.fadeImage(by); };

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
    { label: 'Export SVG', onClick: exportSvg,
      title: 'The drawing as a scalable image, laid out by the core — the same figure `solventc '
        + '--output` writes' },
    null,
    { label: 'Trace image…', onClick: () => void traceImage(),
      title: 'Put a picture behind the drawing to draw over.  It is scenery — not saved, not '
        + 'exported, not undone' },
    { label: 'Remove image', onClick: () => view.removeImage(),
      title: 'Take the traced picture away again' },
    { label: 'Fade image', key: '[', onClick: fade(-0.1),
      title: 'Fade the traced picture, while it is the selected thing' },
    { label: 'Brighten image', key: ']', onClick: fade(0.1),
      title: 'Bring the traced picture back up, while it is the selected thing' },
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
    { label: 'Program', key: '⌘p', onClick: () => toggleProgramPanel(),
      title: 'The program this drawing is written as — edit it and the drawing follows' },
    { label: 'Re-place dimensions', onClick: () => {
      // the focused dimension if there is one, otherwise every dimension on the drawing
      const n = view.resetCallouts(currentConstraint);
      toast(n ? `${n} dimension(s) put back` : 'no dimension has been moved');
    } },
  ]],
  ['Insert', [
    { label: 'Plane…', onClick: () => void insertPlane(),
      title: 'A view of space to draw in: the page, one folded from another view, or an '
        + 'explicit basis.  Two clicks then say where it sits' },
    { label: 'Three views', onClick: () => threeViews(view),
      title: 'Front, top and right views in the standard third-angle layout, aligned' },
    null,
    { label: 'Draw on page', onClick: () => view.drawOnPage(),
      title: 'The next points go on the page itself rather than in the current view' },
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

/* -- keyboard ------------------------------------------------------------------- */

const TOOL_KEYS: Record<string, Tool> = {
  p: 'point', l: 'line', r: 'rect', c: 'circle', a: 'arc', 3: 'arc3', s: 'spline',
  w: 'splinefit', o: 'ellipse',
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

/* A canvas press takes the selection over from the constraint that had the focus — and one
 * that picked nothing shuts the constraints window, which is the only thing that does. */
view.onSelect = () => {
  if (currentConstraint) focusConstraint(null);
  if (!view.selected.length) closePanel();
  showStatementFor();
};
/* A dimension on the drawing and its row in the window are the same constraint, so clicking
 * either does the same thing — and double-clicking either opens the same number.  One clicked
 * on the drawing takes no selection with it, so it opens the window on what it holds. */
view.onPickConstraint = (c) => {
  focusConstraint(c);
  openPanel(view.highlight);
  refresh();
  view.draw();
};
view.onEditConstraint = (c) => editValue(c);
view.onDimension = onDimension;
view.onChanged = () => { refresh(); refreshProgram(); };
view.onPicked = refreshPanel;
// the source changed without the drawing's structure doing so — a drag wrote its seeds back, or a
// number was spliced.  Never per frame: `onDragFrame` is the frame seam and this is not wired to it
view.onProgram = () => { refreshProgram(); };
view.onDragFrame = refreshStatus;
view.onStatus = toast;
hooks.focusChanged = showStatementFor;
bindProgramPanel();
new ResizeObserver(() => view.resize()).observe(canvas);
view.resize();
view.fit();
view.afterEdit();
