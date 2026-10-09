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
 *             The drawing holds still while you carry it: nothing is solved until it lands.
 *             On two points, where you put it is *which* dimension it is: above or below them
 *             the run between them, out to either side the rise, across them their length.
 *             Every dimension is called out on the drawing: click one to select it, drag it
 *             where you want it, double-click it to change its number.  Edit ▸ Re-place
 *             dimensions undoes the arranging; Options turns the lot off.  A number may be an
 *             expression over the document's numbers — `w / 2`, `sin(h * 10)` — and the core
 *             evaluates them (Solution ▸ Diagnose lists them).  Typing a number over one that
 *             reads a `param` changes the param.  An unknown is declared, `param a: Length`:
 *             `a` on two dimensions ties them to each other and leaves what they are worth to
 *             the solver
 *   editing   F fix/unfix · G construction · Del delete · Ctrl+Z undo · ⇧Ctrl+Z redo ·
 *             Ctrl+X/C/V cut, copy, paste the selection
 *   workspace one scene in space: every sketch stands on its own plane and solids are drawn
 *             under them.  Right-drag orbits, middle-drag (or ⇧ right-drag) pans, the wheel
 *             zooms.  The chooser in the viewport's upper right is the plane the next thing is
 *             drawn on — std.front, std.side, std.top and every plane the document names — and
 *             choosing one turns the view square on to it.  Insert ▸ Plane… asks what a new
 *             view is and takes two clicks for where it sits; J projects two points, one in each
 *             of two views, onto one point in space
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
import * as solids from '../core/mesh.js';
import { related } from '../core/modules.js';
import type { ExportJob, ExportReply } from './export-worker.js';
import { drawingActive, exportDrawingSvg } from './drawing.js';
import { CONSTRAINT_BUTTONS } from './commands.js';
import {
  about, alternatives, doOpen, flipBranch, insertPlane, leaveExample, openCase, openExample, options,
  report, reportSolve, showDiagnosis,
} from './dialogs.js';
import { editValue, onDimension } from './dimbox.js';
import { bindProgramPanel, openDrawing, refreshProgram, resetProgramFiles, saveProjectFile,
  showStatementFor, toggleProgramPanel } from './program.js';
import { closePanel, openPanel, refresh, refreshPanel, refreshStatus } from './lists.js';
import {
  aboutBadge, barConstraints, barTools, canvas, currentConstraint, focusConstraint, hooks, menubar,
  planeSelect, view, initialExample,
} from './shell.js';
import {
  MenuItem, ToolbarButton, addButton, addMenu, addSeparator, askChoice, closeMenus, download, openImage,
  refining, toast,
} from './ui.js';
import { objectName, type Refining } from './field-preview.js';
import { Tool } from './view.js';
import { meshing } from './loading.js';

/* -- toolbars ---------------------------------------------------------------- */

const toolButtons = new Map<Tool, HTMLButtonElement>();

/** The buttons follow the view, so Escape backing out of a tool updates them too.  Select is
 *  not a button: it is what the toolbar looks like with nothing pressed, so clicking the
 *  pressed tool puts it down and lands you back there. */
view.onTool = (t) => {
  for (const [k, b] of toolButtons) b.setAttribute('aria-pressed', String(k === t));
};
/** The drawing tools, with the key each is taken up by — `TOOL_KEYS` is read off this. */
const TOOLS: [string, Tool, string][] = [
  ['Point', 'point', 'p'], ['Line', 'line', 'l'], ['Rect', 'rect', 'r'],
  ['Circle', 'circle', 'c'], ['Arc', 'arc', 'a'], ['Arc 3-pt', 'arc3', '3'],
  ['Spline', 'spline', 's'], ['Spline fit', 'splinefit', 'w'],
  ['Plane', 'plane', 'n'], ['Axis', 'axis', 'x'],
];
for (const [label, tool, key] of TOOLS) {
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
  if (drawingActive()) { exportDrawingSvg(); return; }
  download('sketch.svg', io.svg(view.sketch, view.width));
  toast('exported sketch.svg');
}

/** **The object, as a file.**
 *
 *  Two formats because they answer two questions.  glTF carries what the kernel knows — every
 *  face of every object under the name the document gives it, and the normals that make a bore's
 *  wall shade round — and is what a viewer opens.  STL carries triangles and nothing else, and
 *  is what a printer takes.  Neither is a boundary representation, so neither is what a
 *  machinist's STEP would be; that is a different thing and is not this.
 *
 *  What is exported is the document's **objects** — the solids nothing else is made of.  A bore
 *  is a hole in a part, not a part beside it. */
async function exportSolid(kind: 'glb' | 'stl'): Promise<void> {
  const objs = solids.objects(view.sketch);
  if (objs.length === 0) {
    toast('this drawing has no solid — a `solid` statement makes one from a `face`');
    return;
  }
  const stem = objs.length === 1 ? objs[0].name.replace(/[^\w.-]/g, '_') : 'solids';
  // the core refuses a solid it cannot write — a swept one still refining, above all — and says
  // why; unsaid, a menu item that does nothing reads as broken
  try { await exportFile(kind, objs, stem); }
  catch (e) { toast(`cannot export: ${e instanceof Error ? e.message : String(e)}`); }
}

async function exportFile(kind: 'glb' | 'stl', objs: { name: string; index: number }[], stem: string): Promise<void> {
  if (kind === 'glb') {
    // one scene holds every object, so glTF need not be told which part of an assembly to be
    download(`${stem}.glb`, solids.glb(view.sketch));
    toast(`exported ${stem}.glb — ${objs.length} object(s), every face named`);
    return;
  }
  // A swept solid still refining has no final surface; the preview as it stands is offered,
  // named as one, since it may be coarse or open.
  const still = new Set(objs.filter((o) => solids.provisional(view.sketch, o.index)).map((o) => o.index));
  const refining = objs.filter((o) => still.has(o.index));
  let preview = false;
  if (refining.length) {
    const names = refining.map((o) => o.name).join(', ');
    const choice = await askChoice('Export solid (STL)',
      `${names} ${refining.length === 1 ? 'is' : 'are'} still being refined.`, [
        { title: 'Export the preview', description: 'the surface as it stands now: it may be coarse, or open where refinement has not reached' },
        { title: 'Cancel', description: 'wait for the refinement to finish' },
      ]);
    if (choice !== 0) return;
    preview = true;
  }
  // an STL is a triangle soup with no grouping, so several objects go in one file as one soup
  const parts = objs.map((o) => preview && still.has(o.index)
    ? solids.stlPreview(view.sketch, o.index) : solids.stl(view.sketch, o.index));
  const name = `${stem}${preview ? '-preview' : ''}.stl`;
  download(name, parts.length === 1 ? parts[0] : joinStl(parts));
  toast(preview ? `exported ${name} — a preview, not the finished surface` : `exported ${name}`);
}

/** **A solid's exact export, made in a worker** (`export-worker.ts`): the core's own kernel builds
 *  each object — a swept body of the generating class as one sector patterned — and writes its STEP
 *  parsed back against it, or its STL held to the fabrication tolerance and to its material field;
 *  the functions `solventc --step/--stl` calls, so the file is the one the terminal writes. It takes
 *  seconds, so the page goes on drawing, and says so; a refusal is the export's, its stage named. */
let exporter: Worker | null = null;
let exportJob = 0;
const exportsWaiting = new Map<number, { name: string; kind: 'step' | 'stl' }>();
function exactExport(kind: 'step' | 'stl'): void {
  const objs = solids.objects(view.sketch);
  if (objs.length === 0) {
    toast('this drawing has no solid — a `solid` statement makes one from a `face`');
    return;
  }
  if (!exporter) {
    const w = new Worker(new URL('./export-worker.bundle.js', import.meta.url), { type: 'module' });
    w.onmessage = (ev: MessageEvent<ExportReply>) => {
      const reply = ev.data;
      const asked = exportsWaiting.get(reply.id);
      if (!asked) return;
      exportsWaiting.delete(reply.id);
      if ('error' in reply) { toast(`cannot export ${asked.name}: ${reply.error}`); return; }
      download(`${asked.name}.${asked.kind}`, reply.bytes);
      toast(`exported ${asked.name}.${asked.kind} — by the core's kernel, its file checked`);
    };
    w.onerror = (ev) => { exportsWaiting.clear(); toast(`the export stopped: ${ev.message || 'its worker failed'}`); exporter = null; };
    exporter = w;
  }
  const text = view.source;
  const modules = related(text).filter((f) => f.provided).map((f): [string, string] => [f.name, f.text]);
  for (const o of objs) {
    const id = ++exportJob;
    const name = o.name.replace(/[^\w.-]/g, '_');
    exportsWaiting.set(id, { name, kind });
    const job: ExportJob = { id, text, modules, x: view.sketch.getX(), solid: o.index, kind, tolerance: 0.01 };
    exporter.postMessage(job);
  }
  toast(`exporting ${objs.length === 1 ? objs[0].name : `${objs.length} objects`} as ${kind.toUpperCase()} — a swept body takes seconds`);
}

/** **The background refinement, said in the footer** — one entry an object, while any is going:
 *  its phase (the rough first pass, tracing sharp edges, the final pass, and a repair's rebuilds),
 *  a bar of its estimated progress, its triangles so far and the time taken. The estimate is how
 *  far the worst facet waiting has come toward the criteria, and no count of work left, which no
 *  refinement knows. Finished, the line says so for a few seconds; failed, it stays with why.
 *
 *  **An exact surface's build is said beside its preview**: the stage it is on in the core's
 *  words, a bar of the stages done, a spinner and a clock. A stage of a gear runs for seconds with
 *  nothing said, so the page ticks the spinner and the clock itself while one is building: work
 *  going on never looks like work that has stopped. Built, the entry says so; refused, the preview
 *  is the surface and the entry says it is the field's. */
let refineTimer = 0;
let refineTick = 0;
let refineList: Refining[] = [];
const SPIN = '◐◓◑◒';
function showRefining(list: Refining[]): void {
  refineList = list;
  clearTimeout(refineTimer);
  clearInterval(refineTick);
  refineTick = 0;
  if (!list.length) { refining(''); return; }
  const clock = (ms: number): string => {
    const s = Math.floor(ms / 1000);
    return `${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}`;
  };
  const bar = (f: number): string => {
    const n = Math.round(Math.max(0, Math.min(1, f)) * 8);
    return '▰'.repeat(n) + '▱'.repeat(8 - n);
  };
  const building = (r: Refining): boolean => !!r.exact && !r.exact.built && !r.exact.error;
  const spin = SPIN[Math.floor(performance.now() / 250) % SPIN.length];
  const failed = list.filter((r) => (r.error || r.progress?.failed) && !r.exact?.built);
  const parts = list.map((r) => {
    const p = r.progress;
    const x = r.exact;
    if (x?.built) return `${objectName(r.name)}: exact solid · ${x.triangles.toLocaleString()} triangles · ${clock(performance.now() - r.since)}`;
    // the preview, as it stands: refined, or the pass in hand (what it is doing is the core's words)
    const preview = r.error || p?.failed ? `failed — ${r.error ?? p?.stage}`
      : r.done ? `refined · ${r.triangles.toLocaleString()} triangles`
      : `${p?.doing ?? 'starting'} ${bar(p?.within ?? 0)} ${r.triangles.toLocaleString()} triangles`;
    if (x && building(r)) {
      const shown = r.done ? `preview ${r.triangles.toLocaleString()} triangles` : `preview ${r.triangles.toLocaleString()} triangles…`;
      return `${objectName(r.name)}: exact solid ${spin} ${x.doing} ${bar(x.done / Math.max(1, x.total))} ${clock(performance.now() - r.since)}`
        + ` · ${shown}`;
    }
    const field = x?.error ? ' (the field\'s surface: no exact solid for it)' : '';
    return `${objectName(r.name)}: ${preview}${field} · ${clock(r.elapsed)}`;
  });
  const text = parts.join('   |   ');
  if (list.some(building)) {
    // the stage in hand says nothing till it ends: the clock and the spinner say it is going
    refineTick = window.setInterval(() => showRefining(refineList), 250);
    refining(text);
    return;
  }
  if (failed.length) { refining(text, 'failed'); return; }
  if (list.every((r) => r.done || r.exact?.built)) {
    refining(text, 'finished');
    refineTimer = window.setTimeout(() => refining(''), 5000);
    return;
  }
  refining(text);
}

/** Several binary STLs as one: the header of the first, the summed count, and every record. */
function joinStl(parts: Uint8Array[]): Uint8Array {
  const n = parts.reduce((a, p) => a + (new DataView(p.buffer, p.byteOffset).getUint32(80, true)), 0);
  const out = new Uint8Array(84 + n * 50);
  out.set(parts[0].subarray(0, 84));
  new DataView(out.buffer).setUint32(80, n, true);
  let at = 84;
  for (const p of parts) {
    out.set(p.subarray(84), at);
    at += p.length - 84;
  }
  return out;
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
    { label: 'New', onClick: () => view.newDocument() },
    { label: 'Open…', onClick: () => void doOpen() },
    { label: 'Open drawing folder…',
      onClick: () => void openDrawing().then((opened) => { if (opened) leaveExample(); }) },
    { label: 'Open examples…', onClick: () => void openCase() },
    null,
    { label: 'Save', onClick: () => {
      if (!saveProjectFile()) download('sketch.json', io.dumps(view.sketch));
    } },
    { label: 'Export SVG', onClick: exportSvg,
      title: 'The drawing as a scalable image, laid out by the core — the same figure `solventc '
        + '--output` writes' },
    { label: 'Export solid (glTF)', onClick: () => void exportSolid('glb'),
      title: 'The object as a viewer opens it: every face a named node, in metres.  What '
        + '`solventc --gltf` writes' },
    { label: 'Export solid (STL)', onClick: () => void exportSolid('stl'),
      title: 'The object as a printer takes it: triangles, welded so every edge has its '
        + 'partner, from the drawing\'s own mesh' },
    { label: 'Export solid (STEP)', onClick: () => exactExport('step'),
      title: 'The exact solid, built by the core\'s own kernel and held to 10 µm: what '
        + '`solventc --step --tolerance` writes' },
    { label: 'Export solid for fabrication (STL)', onClick: () => exactExport('stl'),
      title: 'The exact solid meshed within 10 µm and held to its material field: what '
        + '`solventc --stl --tolerance` writes' },
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
    { label: 'Show solid', key: '⇧⌘b', onClick: () => view.setShowSolid(!view.showSolid),
      title: 'Fill each object\u2019s surfaces, not only its edges — off, a solid is a wireframe' },
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
      title: 'A plane to draw in, over two lines: the one it runs along, then the one that says '
        + 'which way is up in it' },
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
for (const [label, items] of MENUS) {
  // The file menu works in either view. Geometry commands need a selected model file.
  const shared = new Set(['New', 'Open…', 'Open drawing folder…', 'Open examples…',
    'Save', 'Export SVG', 'Program']);
  for (const item of items) {
    if (!item || shared.has(item.label)) continue;
    const action = item.onClick;
    item.onClick = () => {
      if (drawingActive()) { toast('Select a .sv file to work on its geometry'); return; }
      action();
    };
  }
  addMenu(menubar, label, items);
}
aboutBadge.addEventListener('click', () => void about());

/* -- keyboard ------------------------------------------------------------------- */

const TOOL_KEYS: Record<string, Tool> =
  Object.fromEntries(TOOLS.map(([, tool, key]) => [key, tool]));
/** Every accelerator in the app, read off the buttons and menu items themselves so there is
 *  one list and not two.  The token is the chip the control prints, lowercased: '⇧l', '⌘z'. */
const ACTION_KEYS = new Map<string, () => void>(
  [...TOOL_BUTTONS, ...CONSTRAINT_BUTTONS, ...MENUS.flatMap(([, items]) => items)]
    .flatMap((b) => (b?.key ? [[b.key, b.onClick] as [string, () => void]] : [])));

window.addEventListener('keydown', (e) => {
  const t = e.target as HTMLElement | null;
  if (t && (t.tagName === 'INPUT' || t.tagName === 'SELECT' || t.tagName === 'TEXTAREA')) return;
  if (drawingActive()) {
    if (e.key === 'Escape') closeMenus();
    if ((e.metaKey || e.ctrlKey) && e.key.toLowerCase() === 'p') {
      e.preventDefault(); toggleProgramPanel();
    }
    return;
  }
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

/* -- the plane being drawn on ------------------------------------------------- */

/** The chooser follows the view: the planes on offer and the one being drawn on.  Rebuilt only
 *  when the list itself changed, so a repaint does not close it under the pointer. */
function refreshPlanes(): void {
  const names = view.planeChoices();
  const current = view.planeName;
  if (!names.includes(current)) names.push(current);
  const shown = [...planeSelect.options].map((o) => o.value);
  if (shown.length !== names.length || names.some((n, i) => shown[i] !== n)) {
    planeSelect.replaceChildren(...names.map((n) => new Option(n, n)));
  }
  planeSelect.value = current;
}
planeSelect.addEventListener('change', () => {
  view.choosePlane(planeSelect.value);
  planeSelect.blur();              // the keys are the drawing's again: a tool letter, Escape
});
view.onDimension = onDimension;
view.onChanged = () => { refresh(); refreshProgram(); refreshPlanes(); };
view.onPicked = refreshPanel;
// the source changed without the drawing's structure doing so — a drag wrote its seeds back, or a
// number was spliced.  Never per frame: `onDragFrame` is the frame seam and this is not wired to it
view.onProgram = () => { refreshProgram(); };
view.onLoad = () => { if (resetProgramFiles()) leaveExample(); };
view.onDragFrame = refreshStatus;
view.onStatus = toast;
view.onRefine = (list) => { showRefining(list); meshing(list); };
hooks.focusChanged = showStatementFor;
bindProgramPanel();
new ResizeObserver(() => view.resize()).observe(canvas);
view.resize();
view.afterEdit();
view.fit();
window.addEventListener('popstate', () => {
  const url = new URL(location.href);
  void openExample(url.searchParams.get('example') ?? 'rect_fillets', 'none',
    url.searchParams.get('file') ?? undefined);
});
await openExample(initialExample, 'replace', new URL(location.href).searchParams.get('file') ?? undefined);
