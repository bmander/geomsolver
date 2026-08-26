/* The measuring half of `tools/overlay.mjs`, run inside the browser.
 *
 * It drives the **real** `CodeEditor` — not a copy of it — and checks the invariant the control
 * exists to keep: the coloured copy and the box in front of it put every character in the same
 * place, so the caret always sits in the text it appears to be in.
 *
 * There is no API that reports where a textarea draws its caret, so the invariant is checked in
 * three pieces, and between them they cover every way the two layers have actually come apart:
 *
 *   1. the two lay text out the same way at all — same computed metrics, and the box's own idea
 *      of how big its text is agrees with a plain `<pre>` of the same text;
 *   2. cutting the text into coloured runs moves nothing — the spanned copy places every
 *      character exactly where an unspanned mirror of it does (this is what an italic, a bold, a
 *      ligature or a kern across a run boundary breaks);
 *   3. the copy follows the box's scrolling exactly, at the extremes as well as the middle (this
 *      is what a scrollbar's width breaks, by shortening one layer's scroll range).
 *
 * The harness bundles this and hands the case in `window.CASE`, so the page never guesses at the
 * language: the runs come from the core, the same as in the app. */
import { CodeEditor } from '../src/app/editor.js';

const { text, runs, lit } = window.CASE;

const ed = new CodeEditor(document.getElementById('pcode'), () => runs);
ed.setText(text);
// and mark a range, because a mark is the other thing that can move a glyph: `.lit` may tint and
// thicken an outline, never change the face.  The check below is exactly the one that catches it
// if it ever does.
if (lit) ed.setLit(lit);

const copy = document.querySelector('.code-copy');
const box = ed.box;
const out = [];
const say = (s) => out.push(s);
let bad = 0;
const fail = (s) => { bad++; say(`FAIL  ${s}`); };

/* -- 1. do the two layers set text the same way? ----------------------------------- */

const a = getComputedStyle(copy), b = getComputedStyle(box);
for (const p of ['fontFamily', 'fontSize', 'fontWeight', 'fontStyle', 'lineHeight',
                 'letterSpacing', 'wordSpacing', 'paddingTop', 'paddingLeft', 'whiteSpace',
                 'tabSize', 'fontKerning', 'fontVariantLigatures', 'textIndent']) {
  if (a[p] !== b[p]) fail(`${p}: the copy has ${a[p]}, the box has ${b[p]}`);
}

// a plain `<pre>` of the same text, styled identically, off to one side: the stand-in for "how
// this text is laid out" that both the box and the copy have to agree with
const mirror = document.createElement('pre');
mirror.className = 'code-copy';
// no min-width/height: the mirror measures the *text*, not the panel it would fill
mirror.style.cssText = 'position:absolute; visibility:hidden; top:0; left:0; min-width:0; min-height:0;';
mirror.textContent = `${text}\n`;
document.getElementById('pcode').append(mirror);

// the box's own measure of its text, against the mirror's.  A textarea's scrollWidth omits the
// trailing padding a block counts, so the width is compared allowing for exactly that.
const pad = parseFloat(a.paddingLeft) + parseFloat(a.paddingRight);
const dw = mirror.scrollWidth - box.scrollWidth, dh = mirror.scrollHeight - box.scrollHeight;
say(`text size   mirror ${mirror.scrollWidth}x${mirror.scrollHeight}` +
    `   box ${box.scrollWidth}x${box.scrollHeight}   (width differs by ${dw}, height by ${dh})`);
if (dw !== 0 && dw !== parseFloat(a.paddingRight) && dw !== pad) {
  fail(`the box and a plain <pre> disagree about how wide this text is, by ${dw}px`);
}
// height only means something once the text is taller than the box, since a textarea's
// scrollHeight never reports less than the box it sits in
const line = parseFloat(a.lineHeight);
if (mirror.scrollHeight > box.clientHeight && dh !== line && dh !== 0) {
  fail(`the box and a plain <pre> disagree about how tall this text is, by ${dh}px ` +
       `(a line is ${line}px)`);
}

/* -- 2. do the coloured runs move anything? ---------------------------------------- */

/** Every text node of `root`, with the offset into `text` at which it starts. */
function nodesOf(root) {
  const w = document.createTreeWalker(root, NodeFilter.SHOW_TEXT);
  const found = [];
  let n, seen = 0;
  while ((n = w.nextNode())) {
    found.push({ node: n, from: seen });
    seen += n.textContent.length;
  }
  return found;
}

/** Where `root` draws `text[off]`, relative to `root` itself. */
function glyphIn(root, nodes, off, origin) {
  for (const { node, from } of nodes) {
    const i = off - from;
    if (i < 0 || i >= node.textContent.length) continue;
    const r = document.createRange();
    r.setStart(node, i);
    r.setEnd(node, i + 1);
    const box = r.getBoundingClientRect();
    if (box.width === 0 && box.height === 0) return null;
    return { x: box.x - origin.x, y: box.y - origin.y, w: box.width };
  }
  return null;
}

const spanned = nodesOf(copy), plain = nodesOf(mirror);
const co = copy.getBoundingClientRect(), mo = mirror.getBoundingClientRect();
let checked = 0, moved = 0;
const misses = [];
let ln = 0;
for (let off = 0; off < text.length; off++) {
  if (text[off] === '\n') { ln++; continue; }
  const g = glyphIn(copy, spanned, off, co);
  const m = glyphIn(mirror, plain, off, mo);
  if (!g || !m) continue;
  checked++;
  // Sub-pixel disagreement is the floor of the technique: the copy is many inline boxes and the
  // plain text is one, and each box's advance is rounded on its own — about 0.06px per line in
  // practice, constant along the line rather than piling up.  What must never happen is a shift
  // anyone could see, so the bar is half a pixel: a fifteenth of a character.
  if (Math.abs(g.x - m.x) > 0.5 || Math.abs(g.y - m.y) > 0.5) {
    moved++;
    if (misses.length < 8) {
      const col = off - (text.lastIndexOf('\n', off - 1) + 1);
      misses.push(`  line ${ln} col ${col} ${JSON.stringify(text[off])}: coloured at ` +
                  `(${g.x.toFixed(2)}, ${g.y.toFixed(2)}), plain at (${m.x.toFixed(2)}, ` +
                  `${m.y.toFixed(2)}) — out by (${(g.x - m.x).toFixed(2)}, ${(g.y - m.y).toFixed(2)})`);
    }
  }
}
say(`glyphs      ${checked} compared against the plain text, ${moved} moved by their colouring`);
if (moved) { bad++; out.push(...misses); }

/* -- 3. does the copy follow the box, all the way to the ends? --------------------- */

/** The copy is *translated*, never scrolled; this reads back where it has been put. */
function shown() {
  const t = new DOMMatrixReadOnly(getComputedStyle(copy).transform);
  return { x: -t.e, y: -t.f };
}
const stops = [0, 1, 7, Math.floor(box.scrollHeight / 3), box.scrollHeight];
for (const want of stops) {
  box.scrollTop = want;
  box.dispatchEvent(new Event('scroll'));
  const at = shown();
  if (at.y !== box.scrollTop) {
    fail(`scrolled to ${box.scrollTop}, the copy shows ${at.y} — ` +
         `${box.scrollTop - at.y}px of text out from under the caret`);
    break;
  }
}
for (const want of [0, 13, box.scrollWidth]) {
  box.scrollLeft = want;
  box.dispatchEvent(new Event('scroll'));
  const at = shown();
  if (at.x !== box.scrollLeft) {
    fail(`scrolled across to ${box.scrollLeft}, the copy shows ${at.x}`);
    break;
  }
}
say(`scrolling   followed to the top, the bottom and the far right`);

mirror.remove();
say('');
say(bad ? `${bad} PROBLEM(S)` : 'aligned');
document.getElementById('out').textContent = out.join('\n');
