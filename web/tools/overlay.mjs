/* Does the program panel's coloured copy line up with the box somebody types in?
 *
 * `app/editor.ts` draws the source twice — a `<pre>` carrying the colour and a textarea in front
 * taking the typing with its own glyphs invisible — so the two have to put every character in
 * exactly the same place.  That is a *rendering* property: no unit test can see it, because it
 * depends on what a font does with the text, and it goes wrong in ways that look like the caret
 * wandering off the words.
 *
 * So this drives a real browser.  It asks the core for the runs, builds the same control the app
 * builds, and does what a person does: point at a glyph in the coloured copy and ask the box in
 * front which character is there.
 *
 *     node tools/overlay.mjs            # the gear, and a page of awkward cases
 *     node tools/overlay.mjs --keep     # leave the page behind, to open in a real browser
 *
 * The page is bundled with esbuild and loaded from `file://`: headless Chrome will not load an
 * ES module from a file URL, and serving the directory instead makes it hang on this platform.
 * Chrome is found where macOS puts it, or through $CHROME.
 */
import { execFileSync } from 'node:child_process';
import { mkdtempSync, rmSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';

import { initCore } from '../dist/core/wasm.js';
import { highlight } from '../dist/core/program.js';
import * as examples from '../dist/core/examples.js';

const here = dirname(fileURLToPath(import.meta.url));
const root = () => join(here, '..');
const CHROME = process.env.CHROME ||
  '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome';

/** The sources worth checking.  Each is a way the two layers could part company. */
function cases() {
  return [
    // the reported case: a long document, whose end is only reached by scrolling
    ['the gear', examples.source('gear')],
    // a run boundary mid-line, so kerning and ligatures cannot form across it in one layer
    ['tight punctuation', [
      'point p at (0,0)',
      'radius(c)==4',
      'distance(a,b)==12.5  // a comment right up against it',
      'param w = a*b/(c+d)-e',
      '',
    ].join('\n')],
    // characters a monospace face may not have, which fall back to another font
    ['not all ascii', [
      '// an em dash — and a section §, a pi π, a square ², a times ×',
      'point p at (0, 0)   // — — — — — — — — — —',
      'radius(c) == 3      // π² ×',
      'line l(p, q)        // the last line, after everything above it',
      '',
    ].join('\n')],
    // long lines, so the box carries a horizontal scrollbar and the copy does not
    ['long lines', [
      `// ${'x'.repeat(200)}`,
      `point ${'p'.repeat(60)} at (0, 0)`,
      'line l(a, b)',
      '',
    ].join('\n')],
  ];
}

await initCore();

const profile = mkdtempSync(join(tmpdir(), 'overlay-profile-'));
const here_case = join(here, '.overlay-case.js');
const here_page = join(here, '.overlay-page.js');

// the measuring half, as one classic script a `file://` page may load
execFileSync('npx', ['esbuild', join(here, 'overlay-page.js'), '--bundle', '--format=iife',
                     `--outfile=${here_page}`, '--log-level=warning'],
             { cwd: root(), stdio: 'inherit' });

const ARGS = [
  '--headless', '--disable-gpu', '--no-sandbox', '--disable-extensions',
  '--no-first-run', '--no-default-browser-check', `--user-data-dir=${profile}`,
  '--window-size=420,1000', '--virtual-time-budget=5000', '--dump-dom',
  `file://${join(here, 'overlay.html')}`,
];
const OPTS = { encoding: 'utf8', stdio: ['ignore', 'pipe', 'ignore'], maxBuffer: 64 << 20,
               timeout: 20_000, killSignal: 'SIGKILL' };

let failed = 0;
try {
  for (const [name, text] of cases()) {
    // a lit range over the middle of the document, so the mark the panel puts on a picked
    // statement is under the same test as the colouring is
    const lit = { lo: Math.floor(text.length / 3), hi: Math.floor((text.length * 2) / 3) };
    const CASE = { text, runs: highlight(text), lit };
    writeFileSync(here_case, `window.CASE = ${JSON.stringify(CASE)};\n`);
    // Chrome prints the DOM and then does not always exit, so the timeout is the normal path
    // and what it collected on the way is the answer.
    let dom = '';
    try {
      dom = execFileSync(CHROME, ARGS, OPTS);
    } catch (e) {
      dom = typeof e.stdout === 'string' ? e.stdout : (e.stdout?.toString('utf8') ?? '');
      if (!dom) throw e;
    }
    const body = /<pre id="out">([\s\S]*?)<\/pre>/.exec(dom);
    const report = body
      ? body[1].replace(/&lt;/g, '<').replace(/&gt;/g, '>').replace(/&quot;/g, '"')
          .replace(/&#39;/g, "'").replace(/&amp;/g, '&')
      : '(the page produced no report — did it fail to load?)';
    const ok = /\naligned$/.test(report.trim()) || report.trim().endsWith('aligned');
    if (!ok) failed++;
    console.log(`\n=== ${name} — ${ok ? 'aligned' : 'MISALIGNED'} ===`);
    console.log(report.trim());
  }
} finally {
  rmSync(profile, { recursive: true, force: true });
  if (!process.argv.includes('--keep')) {
    rmSync(here_case, { force: true });
    rmSync(here_page, { force: true });
  }
}

console.log(failed ? `\n${failed} case(s) misaligned` : '\nevery case aligned');
process.exit(failed ? 1 : 0);
