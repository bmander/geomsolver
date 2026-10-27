/* **What opening an assembly costs the page** (#669), beside what `solventc` takes to export it.
 *
 *     node tools/open-bench.mjs                      the case below, in the page and natively
 *     node tools/open-bench.mjs hub_motor            another example: a catalog key or a path
 *                                                    under rust/examples (`vtwin/assembly.sv`)
 *     node tools/open-bench.mjs --no-native          the page alone
 *     node tools/open-bench.mjs --json out.json      the numbers as JSON as well
 *     node tools/open-bench.mjs --timeout 600        seconds to wait for every object (default 900)
 *
 * The page is the built app (`npm run build`), served as `npm run serve` serves it and opened in
 * headless Chromium by Playwright at `?example=…`, as a person opening the example does.  The app
 * marks what it does on the User Timing line (`performance.mark`, `gcs:*`): the reading done
 * (`gcs:read`), each static object built on the page's thread (`gcs:built`), the modal put away
 * (`gcs:loaded`), each object handed to three.js and rendered (`gcs:drawn`), each swept object's
 * surface applied, field or exact (`gcs:surface`), and an exact build refused (`gcs:unbuilt`).
 * Read here are:
 *
 *   * the times, from navigation: to the modal put away, to the first object drawn, and to each
 *     object drawn in its final form (a static object's first drawing; a swept one's exact
 *     surface, or its field's finished one where the exact build is refused);
 *   * the main thread's long tasks (`PerformanceObserver`, `longtask`): the longest, their sum,
 *     and the total blocking time (what each holds past 50 ms);
 *   * `renderer.info` once settled: draw calls, triangles, geometries, programs, and the
 *     triangles and meshes each object holds;
 *   * the peak JS heap of the page and of each worker, sampled through the DevTools protocol
 *     (`Runtime.getHeapUsage`: the V8 heap, and the backing stores beside it, where a wasm
 *     core's memory is);
 *   * and natively, `build/solventc --no-check FILE --step-assembly …` timed: wall, and each
 *     stage `--progress` says.
 *
 * **Headless Chromium draws WebGL in software** (SwiftShader): the draw-call, triangle and
 * geometry counts are the page's, the times of anything the GPU does are not a GPU's.  Read the
 * times beside each other, on a quiet machine (`uptime`).
 *
 * Playwright is not a dependency of the project.  It is found as Node finds a package required
 * from here, `NODE_PATH` included (`playwright`, else `playwright-core`), and drives the browser
 * it installed, or `$CHROME`.  Without it this says so and exits 0, so `make bench` runs the
 * other two benchmarks regardless. */
import { spawn, spawnSync } from 'node:child_process';
import { existsSync, mkdtempSync, rmSync, statSync, writeFileSync } from 'node:fs';
import { createRequire } from 'node:module';
import { createServer } from 'node:net';
import { tmpdir } from 'node:os';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

const web = join(dirname(fileURLToPath(import.meta.url)), '..');
const repo = join(web, '..');

/** The case: the largest assembly in `rust/examples/` (see docs/build-performance.md). */
const CASE = 'gear_train/train.sv';

function usage(message) {
  if (message) console.error(`open-bench: ${message}`);
  console.error('usage: node tools/open-bench.mjs [EXAMPLE] [--no-native] [--json PATH] [--timeout S]');
  process.exit(2);
}

const opts = { example: CASE, native: true, json: null, timeout: 900 };
{
  const args = process.argv.slice(2);
  for (let i = 0; i < args.length; i++) {
    const a = args[i];
    if (a === '--no-native') opts.native = false;
    else if (a === '--json') opts.json = args[++i] ?? usage('--json needs a path');
    else if (a === '--timeout') opts.timeout = Number(args[++i] ?? usage('--timeout needs seconds'));
    else if (a.startsWith('-')) usage(`unknown option ${a}`);
    else opts.example = a;
  }
  if (!(opts.timeout > 0)) usage('--timeout needs a number of seconds');
}

function playwright() {
  const require = createRequire(import.meta.url);
  for (const name of ['playwright', 'playwright-core']) {
    try { return require(name); } catch { /* the next */ }
  }
  return null;
}

/** A port nothing is listening on. */
function freePort() {
  return new Promise((then, fail) => {
    const s = createServer();
    s.once('error', fail);
    s.listen(0, '127.0.0.1', () => { const { port } = s.address(); s.close(() => then(port)); });
  });
}

/** `tools/serve.js` on `port`, once it says it is listening. */
function serve(port) {
  return new Promise((then, fail) => {
    const child = spawn(process.execPath, [join(web, 'tools', 'serve.js')],
      { cwd: web, env: { ...process.env, PORT: String(port) }, stdio: ['ignore', 'pipe', 'inherit'] });
    child.once('error', fail);
    child.once('exit', (code) => fail(new Error(`the server stopped (${code})`)));
    child.stdout.on('data', (d) => { if (String(d).includes('http://')) then(child); });
  });
}

const ms = (t) => (t === undefined || t === null ? '—' : t >= 10000 ? `${(t / 1000).toFixed(1)} s`
  : t >= 1000 ? `${(t / 1000).toFixed(2)} s` : `${Math.round(t)} ms`);
const mb = (b) => (b === undefined ? '—' : `${(b / 2 ** 20).toFixed(1)} MB`);

/** **The page's heaps, sampled.**  The page's own through its CDP session; each worker's through
 *  the same session's auto-attach, unflattened, so this session's messages to it are its own and
 *  do not disturb Playwright's.  The peak of each: V8's used heap, and the backing stores (array
 *  buffers, a wasm instance's memory) where this Chromium reports them. */
async function heapSampler(cdp) {
  const peaks = new Map();       // label → { used, backing }
  const workers = new Map();     // sessionId → { label, pending: Map<id, resolve> }
  let next = 1, count = 0;
  const note = (label, u) => {
    const p = peaks.get(label) ?? { used: 0, backing: 0 };
    p.used = Math.max(p.used, u.usedSize ?? 0);
    p.backing = Math.max(p.backing, u.backingStorageSize ?? 0);
    peaks.set(label, p);
  };
  cdp.on('Target.attachedToTarget', (e) => {
    if (e.targetInfo.type !== 'worker') return;
    const url = e.targetInfo.url ?? '';
    const kind = url.includes('mesh-worker') ? 'mesh worker' : url.includes('export-worker')
      ? 'export worker' : url.includes('claim') ? 'claims worker' : 'worker';
    workers.set(e.sessionId, { label: `${kind} ${++count}`, pending: new Map() });
    // the worker paused for nothing: auto-attach was asked not to wait
  });
  cdp.on('Target.detachedFromTarget', (e) => workers.delete(e.sessionId));
  cdp.on('Target.receivedMessageFromTarget', (e) => {
    const w = workers.get(e.sessionId);
    if (!w) return;
    const m = JSON.parse(e.message);
    const done = w.pending.get(m.id);
    if (done) { w.pending.delete(m.id); done(m.result); }
  });
  await cdp.send('Target.setAutoAttach', { autoAttach: true, waitForDebuggerOnStart: false, flatten: false });
  const ask = (sessionId, w) => new Promise((then) => {
    const id = next++;
    w.pending.set(id, then);
    cdp.send('Target.sendMessageToTarget', { sessionId, message: JSON.stringify({ id, method: 'Runtime.getHeapUsage' }) })
      .catch(() => { w.pending.delete(id); then(null); });
    setTimeout(() => { if (w.pending.delete(id)) then(null); }, 2000);
  });
  let running = true;
  const loop = (async () => {
    while (running) {
      try { note('page', await cdp.send('Runtime.getHeapUsage')); } catch { /* navigating */ }
      await Promise.all([...workers].map(async ([sid, w]) => {
        const u = await ask(sid, w);
        if (u) note(w.label, u);
      }));
      await new Promise((r) => setTimeout(r, 250));
    }
  })();
  return { peaks, stop: async () => { running = false; await loop; } };
}

/** Open the example in the page and wait for it to settle; what it measured. */
async function page(pw) {
  const port = await freePort();
  const server = await serve(port);
  const browser = await pw.chromium.launch({
    executablePath: process.env.CHROME || undefined,
    // WebGL in software where there is no GPU, which headless is
    args: ['--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'],
  });
  try {
    const context = await browser.newContext({ viewport: { width: 1600, height: 1000 } });
    const p = await context.newPage();
    const errors = [];
    p.on('pageerror', (e) => errors.push(String(e)));
    p.on('console', (m) => { if (m.type() === 'error') errors.push(m.text()); });
    // the long tasks from the very start: the reading is one
    await p.addInitScript(() => {
      const w = /** @type {any} */ (window);
      w.__longtasks = [];
      new PerformanceObserver((list) => {
        for (const e of list.getEntries()) w.__longtasks.push({ start: e.startTime, duration: e.duration });
      }).observe({ type: 'longtask', buffered: true });
    });
    const cdp = await context.newCDPSession(p);
    const heaps = await heapSampler(cdp);
    const url = `http://127.0.0.1:${port}/?example=${encodeURIComponent(opts.example)}`;
    const started = Date.now();
    await p.goto(url, { waitUntil: 'load' });
    // settled: the modal away, every swept object's exact surface (or its refusal and the
    // field's finished one) applied, and nothing new drawn for a while
    const QUIET_MS = 3000;
    let settled = false;
    let lastDrawn = -1, quietSince = Date.now();
    while (Date.now() - started < opts.timeout * 1000) {
      const s = await p.evaluate(() => {
        const marks = performance.getEntriesByType('mark').filter((m) => m.name.startsWith('gcs:'));
        const g = /** @type {any} */ (window).gcsBench;
        const refining = g ? g.refining() : [];
        return {
          loaded: marks.some((m) => m.name === 'gcs:loaded'),
          drawn: marks.filter((m) => m.name === 'gcs:drawn').length,
          open: refining.filter((r) => !(r.exact?.built || ((r.exact?.error || !r.exact) && (r.done || r.error)))).length,
        };
      });
      if (s.drawn !== lastDrawn) { lastDrawn = s.drawn; quietSince = Date.now(); }
      if (s.loaded && !s.open && Date.now() - quietSince > QUIET_MS) { settled = true; break; }
      await new Promise((r) => setTimeout(r, 250));
    }
    await heaps.stop();
    const got = await p.evaluate(() => {
      const w = /** @type {any} */ (window);
      const marks = performance.getEntriesByType('mark').filter((m) => m.name.startsWith('gcs:'))
        .map((m) => ({ name: m.name, at: m.startTime, detail: /** @type {any} */ (m).detail ?? null }));
      return { marks, longtasks: w.__longtasks ?? [], counts: w.gcsBench?.counts() ?? null,
               refining: w.gcsBench?.refining() ?? [] };
    });
    return { ...got, settled, heaps: Object.fromEntries(heaps.peaks), errors };
  } finally {
    await browser.close();
    server.kill();
  }
}

/** The page's numbers, from its marks. */
function summarize(r) {
  const at = (name) => r.marks.find((m) => m.name === name)?.at;
  const drawn = r.marks.filter((m) => m.name === 'gcs:drawn');
  const surfaces = r.marks.filter((m) => m.name === 'gcs:surface');
  const unbuilt = new Map(r.marks.filter((m) => m.name === 'gcs:unbuilt').map((m) => [m.detail.name, m.detail.error]));
  const swept = new Set(r.refining.map((x) => x.name));
  // each object: its first drawing, and the first drawing in its final form
  const objects = new Map();
  for (const d of drawn) {
    const { name, triangles } = d.detail;
    let o = objects.get(name);
    if (!o) { o = { name, first: d.at, final: undefined, triangles, swept: swept.has(name) }; objects.set(name, o); }
    if (o.final !== undefined) continue;
    if (!o.swept) { o.final = d.at; o.triangles = triangles; continue; }
    // a swept object is final once drawn after the surface that is its last word
    const last = surfaces.filter((s) => s.detail.name === name && s.at <= d.at
      && (s.detail.exact || (unbuilt.has(name) && !s.detail.provisional)));
    if (last.length) { o.final = d.at; o.triangles = triangles; o.exact = last.at(-1).detail.exact; }
  }
  for (const o of objects.values()) if (o.swept && unbuilt.has(o.name)) o.refused = unbuilt.get(o.name);
  const built = r.marks.filter((m) => m.name === 'gcs:built').map((m) => m.detail);
  const tasks = r.longtasks;
  const longest = tasks.reduce((a, t) => Math.max(a, t.duration), 0);
  const total = tasks.reduce((a, t) => a + t.duration, 0);
  const blocking = tasks.reduce((a, t) => a + Math.max(0, t.duration - 50), 0);
  const finals = [...objects.values()].map((o) => o.final).filter((t) => t !== undefined);
  return {
    read: at('gcs:read'),
    loaded: at('gcs:loaded'),
    firstDrawn: drawn.length ? Math.min(...drawn.map((d) => d.at)) : undefined,
    allFinal: finals.length === objects.size && finals.length ? Math.max(...finals) : undefined,
    objects: [...objects.values()].sort((a, b) => (a.final ?? Infinity) - (b.final ?? Infinity)),
    built,
    longtasks: { count: tasks.length, longest, total, blocking },
    counts: r.counts,
  };
}

/** `solventc --no-check FILE --step-assembly …`, timed: wall, the STEP's size, and each stage
 *  as `--progress` says it. */
function native() {
  const solventc = join(repo, 'build', 'solventc');
  if (!existsSync(solventc)) return { skipped: 'no build/solventc (make solventc)' };
  const file = join(repo, 'rust', 'examples', opts.example);
  if (!existsSync(file)) return { skipped: `${opts.example} is not a file under rust/examples` };
  const dir = mkdtempSync(join(tmpdir(), 'open-bench-'));
  const out = join(dir, 'assembly.step');
  try {
    const t0 = performance.now();
    const r = spawnSync(solventc, ['--no-check', '--progress', file, '--step-assembly', out],
                        { stdio: ['ignore', 'ignore', 'pipe'], maxBuffer: 1 << 28, encoding: 'utf8' });
    const wall = performance.now() - t0;
    const stages = (r.stderr ?? '').split('\n').filter((l) => l.trim());
    if (r.status !== 0) return { failed: stages.slice(-3).join('\n') || `status ${r.status}`, wall };
    return { wall, bytes: existsSync(out) ? statSync(out).size : undefined, stages };
  } finally {
    rmSync(dir, { recursive: true, force: true });
  }
}

const pw = playwright();
if (!pw) {
  console.log('open-bench: skipped — Playwright is not installed where Node can find it '
            + '(`npm i -g playwright`, or NODE_PATH): the page is not measured');
  process.exit(0);
}
if (!existsSync(join(web, 'dist', 'app', 'bundle.js'))) usage('build the app first (npm run build)');

console.log(`open-bench: ${opts.example}`);
const raw = await page(pw);
const s = summarize(raw);
const n = opts.native ? native() : { skipped: '--no-native' };

const rows = [];
rows.push(['reading done (elaborate, solve, diagnose)', ms(s.read)]);
rows.push(['modal put away (loaded)', ms(s.loaded)]);
rows.push(['first object drawn', ms(s.firstDrawn)]);
rows.push([`every object drawn final (${s.objects.filter((o) => o.final !== undefined).length} of ${s.objects.length})`,
           ms(s.allFinal)]);
console.log('\npage (from navigation)');
for (const [k, v] of rows) console.log(`  ${k.padEnd(48)} ${v.padStart(9)}`);
console.log('\nmain thread');
console.log(`  ${'long tasks'.padEnd(48)} ${String(s.longtasks.count).padStart(9)}`);
console.log(`  ${'longest'.padEnd(48)} ${ms(s.longtasks.longest).padStart(9)}`);
console.log(`  ${'held in long tasks, in all'.padEnd(48)} ${ms(s.longtasks.total).padStart(9)}`);
console.log(`  ${'total blocking time (past 50 ms each)'.padEnd(48)} ${ms(s.longtasks.blocking).padStart(9)}`);
if (s.counts) {
  const c = s.counts;
  console.log('\nrenderer.info, settled');
  for (const [k, v] of [['draw calls', c.calls], ['triangles', c.triangles], ['lines', c.lines],
                        ['geometries', c.geometries], ['programs', c.programs],
                        ['meshes in objects', c.objects.reduce((a, o) => a + o.meshes, 0)]]) {
    console.log(`  ${k.padEnd(48)} ${String(v).padStart(9)}`);
  }
}
console.log('\npeak heap (V8 used + backing stores)');
for (const [k, v] of Object.entries(raw.heaps)) {
  console.log(`  ${k.padEnd(48)} ${mb(v.used).padStart(9)} + ${mb(v.backing)}`);
}
console.log('\nobjects, in the order they were final');
const meshesOf = new Map((s.counts?.objects ?? []).map((o) => [o.index, o]));
const indexOf = new Map(raw.marks.filter((m) => m.name === 'gcs:drawn').map((m) => [m.detail.name, m.detail.index]));
const builtOf = new Map(s.built.map((b) => [b.name, b.ms]));
console.log(`  ${'object'.padEnd(40)} ${'first'.padStart(9)} ${'final'.padStart(9)} ${'built'.padStart(9)} ${'triangles'.padStart(10)} ${'meshes'.padStart(7)}`);
for (const o of s.objects) {
  const m = meshesOf.get(indexOf.get(o.name));
  const how = o.swept ? (o.refused ? ' (field: exact refused)' : ' (swept)') : '';
  console.log(`  ${(o.name + how).padEnd(40)} ${ms(o.first).padStart(9)} ${ms(o.final).padStart(9)} `
            + `${ms(builtOf.get(o.name)).padStart(9)} ${String(m?.triangles ?? o.triangles).padStart(10)} `
            + `${String(m?.meshes ?? '—').padStart(7)}`);
}
console.log('\nsolventc --no-check --step-assembly (native)');
if (n.skipped) console.log(`  skipped: ${n.skipped}`);
else if (n.failed) console.log(`  failed after ${ms(n.wall)}: ${n.failed}`);
else {
  console.log(`  ${'wall'.padEnd(48)} ${ms(n.wall).padStart(9)}`);
  console.log(`  ${'STEP written'.padEnd(48)} ${mb(n.bytes).padStart(9)}`);
  console.log('  stages (--progress):');
  for (const l of n.stages) console.log(`    ${l}`);
}
if (!raw.settled) console.log(`\nopen-bench: not settled within ${opts.timeout} s; the numbers are as they stood`);
if (raw.errors.length) console.log(`\npage errors:\n  ${raw.errors.slice(0, 5).join('\n  ')}`);
if (opts.json) writeFileSync(opts.json, JSON.stringify({ example: opts.example, page: s, heaps: raw.heaps,
  native: n, settled: raw.settled, errors: raw.errors }, null, 2));
