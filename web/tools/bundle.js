/* Roll the page and its two workers each into one module beside its entry (`dist/app/bundle.js`,
 * `mesh-worker.bundle.js`, `export-worker.bundle.js`), after `tsc` and `copy-wasm.js`.
 *
 * The core's size is written in as `GCS_WASM_BYTES`, so the page can say how much of the module
 * has arrived: a host that compresses it (GitHub Pages) sends the compressed size as the
 * Content-Length, and the body yields the module itself.  The node-only fallback imports in
 * `core/wasm.ts` are left external. */
import { build } from 'esbuild';
import { statSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const bytes = statSync(join(root, 'dist/wasm/gcs.wasm')).size;

for (const [entry, out] of [['main', 'bundle'], ['mesh-worker', 'mesh-worker.bundle'],
                            ['export-worker', 'export-worker.bundle']]) {
  await build({
    absWorkingDir: root,
    entryPoints: [`dist/app/${entry}.js`],
    outfile: `dist/app/${out}.js`,
    bundle: true,
    format: 'esm',
    target: 'es2022',
    sourcemap: true,
    external: ['node:fs/promises', 'node:url'],
    define: { GCS_WASM_BYTES: String(bytes) },
    logLevel: 'info',
  });
}
