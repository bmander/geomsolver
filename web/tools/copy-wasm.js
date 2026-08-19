/* Copy the emscripten artifacts next to the compiled modules: dist/core/wasm.js imports
 * ../wasm/gcs.js, and tsc does not emit files it did not compile. */
import { copyFileSync, mkdirSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
mkdirSync(join(root, 'dist/wasm'), { recursive: true });
for (const f of ['gcs.js', 'gcs.wasm']) {
  copyFileSync(join(root, 'src/wasm', f), join(root, 'dist/wasm', f));
}
