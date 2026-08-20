/* Copy the WebAssembly module next to the compiled modules: dist/core/wasm.js resolves
 * ../wasm/gcs.wasm relative to itself, and tsc does not emit files it did not compile. */
import { copyFileSync, mkdirSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

import { existsSync } from 'node:fs';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const src = join(root, 'src/wasm/gcs.wasm');
if (!existsSync(src)) {
  console.error('web/src/wasm/gcs.wasm is missing — build the core first:\n    make wasm');
  process.exit(1);
}
mkdirSync(join(root, 'dist/wasm'), { recursive: true });
copyFileSync(src, join(root, 'dist/wasm/gcs.wasm'));
