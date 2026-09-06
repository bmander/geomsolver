/* Keep authored example drawings and their model dependencies available on a static host. */
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

const web = join(dirname(fileURLToPath(import.meta.url)), '..');
const root = join(web, '..', 'rust', 'examples');
const files = {};
function collect(path = '') {
  for (const entry of readdirSync(join(root, path), { withFileTypes: true })) {
    const name = path ? `${path}/${entry.name}` : entry.name;
    if (entry.isDirectory()) collect(name);
    else if (/\.svd?$/.test(name)) files[name] = readFileSync(join(root, name), 'utf8');
  }
}
collect();
mkdirSync(join(web, 'dist/examples'), { recursive: true });
writeFileSync(join(web, 'dist/examples/sources.json'), JSON.stringify(files));
