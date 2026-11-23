/* Keep authored example drawings, their model dependencies — the exchange files a `use` reads a
 * body from among them (#993 stage 2) — and the text beside them (a README) available on a static
 * host.  Every page that opens an example fetches this one file whole, so an exchange file packed
 * here costs its whole size on each of those openings: keep the examples' own small.  Read as
 * UTF-8, lossily, as the page reads a file it fetches and solventc reads an exchange file: its
 * geometry is ASCII, and only its strings may carry other bytes. */
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
// the project's file kinds, the app's one list (`tsc` has written it before this runs)
import { projectFile } from '../dist/app/project-explorer.js';

const web = join(dirname(fileURLToPath(import.meta.url)), '..');
const root = join(web, '..', 'rust', 'examples');
const files = {};
function collect(path = '') {
  for (const entry of readdirSync(join(root, path), { withFileTypes: true })) {
    const name = path ? `${path}/${entry.name}` : entry.name;
    if (entry.isDirectory()) collect(name);
    else if (projectFile(name)) {
      files[name] = readFileSync(join(root, name), 'utf8');
    }
  }
}
collect();
mkdirSync(join(web, 'dist/examples'), { recursive: true });
writeFileSync(join(web, 'dist/examples/sources.json'), JSON.stringify(files));
