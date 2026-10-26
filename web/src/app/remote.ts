/* Authored example drawings, fresh from the demo server with packaged sources as the
 * static-host fallback. The browser gathers texts; the core parses, solves, and renders. */
import * as examples from '../core/examples.js';
import * as modules from '../core/modules.js';
import { info, type DrawingBundle } from '../core/drawing.js';
import { fromSketch } from '../core/program.js';
import { exampleTarget } from './example-catalog.js';
import { projectFile } from './project-explorer.js';

/** `source` is the file selected, which may be a text file; `entry` is the drawing or model the
 *  project opens on, which a text file is shown beside. */
export interface ExampleBundle extends DrawingBundle {
  key: string; directory?: string; entry: string;
}

async function fetchText(path: string): Promise<string | null> {
  try {
    const r = await fetch(path, { cache: 'no-store' });
    return r.ok ? await r.text() : null;
  } catch {
    return null;
  }
}

/** What the host has of the examples: the sources packed into the build, and the examples'
 *  own listing where it serves them — the demo server lists them at `examples/index.json`, and
 *  only a host that does is asked for a file.  A static host (GitHub Pages) has neither the list
 *  nor the files, and asking it for each would be a refusal per file, one after another down the
 *  chain of uses. */
interface Host { packed: string | null; index: string | null }

const host = async (): Promise<Host> => {
  const [packed, index] = await Promise.all(
    [fetchText('dist/examples/sources.json'), fetchText('examples/index.json')]);
  return { packed, index };
};

/** Asked for while the core is still arriving, by a page that opens on an example: the one
 *  opening after it takes it, and any later one asks again. */
let early: Promise<Host> | null = null;
export function prefetch(): void { early ??= host(); }

/** Main authored drawing and the files it reads. Fresh files on the demo server override
 *  the source bundle shipped with static builds. Geometry parsing stays in the core. */
export async function drawing(key: string, selectedFile?: string): Promise<ExampleBundle> {
  const entry = exampleTarget(key);
  const target = entry.target;
  const stem = key.split(':')[0];
  const asking = early ?? host();
  early = null;
  const { packed, index } = await asking;
  const served = index !== null;
  const fallback: Record<string, string> = packed === null ? {} : JSON.parse(packed);
  const files: Record<string, string> = {};
  const asked = new Set<string>();
  const visitedModels = new Set<string>(), visitedDrawings = new Set<string>();
  const directory = target.kind === 'directory' ? target.path : undefined;
  const main = target.kind === 'directory' ? `${target.path}/${target.entry}` : target.path;
  const resolve = (path: string, from: string) =>
    decodeURIComponent(new URL(path, `https://example.invalid/${from}`).pathname.slice(1));
  const read = async (path: string): Promise<string | null> => {
    if (asked.has(path)) return files[path] ?? null;
    asked.add(path);
    const text = (served ? await fetchText(`examples/${path}`) : null) ?? fallback[path] ?? null;
    if (text !== null) files[path] = text;
    return text;
  };
  const model = async (path: string, root: string): Promise<void> => {
    const id = `${root}\0${path}`;
    if (visitedModels.has(id)) return;
    visitedModels.add(id);
    const text = await read(path);
    if (text === null) return;  // The core can supply a standard library or diagnose a missing file.
    for (const name of modules.uses(text)) {
      for (const next of modules.searchPaths(name, root)) {
        if (await read(next) !== null) {
          await model(next, root);
          break;
        }
      }
    }
  };
  const visit = async (path: string): Promise<void> => {
    if (visitedDrawings.has(path)) return;
    visitedDrawings.add(path);
    const text = await read(path);
    if (text === null) throw new Error(`cannot load example drawing: ${path}`);
    const doc = info(text);
    for (const imported of doc.imports) {
      const next = resolve(imported, path);
      await visit(next);
    }
    for (const imported of doc.models) {
      const next = resolve(imported, path);
      if (key.includes(':') && path === main && next === `${stem}.sv`) {
        const generated = examples.build(key);
        try { files[next] = fromSketch(generated); }
        finally { generated.dispose(); }
        asked.add(next);
      }
      await model(next, next);
    }
  };
  if (directory) {
    const paths: string[] = index === null ? Object.keys(fallback) : JSON.parse(index);
    const members = paths.filter((p) => p.startsWith(`${directory}/`) && projectFile(p));
    await Promise.all(members.map(read));
    // Read all files, including part drawings unreachable from the assembly. Follow their
    // external dependencies too, while the core remains responsible for resolving names.
    for (const path of members.filter((p) => p.endsWith('.svd'))) await visit(path);
  }
  if (main.endsWith('.svd')) await visit(main);
  else {
    await model(main, main);
    if (!(main in files)) throw new Error(`cannot load example file: ${main}`);
  }
  const selected = selectedFile ?? entry.file;
  return { source: selected && selected in files ? selected : main, files,
    key: entry.key, directory, entry: main };
}
