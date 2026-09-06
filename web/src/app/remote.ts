/* Authored example drawings, fresh from the demo server with packaged sources as the
 * static-host fallback. The browser gathers texts; the core parses, solves, and renders. */
import * as examples from '../core/examples.js';
import * as modules from '../core/modules.js';
import { info, type DrawingBundle } from '../core/drawing.js';
import { fromSketch } from '../core/program.js';

async function fetchText(path: string): Promise<string | null> {
  try {
    const r = await fetch(path, { cache: 'no-store' });
    return r.ok ? await r.text() : null;
  } catch {
    return null;
  }
}

/** A key that could name a file: a case's plain name.  One with arguments (`truss:50`) names a
 *  function built in the core, never a file, and nothing with a path separator is asked for. */
function fileKey(key: string): boolean {
  return /^[\w-]+$/.test(key);
}

/** Main authored drawing and the files it reads. Fresh files on the demo server override
 *  the source bundle shipped with static builds. Geometry parsing stays in the core. */
export async function drawing(key: string): Promise<DrawingBundle> {
  const stem = key.split(':')[0];
  if (!fileKey(stem)) throw new Error(`invalid example: ${key}`);
  const packed = await fetchText('dist/examples/sources.json');
  const fallback: Record<string, string> = packed === null ? {} : JSON.parse(packed);
  const files: Record<string, string> = {};
  const asked = new Set<string>();
  const main = `${stem}.svd`;
  const resolve = (path: string, from: string) =>
    decodeURIComponent(new URL(path, `https://example.invalid/${from}`).pathname.slice(1));
  const read = async (path: string): Promise<string | null> => {
    if (asked.has(path)) return files[path] ?? null;
    asked.add(path);
    const text = await fetchText(`examples/${path}`) ?? fallback[path] ?? null;
    if (text !== null) files[path] = text;
    return text;
  };
  const model = async (path: string, root: string): Promise<void> => {
    const text = await read(path);
    if (text === null) return;  // The core can supply a standard library or diagnose a missing file.
    for (const name of modules.uses(text)) {
      const next = resolve(modules.pathOf(name), root);
      if (!asked.has(next)) await model(next, root);
    }
  };
  const visit = async (path: string): Promise<void> => {
    const text = await read(path);
    if (text === null) throw new Error(`cannot load example drawing: ${path}`);
    const doc = info(text);
    for (const imported of doc.imports) {
      const next = resolve(imported, path);
      if (!asked.has(next)) await visit(next);
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
  await visit(main);
  return { source: main, files };
}
