/* A case's program, fresh from the server when there is one.
 *
 * The page is static files and needs no server.  But the dev server (`npm run serve`) also hands
 * out `rust/examples/` as `examples/…`, and a case read from there is the file on disk — so
 * editing a document and refreshing the page shows the edit with no wasm rebuilt, and a file that
 * is not in the case library at all opens by URL (`?example=<name>`).  The modules it `use`s come
 * the same way (`core/modules`), and anything the server has not got falls to the copies compiled
 * into the core — which is also what happens with no server at all, since then nothing fetches
 * and the caller reads the compiled-in case. */
import * as examples from '../core/examples.js';
import * as modules from '../core/modules.js';

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

/** The case's program: the server's file when it has one — its modules handed to the core first —
 *  else the copy compiled into the core, linked against the library alone. */
export async function source(key: string): Promise<string> {
  const fresh = fileKey(key) ? await fetchText(`examples/${key}.sv`) : null;
  modules.forget();
  if (fresh === null) return examples.source(key);
  await modules.link(fresh, (p) => fetchText(`examples/${p}`));
  return fresh;
}
