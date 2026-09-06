/* Modules a document `use`s, handed to the core by the host.
 *
 * The core takes text and has no filesystem: in the terminal `solventc` reads `engine.parts` as
 * `engine/parts.sv` beside the document and comes to the library compiled in second.  A browser
 * has no beside — unless its page has a server to ask.  `link` is that: it asks the core which
 * modules a text uses, fetches each through the function it is given, hands the text over, and
 * follows the fetched texts' own `use`s the same way.  What the host has not got is left to the
 * library, so a document of the compiled-in kind links as it always did.  No resolution happens
 * here: the core links, and the same way it links for the CLI. */
import { core, takeJson, takeStr, withStr } from './wasm.js';

/** The effective source, including modules supplied by the host and shared libraries. */
export function source(name: string): string | null {
  const p = withStr(name, (np, nn) => core().gcs_module_source(np, nn));
  return p ? takeStr(p) : null;
}

export interface SourceFile { name: string; path: string; text: string }

/** All imports reachable from a document, once each, in dependency traversal order. */
export function related(text: string): SourceFile[] {
  const files: SourceFile[] = [];
  const seen = new Set<string>();
  const queue = uses(text);
  for (let i = 0; i < queue.length; i++) {
    const name = queue[i];
    if (seen.has(name)) continue;
    seen.add(name);
    const text = source(name);
    if (text === null) continue;
    files.push({ name, path: pathOf(name), text });
    queue.push(...uses(text));
  }
  return files;
}

/** The module names a text `use`s directly, as written (`engine.parts`). */
export function uses(text: string): string[] {
  return withStr(text, (p, n) => takeJson<string[]>(core().gcs_program_uses(p, n))) ?? [];
}

/** Hand the core a module's text under its `use` name; it outranks the library's copy. */
export function provide(name: string, text: string): void {
  withStr(name, (np, nn) => withStr(text, (tp, tn) => core().gcs_module_set(np, nn, tp, tn)));
}

/** Forget everything handed over: the next program read links against the library alone. */
export function forget(): void {
  core().gcs_module_forget();
}

/** A `use` name as the path beside the document: `engine.parts` is `engine/parts.sv`. */
export function pathOf(name: string): string {
  return name.split('.').join('/') + '.sv';
}

/** Project paths beside a root model, then its ancestors, nearest first. */
export function searchPaths(name: string, root: string): string[] {
  const directories = root.split('/').slice(0, -1);
  return Array.from({ length: directories.length + 1 }, (_, i) =>
    [...directories.slice(0, directories.length - i), pathOf(name)].join('/'));
}

/** Provide current project texts when opening a model or an individual component. */
export function provideProject(root: string, files: Record<string, string>): void {
  forget();
  const seen = new Set<string>(), queue = uses(files[root] ?? '');
  for (let i = 0; i < queue.length; i++) {
    const name = queue[i];
    if (seen.has(name)) continue;
    seen.add(name);
    const path = searchPaths(name, root).find((candidate) => candidate in files);
    if (path !== undefined) provide(name, files[path]);
    const text = source(name);
    if (text !== null) queue.push(...uses(text));
  }
}

/** Fetch and hand over every module `text` reaches, through `fetchText(path)` — `null` for one
 *  the host has not got, which is then the library's.  Each name is asked for once, however many
 *  texts use it.  Returns the names handed over. */
export async function link(
  text: string,
  fetchText: (path: string) => Promise<string | null>,
): Promise<string[]> {
  const got: string[] = [];
  const asked = new Set<string>();
  const queue = uses(text);
  while (queue.length) {
    const name = queue.shift()!;
    if (asked.has(name)) continue;
    asked.add(name);
    const t = await fetchText(pathOf(name));
    if (t === null) continue;
    provide(name, t);
    got.push(name);
    queue.push(...uses(t));
  }
  return got;
}
