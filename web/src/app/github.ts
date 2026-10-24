/* A project read from a GitHub repository: the tree from GitHub's API, the files from
 * raw.githubusercontent.com, so a public one needs no token and the API is asked once or twice.
 * A private one is read with a token the person gives, every file then through the API, whose
 * answers a page may read with an `Authorization` header (the raw host's are not).  The browser
 * gathers texts; the core parses, solves, and renders. */
import type { DrawingBundle } from '../core/drawing.js';
import { projectFile, sourceFile } from './project-explorer.js';

/** What a GitHub address names: a repository, and within it optionally a ref and a folder or a
 *  file.  `rest` is everything after `tree/` or `blob/`, which is the ref and then the path, one
 *  not told from the other until the repository says which refs it has (a branch may hold a
 *  slash). */
export interface GithubAddress {
  owner: string; repo: string; rest?: string; blob: boolean;
}

/** `https://github.com/o/r`, `…/tree/ref/dir`, `…/blob/ref/file.sv`, or the bare `o/r`. */
export function parseGithub(text: string): GithubAddress | null {
  let path = text.trim();
  if (/^[\w.-]+\/[\w.-]+\/?$/.test(path)) path = `https://github.com/${path}`;
  else if (!/^[a-z]+:\/\//i.test(path)) path = `https://${path}`;
  let url: URL;
  try { url = new URL(path); } catch { return null; }
  if (!/^(?:www\.)?github\.com$/i.test(url.hostname)) return null;
  const parts = url.pathname.split('/').filter(Boolean).map(decodeURIComponent);
  if (parts.length < 2) return null;
  const owner = parts[0], repo = parts[1].replace(/\.git$/, '');
  if (parts.length === 2) return { owner, repo, blob: false };
  if ((parts[2] !== 'tree' && parts[2] !== 'blob') || parts.length < 4) return null;
  return { owner, repo, rest: parts.slice(3).join('/'), blob: parts[2] === 'blob' };
}

/** The most files one address may fetch, so a large repository is refused rather than read;
 *  and the most fetched at once, under GitHub's limit on concurrent requests. */
const MAX_FILES = 400, AT_ONCE = 8;

/** A GitHub project as a bundle the panel opens: its paths start at the folder named (its last
 *  segment, or the repository's name), the way an example directory's do, and `directory` is
 *  that segment so the tree is titled with it.  A file address selects that file. */
export interface GithubBundle extends DrawingBundle {
  directory: string; label: string;
}

/** The headers one request carries: what it accepts, and the person's token if they gave one. */
function headers(accept: string, token?: string): Record<string, string> {
  return token ? { Accept: accept, Authorization: `Bearer ${token}` } : { Accept: accept };
}

async function json(url: string, token?: string): Promise<{ status: number; body: any }> {
  const r = await fetch(url, { headers: headers('application/vnd.github+json', token) });
  return { status: r.status, body: r.ok ? await r.json() : null };
}

function refused(status: number, what: string, token?: string): Error {
  return new Error(status === 404
    ? `${what} not found${token ? '' : ' (a private repository needs a token)'}`
    : status === 401 ? 'GitHub did not accept the token'
    : status === 403 || status === 429
      ? `GitHub refused the request: ${token ? 'what the token may read' : 'its rate limit'}, `
        + 'most likely'
    : `GitHub answered ${status} for ${what}`);
}

const segments = (path: string): string => path.split('/').map(encodeURIComponent).join('/');

interface Tree { entries: { path: string; type: string }[]; truncated: boolean }

/** The repository's tree at `ref`, or null where the ref is not one. */
async function tree(base: string, ref: string, token?: string): Promise<Tree | null> {
  const r = await json(`${base}/git/trees/${segments(ref)}?recursive=1`, token);
  if (r.status === 404 || r.status === 422) return null;
  if (!r.body) throw refused(r.status, ref, token);
  return { entries: r.body.tree ?? [], truncated: !!r.body.truncated };
}

/** Read the project at a GitHub address, with the person's token where one is given. */
export async function githubProject(address: GithubAddress, token?: string)
  : Promise<GithubBundle> {
  const { owner, repo } = address;
  const name = `${owner}/${repo}`;
  const base = `https://api.github.com/repos/${encodeURIComponent(owner)}/`
    + encodeURIComponent(repo);
  // a repository's own address is its default branch, which both hosts read as `HEAD`
  let found: Tree | null = null, ref = 'HEAD', path = '';
  const parts = address.rest?.split('/') ?? [ref];
  // the shortest ref first: `main/models` is the branch `main` far more often than `main/models`
  for (let i = 1; i <= parts.length && !found; i++) {
    ref = parts.slice(0, i).join('/');
    found = await tree(base, ref, token);
    if (found) path = parts.slice(i).join('/');
  }
  if (!found) {
    throw new Error(`${name} has no such branch, tag or commit`
      + (token ? '' : ', or is private and needs a token'));
  }
  const folder = address.blob ? path.slice(0, Math.max(0, path.lastIndexOf('/'))) : path;
  const under = folder ? `${folder}/` : '';
  const members = found.entries
    .filter((e) => e.type === 'blob' && e.path.startsWith(under) && projectFile(e.path))
    .map((e) => e.path);
  if (!members.some(sourceFile)) {
    throw new Error(`no .sv or .svd files in ${name}${folder ? `/${folder}` : ''}`
      + (found.truncated ? ' (GitHub cut the listing short)' : ''));
  }
  if (members.length > MAX_FILES) {
    throw new Error(`${members.length} files is more than ${MAX_FILES}: open a folder within it`);
  }
  const top = folder ? folder.slice(folder.lastIndexOf('/') + 1) : repo;
  const raw = `https://raw.githubusercontent.com/${encodeURIComponent(owner)}/`
    + `${encodeURIComponent(repo)}/${segments(ref)}`;
  const files: Record<string, string> = {};
  const read = async (p: string): Promise<void> => {
    const r = token
      ? await fetch(`${base}/contents/${segments(p)}?ref=${encodeURIComponent(ref)}`,
        { headers: headers('application/vnd.github.raw+json', token) })
      : await fetch(`${raw}/${segments(p)}`);
    if (!r.ok) throw refused(r.status, p, token);
    files[`${top}/${p.slice(under.length)}`] = await r.text();
  };
  // a few readers drawing from one queue; the first refusal ends the reading
  const queue = [...members];
  await Promise.all(Array.from({ length: Math.min(AT_ONCE, queue.length) }, async () => {
    for (let p = queue.shift(); p !== undefined; p = queue.shift()) await read(p);
  }));
  const source = address.blob ? `${top}/${path.slice(under.length)}` : '';
  return { source, files, directory: top, label: `${name}${folder ? `/${folder}` : ''}` };
}
