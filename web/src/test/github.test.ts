import assert from 'node:assert/strict';
import test from 'node:test';
import { githubPath, githubProject, parseGithub } from '../app/github.js';

test('a GitHub address names a repository, and in it a ref and a folder or a file', () => {
  assert.deepEqual(parseGithub('https://github.com/bmander/geomsolver'),
    { owner: 'bmander', repo: 'geomsolver', blob: false });
  assert.deepEqual(parseGithub('bmander/geomsolver'),
    { owner: 'bmander', repo: 'geomsolver', blob: false });
  assert.deepEqual(parseGithub('github.com/bmander/geomsolver.git'),
    { owner: 'bmander', repo: 'geomsolver', blob: false });
  assert.deepEqual(parseGithub('https://github.com/o/r/tree/feat/x/rust/examples'),
    { owner: 'o', repo: 'r', rest: 'feat/x/rust/examples', blob: false });
  assert.deepEqual(parseGithub('https://github.com/o/r/blob/main/a/b.svd'),
    { owner: 'o', repo: 'r', rest: 'main/a/b.svd', blob: true });
  assert.equal(parseGithub('https://gitlab.com/o/r'), null);
  assert.equal(parseGithub('https://github.com/o'), null);
  assert.equal(parseGithub('https://github.com/o/r/issues/3'), null);
  assert.equal(parseGithub('not an address'), null);
});

test("the page's ?github= address is the same address without its host", () => {
  for (const text of ['o/r', 'o/r.js', 'o/r/tree/feat/x/rust/examples', 'o/r/blob/main/a/b.svd']) {
    const address = parseGithub(text)!;
    assert.deepEqual(address, parseGithub(`https://github.com/${text}`));
    assert.equal(githubPath(address), text);
  }
  assert.equal(githubPath(parseGithub('https://github.com/o/r.git')!), 'o/r');
  assert.equal(parseGithub('gitlab.com/o/r'), null);
  assert.equal(parseGithub('o/r/issues/3'), null);
});

/** A repository with the branch `feat/x`, served as GitHub's API and raw host would. */
function served(asked: string[] = []) {
  const tree = [
    { path: 'README.md', type: 'blob' },
    { path: 'models', type: 'tree' },
    { path: 'models/part.sv', type: 'blob' },
    { path: 'models/part.svd', type: 'blob' },
    { path: 'models/notes.png', type: 'blob' },
    { path: 'other/x.sv', type: 'blob' },
  ];
  return async (input: string | URL | Request) => {
    const url = String(input);
    asked.push(url);
    const api = 'https://api.github.com/repos/o/r';
    if (['HEAD', 'main', 'feat/x'].some((ref) => url === `${api}/git/trees/${ref}?recursive=1`)) {
      return Response.json({ tree, truncated: false });
    }
    if (url.startsWith(`${api}/git/trees/`)) return new Response('', { status: 404 });
    const raw = /^https:\/\/raw\.githubusercontent\.com\/o\/r\/(?:HEAD|main|feat\/x)\/(.+)$/
      .exec(url);
    if (raw && tree.some((e) => e.path === raw[1])) return new Response(`// ${raw[1]}`);
    return new Response('', { status: 404 });
  };
}

test('a repository opens on its default branch, its Solvent and text files in the tree',
  async (t) => {
    const asked: string[] = [];
    t.mock.method(globalThis, 'fetch', served(asked));
    const bundle = await githubProject(parseGithub('o/r')!);
    assert.equal(asked.filter((url) => url.startsWith('https://api.github.com/')).length, 1);
    assert.equal(bundle.directory, 'r');
    assert.deepEqual(Object.keys(bundle.files).sort(),
      ['r/README.md', 'r/models/part.sv', 'r/models/part.svd', 'r/other/x.sv']);
    assert.equal(bundle.files['r/models/part.sv'], '// models/part.sv');
    assert.equal(bundle.source, '');
  });

test('a folder on a branch with a slash opens that folder; a file address selects it',
  async (t) => {
    const asked: string[] = [];
    t.mock.method(globalThis, 'fetch', served(asked));
    const folder = await githubProject(parseGithub('github.com/o/r/tree/feat/x/models')!);
    assert.equal(folder.directory, 'models');
    assert.deepEqual(Object.keys(folder.files).sort(), ['models/part.sv', 'models/part.svd']);
    assert.ok(asked.includes('https://raw.githubusercontent.com/o/r/feat/x/models/part.sv'));
    const file = await githubProject(parseGithub('github.com/o/r/blob/main/models/part.svd')!);
    assert.equal(file.source, 'models/part.svd');
    assert.equal(file.label, 'o/r/models');
  });

test('an address with no Solvent files, or an unknown ref, says so', async (t) => {
  t.mock.method(globalThis, 'fetch', served());
  await assert.rejects(githubProject(parseGithub('github.com/o/r/tree/main/nothing')!), /no \.sv/);
  await assert.rejects(githubProject(parseGithub('github.com/o/r/tree/nope')!), /no such branch/);
});

test('a token reads every file through the API, and only the API sees it', async (t) => {
  const seen: { url: string; auth?: string; accept?: string }[] = [];
  const api = 'https://api.github.com/repos/o/r';
  t.mock.method(globalThis, 'fetch', async (input: string | URL | Request, init?: RequestInit) => {
    const url = String(input);
    const h = (init?.headers ?? {}) as Record<string, string>;
    seen.push({ url, auth: h.Authorization, accept: h.Accept });
    if (!h.Authorization) return new Response('', { status: 404 });
    if (url === `${api}/git/trees/HEAD?recursive=1`) {
      return Response.json({ tree: [{ path: 'a.sv', type: 'blob' }], truncated: false });
    }
    if (url === `${api}/contents/a.sv?ref=HEAD`) return new Response('point p');
    return new Response('', { status: 404 });
  });
  await assert.rejects(githubProject(parseGithub('o/r')!), /needs a token/);
  seen.length = 0;
  const bundle = await githubProject(parseGithub('o/r')!, 'secret');
  assert.deepEqual(bundle.files, { 'r/a.sv': 'point p' });
  assert.ok(seen.every((s) => s.url.startsWith('https://api.github.com/')
    && s.auth === 'Bearer secret'));
  assert.equal(seen.at(-1)!.accept, 'application/vnd.github.raw+json');
});
