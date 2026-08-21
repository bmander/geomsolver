/* Minimal static server for the sketcher: `npm run serve` then open the printed URL.
 * Nothing here is needed in production — the app is static files. */
import { createReadStream, statSync } from 'node:fs';
import { createServer } from 'node:http';
import { extname, join, normalize } from 'node:path';
import { dirname } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const TYPES = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.mjs': 'text/javascript; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.wasm': 'application/wasm',
  '.map': 'application/json',
  '.css': 'text/css; charset=utf-8',
};

/** `…/example/<slug>` opens the sketcher on that case — `/example/pythagoras`, `/gcs/example/truss:50`
 *  behind a `/gcs/` mount.  The page's assets are relative, so the route redirects to the app at
 *  the same mount with the slug as a query (`/gcs/?example=truss:50`) rather than serving the page
 *  from a nested path; a production host does the same with one rewrite rule. */
const EXAMPLE = /^(.*)\/example\/([^/]+)\/?$/;

const port = Number(process.env.PORT ?? 8123);
createServer((req, res) => {
  const url = new URL(req.url ?? '/', 'http://localhost');
  const m = EXAMPLE.exec(url.pathname);
  if (m) {
    res.writeHead(302, { location: `${m[1]}/?example=${m[2]}` }).end();
    return;
  }
  let path = join(root, normalize(decodeURIComponent(url.pathname)).replace(/^(\.\.[/\\])+/, ''));
  try {
    if (statSync(path).isDirectory()) path = join(path, 'index.html');
  } catch {
    res.writeHead(404).end('not found');
    return;
  }
  res.writeHead(200, { 'content-type': TYPES[extname(path)] ?? 'application/octet-stream' });
  createReadStream(path).pipe(res);
}).listen(port, () => console.log(`gcs sketcher: http://localhost:${port}/`));
