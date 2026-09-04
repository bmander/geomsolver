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
  '.sv': 'text/plain; charset=utf-8',
};

/** `…/example/<slug>` opens the sketcher on that case — `/example/pythagoras`, `/gcs/example/truss:50`
 *  behind a `/gcs/` mount.  The page's assets are relative, so the route redirects to the app at
 *  the same mount with the slug as a query (`/gcs/?example=truss:50`) rather than serving the page
 *  from a nested path; a production host does the same with one rewrite rule. */
const EXAMPLE = /^(.*)\/example\/([^/]+)\/?$/;

/** `…/examples/<path>.sv` is that file in `rust/examples/` — the server standing in for the
 *  directory beside the document, which is where `solventc` looks first.  The app fetches a case
 *  and the modules it `use`s from here before falling back to the copies compiled into the core,
 *  so editing a document and refreshing the page shows the edit with no wasm rebuilt.  Read-only,
 *  `.sv` only, one word per path segment, so nothing above that directory can be named. */
const EXAMPLES = /^(?:.*)\/examples\/((?:[\w-]+\/)*[\w-]+\.sv)$/;

const port = Number(process.env.PORT ?? 8123);
createServer((req, res) => {
  // A request line is whatever reached the socket, and `new URL` throws on some of them — `//`
  // is a protocol-relative URL with no host, which a stray probe or a browser extension will
  // send.  Uncaught in a handler that is the whole server, one of those takes the server down
  // and the page it was serving with it.
  let url;
  try {
    url = new URL(req.url ?? '/', 'http://localhost');
  } catch {
    res.writeHead(400).end('bad request');
    return;
  }
  const m = EXAMPLE.exec(url.pathname);
  if (m) {
    res.writeHead(302, { location: `${m[1]}/?example=${m[2]}` }).end();
    return;
  }
  const x = EXAMPLES.exec(url.pathname);
  if (x) {
    const file = join(root, '..', 'rust', 'examples', x[1]);
    try {
      statSync(file);
    } catch {
      res.writeHead(404).end('not found');
      return;
    }
    res.writeHead(200, { 'content-type': TYPES['.sv'], 'cache-control': 'no-store' });
    createReadStream(file).pipe(res);
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
