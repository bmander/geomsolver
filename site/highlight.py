#!/usr/bin/env python3
"""Colour the page's Solvent.

Every `<pre class="sv">` of `index.html` is rewritten in place with a span round each run the
core's own highlighter classifies (`gcs_program_highlight`, the one the sketcher's program panel
is coloured by), so the page and the app agree on what a word is.  The stylesheet in the page says
what each class looks like.  Run it again after editing a block: one already coloured is read back
as its text, and a new one is written plainly and coloured here.

    make && python3 site/highlight.py
"""
import ctypes
import html
import json
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
PAGE = ROOT / "site" / "index.html"


def core():
    for name in ("libgcs.so", "libgcs.dylib"):
        path = ROOT / "build" / name
        if path.exists():
            lib = ctypes.CDLL(str(path))
            lib.gcs_program_highlight.restype = ctypes.c_void_p
            lib.gcs_program_highlight.argtypes = [ctypes.c_char_p, ctypes.c_size_t]
            lib.gcs_str_len.restype = ctypes.c_uint32
            lib.gcs_str_len.argtypes = [ctypes.c_void_p]
            lib.gcs_str_ptr.restype = ctypes.c_void_p
            lib.gcs_str_ptr.argtypes = [ctypes.c_void_p]
            lib.gcs_str_free.argtypes = [ctypes.c_void_p]
            return lib
    sys.exit("site/highlight.py: no build/libgcs: run `make` first")


def runs(lib, source):
    """The core's runs over `source`'s bytes: (class, lo, hi), in order."""
    answer = lib.gcs_program_highlight(source, len(source))
    if not answer:
        sys.exit("site/highlight.py: the core gave no colouring")
    text = ctypes.string_at(lib.gcs_str_ptr(answer), lib.gcs_str_len(answer))
    lib.gcs_str_free(answer)
    return json.loads(text)


def coloured(lib, inner):
    source = html.unescape(re.sub(r"</?span[^>]*>", "", inner)).encode()
    plain = lambda lo, hi: html.escape(source[lo:hi].decode(), quote=False)
    out, at = [], 0
    for cls, lo, hi in runs(lib, source):
        out.append(plain(at, lo))
        out.append(f'<span class="{cls}">{plain(lo, hi)}</span>')
        at = hi
    out.append(plain(at, len(source)))
    return "".join(out)


def main():
    lib = core()
    page = PAGE.read_text()
    block = re.compile(r'(<pre class="sv">)(.*?)(</pre>)', re.S)
    count = len(block.findall(page))
    PAGE.write_text(block.sub(lambda m: m[1] + coloured(lib, m[2]) + m[3], page))
    print(f"site/highlight.py: {count} blocks coloured")


if __name__ == "__main__":
    main()
