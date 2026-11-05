#!/usr/bin/env python3
"""#783 P3: the body words `cut`, `union` and `common` are words of `std.solids`, imported bare
where a file uses them.  Every `.sv` file whose statements use one gets it listed in its
`use std.solids (…)`: names added to a `use std.solids` it has, or a line of its own after its
last `use`.  Statements only — a body word in a comment is no use of it.

    python3 scripts/body_words_imported.py FILE_OR_DIR ...
"""
import re
import sys
from pathlib import Path

REF = r"[A-Za-z_]\w*(?:\[[^\]]*\])?(?:\.[A-Za-z_]\w*(?:\[[^\]]*\])?)*"
STMT = re.compile(rf"^\s*(?:private\s+)?{REF}\s+(cut|union|common)\s+{REF}\s*(?://.*)?$")
USE = re.compile(r"^(\s*)use std\.solids(?:\s*\(([^)]*)\))?\s*(//.*)?$")
ANY_USE = re.compile(r"^\s*use\s")


def words_used(lines):
    found = []
    for line in lines:
        m = STMT.match(line)
        if m and m.group(1) not in found:
            found.append(m.group(1))
    return found


def rewrite(text):
    lines = text.split("\n")
    used = words_used(lines)
    if not used:
        return text
    order = ["cut", "union", "common"]
    for i, line in enumerate(lines):
        m = USE.match(line)
        if m:
            have = [n.strip() for n in (m.group(2) or "").split(",") if n.strip()]
            more = [w for w in order if w in used and w not in have]
            if not more:
                return text
            names = ", ".join(have + more)
            tail = f"   {m.group(3)}" if m.group(3) else ""
            lines[i] = f"{m.group(1)}use std.solids ({names}){tail}"
            return "\n".join(lines)
    last = max((i for i, l in enumerate(lines) if ANY_USE.match(l)), default=None)
    names = ", ".join(w for w in order if w in used)
    if last is None:
        return text  # no `use` at all: a module the caller reads by hand
    indent = re.match(r"^\s*", lines[last]).group(0)
    lines.insert(last + 1, f"{indent}use std.solids ({names})")
    return "\n".join(lines)


def main():
    files = []
    for a in sys.argv[1:]:
        p = Path(a)
        files += sorted(p.rglob("*.sv")) if p.is_dir() else [p]
    for f in files:
        text = f.read_text()
        new = rewrite(text)
        if new != text:
            f.write_text(new)
            print(f)


if __name__ == "__main__":
    main()
