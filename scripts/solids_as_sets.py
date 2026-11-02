#!/usr/bin/env python3
"""Rewrite Solvent text from the retired solid spellings to solids as sets (#737).

    solid(F, depth: d)                    -> solid(std.solids.Prism(F, from: -d, to: 0))
    solid(F, from: a, to: b[, draft: t])  -> solid(std.solids.Prism|Drafted(F, from: a, to: b[, draft: t]))
    solid(F, about: l[, sweep: s][, sense: cw])
                                          -> solid(std.solids.Revolution(F, about: l[, from:, to:]))
    solid(F, through: b)                  -> solid(std.solids.Through(F))
    sweep(tool over t in (a, b), under: m, seen_from: g)
                                          -> solid({ p | p inside(under: m, seen_from: g, over t in (a, b)) tool })
    sweep(tool, through: b, under: m)     -> solid({ p | p inside(under: m) tool })
    sweep(I(...).f over s)                -> solid({ p | p coincident(over s) I(...).f })
    sweep(f over s)                       -> solid({ p | p coincident(over s) f })

A document (.sv) that comes to use a word of `std.solids` is given `use std.solids`.  A Rust
file's string literals are rewritten in place, each given the line beside its `use std`.

    scripts/solids_as_sets.py FILE...      rewrite in place, saying each file changed
"""
import re
import sys

LABELS = {"from", "to", "depth", "draft", "about", "sweep", "sense", "through"}


def close_of(s, i):
    """The index of the bracket closing the one at `i`, skipping strings; None if unbalanced."""
    depth = 0
    k = i
    while k < len(s):
        c = s[k]
        if c in "([{":
            depth += 1
        elif c in ")]}":
            depth -= 1
            if depth == 0:
                return k
        elif c == '"':
            return None
        elif c == "\n" and depth == 0:
            return None
        k += 1
    return None


def split_top(s):
    """`s` split at its top-level commas, each part stripped."""
    parts, depth, cur = [], 0, ""
    for c in s:
        if c in "([{":
            depth += 1
        elif c in ")]}":
            depth -= 1
        if c == "," and depth == 0:
            parts.append(cur)
            cur = ""
        else:
            cur += c
    parts.append(cur)
    return [p.strip() for p in parts if p.strip() != ""]


def label_of(part):
    m = re.match(r"^([A-Za-z_][A-Za-z_0-9]*)\s*:(?!=)\s*(.*)$", part, re.S)
    return (m.group(1), m.group(2).strip()) if m else (None, part)


PLAIN = re.compile(r"^[A-Za-z_0-9.]+$")


def negated(t):
    t = t.strip()
    if t.startswith("-") and PLAIN.match(t[1:]):
        return t[1:]
    if PLAIN.match(t):
        return "-" + t
    return "-(" + t + ")"


def solid_word(args):
    """The `std.solids` word a retired `solid(…)` bracket list says, or None for none."""
    face, labels = [], {}
    for p in split_top(args):
        name, value = label_of(p)
        if name in LABELS:
            if name in labels:
                return None
            labels[name] = value
        elif name is None:
            face.append(p)
        else:
            return None  # some label the rewrite does not know: leave it for a person
    if not labels or not face:
        return None
    f = ", ".join(face)
    keys = set(labels)
    if keys <= {"from", "to", "draft"} and {"from", "to"} <= keys:
        frm, to = labels["from"], labels["to"]
    elif keys <= {"depth", "draft"} and "depth" in keys:
        frm, to = negated(labels["depth"]), "0"
    elif keys <= {"about", "sweep", "sense"} and "about" in keys:
        out = [f, "about: " + labels["about"]]
        turn = labels.get("sweep", "360deg")
        cw = labels.get("sense", "ccw") == "cw"
        if cw:
            out += ["from: " + negated(turn), "to: 0deg"]
        elif "sweep" in labels:
            out += ["to: " + turn]
        return "std.solids.Revolution(" + ", ".join(out) + ")"
    elif keys == {"through"}:
        return "std.solids.Through(" + f + ")"
    else:
        return None
    if "draft" in labels:
        return "std.solids.Drafted(%s, from: %s, to: %s, draft: %s)" % (f, frm, to, labels["draft"])
    return "std.solids.Prism(%s, from: %s, to: %s)" % (f, frm, to)


def sweep_set(args):
    """The set a retired `sweep(…)` says, as `solid({ … })`, or None."""
    parts = split_top(args)
    if not parts:
        return None
    head, labels = parts[0], {}
    for p in parts[1:]:
        name, value = label_of(p)
        if name not in {"under", "seen_from", "through"} or name in labels:
            return None
        labels[name] = value
    m = re.match(r"^(.*?)\s+over\s+([A-Za-z_][A-Za-z_0-9]*)(\s+in\s+\(.*\))?$", head, re.S)
    frames = [k + ": " + labels[k] for k in ("under", "seen_from") if k in labels]
    if m:
        tool, over, interval = m.group(1).strip(), m.group(2), m.group(3)
        family = re.search(r"\)\.[A-Za-z_]", tool) is not None
        if interval is None and not frames and "through" not in labels:
            # a face family, or a face of the drawing over a declared parameter
            return "solid({ p | p coincident(over %s) %s })" % (over, tool)
        if family:
            return None
        clause = "over " + over + (interval or "")
        return "solid({ p | p inside(%s) %s })" % (", ".join(frames + [clause]), tool)
    if "through" in labels:
        inner = (", ".join(frames))
        return "solid({ p | p inside(%s) %s })" % (inner, head.strip())
    return None


WORD = re.compile(r"(?<![A-Za-z_0-9.#])(solid|sweep)\(")


def rewrite(text):
    """`text` rewritten, and whether a word of `std.solids` is now used."""
    out, i, used = [], 0, False
    while True:
        m = WORD.search(text, i)
        if not m:
            out.append(text[i:])
            break
        open_at = m.end() - 1
        close = close_of(text, open_at)
        if close is None:
            out.append(text[i:m.end()])
            i = m.end()
            continue
        inner = text[open_at + 1:close]
        new = None
        if m.group(1) == "solid":
            w = solid_word(inner)
            if w:
                new = "solid(" + w + ")"
                used = True
        else:
            new = sweep_set(inner)
        if new is None:
            out.append(text[i:m.end()])
            i = m.end()
            continue
        out.append(text[i:m.start()])
        out.append(new)
        i = close + 1
    return "".join(out), used


USE = re.compile(r"^use std\.solids\b", re.M)


def with_use(doc, nl="\n"):
    """`doc` given `use std.solids` beside its `use std`, if it lacks it."""
    if USE.search(doc.replace("\\n", "\n")):
        return doc
    m = re.search(r"(^|\n|\\n)(use std)(?=$|\n|\\n|\s*//)", doc)
    if not m:
        m = re.search(r"(^|\n|\\n)(use std[.\w]*(?: \([^)]*\))?)(?=$|\n|\\n)", doc)
    if not m:
        m = re.search(r"(^|\n|\\n)(unit [a-z]+)(?=$|\n|\\n)", doc)
    if not m:
        return None
    sep = "\\n" if m.group(1) == "\\n" or ("\n" not in doc and "\\n" in doc) else "\n"
    return doc[:m.end()] + sep + "use std.solids" + doc[m.end():]


def rewrite_sv(text):
    new, used = rewrite(text)
    if used:
        withu = with_use(new)
        if withu is None:
            withu = "use std.solids\n" + new
        new = withu
    return new


STRING = re.compile(r'r(#*)"(.*?)"\1|"((?:[^"\\]|\\.)*)"', re.S)
# a TypeScript test's template literals and quoted strings, in the same two groups
TS_STRING = re.compile(r'()`((?:[^`\\]|\\.)*)`|"((?:[^"\\]|\\.)*)"', re.S)


def rewrite_rs(text, STRING=STRING):
    out, i, missing = [], 0, 0
    for m in STRING.finditer(text):
        body = m.group(2) if m.group(2) is not None else m.group(3)
        if body is None or ("solid(" not in body and "sweep(" not in body):
            continue
        new, used = rewrite(body)
        if new == body:
            continue
        if used:
            withu = with_use(new)
            if withu is None:
                missing += 1
            else:
                new = withu
        start = m.start(2) if m.group(2) is not None else m.start(3)
        end = m.end(2) if m.group(2) is not None else m.end(3)
        out.append(text[i:start])
        out.append(new)
        i = end
    out.append(text[i:])
    text = "".join(out)
    if missing:
        # a fragment's `use std` is a prelude's: every literal of the file that has one, and none
        # of `std.solids`, is given it
        out, i, missing = [], 0, 0
        for m in STRING.finditer(text):
            body = m.group(2) if m.group(2) is not None else m.group(3)
            if body is None or not re.search(r"(^|\n|\\n)use std(?=$|\n|\\n)", body):
                continue
            if USE.search(body.replace("\\n", "\n")):
                continue
            new = with_use(body)
            start = m.start(2) if m.group(2) is not None else m.start(3)
            end = m.end(2) if m.group(2) is not None else m.end(3)
            out.append(text[i:start])
            out.append(new)
            i = end
        out.append(text[i:])
        text = "".join(out)
        missing = 0 if i else 1
    return text, missing


def main(paths):
    for p in paths:
        text = open(p).read()
        if p.endswith(".rs") or p.endswith(".ts"):
            new, missing = rewrite_rs(text, TS_STRING if p.endswith(".ts") else STRING)
            note = f" ({missing} literal(s) with no `use std` to add `use std.solids` beside)" if missing else ""
        else:
            new, note = rewrite_sv(text), ""
        if new != text:
            open(p, "w").write(new)
            print(f"rewrote {p}{note}")


if __name__ == "__main__":
    main(sys.argv[1:])
