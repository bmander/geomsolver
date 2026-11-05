#!/usr/bin/env python3
"""Rewrite Solvent text from the retired set spellings to the notation of sets (#783).

    p inside S                    -> p in S          (a face formal's: p in a.side)
    p outside S                   -> p not in S
    p inside P, P a plane         -> p distance(<= 0mm, along: n) P   (outside: >=)
    p inside F, F a drawn face    -> some h: p in F at(along: n) h    (outside: not some …)
    p coincident S, S a set       -> p in S          (S a region of std.sets: p in boundary(S))
    S.surface                     -> boundary(S)     (a face's: boundary(a.side))
    l tangent S, S a region       -> l tangent boundary(S)   (envelope(S over …) alike)
    p distance(d, side: outside) S -> p distance(d) S   (a developable sheet's stays)
    p distance(d, side: inside) S  -> p distance(d) { q | q not in S }
    std.sets.Offset(S, d: x)      -> std.sets.Grown(S, d: x)
    std.sets.Sphere(c, …) read as a ball (inside, outside, a side's distance, grown)
                                  -> std.sets.Ball(c, …)
    p inside(under: m, seen_from: g, over t in (a, b)) X
                                  -> some t in (a, b): p in X at(under: m, seen_from: g) t
    p inside(under: m) X          -> some t: p in X at(under: m) t   (the whole roll)
    p coincident(over s) F        -> some s: p in F at s
    p coincident(under: l, over t in (a, b)) F -> some t in (a, b): p in F at(under: l) t
    n := x in f, f a frame        -> n := x at f
    when not w in A               -> when w not in A

A Rust file's string literals are rewritten in place; a document (.sv) whole.  What the script
cannot read is printed for a hand to settle:

    scripts/sets_in_notation.py FILE...      rewrite in place, saying each file changed
"""
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# the std.sets families that are regions once #783 lands, and the one surface
LIB_REGIONS = {"Ball", "Cylinder", "Cone", "ConeThrough", "HalfSpace", "Grown", "Shrunk", "Rod"}
NAME = r"[A-Za-z_][A-Za-z_0-9]*"
REF = NAME + r"(?:\.[A-Za-z_][A-Za-z_0-9]*|\[[^\]\n]*\])*"
# a call, one level of brackets nested in its arguments, and a member after it
CALL = r"[A-Za-z_][\w.]*\((?:[^()\n]|\((?:[^()\n]|\([^()\n]*\))*\))*\)(?:\.[A-Za-z_]\w*)*"
OPERAND = "(?:" + CALL + "|" + REF + ")"

WARN = []


def families(texts):
    """Every component whose body is a set, by name, and whether that body is a region."""
    fam = {}
    for t in texts:
        for m in re.finditer(r"component (" + NAME + r")\([^)]*\)(?:\s*symmetric\([^)]*\))?\s*:=\s*\{", t):
            body = t[m.end():m.end() + 2000]
            depth, k = 1, 0
            while k < len(body) and depth:
                if body[k] == "{":
                    depth += 1
                elif body[k] == "}":
                    depth -= 1
                k += 1
            body = body[:k]
            fam[m.group(1)] = bool(re.search(r"\((?:<=|>=)|\b(?:inside|outside)\b|\bin: \(|"
                                             r"\b(?:in|not in)\b", body))
    return fam


class Doc:
    """What one text declares, by kind."""

    def __init__(self, text, fam):
        code = strip_comments(text)
        self.fam = fam
        self.sets = {}      # name -> (family or None for a literal, region?)
        for m in re.finditer(r"(" + NAME + r")\s*:=\s*\{\s*(" + NAME + r")\s*\|", code):
            body = code[m.end():m.end() + 1500]
            self.sets[m.group(1)] = (None, bool(re.search(
                r"\((?:<=|>=)|\b(?:inside|outside)\b|\bin: \(|,\s*side:", body.split("}")[0])))
        for m in re.finditer(r"(" + NAME + r")\s*:=\s*(?:std\.sets\.)?(" + NAME + r")\(", code):
            name, f = m.group(1), m.group(2)
            if f in LIB_REGIONS or f in ("Sphere", "Offset"):
                self.sets[name] = (f, f != "Sphere")
            elif f in fam:
                self.sets[name] = (f, fam[f])
        self.set_formals = set(re.findall(r"(" + NAME + r"): set\b", code))
        for f in self.set_formals:
            self.sets.setdefault(f, (None, True))
        self.faces = set(re.findall(r"(" + NAME + r")\s*:=\s*face\(", code))
        self.formal_faces = set(re.findall(r"(" + NAME + r"): face\b", code))
        self.planes = set(re.findall(r"(" + NAME + r")\s*:=\s*plane\b", code))
        self.planes |= set(re.findall(r"(" + NAME + r"): plane\b", code))
        self.planes |= {"std.front", "std.top", "std.side"}
        self.frames = set(re.findall(r"(" + NAME + r")\s*:=\s*frame\b", code))
        self.frames |= set(re.findall(r"(" + NAME + r"): frame\b", code))
        self.params = set(re.findall(r"\bparam (" + NAME + r")", code))
        self.sheets = set(re.findall(r"(" + NAME + r")\s*:=\s*std\.panels\.Developable\(", code))
        self.solids = set(re.findall(r"(" + NAME + r")\s*:=\s*solid\(", code))
        # spheres read as balls: inside, outside, a side's distance, grown or rounded
        self.balls = set()
        for name, (f, _) in self.sets.items():
            if f != "Sphere":
                continue
            n = re.escape(name)
            if re.search(r"\b(?:inside|outside) " + n + r"\b(?!\.)|side: (?:in|out)side\) " + n
                         + r"\b|Offset\(" + n + r"\b|Rounded\([^)]*\b" + n + r"\b|Chamfered\([^)]*\b"
                         + n + r"\b", code):
                self.balls.add(name)
                self.sets[name] = ("Ball", True)

    def region(self, ref):
        s = self.sets.get(ref)
        return s is not None and s[1]

    def lib_region(self, ref):
        s = self.sets.get(ref)
        return s is not None and (s[0] in LIB_REGIONS or s[0] in ("Offset",)
                                  or (s[0] in self.fam and self.fam[s[0]]))


def strip_comments(text):
    return re.sub(r"//[^\n]*", "", text)


def split_top(s):
    parts, depth, cur = [], 0, ""
    for c in s:
        if c in "([{":
            depth += 1
        elif c in ")]}":
            depth -= 1
        if c == "," and depth == 0:
            parts.append(cur.strip())
            cur = ""
        else:
            cur += c
    if cur.strip():
        parts.append(cur.strip())
    return parts


def paren_args(s, i):
    """The text inside the parentheses opening at `i`, and the index past them."""
    depth, k = 0, i
    while k < len(s):
        if s[k] == "(":
            depth += 1
        elif s[k] == ")":
            depth -= 1
            if depth == 0:
                return s[i + 1:k], k + 1
        k += 1
    return None, i


def sweeps(code, doc):
    """`p inside(…) X` and `p coincident(… over …) F` as `some …`."""
    out, i = [], 0
    pat = re.compile(r"(" + REF + r") (inside|coincident)\(")
    while True:
        m = pat.search(code, i)
        if not m:
            out.append(code[i:])
            break
        args, past = paren_args(code, m.end() - 1)
        items = split_top(args or "")
        over = [x for x in items if x.startswith("over ")]
        labels = [x for x in items if re.match(r"(under|seen_from):", x)]
        rest = [x for x in items if x not in over and x not in labels]
        if args is None or rest or (m.group(2) == "coincident" and not over):
            out.append(code[i:m.end()])
            i = m.end()
            continue
        om = re.match(r"\s(" + OPERAND + r")", code[past:])
        if not om:
            out.append(code[i:m.end()])
            i = m.end()
            continue
        point, x = m.group(1), om.group(1)
        if over:
            ov = re.match(r"over (" + NAME + r")(?: in (\(.*\)))?$", over[0])
            var, interval = ov.group(1), ov.group(2)
        else:
            var, interval = ("u" if "t" in doc.params else "t"), None
        head = "some " + var + (" in " + interval if interval else "") + ": "
        at = "at(" + ", ".join(labels) + ") " + var if labels else "at " + var
        out.append(code[i:m.start()] + head + point + " in " + x + " " + at)
        i = past + om.end()
    return "".join(out)


def rewrite_code(code, doc):
    """One text's code (comments kept apart), rewritten."""
    code = sweeps(code, doc)
    code = code.replace("std.sets.Offset(", "std.sets.Grown(")
    for b in doc.balls:
        code = re.sub(r"(\b" + re.escape(b) + r"\s*:=\s*std\.sets\.)Sphere\(", r"\1Ball(", code)
    # a distance from a set with a side
    def side(m):
        args, s, y = m.group(1), m.group(2), m.group(3)
        root = y.split(".")[0]
        if root in doc.sheets:
            return m.group(0)
        if s == "outside":
            return "distance(" + args + ") " + y
        return "distance(" + args + ") { q | q not in " + y + " }"
    code = re.sub(r"distance\(([^()\n]*(?:\([^()\n]*\)[^()\n]*)*), side: (inside|outside)\) (" + OPERAND + ")",
                  side, code)
    # `.surface`
    def surface(m):
        y = m.group(1)
        root = y.split(".")[0]
        if root in doc.sets or root in doc.balls:
            return "boundary(" + y + ")"
        return "boundary(" + y + ".side)"
    code = re.sub(r"\b(" + REF + r")\.surface\b", lambda m: surface(m) if not m.group(1).endswith("std") else m.group(0), code)
    # inside / outside
    def side_word(m):
        p_, w, y = m.group(1), m.group(2), m.group(3)
        root = y.split(".")[0].split("(")[0]
        neg = w == "outside"
        if y in doc.planes or root in doc.planes and "." not in y:
            return p_ + " distance(" + (">=" if neg else "<=") + " 0mm, along: n) " + y
        if y in doc.faces:
            return p_ + (" not in " if neg else " in ") + y + "#FACE#"
        if y in doc.formal_faces:
            return p_ + (" not in " if neg else " in ") + y + ".side"
        s_ = doc.sets.get(y)
        if s_ is not None and not s_[1]:
            WARN.append("`%s %s %s` read a surface's one number as a bound: `%s` must become a "
                        "region" % (p_, w, y, y))
        return p_ + (" not in " if neg else " in ") + y
    code = re.sub(r"(" + REF + r") (inside|outside) (" + OPERAND + ")", side_word, code)
    # drawn faces: the prism through one, and what is not in it
    def face_prism(m):
        p_, neg, y = m.group(1), m.group(2), m.group(3)
        return ("not " if neg else "") + "some h: " + p_ + " in " + y + " at(along: n) h"
    code = re.sub(r"(" + REF + r") (not )?in (" + REF + r")#FACE#", face_prism, code)
    # coincident against a set, tangent and envelope against a region
    def coincident(m):
        p_, y = m.group(1), m.group(2)
        root = y.split("(")[0]
        if y.startswith("boundary("):
            return p_ + " in " + y
        if y in doc.sets or root.startswith("std.sets.") or y in doc.formal_faces:
            if doc.lib_region(y) or (root.startswith("std.sets.") and root[9:] in LIB_REGIONS) \
                    or y in doc.set_formals:
                return p_ + " in boundary(" + y + ")"
            if y in doc.sets and doc.sets[y][1] and doc.sets[y][0] is None:
                WARN.append("a point coincident with the region `%s`: written `in`" % y)
            return p_ + " in " + y
        return m.group(0)
    code = re.sub(r"(" + REF + r") coincident (" + OPERAND + r")(?=\s|$|\)|;)", coincident, code)
    # a distance from what is not in a set, its binder apart from the statement's point
    code = re.sub(r"\b(" + NAME + r") distance\(([^()\n]*)\) \{ q \| q not in",
                  lambda m: m.group(1) + " distance(" + m.group(2) + ") { " + ("r" if m.group(1) == "q" else "q")
                  + " | " + ("r" if m.group(1) == "q" else "q") + " not in", code)
    def tangent(m):
        a, args, y = m.group(1), m.group(2) or "", m.group(3)
        if doc.lib_region(y):
            y = "boundary(" + y + ")"
        if doc.lib_region(a):
            a = "boundary(" + a + ")"
        return a + " tangent" + args + " " + y
    code = re.sub(r"(" + REF + r") tangent(\([^()\n]*\))? (" + REF + r")\b", tangent, code)
    code = re.sub(r"meshing\((" + REF + r"), (" + REF + r") over",
                  lambda m: "meshing(" + ", ".join("boundary(" + x + ")" if doc.lib_region(x) else x
                                                   for x in (m.group(1), m.group(2))) + " over", code)
    code = re.sub(r"envelope\((" + REF + r") over",
                  lambda m: "envelope(boundary(" + m.group(1) + ") over" if doc.lib_region(m.group(1))
                  else m.group(0), code)
    # placement in a jointed frame
    code = re.sub(r"(" + NAME + r"\s*:=\s*" + REF + r") in (" + NAME + r")[ \t]*$",
                  lambda m: m.group(1) + " at " + m.group(2) if m.group(2) in doc.frames else m.group(0),
                  code, flags=re.M)
    code = re.sub(r"when not (" + REF + r") in ", r"when \1 not in ", code)
    return code


def rewrite_text(text, fam):
    """A document's text: its code rewritten, its comments as they were."""
    doc = Doc(text, fam)
    # rewrite line by line, keeping each line's comment apart; a statement spanning lines is
    # rewritten as one, its comments put back where they stood
    lines = text.split("\n")
    codes, comments = [], []
    for ln in lines:
        k = ln.find("//")
        if k >= 0:
            codes.append(ln[:k])
            comments.append(ln[k:])
        else:
            codes.append(ln)
            comments.append("")
    joined = "\x00".join(codes)
    new = rewrite_code(joined.replace("\x00", "\n"), doc).split("\n")
    if len(new) != len(lines):
        WARN.append("lines moved; left as they were")
        return text
    return "\n".join(c + m for c, m in zip(new, comments))


def rust_strings(text, fam):
    """A Rust file: each string literal holding Solvent rewritten, its escapes and its line
    breaks kept as they were."""
    def one(m):
        s = m.group(0)
        if not re.search(r"\b(inside|outside|coincident|side:|Offset|surface|tangent|envelope)\b"
                         r"|:= \w+ in \w+|when not", s):
            return s
        body = s[1:-1]
        # `\n` escapes read as the line breaks they are, and put back after
        body = body.replace("\\n", "\x03\n")
        fmt = "{{" in body or "}}" in body
        holes = []
        if fmt:
            # a format string's own holes kept apart, its doubled braces read as the braces
            def hole(h):
                holes.append(h.group(0))
                return "\x04%d\x05" % (len(holes) - 1)
            body = re.sub(r"(?<!\{)\{[A-Za-z0-9_:.?#]*\}(?!\})", hole, body)
            body = body.replace("{{", "{").replace("}}", "}")
        new = rewrite_text(body, fam)
        if fmt:
            new = new.replace("{", "{{").replace("}", "}}")
            new = re.sub(r"\x04(\d+)\x05", lambda h: holes[int(h.group(1))], new)
        new = new.replace("\x03\n", "\\n")
        return '"' + new + '"'
    return re.sub(r'"(?:[^"\\]|\\.)*"', one, text, flags=re.S)


def main():
    texts = []
    for d in ("rust/lib", "rust/examples"):
        for root, _, files in os.walk(os.path.join(ROOT, d)):
            for f in files:
                if f.endswith(".sv"):
                    texts.append(open(os.path.join(root, f)).read())
    fam = families(texts)
    fam["Blade"] = True
    for path in sys.argv[1:]:
        text = open(path).read()
        WARN.clear()
        new = rust_strings(text, fam) if path.endswith(".rs") else rewrite_text(text, fam)
        if new != text:
            open(path, "w").write(new)
            print(path)
        for w in sorted(set(WARN)):
            print("  " + path + ": " + w)


if __name__ == "__main__":
    main()
