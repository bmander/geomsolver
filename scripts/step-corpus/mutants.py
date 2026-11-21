#!/usr/bin/env python3
"""Mutants: STEP files of the corpus each broken by one edit of its text, for holding the kernel's
reader to *never read wrong* on files that were not written right (#993 stage 1). Standard library
only; OCCT (through oracle.py) is the oracle, as for the corpus.

    python3 -I scripts/step-corpus/mutants.py make            # sources chosen, mutants written
    python3 -I scripts/step-corpus/mutants.py oracle --jobs 2 # OCCT's reading of each mutant
    python3 -I scripts/step-corpus/mutants.py table           # the committed table

`make` chooses the sources from the corpus's reference table (`SOURCES` per writer, each at most
`SMALL` bytes, a file OCCT reads valid with nothing for its healing to change and a reference of
its own: `closed`, `mass+occt` or `occt`; per writer the ones that bring surface kinds the
writer's earlier picks do not, then the smallest), and writes each mutant to
`$STEP_CORPUS/mutants/<kind>/<source, its slashes as __>.<k>.step`, the list of them with what was
edited in `mutants/manifest.tsv`. Every edit leaves the file well-formed Part 21 (an entity's
parameter replaced by another of the same syntax, a list one longer or shorter, an entity added);
what it breaks is the solid. The sites are drawn from a `random.Random` seeded by the source's
path and the kind, so the set is remade byte for byte.

The kinds (`KINDS`): an ADVANCED_FACE's `same_sense` flipped (`face_sense`), an ORIENTED_EDGE's
orientation (`oriented_edge`), an EDGE_CURVE's `same_sense` (`edge_sense`), a FACE_BOUND's
orientation (`bound_orientation`); a face dropped from its shell (`face_dropped`) or listed twice
(`face_twice`); an edge's two vertices swapped (`vertices_swapped`); a vertex's point moved by a
thousandth of the part's diagonal (`vertex_nudged`) and by half of it (`vertex_moved`); a radius
scaled by 1.01 (`radius_scaled`: a cylinder's, cone's, sphere's or torus's, else a circle's); a
DIRECTION negated (`direction_negated`, one a placement of a surface uses where there is one); a
B-spline's interior knot moved a quarter of the way to the next (`knot_moved`); a rational
B-spline's weight scaled by 1.5 (`weight_scaled`); a PCURVE's surface pointed at another surface
(`pcurve_retargeted`); the length unit changed (`unit_changed`: millimetres to metres, metres to
millimetres, or a conversion's factor times ten); the stated uncertainty made the part's half
diagonal (`uncertainty_huge`) or zero (`uncertainty_zero`); a reference pointed at an entity of
the wrong type (`wrong_type`: a face's surface at a DIRECTION, an edge's curve at a VERTEX_POINT,
an oriented edge's edge at an ADVANCED_FACE), at no entity (`missing`); and a cycle (`cycle`: a
face whose surface is itself).

`oracle` reads each mutant with OCCT (`oracle.py`'s reading, healing off and on, a process per
file, `TIMEOUT` seconds each: a file OCCT hangs or crashes on is one it reads no solid from).

`table` writes `rust/gcs-core/tests/step_mutants/mutants.tsv` (and `$STEP_CORPUS/mutants.tsv`):
the reference table's columns, a row a mutant, with `mutation` (kind), `mutant_of` (the source)
and `edit` (what was changed). What each must be read to (`ref_from`):

* `refuse` — OCCT reads no solid from it (it fails, hangs, or reads none of positive volume: an
  open or inverted shell), or its analyzer finds the shape invalid while the source was valid and
  its measures moved, or it reads it only by raising its tolerances past `table.CAP` (a vertex far
  off its edges): the mutant represents no solid the reader may return;
* `disputed` — OCCT reads it differently with healing and without (validity, or the measures past
  the bar), or its analyzer finds it invalid while its measures are the source's (a defect that
  moves nothing): a refusal, or a reading within the bar of either of OCCT's readings
  (`alt_volume`, `alt_area` the healed one);
* `occt` / `occt?` — OCCT reads a valid solid and its healing changes nothing: its reading, at the
  bar `table.own` gives a file read by one reader (its edges' slack, at least `table.ALONE`).

The gate is `step_mutants::no_mutant_is_read_wrong` (rust/gcs-core/tests/step_mutants.rs): every
mutant refused, or read to its row's solid within the bar; never a panic, never past the time
bound, never a solid from a `refuse` row.
"""
import json
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.dont_write_bytecode = True
sys.path.insert(0, HERE)
import oracle  # noqa: E402
import stepscan  # noqa: E402
import table  # noqa: E402

CORPUS = table.CORPUS
REPO = table.REPO
COMMITTED = os.path.join(REPO, "rust", "gcs-core", "tests", "step_mutants", "mutants.tsv")
SOURCES = 4
SMALL = 40000
TIMEOUT = 120.0
KINDS = ("face_sense", "oriented_edge", "edge_sense", "bound_orientation", "face_dropped",
         "face_twice", "vertices_swapped", "vertex_nudged", "vertex_moved", "radius_scaled",
         "direction_negated", "knot_moved", "weight_scaled", "pcurve_retargeted", "unit_changed",
         "uncertainty_huge", "uncertainty_zero", "wrong_type", "missing", "cycle")
# how many sites of a kind per source: the flips, cheap and many, twice
PER = {k: 2 for k in ("face_sense", "oriented_edge", "edge_sense", "bound_orientation",
                      "face_dropped", "vertices_swapped")}


# ---- the text, as entities whose parameters have spans --------------------------------------

class Node:
    """A parameter: `kind` one of ref, enum, str, num, unset, derived, list, typed; `span` its
    [start, end) in the text; `value` the referenced id, the enum's word, the number's text; `items`
    a list's or typed parameter's parameters."""
    __slots__ = ("kind", "span", "value", "items")

    def __init__(self, kind, span, value=None, items=None):
        self.kind, self.span, self.value, self.items = kind, span, value, items


class Entity:
    """`#id = …;`: its parts [(name, [Node])] (one for a simple entity), and the span of its
    right-hand side."""
    def __init__(self, id, parts, span):
        self.id, self.parts, self.span = id, parts, span

    def part(self, name):
        for n, ps in self.parts:
            if n == name:
                return ps
        return None

    def kinds(self):
        return [n for n, _ in self.parts]


def skip(text, i):
    """Past whitespace and comments from `i`."""
    n = len(text)
    while i < n:
        if text[i].isspace():
            i += 1
        elif text.startswith("/*", i):
            e = text.find("*/", i + 2)
            i = n if e < 0 else e + 2
        else:
            break
    return i


def param(text, i):
    i = skip(text, i)
    c = text[i]
    if c == "'":
        j = i + 1
        while True:
            if text[j] == "'":
                if text[j + 1:j + 2] == "'":
                    j += 2
                    continue
                break
            j += 1
        return Node("str", (i, j + 1)), j + 1
    if c == "#":
        j = i + 1
        while text[j].isdigit():
            j += 1
        return Node("ref", (i, j), int(text[i + 1:j])), j
    if c == ".":
        j = text.index(".", i + 1)
        return Node("enum", (i, j + 1), text[i + 1:j]), j + 1
    if c == "$":
        return Node("unset", (i, i + 1)), i + 1
    if c == "*":
        return Node("derived", (i, i + 1)), i + 1
    if c == "(":
        items, j = params(text, i)
        return Node("list", (i, j), items=items), j
    if c.isalpha() or c == "_":
        j = i
        while text[j].isalnum() or text[j] == "_":
            j += 1
        items, k = params(text, skip(text, j))
        return Node("typed", (i, k), text[i:j], items), k
    j = i
    while j < len(text) and (text[j].isalnum() or text[j] in "+-.") and text[j] not in ",)":
        j += 1
    return Node("num", (i, j), text[i:j]), j


def params(text, i):
    """The parameters of `( … )` at `i`, and the index past its `)`."""
    assert text[i] == "("
    out, i = [], skip(text, i + 1)
    if text[i] == ")":
        return out, i + 1
    while True:
        p, i = param(text, i)
        out.append(p)
        i = skip(text, i)
        if text[i] == ",":
            i += 1
            continue
        assert text[i] == ")", text[i - 40:i + 40]
        return out, i + 1


def entities(text):
    """The DATA section's entities by id."""
    start = text.index("DATA;") + len("DATA;")
    out, i, n = {}, start, len(text)
    while True:
        i = skip(text, i)
        if i >= n or text.startswith("ENDSEC", i):
            return out
        assert text[i] == "#", text[i:i + 60]
        j = i + 1
        while text[j].isdigit():
            j += 1
        id = int(text[i + 1:j])
        k = skip(text, j)
        assert text[k] == "="
        k = skip(text, k + 1)
        rhs = k
        parts = []
        if text[k] == "(":
            k = skip(text, k + 1)
            while text[k] != ")":
                m = k
                while text[m].isalnum() or text[m] == "_":
                    m += 1
                name = text[k:m]
                ps, k = params(text, skip(text, m))
                parts.append((name, ps))
                k = skip(text, k)
            k += 1
        else:
            m = k
            while text[m].isalnum() or text[m] == "_":
                m += 1
            ps, k = params(text, skip(text, m))
            parts.append((text[rhs:m], ps))
        out[id] = Entity(id, parts, (rhs, k))
        k = skip(text, k)
        assert text[k] == ";", text[k - 60:k + 20]
        i = k + 1


# ---- the edits --------------------------------------------------------------------------------

def edited(text, edits):
    """`text` with each (start, end, new) of `edits` (spans apart) put in."""
    out, at = [], 0
    for s, e, new in sorted(edits):
        out.append(text[at:s])
        out.append(new)
        at = e
    out.append(text[at:])
    return "".join(out)


def appended(text, line):
    """`text` with an entity line put in before the DATA section's ENDSEC."""
    d = text.index("DATA;")
    e = text.index("ENDSEC", d)
    return text[:e] + line + "\n" + text[e:]


def flip(node):
    return {"T": ".F.", "F": ".T."}.get(node.value)


def number(x):
    s = repr(float(x))
    return s.upper() if "e" in s else s


class Mutator:
    def __init__(self, text, size):
        self.text, self.size = text, size
        self.ents = entities(text)
        self.next = max(self.ents) + 1
        self.by = {}
        for e in self.ents.values():
            for k in e.kinds():
                self.by.setdefault(k, []).append(e)
        for v in self.by.values():
            v.sort(key=lambda e: e.id)

    def of(self, *kinds):
        return [e for k in kinds for e in self.by.get(k, [])]

    def sites(self, kind):
        """Every (entity, edit maker) a kind may edit, in the file's order."""
        t, out = self.text, []
        if kind in ("face_sense", "oriented_edge", "edge_sense", "bound_orientation"):
            name, at = {"face_sense": (("ADVANCED_FACE",), 3), "oriented_edge": (("ORIENTED_EDGE",), 4),
                        "edge_sense": (("EDGE_CURVE",), 4),
                        "bound_orientation": (("FACE_BOUND", "FACE_OUTER_BOUND"), 2)}[kind]
            for e in self.of(*name):
                p = e.parts[0][1]
                if len(p) > at and p[at].kind == "enum" and flip(p[at]):
                    out.append((e, lambda p=p[at]: (f".{p.value}. to {flip(p)}", [(*p.span, flip(p))])))
        elif kind in ("face_dropped", "face_twice"):
            for e in self.of("CLOSED_SHELL"):
                lst = e.parts[0][1][1]
                if lst.kind != "list" or len(lst.items) < 2:
                    continue
                for k, f in enumerate(lst.items):
                    if kind == "face_dropped":
                        def drop(lst=lst, k=k, f=f):
                            refs = [t[x.span[0]:x.span[1]] for i, x in enumerate(lst.items) if i != k]
                            return f"#{f.value} dropped", [(*lst.span, "(" + ",".join(refs) + ")")]
                        out.append((e, drop))
                    else:
                        out.append((e, lambda lst=lst, f=f: (f"#{f.value} listed twice",
                                                             [(lst.span[1] - 1, lst.span[1] - 1, f",#{f.value}")])))
        elif kind == "vertices_swapped":
            for e in self.of("EDGE_CURVE"):
                p = e.parts[0][1]
                if p[1].kind == "ref" and p[2].kind == "ref" and p[1].value != p[2].value:
                    out.append((e, lambda p=p: (f"#{p[1].value} and #{p[2].value} swapped",
                                                [(*p[1].span, f"#{p[2].value}"), (*p[2].span, f"#{p[1].value}")])))
        elif kind in ("vertex_nudged", "vertex_moved"):
            by = 1e-3 if kind == "vertex_nudged" else 0.5
            for e in self.of("VERTEX_POINT"):
                p = e.parts[0][1]
                pt = self.ents.get(p[1].value) if p[1].kind == "ref" else None
                if pt is None or pt.kinds() != ["CARTESIAN_POINT"]:
                    continue
                xs = pt.parts[0][1][1].items
                if len(xs) != 3:
                    continue
                out.append((e, lambda p=p, xs=xs, by=by: self.moved(p, xs, by)))
        elif kind == "radius_scaled":
            surfaces = [(e, 2) for e in self.of("CYLINDRICAL_SURFACE", "CONICAL_SURFACE",
                                                "SPHERICAL_SURFACE", "TOROIDAL_SURFACE")]
            for e, at in surfaces or [(e, 2) for e in self.of("CIRCLE")]:
                r = e.parts[0][1][at]
                if r.kind == "num":
                    out.append((e, lambda r=r: (f"radius {r.value} times 1.01",
                                                [(*r.span, number(float(r.value) * 1.01))])))
        elif kind == "direction_negated":
            used = set()
            for e in self.of("PLANE", "CYLINDRICAL_SURFACE", "CONICAL_SURFACE", "SPHERICAL_SURFACE",
                             "TOROIDAL_SURFACE"):
                a = self.ents.get(e.parts[0][1][1].value)
                if a is not None and a.kinds() == ["AXIS2_PLACEMENT_3D"]:
                    used.update(x.value for x in a.parts[0][1][2:] if x.kind == "ref")
            dirs = [d for d in self.of("DIRECTION") if d.id in used] or self.of("DIRECTION")
            for d in dirs:
                xs = d.parts[0][1][1]
                if xs.kind == "list" and all(x.kind == "num" for x in xs.items):
                    out.append((d, lambda xs=xs: ("negated", [(*xs.span, "(" + ",".join(
                        number(-float(x.value)) for x in xs.items) + ")")])))
        elif kind == "knot_moved":
            for e in self.ents.values():
                for name, ps in e.parts:
                    if not name.endswith("_WITH_KNOTS"):
                        continue
                    # a curve's knots are its second list from the end but one, a surface's the
                    # two lists before the spec: (…, mults, knots, spec) or (…, um, vm, uk, vk, spec)
                    lists = [x for x in ps if x.kind == "list"
                             and all(i.kind == "num" for i in x.items) and x.items]
                    surface = name.startswith("B_SPLINE_SURFACE")
                    for ks in (lists[-2:] if surface else lists[-1:]):
                        vals = [float(x.value) for x in ks.items]
                        for i in range(1, len(vals) - 1):
                            if vals[i - 1] < vals[i] < vals[i + 1]:
                                new = vals[i] + 0.25 * (vals[i + 1] - vals[i])
                                out.append((e, lambda x=ks.items[i], new=new: (
                                    f"knot {x.value} to {number(new)}", [(*x.span, number(new))])))
        elif kind == "weight_scaled":
            for e in self.ents.values():
                for name, ps in e.parts:
                    if name in ("RATIONAL_B_SPLINE_CURVE", "RATIONAL_B_SPLINE_SURFACE") and ps:
                        flat = [x for x in ps[0].items] if name.endswith("CURVE") else \
                            [y for x in ps[0].items for y in x.items]
                        for x in flat:
                            if x.kind == "num":
                                out.append((e, lambda x=x: (f"weight {x.value} times 1.5",
                                                            [(*x.span, number(float(x.value) * 1.5))])))
        elif kind == "pcurve_retargeted":
            surfaces = [e.id for e in self.of("PLANE", "CYLINDRICAL_SURFACE", "CONICAL_SURFACE",
                                              "SPHERICAL_SURFACE", "TOROIDAL_SURFACE",
                                              "B_SPLINE_SURFACE_WITH_KNOTS", "B_SPLINE_SURFACE",
                                              "SURFACE_OF_REVOLUTION", "SURFACE_OF_LINEAR_EXTRUSION")]
            for e in self.of("PCURVE"):
                s = e.parts[0][1][1]
                others = [x for x in surfaces if x != s.value]
                if s.kind == "ref" and others:
                    out.append((e, lambda s=s, others=others: self.retarget(s, others, "surface")))
        elif kind == "unit_changed":
            for e in self.ents.values():
                if "LENGTH_UNIT" not in e.kinds():
                    continue
                si = e.part("SI_UNIT")
                if si and len(si) == 2 and si[1].kind == "enum" and si[1].value == "METRE":
                    new = "$" if si[0].kind == "enum" and si[0].value == "MILLI" else ".MILLI."
                    out.append((e, lambda si=si, new=new: (
                        f"length unit prefix {t[si[0].span[0]:si[0].span[1]]} to {new}",
                        [(*si[0].span, new)])))
                cb = e.part("CONVERSION_BASED_UNIT")
                if cb and len(cb) == 2 and cb[1].kind == "ref":
                    m = self.ents.get(cb[1].value)
                    v = m.parts[0][1][0] if m is not None and m.parts[0][1] else None
                    if v is not None and v.kind == "typed" and v.items and v.items[0].kind == "num":
                        x = v.items[0]
                        out.append((m, lambda x=x: (f"conversion factor {x.value} times 10",
                                                    [(*x.span, number(float(x.value) * 10))])))
        elif kind in ("uncertainty_huge", "uncertainty_zero"):
            for e in self.of("UNCERTAINTY_MEASURE_WITH_UNIT"):
                v = e.parts[0][1][0]
                if v.kind == "typed" and v.items and v.items[0].kind == "num":
                    x = v.items[0]
                    new = "0." if kind == "uncertainty_zero" else number(self.size / 2 / self.scale())
                    out.append((e, lambda x=x, new=new: (f"uncertainty {x.value} to {new}", [(*x.span, new)])))
        elif kind == "wrong_type":
            dirs = [d.id for d in self.of("DIRECTION")]
            verts = [d.id for d in self.of("VERTEX_POINT")]
            faces = [d.id for d in self.of("ADVANCED_FACE")]
            for e in self.of("ADVANCED_FACE"):
                if dirs:
                    out.append((e, lambda s=e.parts[0][1][2]: self.retarget(s, dirs, "DIRECTION")))
            for e in self.of("EDGE_CURVE"):
                if verts:
                    out.append((e, lambda s=e.parts[0][1][3]: self.retarget(s, verts, "VERTEX_POINT")))
            for e in self.of("ORIENTED_EDGE"):
                if faces:
                    out.append((e, lambda s=e.parts[0][1][3]: self.retarget(s, faces, "ADVANCED_FACE")))
        elif kind == "missing":
            gone = self.next + 1000
            for e in self.of("ADVANCED_FACE", "EDGE_CURVE", "ORIENTED_EDGE", "VERTEX_POINT",
                             "EDGE_LOOP", "FACE_BOUND", "FACE_OUTER_BOUND"):
                refs = [x for x in walk(e.parts[0][1]) if x.kind == "ref"]
                for r in refs:
                    out.append((e, lambda r=r: (f"#{r.value} to #{gone}, no entity",
                                                [(*r.span, f"#{gone}")])))
        elif kind == "cycle":
            for e in self.of("ADVANCED_FACE"):
                s = e.parts[0][1][2]
                out.append((e, lambda s=s, e=e: (f"its surface #{s.value} to itself", [(*s.span, f"#{e.id}")])))
        return out

    def retarget(self, ref, choices, what):
        new = choices[self.rng.randrange(len(choices))]
        return f"#{ref.value} to #{new} ({what})", [(*ref.span, f"#{new}")]

    def scale(self):
        """Millimetres per the file's unit of length (for an edit made in the file's units)."""
        u = stepscan.units({k: (e.kinds(), [], self.text[e.span[0]:e.span[1]])
                            for k, e in self.ents.items()}, self.text).split(",")[0]
        return {"mm": 1.0, "m": 1000.0, "cm": 10.0, "inch": 25.4}.get(u, 1.0)

    def moved(self, p, xs, by):
        d = [self.rng.gauss(0, 1) for _ in range(3)]
        n = sum(x * x for x in d) ** 0.5 or 1.0
        step = by * self.size / self.scale()
        new = [float(x.value) + step * c / n for x, c in zip(xs, d)]
        id = self.next
        line = f"#{id}=CARTESIAN_POINT('',({','.join(number(x) for x in new)}));"
        return (f"its point #{p[1].value} moved {by:g} of the diagonal to #{id}",
                [(*p[1].span, f"#{id}")], line)

    def mutants(self, kind, seed, count):
        """Up to `count` mutants of `kind`: [(entity, what, text)]."""
        self.rng = random.Random(seed)
        sites = self.sites(kind)
        out = []
        for k in self.rng.sample(range(len(sites)), min(count, len(sites))):
            e, make = sites[k]
            got = make()
            what, edits = got[0], got[1]
            text = edited(self.text, [(s, z, new) for s, z, new in edits])
            if len(got) > 2:
                text = appended(text, got[2])
            out.append((e.id, what, text))
        return out


def walk(nodes):
    for n in nodes:
        yield n
        if n.items:
            yield from walk(n.items)


# ---- the steps -----------------------------------------------------------------------------

def rows():
    lines = [l.rstrip("\n").split("\t") for l in open(table.COMMITTED) if not l.startswith("#")]
    return [dict(zip(lines[0], r)) for r in lines[1:]]


def sources():
    """The files mutated: per writer up to SOURCES small files OCCT reads valid and unhealed, each
    bringing surface kinds the writer's earlier picks do not where one can."""
    by = {}
    for r in rows():
        if r["valid"] != "1" or r["healed"] != "-" or r["ref_from"] not in ("closed", "mass+occt", "occt"):
            continue
        if int(r["bytes"]) > SMALL or r["source"].startswith("variants"):
            continue
        # OCCT's reading of the source its reference's (its oracle on the mutants is no better)
        if not (table.rel(float(r["occt_volume"]), float(r["ref_volume"])) <= float(r["bar_volume"])
                and table.rel(float(r["occt_area"]), float(r["ref_area"])) <= float(r["bar_area"])):
            continue
        writer = r["source"] if r["source"].startswith("brep-bench") else r["source"].split("/")[0]
        by.setdefault(writer, []).append(r)
    out = []
    for writer, rs in sorted(by.items()):
        rs = [(r, json.load(open(os.path.join(CORPUS, "oracle", r["path"] + ".json")))) for r in rs
              if int(r["bytes"]) > 1500]
        # the files of most surface kinds first, the smaller of those first
        rs.sort(key=lambda x: (-len(x[1]["surfaces"]), int(x[0]["bytes"]), x[0]["path"]))
        seen, picked = set(), []
        for r, o in rs:
            if set(o["surfaces"]) - seen and len(picked) < SOURCES:
                picked.append((r, o))
                seen |= set(o["surfaces"])
        for r, o in rs:
            if len(picked) < SOURCES and all(r is not p[0] for p in picked):
                picked.append((r, o))
        out.extend(picked)
    return out


def diagonal(o):
    b = o["bbox"]
    return sum((b[i + 3] - b[i]) ** 2 for i in range(3)) ** 0.5


def make():
    root = os.path.join(CORPUS, "mutants")
    manifest = ["path\tmutant_of\tmutation\tentity\tedit"]
    for r, o in sources():
        text = open(os.path.join(CORPUS, r["path"]), encoding="latin-1", newline="").read()
        m = Mutator(text, diagonal(o))
        flat = r["path"].rsplit(".", 1)[0].replace("/", "__")
        for kind in KINDS:
            for k, (ent, what, mutant) in enumerate(m.mutants(kind, f"{r['path']}:{kind}", PER.get(kind, 1))):
                path = f"mutants/{kind}/{flat}.{k}.step"
                os.makedirs(os.path.dirname(os.path.join(CORPUS, path)), exist_ok=True)
                with open(os.path.join(CORPUS, path), "w", encoding="latin-1", newline="") as fh:
                    fh.write(mutant)
                manifest.append(f"{path}\t{r['path']}\t{kind}\t{ent}\t{what}")
    with open(os.path.join(root, "manifest.tsv"), "w") as fh:
        fh.write("\n".join(manifest) + "\n")
    print(f"{len(manifest) - 1} mutants of {len(set(l.split(chr(9))[1] for l in manifest[1:]))} "
          f"files in {root}")


def manifest():
    lines = [l.rstrip("\n").split("\t") for l in open(os.path.join(CORPUS, "mutants", "manifest.tsv"))]
    return [dict(zip(lines[0], l)) for l in lines[1:]]


def run_oracle(jobs):
    oracle.run_corpus(CORPUS, jobs, TIMEOUT, [], files=[m["path"] for m in manifest()])


def stated(o):
    """The uncertainty a file states, in millimetres (its largest, in its first length unit)."""
    u = (o.get("units") or "mm").split(",")[0]
    scale = {"mm": 1.0, "millimetre": 1.0, "m": 1000.0, "cm": 10.0, "inch": 25.4}.get(u, 1.0)
    vals = [float(x) for x in (o.get("uncertainty") or "").split(",") if x]
    return max(vals, default=0.0) * scale


def reference(m, o, src):
    """(ref_from, volume, area, bar_volume, bar_area, alt, note) for a mutant read so by OCCT. The
    bar is the one OCCT's reading alone has (`table.own`), but no less than the source's, nor than
    AGREE times how far OCCT's reading of the source stands from the source's reference (OCCT's
    own error on that geometry, which a mutant of it shares)."""
    if "error" in o:
        return "refuse", None, None, None, None, None, "OCCT: " + o["error"][-120:]
    if o.get("solids", 0) == 0 or not (o.get("volume", 0) > 0):
        return "refuse", None, None, None, None, None, \
            f"OCCT reads {o.get('solids', 0)} solids, volume {o.get('volume')!r}"
    sv, sa = float(src["ref_volume"]), float(src["ref_area"])
    floor_v = max(float(src["bar_volume"]), table.AGREE * table.rel(float(src["occt_volume"]), sv))
    floor_a = max(float(src["bar_area"]), table.AGREE * table.rel(float(src["occt_area"]), sa))
    v, a, frm, bv, ba = table.own(o, None)
    # OCCT's tolerances beyond the source's and beyond what the mutant states: the slack OCCT had
    # to give the edit (a vertex off its edges, a curve off its surface)
    raised = o["tol_edge"] > 1.001 * max(float(src["tol_edge"]), stated(o))
    if not raised:
        # the tolerance is the file's own statement: no slack of the geometry's, the floor's bar
        bv, ba = max(table.ALONE, table.AGREE * o["spread_volume"]), max(table.ALONE, table.AGREE * o["spread_area"])
    bv, ba = max(bv, floor_v), max(ba, floor_a)
    near = table.rel(v, sv) <= bv and table.rel(a, sa) <= ba
    if raised and max(bv, ba) > table.CAP:
        return "refuse", None, None, None, None, None, \
            f"OCCT reads it only with its edges' tolerance raised to {o['tol_edge']:.3g}"
    healed = o.get("healed", {})
    hv, ha = o.get("healed_volume"), o.get("healed_area")
    if "valid" in healed or table.rel(hv, v) > bv or table.rel(ha, a) > ba:
        return "disputed", v, a, bv, ba, (hv, ha), "OCCT's healing reads it otherwise: " + \
            ";".join(f"{k} {x}" for k, x in sorted(healed.items()))
    if not o["valid"]:
        if near:
            return "disputed", v, a, bv, ba, (hv, ha), "OCCT's analyzer finds it invalid, its measures the source's"
        return "refuse", None, None, None, None, None, "OCCT's analyzer finds it invalid, its measures moved"
    tol = f"; OCCT's edges' tolerance raised to {o['tol_edge']:.3g}" if raised else ""
    return frm, v, a, bv, ba, None, ("the source's solid" if near else "another solid") + tol


def write_table():
    src = {r["path"]: r for r in rows()}
    cols = table.COLUMNS + ["mutation", "mutant_of", "edit"]
    out, missing = [], 0
    for m in manifest():
        path = m["path"]
        try:
            o = json.load(open(os.path.join(CORPUS, "oracle", path + ".json")))
        except (OSError, ValueError):
            missing += 1
            continue
        raw = open(os.path.join(CORPUS, path), "rb").read()
        sha = stepscan.data_sha256(raw)
        if o.get("data_sha256") != sha:
            missing += 1
            continue
        s = stepscan.scan(os.path.join(CORPUS, path))
        frm, v, a, bv, ba, alt, note = reference(m, o, src[m["mutant_of"]])
        row = {c: "" for c in cols}
        row.update(path=path, bytes=len(raw), data_sha256=sha, source="mutants/" + m["mutation"],
                   case=m["mutant_of"], ap=s["ap"], units=s["units"] or "?",
                   uncertainty=s["uncertainty"] or "-", solids=len(s["solids"]),
                   step_faces=sum(x["faces"] for x in s["solids"]),
                   step_edges=sum(x["edges"] for x in s["solids"]),
                   step_vertices=sum(x["vertices"] for x in s["solids"]), healed="-",
                   mass_error="not measured", ref_from=frm, note=note, mutation=m["mutation"],
                   mutant_of=m["mutant_of"], edit=f"#{m['entity']}: {m['edit']}")
        if "error" not in o:
            row.update(occt_faces=o["faces"], occt_edges=o["edges"], seams=o["seams"],
                       degenerate=o["degenerate"], occt_vertices=o["vertices"], voids=o["voids"],
                       valid=int(o["valid"]), tol_edge=f"{o['tol_edge']:.3g}",
                       occt_volume=o["volume"], occt_area=o["area"],
                       occt_spread=f"{o['spread_volume']:.2g},{o['spread_area']:.2g}",
                       healed=";".join(f"{k} {x}" for k, x in sorted(o["healed"].items())) or "-")
        if v is not None:
            row.update(ref_volume=v, ref_area=a, bar_volume=f"{bv:.2g}", bar_area=f"{ba:.2g}")
        if alt:
            row.update(alt_volume=alt[0], alt_area=alt[1])
        out.append(row)
    head = (f"# mutants of the STEP corpus (scripts/step-corpus/mutants.py): {len(out)} files; "
            f"bars relative\n")
    text = head + "\t".join(cols) + "\n" + "".join(
        "\t".join(table.g(r[c]) for c in cols) + "\n" for r in out)
    os.makedirs(os.path.dirname(COMMITTED), exist_ok=True)
    for f in (COMMITTED, os.path.join(CORPUS, "mutants.tsv")):
        with open(f, "w") as fh:
            fh.write(text)
    tally = {}
    for r in out:
        tally.setdefault(r["mutation"], {}).setdefault(r["ref_from"], 0)
        tally[r["mutation"]][r["ref_from"]] += 1
    print(f"{len(out)} rows ({missing} mutants not read by the oracle yet)")
    for k in KINDS:
        print(f"  {k:<20} {tally.get(k, {})}")


def main():
    args = sys.argv[1:]
    if args[:1] == ["make"]:
        make()
    elif args[:1] == ["oracle"]:
        run_oracle(int(args[args.index("--jobs") + 1]) if "--jobs" in args else 2)
    elif args[:1] == ["table"]:
        write_table()
    else:
        print(__doc__)


if __name__ == "__main__":
    main()
