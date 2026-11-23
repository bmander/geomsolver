"""A STEP file (ISO 10303-21) read for what its text says, apart from any kernel: its schema, its
units and uncertainty, its entities by kind (a complex entity's parts each counted), and per solid
the topological entities under it. Standard library only; imported by oracle.py and census.py.

    python3 -I scripts/step-corpus/stepscan.py FILE.step     # prints the scan as JSON

Nothing here judges geometry: it counts what the text names, so that the oracle's counts of what
OCCT reads can be set beside the file's own (OCCT adds edges the file does not write, a pole's
degenerate edge, and can split or merge where it heals).
"""
import hashlib
import json
import re
import sys

ENTITY = re.compile(r"#(\d+)\s*=\s*", re.S)
REF = re.compile(r"#(\d+)")
NAME = re.compile(r"([A-Z_][A-Z0-9_]*)\s*\(")
SOLIDS = ("MANIFOLD_SOLID_BREP", "BREP_WITH_VOIDS")
SHELLS = ("CLOSED_SHELL", "ORIENTED_CLOSED_SHELL", "OPEN_SHELL")
FACES = ("ADVANCED_FACE", "FACE_SURFACE")
EDGES = ("EDGE_CURVE",)
VERTICES = ("VERTEX_POINT",)
LOOPS = ("EDGE_LOOP", "VERTEX_LOOP", "POLY_LOOP")


def data_sha256(raw):
    """The SHA-256 of the DATA section's bytes: the file's content without its header, which a
    writer stamps with the time it wrote (so a file written again with the same solid hashes the
    same)."""
    i = raw.find(b"\nDATA;")
    return hashlib.sha256(raw[i if i >= 0 else 0:]).hexdigest()


def strip_strings(s):
    """`s` with each quoted string emptied (a string may hold `;`, `#` or parentheses)."""
    out, i, n = [], 0, len(s)
    while i < n:
        c = s[i]
        if c == "'":
            j = i + 1
            while j < n:
                if s[j] == "'":
                    if j + 1 < n and s[j + 1] == "'":
                        j += 2
                        continue
                    break
                j += 1
            out.append("''")
            i = j + 1
        else:
            out.append(c)
            i += 1
    return "".join(out)


def statements(text):
    """The DATA section's statements, strings emptied and comments dropped."""
    text = re.sub(r"/\*.*?\*/", "", strip_strings(text), flags=re.S)
    m = re.search(r"\bDATA\s*(\([^;]*\))?\s*;", text)
    body = text[m.end():] if m else text
    e = body.find("ENDSEC")
    if e >= 0:
        body = body[:e]
    return body.split(";")


def kinds_of(rhs):
    """The entity kinds a right-hand side names: one for a simple entity, each part's for a complex
    one `( A(…) B(…) )`."""
    rhs = rhs.strip()
    if not rhs.startswith("("):
        m = NAME.match(rhs)
        return [m.group(1)] if m else []
    names, depth, i = [], 0, 1
    while i < len(rhs):
        c = rhs[i]
        if c == "(":
            depth += 1
        elif c == ")":
            depth -= 1
        elif depth == 0:
            m = NAME.match(rhs, i)
            if m:
                names.append(m.group(1))
                i = m.end() - 1  # at the part's `(`, which the next turn counts
                continue
        i += 1
    return names


def header(text):
    """The header's schema and the writer it names (FILE_NAME's originating system)."""
    m = re.search(r"FILE_SCHEMA\s*\(\s*\(\s*'([^']*)'", text)
    schema = m.group(1).strip() if m else ""
    m = re.search(r"FILE_NAME\s*\((.*?)\)\s*;", text, re.S)
    system = ""
    if m:
        strings = re.findall(r"'((?:[^']|'')*)'", m.group(1))
        # FILE_NAME(name, time_stamp, (author), (organization), preprocessor_version,
        # originating_system, authorisation): the strings in that order, an empty list giving none
        if len(strings) >= 6:
            system = strings[-2]
    return schema, system


def ap(schema):
    """The application protocol a schema name is."""
    s = schema.upper()
    if "AP242" in s or "MANAGED_MODEL_BASED_3D_ENGINEERING" in s or "10303 442" in s:
        return "AP242"
    if "AUTOMOTIVE_DESIGN" in s or "10303 214" in s:
        return "AP214"
    if "CONFIG_CONTROL_DESIGN" in s or "AP203" in s or "10303 203" in s:
        return "AP203"
    return schema or "?"


def entities(path):
    """The DATA section's entity labels, in the order its instances are listed (the order a
    reader's model numbers them in)."""
    text = open(path, "rb").read().decode("latin-1")
    for st in statements(text):
        m = ENTITY.match(st.strip())
        if m:
            yield int(m.group(1))


def scan(path):
    raw = open(path, "rb").read()
    text = raw.decode("latin-1")
    schema, system = header(text[:20000])
    ents = {}
    for st in statements(text):
        m = ENTITY.match(st.strip())
        if not m:
            continue
        rhs = st.strip()[m.end():]
        ents[int(m.group(1))] = (kinds_of(rhs), [int(r) for r in REF.findall(rhs)], rhs)
    census = {}
    for kinds, _, _ in ents.values():
        for k in kinds:
            census[k] = census.get(k, 0) + 1

    def under(root):
        seen, stack = set(), [root]
        while stack:
            e = stack.pop()
            if e in seen or e not in ents:
                continue
            seen.add(e)
            stack.extend(ents[e][1])
        return seen

    def of(ids, kinds):
        return sorted(e for e in ids if any(k in kinds for k in ents[e][0]))

    solids = []
    for e in sorted(ents):
        if any(k in SOLIDS for k in ents[e][0]):
            ids = under(e)
            solids.append({"entity": e, "kind": ents[e][0][0],
                           "shells": len(of(ids, SHELLS)), "faces": len(of(ids, FACES)),
                           "loops": len(of(ids, LOOPS)), "edges": len(of(ids, EDGES)),
                           "vertices": len(of(ids, VERTICES)),
                           "face_list": face_list(ents, e)})
    return {"bytes": len(raw), "sha256": hashlib.sha256(raw).hexdigest(),
            "data_sha256": data_sha256(raw), "schema": schema, "ap": ap(schema),
            "writer": system, "entities": len(ents), "census": dict(sorted(census.items())),
            "solids": solids, "units": units(ents, text), "uncertainty": uncertainty(ents)}


def face_list(ents, solid):
    """A solid's faces in the order its shells list them (outer shell, then each void), each as
    (`#n`, its surface's kind): the order OCCT's reader makes them in."""
    out = []

    def shell(s, seen):
        if s in seen or s not in ents:
            return
        seen.add(s)
        kinds, refs, _ = ents[s]
        if "ORIENTED_CLOSED_SHELL" in kinds:
            for r in refs:
                shell(r, seen)
            return
        for f in refs:
            if f in ents and any(k in FACES for k in ents[f][0]):
                surf = [r for r in ents[f][1] if r in ents and not any(
                    k in ("FACE_BOUND", "FACE_OUTER_BOUND") for k in ents[r][0])]
                kind = ents[surf[-1]][0] if surf else ["?"]
                out.append((f, surface_kind(kind)))
    seen = set()
    for r in ents[solid][1]:
        shell(r, seen)
    return out


SURFACE_KINDS = {"PLANE": "plane", "CYLINDRICAL_SURFACE": "cylinder", "CONICAL_SURFACE": "cone",
                 "SPHERICAL_SURFACE": "sphere", "TOROIDAL_SURFACE": "torus",
                 "DEGENERATE_TOROIDAL_SURFACE": "torus", "SURFACE_OF_REVOLUTION": "revolution",
                 "SURFACE_OF_LINEAR_EXTRUSION": "extrusion", "OFFSET_SURFACE": "offset"}


def surface_kind(kinds):
    """A surface entity's kind, in the oracle's words (`bspline` for every B-spline or Bézier
    surface, rational or not)."""
    for k in kinds:
        if k in SURFACE_KINDS:
            return SURFACE_KINDS[k]
    if any(k.startswith("B_SPLINE_SURFACE") or k.startswith("BEZIER_SURFACE")
           or k in ("UNIFORM_SURFACE", "QUASI_UNIFORM_SURFACE", "RATIONAL_B_SPLINE_SURFACE")
           for k in kinds):
        return "bspline"
    return kinds[0].lower() if kinds else "?"


def units(ents, text=""):
    """The length unit the file's representations name: `mm`, `m`, `inch`… (SI_UNIT's prefix and
    name, or a CONVERSION_BASED_UNIT's name), each one the file names, joined by `,`."""
    seen = []
    for kinds, _, rhs in ents.values():
        if "LENGTH_UNIT" not in kinds:
            continue
        m = re.search(r"CONVERSION_BASED_UNIT\s*\(", rhs)
        if m:
            # the strings were emptied for parsing: the unit's name is read from the text
            named = re.findall(r"CONVERSION_BASED_UNIT\s*\(\s*'([^']*)'", text)
            name = ",".join(sorted({n.lower() for n in named})) or "conversion"
        else:
            m = re.search(r"SI_UNIT\s*\(\s*(\$|\.(\w+)\.)\s*,\s*\.(\w+)\.", rhs)
            name = ((m.group(2) or "") + m.group(3)).lower() if m else "?"
            name = {"millimetre": "mm", "metre": "m", "centimetre": "cm", "micrometre": "um"}.get(
                name.replace("milli", "milli"), name)
        if name not in seen:
            seen.append(name)
    return ",".join(seen)


def uncertainty(ents):
    """The length uncertainties the file states (UNCERTAINTY_MEASURE_WITH_UNIT's values, in the
    file's units), the smallest first, joined by `,`."""
    vals = set()
    for kinds, _, rhs in ents.values():
        if "UNCERTAINTY_MEASURE_WITH_UNIT" in kinds:
            m = re.search(r"LENGTH_MEASURE\s*\(\s*([-+0-9.EeDd]+)", rhs)
            if m:
                vals.add(float(m.group(1).replace("D", "E")))
    return ",".join(f"{v:g}" for v in sorted(vals))


if __name__ == "__main__":
    for p in sys.argv[1:]:
        print(json.dumps(scan(p), indent=1))
