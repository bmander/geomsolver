#!/usr/bin/env python3
"""The corpus's reference table: a row a file, what the gate test (rust/gcs-core/tests/
step_corpus.rs) holds the kernel's reading of it to.

    python3 -I scripts/step-corpus/table.py measure [--jobs 2]   # brep-bench's measurer, each file
    python3 -I scripts/step-corpus/table.py                      # write the table

`measure` runs brep-bench's own measurer (`harness/mass`, built by `brep_bench.py build mass`) over
every file of `$STEP_CORPUS`, into `oracle/<path>.mass.json`: a second reader of the file's exact
geometry, apart from OCCT, which reads each edge's curve in space onto its faces' surfaces and not
the file's curves in their parameters (as the kernel's reader does), and refuses a loop that does
not close on its surface rather than guess.

The table is written to `rust/gcs-core/tests/step_corpus/reference.tsv` (committed) and to
`$STEP_CORPUS/reference.tsv`; `--out FILE` writes FILE instead. A file the oracle could not read
to a solid (`oracle.py` failed, or OCCT reads no solid) has no row. Columns: `path` under the
corpus; `bytes` and `data_sha256` (the DATA section's, not the header's time stamp); `source`
(`cadgenbench`, `brep-bench/<kernel>`, `variants/<habit>`); `case`; the text's `ap`, `units`,
`uncertainty`; `solids` and the entities they hold, `step_faces`, `step_edges`, `step_vertices`
(each once: what `Given.faces`, `edges` and `vertices` name); OCCT's reading with healing off:
`occt_faces`, `occt_edges` (`seams` and `degenerate` among them), `occt_vertices`, `voids`,
`valid`, `healed` (what healing changed, `-` for nothing), `tol_edge`, `occt_volume`,
`occt_area`, `occt_spread` (volume's, area's); `mass_volume`, `mass_area` (or `mass_error`); and
the reference: `ref_volume`, `ref_area`, `ref_from`, `bar_volume`, `bar_area` (relative), `note`; and for a disputed file,
`alt_volume`, `alt_area`, the second reading it may be read to.

**The reference and its bar**, per file (`reference`):

* a brep-bench file that holds its case's solid, as the bench itself grades it (its measurer's
  volume and area within the case's tolerance of the closed forms; where the measurer cannot read
  the file, OCCT's converged reading in its place): the closed forms, at the bench's own grade
  (`closed`, the case's tolerance, else 1e-6);
* otherwise, the file's own solid as two independent readers find it. OCCT's reading counts as one
  only where it has converged: its adaptive integration at 1e-6, 1e-8 and 1e-10 agreeing within
  `CONVERGED` (on some files they differ by 1e-3, and its fixed rule, a call with no tolerance, by
  up to 7e-2: OCCT's measure is no gate where it has not converged). Where both read it, the
  reference is the bench measurer's (exact geometry, about 1e-12 of the solid's size) and the bar
  `AGREE` times their difference, but no less than `FLOOR`; where one alone does, its reading,
  uncorroborated, at the file's slack: `SLACK` times its largest edge tolerance (mm, as OCCT reads
  it: the file's stated uncertainty, or more where an edge needs it) times its area over its
  volume, no less than `ALONE`; where neither does (the measurer refuses and OCCT has not
  converged), OCCT's finest at `AGREE` times its own spread, no less than the slack (`ref_from`
  `occt?`). Where the two disagree past the slack and OCCT's spread together, the measurer is set
  aside and OCCT's finest taken so (`occt!`): on cadgenbench/243 the measurer's area is 2.4% off
  OCCT's, which has converged to 3e-8 (a misread face of the measurer's, not the file's slack).
  A brep-bench file so disputed is `disputed` instead: OCCT's reading the reference, the
  measurer's the alternative (`alt_volume`, `alt_area`), and a refusal, or a reading within the
  bar of either, is accepted. Those files are mostly ones whose loops run against ISO 10303-42
  (which OCCT repairs silently and the bench's measurer, reading by the standard, does not:
  SPEC.md, "STEP is the answer"), and some the measurer misreads. The slack is calibrated on the CADGenBench files both read: their readings differ by
  up to 0.04 of tolerance × area / volume (cadgenbench/242: 2.1e-4 in volume, at tolerances of
  0.02 mm), the gaps a file's uncertainty leaves between an edge's curve and its faces read two
  ways; `SLACK` leaves a margin of two and a half over that.
* a file OCCT reads to no solid of positive volume (`unread`): `refuse` where its text holds no
  solid (the reader must refuse it); else the bench measurer's reading alone, at `UNCORROBORATED`
  (or the closed forms where that reading holds them);
* no row where the bar would pass `CAP` (a part in fifty): a file OCCT reads only by raising its
  tolerances that far is no reference;
* a variant: its source's reference and bar (the solid written is the source's), widened to
  `AGREE` times how far the readers of the variant stand from it, where they stand farther (the
  writer's round trip: inches written to 13 digits, a solid OCCT itself read and wrote).
"""
import json
import math
import os
import subprocess
import sys
import time

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import oracle  # noqa: E402
import stepscan  # noqa: E402

REPO = os.path.dirname(os.path.dirname(HERE))
CORPUS = os.environ.get("STEP_CORPUS", "/home/user/step-corpus")
COMMITTED = os.path.join(REPO, "rust", "gcs-core", "tests", "step_corpus", "reference.tsv")
MANIFEST = os.path.join(REPO, "rust", "gcs-core", "tests", "step_corpus", "brep_bench_cases.tsv")
ACCOUNT = os.path.join(REPO, "rust", "gcs-core", "tests", "step_corpus", "account.tsv")
MASS = os.path.join(CORPUS, "build", "brep-bench", "harness", "mass", "target", "release", "mass")
CONVERGED = 1e-6
AGREE = 3.0
FLOOR = 1e-7
ALONE = 1e-6
SLACK = 0.1
UNCORROBORATED = 1e-5
CAP = 0.02
EXACT = 1e-6

COLUMNS = ("path bytes data_sha256 source case ap units uncertainty solids step_faces step_edges "
           "step_vertices occt_faces occt_edges seams degenerate occt_vertices voids valid healed "
           "tol_edge occt_volume occt_area occt_spread mass_volume mass_area mass_error ref_volume "
           "ref_area ref_from bar_volume bar_area note alt_volume alt_area verdict unchecked "
           "account").split()
VERDICTS = ("gap", "open", "defective", "assembly", "nosolid", "disputed", "reference", "bar",
            "unchecked")


def measure(jobs, timeout=120.0):
    files = oracle.corpus_files(CORPUS)
    todo = []
    for f in files:
        js = os.path.join(CORPUS, "oracle", f + ".mass.json")
        sha = stepscan.data_sha256(open(os.path.join(CORPUS, f), "rb").read())
        if os.path.exists(js):
            try:
                if json.load(open(js)).get("data_sha256") == sha:
                    continue
            except ValueError:
                pass
        todo.append((f, sha))
    print(f"{len(files)} files, {len(todo)} to measure, {jobs} at once", flush=True)
    running = []

    def reap():
        for item in list(running):
            p, f, sha, start = item
            killed = p.poll() is None and time.time() - start > timeout
            if killed:
                p.kill()
            if p.poll() is None:
                continue
            running.remove(item)
            out, err = p.communicate()
            got = None
            for line in reversed(out.strip().splitlines()):
                try:
                    got = json.loads(line)
                    break
                except ValueError:
                    continue
            if killed:
                got = {"error": f"over {timeout:g} s"}
            elif got is None:
                got = {"error": (err.strip() or f"exit {p.returncode}")[-300:]}
            got = {k: got[k] for k in ("volume", "area", "centroid", "error") if k in got}
            got["data_sha256"] = sha
            js = os.path.join(CORPUS, "oracle", f + ".mass.json")
            os.makedirs(os.path.dirname(js), exist_ok=True)
            with open(js, "w") as fh:
                json.dump(got, fh)
    for f, sha in todo:
        while len(running) >= jobs:
            reap()
            time.sleep(0.02)
        p = subprocess.Popen([MASS, os.path.join(CORPUS, f)], stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE, text=True,
                             env={**os.environ, "MASS_THREADS": "1"})
        running.append((p, f, sha, time.time()))
    while running:
        reap()
        time.sleep(0.02)


def rel(a, b):
    return abs(a - b) / abs(b)


def cases():
    out = {}
    for line in open(MANIFEST):
        if line.startswith("#") or line.startswith("case\t"):
            continue
        f = line.rstrip("\n").split("\t")
        out[f[0]] = {"volume": float(f[1]), "area": float(f[2]), "tolerance": float(f[3]),
                     "measured": [k for k in (f[6] if len(f) > 6 else "").split(",") if k]}
    return out


def account():
    """The account of the files not read right (`step_corpus/account.tsv`, made by hand from
    the evidence `check.py` and the readers give): path → (verdict, the faces whose one reference
    is unchecked, why). Its verdicts: `gap` (the reader's: a sound file another reader reads to
    the reference), `defective` (not a solid's boundary by ISO 10303-42, or its edges past
    `sound::LOOSE` off its faces: must be refused), `assembly` (product structure, deferred by the
    stage), `nosolid`, `disputed` (the readers read it differently; the row says between whom),
    `reference` and `bar` (the row's reference or bar was corrected, the why says from what),
    `unchecked` (faces with one reference no second reader has checked), `open` (the evidence in
    hand does not settle it)."""
    out = {}
    if not os.path.exists(ACCOUNT):
        return out
    for line in open(ACCOUNT, encoding="utf-8"):
        if line.startswith("#") or line.startswith("path\t") or not line.strip():
            continue
        f = line.rstrip("\n").split("\t")
        if len(f) not in (4, 5) or f[1] not in VERDICTS or (len(f) == 5 and f[4] not in ("", "occt")):
            raise SystemExit(f"{ACCOUNT}: a row is not path, verdict, faces, why[, reference]: {line!r}")
        out[f[0]] = (f[1], f[2], f[3], f[4] if len(f) == 5 else "")
    return out


def set_aside(f, o, m, why):
    """The reference of a file whose account sets the measurer aside (its fifth column `occt`):
    OCCT's converged reading, at the bars `own` gives `occt!` — where the evidence shows the
    measurer misreads the file (brepkit's torus grown past its hole: the whole spindle torus,
    whose oriented flux is apple less lemon, 1973.92, OCCT's reading, where the measurer reads the
    apple alone, the case's closed form)."""
    alone = max(ALONE, SLACK * o["tol_edge"] * o["area"] / o["volume"])
    return (o["volume"], o["area"], "occt!", max(alone, AGREE * o["spread_volume"]),
            max(alone, AGREE * o["spread_area"]),
            f"the measurer set aside by the account (it reads {m['volume']!r} {m['area']!r}): {why}"
            if m and "volume" in m else f"the measurer set aside by the account: {why}")


def quad_faces(f, sha):
    """A third reader's area of some faces of the file at `f` (`oracle/<f>.quad.faces.tsv`,
    the measurer's columns: its first line `#`, the DATA's SHA-256; then `entity`, `area`): an
    independent quadrature of each face over the file's own boundary, where the two readers'
    integrations are in question. None where there is none, or it is of other bytes."""
    p = os.path.join(CORPUS, "oracle", f + ".quad.faces.tsv")
    if not os.path.exists(p):
        return None
    lines = open(p).read().splitlines()
    if not lines or lines[0] != f"#\t{sha}":
        return None
    out = {}
    for line in lines[2:]:
        e, a = line.split("\t")[:2]
        out[int(e)] = out.get(int(e), 0.0) + float(a)
    return out


def mass_faces(f, sha):
    p = os.path.join(CORPUS, "oracle", f + ".mass.faces.tsv")
    if not os.path.exists(p):
        return None
    lines = open(p).read().splitlines()
    if not lines or lines[0] != f"#\t{sha}":
        return None
    out = {}
    for line in lines[2:]:
        e, a = line.split("\t")[:2]
        try:
            out[int(e)] = out.get(int(e), 0.0) + float(a)
        except ValueError:
            return None
    return out


def readers(o, m):
    """The independent readings of a file that count: [(name, volume, area)]."""
    out = []
    # the measurer reads a CONVERSION_BASED_UNIT's lengths as millimetres (brep-bench 7d811e9: an
    # inch file's volume comes out in cubic inches), so it counts only for a file in millimetres
    if m and "volume" in m and o["units"].split(",")[0] == "mm":
        out.append(("mass", m["volume"], m["area"]))
    if o["spread_volume"] <= CONVERGED and o["spread_area"] <= CONVERGED:
        out.append(("occt", o["volume"], o["area"]))
    return out


def own(o, m):
    """The reference a file's own solid gives, and its bar (the second bullet above)."""
    alone = max(ALONE, SLACK * o["tol_edge"] * o["area"] / o["volume"])
    if m and "volume" in m and disputed(o, m, alone):
        # the two readers disagree past what the file's slack and OCCT's own spread allow: the
        # measurer is the one set aside (OCCT's area converges on every file where this happens)
        return (o["volume"], o["area"], "occt!", max(alone, AGREE * o["spread_volume"]),
                max(alone, AGREE * o["spread_area"]))
    r = readers(o, m)
    if len(r) == 2:
        dv, da = rel(r[1][1], r[0][1]), rel(r[1][2], r[0][2])
        return r[0][1], r[0][2], "mass+occt", max(FLOOR, AGREE * dv), max(FLOOR, AGREE * da)
    if len(r) == 1:
        return r[0][1], r[0][2], r[0][0], alone, alone
    return (o["volume"], o["area"], "occt?", max(alone, AGREE * o["spread_volume"]),
            max(alone, AGREE * o["spread_area"]))


def disputed(o, m, alone):
    """Whether the bench's measurer and OCCT read the file to different solids: their volumes or
    areas apart by more than the slack and AGREE times OCCT's own spread together."""
    if o["units"].split(",")[0] != "mm":
        return False
    return (rel(m["volume"], o["volume"]) > alone + AGREE * o["spread_volume"]
            or rel(m["area"], o["area"]) > alone + AGREE * o["spread_area"])


def reference(path, o, m, known, cs):
    """(volume, area, from, bar_volume, bar_area, note) for the file at `path`."""
    parts = path.split("/")
    if parts[0] == "variants":
        src = known.get("/".join(parts[2:]))
        if src is None:
            return None
        v, a, frm, bv, ba = src[:5]
        devs = [(rel(x[1], v), rel(x[2], a)) for x in readers(o, m)]
        dv = max((d[0] for d in devs), default=0.0)
        da = max((d[1] for d in devs), default=0.0)
        note = f"source's, readers {dv:.1e} {da:.1e} from it"
        return v, a, "source:" + "/".join(parts[2:]), max(bv, AGREE * dv), max(ba, AGREE * da), note
    if parts[0] == "brep-bench":
        cid = "/".join(parts[2:])[:-len(".step")]
        c = cs.get(cid)
        if c is not None:
            tol = c["tolerance"]
            r = readers(o, m)
            judge = r[0] if r else None
            if judge and rel(judge[1], c["volume"]) <= tol and rel(judge[2], c["area"]) <= tol:
                if c["measured"]:
                    # the case's value is a measure (kernels' values of their own models, or the
                    # measurer's of the reference shape), not a closed form: said as such
                    return (c["volume"], c["area"], "measured", tol, tol,
                            f"the case's solid, by {judge[0]}; the case's "
                            f"{' and '.join(c['measured'])} no closed form but a measure")
                return c["volume"], c["area"], "closed", tol, tol, f"the case's solid, by {judge[0]}"
            v, a, frm, bv, ba = own(o, m)
            if frm == "occt!":
                # a brep-bench file the two read differently: OCCT repairs what the standard
                # reads otherwise (loops against the faces' normals, a seam's loop clockwise:
                # SPEC.md), or the measurer misreads; either reading, or a refusal, is accepted
                return (v, a, "disputed", bv, ba,
                        f"OCCT and the measurer disagree: the measurer reads {m['volume']!r} "
                        f"{m['area']!r}", (m["volume"], m["area"]))
            why = (f"not the case's solid: {judge[0]} reads volume {rel(judge[1], c['volume']):.1e} "
                   f"and area {rel(judge[2], c['area']):.1e} off its closed forms") if judge else \
                "no converged reading to judge it by"
            return v, a, frm, bv, ba, why
    v, a, frm, bv, ba = own(o, m)
    return v, a, frm, bv, ba, ""


def checked(f, sha, o, m, ref):
    """`ref` corrected by a third reader's faces where there is one (`quad_faces`): the measurer's
    area with each face the quadrature measured taken at the quadrature's area, its volume beside
    OCCT's where OCCT's has converged (`mass+quad`). The two readers can share an error the
    quadrature shows (brep-bench's Utah teapot: the measurer, OCCT and the kernels whose mean is
    the case's value all read its knob's four patches low, a pole at each, by 3.7e-4 together)."""
    q = quad_faces(f, sha)
    mf = mass_faces(f, sha)
    if not q or not mf or not (m and "volume" in m) or not all(e in mf for e in q):
        return ref
    area = m["area"] + sum(q[e] - mf[e] for e in q)
    # the volume and the bars are the row's as they were: a quadrature of faces corrects an area,
    # and tightens no bar (CADGenBench 211's `mass` row keeps its 8.5e-4)
    if o["spread_volume"] <= CONVERGED and ref[2] in ("closed", "measured"):
        v, bv = m["volume"], max(FLOOR, AGREE * rel(m["volume"], o["volume"]), ref[3])
    else:
        v, bv = ref[0], ref[3]
    note = (f"the measurer's area, its {len(q)} faces {', '.join(f'#{e}' for e in sorted(q))} at "
            f"the quadrature's ({sum(q[e] - mf[e] for e in q):+.3e} mm²); was {ref[2]} "
            f"{ref[0]!r} {ref[1]!r} ({ref[5]}), {rel(ref[1], area):.1e} off in area")
    return (v, area, "mass+quad", bv, max(ALONE, ref[4]), note) + tuple(ref[6:])


def source_of(path):
    parts = path.split("/")
    if parts[0] == "cadgenbench":
        return "cadgenbench", ""
    if parts[0] == "brep-bench":
        return "brep-bench/" + parts[1], "/".join(parts[2:])[:-len(".step")]
    if parts[0] == "variants":
        s, c = source_of("/".join(parts[2:]))
        return "variants/" + parts[1], c
    if parts[0] == "commercial":
        return "commercial/" + parts[1], parts[-1].rsplit(".", 1)[0]
    return parts[0], ""


def g(x):
    return f"{x:.17g}" if isinstance(x, float) else str(x)


def write(outs):
    cs = cases()
    files = oracle.corpus_files(CORPUS)
    # sources before their variants
    files.sort(key=lambda f: (f.startswith("variants/"), f))
    known, rows, skipped = {}, [], []
    acc = account()
    for f in files:
        try:
            o = json.load(open(os.path.join(CORPUS, "oracle", f + ".json")))
        except (OSError, ValueError):
            skipped.append((f, "not read by the oracle yet"))
            continue
        if "error" in o or o.get("solids", 0) == 0 or not (o.get("volume", 0) > 0):
            row = unread(f, o, cs)
            if isinstance(row, str):
                skipped.append((f, row))
            else:
                rows.append(row)
            continue
        sha = stepscan.data_sha256(open(os.path.join(CORPUS, f), "rb").read())
        if sha != o["data_sha256"]:
            skipped.append((f, "the oracle read other bytes: run oracle.py --corpus"))
            continue
        m = None
        try:
            m = json.load(open(os.path.join(CORPUS, "oracle", f + ".mass.json")))
            if m.get("data_sha256") != sha:
                m = None
        except (OSError, ValueError):
            pass
        ref = reference(f, o, m, known, cs)
        if acc.get(f, ("", "", "", ""))[3] == "occt":
            ref = set_aside(f, o, m, acc[f][2][:160])
        if ref is not None:
            ref = checked(f, sha, o, m, ref)
        if ref is None:
            skipped.append((f, "its source has no row"))
            continue
        if max(ref[3], ref[4]) > CAP:
            # a file OCCT reads only with its tolerances raised past a part in CAP (an edge far
            # off its faces, solids it splits): no reference worth a gate
            skipped.append((f, f"no reference within {CAP:g}: {ref[2]}, bars {ref[3]:.2g} "
                            f"{ref[4]:.2g}; {ref[5]}"))
            continue
        known[f] = ref
        source, case = source_of(f)
        healed = ";".join(f"{k} {v}" for k, v in sorted(o["healed"].items())) or "-"
        row = {"path": f, "bytes": o["bytes"], "data_sha256": sha, "source": source, "case": case,
               "ap": o["ap"], "units": o["units"] or "?", "uncertainty": o["uncertainty"] or "-",
               "solids": o["step_solids"], "step_faces": o["step_faces"],
               "step_edges": o["step_edges"], "step_vertices": o["step_vertices"],
               "occt_faces": o["faces"], "occt_edges": o["edges"], "seams": o["seams"],
               "degenerate": o["degenerate"], "occt_vertices": o["vertices"],
               "voids": o["voids"], "valid": int(o["valid"]), "healed": healed,
               "tol_edge": f"{o['tol_edge']:.3g}", "occt_volume": o["volume"],
               "occt_area": o["area"],
               "occt_spread": f"{o['spread_volume']:.2g},{o['spread_area']:.2g}",
               "mass_volume": m["volume"] if m and "volume" in m else "",
               "mass_area": m["area"] if m and "area" in m else "",
               "mass_error": (m.get("error", "") if m else "not measured")[:120].replace("\t", " "),
               "ref_volume": ref[0], "ref_area": ref[1], "ref_from": ref[2],
               "bar_volume": f"{ref[3]:.2g}", "bar_area": f"{ref[4]:.2g}", "note": ref[5],
               "alt_volume": ref[6][0] if len(ref) > 6 else "",
               "alt_area": ref[6][1] if len(ref) > 6 else ""}
        if o["step_solids"] != o["solids"]:
            row["note"] = (row["note"] + "; " if row["note"] else "") + \
                f"OCCT reads {o['solids']} solids of {o['step_solids']}"
        rows.append(row)
    paths = {r["path"] for r in rows}
    for r in rows:
        v, faces, why, _ = acc.get(r["path"], ("", "", "", ""))
        r.update(verdict=v, unchecked=faces, account=why)
    for p in sorted(set(acc) - paths):
        print(f"account: {p} has no row of the table")
    rows.sort(key=lambda r: r["path"])
    head = (f"# the STEP corpus's reference (scripts/step-corpus/table.py): {len(rows)} files; "
            f"bars relative; FLOOR {FLOOR:g}, ALONE {ALONE:g}, AGREE {AGREE:g}, "
            f"CONVERGED {CONVERGED:g}\n")
    text = head + "\t".join(COLUMNS) + "\n" + "".join(
        "\t".join(g(r[c]) for c in COLUMNS) + "\n" for r in rows)
    for out in outs:
        with open(out, "w") as fh:
            fh.write(text)
    print(f"{len(rows)} rows, to {', '.join(outs)}")
    for f, why in skipped:
        print(f"no row: {f}: {why}")


def unread(f, o, cs):
    """The row of a file OCCT reads to no solid of positive volume (or fails on), from the text and
    the bench's measurer alone, or why there is none: a file that represents no solid (its text
    holds no MANIFOLD_SOLID_BREP or BREP_WITH_VOIDS) is one the reader must refuse (`ref_from`
    `refuse`); one the measurer reads to a solid is held to its reading, uncorroborated (`mass`,
    at `UNCORROBORATED`), or to the closed forms where the bench would pass it."""
    why = o.get("error") or f"OCCT reads {o.get('solids', 0)} solids, volume {o.get('volume')}"
    why = why.strip().splitlines()[-1][-160:] if "\n" in why else why[-160:]
    path = os.path.join(CORPUS, f)
    raw = open(path, "rb").read()
    s = stepscan.scan(path)
    sha = stepscan.data_sha256(raw)
    m = None
    try:
        m = json.load(open(os.path.join(CORPUS, "oracle", f + ".mass.json")))
        if m.get("data_sha256") != sha:
            m = None
    except (OSError, ValueError):
        pass
    source, case = source_of(f)
    row = {c: "" for c in COLUMNS}
    row.update(path=f, bytes=len(raw), data_sha256=sha, source=source, case=case, ap=s["ap"],
               units=s["units"] or "?", uncertainty=s["uncertainty"] or "-",
               solids=len(s["solids"]), step_faces=sum(x["faces"] for x in s["solids"]),
               step_edges=sum(x["edges"] for x in s["solids"]),
               step_vertices=sum(x["vertices"] for x in s["solids"]), healed="-",
               mass_volume=m["volume"] if m and "volume" in m else "",
               mass_area=m["area"] if m and "area" in m else "",
               mass_error=(m.get("error", "") if m else "not measured")[:120].replace("\t", " "))
    if not s["solids"]:
        row.update(ref_volume="", ref_area="", ref_from="refuse", bar_volume="", bar_area="",
                   note="the file represents no solid: " + ", ".join(
                       k for k in s["census"] if "SHELL" in k or "SURFACE_MODEL" in k))
        return row
    if m and s["units"].split(",")[0] != "mm":
        m = None
    if not (m and "volume" in m and m["volume"] > 1e-9 * max(1.0, m["area"]) ** 1.5):
        return f"{why}; and the measurer: {(m or {}).get('error', (m or {}).get('volume'))}"
    c = cs.get(case) if source.startswith("brep-bench") else None
    if c and rel(m["volume"], c["volume"]) <= c["tolerance"] and \
            rel(m["area"], c["area"]) <= c["tolerance"]:
        row.update(ref_volume=c["volume"], ref_area=c["area"],
                   ref_from="measured" if c["measured"] else "closed",
                   bar_volume=f"{c['tolerance']:.2g}", bar_area=f"{c['tolerance']:.2g}",
                   note=f"the case's solid, by mass; OCCT: {why}" + (
                       f"; the case's {' and '.join(c['measured'])} no closed form but a measure"
                       if c["measured"] else ""))
    else:
        row.update(ref_volume=m["volume"], ref_area=m["area"], ref_from="mass",
                   bar_volume=f"{UNCORROBORATED:.2g}", bar_area=f"{UNCORROBORATED:.2g}",
                   note=f"OCCT: {why}")
    return row


def main():
    args = sys.argv[1:]
    if args[:1] == ["measure"]:
        measure(int(args[args.index("--jobs") + 1]) if "--jobs" in args else 2)
        return
    outs = [args[args.index("--out") + 1]] if "--out" in args else \
        [COMMITTED, os.path.join(CORPUS, "reference.tsv")]
    write(outs)


if __name__ == "__main__":
    main()
