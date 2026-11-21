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
the reference: `ref_volume`, `ref_area`, `ref_from`, `bar_volume`, `bar_area` (relative), `note`.

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
  `occt?`). The slack is calibrated on the CADGenBench files both read: their readings differ by
  up to 0.04 of tolerance × area / volume (cadgenbench/242: 2.1e-4 in volume, at tolerances of
  0.02 mm), the gaps a file's uncertainty leaves between an edge's curve and its faces read two
  ways; `SLACK` leaves a margin of two and a half over that.
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
MASS = os.path.join(CORPUS, "build", "brep-bench", "harness", "mass", "target", "release", "mass")
CONVERGED = 1e-6
AGREE = 3.0
FLOOR = 1e-7
ALONE = 1e-6
SLACK = 0.1
EXACT = 1e-6

COLUMNS = ("path bytes data_sha256 source case ap units uncertainty solids step_faces step_edges "
           "step_vertices occt_faces occt_edges seams degenerate occt_vertices voids valid healed "
           "tol_edge occt_volume occt_area occt_spread mass_volume mass_area mass_error ref_volume "
           "ref_area ref_from bar_volume bar_area note").split()


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
        out[f[0]] = {"volume": float(f[1]), "area": float(f[2]), "tolerance": float(f[3])}
    return out


def readers(o, m):
    """The independent readings of a file that count: [(name, volume, area)]."""
    out = []
    if m and "volume" in m:
        out.append(("mass", m["volume"], m["area"]))
    if o["spread_volume"] <= CONVERGED and o["spread_area"] <= CONVERGED:
        out.append(("occt", o["volume"], o["area"]))
    return out


def own(o, m):
    """The reference a file's own solid gives, and its bar (the second bullet above)."""
    r = readers(o, m)
    if len(r) == 2:
        dv, da = rel(r[1][1], r[0][1]), rel(r[1][2], r[0][2])
        return r[0][1], r[0][2], "mass+occt", max(FLOOR, AGREE * dv), max(FLOOR, AGREE * da)
    alone = max(ALONE, SLACK * o["tol_edge"] * o["area"] / o["volume"])
    if len(r) == 1:
        return r[0][1], r[0][2], r[0][0], alone, alone
    return (o["volume"], o["area"], "occt?", max(alone, AGREE * o["spread_volume"]),
            max(alone, AGREE * o["spread_area"]))


def reference(path, o, m, known, cs):
    """(volume, area, from, bar_volume, bar_area, note) for the file at `path`."""
    parts = path.split("/")
    if parts[0] == "variants":
        src = known.get("/".join(parts[2:]))
        if src is None:
            return None
        v, a, frm, bv, ba, _ = src
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
                return c["volume"], c["area"], "closed", tol, tol, f"the case's solid, by {judge[0]}"
            v, a, frm, bv, ba = own(o, m)
            why = (f"not the case's solid: {judge[0]} reads volume {rel(judge[1], c['volume']):.1e} "
                   f"and area {rel(judge[2], c['area']):.1e} off its closed forms") if judge else \
                "no converged reading to judge it by"
            return v, a, frm, bv, ba, why
    v, a, frm, bv, ba = own(o, m)
    return v, a, frm, bv, ba, ""


def source_of(path):
    parts = path.split("/")
    if parts[0] == "cadgenbench":
        return "cadgenbench", ""
    if parts[0] == "brep-bench":
        return "brep-bench/" + parts[1], "/".join(parts[2:])[:-len(".step")]
    if parts[0] == "variants":
        s, c = source_of("/".join(parts[2:]))
        return "variants/" + parts[1], c
    return parts[0], ""


def g(x):
    return f"{x:.17g}" if isinstance(x, float) else str(x)


def write(outs):
    cs = cases()
    files = oracle.corpus_files(CORPUS)
    # sources before their variants
    files.sort(key=lambda f: (f.startswith("variants/"), f))
    known, rows, skipped = {}, [], []
    for f in files:
        try:
            o = json.load(open(os.path.join(CORPUS, "oracle", f + ".json")))
        except (OSError, ValueError):
            skipped.append((f, "not read by the oracle yet"))
            continue
        if "error" in o or o.get("solids", 0) == 0 or not (o.get("volume", 0) > 0):
            skipped.append((f, o.get("error") or f"OCCT reads {o.get('solids', 0)} solids, "
                            f"volume {o.get('volume')}"))
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
        if ref is None:
            skipped.append((f, "its source has no row"))
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
               "bar_volume": f"{ref[3]:.2g}", "bar_area": f"{ref[4]:.2g}", "note": ref[5]}
        if o["step_solids"] != o["solids"]:
            row["note"] = (row["note"] + "; " if row["note"] else "") + \
                f"OCCT reads {o['solids']} solids of {o['step_solids']}"
        rows.append(row)
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
