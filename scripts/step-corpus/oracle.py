#!/usr/bin/env python3
"""What Open CASCADE reads of a STEP file: the oracle the kernel's STEP reader is held to (#993
stage 1), in tests only. Needs the OCP bindings (`pip install cadquery-ocp`).

    python3 -I scripts/step-corpus/oracle.py FILE.step...          # a JSON line a file
    python3 -I scripts/step-corpus/oracle.py --faces FILE.step     # and its faces, a TSV row each
    python3 -I scripts/step-corpus/oracle.py --corpus [--jobs 2]   # every file of $STEP_CORPUS
                                                                   # not yet read, into oracle/

For each file it gives:

* the text's own account (`stepscan.py`): schema, writer, units, stated uncertainty, and per
  solid the topological entities under it (`step_faces`, `step_edges`, `step_vertices`: each
  ADVANCED_FACE, EDGE_CURVE and VERTEX_POINT once, as the kernel's `Given.faces`, `edges` and
  `vertices` name them);
* what OCCT reads, **with its shape healing off**: the file's translation alone, with no
  `ShapeProcess` sequence after it (`read.step.sequence` names none, so `XSAlgo_ShapeProcessor`
  runs no operation; the translation itself still sets a vertex's tolerance to cover its edges'
  ends, orders a wire's edges and computes missing pcurves, which no switch turns off):
  solids, shells, faces, edges and vertices, each counted once (`TopExp::MapShapes`, which takes a
  shared sub-shape once whatever its orientation); among the edges, `seams` (closed on a face they
  bound: one edge, used twice by that face) and `degenerate` (a pole's, which OCCT lays where the
  file writes none); `valid` (`BRepCheck_Analyzer`); the largest edge and vertex tolerance;
  `tol_length`, the sum over edges of length times tolerance (mm², the area the edges' slack can
  sweep); the bounding box (exact geometry, no triangles, no tolerance added); and the volume,
  area and centre of mass;
* `volume`/`area` at each integration precision: OCCT's fixed Gauss rule (`gauss`, what a call
  with no tolerance gives) and its adaptive rule at relative tolerances 1e-6, 1e-8 and 1e-10;
  the value reported is the finest's, `spread` the largest relative difference of the adaptive
  ones from it, `gauss_err` the fixed rule's;
* the same file read **with healing on** (OCCT's default, `FromSTEP`), and `healed`, what healing
  changed: counts, validity, tolerances, measures (empty where nothing did), and its edges'
  `tol_length` (healing measures an edge's tolerance where it fixes its parameters);
* per face (`--faces`): its index in OCCT's order, the ADVANCED_FACE it was read from (OCCT keeps
  the shells' order of faces; the surface kinds of the two lists are compared, and `faces_mapped`
  says whether they all agree), its surface kind, orientation and area.
"""
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.dont_write_bytecode = True
sys.path.insert(0, HERE)
import stepscan  # noqa: E402

EPS = (1e-6, 1e-8, 1e-10)
NO_HEALING = "NoHealing"  # a sequence the STEP resource does not name: no operation runs
KINDS = {"GeomAbs_Plane": "plane", "GeomAbs_Cylinder": "cylinder", "GeomAbs_Cone": "cone",
         "GeomAbs_Sphere": "sphere", "GeomAbs_Torus": "torus",
         "GeomAbs_BezierSurface": "bspline", "GeomAbs_BSplineSurface": "bspline",
         "GeomAbs_SurfaceOfRevolution": "revolution", "GeomAbs_SurfaceOfExtrusion": "extrusion",
         "GeomAbs_OffsetSurface": "offset", "GeomAbs_OtherSurface": "other"}


def corpus():
    return os.environ.get("STEP_CORPUS", "/home/user/step-corpus")


def read(path, healing):
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.Interface import Interface_Static
    from OCP.STEPControl import STEPControl_Reader
    Interface_Static.SetCVal_s("read.step.sequence", "FromSTEP" if healing else NO_HEALING)
    r = STEPControl_Reader()
    if r.ReadFile(path) != IFSelect_RetDone:
        raise RuntimeError("OCCT cannot parse the file")
    r.TransferRoots()
    shape = r.OneShape()
    if shape.IsNull():
        raise RuntimeError("OCCT reads no shape")
    return shape


def kind_of(face):
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    return KINDS.get(BRepAdaptor_Surface(face).GetType().name, "other")


def measure(shape, adaptive=True):
    """Volume, area and centre of mass at each precision."""
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    out = {}
    g = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape, g)
    a = GProp_GProps()
    BRepGProp.SurfaceProperties_s(shape, a)
    out["gauss"] = (g.Mass(), a.Mass())
    centre = None
    for eps in (EPS if adaptive else EPS[-1:]):
        g = GProp_GProps()
        BRepGProp.VolumeProperties_s(shape, g, eps)
        a = GProp_GProps()
        BRepGProp.SurfaceProperties_s(shape, a, eps)
        out[eps] = (g.Mass(), a.Mass())
        c = g.CentreOfMass()
        centre = (c.X(), c.Y(), c.Z())
    return out, centre


def rel(a, b):
    return abs(a - b) / abs(b) if b else abs(a - b)


def topology(shape, faces=False):
    from OCP.BRep import BRep_Tool
    from OCP.BRepAdaptor import BRepAdaptor_Curve
    from OCP.BRepBndLib import BRepBndLib
    from OCP.BRepCheck import BRepCheck_Analyzer
    from OCP.Bnd import Bnd_Box
    from OCP.GCPnts import GCPnts_AbscissaPoint
    from OCP.TopAbs import (TopAbs_EDGE, TopAbs_FACE, TopAbs_REVERSED, TopAbs_SHELL,
                            TopAbs_SOLID, TopAbs_VERTEX)
    from OCP.TopExp import TopExp, TopExp_Explorer
    from OCP.TopTools import TopTools_IndexedMapOfShape
    from OCP.TopoDS import TopoDS

    def mapped(kind):
        m = TopTools_IndexedMapOfShape()
        TopExp.MapShapes_s(shape, kind, m)
        return m
    sol, sh, fa, ed, ve = (mapped(k) for k in (TopAbs_SOLID, TopAbs_SHELL, TopAbs_FACE,
                                              TopAbs_EDGE, TopAbs_VERTEX))
    t = {"solids": sol.Extent(), "shells": sh.Extent(), "faces": fa.Extent(),
         "edges": ed.Extent(), "vertices": ve.Extent()}
    # shells per solid: a void is a solid's second shell
    t["voids"] = sum(max(0, mapped_in(sol.FindKey(i), TopAbs_SHELL) - 1)
                     for i in range(1, sol.Extent() + 1))
    seams, degenerate, tol_e, tol_len = set(), 0, 0.0, 0.0
    for i in range(1, fa.Extent() + 1):
        f = TopoDS.Face_s(fa.FindKey(i))
        x = TopExp_Explorer(f, TopAbs_EDGE)
        while x.More():
            e = TopoDS.Edge_s(x.Current())
            if BRep_Tool.IsClosed_s(e, f):
                seams.add(ed.FindIndex(e))
            x.Next()
    for i in range(1, ed.Extent() + 1):
        e = TopoDS.Edge_s(ed.FindKey(i))
        tol = BRep_Tool.Tolerance_s(e)
        tol_e = max(tol_e, tol)
        if BRep_Tool.Degenerated_s(e):
            degenerate += 1
            continue
        try:
            tol_len += GCPnts_AbscissaPoint.Length_s(BRepAdaptor_Curve(e)) * tol
        except Exception:
            pass
    tol_v = max((BRep_Tool.Tolerance_s(TopoDS.Vertex_s(ve.FindKey(i)))
                 for i in range(1, ve.Extent() + 1)), default=0.0)
    t.update(seams=len(seams), degenerate=degenerate, tol_edge=tol_e, tol_vertex=tol_v,
             tol_length=tol_len)
    t["valid"] = bool(BRepCheck_Analyzer(shape).IsValid())
    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, box, False, False)
    t["bbox"] = list(box.Get()) if not box.IsVoid() else []
    kinds = {}
    rows = []
    for i in range(1, fa.Extent() + 1):
        f = TopoDS.Face_s(fa.FindKey(i))
        k = kind_of(f)
        kinds[k] = kinds.get(k, 0) + 1
        if faces:
            from OCP.BRepGProp import BRepGProp
            from OCP.GProp import GProp_GProps
            a = GProp_GProps()
            BRepGProp.SurfaceProperties_s(f, a, 1e-10)
            c = a.CentreOfMass()
            rows.append([i, k, "reversed" if f.Orientation() == TopAbs_REVERSED else "forward",
                         a.Mass(), c.X(), c.Y(), c.Z()])
    t["surfaces"] = dict(sorted(kinds.items()))
    return t, rows


def mapped_in(shape, kind):
    from OCP.TopExp import TopExp
    from OCP.TopTools import TopTools_IndexedMapOfShape
    m = TopTools_IndexedMapOfShape()
    TopExp.MapShapes_s(shape, kind, m)
    return m.Extent()


def oracle(path, faces=False):
    clock = time.time()
    scan = stepscan.scan(path)
    out = {k: scan[k] for k in ("bytes", "sha256", "data_sha256", "schema", "ap", "writer",
                                "units", "uncertainty")}
    out["step_solids"] = len(scan["solids"])
    for k in ("shells", "faces", "edges", "vertices"):
        out["step_" + k] = sum(s[k] for s in scan["solids"])
    out["solid_entities"] = [s["entity"] for s in scan["solids"]]
    shape = read(path, healing=False)
    t, rows = topology(shape, faces)
    out.update(t)
    m, centre = measure(shape)
    v, a = m[EPS[-1]]
    out["volume"], out["area"], out["centre"] = v, a, list(centre)
    out["spread_volume"] = max(rel(m[e][0], v) for e in EPS)
    out["spread_area"] = max(rel(m[e][1], a) for e in EPS)
    out["gauss_err_volume"] = rel(m["gauss"][0], v)
    out["gauss_err_area"] = rel(m["gauss"][1], a)
    out["precisions"] = {str(k): list(x) for k, x in m.items()}
    out["read_seconds"] = round(time.time() - clock, 2)
    # healing on: what it changes
    healed = read(path, healing=True)
    h, _ = topology(healed)
    hm, _ = measure(healed, adaptive=False)
    hv, ha = hm[EPS[-1]]
    changed = {}
    for k in ("solids", "shells", "faces", "edges", "vertices", "seams", "degenerate", "valid"):
        if h[k] != t[k]:
            changed[k] = [t[k], h[k]]
    for k in ("tol_edge", "tol_vertex"):
        if rel(h[k], t[k]) > 1e-9:
            changed[k] = [t[k], h[k]]
    if rel(hv, v) > 1e-12:
        changed["volume"] = rel(hv, v)
    if rel(ha, a) > 1e-12:
        changed["area"] = rel(ha, a)
    out["healed"] = changed
    out["healed_valid"] = h["valid"]
    out["healed_tol_length"] = h["tol_length"]
    if faces:
        out["faces_mapped"] = name_faces(rows, scan)
        out["face_rows"] = rows
    out["seconds"] = round(time.time() - clock, 2)
    return out


def name_faces(rows, scan):
    """Each of OCCT's face rows `[index, kind, …]` given, after its index, the ADVANCED_FACE it
    was read from and that entity's surface kind: OCCT makes a solid's faces in the order its
    shells list them, and whether the two lists' kinds agree face by face is the check (0 for the
    entity where they do not). Whether they all agreed."""
    listed = [f for s in scan["solids"] for f in s["face_list"]]
    mapped = len(listed) == len(rows) and all(
        r[1] == f[1] or f[1] in ("revolution", "extrusion", "offset") for r, f in zip(rows, listed))
    for k, r in enumerate(rows):
        f = listed[k] if k < len(listed) else (0, "?")
        r.insert(1, f[0] if mapped else 0)
        r.insert(3, f[1])
    return mapped


def remap(root):
    """The faces of every file already read named again (`name_faces`), and its units read again,
    from its face table and the text: what a change to the text's scan needs, without OCCT."""
    for f in corpus_files(root):
        js, tsv = (os.path.join(root, "oracle", f + x) for x in (".json", ".faces.tsv"))
        if not (os.path.exists(js) and os.path.exists(tsv)):
            continue
        got = json.load(open(js))
        if "error" in got:
            continue
        rows = [line.split("\t") for line in open(tsv).read().splitlines()[1:]]
        rows = [[int(r[0]), r[2]] + r[4:] for r in rows]
        scan = stepscan.scan(os.path.join(root, f))
        got["faces_mapped"] = name_faces(rows, scan)
        got["units"], got["uncertainty"] = scan["units"], scan["uncertainty"]
        with open(tsv, "w") as fh:
            fh.write(FACE_HEADER + "\n" + "".join("\t".join(str(x) for x in r) + "\n" for r in rows))
        with open(js, "w") as fh:
            json.dump(got, fh, indent=1, sort_keys=True)


FACE_HEADER = "index\tentity\tkind\tstep_kind\torientation\tarea\tcx\tcy\tcz"


def faces_tsv(rows):
    return FACE_HEADER + "\n" + "".join(
        "\t".join(f"{x:.12g}" if isinstance(x, float) else str(x) for x in r) + "\n"
        for r in rows)


def corpus_files(root):
    """Every STEP file of the corpus, by its path under `root`, in order."""
    out = []
    for sub in ("cadgenbench", "brep-bench", "variants"):
        for d, _, fs in os.walk(os.path.join(root, sub)):
            for f in fs:
                if f.endswith(".step"):
                    out.append(os.path.relpath(os.path.join(d, f), root))
    return sorted(out)


def run_corpus(root, jobs, timeout, only):
    """Each file of the corpus not yet read (or changed since), read in a process of its own (a
    file OCCT hangs on is given `timeout` seconds), into `oracle/<path>.json` and its faces into
    `oracle/<path>.faces.tsv`."""
    files = [f for f in corpus_files(root) if not only or any(f.startswith(o) for o in only)]
    todo = []
    for f in files:
        js = os.path.join(root, "oracle", f + ".json")
        if os.path.exists(js):
            try:
                with open(js) as fh:
                    old = json.load(fh)
                if old.get("data_sha256") == stepscan.data_sha256(
                        open(os.path.join(root, f), "rb").read()):
                    continue
            except (ValueError, OSError):
                pass
        todo.append(f)
    print(f"{len(files)} files, {len(todo)} to read, {jobs} at once", flush=True)
    running = []

    def finish(item, killed):
        p, f, start, log = item
        log.close()
        if p.returncode != 0:
            err = open(log.name).read()[-400:].strip().replace("\n", " ")
            why = f"over {timeout:g} s" if killed else f"exit {p.returncode}: {err}"
            with open(os.path.join(root, "oracle", f + ".json"), "w") as fh:
                json.dump({"error": why, "data_sha256": stepscan.data_sha256(
                    open(os.path.join(root, f), "rb").read())}, fh)
        print(f"{f}\t{'ok' if p.returncode == 0 else 'error'}\t{time.time() - start:.1f}s",
              flush=True)

    def reap():
        for item in list(running):
            p, _, start, _ = item
            killed = p.poll() is None and time.time() - start > timeout
            if killed:
                p.kill()
                p.wait()
            if p.poll() is not None:
                running.remove(item)
                finish(item, killed)
    for f in todo:
        while len(running) >= jobs:
            reap()
            time.sleep(0.1)
        os.makedirs(os.path.dirname(os.path.join(root, "oracle", f)), exist_ok=True)
        log = open(os.path.join(root, "oracle", f + ".log"), "w")
        p = subprocess.Popen([sys.executable, "-I", os.path.abspath(__file__), "--into",
                              os.path.join(root, "oracle", f), os.path.join(root, f)],
                             stdout=log, stderr=subprocess.STDOUT)
        running.append((p, f, time.time(), log))
    while running:
        reap()
        time.sleep(0.1)


def main():
    args = sys.argv[1:]
    if args[:1] == ["--into"]:
        into, path = args[1], args[2]
        got = oracle(path, faces=True)
        rows = got.pop("face_rows")
        with open(into + ".faces.tsv", "w") as fh:
            fh.write(faces_tsv(rows))
        with open(into + ".json", "w") as fh:
            json.dump(got, fh, indent=1, sort_keys=True)
        return
    if args[:1] == ["--remap"]:
        remap(corpus())
        return
    if args[:1] == ["--corpus"]:
        jobs, timeout, only = 2, 1800.0, []
        rest = args[1:]
        while rest:
            a = rest.pop(0)
            if a == "--jobs":
                jobs = int(rest.pop(0))
            elif a == "--timeout":
                timeout = float(rest.pop(0))
            else:
                only.append(a)
        run_corpus(corpus(), jobs, timeout, only)
        return
    faces = args[:1] == ["--faces"]
    for path in args[1:] if faces else args:
        got = oracle(path, faces)
        rows = got.pop("face_rows", None)
        print(json.dumps({"file": path, **got}, sort_keys=True), flush=True)
        if rows is not None:
            sys.stdout.write(faces_tsv(rows))


if __name__ == "__main__":
    main()
