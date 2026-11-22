#!/usr/bin/env python3
"""The corpus's files written again by OCCT under other writers' habits (#993 stage 1), in OCP.

    python3 -I scripts/step-corpus/variants.py [--jobs 2]     # every habit of every sample
    python3 -I scripts/step-corpus/variants.py --list         # the sample, a path a line

Each sample file is read by OCCT, healing off (oracle.py's reading), and written once under each
habit, into `$STEP_CORPUS/variants/<habit>/<the source's path>`:

* `nopcurves`: no curves in the surfaces' parameters (`write.surfacecurve.mode` 0): most
  commercial writers write none, and a reader must place each edge on its faces itself;
* `ap203` and `ap242`: the schema (`write.step.schema` AP203, AP242DIS): other entity sets and
  context structures (AP203's CONFIG_CONTROL_DESIGN, AP242's managed model);
* `inches`: the length unit (`write.step.unit` INCH): every length, the uncertainty included,
  in inches, which the reader must read back to millimetres.

The solid written is the source's, so its measures are the source's: a variant's reference is its
source's (the closed form where the source has one, else OCCT's reading of the source), and the
oracle reads each variant as well, so that table.py can say how far OCCT's own reading of the
variant stands from it (the writer's round trip, which the bar must cover).

The sample: the smallest CADGenBench inputs (`SMALLEST`), and brep-bench cases chosen across the
categories, a case or two each (`bench_sample`), in cad-kernel's file where it wrote one, else
OCCT's. Each source is written in a process of its own, so a file's DATA section (the product's
name counts the session's transfers) does not depend on what was written before it.
"""
import os
import subprocess
import sys
import time

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
CORPUS = os.environ.get("STEP_CORPUS", "/home/user/step-corpus")
HABITS = {"nopcurves": {"write.surfacecurve.mode": 0},
          "ap203": {"write.step.schema": "AP203"},
          "ap242": {"write.step.schema": "AP242DIS"},
          "inches": {"write.step.unit": "INCH"}}
SMALLEST = 8
BENCH_SAMPLE = 30
BENCH_KERNELS = ("cad-kernel", "occt")


def bench_sample():
    """About BENCH_SAMPLE brep-bench files: round-robin over the categories in name order, each
    category's cases in name order, taking each case's file from the first of BENCH_KERNELS that
    wrote it (so the sample covers every category before it takes a second case of any)."""
    root = os.path.join(CORPUS, "brep-bench")
    by = {}
    for k in BENCH_KERNELS:
        kr = os.path.join(root, k)
        for d, _, fs in os.walk(kr):
            for f in fs:
                if f.endswith(".step"):
                    cid = os.path.relpath(os.path.join(d, f), kr)[:-5]
                    by.setdefault(cid.split("/")[0], {}).setdefault(
                        cid, os.path.relpath(os.path.join(d, f), CORPUS))
    cats = sorted(by)
    out, i = [], 0
    while len(out) < BENCH_SAMPLE and any(len(by[c]) > i for c in cats):
        for c in cats:
            ids = sorted(by[c])
            # spread the picks through the category rather than its first names
            if i < len(ids) and len(out) < BENCH_SAMPLE:
                out.append(by[c][ids[(i * 7) % len(ids)]])
        i += 1
    return sorted(set(out))


def sample():
    cgb = os.path.join(CORPUS, "cadgenbench")
    smallest = sorted((os.path.getsize(os.path.join(cgb, d, "input.step")), d)
                      for d in os.listdir(cgb)
                      if os.path.exists(os.path.join(cgb, d, "input.step")))[:SMALLEST]
    return sorted(f"cadgenbench/{d}/input.step" for _, d in smallest) + bench_sample()


def write_all(rel):
    """`rel` read (healing off) and written under each habit."""
    sys.path.insert(0, HERE)
    import oracle
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.Interface import Interface_Static
    from OCP.STEPControl import STEPControl_AsIs, STEPControl_Writer
    shape = oracle.read(os.path.join(CORPUS, rel), healing=False)
    defaults = {k: Interface_Static.CVal_s(k) if isinstance(v, str) else Interface_Static.IVal_s(k)
                for h in HABITS.values() for k, v in h.items()}
    for habit, params in HABITS.items():
        for k, v in defaults.items():
            (Interface_Static.SetCVal_s if isinstance(v, str) else Interface_Static.SetIVal_s)(k, v)
        for k, v in params.items():
            (Interface_Static.SetCVal_s if isinstance(v, str) else Interface_Static.SetIVal_s)(k, v)
        w = STEPControl_Writer()
        if w.Transfer(shape, STEPControl_AsIs) != IFSelect_RetDone:
            print(f"{habit}\t{rel}\tnot transferred", file=sys.stderr)
            continue
        dest = os.path.join(CORPUS, "variants", habit, rel)
        os.makedirs(os.path.dirname(dest), exist_ok=True)
        if w.Write(dest + ".part") != IFSelect_RetDone:
            print(f"{habit}\t{rel}\tnot written", file=sys.stderr)
            continue
        os.replace(dest + ".part", dest)
        print(f"{habit}\t{rel}\t{os.path.getsize(dest)} bytes", file=sys.stderr)


def main():
    args = sys.argv[1:]
    if args[:1] == ["--one"]:
        write_all(args[1])
        return
    if args[:1] == ["--list"]:
        print("\n".join(sample()))
        return
    jobs = int(args[args.index("--jobs") + 1]) if "--jobs" in args else 2
    todo = [r for r in sample() if not all(os.path.exists(os.path.join(CORPUS, "variants", h, r))
                                           for h in HABITS)]
    print(f"{len(todo)} sources to write, {jobs} at once", flush=True)
    running = []
    for rel in todo:
        while len(running) >= jobs:
            time.sleep(0.1)
            running = [p for p in running if p.poll() is None]
        running.append(subprocess.Popen([sys.executable, "-I", os.path.abspath(__file__), "--one",
                                         rel], stdout=subprocess.DEVNULL))
    for p in running:
        p.wait()


if __name__ == "__main__":
    main()
