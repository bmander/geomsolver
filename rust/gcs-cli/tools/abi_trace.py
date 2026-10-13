#!/usr/bin/env python3
"""Phase 0's tables (docs/rust-kernel-plan.md) from the native kernel's probe logs
(`backend/probe.hpp`): time per stage and entry point, every Boolean's intersecting face pairs by
surface kind, and the exported shapes. Each argument is one export's log:

    SOLVENT_ABI_TRACE=pinion.abi build/solventc rust/examples/spiral_bevel/gears.sv \
        --solid pair.pinion.body --tolerance --kernel occt --step p.step --stl p.stl
    python3 rust/gcs-cli/tools/abi_trace.py pinion.abi

A call is counted in the stage that next completes on its own thread (a worker's, in the next to
complete anywhere); an entry point that calls another (`split_solid`, `split_solid_fuzzy`) is
counted in both, so the native seconds summed over threads may exceed the stage's."""
import sys, collections, math, pathlib

KINDS = ["plane", "cylinder", "cone", "sphere", "torus", "Bezier", "B-spline", "revolution",
         "extrusion", "offset", "other"]

def kind(k): return KINDS[int(k)] if 0 <= int(k) < len(KINDS) else f"kind {k}"

def read(path):
    calls, marks, ffs, booleans, shapes = [], [], [], [], []
    for line in open(path):
        f = line.rstrip("\n").split("\t")
        if f[0] == "call": calls.append((f[1], float(f[2]), float(f[3]), int(f[4]) if len(f) > 4 else -1))
        elif f[0] == "mark": marks.append((f[1], float(f[2]), int(f[3]) if len(f) > 3 else -1))
        elif f[0] == "ff": ffs.append((f[1], f[2], f[3], float(f[4]), float(f[5]), int(f[6]), int(f[7])))
        elif f[0] == "boolean": booleans.append((f[1], int(f[2]), int(f[3])))
        elif f[0] == "shape": shapes.append(f[1:])
    return calls, marks, ffs, booleans, shapes

def stages(calls, marks):
    """Each call in the stage that next completes on its own thread, or, on a thread that marks
    nothing (a worker's), the stage that next completes anywhere; a stage's wall time is from the
    previous mark on its thread."""
    marks = sorted(marks, key=lambda m: m[1])
    table = collections.OrderedDict()
    last = collections.defaultdict(float)
    for name, at, tid in marks:
        table.setdefault(name, {"wall": at, "calls": collections.Counter(), "seconds": collections.Counter()})
    table["after"] = {"wall": 0.0, "calls": collections.Counter(), "seconds": collections.Counter()}
    marking = {tid for _, _, tid in marks}
    for name, start, seconds, tid in calls:
        own = [m for m in marks if m[1] >= start and (m[2] == tid or tid not in marking)]
        stage = own[0][0] if own else "after"
        table[stage]["calls"][name] += 1
        table[stage]["seconds"][name] += seconds
    return table

def main(files):
    for path in files:
        name = pathlib.Path(path).stem
        calls, marks, ffs, booleans, shapes = read(path)
        print(f"\n## {name}\n")
        print("| stage | done at s | native s (summed over threads) | calls | the three heaviest entry points |")
        print("|---|---|---|---|---|")
        for stage, t in stages(calls, marks).items():
            if not t["calls"] and stage == "after": continue
            heavy = ", ".join(f"{n.removeprefix('solvent_cad_')} {t['calls'][n]}× {s:.2f} s"
                              for n, s in t["seconds"].most_common(3))
            print(f"| {stage} | {t['wall']:.2f} | {sum(t['seconds'].values()):.2f} | {sum(t['calls'].values())} | {heavy} |")
        # the Booleans, by what the export calls them
        per = collections.defaultdict(lambda: [0, 0, 0])
        for what, pairs, curves in booleans:
            per[what][0] += 1; per[what][1] += pairs; per[what][2] += curves
        print("\n| Boolean | runs | face pairs tried | intersection curves |")
        print("|---|---|---|---|")
        for what, (runs, pairs, curves) in sorted(per.items()):
            print(f"| {what} | {runs} | {pairs} | {curves} |")
        # every face pair that met, by the kinds of its two surfaces
        met = collections.defaultdict(lambda: {"curves": 0, "length": 0.0, "angle": math.inf,
                                               "under1": 0, "under5": 0, "tangent": 0, "short": math.inf, "whats": set()})
        missed = collections.Counter()
        for what, k1, k2, length, angle, tangent, points in ffs:
            key = tuple(sorted((kind(k1), kind(k2))))
            if length == 0 and math.isnan(angle):
                missed[key] += 1
                continue
            m = met[key]
            m["curves"] += 1; m["length"] += length; m["short"] = min(m["short"], length)
            m["angle"] = min(m["angle"], angle); m["under1"] += angle < 1; m["under5"] += angle < 5
            m["tangent"] += tangent; m["whats"].add(what)
        print("\n| surfaces meeting | curves | total length mm | shortest mm | least angle ° | under 5° | under 1° | tangent | in |")
        print("|---|---|---|---|---|---|---|---|---|")
        for key, m in sorted(met.items(), key=lambda kv: -kv[1]["curves"]):
            print(f"| {' × '.join(key)} | {m['curves']} | {m['length']:.1f} | {m['short']:.4g} | {m['angle']:.2f} | "
                  f"{m['under5']} | {m['under1']} | {m['tangent']} | {'; '.join(sorted(m['whats']))} |")
        if missed:
            print("\nface pairs tried that met in no curve: " + ", ".join(f"{' × '.join(k)} {n}" for k, n in missed.most_common()))
        for s in shapes:
            what, faces, edges, area, edge = s[:5]
            kinds = ", ".join(f"{kind(k.split(':')[0])} {k.split(':')[1]}" for k in s[5:])
            print(f"\n{what}: {faces} faces ({kinds}), {edges} edges; smallest face {float(area):.4g} mm², shortest edge {float(edge):.4g} mm")

main(sys.argv[1:])
