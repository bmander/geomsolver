#!/usr/bin/env python3
"""The files commercial CAD systems wrote, by writer: who wrote each (`writer`), and what the
kernel's reader does with them, tallied by writer and cause (#993 stage 1). Standard library only.

    python3 -I scripts/step-corpus/commercial.py                  # each file's writer
    python3 -I scripts/step-corpus/commercial.py SURVEY.txt       # and the survey's outcomes by it

SURVEY.txt is `step_survey`'s output over `$STEP_CORPUS/commercial/` (a line a file: path, `read`
or `refused`, the refusal's cause and witness).

NIST removed the originating system from the headers of its MBE PMI files ("Information that
shows which CAD software generated the files has been removed", its README), so a file's writer is
inferred from what its DATA section shows, each habit a writer's own and seen in no other of the
set; `writer` names the habit beside the system it is taken for, and a file whose header names its
system (the D2MI files) is taken at its word:

* `Axis2P3D`, `PartBody` names and ` ;` before each statement's end: CATIA V5;
* faces named `<n>|<hash>` and a `point coincidence tolerance`, beside `CATIA Geometric Set`
  shape aspects: a CATIA model through an AP242 writer that names faces by a persistent id;
* the shape representation named `<part>-None` (an NX reference set): NX;
* reals written `3.5E1`, `0.E0` (an exponent on every real, a zero one too): Creo (PTC's style);
* faces named `no <n>` with a `Confusion accuracy` uncertainty: the AP242 writer of one of NX or
  Creo, which one not settled by its text (`no-N writer`);
* a `SRR` shape representation relationship and `DISTANCE_ACCURACY_VALUE` in capitals: one
  writer (`SRR writer`), SolidWorks or Inventor by the appearance names it writes;
* reals with three-digit exponents (`1.0E-006`) and a `MILLIMETRE` conversion unit: a Windows
  writer (`E-006 writer`), Inventor by elimination among the STC models' four systems;
* none of these: `writer U` — faces unnamed, reals as `0.`, long entities wrapped; where the STC
  models' four systems are CATIA (stc_07, stc_08), Inventor (stc_09) and the no-N writer (stc_06),
  writer U (stc_10) is the fourth, Creo or NX as the no-N writer is the other.
"""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.dont_write_bytecode = True
sys.path.insert(0, HERE)
import stepscan  # noqa: E402

CORPUS = os.environ.get("STEP_CORPUS", "/home/user/step-corpus")


def writer(path):
    raw = open(path, "rb").read().decode("latin-1")
    _, system = stepscan.header(raw[:20000])
    if system.strip():
        return system.strip()
    # each habit counted where the solid's own entities show it, so that a name a PMI entity
    # carries over from another system does not decide
    def many(pattern):
        return len(re.findall(pattern, raw)) >= 20
    if many(r"ADVANCED_FACE\('\d+\|\d+'"):
        return "CATIA via a persistent-id AP242 writer"
    if many(r"'Axis2P3D"):
        return "CATIA V5 (Axis2P3D)"
    if many(r"ADVANCED_FACE\('no \d+'"):
        return "no-N writer (NX or Creo)"
    if re.search(r"-None'", raw):
        return "NX (reference set -None)"
    if "'SRR'" in raw:
        return "SRR writer (SolidWorks or Inventor)"
    if re.search(r"\d\.\d+E-00\d\)", raw):
        return "E-006 writer (Inventor)"
    if many(r"[(,]-?\d\.E0[,)]"):
        return "Creo (E-format reals)"
    return "writer U (Creo or NX)"


def files():
    out = []
    for d, _, fs in os.walk(os.path.join(CORPUS, "commercial")):
        out += [os.path.join(d, f) for f in fs if f.lower().endswith((".stp", ".step"))]
    return sorted(out)


def cause(line):
    """The class of a survey line's outcome: `read`, or the refusal's message with its numbers,
    entities and places taken out."""
    f = line.rstrip("\n").split("\t")
    if f[1] != "refused":
        return f[1]
    msg = f[3].split(": ", 1)[-1]
    msg = re.sub(r"#\d+", "#", msg)
    msg = re.sub(r"\[[^\]]*\]", "[…]", msg)
    msg = re.sub(r"-?\d+(\.\d+)?(e-?\d+)?", "N", msg)
    return f"{f[2]}: {msg[:90]}"


def main():
    who = {p: writer(p) for p in files()}
    if len(sys.argv) < 2:
        for p, w in who.items():
            print(f"{os.path.relpath(p, CORPUS)}\t{w}")
        return
    tally = {}
    for line in open(sys.argv[1]):
        path = line.split("\t")[0]
        if path not in who:
            continue
        tally.setdefault(who[path], {}).setdefault(cause(line), []).append(os.path.basename(path))
    for w, cs in sorted(tally.items()):
        print(f"{w}: {sum(len(v) for v in cs.values())} files")
        for c, fs in sorted(cs.items(), key=lambda x: -len(x[1])):
            print(f"  {len(fs):>3}  {c}\n       {' '.join(sorted(fs))}")


if __name__ == "__main__":
    main()
