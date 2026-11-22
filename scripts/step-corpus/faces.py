#!/usr/bin/env python3
"""The corpus's references face by face: what `step_corpus::nothing_is_read_wrong` holds each face
the kernel reads to, beside the totals of `reference.tsv` (#993 stage 1).

    python3 -I scripts/step-corpus/faces.py build                    # the measurer, per face
    python3 -I scripts/step-corpus/faces.py measure [--jobs 2] [PREFIX...]
    python3 -I scripts/step-corpus/oracle.py --name PATH...          # OCCT's faces named
    python3 -I scripts/step-corpus/faces.py census                   # which files join, and why not

Two readers give a face's area. **OCCT's** is `oracle/<path>.faces.tsv` (`oracle.py`: each of
its faces, healing off, measured by `BRepGProp` at the adaptive rule's 1e-10, with its centroid),
its `entity` column the ADVANCED_FACE it was translated from. That column is filled two ways:
by the order of the shells' faces, where OCCT keeps it and the surface kinds of the two lists
agree face by face (`faces_mapped` in its JSON); and, where they do not, by OCCT's own record of
the translation (`oracle.py --name`, `faces_named` in the JSON), which also names each part of a
face OCCT split. **brep-bench's measurer** (`harness/mass`, patched by `mass-faces.patch` to print
what it already sums: each face's area as ISO 10303-42 reads its loops, its loops put on the face
by inverting its edges' curves in space, never by the file's pcurves) is
`oracle/<path>.mass.faces.tsv`: `entity`, `area` (or the measurer's reason it cannot measure the
face, where it gives one).

`build` copies the bench's measurer from `$STEP_CORPUS/build/brep-bench/harness/mass` (made by
`brep_bench.py prepare`) to `$STEP_CORPUS/build/mass-faces`, applies the patch and builds it;
`measure` runs it over every corpus file and mutant whose table is missing or older than its file's
DATA section (the hash is the table's first line, `#` and the SHA-256). `census` says, for the
files of `reference.tsv`, how many have a face table each reader's faces join onto the file's
ADVANCED_FACEs one to one, and why the others do not.
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.dont_write_bytecode = True
sys.path.insert(0, HERE)
import oracle  # noqa: E402
import stepscan  # noqa: E402

CORPUS = os.environ.get("STEP_CORPUS", "/home/user/step-corpus")
BENCH = os.path.join(CORPUS, "build", "brep-bench", "harness", "mass")
BUILD = os.path.join(CORPUS, "build", "mass-faces")
MASS = os.path.join(BUILD, "target", "release", "mass")
REPO = os.path.dirname(os.path.dirname(HERE))
TABLE = os.path.join(REPO, "rust", "gcs-core", "tests", "step_corpus", "reference.tsv")


def build():
    shutil.rmtree(BUILD, ignore_errors=True)
    os.makedirs(BUILD)
    for x in ("Cargo.toml", "src"):
        src = os.path.join(BENCH, x)
        (shutil.copytree if os.path.isdir(src) else shutil.copy)(src, os.path.join(BUILD, x))
    subprocess.run(["patch", "-p1", "-i", os.path.join(HERE, "mass-faces.patch")], cwd=BUILD,
                   check=True)
    subprocess.run(["cargo", "build", "--release", "-q", "-j",
                    os.environ.get("CARGO_JOBS", "2")], cwd=BUILD, check=True)


def table_of(got, sha):
    head = f"#\t{sha}\nentity\tarea\n"
    if "face_areas" not in got:
        why = got.get("error") or got.get("invalid") or "no faces measured"
        return head + f"0\t{str(why)[:200]}\n"
    return head + "".join(f"{e}\t{a!r}\n" if isinstance(a, float) else f"{e}\t{a}\n"
                          for e, a in got["face_areas"])


def measure(jobs, only, timeout=900.0):
    files = [f for f in oracle.corpus_files(CORPUS) + oracle.mutant_files(CORPUS)
             if not only or any(f.startswith(o) for o in only)]
    todo = []
    for f in files:
        out = os.path.join(CORPUS, "oracle", f + ".mass.faces.tsv")
        sha = stepscan.data_sha256(open(os.path.join(CORPUS, f), "rb").read())
        if os.path.exists(out) and open(out).readline().strip() == f"#\t{sha}":
            continue
        todo.append((f, sha))
    print(f"{len(files)} files, {len(todo)} to measure, {jobs} at once", flush=True)
    running = []

    def reap():
        for item in list(running):
            p, f, sha, start, sink = item
            killed = p.poll() is None and time.time() - start > timeout
            if killed:
                p.kill()
            if p.poll() is None:
                continue
            running.remove(item)
            # its answer through a file: a pipe fills at 64 kB and the measurer waits on it
            _, err = p.communicate()
            sink.seek(0)
            out = sink.read().decode("utf-8", "replace")
            sink.close()
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
                got = {"error": (err.strip() or f"exit {p.returncode}")[-200:].replace("\n", " ")}
            path = os.path.join(CORPUS, "oracle", f + ".mass.faces.tsv")
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "w") as fh:
                fh.write(table_of(got, sha))
    for f, sha in todo:
        while len(running) >= jobs:
            reap()
            time.sleep(0.02)
        sink = tempfile.TemporaryFile()
        p = subprocess.Popen([MASS, os.path.join(CORPUS, f)], stdout=sink,
                             stderr=subprocess.PIPE, text=True,
                             env={**os.environ, "MASS_THREADS": "1", "MASS_FACES": "1"})
        running.append((p, f, sha, time.time(), sink))
    while running:
        reap()
        time.sleep(0.02)


def joined(f, face_list):
    """Whether OCCT's face table of `f` joins the file's faces (`face_list`, entities) one to one,
    and why not."""
    js = os.path.join(CORPUS, "oracle", f + ".json")
    tsv = os.path.join(CORPUS, "oracle", f + ".faces.tsv")
    if not os.path.exists(tsv):
        return "no OCCT face table"
    got = json.load(open(js))
    if "error" in got:
        return "OCCT failed"
    named = got.get("faces_named")
    if not got.get("faces_mapped") and not (named and named[0] == named[1]):
        return "OCCT's faces not named"
    ents = {int(line.split("\t")[1]) for line in open(tsv).read().splitlines()[1:]}
    if ents != set(face_list):
        return "OCCT's faces are not the file's"
    return None


def census():
    rows = [line.rstrip("\n").split("\t") for line in open(TABLE) if not line.startswith("#")]
    head, rows = rows[0], rows[1:]
    rows = [dict(zip(head, r)) for r in rows if r and r[0]]
    tally = {}
    for r in rows:
        f = r["path"]
        if r["ref_from"] == "refuse":
            continue
        scan = stepscan.scan(os.path.join(CORPUS, f))
        faces = [e for s in scan["solids"] for e, _ in s["face_list"]]
        why = joined(f, faces) or "joined"
        m = os.path.join(CORPUS, "oracle", f + ".mass.faces.tsv")
        if not os.path.exists(m):
            mass = "measurer: none"
        else:
            lines = open(m).read().splitlines()[2:]
            ok = {int(x.split("\t")[0]) for x in lines if x.split("\t")[0] != "0"
                  and _num(x.split("\t")[1])}
            mass = "measurer: joined" if ok == set(faces) else (
                "measurer: refused" if not ok else "measurer: some faces")
        for k in (why, mass, (why, mass)):
            tally[k] = tally.get(k, 0) + 1
    for k, n in sorted(tally.items(), key=str):
        print(f"{n}\t{k}")


def _num(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


def main():
    args = sys.argv[1:]
    if args[:1] == ["build"]:
        build()
    elif args[:1] == ["measure"]:
        rest, jobs = args[1:], 2
        if "--jobs" in rest:
            i = rest.index("--jobs")
            jobs = int(rest[i + 1])
            del rest[i:i + 2]
        measure(jobs, rest)
    elif args[:1] == ["census"]:
        census()
    else:
        sys.exit(__doc__)


if __name__ == "__main__":
    main()
