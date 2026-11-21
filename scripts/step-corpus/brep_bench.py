#!/usr/bin/env python3
"""brep-bench's settled cases as other kernels write them: STEP files for the corpus (#993 stage 1).

    python3 -I scripts/step-corpus/brep_bench.py prepare            # copy the bench's checkout
    python3 -I scripts/step-corpus/brep_bench.py build KERNEL...    # build their runners (and
                                                                    # `mass`, the bench's measurer)
    python3 -I scripts/step-corpus/brep_bench.py run KERNEL... [--jobs 2] [--timeout 120] [--case P]
    python3 -I scripts/step-corpus/brep_bench.py references         # the reference shapes' files
    python3 -I scripts/step-corpus/brep_bench.py manifest           # the committed closed forms
    python3 -I scripts/step-corpus/brep_bench.py census             # entity kinds, per source

brep-bench (`$BREP_BENCH`, default /home/user/brep-bench) is read and never written: `prepare`
copies its tracked files to `$STEP_CORPUS/build/brep-bench`, where the runners are built and run
(a runner's build writes `target/` and `Cargo.lock` beside its sources). A case is *settled* where
its header gives `[volume]` and `[area]` and does not say `settled = false` (SPEC.md); only those
are run. A kernel runs a case as the harness does (SPEC.md, "A runner": `<command> CASE OUT.stl
OUT.step`, from the bench's root), and the STEP it writes is kept as
`$STEP_CORPUS/brep-bench/<kernel>/<category>/<case>.step`; each run's outcome (`ok`, `refused`
with the runner's reason, `no-step` with its `step_error`, `crashed`, `timeout`) is a row of
`$STEP_CORPUS/brep-bench/<kernel>/runs.tsv`. A case whose file is there is not run again
(`--force` runs it).

The geomsolver kernel's runner calls `solventc` built from the release its build.py names
(TAG/COMMIT); `build geomsolver` builds it from that commit of this repository's own history
(`git archive`, no network) and puts it where the runner looks.
"""
import json
import os
import re
import shutil
import subprocess
import sys
import time
import tomllib

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
CORPUS = os.environ.get("STEP_CORPUS", "/home/user/step-corpus")
SOURCE = os.environ.get("BREP_BENCH", "/home/user/brep-bench")
BENCH = os.path.join(CORPUS, "build", "brep-bench")
OUT = os.path.join(CORPUS, "brep-bench")
MANIFEST = os.path.join(REPO, "rust", "gcs-core", "tests", "step_corpus", "brep_bench_cases.tsv")
# the bench's grade (harness/run.py's EXACT), where a case gives no tolerance of its own
EXACT = 1e-6
JOBS = int(os.environ.get("JOBS", "2"))
# the OCCT runner's interpreter: an environment of its own with the OCP the bench's cases need
OCCT_OCP = "8.0.1.1.0"
OCCT_PYTHON = os.environ.get("OCCT_PYTHON", os.path.join(CORPUS, "build", "venv-ocp8", "bin",
                                                         "python"))


def kernels():
    with open(os.path.join(BENCH, "kernels.json")) as f:
        return {k["name"]: k for k in json.load(f)["kernels"]}


def cases():
    """Every case: id, its header, settled or not."""
    out = []
    for d, _, fs in os.walk(os.path.join(BENCH, "cases")):
        for f in fs:
            if not f.endswith(".md"):
                continue
            p = os.path.join(d, f)
            text = open(p, encoding="utf-8").read()
            meta = tomllib.loads(text.split("+++", 2)[1])
            cid = os.path.relpath(p, os.path.join(BENCH, "cases"))[:-3].replace(os.sep, "/")
            settled = meta.get("settled", True) and "volume" in meta and "area" in meta
            out.append({"id": cid, "meta": meta, "settled": settled})
    return sorted(out, key=lambda c: c["id"])


def source(k, cid):
    category, _, name = cid.rpartition("/")
    pattern = k.get("source")
    return pattern and os.path.join(BENCH, pattern.format(case=cid, category=category, name=name,
                                                          name_snake=name.replace("-", "_")))


def prepare():
    if os.path.exists(BENCH):
        print(f"{BENCH} is there; remove it to copy the bench afresh")
        return
    os.makedirs(BENCH)
    files = subprocess.run(["git", "-C", SOURCE, "ls-files", "-z"], check=True,
                           capture_output=True).stdout
    tar = subprocess.Popen(["tar", "--null", "-T", "-", "-cf", "-"], cwd=SOURCE,
                           stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    untar = subprocess.Popen(["tar", "-xf", "-", "-C", BENCH], stdin=tar.stdout)
    tar.stdin.write(files)
    tar.stdin.close()
    untar.wait()
    tar.wait()
    commit = subprocess.run(["git", "-C", SOURCE, "rev-parse", "HEAD"], check=True,
                            capture_output=True, text=True).stdout.strip()
    with open(os.path.join(BENCH, "COMMIT"), "w") as f:
        f.write(commit + "\n")
    print(f"brep-bench {commit[:12]} copied to {BENCH}")


def build(names):
    ks = kernels()
    for name in names:
        k = ks[name]
        clock = time.time()
        if name == "mass":
            ok = subprocess.run(["cargo", "build", "-j", os.environ.get("CARGO_JOBS", "2"),
                                 "--release", "--manifest-path",
                                 os.path.join(BENCH, "harness", "mass", "Cargo.toml")]).returncode == 0
            print(f"mass: {'built' if ok else 'FAILED'}")
            continue
        if name == "geomsolver":
            ok = build_geomsolver()
        elif name == "occt":
            # the bench's OCCT cases use OCP 8 (`OCP.collections`); the oracle stays on the
            # system's OCP, so the runner has an environment of its own
            venv = os.path.dirname(os.path.dirname(OCCT_PYTHON))
            if not os.path.exists(OCCT_PYTHON):
                subprocess.run([sys.executable, "-m", "venv", venv], check=True)
            ok = subprocess.run([OCCT_PYTHON, "-m", "pip", "install", "-q",
                                 f"cadquery-ocp=={OCCT_OCP}"]).returncode == 0
        elif k.get("build", [None])[0] == "cargo":
            ok = subprocess.run(["cargo", "build", "-j", os.environ.get("CARGO_JOBS", "2"),
                                 "--release", "--manifest-path",
                                 os.path.join(BENCH, "kernels", name, "Cargo.toml")]).returncode == 0
        else:
            print(f"{name}: not built here (its build is {k.get('build')})")
            continue
        print(f"{name}: {'built' if ok else 'FAILED'} in {time.time() - clock:.0f} s", flush=True)


def build_geomsolver():
    text = open(os.path.join(BENCH, "kernels", "geomsolver", "build.py")).read()
    tag = re.search(r'TAG = "([^"]+)"', text).group(1)
    commit = re.search(r'COMMIT = "([0-9a-f]+)"', text).group(1)
    tree = os.path.join(CORPUS, "build", "geomsolver-" + tag)
    if not os.path.exists(os.path.join(tree, "rust", "Cargo.toml")):
        os.makedirs(tree, exist_ok=True)
        if subprocess.run(["git", "-C", REPO, "cat-file", "-e", commit]).returncode != 0:
            subprocess.run(["git", "-C", REPO, "fetch", "-q", "origin", f"refs/tags/{tag}"],
                           check=True)
        arch = subprocess.Popen(["git", "-C", REPO, "archive", commit, "rust"],
                                stdout=subprocess.PIPE)
        subprocess.run(["tar", "-x", "-C", tree], stdin=arch.stdout, check=True)
        arch.wait()
    if subprocess.run(["cargo", "build", "-j", os.environ.get("CARGO_JOBS", "2"), "--release",
                       "--manifest-path", os.path.join(tree, "rust", "Cargo.toml"),
                       "-p", "gcs-cli"]).returncode:
        return False
    bindir = os.path.join(BENCH, "kernels", "geomsolver", "bin")
    os.makedirs(bindir, exist_ok=True)
    shutil.copy2(os.path.join(tree, "rust", "target", "release", "solventc"),
                 os.path.join(bindir, "solventc"))
    with open(os.path.join(bindir, "TAG"), "w") as f:
        f.write(f"{tag} {commit[:12]}\n")
    return True


def last_json(text):
    for line in reversed(text.strip().splitlines()):
        try:
            return json.loads(line)
        except ValueError:
            continue
    return None


def run(names, jobs, timeout, prefix, force):
    ks = kernels()
    chosen = [c for c in cases() if c["settled"] and c["id"].startswith(prefix)]
    for name in names:
        k = ks[name]
        if not k.get("command") or k.get("step") is not True:
            print(f"{name}: no runner that writes STEP")
            continue
        runs_path = os.path.join(OUT, name, "runs.tsv")
        runs = {}
        if os.path.exists(runs_path):
            for line in open(runs_path).read().splitlines()[1:]:
                f = line.split("\t")
                runs[f[0]] = f
        todo = []
        for c in chosen:
            src = source(k, c["id"])
            if src and not os.path.exists(src):
                runs[c["id"]] = [c["id"], "missing", "0", "no implementation"]
                continue
            step = os.path.join(OUT, name, c["id"] + ".step")
            if not force and c["id"] in runs and runs[c["id"]][1] != "missing" and (
                    runs[c["id"]][1] != "ok" or os.path.exists(step)):
                continue
            todo.append(c)
        print(f"{name}: {len(todo)} cases to run, {jobs} at once", flush=True)
        running = []

        def finish(item, killed):
            p, c, start, tmp = item
            seconds = time.time() - start
            out = open(os.path.join(tmp, "out")).read() if os.path.exists(
                os.path.join(tmp, "out")) else ""
            err = open(os.path.join(tmp, "err")).read() if os.path.exists(
                os.path.join(tmp, "err")) else ""
            reply = last_json(out) or {}
            step = os.path.join(tmp, "result.step")
            if killed:
                row = ["timeout", f"over {timeout:g} s"]
            elif p.returncode != 0:
                row = ["crashed", (err.strip().splitlines() or [f"exit {p.returncode}"])[-1]]
            elif reply.get("status") == "refused":
                row = ["refused", reply.get("reason", "")]
            elif reply.get("status") == "ok" and os.path.exists(step):
                dest = os.path.join(OUT, name, c["id"] + ".step")
                os.makedirs(os.path.dirname(dest), exist_ok=True)
                shutil.move(step, dest)
                row = ["ok", ""]
            elif reply.get("status") == "ok":
                row = ["no-step", str(reply.get("step_error", "no STEP and no step_error"))]
            else:
                row = ["crashed", f"no reply: {out.strip()[-200:]}"]
            shutil.rmtree(tmp, ignore_errors=True)
            why = re.sub(r"\s+", " ", row[1])[:300]
            runs[c["id"]] = [c["id"], row[0], f"{seconds:.2f}", why]
            print(f"{name:<12} {row[0]:<8} {c['id']:<52} {seconds:6.1f}s {why[:60]}", flush=True)

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
        for c in todo:
            while len(running) >= jobs:
                reap()
                time.sleep(0.05)
            tmp = os.path.join(CORPUS, "build", "runs", name, c["id"].replace("/", "--"))
            os.makedirs(tmp, exist_ok=True)
            python = OCCT_PYTHON if name == "occt" else sys.executable
            argv = [a if a != "python3" else python for a in k["command"]]
            argv = [os.path.join(BENCH, a) if a.startswith("kernels/") else a for a in argv]
            p = subprocess.Popen(argv + [c["id"], os.path.join(tmp, "result.stl"),
                                         os.path.join(tmp, "result.step")], cwd=BENCH,
                                 stdout=open(os.path.join(tmp, "out"), "w"),
                                 stderr=open(os.path.join(tmp, "err"), "w"),
                                 stdin=subprocess.DEVNULL)
            running.append((p, c, time.time(), tmp))
        while running:
            reap()
            time.sleep(0.05)
        os.makedirs(os.path.dirname(runs_path), exist_ok=True)
        with open(runs_path, "w") as f:
            f.write("case\toutcome\tseconds\twhy\n")
            for cid in sorted(runs):
                f.write("\t".join(runs[cid]) + "\n")
        tally = {}
        for r in runs.values():
            tally[r[1]] = tally.get(r[1], 0) + 1
        print(f"{name}: {tally}", flush=True)


def references():
    """A case that names a reference shape (SPEC.md, "A reference shape") gives its file: it is
    copied, unzipped, as `reference/<category>/<case>.step` (the file another writer wrote, whose
    answer is its own measures)."""
    import gzip
    for c in cases():
        ref = c["meta"].get("reference")
        if not c["settled"] or not ref:
            continue
        src = os.path.join(BENCH, "cases", os.path.dirname(c["id"]), ref)
        dest = os.path.join(OUT, "reference", c["id"] + ".step")
        os.makedirs(os.path.dirname(dest), exist_ok=True)
        with (gzip.open(src) if src.endswith(".gz") else open(src, "rb")) as f:
            data = f.read()
        with open(dest, "wb") as f:
            f.write(data)
        print(f"reference {c['id']}: {len(data)} bytes")


def manifest():
    """The settled cases' closed forms, with the tolerance the bench grades each at."""
    commit = open(os.path.join(BENCH, "COMMIT")).read().strip()
    with open(MANIFEST, "w") as f:
        f.write(f"# brep-bench {commit}: each settled case's closed forms (its header's values) "
                f"and the relative tolerance the bench grades at (its own, else {EXACT:g})\n")
        f.write("case\tvolume\tarea\ttolerance\tcentroid\treference\n")
        for c in cases():
            if not c["settled"]:
                continue
            m = c["meta"]
            centroid = ",".join(repr(float(x)) for x in m["centroid"]["value"]) \
                if "centroid" in m else ""
            f.write(f"{c['id']}\t{float(m['volume']['value'])!r}\t{float(m['area']['value'])!r}\t"
                    f"{float(m.get('tolerance', EXACT))!r}\t{centroid}\t{m.get('reference', '')}\n")
    print(f"{MANIFEST}: {sum(1 for c in cases() if c['settled'])} settled cases")


def census():
    """Each entity kind the corpus's files hold, counted per source (cadgenbench, and each
    kernel's files): the files holding it and its instances."""
    sys.path.insert(0, HERE)
    import stepscan
    by = {}
    for src, root in [("cadgenbench", os.path.join(CORPUS, "cadgenbench"))] + [
            (k, os.path.join(OUT, k)) for k in sorted(os.listdir(OUT))
            if os.path.isdir(os.path.join(OUT, k))]:
        for d, _, fs in os.walk(root):
            for f in fs:
                if f.endswith(".step"):
                    s = stepscan.scan(os.path.join(d, f))
                    for kind, n in s["census"].items():
                        files, count = by.setdefault(kind, {}).get(src, (0, 0))
                        by[kind][src] = (files + 1, count + n)
    srcs = sorted({s for v in by.values() for s in v}, key=lambda s: (s != "cadgenbench", s))
    print("kind\t" + "\t".join(srcs))
    for kind in sorted(by):
        print(kind + "\t" + "\t".join(f"{by[kind][s][0]}/{by[kind][s][1]}" if s in by[kind] else ""
                                      for s in srcs))


def main():
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        return
    cmd, rest = args[0], args[1:]
    opts = {"--jobs": JOBS, "--timeout": 120.0, "--case": ""}
    names, force = [], False
    while rest:
        a = rest.pop(0)
        if a in opts:
            opts[a] = type(opts[a])(rest.pop(0))
        elif a == "--force":
            force = True
        else:
            names.append(a)
    if cmd == "prepare":
        prepare()
    elif cmd == "build":
        build(names)
    elif cmd == "run":
        run(names, opts["--jobs"], opts["--timeout"], opts["--case"], force)
    elif cmd == "references":
        references()
    elif cmd == "manifest":
        manifest()
    elif cmd == "census":
        census()
    else:
        sys.exit(f"unknown command {cmd}")


if __name__ == "__main__":
    main()
