"""Preserve a producer's indexed topology through exact degenerate-face cleanup."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time

from candidate_io import read_stl


def write_indexed(source, path):
    vertices, triangles = source["vertices"], source["triangles"]
    with path.open("w") as out:
        out.write(f"OFF\n{len(vertices)} {len(triangles)} 0\n")
        for point in vertices:
            out.write(" ".join(format(x, ".17g") for x in point)+"\n")
        for face in triangles:
            out.write("3 "+" ".join(map(str, face))+"\n")


def run(executable, source, output, near_epsilon=None):
    if source.resolve() in (output.resolve(), output.with_suffix(".json").resolve(),
                            output.with_suffix(".input.off").resolve()):
        raise ValueError("choose a new output stem to preserve the source")
    raw = source.read_bytes()
    metadata = json.loads(raw)
    output.parent.mkdir(parents=True, exist_ok=True)
    indexed = output.with_suffix(".input.off")
    write_indexed(metadata, indexed)
    started = time.perf_counter()
    command = [str(executable), str(indexed), str(output)]
    if near_epsilon is not None:
        command.append(str(near_epsilon))
    completed = subprocess.run(command,
                               text=True, capture_output=True)
    seconds = time.perf_counter()-started
    if completed.returncode not in (0, 1):
        failure = dict(source=source.name, source_sha256=hashlib.sha256(raw).hexdigest(),
                       returncode=completed.returncode, stderr=completed.stderr,
                       status="indexed_cleanup_failed_no_accepted_output")
        Path(str(output)+".failure.json").write_text(json.dumps(failure, indent=2)+"\n")
        raise RuntimeError(completed.stderr)
    result = json.loads(Path(str(output)+".cleanup.json").read_text())
    result.update(source=source.name, source_sha256=hashlib.sha256(raw).hexdigest(),
                  stage_wall_seconds=seconds)
    vertices, triangles = read_stl(output.read_bytes())
    metadata.update(vertices=vertices, triangles=triangles, indexed_cleanup=result,
                    producer_failure=not result["success"],
                    pipeline_seconds=metadata.get("pipeline_seconds", metadata["extraction_seconds"])+seconds)
    output.with_suffix(".json").write_text(json.dumps(metadata)+"\n")
    print(completed.stdout.strip(), flush=True)
    return result["success"]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("executable", type=Path)
    parser.add_argument("source", type=Path, help="producer JSON with original vertex indices")
    parser.add_argument("output", type=Path)
    parser.add_argument("--near-epsilon", type=float)
    args = parser.parse_args()
    if args.source.resolve() == args.output.with_suffix(".json").resolve():
        parser.error("choose a new output stem to preserve the source")
    raise SystemExit(0 if run(args.executable, args.source, args.output, args.near_epsilon) else 1)
