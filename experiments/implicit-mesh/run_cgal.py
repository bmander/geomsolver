"""Run the isolated native repair and retain provenance for independent auditing."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time

from candidate_io import read_stl


def run(executable, source, output):
    metadata = json.loads(source.with_suffix(".json").read_text())
    data = source.read_bytes()
    _, faces = read_stl(data)
    started = time.perf_counter()
    if faces:
        completed = subprocess.run([str(executable), str(source), str(output)],
                                   text=True, capture_output=True)
        if completed.returncode not in (0, 1):
            failure = dict(source=source.name, source_sha256=hashlib.sha256(data).hexdigest(),
                           returncode=completed.returncode, stdout=completed.stdout,
                           stderr=completed.stderr, stage_wall_seconds=time.perf_counter()-started,
                           status="repair_failed_no_accepted_output")
            Path(str(output)+".failure.json").write_text(json.dumps(failure, indent=2)+"\n")
            raise RuntimeError(completed.stderr)
        result = json.loads(Path(str(output)+".repair.json").read_text())
        print(completed.stdout.strip())
    else:
        output.write_bytes(data)
        result = dict(success=True, empty_input=True, repair_seconds=0)
    seconds = time.perf_counter()-started
    vertices, triangles = read_stl(output.read_bytes())
    result.update(source=source.name, source_sha256=hashlib.sha256(data).hexdigest(),
                  stage_wall_seconds=seconds)
    metadata.update(vertices=vertices, triangles=triangles, cgal_repair=result,
                    producer_failure=not result["success"],
                    pipeline_seconds=metadata.get("pipeline_seconds", metadata["extraction_seconds"])+seconds)
    output.with_suffix(".json").write_text(json.dumps(metadata)+"\n")
    return result["success"]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("executable", type=Path)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    if args.source.resolve() == args.output.resolve():
        parser.error("source must be retained; choose a different output path")
    raise SystemExit(0 if run(args.executable, args.source, args.output) else 1)
