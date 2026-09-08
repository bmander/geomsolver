"""Explicit cleanup experiment; output remains unchecked until audit.py accepts it.

Remove exactly zero-area encoded faces, then optionally import into Manifold and
simplify. The original STL is retained. This is not a production repair algorithm.
"""
import argparse
import importlib.metadata
import json
from pathlib import Path
import time

import numpy as np
import manifold3d

import audit
from candidate_io import read_stl
from manifold_bench import encode


def run(source, output, tolerance):
    started = time.perf_counter()
    data = source.read_bytes()
    vertices, triangles = read_stl(data)
    ratios = {x: x.as_integer_ratio() for v in vertices for x in v}
    exponent = max((d.bit_length()-1 for _, d in ratios.values()), default=0)
    scaled = {x: n << (exponent-d.bit_length()+1) for x, (n, d) in ratios.items()}
    integers = [tuple(scaled[x] for x in p) for p in vertices]
    kept, removed = [], []
    for i, face in enumerate(triangles):
        a, b, c = [integers[k] for k in face]
        if any(audit.cross(audit.sub(b, a), audit.sub(c, a))):
            kept.append(face)
        else:
            removed.append(i)
    status = None
    imported = tolerance is not None and bool(kept)
    if imported:
        solid = manifold3d.Manifold(manifold3d.Mesh64(np.array(vertices, dtype=np.float64),
                                                     np.array(kept, dtype=np.uint64)))
        if tolerance > 0:
            solid = solid.simplify(tolerance)
        status = str(solid.status())
        mesh = solid.to_mesh64()
        vertices, kept = mesh.vert_properties[:, :3].tolist(), mesh.tri_verts.tolist()
    seconds = time.perf_counter()-started
    metadata = json.loads(source.with_suffix(".json").read_text())
    metadata.update(vertices=vertices, triangles=kept,
                    postprocessing_seconds=seconds,
                    pipeline_seconds=metadata["extraction_seconds"]+seconds,
                    postprocessing=dict(source=source.name, removed_zero_area_faces=removed,
                                        manifold_version=importlib.metadata.version("manifold3d"),
                                        imported_into_manifold=imported,
                                        simplify_tolerance=tolerance))
    if status is not None:
        metadata["library_status"] = status
    output.write_bytes(encode(vertices, kept))
    output.with_suffix(".json").write_text(json.dumps(metadata)+"\n")
    print(f"{output.name}: removed {len(removed)} zero-area faces; "
          f"{len(kept)} output triangles; {seconds*1000:.1f} ms cleanup; {status}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--simplify", type=float)
    args = parser.parse_args()
    if args.source.resolve() == args.output.resolve():
        parser.error("source must be retained; choose a different output path")
    if args.simplify is not None and not 0 <= args.simplify <= .02:
        parser.error("simplification tolerance must be 0..0.02")
    run(args.source, args.output, args.simplify)
