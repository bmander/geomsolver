"""Isolated Manifold callback baseline; no Solvent runtime dependency."""
import argparse
import importlib.metadata
import json
import math
from pathlib import Path
import struct
import time

import manifold3d
import manifold_fixtures as fixtures


def encode(vertices, triangles):
    data = bytearray(b"Unchecked Manifold level-set baseline".ljust(80, b"\0"))
    data += struct.pack("<I", len(triangles))
    for tri in triangles:
        a, b, c = [vertices[i] for i in tri]
        normal = fixtures.cross(fixtures.sub(b, a), fixtures.sub(c, a))
        length = math.sqrt(fixtures.dot(normal, normal))
        if length:
            normal = tuple(x/length for x in normal)
        data += struct.pack("<12fH", *normal, *a, *b, *c, 0)
    return data


def run(name, divisions, output):
    started = time.perf_counter()
    field, extent = fixtures.build(name)
    queries = 0

    def callback(x, y, z):
        nonlocal queries
        queries += 1
        value = field((x, y, z))
        if not math.isfinite(value):
            raise ValueError("unresolved or nonfinite field query")
        return -value  # Manifold requires positive inside.

    setup_seconds = time.perf_counter()-started
    edge_length = 2*extent/divisions
    started = time.perf_counter()
    solid = manifold3d.Manifold.level_set(callback, (-extent,)*3+(extent,)*3,
                                         edge_length, 0, .0002)
    mesh = solid.to_mesh64()  # Include any deferred meshing in extraction time.
    extraction_seconds = time.perf_counter()-started
    vertices = mesh.vert_properties[:, :3].tolist()
    triangles = mesh.tri_verts.tolist()
    stem = f"{name}-divisions{divisions}"
    output.joinpath(stem+".stl").write_bytes(encode(vertices, triangles))
    report = dict(case=name, backend="manifold3d-"+importlib.metadata.version("manifold3d"),
                  depth=None, grid_divisions=divisions, extent=extent, edge_length=edge_length,
                  vertex_tolerance=.0002, setup_seconds=setup_seconds,
                  extraction_seconds=extraction_seconds, point_queries=queries,
                  callback="Python scalar callback; serial LevelSet binding",
                  library_status=str(solid.status()), status="unchecked_expression_baseline",
                  vertices=vertices, triangles=triangles,
                  scope="Closed-form baseline; not a Solvent swept-field adapter or accepted solid.")
    output.joinpath(stem+".json").write_text(json.dumps(report)+"\n")
    print(f"{name}: {len(triangles)} triangles, {queries} queries, "
          f"{extraction_seconds*1000:.1f} ms, {solid.status()}", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case", choices=("all",)+fixtures.NAMES)
    parser.add_argument("divisions", type=int, choices=range(4, 129), metavar="DIVISIONS[4..128]")
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    for name in fixtures.NAMES if args.case == "all" else (args.case,):
        run(name, args.divisions, args.output)
