"""Diagnose indexed components and coincident vertices before/after STL rounding.

This is structural evidence, not an acceptance check or a repair operation.
"""
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import struct
import sys

import audit


def inspect(source):
    vertices, triangles = source["vertices"], source["triangles"]
    groups = audit.components([tuple((i,) for i in t) for t in triangles])
    owner, components = {}, []
    for component, group in enumerate(groups):
        indices = sorted({i for f in group for i in triangles[f]})
        for i in indices:
            owner[i] = component
        faces = [tuple(vertices[i] for i in triangles[f]) for f in group]
        volume = sum(audit.dot(a, audit.cross(b, c)) for a, b, c in faces)/6
        components.append(dict(triangles=len(group), vertices=len(indices),
                               signed_volume_approx=volume,
                               bounds=[[min(vertices[i][k] for i in indices) for k in range(3)],
                                       [max(vertices[i][k] for i in indices) for k in range(3)]]))
    collisions = {}
    for precision in ("native64", "stl32"):
        positions = defaultdict(list)
        for i in owner:
            p = tuple(vertices[i])
            if precision == "stl32":
                p = struct.unpack("<3f", struct.pack("<3f", *p))
            positions[p].append(i)
        collisions[precision] = [dict(position=p, vertex_indices=indices,
                                      components=[owner[i] for i in indices])
                                 for p, indices in positions.items() if len(indices) > 1]
    return dict(indexed_components=components, coincident_vertices=collisions,
                scope="Indexed connectivity and coordinate coincidences only; not a solid certificate.")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("usage: python3 inspect_components.py PRODUCER.json")
    path = Path(sys.argv[1])
    data = path.read_bytes()
    result = inspect(json.loads(data))
    result.update(source=path.name, source_sha256=hashlib.sha256(data).hexdigest())
    path.with_suffix(".structure.json").write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps(result))
