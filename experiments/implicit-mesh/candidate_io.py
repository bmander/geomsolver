"""Generator-side binary STL loading; the independent verifier has its own reader."""
import math
import struct


def read_stl(data):
    if len(data) < 84:
        raise ValueError("incomplete STL header")
    count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84+50*count:
        raise ValueError("invalid STL length")
    ids, vertices, triangles = {}, [], []
    for i in range(count):
        row = struct.unpack_from("<9f", data, 96+50*i)
        if not all(map(math.isfinite, row)):
            raise ValueError("nonfinite STL coordinate")
        face = []
        for k in range(3):
            p = row[3*k:3*k+3]
            if p not in ids:
                ids[p] = len(vertices)
                vertices.append(p)
            face.append(ids[p])
        triangles.append(face)
    return vertices, triangles
