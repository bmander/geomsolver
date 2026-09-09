"""Tessellate a STEP candidate with OCCT; the output still requires an independent audit."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import time

from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.StlAPI import StlAPI_Writer
from material_probes import read


def run(source, output, deflection=.05):
    if not math.isfinite(deflection) or deflection <= 0:
        raise ValueError("deflection must be positive and finite")
    if source.resolve() == output.resolve():
        raise ValueError("preserve the source STEP file")
    shape = read(source)
    started = time.perf_counter()
    mesher = BRepMesh_IncrementalMesh(shape, deflection, False, .2, False)
    if not mesher.IsDone():
        raise RuntimeError("CAD tessellation failed")
    seconds = time.perf_counter()-started
    writer = StlAPI_Writer()
    writer.ASCIIMode = False
    if not writer.Write(shape, str(output)):
        raise RuntimeError("STL write failed")
    data = output.read_bytes()
    report = dict(source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  stl_sha256=hashlib.sha256(data).hexdigest(), deflection_mm=deflection,
                  angular_deflection_rad=.2, tessellation_seconds=seconds,
                  triangles=struct.unpack_from("<I", data, 80)[0],
                  scope="Unchecked CAD tessellation; settings are not a source-geometry accuracy certificate")
    output.with_suffix(".mesh.json").write_text(json.dumps(report, indent=2)+"\n")
    print(json.dumps(report), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--deflection", type=float, default=.05)
    args = parser.parse_args()
    run(args.source, args.output, args.deflection)
