"""Read finite rectangular spline trims from the STEP bound to an existing mesh audit."""
import argparse
import hashlib
import json
from pathlib import Path

from OCP.BRep import BRep_Tool
from OCP.TopAbs import TopAbs_FACE, TopAbs_WIRE
from OCP.TopoDS import TopoDS

from export_face_domains import items, wire_record
from export_mesh_parameters import surface_record
from export_supports import digest
from material_probes import read


def surface_digest(record):
    return hashlib.sha256(json.dumps(record, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def run(parameters, output):
    data = json.loads(parameters.read_text())
    paths = [parameters, Path(data["source_file"]), Path(data["stl_file"])]
    if output.resolve() in [p.resolve() for p in paths]:
        raise ValueError("preserve input artifacts")
    if digest(paths[1]) != data["source_sha256"] or digest(paths[2]) != data["stl_sha256"]:
        raise ValueError("changed mesh audit input")
    raw_faces = list(items(read(paths[1]), TopAbs_FACE))
    if len(raw_faces) != len(data["faces"]):
        raise ValueError("changed face inventory")
    records = []
    for index, (raw, expected) in enumerate(zip(raw_faces, data["faces"])):
        face = TopoDS.Face_s(raw)
        actual = surface_record(BRep_Tool.Surface_s(face))
        if expected["face_index"] != index or surface_digest(actual) != surface_digest(expected["surface"]):
            raise ValueError(f"face {index} support differs from mesh association")
        row = dict(face_index=index, kind=actual["kind"], surface_sha256=surface_digest(actual))
        if actual["kind"] == "polynomial":
            row["wires"] = [wire_record(TopoDS.Wire_s(w), face) for w in items(face, TopAbs_WIRE)]
        records.append(row)
    output.write_text(json.dumps(dict(parameters_file=str(parameters.resolve()),
        parameters_sha256=digest(parameters), faces=records,
        scope="Native spline-face parameter wires; analytical trims are not included"))+"\n")
    print(f"Extracted {sum(r['kind'] == 'polynomial' for r in records)} spline trims", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("parameters", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    run(args.parameters, args.output)
