"""Exact finite-face membership for mesh witnesses on natural rectangular spline faces.

Combined with the separately checked triangle bounds for the same parameters,
these domains transfer forward support-distance bounds to actual finite faces.
This checker does not recompute or authenticate a supplied distance report.
"""
import argparse
from fractions import Fraction as F
import hashlib
import json
from pathlib import Path
import time

from check_face_domains import rectangle
from spline_embedding import polynomial_surface


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify(parameters, trims):
    faces = parameters["faces"]
    expected = list(range(len(faces)))
    if (not faces or [f["face_index"] for f in faces] != expected
            or [f["face_index"] for f in trims["faces"]] != expected):
        raise ValueError("incomplete or reordered face inventory")
    verified, remaining = [], []
    for face, trim in zip(faces, trims["faces"]):
        surface = face["surface"]
        encoded = json.dumps(surface, sort_keys=True, separators=(",", ":")).encode()
        if (trim["kind"] != surface["kind"]
                or trim["surface_sha256"] != hashlib.sha256(encoded).hexdigest()):
            raise ValueError("trim and mesh support disagree")
        if surface["kind"] != "polynomial":
            remaining.append(face["face_index"])
            continue
        _, knots, _ = polynomial_surface(surface)
        if any(k[0] != 0 or k[-1] != 1 for k in knots):
            raise ValueError("bounded spline domain differs from the unit square")
        domain = rectangle(trim["wires"])
        if not face["nodes"] or not face["triangles"]:
            raise ValueError("empty spline-face mesh")
        if any(len(p) != 5 or any(not 0 <= F(x) <= 1 for x in p[:2]) for p in face["nodes"]):
            raise ValueError("mesh parameter vertex leaves the finite face")
        verified.append(dict(face_index=face["face_index"], triangles=len(face["triangles"]), **domain))
    if not verified:
        raise ValueError("no rectangular spline faces were checked")
    return dict(rectangular_faces=verified, unverified_face_indices=remaining,
                rectangular_face_triangles=sum(f["triangles"] for f in verified))


def reference_identity(parameters, reference, member):
    current = {f["face_index"]: f["surface"] for f in parameters["faces"] if f["surface"]["kind"] == "polynomial"}
    rows = [r for r in reference["surfaces"] if r["member"] == member]
    if len(rows) != len(current) or {r["face_index"] for r in rows} != set(current):
        raise ValueError("reference has a different spline-face inventory")
    for r in rows:
        actual = {k:v for k,v in current[r["face_index"]].items() if k != "kind"}
        if actual != r["surface"]:
            raise ValueError("current spline differs from the reference coefficients")
        rectangle(r["wires"])
    return dict(member=member, identical_spline_faces=len(rows),
                scope="Exact coefficient and natural-domain identity; prior reference error bounds are separate evidence")


def run(source, output, reference=None, member=None):
    started = time.perf_counter()
    data = json.loads(source.read_text())
    parameters = Path(data["parameters_file"])
    if digest(parameters) != data["parameters_sha256"]:
        raise ValueError("changed mesh parameter input")
    model = json.loads(parameters.read_text())
    paths = [source, parameters, Path(model["source_file"]), Path(model["stl_file"])]
    if reference is not None:
        paths.append(reference)
    if output.resolve() in [p.resolve() for p in paths]:
        raise ValueError("preserve input artifacts")
    for role in ("source", "stl"):
        if digest(Path(model[f"{role}_file"])) != model[f"{role}_sha256"]:
            raise ValueError("changed STEP or STL input")
    result = dict(status="verified_rectangular_domains", **verify(model, data),
                  input_sha256=digest(source), parameters_sha256=digest(parameters),
                  source_sha256=model["source_sha256"], stl_sha256=model["stl_sha256"],
                  seconds=time.perf_counter()-started,
                  scope="Finite unit-square spline domains and original mesh UV membership. The triangle-bound evaluator also confines all refined witnesses to these same domains. Forward finite-face distance follows when combined with its verified bound for this exact parameter input. Analytical trims, reverse coverage and nominal source accuracy remain separate.")
    if (reference is None) != (member is None):
        raise ValueError("reference and member must be supplied together")
    if reference is not None:
        result["reference_identity"] = dict(file=str(reference), sha256=digest(reference),
            **reference_identity(model, json.loads(reference.read_text()), member))
    result["seconds"] = time.perf_counter()-started
    output.write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps(dict(rectangular_faces=len(result["rectangular_faces"]),
        triangles=result["rectangular_face_triangles"], unverified_faces=len(result["unverified_face_indices"]),
        seconds=result["seconds"])), flush=True)
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--reference", type=Path)
    parser.add_argument("--member", type=int, choices=(0, 1))
    args = parser.parse_args()
    run(args.source, args.output, args.reference, args.member)
