"""Bind every actual STL triangle to its CAD surface and mesher UV coordinates."""
import argparse
import json
from pathlib import Path
import struct
import time

from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.Geom import Geom_BSplineSurface, Geom_ConicalSurface, Geom_Plane, Geom_SphericalSurface
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS

from export_face_domains import items
from export_supports import digest, extract
from material_probes import read


def canonical(vertices):
    return min(tuple(vertices[i:]+vertices[:i]) for i in range(3))


def triangles(data):
    if len(data) < 84:
        raise ValueError("incomplete STL")
    count = struct.unpack_from("<I", data, 80)[0]
    if not count or len(data) != 84+50*count:
        raise ValueError("invalid binary STL length")
    return [list(zip(*[iter(struct.unpack_from("<9f", data, 96+50*i))]*3)) for i in range(count)]


def surface_record(s):
    if isinstance(s, Geom_BSplineSurface):
        return dict(kind="polynomial", **extract(s))
    if not isinstance(s, (Geom_Plane, Geom_SphericalSurface, Geom_ConicalSurface)):
        raise ValueError(f"unsupported surface: {type(s).__name__}")
    position = s.Position()
    result = dict(origin=s.Location().Coord(),
                  basis=[position.XDirection().Coord(), position.YDirection().Coord(), position.Direction().Coord()])
    if isinstance(s, Geom_Plane):
        result.update(kind="plane")
    elif isinstance(s, Geom_SphericalSurface):
        result.update(kind="sphere", radius=s.Radius())
    else:
        result.update(kind="cone", radius=s.RefRadius(), angle=s.SemiAngle())
    return result


def run(source, stl, output):
    started = time.perf_counter()
    if output.resolve() in (source.resolve(), stl.resolve(), stl.with_suffix(".mesh.json").resolve()):
        raise ValueError("preserve source artifacts")
    metadata = json.loads(stl.with_suffix(".mesh.json").read_text())
    if metadata["source_sha256"] != digest(source) or metadata["stl_sha256"] != digest(stl):
        raise ValueError("STEP or STL changed since tessellation")
    encoded = triangles(stl.read_bytes())
    lookup = {canonical(t): i for i, t in enumerate(encoded)}
    if len(lookup) != len(encoded):
        raise ValueError("duplicate encoded triangle")
    shape = read(source)
    mesher = BRepMesh_IncrementalMesh(shape, metadata["deflection_mm"], False,
                                    metadata["angular_deflection_rad"], False)
    if not mesher.IsDone():
        raise ValueError("cannot reproduce CAD triangulation")
    faces, seen = [], set()
    for index, raw in enumerate(items(shape, TopAbs_FACE)):
        face = TopoDS.Face_s(raw)
        location = TopLoc_Location()
        mesh = BRep_Tool.Triangulation_s(face, location)
        if mesh is None or not mesh.HasUVNodes():
            raise ValueError("face has no triangulation with UV coordinates")
        nodes = []
        for i in range(1, mesh.NbNodes()+1):
            xyz = mesh.Node(i).Transformed(location.Transformation()).Coord()
            xyz = struct.unpack("<3f", struct.pack("<3f", *xyz))
            nodes.append([*mesh.UVNode(i).Coord(), *xyz])
        indices = []
        for i in range(1, mesh.NbTriangles()+1):
            a, b, c = [j-1 for j in mesh.Triangle(i).Get()]
            if face.Orientation() == TopAbs_REVERSED:
                b, c = c, b
            key = canonical([tuple(nodes[j][2:]) for j in (a, b, c)])
            if key not in lookup or lookup[key] in seen:
                raise ValueError("reproduced triangle differs from actual STL")
            identifier = lookup[key]
            seen.add(identifier)
            indices.append([identifier, a, b, c])
        faces.append(dict(face_index=index, surface=surface_record(BRep_Tool.Surface_s(face)),
                          nodes=nodes, triangles=indices))
    if len(seen) != len(encoded):
        raise ValueError("CAD association omitted STL triangles")
    output.write_text(json.dumps(dict(source_file=str(source.resolve()), source_sha256=digest(source),
        stl_file=str(stl.resolve()), stl_sha256=digest(stl), faces=faces,
        scope="Complete encoded STL to CAD support/UV association; trim coverage and error bounds unchecked"))+"\n")
    print(json.dumps(dict(faces=len(faces), triangles=len(seen), seconds=time.perf_counter()-started)), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("source", "stl", "output"):
        parser.add_argument(name, type=Path)
    args = parser.parse_args()
    run(args.source, args.stl, args.output)
