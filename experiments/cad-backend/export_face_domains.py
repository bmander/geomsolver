"""Export finite face wires from the already bounded tooth-space STEP surfaces."""
import argparse
import json
from pathlib import Path

from OCP.BRep import BRep_Tool
from OCP.BRepTools import BRepTools_WireExplorer
from OCP.Geom2d import Geom2d_Line
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_FORWARD, TopAbs_REVERSED, TopAbs_WIRE
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS

from export_supports import digest, extract
from material_probes import read


def items(shape, kind):
    explorer = TopExp_Explorer(shape, kind)
    while explorer.More():
        yield explorer.Current()
        explorer.Next()


def wire_record(wire, face):
    explorer = BRepTools_WireExplorer(wire, face)
    edges = []
    while explorer.More():
        edge = explorer.Current()
        curve = BRep_Tool.CurveOnSurface_s(edge, face, 0., 0.)
        if not isinstance(curve, Geom2d_Line):
            raise ValueError("expected a straight parameter boundary")
        orientation = edge.Orientation()
        if orientation not in (TopAbs_FORWARD, TopAbs_REVERSED):
            raise ValueError("unsupported edge orientation")
        edges.append(dict(location=curve.Location().Coord(), direction=curve.Direction().Coord(),
                          interval=BRep_Tool.Range_s(edge, face), reversed=orientation == TopAbs_REVERSED))
        explorer.Next()
    if len(edges) != len(list(items(wire, TopAbs_EDGE))):
        raise ValueError("wire traversal omitted edges")
    return edges


def run(coefficients, output):
    definitions = [json.loads(p.read_text()) for p in coefficients]
    inputs = definitions[0]["inputs"]
    if len(inputs) != 2 or {r['member'] for r in inputs} != {0,1}:
        raise ValueError("expected both tooth-space inputs")
    if any(d['inputs'] != inputs or d['source_sha256'] != definitions[0]['source_sha256'] for d in definitions):
        raise ValueError("surface bounds use different inputs")
    records = []
    for item in inputs:
        path = Path(item['step_file'])
        if digest(path) != item['step_sha256']:
            raise ValueError("changed tooth-space STEP")
        faces = list(items(read(path), TopAbs_FACE))
        if len(faces) != 10:
            raise ValueError("expected ten tooth-space faces")
        for index, raw in enumerate(faces):
            face = TopoDS.Face_s(raw)
            matches = [(str(p), r) for p,d in zip(coefficients,definitions) for r in d['surfaces']
                       if (r['member'],r['face_index']) == (item['member'],index)]
            if len(matches) != 1:
                raise ValueError("face does not have exactly one bounded reference")
            reference_file, reference = matches[0]
            if extract(BRep_Tool.Surface_s(face)) != reference['surface']:
                raise ValueError("face support differs from bounded coefficients")
            records.append(dict(member=item['member'], face_index=index, coefficient_file=reference_file,
                                wires=[wire_record(TopoDS.Wire_s(w),face) for w in items(face,TopAbs_WIRE)]))
    report = dict(inputs=inputs, coefficients=[dict(file=str(p),sha256=digest(p)) for p in coefficients],
                  faces=records, scope='Extracted ordered parameter wires; independent finite-domain check pending')
    output.write_text(json.dumps(report,indent=2)+'\n')
    print(f'Extracted {len(records)} finite face domains',flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path)
    parser.add_argument('coefficients',type=Path,nargs=3)
    args = parser.parse_args()
    run(args.coefficients,args.output)
