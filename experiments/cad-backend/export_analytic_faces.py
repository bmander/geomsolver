"""Extract remaining analytical faces and their finite parameter curves from STEP."""
import argparse
import json
from pathlib import Path

from OCP.BRep import BRep_Tool
from OCP.BRepTools import BRepTools_WireExplorer
from OCP.Geom import Geom_ConicalSurface, Geom_SphericalSurface
from OCP.Geom2d import Geom2d_Line, Geom2d_BSplineCurve
from OCP.TopAbs import TopAbs_FACE, TopAbs_WIRE, TopAbs_EDGE
from OCP.TopoDS import TopoDS

from export_face_domains import items
from export_supports import digest
from material_probes import read


def curve_record(edge, face):
    curve = BRep_Tool.CurveOnSurface_s(edge,face,0.,0.)
    record = dict(interval=BRep_Tool.Range_s(edge,face))
    if isinstance(curve,Geom2d_Line):
        record.update(kind='line',location=curve.Location().Coord(),direction=curve.Direction().Coord())
    elif isinstance(curve,Geom2d_BSplineCurve):
        if curve.IsPeriodic():
            raise ValueError('periodic parameter spline not supported')
        record.update(kind='bspline',degree=curve.Degree(),
            poles=[curve.Pole(i).Coord() for i in range(1,curve.NbPoles()+1)],
            weights=[curve.Weight(i) for i in range(1,curve.NbPoles()+1)],
            knots=[curve.Knot(i) for i in range(1,curve.NbKnots()+1) for _ in range(curve.Multiplicity(i))])
    else:
        raise ValueError('unsupported analytical-face parameter curve')
    return record


def run(indexed, supports, output):
    data = json.loads(indexed.read_text())
    nominal = json.loads(supports.read_text())
    source = Path(data['source_file'])
    if digest(source) != data['source_sha256']:
        raise ValueError('changed source')
    solved = json.loads(source.read_text())
    if solved['module_mm'] != nominal['module_mm']:
        raise ValueError('different nominal module')
    for a,b in zip(solved['members'],nominal['members']):
        if any(a[k] != b[k] for k in ('member','teeth','blank_meridian')):
            raise ValueError('different nominal blank')
    records = []
    for item in data['inputs']:
        path = Path(item['step_file'])
        if digest(path) != item['step_sha256']:
            raise ValueError('changed indexed STEP')
        faces = list(items(read(path),TopAbs_FACE))
        for expected in item['other_faces']:
            face = TopoDS.Face_s(faces[expected['face_index']])
            surface = BRep_Tool.Surface_s(face)
            if type(surface).__name__ != expected['kind']:
                raise ValueError('changed analytical face inventory')
            position = surface.Position()
            record = dict(member=item['member'],face_index=expected['face_index'],origin=surface.Location().Coord(),
                basis=[position.XDirection().Coord(),position.YDirection().Coord(),position.Direction().Coord()])
            if isinstance(surface,Geom_ConicalSurface):
                record.update(kind='cone',radius=surface.RefRadius(),angle=surface.SemiAngle())
            elif isinstance(surface,Geom_SphericalSurface):
                record.update(kind='sphere',radius=surface.Radius())
            else:
                raise ValueError('unexpected analytical surface')
            wires = []
            for raw in items(face,TopAbs_WIRE):
                wire = TopoDS.Wire_s(raw)
                explorer = BRepTools_WireExplorer(wire,face)
                edges = []
                while explorer.More():
                    edges.append(curve_record(explorer.Current(),face))
                    explorer.Next()
                if len(edges) != len(list(items(wire,TopAbs_EDGE))):
                    raise ValueError('wire traversal omitted an edge')
                wires.append(edges)
            records.append(dict(**record,wires=wires))
    output.write_text(json.dumps(dict(indexed_file=str(indexed),indexed_sha256=digest(indexed),
        supports_file=str(supports),supports_sha256=digest(supports),faces=records,
        scope='Analytical supports and parameter-curve coefficients from indexed STEP; bounds not yet checked'))+'\n')
    print(f'Extracted {len(records)} analytical faces',flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('indexed','supports','output'):
        parser.add_argument(name,type=Path)
    args = parser.parse_args()
    run(args.indexed,args.supports,args.output)
