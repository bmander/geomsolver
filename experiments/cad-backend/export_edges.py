"""Export all shared 3D edges and both incident parameter curves from indexed STEP."""
import argparse
import json
from pathlib import Path

from OCP.BRep import BRep_Tool
from OCP.Geom import Geom_BSplineCurve, Geom_Circle, Geom_Line
from OCP.TopAbs import TopAbs_EDGE, TopAbs_FACE, TopAbs_FORWARD, TopAbs_REVERSED
from OCP.TopExp import TopExp
from OCP.TopTools import TopTools_IndexedMapOfShape
from OCP.TopoDS import TopoDS

from export_analytic_faces import curve_record
from export_face_domains import items
from export_supports import digest
from material_probes import read


def spatial_curve(edge):
    curve = BRep_Tool.Curve_s(edge,0.,0.)
    record = dict(interval=BRep_Tool.Range_s(edge))
    if isinstance(curve,Geom_BSplineCurve):
        if curve.IsPeriodic():
            raise ValueError('periodic spatial spline not supported')
        record.update(kind='bspline',degree=curve.Degree(),
            poles=[curve.Pole(i).Coord() for i in range(1,curve.NbPoles()+1)],
            weights=[curve.Weight(i) for i in range(1,curve.NbPoles()+1)],
            knots=[curve.Knot(i) for i in range(1,curve.NbKnots()+1) for _ in range(curve.Multiplicity(i))])
    elif isinstance(curve,Geom_Line):
        record.update(kind='line',location=curve.Lin().Location().Coord(),direction=curve.Lin().Direction().Coord())
    elif isinstance(curve,Geom_Circle):
        position = curve.Position()
        record.update(kind='circle',origin=curve.Location().Coord(),radius=curve.Radius(),
                      basis=[position.XDirection().Coord(),position.YDirection().Coord()])
    else:
        raise ValueError('unsupported spatial edge curve')
    return record


def run(indexed, analytical, output):
    data = json.loads(indexed.read_text())
    other = json.loads(analytical.read_text())
    if other['indexed_sha256'] != digest(indexed):
        raise ValueError('different analytical inventory')
    records = []
    for item in data['inputs']:
        path = Path(item['step_file'])
        if digest(path) != item['step_sha256']:
            raise ValueError('changed indexed STEP')
        shape = read(path)
        edges = TopTools_IndexedMapOfShape()
        TopExp.MapShapes_s(shape,TopAbs_EDGE,edges)
        member = [dict(edge_index=i,curve=spatial_curve(TopoDS.Edge_s(edges.FindKey(i))),incidences=[])
                  for i in range(1,edges.Extent()+1)]
        for fi,raw in enumerate(items(shape,TopAbs_FACE)):
            face = TopoDS.Face_s(raw)
            for raw_edge in items(face,TopAbs_EDGE):
                edge = TopoDS.Edge_s(raw_edge)
                orientation = edge.Orientation()
                if orientation not in (TopAbs_FORWARD,TopAbs_REVERSED):
                    raise ValueError('unsupported edge orientation')
                index = edges.FindIndex(edge)
                if index <= 0:
                    raise ValueError('face edge missing from inventory')
                member[index-1]['incidences'].append(dict(face_index=fi,reversed=orientation==TopAbs_REVERSED,
                                                        curve=curve_record(edge,face)))
        records.append(dict(member=item['member'],edges=member))
    output.write_text(json.dumps(dict(indexed_file=str(indexed),indexed_sha256=digest(indexed),
        analytical_file=str(analytical),analytical_sha256=digest(analytical),members=records,
        scope='Complete shared-edge inventory and incident parameter curves; correspondence not checked'))+'\n')
    print(f"Extracted {sum(len(r['edges']) for r in records)} shared edges",flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('indexed','analytical','output'):
        parser.add_argument(name,type=Path)
    args = parser.parse_args()
    run(args.indexed,args.analytical,args.output)
