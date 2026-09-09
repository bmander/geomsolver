"""Export indexed STEP vertex positions and their shared-edge endpoint incidence."""
import argparse
import json
from pathlib import Path

from OCP.BRep import BRep_Tool
from OCP.TopAbs import TopAbs_EDGE,TopAbs_VERTEX
from OCP.TopExp import TopExp
from OCP.TopTools import TopTools_IndexedMapOfShape
from OCP.TopoDS import TopoDS

from export_supports import digest
from material_probes import read


def run(edges,output):
    edge_data=json.loads(edges.read_text())
    indexed=Path(edge_data['indexed_file'])
    if digest(indexed)!=edge_data['indexed_sha256']:
        raise ValueError('changed indexed edge reference')
    data=json.loads(indexed.read_text())
    members=[]
    for item in data['inputs']:
        path=Path(item['step_file'])
        if digest(path)!=item['step_sha256']:
            raise ValueError('changed indexed STEP')
        shape=read(path)
        vertices,curves=TopTools_IndexedMapOfShape(),TopTools_IndexedMapOfShape()
        TopExp.MapShapes_s(shape,TopAbs_VERTEX,vertices)
        TopExp.MapShapes_s(shape,TopAbs_EDGE,curves)
        points=[BRep_Tool.Pnt_s(TopoDS.Vertex_s(vertices.FindKey(i))).Coord() for i in range(1,vertices.Extent()+1)]
        records=[]
        for i in range(1,curves.Extent()+1):
            edge=TopoDS.Edge_s(curves.FindKey(i))
            ends=[vertices.FindIndex(v) for v in (TopExp.FirstVertex_s(edge),TopExp.LastVertex_s(edge))]
            if any(v<=0 for v in ends):raise ValueError('edge endpoint missing from vertex inventory')
            records.append(dict(edge_index=i,vertices=ends))
        members.append(dict(member=item['member'],vertices=points,edges=records))
    output.write_text(json.dumps(dict(edges_file=str(edges),edges_sha256=digest(edges),members=members,
        scope='Actual vertex positions and edge incidence; endpoint geometry not checked'))+'\n')
    print(f"Extracted {sum(len(m['vertices']) for m in members)} vertices",flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('edges',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();run(args.edges,args.output)
