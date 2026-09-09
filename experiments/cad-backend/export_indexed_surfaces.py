"""Associate every indexed STEP B-spline with a rotated bounded tooth-space support."""
import argparse
import json
import math
from pathlib import Path

from OCP.BRep import BRep_Tool
from OCP.Geom import Geom_BSplineSurface
from OCP.TopAbs import TopAbs_FACE, TopAbs_WIRE
from OCP.TopoDS import TopoDS

from export_face_domains import items, wire_record
from export_supports import digest, extract
from material_probes import read


def rotated(point, index, teeth):
    angle = math.tau*index/teeth
    c,s = math.cos(angle),math.sin(angle)
    x,y,z = point
    return c*x-s*y,s*x+c*y,z


def run(source, spaces, reports, output):
    data = json.loads(source.read_text())
    bounded = json.loads((spaces/'report.json').read_text())
    if bounded['source_sha256'] != digest(source):
        raise ValueError('changed tooth-space source')
    records, inputs = [], []
    for member,report_file in zip(data['members'],reports):
        m,n = member['member'],int(member['teeth'])
        report = json.loads(report_file.read_text())
        row, = [r for r in report['results'] if r['member']==m and r['cuts']==n]
        tool = spaces/f'member{m}-space-16.step'
        checked, = [r for r in bounded['results'] if r['member']==m and r['subdivisions']==16]
        path = Path(row['step_file'])
        if (report['source_sha256'] != digest(source) or not row['passes_local_checks']
                or not checked['passes_local_checks'] or checked['step_sha256'] != digest(tool)
                or row['tool_sha256'] != digest(tool) or row['step_sha256'] != digest(path)):
            raise ValueError('changed or unchecked indexed inputs')
        refs = [extract(BRep_Tool.Surface_s(TopoDS.Face_s(f))) for f in items(read(tool),TopAbs_FACE)]
        if len(refs) != 10:
            raise ValueError('expected ten tooth-space supports')
        candidates = [(i,j,rotated(s['poles'][len(s['poles'])//2][len(s['poles'][0])//2],i,n)) for i in range(n) for j,s in enumerate(refs)]
        other = []
        for fi,raw in enumerate(items(read(path),TopAbs_FACE)):
            face = TopoDS.Face_s(raw)
            surface = BRep_Tool.Surface_s(face)
            if not isinstance(surface,Geom_BSplineSurface):
                other.append(dict(face_index=fi,kind=type(surface).__name__))
                continue
            net = extract(surface)
            matches = sorted((math.dist(net['poles'][len(net['poles'])//2][len(net['poles'][0])//2],p),i,j) for i,j,p in candidates)
            error,index,reference = matches[0]
            # Interior-pole association is only an adapter selection. The independent
            # checker compares every coefficient; no surface bound uses this sample.
            if error > 1e-7 or matches[1][0] < 1e-7:
                raise ValueError('unmatched or ambiguous indexed support')
            records.append(dict(member=m,face_index=fi,index=index,reference_face=reference,surface=net,
                                wires=[wire_record(TopoDS.Wire_s(w),face) for w in items(face,TopAbs_WIRE)]))
        inputs.append(dict(member=m,teeth=n,step_file=str(path),step_sha256=digest(path),
                           tool_file=str(tool),tool_sha256=digest(tool),references=refs,other_faces=other))
    output.write_text(json.dumps(dict(source_file=str(source),source_sha256=digest(source),inputs=inputs,
        surfaces=records,scope='Actual indexed support coefficients; independent rotation-transfer bounds pending'))+'\n')
    print(f'Extracted {len(records)} indexed B-spline supports',flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('source','spaces','pinion_report','gear_report','output'):
        parser.add_argument(name,type=Path)
    args = parser.parse_args()
    run(args.source,args.spaces,[args.pinion_report,args.gear_report],args.output)
