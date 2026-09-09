"""Bind the shared-edge inventory to both previously extracted face inventories."""
from collections import Counter
from fractions import Fraction as F
import json
from pathlib import Path

from check_face_domains import digest


def load(path):
    data = json.loads(path.read_text())
    files = {}
    for label in ('indexed','analytical'):
        p = Path(data[f'{label}_file'])
        if digest(p) != data[f'{label}_sha256']:
            raise ValueError('changed edge reference')
        files[label] = json.loads(p.read_text())
    indexed,analytical = files['indexed'],files['analytical']
    if analytical['indexed_sha256'] != data['indexed_sha256']:
        raise ValueError('face inventories use different indexed solids')
    if digest(Path(indexed['source_file'])) != indexed['source_sha256']:
        raise ValueError('changed source')
    for item in indexed['inputs']:
        if digest(Path(item['step_file'])) != item['step_sha256']:
            raise ValueError('changed indexed STEP')
    faces = {(r['member'],r['face_index']):dict(kind='bspline',**r) for r in indexed['surfaces']}
    if len(faces)!=len(indexed['surfaces']):
        raise ValueError('duplicate spline face inventory')
    for r in analytical['faces']:
        key = r['member'],r['face_index']
        if key in faces:
            raise ValueError('duplicate face inventory')
        faces[key] = r
    if len(data['members']) != 2 or {r['member'] for r in data['members']} != {0,1}:
        raise ValueError('incomplete member edges')
    expected,actual = {},{}
    def key(curve):return json.dumps(curve,sort_keys=True,separators=(',',':'))
    for pair,face in faces.items():
        if face['kind']=='bspline':
            curves = [dict(kind='line',**{k:v for k,v in c.items() if k!='reversed'}) for w in face['wires'] for c in w]
        else:
            curves = [c for w in face['wires'] for c in w]
        expected[pair] = Counter(map(key,curves))
        actual[pair] = Counter()
    for member in data['members']:
        indices = [r['edge_index'] for r in member['edges']]
        if set(indices) != set(range(1,len(indices)+1)) or len(set(indices)) != len(indices):
            raise ValueError('missing or duplicate shared edge')
        for edge in member['edges']:
            lo,hi = map(F,edge['curve']['interval'])
            if lo>=hi:raise ValueError('empty shared-edge interval')
            uses = edge['incidences']
            if len(uses) != 2 or {r['reversed'] for r in uses} != {False,True}:
                raise ValueError('shared edge lacks two opposite incidences')
            for use in uses:
                if edge['curve']['interval'] != use['curve']['interval']:
                    raise ValueError('3D and parameter-curve intervals disagree')
                pair = member['member'],use['face_index']
                if pair not in actual:
                    raise ValueError('edge uses an unknown face')
                actual[pair][key(use['curve'])] += 1
    if actual != expected:
        raise ValueError('edge incidences do not cover the complete face-wire inventory')
    return data,faces
