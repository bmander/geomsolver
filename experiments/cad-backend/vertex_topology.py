"""Closed face-wire incidence and single-cycle combinatorial vertex links."""
from collections import defaultdict
import json


def link_cycle(links):
    neighbors=defaultdict(list)
    for a,b in links:
        neighbors[a].append(b);neighbors[b].append(a)
    if not neighbors or any(len(v)!=2 for v in neighbors.values()):
        raise ValueError('vertex link is not two-regular')
    todo=[next(iter(neighbors))];seen=set()
    while todo:
        node=todo.pop()
        if node not in seen:
            seen.add(node);todo.extend(neighbors[node])
    if seen!=set(neighbors):raise ValueError('vertex has disconnected incident fans')
    return len(seen)


def check(vertices,edges,faces):
    key=lambda c:json.dumps(c,sort_keys=True,separators=(',',':'))
    lookups=defaultdict(dict)
    for member in edges['members']:
        for edge in member['edges']:
            for use in edge['incidences']:
                pair=member['member'],use['face_index'];signature=key(use['curve'])
                if signature in lookups[pair]:raise ValueError('ambiguous face-wire edge association')
                lookups[pair][signature]=edge['edge_index'],use['reversed']
    ends={(m['member'],e['edge_index']):e['vertices'] for m in vertices['members'] for e in m['edges']}
    fans=defaultdict(list);wire_count=0
    for pair,face in faces.items():
        member,_=pair
        for wire in face['wires']:
            if not wire:raise ValueError('empty face wire')
            wire_count+=1
            oriented=[]
            for curve in wire:
                normalized=(dict(kind='line',**{k:v for k,v in curve.items() if k!='reversed'})
                            if face['kind']=='bspline' else curve)
                edge,reversed=lookups[pair][key(normalized)]
                oriented.append((edge,int(reversed),1-int(reversed)))
            for incoming,outgoing in zip(oriented,oriented[1:]+oriented[:1]):
                vertex=ends[(member,incoming[0])][incoming[2]]
                if vertex!=ends[(member,outgoing[0])][outgoing[1]]:
                    raise ValueError('face wire does not close through its named vertices')
                fans[(member,vertex)].append(((incoming[0],incoming[2]),(outgoing[0],outgoing[1])))
    expected={(m['member'],i) for m in vertices['members'] for i in range(1,len(m['vertices'])+1)}
    if set(fans)!=expected:raise ValueError('incomplete vertex corner inventory')
    cycles=[dict(member=m,vertex_index=v,incident_half_edges=link_cycle(links)) for (m,v),links in sorted(fans.items())]
    return dict(closed_face_wires=wire_count,single_cycle_vertices=len(cycles),vertices=cycles,
                scope='Combinatorial wire and vertex-link consistency; geometric trim simplicity and embedding are separate')
