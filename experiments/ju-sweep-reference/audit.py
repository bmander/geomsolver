"""Independent indexed-mesh and exact-coordinate audits; never repairs geometry.

Triangle predicates are the existing independent integer/Fraction verifier.
This experiment checks both original binary64 and float32-encoded coordinates.
Accuracy checks are sampled comparisons, not whole-surface certificates.
"""
import collections
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
import time

import meshio
import numpy as np
from scipy.spatial import cKDTree

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "rust/gcs-core/tests/verification"))
import stl_embedding as exact


def topology(vertices, faces):
    edges = collections.defaultdict(list)
    links = collections.defaultdict(lambda: collections.defaultdict(list))
    parent = list(range(len(vertices)))
    used = set(map(int, faces.flat))
    def root(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    for face, (a, b, c) in enumerate(faces.tolist()):
        for x, y, z in [(a,b,c),(b,c,a),(c,a,b)]:
            edges[min(x,y),max(x,y)].append((x,y,face))
            parent[root(x)] = root(y)
            links[x][y].append(z)
            links[x][z].append(y)
    bad_links = 0
    for link in links.values():
        pending = [next(iter(link))]; visited = set()
        while pending:
            a = pending.pop()
            if a not in visited:
                visited.add(a); pending.extend(link[a])
        bad_links += len(visited) != len(link) or any(len(v) != 2 for v in link.values())
    groups = collections.defaultdict(lambda: [0,0,0,0.])
    for v in used: groups[root(v)][0] += 1
    for a,b in edges: groups[root(a)][1] += 1
    volumes = collections.defaultdict(list)
    for a,b,c in faces.tolist():
        groups[root(a)][2] += 1
        volumes[root(a)].append(float(np.dot(vertices[a],np.cross(vertices[b],vertices[c])))/6.)
    components = []
    for key,(v,e,f,_) in groups.items():
        components.append(dict(vertices=v,edges=e,faces=f,euler=v-e+f,signed_volume=math.fsum(volumes[key])))
    return dict(vertices=len(used), unused_vertices=len(vertices)-len(used), edges=len(edges),
        triangles=len(faces), boundary_edges=sum(len(v)==1 for v in edges.values()),
        nonmanifold_edges=sum(len(v)>2 for v in edges.values()),
        inconsistent_edges=sum(len(v)==2 and v[0][:2] != v[1][1::-1] for v in edges.values()),
        nonmanifold_vertex_links=int(bad_links), components=components,
        signed_volume=math.fsum(v['signed_volume'] for v in components))


def embedding(vertices, faces, seconds=120):
    start = time.monotonic()
    values = vertices.tolist()
    ratios = {x:x.as_integer_ratio() for row in values for x in row}
    exponent = max(d.bit_length()-1 for _,d in ratios.values())
    integer = {x:n << (exponent-(d.bit_length()-1)) for x,(n,d) in ratios.items()}
    points = [tuple(integer[x] for x in row) for row in values]
    triangles = [tuple(points[i] for i in f) for f in faces.tolist()]
    degenerate = [i for i,(a,b,c) in enumerate(triangles) if not any(exact.cross(exact.sub(b,a),exact.sub(c,a)))]
    used = set(map(int,faces.flat))
    aliases = len(used)-len({points[i] for i in used})
    failures = []; tested = 0; complete = True
    degenerate_set = set(degenerate)
    good = [i for i in range(len(triangles)) if i not in degenerate_set]
    usable = [triangles[i] for i in good]
    for a,b in exact.candidate_pairs(usable):
        tested += 1
        if exact.improper_intersection(usable[a],usable[b]):
            failures.append([good[a],good[b]])
        if len(failures) >= 16 or time.monotonic()-start > seconds:
            complete = False; break
    numerator = sum(exact.dot(a,exact.cross(b,c)) for a,b,c in triangles)
    return dict(degenerate_triangles=len(degenerate), first_degenerate=degenerate[:16],
        coordinate_aliases=aliases, improper_pairs=failures, tested_pairs=tested,
        pair_scan_complete=complete, integer_scale_exponent=exponent,
        exact_signed_volume_as_float=float(numerator/(6*(1 << exponent)**3)),
        seconds=time.monotonic()-start,
        scope='Exact predicates on represented coordinates; no source-accuracy or topology-equivalence claim')


def capsule_distance(points, a, b, radius):
    ab = b-a
    t = np.clip((points-a)@ab/(ab@ab),0.,1.)
    return np.linalg.norm(points-a-t[:,None]*ab,axis=1)-radius


def sampled_capsule(vertices, faces, capsule):
    a,b,radius = np.array(capsule['a']),np.array(capsule['b']),capsule['radius']
    tri = vertices[faces]
    samples = np.concatenate([vertices,tri.mean(axis=1),0.5*(tri[:,0]+tri[:,1]),
                              0.5*(tri[:,1]+tri[:,2]),0.5*(tri[:,2]+tri[:,0])])
    forward = np.abs(capsule_distance(samples,a,b,radius))
    # Uniform cylindrical samples and hemispherical samples of the exact capsule.
    axis = (b-a)/np.linalg.norm(b-a)
    helper = np.eye(3)[np.argmin(np.abs(axis))]
    u = np.cross(axis,helper); u /= np.linalg.norm(u); v = np.cross(axis,u)
    angles = np.arange(96)*2*math.pi/96
    radial = np.cos(angles)[:,None]*u+np.sin(angles)[:,None]*v
    points = [a+t*(b-a)+radius*radial for t in np.linspace(0,1,25)]
    for centre,sign in [(a,-1),(b,1)]:
        for z in np.linspace(0,1,17):
            points.append(centre+radius*(sign*z*axis+math.sqrt(1-z*z)*radial))
    points = np.concatenate(points)
    # Distance to the closest among nearby candidate triangles is an upper bound
    # at each sample, not a proof of closest-triangle discovery or continuous coverage.
    tree = cKDTree(tri.mean(axis=1)); maximum = 0.
    for first in range(0,len(points),128):
        p = points[first:first+128]
        _,indices = tree.query(p,k=min(32,len(tri)))
        if indices.ndim == 1: indices = indices[:,None]
        t = tri[indices]; q = p[:,None,:]
        x,y,z = t[:,:,0],t[:,:,1],t[:,:,2]
        xy,xz,xq = y-x,z-x,q-x
        dot = lambda p,q: np.sum(p*q,axis=-1)
        aa,ab,bb = dot(xy,xy),dot(xy,xz),dot(xz,xz)
        denominator = aa*bb-ab*ab
        with np.errstate(invalid='ignore',divide='ignore'):
            s = (bb*dot(xq,xy)-ab*dot(xq,xz))/denominator
            t0 = (aa*dot(xq,xz)-ab*dot(xq,xy))/denominator
            normal = np.cross(xy,xz)
            distance = dot(xq,normal)**2/dot(normal,normal)
        distance = np.where((s>=0)&(t0>=0)&(s+t0<=1),distance,np.inf)
        for e,f in [(x,y),(y,z),(z,x)]:
            direction = f-e
            with np.errstate(invalid='ignore',divide='ignore'):
                fraction = np.clip(dot(q-e,direction)/dot(direction,direction),0.,1.)
            residual = q-e-fraction[:,:,None]*direction
            distance = np.minimum(distance,dot(residual,residual))
        maximum = max(maximum,float(np.sqrt(np.min(distance,axis=1)).max()))
    exact_volume = math.pi*radius**2*np.linalg.norm(b-a)+4*math.pi*radius**3/3
    return dict(mesh_samples=len(samples),max_mesh_to_capsule=float(forward.max()),
        capsule_samples=len(points),max_sampled_capsule_to_mesh_upper=maximum,
        analytic_volume=exact_volume,scope='Sampled distances only; not a whole-surface certificate')


def audit(path, capsule=None, seconds=120):
    path = Path(path); mesh = meshio.read(path)
    vertices = np.asarray(mesh.points[:,:3],dtype=np.float64)
    blocks = [c.data for c in mesh.cells if c.type=='triangle']
    faces = np.concatenate(blocks).astype(np.int64)
    if not np.isfinite(vertices).all(): raise ValueError('Nonfinite vertices')
    if faces.min()<0 or faces.max()>=len(vertices): raise ValueError('Invalid index')
    report = dict(file=str(path),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        topology=topology(vertices,faces),binary64=embedding(vertices,faces,seconds))
    encoded = vertices.astype(np.float32).astype(np.float64)
    stl = bytearray(80)+struct.pack('<I',len(faces))
    for face in faces:
        stl += bytes(12)+struct.pack('<9f',*encoded[face].flat)+bytes(2)
    path.with_suffix('.stl').write_bytes(stl)
    # Analyze coordinates decoded from the actual written bytes, preserving
    # original indexed incidence separately from any aliases created by encoding.
    decoded = np.array([struct.unpack_from('<9f',stl,96+50*i) for i in range(len(faces))]).reshape(-1,3)
    report['binary32'] = embedding(decoded,np.arange(len(decoded)).reshape(-1,3),seconds)
    report['binary32']['coordinate_aliases'] = len(set(map(int,faces.flat)))-len({tuple(encoded[i]) for i in set(map(int,faces.flat))})
    try: report['binary32']['encoded_topology'] = exact.stl_topology.check(stl)
    except ValueError as error: report['binary32']['topology_refusal'] = str(error)
    if capsule: report['capsule'] = sampled_capsule(vertices,faces,capsule)
    np.savez_compressed(path.with_suffix('.npz'),vertices=vertices,faces=faces)
    path.with_suffix('.audit.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


if __name__=='__main__':
    print(json.dumps(audit(sys.argv[1]),indent=2))
