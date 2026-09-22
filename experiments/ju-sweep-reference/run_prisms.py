"""Exercise the unmodified reference CLI's OBJ input with sharp prism tools.

The CLI's OBJ path supplies its own motion: (0.14,0.51,0.5) to
(0.86,0.51,0.51), and -r 2 means a full turn about z (angle = r*pi*t).
Subdivision adds coplanar triangles; it does not round the tool's edges.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time

import numpy as np
import yaml

from audit import audit, exact, topology
from export_stls import write_stl
from run import COMMIT


def prism(triangular=False,subdivisions=3):
    if triangular:
        yz=[[-.09,-.04],[.09,-.04],[0.,.08]]
        vertices=[[x,y,z] for x in [-.15,.15] for y,z in yz]
        faces=[[0,1,2],[3,5,4],[0,3,1],[1,3,4],[1,4,2],[2,4,5],[2,5,0],[0,5,3]]
    else:
        vertices=[[x,y,z] for x in [-.15,.15] for y in [-.09,.09] for z in [-.06,.06]]
        quads=[[0,1,3,2],[4,6,7,5],[0,4,5,1],[2,3,7,6],[0,2,6,4],[1,5,7,3]]
        faces=[f for a,b,c,d in quads for f in [[a,b,c],[a,c,d]]]
    v,f=np.array(vertices),np.array(faces)
    normals=np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]])
    inward=(normals*v[f].mean(axis=1)).sum(axis=1)<0
    f[inward]=f[inward,::-1]
    for _ in range(subdivisions):
        vertices=v.tolist(); edges={}; faces=[]
        def midpoint(a,b):
            key=tuple(sorted((a,b)))
            if key not in edges:
                edges[key]=len(vertices);vertices.append(((v[a]+v[b])*.5).tolist())
            return edges[key]
        for a,b,c in f.tolist():
            ab,bc,ca=midpoint(a,b),midpoint(b,c),midpoint(c,a)
            faces.extend([[a,ab,ca],[ab,b,bc],[ca,bc,c],[ab,bc,ca]])
        v,f=np.array(vertices),np.array(faces)
    return v,f


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--case',action='append',choices=['prism-translate','prism-tumble','triangular-prism-tumble'])
    parser.add_argument('--epsilon',type=float,default=.002)
    parser.add_argument('--timeout',type=float,default=300)
    parser.add_argument('--audit-seconds',type=float,default=15)
    parser.add_argument('--analytic',action='store_true',help='Use the exact box-SDF API adapter instead of the CLI OBJ adapter')
    args=parser.parse_args();source=args.source.resolve();output=args.output.resolve()
    assert subprocess.check_output(['git','-C',str(source),'rev-parse','HEAD'],text=True).strip()==COMMIT
    binary=source/('build/ju_prism_probe' if args.analytic else 'build/generalized_sweep')
    output.mkdir(parents=True,exist_ok=True)
    selected=args.case or (['prism-tumble','prism-translate'] if args.analytic else ['prism-translate','prism-tumble','triangular-prism-tumble'])
    if args.analytic and 'triangular-prism-tumble' in selected:
        raise SystemExit('The analytic adapter currently implements rectangular prisms only')
    for name in selected:
        case=('analytic-' if args.analytic else '')+name
        folder=output/case
        if folder.exists(): print('SKIP',folder,flush=True);continue
        folder.mkdir(); v,f=prism(name.startswith('triangular'))
        obj=folder/'tool.obj'
        obj.write_text(''.join('v '+' '.join(format(x,'.17g') for x in row)+'\n' for row in v)+
                       ''.join('f '+' '.join(str(int(x)+1) for x in row)+'\n' for row in f))
        write_stl(folder/'tool-at-start.stl',v+np.array([.14,.51,.5]),f)
        tool_check=exact.check((folder/'tool-at-start.stl').read_bytes())
        config=yaml.safe_load((source/'example/flipping_torus/config.yaml').read_text())
        config['parameters']['epsilon_env']=args.epsilon
        (folder/'config.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
        rotations=0 if name=='prism-translate' else 2
        command=[str(binary),str(folder/'mesh'),'-f',str(obj),'-c',str(folder/'config.yaml'),'-r',str(rotations)]
        if args.analytic: command=[str(binary),str(folder/'mesh'),str(rotations*180),str(args.epsilon)]
        result=dict(case=case,commit=COMMIT,command=command,cwd=str(folder),
            binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),
            tool_sha256=hashlib.sha256(obj.read_bytes()).hexdigest(),tool_audit=tool_check,
            tool_topology=topology(v,f),generation_timeout=args.timeout,audit_seconds=args.audit_seconds,
            motion=dict(start=[.14,.51,.5],end=[.86,.51,.51],axis=[0,0,1],angle_degrees=rotations*180),
            input_representation='Sharp, subdivided planar OBJ faces; reference mesh distance and gradient callback')
        if args.analytic:
            result['input_representation']='Exact sharp box signed-distance callback through upstream API'
            result['adapter_sha256']=hashlib.sha256(Path(__file__).with_name('prism_probe.cpp').read_bytes()).hexdigest()
            result['max_split']=200000
        print('START',case,flush=True);start=time.monotonic()
        with (folder/'run.log').open('w') as log:
            try:
                run=subprocess.run(command,cwd=folder,stdout=log,stderr=subprocess.STDOUT,timeout=args.timeout)
                result.update(status='ok' if run.returncode==0 else 'failed',returncode=run.returncode)
            except subprocess.TimeoutExpired:result['status']='timeout'
        result['run_seconds']=time.monotonic()-start
        (folder/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        if result['status']=='ok':
            try:result['audit']=audit(folder/'mesh/sweep_surface.msh',seconds=args.audit_seconds)
            except Exception as error:result['audit_error']=repr(error)
        (folder/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print('DONE',case,result['status'],result['run_seconds'],flush=True)


if __name__=='__main__':main()
