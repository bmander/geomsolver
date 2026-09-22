"""Export inspection pairs: analytic tool at t=0 and unchanged sweep triangles.

Only the three selected reference examples are supported. Tool STL is our
parametric tessellation of the YAML primitive, not an upstream sweep result.
"""
import argparse
import json
from pathlib import Path
import shutil
import struct

import numpy as np
import yaml

from audit import topology


def sphere(radius, around=96, rings=48):
    vertices = [[0.,0.,radius]]
    for j in range(1,rings):
        theta = np.pi*j/rings
        for i in range(around):
            phi = 2*np.pi*i/around
            vertices.append([radius*np.sin(theta)*np.cos(phi),radius*np.sin(theta)*np.sin(phi),radius*np.cos(theta)])
    south = len(vertices); vertices.append([0.,0.,-radius])
    faces = []
    for i in range(around):
        following = (i+1)%around
        faces.append([0,1+i,1+following])
        faces.append([south,1+(rings-2)*around+following,1+(rings-2)*around+i])
    for j in range(rings-2):
        for i in range(around):
            a=1+j*around+i; b=1+j*around+(i+1)%around
            faces.extend([[a,a+around,b],[b,a+around,b+around]])
    v,f=np.array(vertices),np.array(faces)
    normal=np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]])
    inward=(normal*v[f].mean(axis=1)).sum(axis=1)<0
    f[inward]=f[inward,::-1]
    return v,f


def torus(major,minor,around=96,tube=48):
    vertices=[]; faces=[]
    for i in range(around):
        u=2*np.pi*i/around
        for j in range(tube):
            v=2*np.pi*j/tube
            vertices.append([minor*np.sin(v),(major+minor*np.cos(v))*np.cos(u),(major+minor*np.cos(v))*np.sin(u)])
    for i in range(around):
        for j in range(tube):
            a=i*tube+j; b=((i+1)%around)*tube+j
            c=i*tube+(j+1)%tube; d=((i+1)%around)*tube+(j+1)%tube
            faces.extend([[a,b,c],[c,b,d]])
    v,f=np.array(vertices),np.array(faces)
    centres=v[f].mean(axis=1)
    radial=centres.copy()
    radial[:,1:] *= (1-major/np.linalg.norm(centres[:,1:],axis=1))[:,None]
    normal=np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]])
    inward=(normal*radial).sum(axis=1)<0
    f[inward]=f[inward,::-1]
    return v,f


def write_stl(path,vertices,faces):
    data=bytearray(80)+struct.pack('<I',len(faces))
    for tri in vertices[faces]:
        normal=np.cross(tri[1]-tri[0],tri[2]-tri[0]); normal/=np.linalg.norm(normal)
        data+=struct.pack('<12fH',*normal,*tri.flat,0)
    path.write_bytes(data)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--results',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--case',action='append',required=True,
                        choices=['simple-stock','letter_L-stock','flipping_torus-stock'])
    args=parser.parse_args()
    for name in args.case:
        example=name.removesuffix('-stock')
        definition=yaml.safe_load((args.source/'example'/example/'sweep.yaml').read_text())
        primitive,transform=definition['primitive'],definition['transform']
        if transform['type']=='compose': transform=transform['transforms'][0]
        position=np.array(transform.get('points',transform.get('control_points'))[0])
        if primitive['type']=='ball': v,f=sphere(primitive['radius'])
        else:
            assert primitive['type']=='torus' and primitive['normal']==[1.,0.,0.]
            v,f=torus(primitive['major_radius'],primitive['minor_radius'])
        v+=position+np.array(primitive['center'])
        folder=args.output/name; folder.mkdir(parents=True,exist_ok=True)
        write_stl(folder/'tool-at-start.stl',v,f)
        shutil.copyfile(args.results/name/'mesh/sweep_surface.stl',folder/'swept-solid.stl')
        report=topology(v.astype(np.float32).astype(np.float64),f)
        (folder/'tool.json').write_text(json.dumps(dict(primitive=primitive,start_position=position.tolist(),
            provenance='Parametric tessellation of reference YAML primitive at t=0',topology=report),indent=2)+'\n')
        print(folder)


if __name__=='__main__': main()
