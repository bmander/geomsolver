"""Sample a prism mesh against independently evaluated continuous-motion input.

This tests the min-over-time input field, not whole-surface distance or topology.
The temporal sampling remainder uses the maximum rigid-body point speed; ordinary
floating-point evaluation is not interval-certified. It is not a repair step.
"""
import argparse
import json
from pathlib import Path

import numpy as np


def check(path,angle_degrees=360.,sample_count=4096,time_intervals=2048):
    mesh=np.load(path);v,f=mesh['vertices'],mesh['faces']
    vi=np.linspace(0,len(v)-1,min(len(v),sample_count),dtype=int)
    fi=np.linspace(0,len(f)-1,min(len(f),sample_count),dtype=int)
    points=np.concatenate([v[vi],v[f[fi]].mean(axis=1)])
    times=np.linspace(0,1,time_intervals+1)
    angle=np.deg2rad(angle_degrees);c,s=np.cos(times*angle),np.sin(times*angle)
    centre=np.array([.14,.51,.5])+times[:,None]*np.array([.72,0.,.01])
    half=np.array([.15,.09,.06]); values=[]; minimizing_times=[]
    for i in range(0,len(points),64):
        relative=points[i:i+64,None,:]-centre[None,:,:]
        local=np.empty_like(relative)
        local[:,:,0]=c*relative[:,:,0]+s*relative[:,:,1]
        local[:,:,1]=-s*relative[:,:,0]+c*relative[:,:,1]
        local[:,:,2]=relative[:,:,2]
        d=np.abs(local)-half
        sdf=np.linalg.norm(np.maximum(d,0.),axis=2)+np.minimum(d.max(axis=2),0.)
        values.extend(sdf.min(axis=1).tolist())
        minimizing_times.extend(times[sdf.argmin(axis=1)].tolist())
    values=np.array(values)
    speed=np.linalg.norm([.72,0.,.01])+abs(angle)*np.linalg.norm(half[:2])
    remainder=speed/(2*time_intervals)
    witnesses=[]
    for i in np.argsort(values)[:8]:
        item=dict(point=points[i].tolist(),field_value=float(values[i]),time=minimizing_times[i])
        if i<len(vi): item.update(kind='vertex',index=int(vi[i]))
        else:
            index=int(fi[i-len(vi)]); triangle=v[f[index]]
            item.update(kind='triangle centroid',index=index,
                        triangle_area=float(np.linalg.norm(np.cross(triangle[1]-triangle[0],triangle[2]-triangle[0]))*.5))
        witnesses.append(item)
    return dict(file=str(path),surface_samples=len(points),time_samples=len(times),worst_inside_samples=witnesses,
        maximum_tool_point_speed_bound=float(speed),temporal_sampling_remainder=float(remainder),
        sampled_min_field_min=float(values.min()),sampled_min_field_max=float(values.max()),
        min_field_interval_over_samples=[float(values.min()-remainder),float(values.max())],
        samples_with_field_below_minus_002=int((values<-.002).sum()),
        samples_with_field_above_002_after_remainder=int((values-remainder>.002).sum()),
        scope='Sampled original float64 vertices and centroids; min-over-time box field, not whole-surface or repaired-mesh accuracy')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mesh',type=Path)
    parser.add_argument('--angle',type=float,default=360.)
    args=parser.parse_args();report=check(args.mesh,args.angle)
    args.mesh.with_suffix('.field-check.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
