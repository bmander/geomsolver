"""Render saved original meshes; does not simplify or repair them."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('results',type=Path)
    parser.add_argument('output',type=Path)
    parser.add_argument('--case',action='append')
    args = parser.parse_args()
    names = args.case or ['simple-stock','letter_L-stock','flipping_torus-stock']
    fig = plt.figure(figsize=(5*len(names),5),layout='constrained')
    for i,name in enumerate(names):
        ax = fig.add_subplot(1,len(names),i+1,projection='3d')
        folder = args.results/name
        mesh = np.load(folder/'mesh/sweep_surface.npz')
        v,f = mesh['vertices'],mesh['faces']
        collection = Poly3DCollection(v[f],facecolors='#67aac5',edgecolors='#294957',
                                     linewidths=.08,alpha=1.,shade=True,
                                     lightsource=matplotlib.colors.LightSource(azdeg=315,altdeg=45))
        ax.add_collection3d(collection)
        centre = (v.max(axis=0)+v.min(axis=0))*.5
        half = (v.max(axis=0)-v.min(axis=0)).max()*.55
        ax.set_xlim(centre[0]-half,centre[0]+half)
        ax.set_ylim(centre[1]-half,centre[1]+half)
        ax.set_zlim(centre[2]-half,centre[2]+half)
        ax.set_box_aspect((1,1,1)); ax.view_init(elev=23,azim=-65)
        ax.set_axis_off()
        result = json.loads((folder/'result.json').read_text())
        ax.set_title(f"{name}\n{len(f):,} triangles · {result['run_seconds']:.2f} s",fontsize=12)
    fig.savefig(args.output,dpi=180)


if __name__=='__main__': main()
