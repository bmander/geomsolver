"""Cross-check meshio geometry against the reference's one-block binary MSH.

Intentionally refuses other layouts; this is not a general Gmsh reader. It does
not read attributes and does not change geometry or indices.
"""
import json
from pathlib import Path
import struct
import sys

import meshio
import numpy as np


def check(path):
    path=Path(path); data=path.read_bytes()
    assert data.startswith(b'$MeshFormat\n4.1 1 8\n\x01\x00\x00\x00')
    i=data.index(b'$Nodes\n')+len(b'$Nodes\n')
    blocks,n,lo,hi=struct.unpack_from('<4Q',data,i); i+=32
    assert blocks==1 and lo==1 and hi==n
    dimension,entity,param,count=struct.unpack_from('<3iQ',data,i); i+=20
    assert param==0 and count==n
    tags=np.frombuffer(data,dtype='<u8',count=n,offset=i); i+=8*n
    assert np.array_equal(tags,np.arange(1,n+1))
    vertices=np.frombuffer(data,dtype='<f8',count=3*n,offset=i).reshape(-1,3)
    i+=24*n
    assert data[i:].lstrip(b'\n').startswith(b'$EndNodes\n$Elements\n')
    i=data.index(b'$Elements\n',i)+len(b'$Elements\n')
    blocks,nfaces,lo,hi=struct.unpack_from('<4Q',data,i); i+=32
    assert blocks==1 and lo==1 and hi==nfaces
    dimension,entity,kind,count=struct.unpack_from('<3iQ',data,i); i+=20
    assert dimension==2 and kind==2 and count==nfaces
    elements=np.frombuffer(data,dtype='<u8',count=4*nfaces,offset=i).reshape(-1,4)
    assert np.array_equal(elements[:,0],np.arange(1,nfaces+1))
    faces=elements[:,1:]-1
    mesh=meshio.read(path)
    assert np.array_equal(vertices,mesh.points)
    assert np.array_equal(faces,mesh.cells_dict['triangle'])
    return dict(file=str(path),nodes=n,triangles=nfaces,coordinates_match=True,indices_match=True)


if __name__=='__main__':
    results=[check(path) for path in sys.argv[1:]]
    print(json.dumps(results,indent=2))
