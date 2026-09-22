"""Controls for the experiment's mesh decoding and combined audit."""
import tempfile
import unittest
from pathlib import Path

import meshio
import numpy as np

from audit import audit, topology


class AuditControls(unittest.TestCase):
    def setUp(self):
        self.vertices = np.array([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])
        self.faces = np.array([[0,2,1],[0,1,3],[1,2,3],[2,0,3]])

    def test_native_and_encoded_tetrahedron(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'tetra.msh'
            meshio.write(path,meshio.Mesh(self.vertices,[('triangle',self.faces)]),file_format='gmsh22')
            report = audit(path)
            self.assertEqual(report['topology']['boundary_edges'],0)
            self.assertEqual(report['topology']['nonmanifold_vertex_links'],0)
            self.assertEqual(report['topology']['components'][0]['euler'],2)
            for encoding in ('binary64','binary32'):
                self.assertTrue(report[encoding]['pair_scan_complete'])
                self.assertFalse(report[encoding]['improper_pairs'])
                self.assertEqual(report[encoding]['degenerate_triangles'],0)
                self.assertAlmostEqual(report[encoding]['exact_signed_volume_as_float'],1/6)
            self.assertEqual(report['binary32']['encoded_topology']['genus'],0)

    def test_open_surface(self):
        report = topology(self.vertices,self.faces[:-1])
        self.assertEqual(report['boundary_edges'],3)
        self.assertGreater(report['nonmanifold_vertex_links'],0)

    def test_float32_collapse_is_distinct_from_native_validity(self):
        vertices = self.vertices.copy()
        vertices[:,0] *= 2**-30
        vertices[:,0] += 1.
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'thin.msh'
            meshio.write(path,meshio.Mesh(vertices,[('triangle',self.faces)]),file_format='gmsh22')
            report = audit(path)
            self.assertFalse(report['binary64']['improper_pairs'])
            self.assertEqual(report['binary64']['degenerate_triangles'],0)
            self.assertGreater(report['binary32']['degenerate_triangles'],0)
            self.assertEqual(report['binary32']['coordinate_aliases'],1)
            self.assertIn('topology_refusal',report['binary32'])


if __name__=='__main__': unittest.main()
