"""Finite-domain controls that preserve corner positions or bounding boxes."""
from copy import deepcopy
import unittest

from check_face_domains import rectangle


def square():
    # Non-unit line speeds and nonzero parameter origins must still give the
    # exact same boundary; endpoint geometry, not a particular encoding, matters.
    return [[dict(location=[-1,0],direction=[2,0],interval=[.5,1],reversed=False),
             dict(location=[1,0],direction=[0,1],interval=[0,1],reversed=False),
             dict(location=[0,1],direction=[1,0],interval=[0,1],reversed=True),
             dict(location=[0,0],direction=[0,1],interval=[0,1],reversed=True)]]


class FaceDomains(unittest.TestCase):
    def test_rectangle_and_reverse_orientation(self):
        wires = square()
        self.assertEqual(rectangle(wires)['domain'],[[0,1],[0,1]])
        wires[0].reverse()
        for edge in wires[0]:
            edge['reversed'] = not edge['reversed']
        self.assertEqual(rectangle(wires)['holes'],0)

    def test_split_side_covers_once_and_rejects_overlap(self):
        wires = square()
        edge = wires[0].pop(0)
        a,b = deepcopy(edge),deepcopy(edge)
        a['interval'][1] = b['interval'][0] = .75
        wires[0][0:0] = [a,b]
        self.assertEqual(rectangle(wires)['boundary_segments'],5)
        b['interval'][0] = .625
        with self.assertRaises(ValueError):
            rectangle(wires)

    def test_hole_gap_duplicate_and_disconnected_cycles_refused(self):
        cases = []
        wires = square();wires.append(deepcopy(wires[0]));cases.append(wires)
        wires = square();wires[0][0]['interval'][1] = .75;cases.append(wires)
        wires = square();wires[0][1] = deepcopy(wires[0][0]);cases.append(wires)
        wires = square();wires[0][1]['reversed'] = True;cases.append(wires)
        wires = square();wires[0][0],wires[0][1] = wires[0][1],wires[0][0];cases.append(wires)
        for wires in cases:
            with self.subTest(wires=wires), self.assertRaises(ValueError):
                rectangle(wires)

    def test_outside_edge_refused_even_with_tiny_displacement(self):
        wires = square()
        wires[0][2]['location'][1] += 1e-14
        with self.assertRaises(ValueError):
            rectangle(wires)


if __name__ == '__main__':
    unittest.main()
