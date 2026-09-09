"""Exact quarter-turn controls for the whole-support coefficient bound."""
from copy import deepcopy
from fractions import Fraction as F
import unittest

from check_indexed_surfaces import rotation_bound


def plane():
    return dict(degrees=[1,1],knots=[[0,0,1,1],[0,0,1,1]],weights=[[1,1],[1,1]],
                poles=[[[2,0,1],[2,1,1]],[[3,0,1],[3,1,1]]])


class IndexedSurfaces(unittest.TestCase):
    def test_four_exact_quarter_turns(self):
        reference = plane()
        transforms = [lambda x,y:(x,y),lambda x,y:(-y,x),lambda x,y:(-x,-y),lambda x,y:(y,-x)]
        for index,turn in enumerate(transforms):
            surface = deepcopy(reference)
            surface['poles'] = [[[*turn(x,y),z] for x,y,z in row] for row in reference['poles']]
            self.assertLess(rotation_bound(reference,surface,index,4),F('0.000000001'))

    def test_corner_distortion_is_bounded_and_exceeds_target(self):
        reference = plane()
        surface = deepcopy(reference)
        surface['poles'][0][0][2] += .1
        bound = rotation_bound(reference,surface,0,24)
        # At this clamped corner the actual surface moves by exactly the pole
        # difference; the support bound must contain that known geometric error.
        actual = abs(F(surface['poles'][0][0][2])-1)
        self.assertGreaterEqual(bound,actual)
        self.assertLess(bound,actual+F('0.000000001'))
        self.assertGreater(bound,F('0.00000001'))

    def test_basis_changes_and_invalid_indices_refused(self):
        reference = plane()
        for key in ('degrees','knots','weights'):
            surface = deepcopy(reference)
            if key == 'degrees':surface[key][0] = 2
            else:surface[key][0][0] += .1
            with self.assertRaises(ValueError):
                rotation_bound(reference,surface,0,24)
        with self.assertRaises(ValueError):
            rotation_bound(reference,reference,24,24)


if __name__ == '__main__':
    unittest.main()
