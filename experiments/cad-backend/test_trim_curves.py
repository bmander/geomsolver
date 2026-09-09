from fractions import Fraction as F
import unittest

from trim_curves import restrict, pieces, monotone_axis, separated


class TrimCurves(unittest.TestCase):
    def test_polynomial_extension_and_restriction(self):
        # Bezier representation of (t,t^2); endpoints outside the base domain
        # must be evaluated, not silently clamped to the original end poles.
        poles = [(F(0),F(0)),(F(1,2),F(0)),(F(1),F(1))]
        for lo,hi in [(-2,0),(-1,2),(1,3),(F(1,4),F(3,4))]:
            q = restrict(poles,F(lo),F(hi))
            self.assertEqual(q[0],(lo,lo*lo))
            self.assertEqual(q[-1],(hi,hi*hi))
            self.assertEqual(q[1][1],lo*hi)

    def test_finite_multispan_and_extensions(self):
        curve = dict(kind='bspline',degree=1,poles=[[0,0],[1,1],[2,0]],
                     weights=[1,1,1],knots=[0,0,1,2,2],interval=[-.5,2.5])
        p = pieces(curve)
        self.assertEqual(p, [[(F(-1,2),F(-1,2)),(1,1)],[(1,1),(F(5,2),F(-1,2))]])
        self.assertEqual(monotone_axis(p),dict(axis=0,sign=1))
        curve['interval'] = [3,4]
        self.assertEqual(pieces(curve),[[(3,-1),(4,-2)]])
        curve['interval'] = [-2,-1]
        self.assertEqual(pieces(curve),[[(-2,-2),(-1,-1)]])

    def test_monotonicity_is_global_and_strict(self):
        self.assertEqual(monotone_axis([[(0,0),(0,0),(1,1)]]),dict(axis=0,sign=1))
        self.assertIsNone(monotone_axis([[(0,0),(0,0)]]))
        self.assertIsNone(monotone_axis([[(0,0),(1,1)],[(1,1),(0,0)]]))
        # A closed cubic loop cannot obtain an individual-curve certificate.
        self.assertIsNone(monotone_axis([[(0,0),(1,1),(-1,1),(0,0)]]))
        with self.assertRaises(ValueError):
            monotone_axis([[(0,0),(1,1)],[(-2,-2),(-1,-1)]])

    def test_unsupported_rational_curve_refused(self):
        curve = dict(kind='bspline',degree=1,poles=[[0,0],[1,1]],weights=[1,2],
                     knots=[0,0,1,1],interval=[0,1])
        with self.assertRaises(ValueError):pieces(curve)

    def test_separation_requires_complete_cover(self):
        a = [[(F(0),F(0)),(F(1,2),F(1)),(F(1),F(0))]]
        b = [[(F(0),F(3,4)),(F(1),F(3,4))]]
        self.assertFalse(separated(a,b,max_cells=1)[0])
        self.assertTrue(separated(a,b,max_cells=32)[0])
        # Interior crossing and exact contact cannot be accepted as separation.
        for y in (F(1,4),F(1,2)):
            self.assertFalse(separated(a,[[(F(0),y),(F(1),y)]],max_cells=32)[0])
        with self.assertRaises(ValueError):separated(a,b,max_cells=0)
