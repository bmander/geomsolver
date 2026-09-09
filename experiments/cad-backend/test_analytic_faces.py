"""Known geometry and polynomial extensions for the analytical-face support audit."""
from copy import deepcopy
from fractions import Fraction as F
import unittest

from check_analytic_faces import cone_bound, sphere_bound
from interval_jet import f, point
from parameter_bounds import curve_bounds


class ParameterBounds(unittest.TestCase):
    def test_cubic_extensions_are_included_without_clamping(self):
        # x=t, y=(2t-1)^3 has y(-1/4)=-27/8 and y(5/4)=27/8.
        curve = dict(kind='bspline',degree=3,knots=[0,0,0,0,1,1,1,1],weights=[1]*4,
                     poles=[[F(i,3),y] for i,y in enumerate((-1,1,-1,1))],interval=[-F(1,4),F(5,4)])
        bounds,extensions = curve_bounds(curve)
        self.assertEqual(extensions,2)
        for (lo,hi),a,b in zip(bounds,(-F(1,4),-F(27,8)),(F(5,4),F(27,8))):
            self.assertLessEqual(lo,a);self.assertGreaterEqual(hi,b)
        self.assertLess(bounds[1][0],-1)
        self.assertGreater(bounds[1][1],1)

    def test_piecewise_linear_extensions_use_end_spans(self):
        curve = dict(kind='bspline',degree=1,knots=[0,0,1,2,2],weights=[1]*3,
                     poles=[[0,0],[1,2],[2,1]],interval=[-1,3])
        bounds,n = curve_bounds(curve)
        self.assertEqual(n,2)
        for actual,expected in zip(bounds,((-1,3),(-2,2))):
            self.assertLessEqual(actual[0],expected[0])
            self.assertGreaterEqual(actual[1],expected[1])

    def test_rational_and_invalid_splines_are_refused(self):
        curve = dict(kind='bspline',degree=1,knots=[0,0,1,1],weights=[1,1],poles=[[0,0],[1,1]],interval=[0,1])
        for key,value in [('weights',[1,2]),('knots',[0,1,0,1]),('interval',[1,0])]:
            wrong=deepcopy(curve);wrong[key]=value
            with self.assertRaises(ValueError):curve_bounds(wrong)


class AnalyticalSupports(unittest.TestCase):
    def test_displaced_sphere_attains_the_bound(self):
        record = dict(origin=[3,4,0],radius=4,basis=[[1,0,0],[0,1,0],[0,0,1]])
        # The point center + 4*center/5 has radius 9 about the origin;
        # its distance to the unit sphere is exactly 8.
        bound = sphere_bound(record,1)
        self.assertGreaterEqual(bound,8)
        self.assertLess(bound-8,F(1,10**40))
        record['basis'][0][0]=2
        with self.assertRaises(ValueError):sphere_bound(record,1)

    def test_cone_angle_and_offset_against_exact_meridian_distance(self):
        meridian=[[1,0,1],[2,0,2]]
        record=dict(origin=[0,0,2],radius=2,angle=.7853981633974483,basis=[[1,0,0],[0,1,0],[0,0,1]])
        self.assertLess(cone_bound(record,meridian,(-F(1),F(5))),F('0.000000000001'))
        record['origin'][2]=2.01
        bound=cone_bound(record,meridian,(-F(1),F(5)))
        # At v=0 the known rz point is (2,2.01), so its exact distance
        # to R=Z supplies a lower bound that this whole-support bound must cover.
        squared=(F(2.01)-2)**2/2
        self.assertGreaterEqual(bound,f.arithmetic.sqrt(point(squared))[0])
        self.assertLess(bound,F('.007072'))
        self.assertGreater(bound,F('.001'))
        record['origin'][0]=.1
        with self.assertRaises(ValueError):cone_bound(record,meridian,(-F(1),F(5)))


if __name__ == '__main__':
    unittest.main()
