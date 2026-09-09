import unittest
from fractions import Fraction as F

from spline_embedding import check, positive_symmetric, jacobian_bounds


def surface(poles,degrees=(1,1),knots=None):
    if knots is None:knots=[[0]*(d+1)+[1]*(d+1) for d in degrees]
    return dict(poles=poles,degrees=list(degrees),knots=knots,
                weights=[[1 for p in row] for row in poles])


class SplineEmbedding(unittest.TestCase):
    def test_saddle_is_an_embedded_graph(self):
        s = surface([[[0,0,0],[0,1,0]],[[1,0,0],[1,1,2]]])
        self.assertTrue(check(s)['verified'])
        self.assertTrue(check(s)['regular_everywhere'])

    def test_cusped_surface_is_not_accepted(self):
        # S(u,v)=((u-.5)^2, v, (u-.5)^3), a projection fold. Its
        # derivative also vanishes at the cusp; a positive test cannot ignore it.
        x = [F(1,4),F(-1,12),F(-1,12),F(1,4)]
        z = [F(-1,8),F(1,8),F(-1,8),F(1,8)]
        s = surface([[[a,0,b],[a,1,b]] for a,b in zip(x,z)],degrees=(3,1))
        self.assertFalse(check(s)['verified'])

    def test_regular_self_intersection_is_not_accepted(self):
        # (t^2-1,v,t^3-t), t=3u-1.5. Distinct interior parameters
        # t=-1 and t=1 agree. Its tangent never vanishes: the squared
        # norm is 9*(t^2-1/9)^2+8/9. Local regularity is insufficient.
        x = [F(5,4),F(-7,4),F(-7,4),F(5,4)]
        z = [F(-15,8),F(31,8),F(-31,8),F(15,8)]
        s = surface([[[a,0,b],[a,1,b]] for a,b in zip(x,z)],degrees=(3,1))
        self.assertFalse(check(s)['verified'])

    def test_one_global_output_scaling(self):
        # (u + .01*v^2, v + 3*u^2, 0) needs balanced projection rows.
        grid = [[[u+v2,v+u2,0] for v,v2 in zip((0,F(1,2),1),(0,0,F(1,100)))]
                for u,u2 in zip((0,F(1,2),1),(0,0,3))]
        result = check(surface(grid,degrees=(2,2)))
        self.assertTrue(result['verified'])
        self.assertLess(result['second_output_scale'],1)

    def test_symmetric_part_requires_off_diagonal_control(self):
        self.assertFalse(positive_symmetric([[(F(1),F(1)),(F(3),F(3))],
                                             [(F(3),F(3)),(F(1),F(1))]])['verified'])

    def test_physical_knot_spans_and_repeated_knots(self):
        s = surface([[[0,0,0],[0,3,0]],[[1,0,0],[1,3,0]],[[2,0,0],[2,3,0]]],
                    knots=[[0,0,2,4,4],[0,0,3,3]])
        j = jacobian_bounds(s,[[1,0,0],[0,1,0]])
        self.assertEqual(j,[[(F(1,2),F(1,2)),(0,0)],[(0,0),(1,1)]])
        self.assertTrue(check(s)['verified'])
        # This basis guarantees only C0 continuity; the conservative checker
        # does not infer matching derivatives from these particular poles.
        self.assertFalse(check(s)['regular_everywhere'])

    def test_rational_and_discontinuous_surfaces_refused(self):
        s = surface([[[0,0,0],[0,1,0]],[[1,0,0],[1,1,0]]])
        s['weights'][0][0]=2
        with self.assertRaises(ValueError):check(s)
        s = surface([[[x,0,0],[x,1,0]] for x in (0,1,2,3)],
                    knots=[[0,0,F(1,2),F(1,2),1,1],[0,0,1,1]])
        with self.assertRaises(ValueError):check(s)

    def test_trim_domain_is_bound_to_the_proof(self):
        s = surface([[[0,0,0],[0,1,0]],[[1,0,0],[1,1,0]]])
        self.assertTrue(check(s,[[0,1],[0,1]])['verified'])
        with self.assertRaises(ValueError):check(s,[[0,2],[0,1]])
