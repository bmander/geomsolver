from fractions import Fraction as F
import unittest

from bernstein import patches, coordinate, midpoint
from spline_separation import Node, separate


def node(grid,degrees=(1,1)):
    return Node.surface(dict(poles=grid,degrees=list(degrees),
                             knots=[[0]*(d+1)+[1]*(d+1) for d in degrees],
                             weights=[[1 for p in row] for row in grid]))


class SplineSeparation(unittest.TestCase):
    def test_oblique_planes_have_positive_whole_surface_gap(self):
        a = node([[[x,y,x] for y in (0,1)] for x in (0,1)])
        b = node([[[x,y,x+F(1,10)] for y in (0,1)] for x in (0,1)])
        result = separate(a,b)
        self.assertTrue(result['verified'])
        self.assertGreater(result['distance_lower'],0)
        self.assertEqual(result['tested_cells'],1)

    def test_crossing_and_touching_cannot_pass(self):
        a = node([[[x,y,0] for y in (0,1)] for x in (0,1)])
        for shift in (F(-1,2),F(0)):
            b = node([[[x,y,x+shift] for y in (0,1)] for x in (0,1)])
            result = separate(a,b,max_cells=8)
            self.assertFalse(result['verified'])
            self.assertIsNone(result['distance_lower'])
            self.assertGreater(result['unresolved_cells'],0)

    def test_exact_tensor_subdivision_preserves_polynomial(self):
        # S(u,v)=(u^2,v^3,u*v), including the children's complete domains.
        root = node([[[x,y,F(i,2)*F(j,3)] for j,y in enumerate((0,0,0,1))]
                     for i,x in enumerate((0,0,1))],degrees=(2,3))
        children = root.split()
        self.assertEqual([c.knots[0][0] for c in children],[0,F(1,2)])
        self.assertEqual([c.knots[0][-1] for c in children],[F(1,2),1])
        self.assertEqual(children[0].grid[-1],children[1].grid[0])
        for child in children:
            data=dict(degrees=child.degrees,knots=child.knots,poles=child.grid,
                      weights=[[1 for p in row] for row in child.grid])
            for patch in patches(data):
                u,v = [sum(d)/2 for d in patch['domain']]
                actual = [midpoint(coordinate(patch['poles'],axis)) for axis in range(3)]
                self.assertEqual(actual,[u*u,v*v*v,u*v])

    def test_invalid_budget(self):
        a=node([[[x,y,0] for y in (0,1)] for x in (0,1)])
        with self.assertRaises(ValueError):separate(a,a,max_cells=0)
