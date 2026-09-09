from fractions import Fraction as F
import unittest

from trim_loops import close_wire, corner_separator, adjacent, jordan, displacement_bound, chart_domain


def polygon(points):
    points = [tuple(F(x) for x in p) for p in points]
    return [[[a,b]] for a,b in zip(points,points[1:]+points[:1])]


class TrimLoops(unittest.TestCase):
    def test_closed_representative_and_explicit_error(self):
        curves = polygon([(0,0),(1,0),(0,1)])
        curves[1][0][0] = (F(1),F(1,100))
        changed,error = close_wire(curves)
        self.assertEqual(changed[0][-1][-1],(F(1),F(1,200)))
        self.assertEqual(error,[[0,F(1,200)],[0,F(1,200)],[0,0]])
        self.assertEqual(curves[0][-1][-1],(1,0))
        self.assertEqual(jordan(changed)['status'],'verified')

    def test_adjacent_edges_and_retracing(self):
        p = [(F(-1),F(0)),(F(0),F(0))]
        q = [(F(0),F(0)),(F(1),F(1))]
        self.assertTrue(corner_separator(p,q))
        self.assertTrue(adjacent([p],[q])[0])
        self.assertFalse(adjacent([p],[list(reversed(p))],max_cells=16)[0])
        with self.assertRaises(ValueError):adjacent([p],[[(1,0),(2,0)]])

    def test_bow_tie_and_second_adjacent_contact_refused(self):
        self.assertEqual(jordan(polygon([(0,0),(1,1),(0,1),(1,0)]),max_cells=16)['status'],'unresolved')
        p = [[(F(-1),F(0)),(F(0),F(0))]]
        q = [[(F(0),F(0)),(F(0),F(1)),(F(-2),F(-1))]]
        self.assertFalse(adjacent(p,q,max_cells=32)[0])

    def test_multispan_and_constant_piece(self):
        curves = polygon([(0,0),(1,0),(1,1),(0,1)])
        curves[0] = [[(F(0),F(0)),(F(1,2),F(0))],[(F(1,2),F(0)),(F(1),F(0))]]
        self.assertEqual(jordan(curves)['status'],'verified')
        curves[0].insert(1,[(F(1,2),F(0)),(F(1,2),F(0))])
        self.assertEqual(jordan(curves,max_cells=16)['status'],'unresolved')

    def test_world_displacement_and_frame_validation(self):
        original = polygon([(0,0),(1,0),(0,1)])
        face = dict(kind='sphere',radius=10,basis=[[1,0,0],[0,1,0],[0,0,1]])
        self.assertEqual(displacement_bound(face,original,original,[[F(1,100),F(1,200)]]),F(3,20))
        face['kind']='cone'
        self.assertEqual(displacement_bound(face,original,original,[[F(1,100),F(1,200)]]),F(23,200))
        face['basis'][0][0]=2
        with self.assertRaises(ValueError):displacement_bound(face,original,original,[[0,0]])

    def test_chart_period_poles_and_apex(self):
        face = dict(kind='sphere',radius=10,basis=[[1,0,0],[0,1,0],[0,0,1]])
        simple = polygon([(0,0),(1,0),(1,1),(0,1)])
        self.assertTrue(chart_domain(face,simple)['injective_rectangle'])
        self.assertFalse(chart_domain(face,polygon([(0,0),(7,0),(7,1),(0,1)]))['injective_rectangle'])
        self.assertFalse(chart_domain(face,polygon([(0,1),(1,1),(1,2),(0,2)]))['injective_rectangle'])
        face.update(kind='cone',angle=1)
        self.assertTrue(chart_domain(face,simple)['injective_rectangle'])
        self.assertFalse(chart_domain(face,polygon([(0,-20),(1,-20),(1,1),(0,1)]))['injective_rectangle'])
