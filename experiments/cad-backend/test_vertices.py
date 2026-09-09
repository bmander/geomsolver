"""Known endpoint distances and vertex links that distinguish touching shells."""
from fractions import Fraction as F
import unittest

from check_vertices import endpoint_bound
from curve_jets import Curve
from vertex_topology import link_cycle


class Vertices(unittest.TestCase):
    def test_known_endpoint_distance(self):
        line=Curve(dict(kind='line',location=[0,0,0],direction=[1,2,0],interval=[0,1]))
        bound=endpoint_bound(line,1,[4,6,0])
        self.assertGreaterEqual(bound,5)
        self.assertLess(bound-5,F(1,10**40))

    def test_cycle_including_two_distinct_ends_of_a_closed_edge(self):
        self.assertEqual(link_cycle([((1,0),(1,1)),((1,1),(1,0))]),2)
        self.assertEqual(link_cycle([(0,1),(1,2),(2,0)]),3)

    def test_two_closed_fans_at_one_vertex_are_refused(self):
        # Each edge incidence has degree two, but joining two shells at one
        # named vertex gives two circles rather than a manifold vertex link.
        with self.assertRaises(ValueError):
            link_cycle([(0,1),(1,2),(2,0),(3,4),(4,5),(5,3)])
        with self.assertRaises(ValueError):link_cycle([(0,1),(1,2)])


if __name__=='__main__':unittest.main()
