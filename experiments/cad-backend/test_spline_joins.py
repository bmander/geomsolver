from fractions import Fraction as F
import unittest

from spline_embedding import check
from spline_joins import joined,prepare


def plane(y0,y1,shift=0):
    return dict(degrees=[1,1],knots=[[0,0,1,1],[0,0,1,1]],weights=[[1,1],[1,1]],
                poles=[[[F(u)+shift,F(y0),F(0)],[F(u)+shift,F(y1),F(0)]] for u in (0,1)])


class SplineJoins(unittest.TestCase):
    def test_both_boundary_orientations(self):
        for a,b,ends in [(plane(0,1),plane(1,2),[1,0]),(plane(1,0),plane(2,1),[0,1])]:
            surface=joined(a,b,ends)
            self.assertEqual(surface['knots'][1],[0,0,1,2,2])
            self.assertTrue(check(surface,[[0,1],[0,2]])['verified'])

    def test_overlap_away_from_shared_edge_refused(self):
        self.assertFalse(check(joined(plane(0,1),plane(1,F(1,2)),[1,0]))['verified'])

    def test_gapped_boundary_cannot_be_joined_implicitly(self):
        with self.assertRaises(ValueError):joined(plane(0,1),plane(1,2,F(1,100)),[1,0])

    def test_explicit_shared_coefficients_and_bounded_movement(self):
        delta=F(1,10**10)
        faces={(0,0):dict(kind='bspline',surface=plane(0,1)),
               (0,1):dict(kind='bspline',surface=plane(1,2,delta))}
        curve=dict(kind='bspline',degree=1,knots=[0,0,1,1],weights=[1,1],interval=[0,1],poles=[[0,1,0],[1,1,0]])
        edges=dict(members=[dict(member=0,edges=[dict(edge_index=1,curve=curve,incidences=[
            dict(face_index=i,reversed=bool(i),curve=dict(kind='line',location=[0,1-i],direction=[1,0],interval=[0,1]))
            for i in (0,1)])])])
        surfaces,joins,movement=prepare(faces,edges)
        self.assertEqual(surfaces[0,0]['poles'][0][-1],(delta/2,1,0))
        self.assertEqual(surfaces[0,0]['poles'][0][-1],surfaces[0,1]['poles'][0][0])
        self.assertLessEqual(max(movement.values()),delta)
        self.assertEqual(faces[0,0]['surface']['poles'][0][-1],[0,1,0])
        self.assertTrue(check(joined(surfaces[0,0],surfaces[0,1],joins[0]['boundaries']))['verified'])
