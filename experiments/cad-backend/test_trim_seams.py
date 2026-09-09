from fractions import Fraction as F
import unittest

from trim_seams import periodic_wire, strict_strip, check


def fixture():
    points = [(F(0),F(0)),(F(6),F(0)),(F(6),F(1)),(F(0),F(1))]
    curves = [[[a,b]] for a,b in zip(points,points[1:]+points[:1])]
    face = dict(kind='sphere',radius=1,basis=[[1,0,0],[0,1,0],[0,0,1]])
    return face,curves,[1,2,3,2]


class TrimSeams(unittest.TestCase):
    def test_exact_periodic_identification(self):
        face,curves,indices = fixture()
        result = check(face,curves,indices)
        self.assertTrue(result['embedded_annular_representative'])
        self.assertGreater(F(result['period_correction_bound']),F(28,100))
        self.assertEqual(result['seam_positions'],[1,3])

    def test_mismatched_seam_intervals_refused(self):
        face,curves,indices = fixture()
        curves[1][0][-1] = (F(6),F(2))
        with self.assertRaises(ValueError):check(face,curves,indices)

    def test_adjacent_seam_arcs_refused(self):
        face,curves,_ = fixture()
        with self.assertRaises(ValueError):check(face,curves,[1,2,2,3])

    def test_clamping_has_explicit_nonzero_error(self):
        face,curves,indices = fixture()
        curves[0][0].insert(1,(F(-1,100),F(0)))
        changed,error,seams,domain = periodic_wire(curves,indices)
        self.assertEqual(error[0][0],F(1,100))
        self.assertTrue(strict_strip(changed,seams,domain))
        self.assertEqual(curves[0][0][1][0],F(-1,100))

    def test_extra_strip_contact_and_pole_crossing_refused(self):
        face,curves,indices = fixture()
        changed,_,seams,domain = periodic_wire(curves,indices)
        changed[0] = [[(F(0),F(0)),(F(0),F(1,2))],[(F(0),F(1,2)),(F(6),F(0))]]
        self.assertFalse(strict_strip(changed,seams,domain))
        face['kind']='cone';face['angle']=0
        face['radius']=1
        self.assertTrue(check(face,curves,indices)['embedded_annular_representative'])
        face['kind']='sphere'
        for c in curves:
            c[0] = [(u,2*v) for u,v in c[0]]
        self.assertFalse(check(face,curves,indices)['embedded_annular_representative'])
