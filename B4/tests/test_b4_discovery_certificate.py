import copy
import math
from pathlib import Path
import random
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'code'))
from discovery_mesh import mesh25
from discovery_certificate import verify_mesh, absent_certified
from offline_environment_b4 import emission_contains


class CertificateTests(unittest.TestCase):
    def test_exact_certificate(self):
        result = verify_mesh(mesh25())
        self.assertEqual((result['vertices'],result['triangles']),(25,36))
        self.assertLess(result['max_edge_m'],1000)
        self.assertGreater(result['minimum_boundary_margin_m'],.49)

    def test_no_premature_absence(self):
        m=mesh25(); self.assertFalse(absent_certified(range(24),m))
        self.assertTrue(absent_certified(range(25),m))

    def test_hole_rejected(self):
        m=mesh25(); m['triangles'].pop()
        with self.assertRaises(ValueError): verify_mesh(m)

    def test_flipped_triangle_rejected(self):
        m=mesh25(); m['triangles'][0]=tuple(reversed(m['triangles'][0]))
        with self.assertRaises(ValueError): verify_mesh(m)

    def test_boundary_shrink_rejected(self):
        m=mesh25(); m['vertices']=[(x*.999,y*.999) for x,y in m['vertices']]
        with self.assertRaises(ValueError): verify_mesh(m)

    def test_long_edge_rejected(self):
        m=mesh25(); m['range']=970
        with self.assertRaises(ValueError): verify_mesh(m)

    def test_random_counterexample_search_not_proof(self):
        rng=random.Random(64090001); mesh=mesh25()
        for i in range(10000):
            r=1800 if i % 5 == 0 else 1800*math.sqrt(rng.random())
            a=rng.uniform(0,2*math.pi); g=(r*math.cos(a),r*math.sin(a))
            s={'x':g[0],'y':g[1],'kind':'directional','direction_deg':rng.uniform(0,360)}
            self.assertTrue(any(math.dist(g,q)<=1000 and emission_contains(s,q) for q in mesh['vertices']))


if __name__ == '__main__':
    unittest.main()
