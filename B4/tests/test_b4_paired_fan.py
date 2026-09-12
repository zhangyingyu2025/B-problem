import itertools
import math
from pathlib import Path
import random
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'code'))
from paired_fan import paired_fan, theorem_margin


def check_geometry(r, lb, true_angle, error, orientation_offset):
    # Rotate the physical halfplane with the source/anchor geometry; compute
    # its dot predicate independently of OfflineTransport's angular predicate.
    a = math.radians(true_angle); g = (r*math.cos(a),r*math.sin(a))
    emission = math.radians(true_angle+180+orientation_offset)
    normal = (math.cos(emission),math.sin(emission))
    ps = paired_fan((0.,0.),true_angle+error,lb)
    inside = []
    for p in ps:
        assert math.dist(p,g) <= r+1e-9, (r,lb,true_angle,error,p)
        projection = math.fsum((normal[0]*(p[0]-g[0]),normal[1]*(p[1]-g[1])))
        inside.append(projection >= -1e-10)
    assert any(inside), (r,lb,true_angle,error,orientation_offset)
    if not inside[0]:
        assert inside[1]


class FanTests(unittest.TestCase):
    def test_strict_ratio_margin(self):
        self.assertGreater(theorem_margin()['ratio_margin'],.08)

    def test_deterministic_endpoints(self):
        for r,ratio,angle,error,orientation in itertools.product(
                (20.0000001,21,100,1000,1500),(.01,.5,1),
                (0,37,90,179,270,359),(-1,0,1),(-90,-89.999999,0,89.999999,90)):
            check_geometry(r,r*ratio,angle,error,orientation)

    def test_100000_random_cases(self):
        rng=random.Random(64090002)
        for i in range(100000):
            r=rng.uniform(20.000001,1500)
            lb=r*rng.uniform(.001,1)
            e=(-1,1)[i%2] if i%3==0 else rng.uniform(-1,1)
            orientation=(-90,90)[i%2] if i%5==0 else rng.uniform(-90,90)
            check_geometry(r,lb,rng.uniform(0,360),e,orientation)

    def test_invalid_inputs(self):
        for lb in (0,-1,1501,float('nan'),True):
            with self.assertRaises(ValueError): paired_fan((0.,0.),0.,lb)

    def test_known_39m_counterexample(self):
        # G is 21 m east of S, emission west: both old +/-1 degree
        # moves of 39 m cross to the back side, despite a failed clear at S.
        for sign in (-1,1):
            a=math.radians(sign); p=(39*math.cos(a),39*math.sin(a))
            self.assertGreater(p[0],21)
        check_geometry(21,20,0,0,0)


if __name__ == '__main__':
    unittest.main()
