import math
from pathlib import Path
import random
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'code'))
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'B3/code'))
from b4_geometry import geometry, lower_distance, fallback_cover
from solver_g0 import G0Solver, RobotPort
from offline_environment_b4 import OfflineTransportB4
from protocol_adapter import Client


class GeometryTests(unittest.TestCase):
    def test_negative_observation_does_not_shrink(self):
        case={'seed':1,'sources':[{'channel':1,'x':500,'y':0,'radius':1000,'kind':'directional','direction_deg':180}]}
        client=Client(OfflineTransportB4(case),'offline-test');client.enter()
        solver=G0Solver(RobotPort(client));solver.observe((0,0),1,'test')
        before=solver.tracks[1].geometry
        solver.observe((501,0),1,'backside ordinary')
        self.assertEqual(solver.tracks[1].geometry,before)
        self.assertEqual(solver.tracks[1].type_state,'unknown')
        client.exit()

    def test_distance_is_to_edges_not_vertices(self):
        k=[(-10,-10),(10,-10),(10,10),(-10,10)]
        self.assertAlmostEqual(lower_distance((0,20),k),10,places=5)
        self.assertEqual(lower_distance((0,0),k),0)

    def test_geometry_retains_known_truth(self):
        rng=random.Random(64090003)
        for _ in range(30):
            g=(rng.uniform(-400,400),rng.uniform(-400,400)); dirs=[]; signals=[]
            for k in range(4):
                a=rng.uniform(0,2*math.pi);r=rng.uniform(100,900)
                p=(g[0]+r*math.cos(a),g[1]+r*math.sin(a))
                theta=math.degrees(math.atan2(g[1]-p[1],g[0]-p[0]))+rng.uniform(-.99,.99)
                dirs.append({'x':p[0],'y':p[1],'svd_deg':theta%360});signals.append(p)
                out=geometry(dirs,signals);self.assertEqual(out['status'],'bounded')
                self.assertLessEqual(lower_distance(g,out['polygon']),1e-5)

    def test_fallback_covers_whole_first_wedge(self):
        rng=random.Random(64090004);obs={'x':137.,'y':-220.,'svd_deg':37.}
        points=fallback_cover(obs)
        for _ in range(5000):
            d=rng.uniform(0,1500);a=math.radians(37+rng.uniform(-1,1))
            g=(obs['x']+d*math.cos(a),obs['y']+d*math.sin(a))
            self.assertLess(min(math.dist(g,p) for p in points),20)
        self.assertLessEqual(len(points),108)


if __name__=='__main__': unittest.main()
