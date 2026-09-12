import math
import random
import sys
import unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'B4/code'))
sys.path.insert(0,str(ROOT/'B3/code'))
sys.path.insert(0,str(ROOT/'B3/experimental/E13'))

import solver_g35 as g

class G40MathTests(unittest.TestCase):
    def setUp(self):
        self.obs={'x':0.,'y':0.,'svd_deg':0.}

    def test_safe_segment_points_are_within_all_first_sector_candidates(self):
        a=(300.,-1200.);b=(900.,900.)
        p=g.safe_point_on_segment(self.obs,a,b,0)
        self.assertIsNotNone(p)
        self.assertLessEqual(g.safe_rho(self.obs,p),1000.00001)
        # Dense deterministic check of the theorem's sector envelope.
        for i in range(61):
            r=1500*i/60
            for j in range(41):
                e=-g.ALPHA+2*g.ALPHA*j/40
                src=(r*math.cos(e),r*math.sin(e))
                self.assertLessEqual(math.dist(p,src),1000.00002)

    def test_symmetric_backside_pair_radius(self):
        up=(565.355,330.004);lo=(565.355,-330.004)
        R=g.chord_radial_upper(self.obs,up,lo)
        self.assertIsNotNone(R)
        self.assertAlmostEqual(R,565.4411,places=2)

    def test_certified_disk_clipping_never_excludes_circle_points(self):
        geom={'status':'bounded','polygon':[(-1000.,-1000.),(1000.,-1000.),(1000.,1000.),(-1000.,1000.)],
              'mec':((0.,0.),1500.),'centers':[(0.,0.)]}
        out=g.clip_radial_outer(geom,(100.,-50.),300.)
        # Cardinal and many angular boundary points must remain inside clipped polygon.
        poly=out['polygon']
        def inside(p):
            vals=[]
            for a,b in zip(poly,poly[1:]+poly[:1]):
                vals.append((b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0]))
            return all(v>=-2e-5 for v in vals) or all(v<=2e-5 for v in vals)
        for k in range(360):
            e=2*math.pi*k/360
            p=(100+300*math.cos(e),-50+300*math.sin(e))
            self.assertTrue(inside(p))

    def test_active_strip_cover_contains_dense_parallelogram_samples(self):
        first={'x':0.,'y':0.,'svd_deg':0.}
        safe={'x':800.,'y':600.,'svd_deg':55.}
        D1=900.
        poly=g._strip_polygon(first,1500.,safe,D1)
        self.assertIsNotNone(poly)
        z=g._active_strip_centers(first,1500.,safe,D1,(safe['x'],safe['y']),max_clears=30)
        self.assertIsNotNone(z)
        centers=z['centers']
        # Bilinear samples over convex combinations of the parallelogram vertices.
        # Sort cyclically then interpolate between opposite edges.
        for i in range(31):
            s=i/30
            left=(poly[0][0]*(1-s)+poly[1][0]*s,poly[0][1]*(1-s)+poly[1][1]*s)
            right=(poly[3][0]*(1-s)+poly[2][0]*s,poly[3][1]*(1-s)+poly[2][1]*s)
            for j in range(31):
                t=j/30
                p=(left[0]*(1-t)+right[0]*t,left[1]*(1-t)+right[1]*t)
                self.assertLessEqual(min(math.dist(p,c) for c in centers),g.STRIP_CLEAR_R+2e-5)

if __name__=='__main__':
    unittest.main()
