from pathlib import Path
import sys
import unittest
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'B3/experimental/E13'))
sys.path.insert(0, str(ROOT/'B3/code'))
from measurement_options import choose_points
from e12_coverage_hook import E11


class MeasurementOptionsTests(unittest.TestCase):
    def test_joint_side_choice_can_improve_nearest_current_choice(self):
        start = (-200., 0.)
        fixed = [('cover', 1, (0., 200.))]
        choices = {('supp', 2): [(0., -100.), (0., 100.)]}
        route = choose_points(start, fixed, choices)
        old = E11.e5.fast_open_route(start, fixed+[('supp', 2, choices['supp', 2][0])])
        self.assertLess(E11.e5.route_cost(start, route), E11.e5.route_cost(start, old))
        self.assertEqual(len(route), 2)
        self.assertIn(('supp', 2, (0., 100.)), route)


if __name__ == '__main__': unittest.main()
