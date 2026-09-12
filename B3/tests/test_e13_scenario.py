import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'B3/experimental/E13'))
from scenario_route import route_cost_with_first, scenarios, unseen_support


class ScenarioTests(unittest.TestCase):
    def test_dependency_changes_optimistic_route(self):
        nodes = [('cover', 1, (100., 0.)), ('supp', 2, (300., 0.)),
                 ('future_clear', 2, (200., 0.))]
        self.assertEqual(route_cost_with_first((0., 0.), nodes, [set(), set(), {1}], 0), 400.)
        self.assertEqual(route_cost_with_first((0., 0.), nodes, [set(), set(), set()], 0), 300.)

    def test_silence_restricts_forecast_without_becoming_certificate(self):
        support = unseen_support(((0., 0.),))
        self.assertTrue(support)
        self.assertTrue(all(1000 < math.hypot(*p) <= 1800 for p in support))
        sites = [(i+1, (1130*math.cos(i*math.pi/3), 1130*math.sin(i*math.pi/3))) for i in range(6)]
        tasks = [('cover', i, p) for i, p in sites]
        silent = {c: [(0., 0.)] for c in range(1, 21)}
        worlds = scenarios(tasks, {}, set(), silent, sites)
        self.assertEqual(worlds, scenarios(tasks, {}, set(), silent, sites))
        for nodes, deps in worlds:
            for i in range(len(tasks), len(nodes)):
                self.assertGreater(math.hypot(*nodes[i][2]), 1000)
                self.assertTrue(deps[i])
                self.assertTrue(all(math.dist(nodes[j][2], nodes[i][2]) <= 1000 for j in deps[i]))

    def test_no_future_discovery_after_complete_skeleton(self):
        worlds = scenarios([('clear', 1, (0., 0.))], {}, set(), {c: [] for c in range(1, 21)}, [])
        self.assertTrue(all(len(nodes) == 1 for nodes, _ in worlds))


if __name__ == '__main__':
    unittest.main()
