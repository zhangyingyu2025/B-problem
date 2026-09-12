"""An actual mid-leg observation must be able to cancel a stale destination."""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'B3/experimental/E13'))
from solver import build_e13
from event_route import advance
from offline_environment import OfflineTransport
from protocol_adapter import Client


class EventRouteTests(unittest.TestCase):
    def test_owner_second_bearing_invalidates_old_supplement(self):
        case = {'seed': 54000201, 'sources': [{'channel': 1, 'x': 400., 'y': 0., 'radius': 1200.}]}
        client = Client(OfflineTransport(case, error_mode='zero'), 'offline-test')
        client.enter()
        engine = build_e13(client, 'R')
        engine.measure((0., 0.), 1, None, 'test_initial')
        engine.segment_points = lambda *args: [(.5, (500., 500.), 70.)]
        self.assertFalse(advance(engine, ('supp', 1, (750., 120.)), []))
        self.assertEqual(client.position, (500., 500.))
        self.assertEqual(len(engine.tracks[1]['dirs']), 2)
        self.assertEqual(sum(e.get('kind') == 'E13_owner_second_bearing' for e in client.events), 1)
        client.exit()

    def test_new_clear_task_cancels_old_destination(self):
        case = {'seed': 54000200, 'sources': [{'channel': 1, 'x': 400., 'y': 0., 'radius': 1200.}]}
        client = Client(OfflineTransport(case, error_mode='zero'), 'offline-test')
        client.enter()
        engine = build_e13(client, 'R')
        engine.measure((0., 0.), 1, None, 'test_initial')
        self.assertFalse(engine.eligible_targets())
        # Isolate the trigger from the already-tested geometric point generator.
        engine.segment_points = lambda *args: [(.5, (500., 500.), 70.)]
        destination = (1000., 1000.)
        self.assertFalse(advance(engine, ('cover', 1, destination), [(1, destination)]))
        self.assertEqual(client.position, (500., 500.))
        self.assertTrue(engine.eligible_targets())
        self.assertEqual(client.stats['clear_attempts'], 0)
        replans = [e for e in client.events if e.get('kind') == 'E13_transit_replan']
        self.assertEqual(len(replans), 1)
        self.assertEqual(replans[0]['new_action'][0], 'clear')
        self.assertTrue(engine.local_clear_from_mec(engine.tracks[1]))
        self.assertEqual(engine.env.cleared, {1})
        client.exit()


if __name__ == '__main__':
    unittest.main()
