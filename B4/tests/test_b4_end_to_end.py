from pathlib import Path
import sys
import unittest

BASE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(BASE/'code'));sys.path.insert(0,str(BASE.parent/'B3/code'))
from run_offline import run_case
from offline_environment_b4 import make_case, OfflineTransportB4
from protocol_adapter import Client
from solver_g0 import G0Solver, RobotPort


class EndToEndTests(unittest.TestCase):
    def test_independent_stress_patterns(self):
        for i,pattern in enumerate(('backface_origin','coverage_edge','cluster','origin','adversarial_fan')):
            with self.subTest(pattern=pattern):
                r=run_case(make_case(64090010+i,pattern=pattern),error_mode='endpoints')
                self.assertTrue(r['full_clear'],r['error']);self.assertEqual(r['stats']['fan_invariant_failures'],0)

    def test_forced_fallback_is_finite_and_direction_independent(self):
        env=OfflineTransportB4(make_case(64090020,count=10,pattern='all_directional'))
        c=Client(env,'offline-test');c.enter();s=G0Solver(RobotPort(c),fan_limit=0);s.run_all();c.exit()
        self.assertEqual(len(s.cleared),10);self.assertGreater(s.stats['fallback_triggers'],0)
        self.assertLessEqual(s.stats['fallback_clear_cover_count'],108*10)


if __name__=='__main__':unittest.main()
