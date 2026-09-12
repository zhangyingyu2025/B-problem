import ast
from pathlib import Path
import sys
import unittest

BASE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(BASE/'code'))
from run_offline import run_case
from offline_environment_b4 import make_case, OfflineTransportB4
from protocol_adapter import Client
from solver_g0 import RobotPort
from solver_g1 import G1Solver
from test_b4_source_blind import ReplayPort


class G1Tests(unittest.TestCase):
    def test_source_blind_replay(self):
        env=OfflineTransportB4(make_case(64090030,pattern='all_directional'),error_mode='endpoints')
        client=Client(env,'offline-test');client.enter();s=G1Solver(RobotPort(client));s.run_all();client.exit()
        replay=ReplayPort(client.events);r=G1Solver(replay);r.run_all()
        self.assertEqual(replay.index,len(replay.events));self.assertEqual(r.cleared,s.cleared)
        tree=ast.parse((BASE/'code/solver_g1.py').read_text(encoding='utf-8'))
        for node in ast.walk(tree):
            if isinstance(node,ast.ImportFrom): self.assertNotIn('offline_environment',node.module or '')
            if isinstance(node,ast.Attribute): self.assertNotIn(node.attr,('_sources','sources','transport','case','direction_deg'))

    def test_patterns_all_clear(self):
        for i,pattern in enumerate(('backface_origin','coverage_edge','cluster','origin','adversarial_fan','boundary')):
            with self.subTest(pattern=pattern):
                r=run_case(make_case(64090031+i,pattern=pattern),variant='G1',error_mode='endpoints')
                self.assertTrue(r['full_clear'],r['error'])
                self.assertEqual(r['stats']['fan_invariant_failures'],0)

    def test_forced_fallback(self):
        env=OfflineTransportB4(make_case(64090037,count=10,pattern='all_directional'))
        client=Client(env,'offline-test');client.enter();s=G1Solver(RobotPort(client),fan_limit=0);s.run_all();client.exit()
        self.assertEqual(len(s.cleared),10)
        self.assertLessEqual(s.stats['fallback_clear_cover_count'],108*10)


if __name__=='__main__':unittest.main()
