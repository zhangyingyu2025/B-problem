import ast
from pathlib import Path
import sys
import unittest

BASE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(BASE/'code'));sys.path.insert(0,str(BASE.parent/'B3/code'))
from offline_environment_b4 import OfflineTransportB4, make_case
from protocol_adapter import Client
from solver_g0 import G0Solver, RobotPort


class ReplayPort:
    """Only action responses/state. No transport, case, source list, or seed."""
    def __init__(self,events):
        self.events=[e for e in events if e.get('kind')=='action' and e['path'] in ('/measure','/clear')]
        self.index=0;self.position=(0.,0.);self.channel=1;self.virtual_time_s=0.
    def perform(self,path,p,c,reason):
        event=self.events[self.index];self.index+=1
        assert (path,tuple(p),c,reason)==(event['path'],tuple(event['payload']['position'][k] for k in ('x','y')),event['payload']['channel'],event['decision_reason'])
        self.position=tuple(p);self.channel=event['after']['channel'];self.virtual_time_s=event['response']['virtual_time_s']
        return event['response']
    def measure(self,p,c,reason):return self.perform('/measure',p,c,reason)
    def clear(self,p,c,reason):return self.perform('/clear',p,c,reason)
    def record(self,event):pass


class SourceBlindTests(unittest.TestCase):
    def test_replay_needs_only_observations(self):
        env=OfflineTransportB4(make_case(64090005,pattern='all_directional'))
        client=Client(env,'offline-test');client.enter();s=G0Solver(RobotPort(client));s.run_all();client.exit()
        replay=ReplayPort(client.events);r=G0Solver(replay);r.run_all()
        self.assertEqual(replay.index,len(replay.events));self.assertEqual(r.cleared,s.cleared)
        self.assertFalse(hasattr(r,'case'));self.assertFalse(hasattr(r,'sources'))

    def test_solver_never_imports_or_reads_truth(self):
        tree=ast.parse((BASE/'code/solver_g0.py').read_text(encoding='utf-8'))
        for node in ast.walk(tree):
            if isinstance(node,ast.ImportFrom): self.assertNotIn('offline_environment',node.module or '')
            if isinstance(node,ast.Attribute): self.assertNotIn(node.attr,('_sources','sources','transport','case','direction_deg'))


if __name__=='__main__':unittest.main()
