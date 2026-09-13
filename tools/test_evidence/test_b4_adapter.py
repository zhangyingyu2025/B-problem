import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parent))
import run_b4_test as runner
from export import audit
from B4.code.offline_environment_b4 import OfflineTransportB4
from B4.code.run_offline import canonical_case


class B4AdapterTests(unittest.TestCase):
    def test_v2_offline_log_export_and_dedup(self):
        runner.verify_source()
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);folder=root/'B4/results/v2_rehearsal/run-test'
            case=canonical_case(64000000)
            result=runner.run_session(folder,'rehearsal','offline-test','unused',OfflineTransportB4(case))
            self.assertEqual(result['status'],'completed')
            self.assertEqual(result['virtual_time_s'],5801.433429)
            self.assertTrue((folder/'simulator_logs').is_dir())
            report=audit(folder/'actions.jsonl')
            self.assertTrue(report['normal_exit_and_audit_ok'])
            self.assertEqual(report['cleared_count'],len(case['sources']))
            with patch.object(runner,'ROOT',root),contextlib.redirect_stdout(io.StringIO()):
                runner.record(folder,'rehearsal','TEST-CODE')
                runner.record(folder,'rehearsal','TEST-CODE')
                with self.assertRaises(ValueError):runner.record(folder,'formal','TEST-CODE')
            self.assertEqual(len(json.loads((folder.parent/'summary/runs.json').read_text(encoding='utf-8'))),1)
            self.assertFalse((root/'B4/results/v2_formal').exists())

    def test_transport_failure_retained_without_retry(self):
        def broken(path,payload):raise TimeoutError('synthetic timeout')
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp)/'run'
            result=runner.run_session(folder,'rehearsal','offline-test','unused',broken)
            self.assertEqual(result['status'],'incomplete')
            self.assertIn('synthetic timeout',result['error'])
            events=[json.loads(x) for x in (folder/'actions.jsonl').read_text().splitlines()]
            self.assertEqual(len(events),1)
            self.assertEqual(events[0]['kind'],'transport_failure')


if __name__=='__main__':unittest.main()
