import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0,str(Path(__file__).resolve().parent))
from export import audit, draw_svg, export, update_ledger

class EvidenceTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.folder=Path(self.tmp.name)
        # Self-contained synthetic fixture; no ignored/private rehearsal files.
        self.events=[];state={'position':[0.,0.],'channel':1,'virtual_time_s':0.}
        specs=[('/enter',None,None,0.,{}),('/measure',[30.,40.],2,16.,{'measure_result':'direction','svd_deg':10.}),
               ('/clear',[30.,40.],2,21.,{'clear_result':'success'}),('/measure',[30.,40.],3,27.,{'measure_result':'no_signal'}),
               ('/clear',[30.,40.],3,30.,{'clear_result':'no_target_in_range'}),('/exit',None,None,30.,{'exit_reason':'user_exit'})]
        for i,(path,pos,ch,vt,result) in enumerate(specs):
            before=copy.deepcopy(state);payload={'request_id':f'fixture-{i}'}
            if pos is not None:
                payload.update(position=dict(zip(('x','y'),pos)),channel=ch);state['position']=pos
                if path=='/measure':state['channel']=ch
            state['virtual_time_s']=vt
            self.events.append({'kind':'action','path':path,'payload':payload,
                                'response':{'accepted':True,'virtual_time_s':vt,'real_timestamp_ms':100000+400*i,**result},
                                'before':before,'after':copy.deepcopy(state),'wall_elapsed_s':.4*i})

    def write(self,events):
        p=self.folder/'actions.jsonl';p.write_text('\n'.join(json.dumps(x) for x in events),encoding='utf-8');return p

    def test_rehearsal_metrics_and_svg_all_actions(self):
        r=audit(self.write(self.events)); self.assertTrue(r['normal_exit_and_audit_ok'])
        self.assertEqual(r['cleared_count'],1);self.assertAlmostEqual(r['mean_time_per_clear_s'],30.)
        self.assertEqual(r['measurements'],2);self.assertEqual(r['switches'],2)
        self.assertAlmostEqual(r['server_response_elapsed_s'],2.)
        self.assertIsNone(r['clear_fraction'])
        svg=ET.fromstring(draw_svg(r,'演练 <test>'))
        self.assertEqual(len(svg.findall('.//{http://www.w3.org/2000/svg}title')),4)
        points=svg.find('{http://www.w3.org/2000/svg}polyline').attrib['points'].split()
        self.assertEqual(len(points),5)

    def test_retry_duplicate_counted_once(self):
        e=next(e for e in self.events if e.get('kind')=='action' and e['path']=='/clear')
        idx=self.events.index(e);self.events.insert(idx,copy.deepcopy(e))
        self.assertEqual(audit(self.write(self.events))['cleared_count'],1)

    def test_unresolved_action_is_incomplete(self):
        self.events.insert(-1,{'kind':'transport_failure','payload':{'request_id':'unconfirmed'}})
        r=audit(self.write(self.events));self.assertFalse(r['normal_exit_and_audit_ok'])
        self.assertIsNone(r['mean_time_per_clear_s']);self.assertIsNone(r['server_response_elapsed_s'])

    def test_missing_exit_is_incomplete(self):
        r=audit(self.write(self.events[:-1]));self.assertFalse(r['normal_exit_and_audit_ok'])
        self.assertIsNone(r['mean_time_per_clear_s'])

    def test_conflicting_request_rejected(self):
        e=copy.deepcopy(self.events[0]);e['response']['virtual_time_s']=99;self.events.insert(1,e)
        with self.assertRaises(ValueError):audit(self.write(self.events))

    def test_cost_mismatch_not_published(self):
        e=self.events[-1];e['response']['virtual_time_s']+=1;e['after']['virtual_time_s']+=1
        r=audit(self.write(self.events));self.assertFalse(r['normal_exit_and_audit_ok']);self.assertIsNone(r['mean_time_per_clear_s'])

    def test_export_protects_input_and_labels_rehearsal(self):
        log=self.write(self.events); original=log.read_bytes()
        manifest=self.folder/'manifest.json'
        manifest.write_text(json.dumps({'runs':[{'problem':'B3','mode':'rehearsal','case_code':None,'actions':'actions.jsonl'}]}),encoding='utf-8')
        output=self.folder/'out';r=export(manifest,output)
        self.assertEqual(log.read_bytes(),original)
        self.assertIn('演练',(output/'table.md').read_text(encoding='utf-8'))
        self.assertIsNone(r[0]['case_code'])
        with self.assertRaises(ValueError):export(manifest,output)
        export(manifest,self.folder/'ledger-artifacts',write_tables=False)
        self.assertFalse((self.folder/'ledger-artifacts/table.csv').exists())
        self.assertTrue((self.folder/'ledger-artifacts/run-01-trajectory.svg').exists())

    def test_cumulative_table_dedup_new_session_and_lock(self):
        log=self.write(self.events)
        manifest=self.folder/'manifest.json'
        manifest.write_text(json.dumps({'runs':[{'problem':'B3','mode':'rehearsal','actions':str(log)}]}),encoding='utf-8')
        reports=export(manifest,self.folder/'out')
        ledger=self.folder/'ledger'
        self.assertEqual(len(update_ledger(reports,ledger)),1)
        reports[0]['case_code']='case-1'
        self.assertEqual(len(update_ledger(reports,ledger)),1)
        again=copy.deepcopy(reports);again[0]['case_code']=None
        self.assertEqual(update_ledger(again,ledger)[0]['case_code'],'case-1')
        # Separate session with identical performance must still get its own row.
        for e in self.events:e['payload']['request_id']+='-new'
        new=audit(self.write(self.events))
        other=copy.deepcopy(reports[0]);other.update(new)
        other['actions_path']=str(self.folder/'second-session/actions.jsonl')
        self.assertEqual(len(update_ledger([other],ledger)),2)
        import csv
        with (ledger/'table.csv').open(encoding='utf-8-sig',newline='') as f:
            rows=list(csv.reader(f))
        self.assertEqual(len(rows),3)
        rows[1][3]='manually-pasted-code'
        with (ledger/'table.csv').open('w',encoding='utf-8-sig',newline='') as f:
            csv.writer(f).writerows(rows)
        self.assertEqual(update_ledger(again,ledger)[0]['case_code'],'manually-pasted-code')
        mixed=copy.deepcopy(other);mixed['log_sha256']='different-formal-log';mixed['mode']='formal'
        with self.assertRaises(ValueError):update_ledger([mixed],ledger)
        invalid=copy.deepcopy(reports);invalid[0]['mode']='formal'
        with self.assertRaises(ValueError):update_ledger(invalid,ledger)
        self.assertFalse((ledger/'.update.lock').exists())
        (ledger/'.update.lock').touch()
        with self.assertRaises(FileExistsError):update_ledger(reports,ledger)
        self.assertTrue((ledger/'.update.lock').exists())

    def test_formal_runner_mocked_no_network(self):
        from unittest.mock import patch
        import contextlib,io
        sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'B3/code'))
        import run_e13_formal as runner
        args=['run_e13_formal.py','--robot-id','test','--formal-ready','--output-dir','B3/结果/e13_formal/run-1']
        result={'status':'completed','cleared_count':1}
        with patch.object(runner,'ROOT',self.folder), patch.object(sys,'argv',args), \
             patch.object(runner,'HTTPTransport') as transport,patch.object(runner,'Client'), \
             patch.object(runner,'run_with_client',return_value=result) as run, \
             patch.object(runner,'provenance',return_value={}), contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(runner.main(),0)
            self.assertEqual(run.call_args.args[1],'operator_declared_official_formal')
            saved=json.loads((self.folder/'B3/结果/e13_formal/run-1/result.json').read_text(encoding='utf-8'))
            self.assertTrue(saved['official_formal']);self.assertFalse(saved['official_rehearsal'])
            self.assertIsNone(saved['true_source_count'])
            with self.assertRaises(FileExistsError):runner.main()
            args[-1]='B3/结果/e13_rehearsal/run-2'
            with self.assertRaises(SystemExit):runner.main()
            self.assertEqual(transport.call_count,1)


if __name__=='__main__':unittest.main()
