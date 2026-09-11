import json
import math
from pathlib import Path
import random
import sys
import tempfile
import unittest

BASE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(BASE/'code'))
from coverage import sites,certificate,clearance_grid
from channel_manager import ChannelManager,channel_order
from source_track import SourceTrack
from protocol_adapter import Client,ProtocolError,BudgetExceeded
from offline_environment import OfflineTransport,make_case
from run_b3 import run_offline
from scheduler import insertion_distance


class CoverageTests(unittest.TestCase):
    def test_analytical_certificate(self):
        c=certificate();self.assertLess(c['annulus_upper_m'],1000)
        self.assertAlmostEqual(c['open_route_length_m'],6780)
        route=sites();self.assertAlmostEqual(sum(math.dist(a,b) for a,b in zip(route,route[1:])),6780)
    def test_boundary_sector_midlines_and_random(self):
        route=sites();rng=random.Random(20260911)
        points=[(0,0)]+[(1800*math.cos(math.radians(t)),1800*math.sin(math.radians(t))) for t in range(360)]
        for _ in range(4000):
            r=1800*math.sqrt(rng.random());t=2*math.pi*rng.random();points.append((r*math.cos(t),r*math.sin(t)))
        self.assertLessEqual(max(min(math.dist(q,p) for p in route) for q in points),1000+1e-8)
    def test_rotated_cover(self):
        route=sites(rotation_deg=17.3)
        for t in range(720):
            q=(1800*math.cos(math.pi*t/360),1800*math.sin(math.pi*t/360))
            self.assertLessEqual(min(math.dist(q,p) for p in route),1000)
    def test_finite_grid_contains_all_first_wedge_sources(self):
        observation={'x':300,'y':-200,'svd_deg':359.5};grid=clearance_grid(observation)
        self.assertLessEqual(len(grid),108)
        for r in [5,20,500,1000,1500]:
            for error in [-1,0,1]:
                t=math.radians(359.5+error);g=(300+r*math.cos(t),-200+r*math.sin(t))
                self.assertLess(min(math.dist(g,q) for q in grid),20)
    def test_grid_step_rejected(self):
        with self.assertRaises(ValueError): clearance_grid({'x':0,'y':0,'svd_deg':0},cell_size=30)


class StateTests(unittest.TestCase):
    def test_ten_does_not_finish_discovery_sixteen_does(self):
        m=ChannelManager()
        for c in range(1,11): m.register(c,0,{'measure_result':'near'},(0,0))
        self.assertFalse(m.discovery_complete())
        for c in range(11,17): m.register(c,0,{'measure_result':'near'},(0,0))
        self.assertTrue(m.discovery_complete())
        self.assertEqual(m.evidence()['20']['state'],'EXCLUDED_BY_COUNT_BOUND')
    def test_exact_site_evidence_not_count(self):
        m=ChannelManager()
        for _ in range(10): m.register(1,0,{'measure_result':'no_signal'},(0,0))
        self.assertEqual(m.evidence()['1']['state'],'UNKNOWN')
        for site in range(1,7): m.register(1,site,{'measure_result':'no_signal'},(0,0))
        self.assertEqual(m.evidence()['1']['state'],'ABSENT_CERT')
    def test_clear_does_not_remove_discovery_evidence(self):
        m=ChannelManager()
        t=m.register(1,0,{'measure_result':'near'},(0,0));t.cleared=True
        self.assertNotIn(1,m.discovery_channels());self.assertEqual(len(m.tracks),1)
    def test_b1_all_observations_and_no_signal_excluded(self):
        t=SourceTrack(1)
        t.observe((-100,-100),{'measure_result':'direction','svd_deg':45})
        t.observe((100,-100),{'measure_result':'direction','svd_deg':135})
        t.observe((0,200),{'measure_result':'no_signal'})
        self.assertEqual(len(t.observations),2)
        self.assertIsNotNone(t.clear_point((0,0)))
        t.observe((0,100),{'measure_result':'direction','svd_deg':270})
        self.assertEqual(len(t.observations),3)
    def test_current_point_clearance_and_near(self):
        t=SourceTrack(1);t.observe((8,9),{'measure_result':'near'})
        self.assertEqual(t.clear_point((8,9)),(8,9))
    def test_channel_order_and_detour(self):
        self.assertEqual(channel_order([2,3,5],3),[3,2,5])
        self.assertEqual(insertion_distance((0,0),(5,0),(10,0)),0)


class ProtocolTests(unittest.TestCase):
    def test_real_local_http_transport(self):
        from http.server import BaseHTTPRequestHandler,HTTPServer
        import threading
        from protocol_adapter import HTTPTransport
        env=OfflineTransport({'sources':[]})
        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                self.server.content_type=self.headers.get('Content-Type')
                request=json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                code,body=env(self.path,request)
                encoded=json.dumps(body).encode('utf-8')
                self.send_response(code);self.send_header('Content-Type','application/json')
                self.send_header('Content-Length',str(len(encoded)));self.end_headers();self.wfile.write(encoded)
            def log_message(self,*args): pass
        server=HTTPServer(('127.0.0.1',0),Handler)
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        try:
            c=Client(HTTPTransport(f'http://127.0.0.1:{server.server_port}'),'offline-test')
            c.enter();self.assertEqual(c.measure((300,400),1)['virtual_time_s'],105);c.exit()
            self.assertEqual(server.content_type,'application/json')
        finally: server.shutdown();server.server_close();thread.join(timeout=2)

    def test_official_timing_example(self):
        env=OfflineTransport({'sources':[]});c=Client(env,'offline-test');c.enter()
        self.assertEqual(c.measure((300,400),1)['virtual_time_s'],105)
        self.assertEqual(c.measure((300,400),2)['virtual_time_s'],111)
        self.assertEqual(c.clear((300,0),3)['virtual_time_s'],194)
        self.assertEqual(c.channel,2)
        self.assertEqual(c.measure((300,0),2)['virtual_time_s'],199)
        self.assertEqual(c.exit()['exit_reason'],'user_exit')
    def test_accepted_false_keeps_state(self):
        env=OfflineTransport({'sources':[]});c=Client(env,'offline-test');c.enter();c.measure((300,0),2)
        c.transport=lambda p,b:(200,{'accepted':False,'real_timestamp_ms':0,'virtual_time_s':0})
        with self.assertRaises(ProtocolError): c.measure((500,0),3)
        self.assertEqual(c.position,(300,0));self.assertEqual(c.channel,2);self.assertEqual(c.virtual_time_s,66)
    def test_lost_response_retries_same_id_once_execution(self):
        env=OfflineTransport({'sources':[]});records=[];lost=[False]
        def transport(path,payload):
            records.append((path,json.dumps(payload,sort_keys=True)))
            response=env(path,payload)
            if path=='/measure' and not lost[0]: lost[0]=True;raise TimeoutError('response lost after execution')
            return response
        c=Client(transport,'offline-test');c.enter();c.measure((300,400),1)
        self.assertEqual(c.virtual_time_s,105);self.assertEqual(c.stats['measurements'],1)
        self.assertEqual(records[-1],records[-2])
    def test_http_error_and_missing_fields_abort(self):
        c=Client(lambda p,b:(409,{'accepted':False}),'x')
        with self.assertRaises(ProtocolError): c.enter()
        c=Client(lambda p,b:(200,{'accepted':True}),'x')
        with self.assertRaises(ProtocolError): c.enter()
    def test_enter_remaining_budget_not_hardcoded(self):
        response={'accepted':True,'virtual_time_s':0,'real_timestamp_ms':0,'max_virtual_duration_s':360000,
                  'max_real_duration_s':1200,'remaining_real_duration_s':10}
        now=[100.];c=Client(lambda p,b:(200,response),'x',clock=lambda:now[0]);c.enter()
        self.assertEqual(c.deadline,110);now[0]=109
        with self.assertRaises(BudgetExceeded): c.measure((0,0),1)
    def test_no_new_action_after_retries_exhausted(self):
        calls=[]
        def transport(path,payload): calls.append(payload['request_id']);raise TimeoutError()
        c=Client(transport,'x',retries=2)
        with self.assertRaises(ProtocolError): c.enter()
        self.assertEqual(len(calls),3);self.assertEqual(len(set(calls)),1)
    def test_repeat_location_fixed_quantized_error(self):
        env=OfflineTransport(make_case(3));c=Client(env,'offline-test');c.enter()
        s=next(iter(env._sources.values()));p=(s['x']+100,s['y'])
        a=c.measure(p,s['channel']);b=c.measure(p,s['channel'])
        self.assertEqual(a['svd_deg'],b['svd_deg'])
        self.assertLessEqual(abs((a['svd_deg']-180+180)%360-180),1)


class IntegrationTests(unittest.TestCase):
    def test_all_near_sixteen_stops_at_origin(self):
        case={'seed':0,'sources':[{'channel':i,'x':0,'y':0,'radius':1000} for i in range(1,17)]}
        r=run_offline(case,'C')
        self.assertEqual(r['status'],'completed');self.assertEqual(r['clear_fraction'],1)
        self.assertEqual(r['metrics']['movement_m'],0);self.assertEqual(r['metrics']['measurements'],16)
    def test_three_strategies_same_boundary_case(self):
        case=make_case(7,pattern='boundary')
        for strategy in 'ABC':
            r=run_offline(case,strategy,error_mode='endpoints')
            self.assertEqual(r['status'],'completed',r.get('error'))
            self.assertEqual(r['clear_fraction'],1)
            self.assertLess(r['accounting_check']['difference_s'],1e-4)
            self.assertLessEqual(r['metrics']['b2_calls'],len(case['sources']))
    def test_no_fake_success_on_action_budget(self):
        r=run_offline(make_case(1),'C',config={'max_actions':1})
        self.assertEqual(r['status'],'incomplete')
        self.assertLess(r['clear_fraction'],1)


if __name__=='__main__': unittest.main()
