import hashlib, json, sys, unittest
from pathlib import Path

B4ROOT=Path(__file__).resolve().parents[1]
REPOROOT=B4ROOT.parent
if str(REPOROOT) not in sys.path:
    sys.path.insert(0,str(REPOROOT))

from B4.code.run_offline import run_case, canonical_case, _case_sha256
from B4.code.discovery_convex_certificate import mesh21, verify_local_convex_certificate


def _norm_sha(path):
    text=path.read_text(encoding='utf-8').replace('\r\n','\n').replace('\r','\n')
    return hashlib.sha256(text.encode('utf-8')).hexdigest()


class FinalB4Tests(unittest.TestCase):
    def test_source_manifest(self):
        manifest=json.loads((B4ROOT/'source_manifest.json').read_text(encoding='utf-8'))
        for rel, expected in manifest['sha256_lf'].items():
            path=B4ROOT/rel
            self.assertTrue(path.is_file(),f'missing source file: {rel}')
            self.assertEqual(_norm_sha(path),expected,f'source mismatch: {rel}')

    def test_final_certificate(self):
        result=verify_local_convex_certificate(mesh21(1868.0,997.5))
        self.assertTrue(result['valid'])
        self.assertEqual(result['vertices'],21)

    def test_seed_64000000_exact(self):
        case=canonical_case(64000000)
        expected='eb39fc4cae0881e79168f4c8701bbe2f278fe4a479c75d7ad27761a41bd50770'
        self.assertEqual(_case_sha256(case),expected)
        r=run_case(64000000)
        self.assertEqual(r['case_sha256'],expected)
        self.assertTrue(r['full_clear'])
        self.assertAlmostEqual(r['virtual_time_s'],5801.433429,places=9)
        self.assertAlmostEqual(r['metrics']['movement_m'],20597.167130984515,places=9)
        self.assertEqual(r['metrics']['measurements'],269)
        self.assertEqual(r['metrics']['switches'],252)
        self.assertEqual(r['metrics']['clear_failures'],10)

    def test_first_three_match_frozen(self):
        ref=json.loads((B4ROOT/'results/dev50_reference.json').read_text(encoding='utf-8'))['rows'][:3]
        for old in ref:
            r=run_case(old['seed'])
            self.assertTrue(r['full_clear'])
            self.assertAlmostEqual(r['virtual_time_s'],old['vt'],places=9)
            self.assertAlmostEqual(r['metrics']['movement_m'],old['move'],places=9)
            self.assertEqual(r['metrics']['measurements'],old['meas'])
            self.assertEqual(r['metrics']['switches'],old['switch'])
            self.assertEqual(r['metrics']['clear_failures'],old['cf'])

if __name__=='__main__': unittest.main()
