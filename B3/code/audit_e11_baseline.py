"""Read-only audit of local E11 evidence; never contacts the simulator."""
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'B3/结果/e12_phase_a'


def digest(path, normalized=False):
    data = path.read_bytes()
    return hashlib.sha256(data.replace(b'\r\n', b'\n') if normalized else data).hexdigest()


def audit():
    rows = []
    inventory = []
    for folder in sorted((ROOT / 'B3/结果/rehearsal_logs').iterdir()):
        if not folder.is_dir():
            continue
        rp, lp = folder/'result.json', folder/'actions.jsonl'
        result = json.loads(rp.read_text(encoding='utf-8')) if rp.exists() else {}
        if 'E11' not in folder.name and result.get('strategy') != 'E11':
            continue
        for p in (rp, lp):
            if p.exists():
                inventory.append({'path': p.relative_to(ROOT).as_posix(), 'sha256': digest(p)})
        log = [json.loads(x) for x in lp.read_text(encoding='utf-8').splitlines()] if lp.exists() else []
        actions = [x for x in log if x.get('kind') == 'action']
        accepted = [x for x in actions if x.get('response', {}).get('accepted') is True]
        enters = [x for x in accepted if x['path'] == '/enter']
        work = [x for x in actions if x['path'] in ('/measure', '/clear')]
        row = {'run_id': folder.name, 'true_source_count': result.get('true_source_count'),
               'classification': 'incomplete_or_unverified'}
        if not enters and not work and log and all(x.get('kind') == 'transport_failure' and x.get('path') == '/enter' for x in log):
            row.update(classification='enter_transport_failure', failures=len(log))
            rows.append(row)
            continue
        if not result:
            rows.append(row)
            continue
        m = result['metrics']
        measured = [x for x in accepted if x['path'] == '/measure']
        cleared = [x for x in accepted if x['path'] == '/clear']
        discovered = {x['payload']['channel'] for x in measured if x['response']['measure_result'] in ('near', 'direction')}
        successes = {x['payload']['channel'] for x in cleared if x['response']['clear_result'] == 'success'}
        movement = sum(math.dist(x['before']['position'], x['after']['position']) for x in accepted)
        switches = sum(x['before']['channel'] != x['after']['channel'] for x in measured)
        failures = sum(x['response']['clear_result'] != 'success' for x in cleared)
        time_sum = movement/5 + 5*len(measured) + switches + 5*(len(cleared)-failures) + 3*failures
        k0 = set()
        for x in work:
            p = x['payload']['position']
            if (p['x'], p['y']) != (0, 0):
                break
            if x['path'] == '/measure' and x['response']['measure_result'] in ('near', 'direction'):
                k0.add(x['payload']['channel'])
        rotation = result['rotation_deg']
        sites = [(0., 0.)]+[(1130*math.cos(math.radians(rotation+60*k)), 1130*math.sin(math.radians(rotation+60*k))) for k in range(6)]
        absent = {}
        for ch in set(range(1, 21))-discovered:
            pts = [x['payload']['position'] for x in measured if x['payload']['channel'] == ch and x['response']['measure_result'] == 'no_signal']
            absent[str(ch)] = [i for i, p in enumerate(sites) if any(math.dist(p, (q['x'], q['y'])) < 1e-7 for q in pts)]
        hashes = result.get('provenance', {}).get('code_sha256_lf', {})
        checks = {
            'enter_exit': bool(enters) and bool(accepted) and accepted[-1]['path'] == '/exit',
            'all_actions_accepted': len(actions) == len(accepted),
            'completed': result.get('status') == 'completed' and result.get('error') is None,
            'discovery_complete': result.get('discovery_complete') is True,
            'no_uncleared': result.get('uncleared_channels') == [],
            'channels_match': discovered == successes == set(result['discovered_channels']) == set(result['cleared_channels']),
            'counts_match': len(discovered) == result['discovered_count'] == result['cleared_count'],
            'coverage_evidence': len(discovered) == 16 or all(len(ids) == 7 for ids in absent.values()),
            'accounting': abs(time_sum-m['total_virtual_time_s']) < 1e-4,
            'metrics': abs(movement-m['movement_m']) < 1e-7 and switches == m['switches'] and len(measured) == m['measurements'] and len(cleared) == m['clear_attempts'] and failures == m['clear_failures'],
            'source_hashes_match': bool(hashes) and all((ROOT/p).exists() and digest(ROOT/p, True) == h for p, h in hashes.items()),
            'true_count_if_available': row['true_source_count'] is None or row['true_source_count'] == len(successes),
        }
        row.update(classification='valid' if all(checks.values()) else 'audit_failed', checks=checks,
                   discovered_count=len(discovered), cleared_count=len(successes), k0=len(k0), rotation_deg=rotation,
                   metrics=m, accounting_difference_s=abs(time_sum-m['total_virtual_time_s']), absent_site_evidence=absent)
        rows.append(row)
    valid = [r for r in rows if r['classification'] == 'valid']
    historical = {}
    for name in ('e11_fixed12.json', 'e11_unseen80.json'):
        p = ROOT/'B3/结果'/name
        r = json.loads(p.read_text(encoding='utf-8'))
        actual = sum(x['vt'] for x in r['runs'])/sum(x['n'] for x in r['runs'])
        assert abs(actual-r['summary']['weighted_time_per_source_s']) < 1e-9
        assert all(x['cleared'] == x['n'] for x in r['runs'])
        historical[name] = {'summary': r['summary'], 'sha256': digest(p), 'recomputed_from_saved_rows': True, 'rerun': False}
    return {'scope': 'local_e11_baseline_audit', 'runs': rows, 'raw_files': inventory, 'historical': historical,
            'summary': {'starts': len(rows), 'valid_runs': len(valid),
                        'enter_failures': sum(r['classification'] == 'enter_transport_failure' for r in rows),
                        'discovered': sum(r['discovered_count'] for r in valid),
                        'cleared': sum(r['cleared_count'] for r in valid),
                        'weighted_time_per_discovered_source_s': sum(r['metrics']['total_virtual_time_s'] for r in valid)/sum(r['discovered_count'] for r in valid) if valid else None,
                        'true_counts_verified': sum(r['true_source_count'] is not None for r in valid)}}


if __name__ == '__main__':
    OUT.mkdir(parents=True, exist_ok=True)
    data = audit()
    (OUT/'baseline_audit.json').write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(data['summary'], ensure_ascii=False))
    if any(r['classification'] == 'audit_failed' for r in data['runs']):
        raise SystemExit(1)
