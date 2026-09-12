"""Verify the actual B4-start sources plus unchanged R action traces.

The old E12 manifest predates jty's approved B2 fix 62f628d. It remains
unchanged; this check does not pretend that its old B2 hash check passes.
"""
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'B3/tests'))
from test_e13_r_equivalence import compare_case


def verify_sources():
    state=json.loads((ROOT/'B4/project_state.json').read_text(encoding='utf-8'))
    changed=[n for n,h in state['protected_python_sha256_lf'].items()
             if hashlib.sha256((ROOT/n).read_bytes().replace(b'\r\n',b'\n')).hexdigest()!=h]
    if changed: raise RuntimeError('sources changed since B4 start: '+repr(changed))
    return len(state['protected_python_sha256_lf'])


if __name__=='__main__':
    count=verify_sources()
    with ProcessPoolExecutor(max_workers=4) as pool:
        rows=list(pool.map(compare_case,range(52001500,52001600)))
    failed=[r for r in rows if not(r['events_equal'] and r['metrics_equal'] and r['all_cleared'])]
    verify_sources()
    report={'source_files':count,'cases':len(rows),'failed':failed,
            'basis':'B4-start source manifest plus original frozen R 100-case actions and metrics',
            'legacy_check':'old E12 manifest rejects prior B2/code/b2_solver.py update 62f628d; not modified'}
    out=ROOT/'B4/results/previous_regression.json';out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2));assert not failed
