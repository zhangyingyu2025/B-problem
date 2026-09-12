"""One-command B1 verification and frozen synthetic artifacts.

Run from any working directory. Never calls network/simulator or installs packages.
"""
from __future__ import annotations
import hashlib
import io
import json
import math
from pathlib import Path
import platform
import subprocess
import sys
import time
import unittest

from run_b1 import main as run_case

BASE = Path(__file__).resolve().parents[1]
ROOT = BASE.parents[1]
SEED = 20260910


def write_json(path, obj):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(obj,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')


def source_integrity():
    manifest=json.loads((BASE/'input_manifest.json').read_text(encoding='utf-8'))
    rows=[]
    for item in manifest['files']:
        path=ROOT/item['path']
        actual=hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None
        rows.append({'path':item['path'],'unchanged':actual==item['sha256']})
    return rows


def main():
    results=BASE/'结果'
    results.mkdir(parents=True,exist_ok=True)
    before=source_integrity()
    if not all(r['unchanged'] for r in before):
        print('Protected input hash mismatch; aborting.',file=sys.stderr)
        return 1
    stream=io.StringIO()
    suite=unittest.defaultTestLoader.discover(str(BASE/'tests'))
    started=time.perf_counter()
    test_result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
    elapsed=time.perf_counter()-started
    (results/'test_log.txt').write_text(stream.getvalue(),encoding='utf-8')
    print(stream.getvalue())
    validation={'seed':SEED,'independent_oracle_seed':20260911,'tests_run':test_result.testsRun,
                'failures':len(test_result.failures),'errors':len(test_result.errors),
                'elapsed_seconds':elapsed,'python':sys.version,'platform':platform.platform(),
                'success':test_result.wasSuccessful(),
                'oracle_random_systems':160,'synthetic_random_targets':30,
                'target_observation_counts':[3,4,5], 'case_results':[]}
    if not test_result.wasSuccessful():
        write_json(results/'validation_results.json',validation)
        return 1
    cases=sorted((BASE/'examples').glob('*.json'))
    frozen={'data_kind':'synthetic','seed':SEED,'cases':{}}
    for case in cases:
        out=results/(case.stem+'.json')
        svg=BASE/'图片'/(case.stem+'.svg')
        args=[str(case),'--output',str(out),'--svg',str(svg)]
        exit_code=run_case(args)
        first,first_svg=out.read_bytes(),svg.read_bytes()
        if exit_code!=0 or run_case(args)!=0 or first!=out.read_bytes() or first_svg!=svg.read_bytes():
            raise RuntimeError('Case failed or non-deterministic output: '+case.name)
        r=json.loads(first)
        validation['case_results'].append({'case_id':r['case_id'],'status':r['status'],'deterministic_json_and_svg':True})
        frozen['cases'][r['case_id']]={k:r[k] for k in ('status','input_sha256','diameter_m','farthest_pair',
                                                          'diameter_circle','minimum_enclosing_circle','circle_crosscheck_passed')}
    after=source_integrity()
    validation['protected_inputs']=after
    validation['protected_inputs_unchanged']=all(r['unchanged'] for r in after)
    if not validation['protected_inputs_unchanged']:
        raise RuntimeError('Protected inputs changed during run')
    write_json(results/'validation_results.json',validation)
    write_json(results/'frozen_numbers.json',frozen)
    artifact_rows=[]
    for path in sorted(BASE.rglob('*')):
        if path.is_file() and '__pycache__' not in path.parts and path.name!='artifact_manifest.json':
            artifact_rows.append({'path':path.relative_to(ROOT).as_posix(),
                                  'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
    write_json(results/'artifact_manifest.json',{'files':artifact_rows,'runtime':sys.version,'dependency_policy':'Python standard library only'})
    print('B1 verified. Frozen synthetic results:',results/'frozen_numbers.json')
    return 0


if __name__=='__main__':
    raise SystemExit(main())
