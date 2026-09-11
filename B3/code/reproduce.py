"""Fixed offline comparison, regression, hashes. Never starts an official test."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
from pathlib import Path
import statistics
import subprocess
import sys
import time
from coverage import certificate
from offline_environment import make_case
from run_b3 import run_offline,provenance

BASE=Path(__file__).resolve().parents[1];ROOT=BASE.parent;OUT=BASE/'结果'


def dump(path,data):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')


def verify():
    manifest=json.loads((BASE/'dependency_manifest.json').read_text(encoding='utf-8'))
    for item in manifest['files']:
        digest=hashlib.sha256((ROOT/item['path']).read_bytes().replace(b'\r\n',b'\n')).hexdigest()
        assert digest==item['sha256_lf'],item['path']
    return len(manifest['files'])


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--verify-only',action='store_true')
    parser.add_argument('--skip-tests',action='store_true',help='use only after unchanged regression tests have passed')
    args=parser.parse_args();OUT.mkdir(parents=True,exist_ok=True)
    dependencies=verify()
    if args.verify_only:
        manifest=json.loads((OUT/'artifact_manifest.json').read_text(encoding='utf-8'))
        for item in manifest['files']:
            assert hashlib.sha256((ROOT/item['path']).read_bytes().replace(b'\r\n',b'\n')).hexdigest()==item['sha256_lf'],item['path']
        print('PASS: B1/B2 dependencies and B3 artifact hashes');return
    if not args.skip_tests:
        logs=[]
        for name in ['B1','B2','B3']:
            proc=subprocess.run([sys.executable,'-X','utf8','-m','unittest','discover','-s',str(ROOT/name/'tests'),'-v'],
                                capture_output=True,text=True,encoding='utf-8',cwd=ROOT)
            logs.append(name+'\n'+proc.stdout+proc.stderr)
            (OUT/'test_log.txt').write_text('\n'.join(logs),encoding='utf-8')
            if proc.returncode: raise RuntimeError(name+' regression failed')
            print(name+' regression passed',flush=True)
    cases=[make_case(20260911+i) for i in range(8)]
    cases += [make_case(20261001,pattern='boundary'),make_case(20261002,pattern='boundary'),
              make_case(20261003,pattern='cluster'),make_case(20261004,pattern='origin')]
    dump(BASE/'examples/comparison_cases.json',{'data_kind':'synthetic_offline','cases':cases})
    batch=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S')
    rows=[];start=time.perf_counter();version=provenance()
    for index,case in enumerate(cases):
        for strategy in 'ABC':
            if time.perf_counter()-start>600: raise RuntimeError('offline comparison time budget reached; partial rows saved')
            log=OUT/'offline_logs'/batch/f'{case["seed"]}_{strategy}.jsonl'
            mode='endpoints' if case['pattern']=='boundary' else 'fixed_field'
            result=run_offline(case,strategy,log,error_mode=mode)
            result['action_log']=log.relative_to(ROOT).as_posix();rows.append(result)
            dump(OUT/'comparison.json',{'data_kind':'synthetic_offline','official_rehearsal':False,'provenance':version,'runs':rows})
            print(f'{index+1:02d}/12 {strategy}: {result["status"]}, cleared={result["cleared_count"]}/{result["true_source_count"]}, virtual={result["metrics"]["total_virtual_time_s"]:.3f}s',flush=True)
            if result['status']!='completed' or result['clear_fraction']!=1 or result['accounting_check']['difference_s']>1e-4:
                raise RuntimeError('offline failure retained; stop before freezing')
    summary={}
    for strategy in 'ABC':
        group=[r for r in rows if r['strategy']==strategy]
        summary[strategy]={'runs':len(group),'all_cleared':all(r['clear_fraction']==1 for r in group),
            'mean_total_virtual_s':statistics.mean(r['metrics']['total_virtual_time_s'] for r in group),
            'mean_per_source_s':statistics.mean(r['metrics']['mean_time_per_clear_s'] for r in group),
            'mean_movement_m':statistics.mean(r['metrics']['movement_m'] for r in group),
            'mean_measurements':statistics.mean(r['metrics']['measurements'] for r in group),
            'mean_switches':statistics.mean(r['metrics']['switches'] for r in group),
            'mean_b2_calls':statistics.mean(r['metrics']['b2_calls'] for r in group),
            'mean_onroute_observations':statistics.mean(r['metrics']['onroute_observations'] for r in group),
            'max_wall_s':max(r['metrics']['wall_time_s'] for r in group),
            'guaranteed_clear_failures':sum(r['metrics']['guaranteed_clear_failures'] for r in group),
            'grid_clear_attempts':sum(r['metrics']['grid_clear_attempts'] for r in group)}
    winner=min(summary,key=lambda k:summary[k]['mean_total_virtual_s'])
    dump(OUT/'frozen_numbers.json',{'scope':'offline_candidate_only_not_official_freeze','summary':summary,
        'lowest_offline_mean_strategy':winner,'coverage':certificate(),'official_rehearsal_count':0,
        'formal_test_count':0,'dependency_files_verified':dependencies,'provenance':version})
    verify()
    paths=[p for p in BASE.rglob('*') if p.is_file() and '__pycache__' not in p.parts and not {'offline_logs','rehearsal_logs','formal_logs'}.intersection(p.parts)
           and p.name!='artifact_manifest.json' and p.suffix!='.png']
    dump(OUT/'artifact_manifest.json',{'files':[{'path':p.relative_to(ROOT).as_posix(),
         'sha256_lf':hashlib.sha256(p.read_bytes().replace(b'\r\n',b'\n')).hexdigest()} for p in sorted(paths)]})
    print(json.dumps({'status':'offline_passed','summary':summary},ensure_ascii=False),flush=True)


if __name__=='__main__': main()
