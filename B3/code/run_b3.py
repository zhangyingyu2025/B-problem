"""Offline runner and explicitly operator-enabled official rehearsal runner."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
from b3_solver import Solver
from offline_environment import OfflineTransport,make_case
from protocol_adapter import Client,HTTPTransport

BASE=Path(__file__).resolve().parents[1]
ROOT=BASE.parent


def provenance():
    try: commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
    except (OSError,subprocess.CalledProcessError): commit=None
    paths=sorted((BASE/'code').glob('*.py'))+[ROOT/'B1/code/b1_geometry.py',ROOT/'B2/code/b2_solver.py',ROOT/'B2/code/domains.py',ROOT/'B2/code/geometry.py']
    return {'git_head':commit,'code_sha256_lf':{p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes().replace(b'\r\n',b'\n')).hexdigest() for p in paths}}


def run_offline(case,strategy='C',log_path=None,error_mode='fixed_field',config=None):
    environment=OfflineTransport(case,error_mode=error_mode)
    client=Client(environment,'offline-test',log_path,session_id=f'offline-{case.get("seed",0)}-{strategy}')
    result=Solver(client,strategy,**(config or {})).run()
    result.update(data_kind='synthetic_offline',case_seed=case.get('seed'),pattern=case.get('pattern'),error_mode=error_mode,
                  true_source_count=len(case['sources']),clear_fraction=len(environment.cleared)/len(case['sources']),
                  official_rehearsal=False)
    result['accounting_check']={'components_sum_s':client.stats['movement_m']/5+5*client.stats['measurements']+client.stats['switches']+
                               5*client.stats['clear_successes']+3*client.stats['clear_failures']}
    result['accounting_check']['difference_s']=abs(result['accounting_check']['components_sum_s']-client.virtual_time_s)
    return result


def main():
    parser=argparse.ArgumentParser(description='B3 offline verification or operator-confirmed rehearsal; no formal-test mode')
    parser.add_argument('--mode',choices=['offline','rehearsal'],default='offline')
    parser.add_argument('--strategy',choices=list('ABC'),default='C')
    parser.add_argument('--seed',type=int,default=20260911)
    parser.add_argument('--case',type=Path)
    parser.add_argument('--robot-id')
    parser.add_argument('--url',default='http://127.0.0.1:2026')
    parser.add_argument('--rehearsal-ready',action='store_true',help='operator confirms simulator is in rehearsal mode with interface ready')
    parser.add_argument('--output-dir',type=Path,default=BASE/'结果/manual_run')
    args=parser.parse_args()
    if args.mode=='rehearsal' and (not args.robot_id or not args.rehearsal_ready):
        parser.error('rehearsal requires --robot-id and --rehearsal-ready; HTTP cannot determine formal/rehearsal mode')
    args.output_dir.mkdir(parents=True,exist_ok=True)
    trace=args.output_dir/'actions.jsonl'
    if trace.exists(): parser.error('output already has actions.jsonl; choose a new run directory to preserve logs')
    if args.mode=='offline':
        case=json.loads(args.case.read_text(encoding='utf-8-sig')) if args.case else make_case(args.seed)
        (args.output_dir/'synthetic_case.json').write_text(json.dumps(case,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        result=run_offline(case,args.strategy,trace)
    else:
        client=Client(HTTPTransport(args.url),args.robot_id,trace)
        result=Solver(client,args.strategy).run()
        result.update(data_kind='operator_declared_official_rehearsal',official_rehearsal=True,
                      clear_fraction=None,clear_fraction_note='official true source count must be supplied after rehearsal ends')
    result['provenance']=provenance()
    (args.output_dir/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps({k:result.get(k) for k in ['status','strategy','data_kind','cleared_count','clear_fraction','error']},ensure_ascii=False))
    if result['status']!='completed': raise SystemExit(2)


if __name__=='__main__': main()
