"""Explicit operator-started formal run; reuses the unchanged E13-R strategy."""
import argparse
import hashlib
import json
from pathlib import Path

from run_e13_rehearsal import ROOT, Client, HTTPTransport, provenance, run_with_client


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-id',required=True)
    parser.add_argument('--formal-ready',action='store_true')
    parser.add_argument('--url',default='http://127.0.0.1:2026')
    parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args()
    if not args.formal_ready:
        parser.error('requires --formal-ready after selecting formal test in the simulator')
    root=(ROOT/'B3/结果/e13_formal').resolve()
    out=(ROOT/args.output_dir).resolve()
    if out==root or not out.is_relative_to(root) or out.parts[len(root.parts)]=='summary':
        parser.error('output must be a new run directory inside B3/结果/e13_formal (not summary)')
    out.mkdir(parents=True,exist_ok=False)
    (out/'simulator_logs').mkdir()
    evidence=provenance()
    evidence['formal_runner_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    (out/'provenance.json').write_text(json.dumps(evidence,ensure_ascii=False,indent=2),encoding='utf-8')
    try:
        result=run_with_client(Client(HTTPTransport(args.url),args.robot_id,out/'actions.jsonl'),
                               'operator_declared_official_formal')
    except Exception as exc:
        result={'status':'error','error':f'{type(exc).__name__}: {exc}'}
    result.update(strategy='E13-R',data_kind='operator_declared_official_formal',
                  official_formal=True,official_rehearsal=False,true_source_count=None,
                  clear_fraction=None,clear_fraction_note='正式案例总源数未通过接口公开，不计算清除比例。',provenance=evidence)
    (out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False,indent=2))
    return 0 if result['status']=='completed' else 2


if __name__=='__main__':raise SystemExit(main())
