"""G41 paired post-hoc audit. Truth counts are used only in this reporter."""
import argparse
from collections import defaultdict
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'B4/code'))
from paired_report import load,compare


def action_costs(path):
    out=defaultdict(lambda:{'movement_m':0.,'movement_s':0.,'measurement_s':0.,'switch_s':0.,'clear_s':0.,'actions':0})
    for line in path.read_text(encoding='utf-8').splitlines():
        e=json.loads(line)
        if e.get('kind')!='action' or e['path'] not in ('/measure','/clear'):continue
        r=out[e.get('decision_reason','')];distance=math.dist(e['before']['position'],e['after']['position'])
        r['movement_m']+=distance;r['movement_s']+=distance/5;r['actions']+=1
        if e['path']=='/measure':
            r['measurement_s']+=5;r['switch_s']+=int(e['before']['channel']!=e['payload']['channel'])
        else:r['clear_s']+=5 if e['response']['clear_result']=='success' else 3
    return dict(out)


def build(base_path,candidate_path):
    b=load([base_path]);c=load([candidate_path]);report=compare(b,c)
    bad=[];allstats={};rows=list(c.values())
    for seed,r in c.items():
        failed={k:v for k,v in r['stats'].items() if ('invariant_failures' in k or 'guaranteed_clear_failures'==k) and v}
        if failed or not r['full_clear']:bad.append({'seed':seed,'error':r['error'],'failures':failed})
    keys=sorted({k for r in rows for k in r['stats'] if k.startswith('jcr_') or k=='planner_wall_time_s'})
    for k in keys:
        vs=[r['stats'].get(k) for r in rows]
        if k=='jcr_realized_movement_saved_m':
            vs=[b[r['seed']]['metrics']['movement_m']-r['metrics']['movement_m'] for r in rows]
        allstats[k]={'sum':sum(vs),'mean':statistics.mean(vs),'max':max(vs)}
    bs,cs=report['base_summary'],report['candidate_summary']
    gate={'all_full_clear_no_invariants':not bad,
          'weighted_improved':cs['weighted_s_per_source'] is not None and cs['weighted_s_per_source']<bs['weighted_s_per_source'],
          'movement_within_one_percent':cs['mean_movement_m']<=bs['mean_movement_m']*1.01,
          'planner_within_30s':allstats['planner_wall_time_s']['max']<=30 and allstats['jcr_planner_budget_exhausted']['sum']==0}
    report.update(g41_statistics=allstats,correctness_failures=bad,stage1_gate=gate,stage1_passed=all(gate.values()),
                  real_saved_note='realized movement is paired G40 minus G41; prediction is upper-bound difference, not causal savings',
                  baseline_file=str(base_path),candidate_file=str(candidate_path))
    for p in report['pairs']:
        seed=p['seed'];p['jcr_realized_movement_saved_m']=b[seed]['metrics']['movement_m']-c[seed]['metrics']['movement_m']
        p['jcr_predicted_eq_m_saved']=c[seed]['stats']['jcr_predicted_eq_m_saved']
        p['jcr_pair_deferrals']=c[seed]['stats']['jcr_pair_deferrals']
        p['jcr_pair_resolutions']=c[seed]['stats']['jcr_pair_resolutions']
    worst=[]
    for p in report['worst10']:
        seed=p['seed'];bc=action_costs(base_path.with_suffix('')/f'{seed}-G40-actions.jsonl')
        cc=action_costs(candidate_path.with_suffix('')/f'{seed}-G41-actions.jsonl')
        worst.append({'seed':seed,'N':p['N'],'delta_time_s':p['delta_time_s'],'base':bc,'candidate':cc})
    report['worst10_action_costs']=worst
    report['inputs_sha256']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (base_path,candidate_path)}
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base',type=Path,required=True);p.add_argument('--candidate',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():p.error('refusing overwrite')
    r=build(a.base,a.candidate)
    a.output.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    with a.output.with_suffix('.csv').open('w',encoding='utf-8-sig',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(r['pairs'][0]));w.writeheader();w.writerows(r['pairs'])
    compact={k:v for k,v in r.items() if k not in ('pairs','worst10_action_costs')}
    compact['raw_paired_report']=a.output.as_posix()
    (ROOT/'B4/g41_validation.json').write_text(json.dumps(compact,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'summary':r['summary'],'gate':r['stage1_gate'],'g41_statistics':r['g41_statistics']},indent=2))
