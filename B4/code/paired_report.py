"""Auditable paired performance tables; case hashes and error modes must match."""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics

from run_offline import summarize, quantile


def load(paths):
    rows={}
    for path in paths:
        for row in json.loads(Path(path).read_text(encoding='utf-8'))['runs']:
            if row['seed'] in rows: raise ValueError('overlapping batch seeds')
            rows[row['seed']]=row
    return rows


def tails(rows):
    xs=[r['s_per_source'] for r in rows]
    full=all(r['full_clear'] for r in rows)
    return {'runs':len(rows),'full_clear':full,
            'weighted':sum(r['metrics']['virtual_time_s'] for r in rows)/sum(r['N'] for r in rows) if full else None,
            'median':statistics.median(xs) if full else None,
            'P95':quantile(xs,.95) if full else None,
            'CVaR95':statistics.mean(sorted(xs)[-max(1,math.ceil(len(xs)*.05)):]) if full else None,
            'max':max(xs) if full else None}


def compare(base,candidate):
    if base.keys()!=candidate.keys(): raise ValueError('unpaired seed sets')
    pairs=[]
    for seed,b in sorted(base.items()):
        c=candidate[seed]
        for k in ('case_sha256','error_mode','N','pattern'):
            if b[k]!=c[k]: raise ValueError(f'{seed}: mismatched {k}')
        pairs.append({'seed':seed,'N':b['N'],'base_variant':b['variant'],'candidate_variant':c['variant'],
                      'case_sha256':b['case_sha256'],'error_mode':b['error_mode'],
                      'base_full_clear':b['full_clear'],'candidate_full_clear':c['full_clear'],
                      'base_s_per_source':b['s_per_source'],'candidate_s_per_source':c['s_per_source'],
                      'delta_s_per_source':c['s_per_source']-b['s_per_source'],
                      'delta_time_s':c['metrics']['virtual_time_s']-b['metrics']['virtual_time_s'],
                      **{f'delta_{k}_s':c['cost_s'][k]-b['cost_s'][k] for k in b['cost_s']}})
    def group(ps):
        ds=[p['delta_time_s'] for p in ps]
        return {'base':tails([base[p['seed']] for p in ps]),'candidate':tails([candidate[p['seed']] for p in ps]),
                'win_rate':sum(d<0 for d in ds)/len(ds),'median_delta_time_s':statistics.median(ds),
                'P95_delta_time_s':quantile(ds,.95),'worst_delta_time_s':max(ds)}
    return {'summary':group(pairs),'base_summary':summarize(list(base.values())),
            'candidate_summary':summarize(list(candidate.values())),
            'by_N':{str(n):group([p for p in pairs if p['N']==n]) for n in sorted({p['N'] for p in pairs})},
            'worst10':sorted(pairs,key=lambda p:p['delta_time_s'],reverse=True)[:10],'pairs':pairs}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--base',nargs='+',type=Path,required=True)
    p.add_argument('--candidate',nargs='+',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists() or a.output.with_suffix('.csv').exists(): p.error('refusing overwrite')
    result=compare(load(a.base),load(a.candidate))
    a.output.write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    with a.output.with_suffix('.csv').open('w',newline='',encoding='utf-8-sig') as f:
        w=csv.DictWriter(f,fieldnames=list(result['pairs'][0]));w.writeheader();w.writerows(result['pairs'])
    print(json.dumps(result['summary'],indent=2))


if __name__=='__main__':main()
