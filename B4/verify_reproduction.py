"""Compare the consolidated solver against the frozen best DEV50 metrics."""
import argparse, json, math, sys
from pathlib import Path
ROOT=Path(__file__).resolve().parent
REPOROOT=ROOT.parent
if str(REPOROOT) not in sys.path:
    sys.path.insert(0,str(REPOROOT))
from B4.code.run_offline import run_case

FIELDS=(
    ('virtual_time_s','vt'),
)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--count',type=int,default=5,choices=range(1,51))
    args=ap.parse_args()
    ref=json.loads((ROOT/'results/dev50_reference.json').read_text(encoding='utf-8'))
    expected={r['seed']:r for r in ref['rows'][:args.count]}
    mismatches=[]; rows=[]
    for seed in sorted(expected):
        new=run_case(seed); old=expected[seed]; rows.append(new)
        checks={
            'full_clear':(new['full_clear'],old['full']),
            'virtual_time_s':(new['virtual_time_s'],old['vt']),
            'movement_m':(new['metrics']['movement_m'],old['move']),
            'measurements':(new['metrics']['measurements'],old['meas']),
            'switches':(new['metrics']['switches'],old['switch']),
            'clear_failures':(new['metrics']['clear_failures'],old['cf']),
        }
        for name,(a,b) in checks.items():
            ok=(a==b) if not isinstance(a,float) else math.isclose(a,b,rel_tol=0,abs_tol=1e-9)
            if not ok:mismatches.append({'seed':seed,'field':name,'new':a,'reference':b})
    sources=sum(r['N'] for r in rows)
    weighted=sum(r['virtual_time_s'] for r in rows)/sources
    out={'runs':len(rows),'full_clear_runs':sum(r['full_clear'] for r in rows),
         'sources':sources,'weighted_s_per_source':weighted,'mismatches':mismatches}
    print(json.dumps(out,ensure_ascii=False,indent=2))
    if mismatches: raise SystemExit(1)

if __name__=='__main__':main()
