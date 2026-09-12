"""Reproduce B2 placement-mesh convergence diagnostics for representative synthetic cases."""
import json
import sys
from pathlib import Path

HERE=Path(__file__).resolve().parent
if str(HERE) not in sys.path: sys.path.insert(0,str(HERE))
from b2_solver import solve_b2

CASES={
    'central_synthetic':{'x':0,'y':0,'svd_deg':0},
    'edge_synthetic':{'x':1500,'y':0,'svd_deg':0},
    'wrap_synthetic':{'x':0,'y':0,'svd_deg':359.5},
}


def compact(name,result):
    rec=result.get('recommended') or {}
    wd=rec.get('worst_diameter') or {}
    return {
        'case':name,
        'status':result['status'],
        'recommended_local_m':rec.get('point_local_m'),
        'worst_diameter_lower_m':wd.get('sample_lower_m'),
        'worst_diameter_upper_m':wd.get('envelope_upper_m'),
        'outer_convergence':result.get('outer_convergence'),
        'inner_convergence':result.get('inner_convergence'),
        'evaluated_candidates':result.get('search',{}).get('evaluated_candidates'),
    }


def main():
    payload=[]
    for name,obs in CASES.items():
        r=solve_b2(obs)
        row=compact(name,r);payload.append(row)
        print(name, row['status'], row['recommended_local_m'],
              f"D_upper={row['worst_diameter_upper_m']:.6f}",
              'outer='+row['outer_convergence']['status'],
              'inner='+row['inner_convergence']['status'])
        for stage in row['outer_convergence'].get('stages',[]):
            print(' ',stage)
    if len(sys.argv)>1:
        p=Path(sys.argv[1]);p.parent.mkdir(parents=True,exist_ok=True)
        p.write_text(json.dumps(payload,ensure_ascii=False,indent=2),encoding='utf-8')

if __name__=='__main__': main()
