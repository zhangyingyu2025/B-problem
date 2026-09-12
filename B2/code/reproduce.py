"""One-command B2 regression, synthetic reproduction and artifact verification.

Repository layout expected:
  B1/  verified B1 geometry implementation
  B2/  this directory

No simulator, network access, package installation, or official raw problem files are
required. The official problem/reference hashes remain documented in B1, but GitHub
reproduction validates only the explicit B1 dependency snapshot used by B2.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile

from run_b2 import run_file
from geometry import precise_metrics
from domains import Problem

BASE = Path(__file__).resolve().parents[1]   # B2
ROOT = BASE.parent                           # repository root
OUT = BASE / '结果'
B1 = ROOT / 'B1'


def save(name, value):
    (OUT / name).write_text(
        json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n',
        encoding='utf-8'
    )


TEXT_SUFFIXES = {'.py', '.md', '.json', '.txt', '.csv', '.yaml', '.yml', '.svg'}


def canonical_sha256(path: Path) -> str:
    """Hash text files after normalizing line endings to LF.

    Git/Windows may transparently convert LF <-> CRLF on checkout.  B2 only needs
    semantic identity of its text dependencies, so line-ending changes must not
    trigger a false integrity failure.  Binary files, if ever added, are hashed
    byte-for-byte.
    """
    data = path.read_bytes()
    if path.suffix.lower() in TEXT_SUFFIXES:
        data = data.replace(b'\r\n', b'\n').replace(b'\r', b'\n')
    return hashlib.sha256(data).hexdigest()


def verify_manifest(path: Path) -> int:
    data = json.loads(path.read_text(encoding='utf-8'))
    for item in data['files']:
        target = ROOT / item['path']
        if not target.is_file():
            raise AssertionError('missing manifest file: ' + item['path'])
        actual = canonical_sha256(target)
        if actual != item['sha256']:
            raise AssertionError('hash mismatch: ' + item['path'])
    return len(data['files'])


def run_tests() -> list[str]:
    logs=[]
    for folder in (B1 / 'tests', BASE / 'tests'):
        proc=subprocess.run(
            [sys.executable,'-X','utf8','-m','unittest','discover','-s',str(folder),'-v'],
            cwd=ROOT,capture_output=True,encoding='utf-8'
        )
        logs.append(folder.relative_to(ROOT).as_posix()+'\n'+proc.stdout+proc.stderr)
        if proc.returncode:
            (OUT/'test_log.txt').write_text('\n'.join(logs),encoding='utf-8')
            raise RuntimeError('tests failed; results not frozen')
    (OUT/'test_log.txt').write_text('\n'.join(logs),encoding='utf-8')
    return logs


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--verify-only',action='store_true')
    args=parser.parse_args()
    OUT.mkdir(parents=True,exist_ok=True)

    b1_dependencies=verify_manifest(BASE/'b1_dependency_manifest.json')
    if args.verify_only:
        count=verify_manifest(OUT/'artifact_manifest.json')
        print(f'PASS: {b1_dependencies} B1 dependency files, {count} B2 artifacts')
        return

    run_tests()
    cases={}
    for path in sorted((BASE/'examples').glob('*.json')):
        cases[path.stem]=run_file(path,OUT)

    frozen={};checks=[]
    for name,result in cases.items():
        best=result['recommended']
        if best is None:
            assert result['status']=='inconsistent_observation'
            frozen[name]={'status':result['status']}
            continue
        assert best['high_precision_check']['status']=='passed'
        assert best['status']=='tolerance_reached_float'
        problem=Problem(result['observation'],result['error_deg'],**result['physical_parameters'])
        assert problem.effective_domain(best['point_local_m'])
        lo,hi=best['source_interval']['observed_deg']
        dense_d=dense_r=0.0;checked=0;boundary_empty=[]
        angles=[lo+(hi-lo)*i/256 for i in range(257)]
        angles.extend(best[key]['worst_sample_observation_local_deg']
                      for key in ['worst_diameter','worst_mec_radius'])
        for angle in sorted(set(angles)):
            reference=precise_metrics(best['point_local_m'],angle,problem.alpha)
            # The angular interval is represented by binary floats, while the
            # independent recheck re-encodes the endpoint as an exact decimal
            # rational.  At a closed extreme the exact intersection can be
            # empty even though points an ulp inward are bounded and tend to
            # zero diameter.  Record that measure-zero boundary case instead
            # of pretending it is a bounded numerical metric.
            if reference['status']=='empty' and angle in (lo,hi):
                boundary_empty.append(angle)
                continue
            if reference['status']!='bounded':
                raise AssertionError('high precision dense reference failure: '+name)
            dense_d=max(dense_d,reference['diameter_m'])
            dense_r=max(dense_r,reference['mec_radius_m'])
            checked+=1
        assert dense_d<=best['worst_diameter']['envelope_upper_m']+1e-6
        assert dense_r<=best['worst_mec_radius']['envelope_upper_m']+1e-6
        checks.append({
            'case':name,'reference_digits':65,'requested_angle_count':len(set(angles)),
            'dense_angle_count':checked,'boundary_empty_angles_deg':boundary_empty,
            'dense_max_diameter_m':dense_d,'dense_max_mec_radius_m':dense_r,
            'inside_adaptive_envelopes':True,'is_global_certificate':False,
            'boundary_empty_semantics':'float-derived closed endpoint is empty under exact re-encoding; interior samples remain bounded and excluded point has zero limiting diameter'
        })
        frozen[name]={
            'data_kind':result['data_kind'],'status':result['status'],
            'recommended_local_m':best['point_local_m'],
            'recommended_world_m':best['point_world_m'],
            'movement_m':best['movement_m'],
            'worst_diameter_bounds_m':[best['worst_diameter'][k] for k in ['sample_lower_m','envelope_upper_m']],
            'worst_mec_radius_bounds_m':[best['worst_mec_radius'][k] for k in ['sample_lower_m','envelope_upper_m']],
            'clearance_20m_assessment':best['clearance_20m_assessment'],
            'candidate_count':len(result['candidates']),
            'source_result':f'B2/结果/{name}.json'
        }

    save('robustness_results.json',{
        'status':'passed_with_documented_float_limits','seed':20260911,
        'checks':checks,'b1_dependency_hash_count':b1_dependencies,
        'baseline_failures':{
            'forward':'unbounded pure angular intersection',
            'lateral':'target (1500,0), R=1500, s2=(0,600) loses signal'
        },
        'scope':'synthetic offline validation; no simulator; no rigorous interval certificate'
    })
    save('frozen_numbers.json',{
        'scope':'validated synthetic examples of a conservative numerical strategy',
        'not_global_optima':True,'cases':frozen
    })

    # Determinism check through the public JSON entry point.
    with tempfile.TemporaryDirectory() as temp:
        repeated=run_file(BASE/'examples/central_synthetic.json',temp)
        assert repeated==cases['central_synthetic']

    verify_manifest(BASE/'b1_dependency_manifest.json')
    entries=[]
    for path in sorted(BASE.rglob('*')):
        if not path.is_file() or '__pycache__' in path.parts or path.suffix=='.png':
            continue
        if path==OUT/'artifact_manifest.json':
            continue
        entries.append({
            'path':path.relative_to(ROOT).as_posix(),
            'sha256':canonical_sha256(path)
        })
    save('artifact_manifest.json',{
        'algorithm':'sha256-canonical-text-lf-v1','scope':'B2 repository artifacts; text files are hashed after LF line-ending normalization; optional PNG previews excluded',
        'files':entries
    })
    print(json.dumps({'status':'passed','frozen_cases':list(frozen),'dense_checks':checks},ensure_ascii=False))


if __name__=='__main__':
    main()
