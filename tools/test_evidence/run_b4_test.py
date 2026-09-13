"""One operator-started B4 V2 session, then case-code capture and evidence export."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
from datetime import datetime

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from B4.code.protocol import Client, HTTPTransport
from B4.code.solver import B4Solver, RobotPort
from export import export_config, update_ledger


def dump(path,data):
    path.write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')


def fingerprint():
    return {p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes().replace(b'\r\n',b'\n')).hexdigest()
            for p in sorted((ROOT/'B4/code').rglob('*.py'))}


def verify_source():
    manifest=json.loads((ROOT/'B4/source_manifest.json').read_text(encoding='utf-8'))
    for name,expected in manifest['sha256_lf'].items():
        path=ROOT/'B4'/name
        actual=hashlib.sha256(path.read_text(encoding='utf-8').encode('utf-8')).hexdigest()
        if actual!=expected:raise ValueError(f'B4 V2 source differs from supplied manifest: {name}')


def run_session(folder, mode, robot_id, url, transport=None):
    """No truth access: the supplied solver receives only RobotPort."""
    folder.mkdir(parents=True,exist_ok=False)
    (folder/'simulator_logs').mkdir()
    dump(folder/'session.json',dict(problem='B4',mode=mode,strategy='B4 V2 B4Solver',
         robot_id=robot_id,started_local=datetime.now().isoformat(),code_sha256_lf=fingerprint()))
    raw=transport if transport is not None else HTTPTransport(url)
    # Preserve uncertain requests for audit; do not retry by starting another session.
    def logged_transport(path,payload):
        try:
            status,body=raw(path,payload)
        except Exception as exc:
            client.record(dict(kind='transport_failure',path=path,payload=payload,error=str(exc)))
            raise
        if status==200 and isinstance(body,dict) and body.get('accepted') is True:
            client.record(dict(kind='invalid_accepted_response',path=path,payload=payload,
                               response=body,note='Pending Client validation; resolved by matching action record'))
        return status,body
    client=Client(logged_transport,robot_id,log_path=folder/'actions.jsonl')
    solver=None;error=None
    try:
        client.enter();solver=B4Solver(RobotPort(client));solver.run_all()
    except Exception as exc:
        error=f'{type(exc).__name__}: {exc}'
    finally:
        if client.active:
            try:client.exit()
            except Exception as exc:error=(error+'; ' if error else '')+f'exit: {exc}'
    complete=bool(solver is not None and solver.discovery_complete() and solver.cleared==set(solver.tracks))
    result=dict(problem='B4',mode=mode,strategy='B4 V2 B4Solver',status='completed' if complete and error is None else 'incomplete',
                error=error,discovery_complete=complete,cleared_count=len(solver.cleared) if solver else 0,
                virtual_time_s=client.virtual_time_s,metrics=client.stats,solver_stats=solver.stats if solver else {},
                true_source_count=None,clear_fraction=None)
    dump(folder/'result.json',result)
    return result


def record(folder,mode,code):
    session=json.loads((folder/'session.json').read_text(encoding='utf-8'))
    if session['problem']!='B4' or session['mode']!=mode:raise ValueError('session problem/mode mismatch')
    if not code.strip():raise ValueError('case code required')
    config={'runs':[dict(problem='B4',mode=mode,case_code=code.strip(),actions='actions.jsonl')]}
    stamp=datetime.now().strftime('%Y%m%d-%H%M%S-%f')
    dump(folder/f'evidence-manifest-{stamp}.json',config)
    output=folder/f'evidence-{stamp}'
    reports=export_config(config,folder,output,write_tables=False)
    ledger=ROOT/'B4/results'/('v2_'+mode)/'summary'
    rows=update_ledger(reports,ledger)
    print(f'本局结果：清除 {reports[0]["cleared_count"]} 个；平均时间 {reports[0]["mean_time_per_clear_s"]} 秒/源')
    print(f'累计记录：{len(rows)}；总表：{ledger / "table.csv"}')
    print(f'本局轨迹：{output / "run-01-trajectory.svg"}')
    print(f'模拟器原名日志及截图保存到：{folder / "simulator_logs"}')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode',choices=['rehearsal','formal'],required=True)
    parser.add_argument('--ready',action='store_true',help='operator has selected the matching B4 simulator mode')
    parser.add_argument('--robot-id',default='202623001400')
    parser.add_argument('--url',default='http://127.0.0.1:2026')
    parser.add_argument('--record-only',type=Path,help='only record a code/export existing logs; no simulator connection')
    parser.add_argument('--case-code')
    args=parser.parse_args()
    base=ROOT/'B4/results'/('v2_'+args.mode)
    if args.record_only:
        folder=args.record_only.resolve()
        if not folder.is_relative_to(base.resolve()):parser.error('record-only path must belong to the selected test mode')
    else:
        if not args.ready:parser.error('--ready required after selecting the matching mode in the simulator')
        attempts=list(base.glob('run-*/session.json'))
        limit=5 if args.mode=='rehearsal' else 3
        if len(attempts)>=limit:parser.error(f'{limit} local attempts already recorded; use --record-only for export, not another test')
        # A single-session lock prevents overlapping commands from contacting the API.
        base.mkdir(parents=True,exist_ok=True)
        lock=ROOT/'B4/results/.v2-test.lock'
        with lock.open('x'):pass
        try:
            verify_source()
            if len(list(base.glob('run-*/session.json')))>=limit:
                raise ValueError('Local test attempt limit reached')
            folder=base/('run-'+datetime.now().strftime('%Y%m%d-%H%M%S-%f'))
            print(f'本轮 {len(attempts)+1}/{limit}；结果目录：{folder}',flush=True)
            result=run_session(folder,args.mode,args.robot_id,args.url)
            if result['status']!='completed':print(f'本轮异常：{result["error"]}。仍保留并导出确认动作；不要为补表重新测试。')
        finally:lock.unlink()
    print(f'模拟器日志存放目录：{folder / "simulator_logs"}',flush=True)
    code=args.case_code or input('请粘贴本局模拟器案例编码，按回车：').strip()
    while not code:code=input('案例编码不能为空，请粘贴：').strip()
    record(folder,args.mode,code)
    return 0


if __name__=='__main__':raise SystemExit(main())
