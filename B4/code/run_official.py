"""Run the final B4 solver against the official/local HTTP simulator."""
import argparse, json, sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
if str(HERE) not in sys.path: sys.path.insert(0,str(HERE))
from .protocol import Client, HTTPTransport
from .solver import B4Solver, RobotPort

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--base-url',default='http://127.0.0.1:2026')
    ap.add_argument('--robot-id',required=True)
    ap.add_argument('--log',type=Path,default=Path('B4_final_run.jsonl'))
    args=ap.parse_args()
    client=Client(HTTPTransport(args.base_url),args.robot_id,log_path=args.log)
    solver=None
    try:
        client.enter(); solver=B4Solver(RobotPort(client)); solver.run_all()
    finally:
        if client.active: client.exit()
    print(json.dumps({'virtual_time_s':client.virtual_time_s,'metrics':client.stats,
                      'solver_stats':solver.stats if solver else {}},ensure_ascii=False,indent=2))
if __name__=='__main__': main()
