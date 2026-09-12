"""Operator-enabled rehearsal runner for the current E13-R B3 strategy.

Safety policy:
- supports ONLY synthetic offline verification and operator-declared rehearsal;
- deliberately provides no formal-test mode;
- rehearsal requires both --robot-id and --rehearsal-ready;
- refuses to overwrite an existing actions.jsonl or result.json.

E13-R remains source-blind: the live solver sees only protocol observations.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / "B3"
CODE = BASE / "code"
E13 = BASE / "experimental" / "E13"
E13_SOLVER = E13 / "solver.py"

for p in (CODE, E13):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from offline_environment import OfflineTransport, make_case
from protocol_adapter import Client, HTTPTransport


def load_e13():
    if not E13_SOLVER.is_file():
        raise FileNotFoundError(f"E13 solver not found: {E13_SOLVER}")
    spec = importlib.util.spec_from_file_location("cumcm_e13_live", E13_SOLVER)
    module = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(module)
    return module


def provenance():
    try:
        commit = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True, stderr=subprocess.DEVNULL
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        commit = None

    paths = [
        ROOT / "B1" / "code" / "b1_geometry.py",
        CODE / "protocol_adapter.py",
        CODE / "offline_environment.py",
        CODE / "run_e11_rehearsal.py",
        CODE / "e12_coverage_hook.py",
        Path(__file__),
    ]
    paths += sorted(E13.glob("*.py"))
    paths += sorted((BASE / "experimental" / "E11").glob("*.py"))
    hashes = {}
    for p in paths:
        if p.is_file():
            hashes[p.relative_to(ROOT).as_posix()] = hashlib.sha256(
                p.read_bytes().replace(b"\r\n", b"\n")
            ).hexdigest()
    return {"git_head": commit, "code_sha256_lf": hashes}


def run_with_client(client: Client, data_kind: str, true_source_count=None):
    e13 = load_e13()
    client.enter()
    started = time.perf_counter()
    solver = None
    error = None
    status = "error"
    discovery_complete = False
    uncleared = []
    try:
        solver = e13.build_e13(client, variant="R")
        solver.run_all()
        discovery_complete = solver.discovery_complete()
        uncleared = sorted(c for c in solver.tracks if c not in solver.env.cleared)
        status = "completed" if discovery_complete and not uncleared else "incomplete"
    except Exception as exc:
        if solver is not None:
            discovery_complete = solver.discovery_complete()
            uncleared = sorted(c for c in solver.tracks if c not in solver.env.cleared)
        error = f"{type(exc).__name__}: {exc}"
        status = "error"
    finally:
        if client.active:
            try:
                client.exit()
            except Exception as exit_exc:
                if error is None:
                    error = f"exit failed: {type(exit_exc).__name__}: {exit_exc}"
                    status = "error"

    wall = time.perf_counter() - started
    discovered = sorted(solver.tracks) if solver is not None else []
    cleared = sorted(solver.env.cleared) if solver is not None else []
    result = {
        "status": status,
        "strategy": "E13-R",
        "data_kind": data_kind,
        "official_rehearsal": data_kind == "operator_declared_official_rehearsal",
        "discovery_complete": discovery_complete,
        "discovered_count": len(discovered),
        "discovered_channels": discovered,
        "cleared_count": len(cleared),
        "cleared_channels": cleared,
        "uncleared_channels": uncleared,
        "rotation_deg": getattr(solver, "rotation", None) if solver is not None else None,
        "metrics": {
            "total_virtual_time_s": client.virtual_time_s,
            "mean_time_per_clear_s": client.virtual_time_s / len(cleared) if cleared else None,
            "wall_time_s": wall,
            **client.stats,
            "b1plus_updates": getattr(solver, "b1plus_updates", None) if solver is not None else None,
            "b1plus_empty": getattr(solver, "b1plus_empty", None) if solver is not None else None,
            "guaranteed_clear_failures": getattr(solver, "guaranteed_clear_failures", None) if solver is not None else None,
            "policy_localizations": getattr(solver, "policy_localizations", None) if solver is not None else None,
            "risky_clear_tasks": getattr(solver, "risky_clear_tasks", None) if solver is not None else None,
            "recovery_movement_m": getattr(solver, "recovery_movement_m", None) if solver is not None else None,
            "joint_supp": getattr(solver, "joint_supp", None) if solver is not None else None,
            "joint_steps": getattr(solver, "joint_steps", None) if solver is not None else None,
            "transit_attempts": getattr(solver, "transit_attempts", None) if solver is not None else None,
            "transit_dirs": getattr(solver, "transit_dirs", None) if solver is not None else None,
        },
        "error": error,
    }

    if true_source_count is not None:
        result["true_source_count"] = true_source_count
        result["clear_fraction"] = len(cleared) / true_source_count if true_source_count else None
    else:
        result["true_source_count"] = None
        result["clear_fraction"] = None
        result["clear_fraction_note"] = (
            "Official simulator true source count is not exposed by the HTTP protocol; "
            "read it from the rehearsal result page after the run."
        )
    return result


def main():
    parser = argparse.ArgumentParser(
        description="E13-R synthetic verification or operator-confirmed rehearsal; no formal-test mode"
    )
    parser.add_argument("--mode", choices=["offline", "rehearsal"], default="offline")
    parser.add_argument("--seed", type=int, default=52001500)
    parser.add_argument("--count", type=int, choices=range(10, 17), metavar="10..16")
    parser.add_argument("--robot-id")
    parser.add_argument("--url", default="http://127.0.0.1:2026")
    parser.add_argument(
        "--rehearsal-ready",
        action="store_true",
        help="operator confirms simulator is in rehearsal mode and the API is ready",
    )
    parser.add_argument("--output-dir", type=Path, default=BASE / "结果" / "e13_manual_run")
    args = parser.parse_args()

    if args.mode == "rehearsal" and (not args.robot_id or not args.rehearsal_ready):
        parser.error(
            "rehearsal requires --robot-id and --rehearsal-ready; "
            "the HTTP API cannot determine rehearsal/formal mode"
        )

    out = args.output_dir if args.output_dir.is_absolute() else ROOT / args.output_dir
    out.mkdir(parents=True, exist_ok=True)
    trace = out / "actions.jsonl"
    result_path = out / "result.json"
    if trace.exists() or result_path.exists():
        parser.error("output directory already contains run logs; choose a new directory")

    if args.mode == "offline":
        case = make_case(args.seed, count=args.count)
        (out / "synthetic_case.json").write_text(
            json.dumps(case, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        client = Client(
            OfflineTransport(case),
            "offline-test",
            trace,
            session_id=f"e13-live-offline-{args.seed}",
        )
        result = run_with_client(client, "synthetic_offline_E13_R", len(case["sources"]))
        result.update(case_seed=args.seed, pattern=case.get("pattern"), official_rehearsal=False)
    else:
        client = Client(HTTPTransport(args.url), args.robot_id, trace)
        result = run_with_client(client, "operator_declared_official_rehearsal")

    result["provenance"] = provenance()
    result_path.write_text(
        json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps({
        "status": result["status"],
        "strategy": result["strategy"],
        "discovery_complete": result["discovery_complete"],
        "discovered_count": result["discovered_count"],
        "cleared_count": result["cleared_count"],
        "virtual_time_s": result["metrics"]["total_virtual_time_s"],
        "mean_time_per_clear_s": result["metrics"]["mean_time_per_clear_s"],
        "rotation_deg": result["rotation_deg"],
        "error": result["error"],
    }, ensure_ascii=False, indent=2))
    if result["status"] != "completed":
        raise SystemExit(2)


if __name__ == "__main__":
    main()
