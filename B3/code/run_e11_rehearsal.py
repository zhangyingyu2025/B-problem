"""Operator-enabled rehearsal runner for the experimental E11 B3 strategy.

Safety policy:
- supports ONLY synthetic offline verification and operator-declared rehearsal;
- there is deliberately no formal-test mode;
- rehearsal requires both --robot-id and --rehearsal-ready;
- refuses to overwrite an existing actions.jsonl.

The E11 algorithm remains in B3/experimental/E11 so the frozen A/B/C solver is
not modified.  This runner injects the real protocol Client into the already
validated E11 decision logic.
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
EXP = BASE / "experimental" / "E11" / "e11_experiment.py"

# Script directory is on sys.path when invoked normally, but keep this explicit
# for import-by-path and unusual launchers.
CODE = BASE / "code"
if str(CODE) not in sys.path:
    sys.path.insert(0, str(CODE))

from offline_environment import OfflineTransport, make_case
from protocol_adapter import Client, HTTPTransport, ProtocolError


class ClientEnv:
    """Minimal environment surface expected by the E11 decision logic.

    It intentionally exposes no source coordinates, source count, or reception
    radius.  Decisions therefore use only protocol observations and local state.
    """

    def __init__(self, client: Client):
        self.c = client
        self.cleared: set[int] = set()

    @property
    def pos(self):
        return self.c.position

    @property
    def channel(self):
        return self.c.channel

    @property
    def vt(self):
        return self.c.virtual_time_s

    @property
    def stats(self):
        return self.c.stats

    def measure(self, p, ch, reason="E11_measure"):
        body = self.c.measure(p, ch, reason)
        result = body["measure_result"]
        out = {"measure_result": result}
        if result == "direction":
            out["svd_deg"] = body["svd_deg"]
        return out

    def clear(self, p, ch, reason="E11_clear"):
        body = self.c.clear(p, ch, reason)
        ok = body["clear_result"] == "success"
        if ok:
            self.cleared.add(ch)
        return ok


def load_e11():
    if not EXP.is_file():
        raise FileNotFoundError(
            f"E11 experiment not found: {EXP}. "
            "Install E11成果代码_解压到仓库根目录.zip first."
        )
    spec = importlib.util.spec_from_file_location("cumcm_e11_live", EXP)
    module = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(module)
    return module


def build_solver(e11, env: ClientEnv):
    """Instantiate E11 without giving it a synthetic source list.

    The inheritance tree historically constructs an offline environment in
    E5Solver.__init__.  We replace that constructor hook with this source-blind
    ClientEnv before instantiation.  The dummy case is used only to initialize
    generic track/site containers; it contains zero hidden sources.
    """
    e5 = e11.e9.e6.e5
    old_factory = e5.e1.OfficialEnv
    e5.e1.OfficialEnv = lambda _case: env
    try:
        dummy = {"kind": "live_protocol", "seed": 0, "pattern": "random", "sources": []}
        solver = e11.E11Solver(dummy)
    finally:
        e5.e1.OfficialEnv = old_factory

    # Preserve detailed decision reasons in protocol logs.  The prototype's
    # DSolver.measure discards the reason when crossing the env boundary.
    def live_measure(p, c, site_idx=None, reason=""):
        res = env.measure(p, c, "E11:" + (reason or "measure"))
        solver.handle(p, c, res, site_idx, reason)
        return res

    solver.measure = live_measure
    solver.env = env
    return solver


def provenance():
    try:
        commit = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True, stderr=subprocess.DEVNULL
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        commit = None
    paths = [
        ROOT / "B1" / "code" / "b1_geometry.py",
        BASE / "code" / "protocol_adapter.py",
        BASE / "code" / "offline_environment.py",
        Path(__file__),
    ]
    paths += sorted((BASE / "experimental" / "E11").glob("*.py"))
    hashes = {}
    for p in paths:
        if p.is_file():
            hashes[p.relative_to(ROOT).as_posix()] = hashlib.sha256(
                p.read_bytes().replace(b"\r\n", b"\n")
            ).hexdigest()
    return {"git_head": commit, "code_sha256_lf": hashes}


def run_with_client(client: Client, data_kind: str, true_source_count=None):
    e11 = load_e11()
    env = ClientEnv(client)
    client.enter()
    started = time.perf_counter()
    error = None
    try:
        solver = build_solver(e11, env)
        solver.run_all()
        discovery_complete = solver.discovery_complete()
        uncleared = sorted(c for c in solver.tracks if c not in env.cleared)
        status = "completed" if discovery_complete and not uncleared else "incomplete"
    except Exception as exc:  # retain trace log and structured result before re-raising
        solver = locals().get("solver")
        discovery_complete = solver.discovery_complete() if solver is not None else False
        uncleared = sorted(c for c in solver.tracks if c not in env.cleared) if solver is not None else []
        status = "error"
        error = f"{type(exc).__name__}: {exc}"
    finally:
        if client.active:
            try:
                client.exit()
            except Exception as exit_exc:
                if error is None:
                    status = "error"
                    error = f"exit failed: {type(exit_exc).__name__}: {exit_exc}"

    wall = time.perf_counter() - started
    discovered = sorted(solver.tracks) if solver is not None else []
    result = {
        "status": status,
        "strategy": "E11",
        "data_kind": data_kind,
        "official_rehearsal": data_kind == "operator_declared_official_rehearsal",
        "discovery_complete": discovery_complete,
        "discovered_count": len(discovered),
        "discovered_channels": discovered,
        "cleared_count": len(env.cleared),
        "cleared_channels": sorted(env.cleared),
        "uncleared_channels": uncleared,
        "rotation_deg": getattr(solver, "rotation", None) if solver is not None else None,
        "metrics": {
            "total_virtual_time_s": client.virtual_time_s,
            "mean_time_per_clear_s": client.virtual_time_s / len(env.cleared) if env.cleared else None,
            "wall_time_s": wall,
            **client.stats,
            "joint_supp": getattr(solver, "joint_supp", None) if solver is not None else None,
            "joint_steps": getattr(solver, "joint_steps", None) if solver is not None else None,
            "transit_attempts": getattr(solver, "transit_attempts", None) if solver is not None else None,
            "transit_dirs": getattr(solver, "transit_dirs", None) if solver is not None else None,
            "center_failures": getattr(solver, "center_failures", None) if solver is not None else None,
            "local_radial_attempts": getattr(solver, "local_radial_attempts", None) if solver is not None else None,
        },
        "error": error,
    }
    if true_source_count is not None:
        result["true_source_count"] = true_source_count
        result["clear_fraction"] = len(env.cleared) / true_source_count if true_source_count else None
        result["accounting_check"] = {
            "components_sum_s": client.stats["movement_m"] / 5
            + 5 * client.stats["measurements"]
            + client.stats["switches"]
            + 5 * client.stats["clear_successes"]
            + 3 * client.stats["clear_failures"]
        }
        result["accounting_check"]["difference_s"] = abs(
            result["accounting_check"]["components_sum_s"] - client.virtual_time_s
        )
    else:
        result["true_source_count"] = None
        result["clear_fraction"] = None
        result["clear_fraction_note"] = (
            "Official simulator true source count must be read from the rehearsal result page. "
            "discovery_complete is the algorithm's seven-site certificate under the problem assumptions."
        )
    return result


def main():
    parser = argparse.ArgumentParser(
        description="E11 synthetic verification or operator-confirmed rehearsal; no formal-test mode"
    )
    parser.add_argument("--mode", choices=["offline", "rehearsal"], default="offline")
    parser.add_argument("--seed", type=int, default=31415900)
    parser.add_argument("--count", type=int, choices=range(10, 17), metavar="10..16")
    parser.add_argument("--robot-id")
    parser.add_argument("--url", default="http://127.0.0.1:2026")
    parser.add_argument(
        "--rehearsal-ready",
        action="store_true",
        help="operator confirms simulator is in rehearsal mode with interface ready",
    )
    parser.add_argument("--output-dir", type=Path, default=BASE / "结果" / "e11_manual_run")
    args = parser.parse_args()

    if args.mode == "rehearsal" and (not args.robot_id or not args.rehearsal_ready):
        parser.error(
            "rehearsal requires --robot-id and --rehearsal-ready; HTTP cannot determine formal/rehearsal mode"
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
        transport = OfflineTransport(case)
        client = Client(
            transport,
            "offline-test",
            trace,
            session_id=f"e11-live-offline-{args.seed}",
        )
        result = run_with_client(client, "synthetic_offline_E11_live_adapter", len(case["sources"]))
        result.update(case_seed=args.seed, pattern=case.get("pattern"), official_rehearsal=False)
    else:
        client = Client(HTTPTransport(args.url), args.robot_id, trace)
        result = run_with_client(client, "operator_declared_official_rehearsal")

    result["provenance"] = provenance()
    result_path.write_text(
        json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(
        json.dumps(
            {
                "status": result["status"],
                "strategy": result["strategy"],
                "discovery_complete": result["discovery_complete"],
                "discovered_count": result["discovered_count"],
                "cleared_count": result["cleared_count"],
                "virtual_time_s": result["metrics"]["total_virtual_time_s"],
                "mean_time_per_clear_s": result["metrics"]["mean_time_per_clear_s"],
                "rotation_deg": result["rotation_deg"],
                "error": result["error"],
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    if result["status"] != "completed":
        raise SystemExit(2)


if __name__ == "__main__":
    main()
