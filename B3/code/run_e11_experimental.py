"""Offline-only runner for the E11 joint-route prototype.

This does NOT add E11 to the official/rehearsal CLI.  It deliberately uses the
repository's OfflineTransport + protocol Client through the experimental code.
Run from repository root.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path
import statistics
import sys

ROOT = Path(__file__).resolve().parents[2]
EXP = ROOT / "B3" / "experimental" / "E11" / "e11_experiment.py"


def load_e11():
    if not EXP.is_file():
        raise FileNotFoundError(f"E11 experiment not found: {EXP}")
    spec = importlib.util.spec_from_file_location("cumcm_e11", EXP)
    module = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(module)
    return module


def fixed12(make_case):
    cases = [make_case(20260911 + i) for i in range(8)]
    cases += [
        make_case(20261001, pattern="boundary"),
        make_case(20261002, pattern="boundary"),
        make_case(20261003, pattern="cluster"),
        make_case(20261004, pattern="origin"),
    ]
    return cases


def summarize(rows):
    total_sources = sum(r["n"] for r in rows)
    total_virtual = sum(r["vt"] for r in rows)
    return {
        "runs": len(rows),
        "sources": total_sources,
        "all_cleared": all(r["cleared"] == r["n"] for r in rows),
        "weighted_time_per_source_s": total_virtual / total_sources,
        "mean_time_per_source_s": statistics.mean(r["vt"] / r["n"] for r in rows),
        "mean_total_virtual_s": statistics.mean(r["vt"] for r in rows),
        "mean_movement_m": statistics.mean(r["movement"] for r in rows),
        "mean_measurements": statistics.mean(r["measurements"] for r in rows),
        "max_time_per_source_s": max(r["vt"] / r["n"] for r in rows),
    }


def main():
    parser = argparse.ArgumentParser(description="Run E11 offline prototype only")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--fixed12", action="store_true", help="run the historical 12-case offline comparison set")
    group.add_argument("--seed", type=int, help="run one synthetic random case")
    group.add_argument("--start-seed", type=int, help="first seed for a random validation batch")
    parser.add_argument("--runs", type=int, default=20, help="number of cases with --start-seed (default: 20)")
    parser.add_argument("--count", type=int, choices=range(10, 17), metavar="10..16", help="force source count for random cases")
    parser.add_argument("--output", type=Path, help="optional JSON result path")
    args = parser.parse_args()

    e11 = load_e11()
    if args.fixed12:
        cases = fixed12(e11.make_case)
    elif args.seed is not None:
        cases = [e11.make_case(args.seed, count=args.count)]
    else:
        start = args.start_seed if args.start_seed is not None else 20263000
        if args.runs < 1:
            parser.error("--runs must be >= 1")
        cases = [e11.make_case(start + i, count=args.count) for i in range(args.runs)]

    rows = []
    for i, case in enumerate(cases, 1):
        row = e11.run_case(case)
        rows.append(row)
        print(
            f"{i:02d}/{len(cases)} seed={row['seed']} n={row['n']} "
            f"clear={row['cleared']}/{row['n']} vt={row['vt']:.3f}s "
            f"per={row['vt']/row['n']:.3f}s rotation={row['rotation']}deg",
            flush=True,
        )

    payload = {
        "scope": "E11_offline_experimental_not_rehearsal_ready",
        "summary": summarize(rows),
        "runs": rows,
    }
    print(json.dumps(payload["summary"], ensure_ascii=False, indent=2))
    if args.output:
        out = args.output if args.output.is_absolute() else ROOT / args.output
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(f"wrote {out}")


if __name__ == "__main__":
    main()
