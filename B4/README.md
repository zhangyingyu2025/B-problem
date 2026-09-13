# B4 Final Consolidated Solver (V2 isolated package)

正式测试与论文材料请先读 [论文交接.md](论文交接.md)。三局正式证据位于 `results/v2_formal/`；跨电脑重建表图使用 `final_formal_manifest.json`。

This directory is the standalone B4 solver. It does not depend on the historical `solver_gXX.py` chain or on repository B1/B3 modules.

## Important V2 change

All internal imports are package-relative (`B4.code...`). This prevents stale or same-named modules elsewhere in the repository from being imported accidentally. `source_manifest.json` fingerprints the executable source set using SHA256 after LF normalization.

## Verify after copying into the repository

From the repository root:

```powershell
python B4/diagnose.py
python -m unittest discover -s B4/tests -v
python B4/verify_reproduction.py --count 50
```

Expected unit tests: 4 tests, all `ok`.

Expected DEV50 summary:

- runs: 50
- full_clear_runs: 50
- sources: 648
- weighted_s_per_source: 450.56364499382715
- mismatches: []

The DEV50 cases are frozen in `results/dev50_cases.json`, so reproduction does not regenerate the random cases.

## Entry points

Offline:

```powershell
python B4/run_offline.py --count 1
```

Official/local HTTP simulator:

```powershell
python B4/run_official.py --help
```

Python standard library only; no third-party package is required.
