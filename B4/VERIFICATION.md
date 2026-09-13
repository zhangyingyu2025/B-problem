# Verification

Final V2 package was tested from a clean arbitrary repository root with:

```text
python -m unittest discover -s B4/tests -v
```

Result: 4/4 tests passed.

A full paired DEV50 replay against the frozen reference produced:

```text
runs: 50
full_clear_runs: 50
sources: 648
weighted_s_per_source: 450.56364499382715
mismatch_count: 0
```

Core source files are fingerprinted in `source_manifest.json`. Run `python B4/diagnose.py` to print both source hashes and the exact filesystem paths of imported B4 modules.

## Cross-platform V3 note

The final 21 certificate coordinates are frozen as IEEE-754 literals instead of
being regenerated with platform `sin/cos`.  This removes the earliest symmetric
routing branch that can differ between Windows and Linux at ulp scale.  The frozen
coordinates are bit-for-bit equal to the original Linux reference coordinates.
