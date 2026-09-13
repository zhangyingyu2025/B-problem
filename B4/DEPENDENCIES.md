# Dependencies

Runtime dependency: Python standard library only.

All B4-required geometry/protocol code is contained under `B4/code/`, including vendored B1 geometry in `B4/code/vendor/`.

There is no runtime dependency on repository-level B1, B2, B3, historical B4 `solver_gXX.py` files, NumPy, SciPy, Shapely, NetworkX, or other third-party packages.

V2 uses package-relative imports to prevent accidental loading of stale modules outside this B4 directory.
