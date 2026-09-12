"""G2 analytical sanity checks, not a completed B2 optimizer. Standard library.

All coordinates/readings below are synthetic. No simulator is contacted.
The envelope upper bound is analytical; its reported float evaluation is NOT
an outward-rounded interval certificate. Samples are lower bounds only.
"""
from pathlib import Path
import hashlib
import json
import math
import platform
import random
import sys

BASE = Path(__file__).resolve().parents[1]
ROOT = BASE.parent
sys.path.insert(0, str(ROOT / "B1" / "code"))
import b1_geometry as geo

ALPHA = 1.0
SEED = 20260911


def planes(p, theta, alpha=ALPHA):
    return geo.observations_to_halfplanes([
        {"x": p[0], "y": p[1], "svd_deg": theta % 360}
    ], alpha)[0]


def diameter(h):
    hs = geo._rational_planes(h)
    if geo._feasible(hs) is None:
        return {"status": "empty", "diameter_m": None}
    direction, _ = geo._recession(hs)
    if direction is not None:
        return {"status": "unbounded", "diameter_m": None}
    vertices, ill = geo._vertices(hs)
    if ill or not vertices:
        return {"status": "numerically_uncertain", "diameter_m": None}
    d2, _ = geo._exact_diameter(vertices)
    return {"status": "bounded", "diameter_m": math.sqrt(float(d2))}


def safe(p):
    a, b = p
    alpha = math.radians(ALPHA)
    q = a*a+b*b
    return q <= 1000**2 and q <= 2000*(a*math.cos(alpha)-abs(b)*math.sin(alpha))


def main():
    rng = random.Random(SEED)
    candidates = [(500, 600), (800, 550), (900, 400), (950, 300)]
    triangle = [(0, 0), (1500, 1500*math.tan(math.radians(ALPHA))),
                (1500, -1500*math.tan(math.radians(ALPHA)))]
    source_grid = [(r*math.cos(math.radians(phi)), r*math.sin(math.radians(phi)))
                   for r in (5.01, 100, 500, 1000, 1500)
                   for phi in (-1, 0, 1)]
    results = []
    for p in candidates:
        assert safe(p)
        # The point is above the entire triangle; atan2 has no branch crossing.
        bearings = [math.degrees(math.atan2(y-p[1], x-p[0])) for x,y in triangle]
        assert max(bearings) < -(3*ALPHA+0.1)
        # Distance from p to the infinite first wedge already exceeds 5 m;
        # its subset T can only be farther away.
        assert p[1]*math.cos(math.radians(ALPHA))-p[0]*math.sin(math.radians(ALPHA)) > 5
        lo, hi = min(bearings)-ALPHA, max(bearings)+ALPHA
        assert hi-lo < 180-2*ALPHA
        envelope = diameter(planes((0,0), 0) + planes(p, (lo+hi)/2, ALPHA+(hi-lo)/2))
        assert envelope["status"] == "bounded"
        sample_max = -1.0
        witness = None
        checks = 0
        for g in source_grid:
            assert 5 < math.dist(p,g) <= max(1000, math.hypot(*g))+1e-9
            true = math.degrees(math.atan2(g[1]-p[1],g[0]-p[0]))
            for error in (-1, 0, 1):
                item = diameter(planes((0,0), 0)+planes(p, true+error))
                assert item["status"] == "bounded"
                d = item["diameter_m"]
                assert d <= envelope["diameter_m"]+1e-7
                if d > sample_max:
                    sample_max, witness = d, {"target_m":g, "second_error_deg":error}
                checks += 1
        # Random range checks supplement the proof; they do not replace it.
        for _ in range(1000):
            r = rng.uniform(5.001,1500)
            phi = math.radians(rng.uniform(-1,1))
            g = (r*math.cos(phi),r*math.sin(phi))
            assert 5 < math.dist(p,g) <= max(1000,r)+1e-9
        results.append({"second_point_m": p, "movement_m":math.hypot(*p),
                        "sample_count":checks, "sample_lower_bound_m":sample_max,
                        "sample_witness":witness,
                        "analytic_envelope_upper_bound_float_m":envelope["diameter_m"],
                        "observation_interval_deg": [lo,hi]})
    lateral = {"target_m":[1500,0], "radius_m":1500, "second_point_m":[0,600],
               "second_range_m":math.hypot(1500,600)}
    assert lateral["second_range_m"] > lateral["radius_m"]
    forward = diameter(planes((0,0),0)+planes((500,0),0))
    assert forward["status"] == "unbounded"
    verified = {}
    dep_manifest=BASE/"b1_dependency_manifest.json"
    entries=json.loads(dep_manifest.read_text(encoding="utf-8"))["files"]
    for entry in entries:
        assert hashlib.sha256((ROOT/entry["path"]).read_bytes()).hexdigest() == entry["sha256"]
    verified["b1_dependencies"] = len(entries)
    inputs = [Path(__file__), ROOT/"B1/code/b1_geometry.py", dep_manifest]
    output = {"stage":"G2_feasibility_only", "data_type":"synthetic",
              "python":platform.python_version(), "seed":SEED,
              "input_sha256":{p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs},
              "candidate_results":results, "lateral_signal_failure":lateral,
              "forward_unbounded_failure":forward, "dependency_hash_checks":verified,
              "limitations":["No continuous optimality claim", "Sample maxima are lower bounds",
                              "Float upper bounds are not interval arithmetic certificates",
                              "Triangle envelope ignores the 1800 m domain for a conservative bound"]}
    target = BASE/"结果/G2可行性核验.json"
    target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text(json.dumps(output,ensure_ascii=False,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    print(json.dumps(output,ensure_ascii=False,indent=2))


if __name__ == "__main__":
    main()
