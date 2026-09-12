# B4 G40 — backbone-safe resolver

## Status

G40 is a conservative extension of G34-R80. It does **not** add dedicated probe measurements.
It only upgrades measurements that G34 would already take when their positions satisfy a
strict guaranteed-in-range certificate, and then uses two bounded-error bearings to replace
some active fan travel by a finite certified clear sweep.

G34 adaptive fan / center probe / finite fallback remain available unchanged as fallback.

## New mathematics implemented

After the first `direction` at observation `O` with measured bearing center `theta`, the true
source is inside the radius-1500, ±1° sector. Let its far endpoints be `A-` and `A+`.
For any point `q`, define

`rho(q) = max(|q-O|, |q-A-|, |q-A+|)`.

If `rho(q) <= 1000 m`, then every source still feasible after the first bearing is at distance
at most 1000 m from `q`. Therefore a `no_signal` at `q` is necessarily directional backside,
not a range failure. G40 uses this certificate only on measurements G34 was already going to
perform.

Every such probe also gives a certified distance disk `B(q,rho(q))`, independent of the
measurement result. Opposite-side safe backside probes can additionally yield a radial upper
bound through the chord/ray intersection theorem.

If a safe probe returns a second direction, each bearing plus its certified range upper bound
gives an outer strip of half-width `D*sin(1°)`. Two nonparallel strips form an outer
parallelogram. When its certified one-dimensional disk cover has worst-case completion time
at most 100 s, G40 may replace an active fan task with that finite clear sweep. Exhaustion of
the sweep without a clear is treated as an invariant failure; none occurred in validation.

## Fresh paired validation

Two seed blocks not used for G40 development were paired against G34-R80:

- `64002270–64002319`, 50 runs / 650 sources
- `64002320–64002369`, 50 runs / 651 sources

Combined 100-run result:

| metric | G34-R80 | G40 |
|---|---:|---:|
| sources | 1301 | 1301 |
| full-clear runs | 100/100 | 100/100 |
| weighted s/source | 468.112 | **462.824** |
| mean movement/run | 22.294 km | **21.983 km** |
| mean measurements/run | 257.90 | **256.21** |
| P95 s/source | ~626.6 | **~620.3** |
| CVaR95 s/source | 641.51 | **639.27** |
| fan steps | 185 | **163** |
| guaranteed-clear failures | 0 | **0** |
| fallback triggers | 0 | **0** |

G40 was faster in 36 paired runs, identical in 55, and slower in 9. The many exact ties are
expected because the passive policy changes nothing when no existing G34 measurement happens
to satisfy the safe-probe certificate.

The new two-strip resolver executed 44 times in the 100 fresh runs, with 139 total clear
attempts and **44/44 certified completion successes**; invariant failures were zero.

## Stress validation

Ten runs each were also executed for `all_directional`, `boundary`, `adversarial_fan`,
`coverage_edge`, and `backface_origin`. All 50/50 runs cleared every source, with zero fan
invariant failures, zero guaranteed-clear failures, and zero fallback triggers.

## Tests

`test_b4_g40.py` adds deterministic checks for:

- the three-disk guaranteed-in-range sector envelope;
- the opposite safe-backside chord radial bound;
- conservative circumscribed-disk clipping;
- dense verification of the finite two-strip clear cover.

The four G40 tests pass. Existing executable B4 correctness tests also pass in the sandbox;
the old protocol test cannot be imported there because the compact sandbox dependency tree
omits its legacy `offline_environment.py` fixture.

## Recommendation

G40 is the current preferred post-G34 candidate. Its improvement is smaller than the early
21-point breakthrough, but it is structurally cleaner: it improves movement and measurement
cost simultaneously and adds no dedicated probe action. Do not use G38 as the mainline;
G38 actively seeks safe probes and its extra measurement/switch cost reduced generalization.
