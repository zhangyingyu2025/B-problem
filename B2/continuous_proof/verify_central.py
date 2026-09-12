"""Certify a continuous-placement lower bound for the central synthetic B2 case.

Scope
-----
This script does not modify the frozen B2 solver results.  It studies the
central synthetic case

    s1=(0,0), theta1=0, D0=B(0,1800),
    alpha=1 degree, 1000 <= R <= 1500, near=5 m,

and the central-case effective domain C0.  The first source set is

    K = D0 intersect W1 intersect {5 < r1 <= 1500}.

The reduction is as follows.  Let

    A = 1500 * (cos(alpha), -sin(alpha))

be the lower endpoint of the first bearing wedge.  The endpoint A belongs to
K.  For every admissible p on the lower branch of C0, define

    beta = arg(A-p),  R_A = |A-p|.

The reading theta_A=beta-alpha is physically possible because A is a source
and the true direction from p to A is beta.  At this reading, A is one vertex
of P(theta_A;p).  The opposite vertex C lies on the upper ray of W1 and the
lower ray of W2, and its radius from the first detector is

    r_C(beta,R_A)
      = [1500 sin(beta-alpha) + R_A sin(2 alpha)]
        / sin(beta-3 alpha).

For fixed beta, r_C increases with R_A.  Since p lies in B(0,1000), the
smallest feasible R_A occurs when |p|=1000.  The effective-domain angular
separation gives beta>3.1 degrees, and the radius constraint gives
beta<=arcsin(2/3)-alpha<40.811 degrees.  Therefore the two-dimensional
continuous lower-bound problem reduces to the one-dimensional interval

    beta in [3 degrees, 41 degrees].

The central instance is invariant under reflection y -> -y, so the certified
lower branch also certifies the upper branch.  A near-boundary point is then
evaluated by the original adaptive inner-angle solver to obtain a feasible
upper endpoint.  The lower endpoint is produced by a Decimal branch-and-bound
with explicit outward rounding.  The upper endpoint is still a floating
envelope and is not claimed to be an interval certificate.
"""
from __future__ import annotations

import argparse
from collections import deque
from decimal import Decimal, localcontext, ROUND_CEILING, ROUND_FLOOR
import json
import math
from pathlib import Path
import sys


BASE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BASE / "code"))

from b2_solver import evaluate_candidate  # noqa: E402
from domains import Problem  # noqa: E402
from geometry import PI as DECIMAL_PI, decimal_unit  # noqa: E402


DEC = Decimal
PREC = 100
PAD = DEC("1e-55")
LENGTH = DEC(1500)
DOMAIN_RADIUS = DEC(1000)
ALPHA = DEC(1)
THREE_ALPHA = DEC(3)
LOWER_BETA = DEC(3)
UPPER_BETA = DEC(41)
ACTUAL_LOWER_BETA = DEC("3.1")
NUMERICAL_UPPER_BETA = DEC("40.810314895779")
LOWER_TARGET_R = DEC("1623.49551")
INWARD_SCALE = DEC("0.999999999")

with localcontext() as context:
    context.prec = PREC
    DEG_TO_RAD = DECIMAL_PI / DEC(180)


def _round_down(value):
    with localcontext() as context:
        context.prec = PREC
        context.rounding = ROUND_FLOOR
        return +value


def _round_up(value):
    with localcontext() as context:
        context.prec = PREC
        context.rounding = ROUND_CEILING
        return +value


def _add_down(left, right):
    with localcontext() as context:
        context.prec = PREC
        context.rounding = ROUND_FLOOR
        return left + right


def _add_up(left, right):
    with localcontext() as context:
        context.prec = PREC
        context.rounding = ROUND_CEILING
        return left + right


def _sub_down(left, right):
    with localcontext() as context:
        context.prec = PREC
        context.rounding = ROUND_FLOOR
        return left - right


def _sub_up(left, right):
    with localcontext() as context:
        context.prec = PREC
        context.rounding = ROUND_CEILING
        return left - right


def _mul_down(left, right):
    with localcontext() as context:
        context.prec = PREC
        context.rounding = ROUND_FLOOR
        return left * right


def _mul_up(left, right):
    with localcontext() as context:
        context.prec = PREC
        context.rounding = ROUND_CEILING
        return left * right


def _div_down(left, right):
    with localcontext() as context:
        context.prec = PREC
        context.rounding = ROUND_FLOOR
        return left / right


def _div_up(left, right):
    with localcontext() as context:
        context.prec = PREC
        context.rounding = ROUND_CEILING
        return left / right


def bounded_interval(center, radius):
    """Return a directed outward interval [center-radius, center+radius]."""

    if radius < 0:
        raise ValueError("interval radius must be nonnegative")
    return Interval(_sub_down(center, radius), _add_up(center, radius))


class Interval:
    """Closed Decimal interval with explicit outward rounding."""

    __slots__ = ("lo", "hi")

    def __init__(self, lo, hi):
        lo = lo if isinstance(lo, DEC) else DEC(str(lo))
        hi = hi if isinstance(hi, DEC) else DEC(str(hi))
        if lo > hi:
            raise ValueError("interval endpoints are reversed")
        self.lo = lo
        self.hi = hi

    @staticmethod
    def point(value):
        value = value if isinstance(value, DEC) else DEC(str(value))
        return bounded_interval(value, PAD)

    def __add__(self, other):
        other = as_interval(other)
        return Interval(
            _add_down(self.lo, other.lo),
            _add_up(self.hi, other.hi),
        )

    __radd__ = __add__

    def __sub__(self, other):
        other = as_interval(other)
        return Interval(
            _sub_down(self.lo, other.hi),
            _sub_up(self.hi, other.lo),
        )

    def __rsub__(self, other):
        return as_interval(other) - self

    def __mul__(self, other):
        other = as_interval(other)
        with localcontext() as context:
            context.prec = PREC
            context.rounding = ROUND_FLOOR
            lows = (
                self.lo * other.lo,
                self.lo * other.hi,
                self.hi * other.lo,
                self.hi * other.hi,
            )
        with localcontext() as context:
            context.prec = PREC
            context.rounding = ROUND_CEILING
            highs = (
                self.lo * other.lo,
                self.lo * other.hi,
                self.hi * other.lo,
                self.hi * other.hi,
            )
        return Interval(min(lows), max(highs))

    __rmul__ = __mul__

    def __truediv__(self, other):
        other = as_interval(other)
        if other.lo <= 0 <= other.hi:
            raise ZeroDivisionError("interval divisor contains zero")
        with localcontext() as context:
            context.prec = PREC
            context.rounding = ROUND_FLOOR
            lows = (
                self.lo / other.lo,
                self.lo / other.hi,
                self.hi / other.lo,
                self.hi / other.hi,
            )
        with localcontext() as context:
            context.prec = PREC
            context.rounding = ROUND_CEILING
            highs = (
                self.lo / other.lo,
                self.lo / other.hi,
                self.hi / other.lo,
                self.hi / other.hi,
            )
        return Interval(min(lows), max(highs))

    def width(self):
        return self.hi - self.lo

    def __repr__(self):
        return f"Interval({self.lo}, {self.hi})"


def as_interval(value):
    return value if isinstance(value, Interval) else Interval.point(value)


def sin_cos_interval_deg(angle):
    """Return conservative sin and cos intervals on an angle interval.

    decimal_unit supplies a 65-digit midpoint evaluation.  PAD covers its
    final rounding error.  The Lipschitz radius uses the high-precision
    Decimal value of pi, not the binary float value.
    """

    angle = as_interval(angle)
    with localcontext() as context:
        context.prec = PREC
        midpoint = (angle.lo + angle.hi) / DEC(2)
        radius = (angle.hi - angle.lo) * DEG_TO_RAD / DEC(2)
        radius += PAD
    cosine, sine = decimal_unit(midpoint)
    return bounded_interval(sine, radius), bounded_interval(cosine, radius)


def sqrt_interval(value):
    value = as_interval(value)
    if value.hi < 0:
        raise ValueError("square root interval is negative")
    low = max(DEC(0), value.lo)
    with localcontext() as context:
        context.prec = PREC
        context.rounding = ROUND_FLOOR
        low_root = low.sqrt()
    with localcontext() as context:
        context.prec = PREC
        context.rounding = ROUND_CEILING
        high_root = value.hi.sqrt()
    return Interval(_sub_down(low_root, PAD), _add_up(high_root, PAD))


SIN_TWO_ALPHA = sin_cos_interval_deg(Interval.point(DEC(2)))[0]
COS_TWO_ALPHA = sin_cos_interval_deg(Interval.point(DEC(2)))[1]


def radius_lower_bound(beta_interval):
    """Lower-bound r_C for every feasible p with beta in beta_interval."""

    beta_interval = as_interval(beta_interval)
    x_interval = beta_interval + ALPHA
    sin_x, cos_x = sin_cos_interval_deg(x_interval)

    radius_constant = Interval.point(DOMAIN_RADIUS)
    length_constant = Interval.point(LENGTH)
    discriminant = (
        radius_constant * radius_constant
        - length_constant * length_constant * sin_x * sin_x
    )
    if discriminant.hi < 0:
        return None
    if discriminant.lo < 0:
        discriminant.lo = DEC(0)

    # For fixed beta, R_A is minimized at |p|=1000.  The radius is increasing
    # in R_A, so the lower endpoint of the distance and denominator upper
    # endpoint give a safe lower bound.
    distance_lower = _sub_down(
        _mul_down(LENGTH, cos_x.lo),
        sqrt_interval(discriminant).hi,
    )
    if distance_lower < 0:
        distance_lower = DEC(0)

    sin_beta_minus_alpha, _ = sin_cos_interval_deg(beta_interval - ALPHA)
    numerator_lower = _add_down(
        _mul_down(LENGTH, sin_beta_minus_alpha.lo),
        _mul_down(distance_lower, SIN_TWO_ALPHA.lo),
    )
    denominator, _ = sin_cos_interval_deg(
        beta_interval - THREE_ALPHA
    )
    if denominator.hi <= 0:
        return None
    return _div_down(numerator_lower, denominator.hi)


def certify_lower_bound(target, max_boxes=300000, min_width=DEC("1e-8")):
    """Branch-and-bound beta, pruning every box whose lower bound reaches target."""

    with localcontext() as context:
        context.prec = PREC
        queue = deque([Interval(LOWER_BETA, UPPER_BETA)])
        processed = 0
        pruned = 0
        unclosed = []

        while queue:
            beta = queue.popleft()
            processed += 1
            lower = radius_lower_bound(beta)
            if lower is None or lower >= target:
                pruned += 1
                continue
            if beta.width() <= min_width:
                unclosed.append(
                    {
                        "beta_deg": [str(beta.lo), str(beta.hi)],
                        "lower_r_m": str(lower),
                    }
                )
                continue
            midpoint = (beta.lo + beta.hi) / DEC(2)
            queue.append(Interval(beta.lo, midpoint))
            queue.append(Interval(midpoint, beta.hi))
            if processed > max_boxes:
                return {
                    "status": "box_budget_reached",
                    "target_r_C_m": str(target),
                    "processed_boxes": processed,
                    "pending_boxes": len(queue),
                    "unclosed": unclosed,
                }

        return {
            "status": "closed" if not unclosed else "unclosed_boxes",
            "target_r_C_m": str(target),
            "processed_boxes": processed,
            "pruned_boxes": pruned,
            "pending_boxes": len(queue),
            "unclosed": unclosed,
        }


def diameter_lower_bound(radius):
    """Return a directed lower bound for the common-chord diameter D(radius)."""

    projection = _mul_up(LENGTH, COS_TWO_ALPHA.hi)
    if radius <= projection:
        raise ValueError(
            "diameter is not increasing on the declared radius interval"
        )
    with localcontext() as context:
        context.prec = PREC
        context.rounding = ROUND_FLOOR
        squared = (
            LENGTH * LENGTH
            + radius * radius
            - DEC(2) * LENGTH * radius * COS_TWO_ALPHA.hi
        )
    if squared < 0:
        raise ValueError("negative squared diameter")
    return sqrt_interval(Interval(squared, squared)).lo


def boundary_distance(beta):
    """Return the smaller positive root for |p|=1000 on the ray A-p."""

    x_value = beta + ALPHA
    cosine_x, sine_x = decimal_unit(x_value)
    with localcontext() as context:
        context.prec = PREC
        discriminant = (
            DOMAIN_RADIUS * DOMAIN_RADIUS
            - LENGTH * LENGTH * sine_x * sine_x
        )
        if discriminant < 0:
            return None
        root = discriminant.sqrt()
        return LENGTH * cosine_x - root


def radius_from_distance(beta, distance):
    """Evaluate r_C for an exact beta and an admissible R_A."""

    _, sine_beta_minus_alpha = decimal_unit(beta - ALPHA)
    _, sine_two_alpha = decimal_unit(DEC(2))
    _, sine_beta_minus_three_alpha = decimal_unit(
        beta - THREE_ALPHA
    )
    with localcontext() as context:
        context.prec = PREC
        return (
            LENGTH * sine_beta_minus_alpha
            + distance * sine_two_alpha
        ) / sine_beta_minus_three_alpha


def radius_on_boundary(beta):
    distance = boundary_distance(beta)
    if distance is None:
        return None
    return radius_from_distance(beta, distance)


def find_boundary_minimum():
    """Locate the lower-branch boundary minimum for candidate construction."""

    low = DEC("3.1")
    high = NUMERICAL_UPPER_BETA
    for _ in range(260):
        left = (DEC(2) * low + high) / DEC(3)
        right = (low + DEC(2) * high) / DEC(3)
        left_value = radius_on_boundary(left)
        right_value = radius_on_boundary(right)
        if left_value < right_value:
            high = right
        else:
            low = left
    return (low + high) / DEC(2)


def boundary_point(beta):
    """Return the point p on |p|=1000 that realizes the beta reduction."""

    distance = boundary_distance(beta)
    if distance is None:
        raise ValueError("beta is outside the feasible boundary interval")
    cosine_beta, sine_beta = decimal_unit(beta)
    cosine_alpha, sine_alpha = decimal_unit(ALPHA)
    return (
        LENGTH * cosine_alpha - distance * cosine_beta,
        -LENGTH * sine_alpha - distance * sine_beta,
    )


def candidate_point():
    """Return a strict-interior point arbitrarily close to the boundary."""

    beta = find_boundary_minimum()
    point = boundary_point(beta)
    return tuple(float(INWARD_SCALE * value) for value in point)


def endpoint_membership_certificate():
    """Record the exact K-membership facts for A."""

    cosine_alpha, sine_alpha = decimal_unit(ALPHA)
    endpoint = (
        LENGTH * cosine_alpha,
        -LENGTH * sine_alpha,
    )
    inside_d0 = LENGTH < DEC(1800)
    range_ok = DEC(5) < LENGTH <= LENGTH
    return {
        "name": "A",
        "coordinates_m": [str(value) for value in endpoint],
        "norm_m": str(LENGTH),
        "inside_D0": inside_d0,
        "first_range_m": str(LENGTH),
        "range_condition": "5 < 1500 <= 1500",
        "wedge_position": "lower boundary ray of W1",
        "belongs_to_K": inside_d0 and range_ok,
        "role": "active physical source for theta_A=beta-alpha",
    }


def beta_domain_certificate():
    upper_sine = sin_cos_interval_deg(Interval.point(DEC("41.811")))[0]
    two_thirds = DEC(2) / DEC(3)
    chord_projection = _mul_up(LENGTH, COS_TWO_ALPHA.hi)
    radius_margin = _sub_down(LOWER_TARGET_R, chord_projection)
    return {
        "declared_interval_deg": [str(LOWER_BETA), str(UPPER_BETA)],
        "actual_lower_deg": str(ACTUAL_LOWER_BETA),
        "numerical_upper_deg": str(NUMERICAL_UPPER_BETA),
        "lower_reason": (
            "Let b_code=-L tan(alpha)-(L-a)tan(3.1deg) and "
            "b_exact=-L sin(alpha)-(L cos(alpha)-a)tan(3.1deg). "
            "b_code-b_exact=-L(1-cos(alpha))(tan(3.1deg)-tan(alpha))<0. "
            "Thus b<b_code implies b<b_exact and beta>3.1deg; the "
            "outward interval starts at 3deg"
        ),
        "upper_reason": (
            "|p|<=1000 gives 1500*sin(beta+alpha)<=1000. The interval "
            "certificate proves sin(41.811deg)>2/3, hence "
            "beta<=asin(2/3)-1deg<40.811deg<41deg"
        ),
        "upper_sine_lower": str(upper_sine.lo),
        "upper_sine_margin": str(upper_sine.lo - two_thirds),
        "chord_radius_margin_m": str(radius_margin),
        "declared_interval_contains_domain": True,
    }


def build_report():
    with localcontext() as context:
        context.prec = PREC
        certification = certify_lower_bound(LOWER_TARGET_R)
        lower_diameter = diameter_lower_bound(LOWER_TARGET_R)

    problem = Problem({"x": 0, "y": 0, "svd_deg": 0})
    point = candidate_point()
    if not problem.effective_domain(point):
        raise RuntimeError("near-boundary candidate is outside C0_sub")

    inner = evaluate_candidate(
        problem,
        point,
        max_intervals=4096,
        abs_tolerance_m=1e-7,
        rel_tolerance=0.0,
        high_precision=True,
    )
    if inner["status"] != "tolerance_reached_float":
        raise RuntimeError("inner-angle upper envelope did not converge")

    upper_diameter = DEC(str(inner["worst_diameter"]["envelope_upper_m"]))
    gap = _sub_up(upper_diameter, lower_diameter)
    mirror_point = (point[0], -point[1])

    return {
        "schema_version": 2,
        "case": "central_synthetic",
        "scope": {
            "source_model": (
                "K=D0 intersect W1 intersect {5<r1<=1500} "
                "for the central instance"
            ),
            "placement_domain": (
                "complete central-case C0: lower branch is certified, "
                "upper branch follows by exact reflection"
            ),
            "not_certified": [
                "the full C_safe outside the stated C0 domain",
                "non-central transformed physical instances",
                "formal proof-assistant arithmetic for the floating upper bound",
            ],
        },
        "reduction": {
            "active_source_endpoint": endpoint_membership_certificate(),
            "beta_domain": beta_domain_certificate(),
            "radius_formula": (
                "r_C=(1500 sin(beta-alpha)+R_A sin(2alpha))"
                "/sin(beta-3alpha)"
            ),
            "radius_monotone_in_R_A": True,
            "outer_dimension_after_reduction": 1,
        },
        "branch_symmetry": {
            "status": "closed_by_exact_reflection",
            "reflection": "(x,y)->(x,-y)",
            "argument": (
                "D0, W1, K, T and C0 are invariant in the central "
                "instance; the source wedge and its error interval mirror "
                "to the reflected branch without changing diameter"
            ),
            "mirrored_feasible_candidate_m": list(mirror_point),
        },
        "interval_arithmetic": {
            "decimal_precision": PREC,
            "padding": str(PAD),
            "rounding": (
                "ROUND_FLOOR for lower endpoints and ROUND_CEILING "
                "for upper endpoints"
            ),
            "trigonometric_bound": (
                "65-digit midpoint from decimal_unit plus a Lipschitz "
                "radius using the high-precision Decimal pi constant"
            ),
            "certificate_grade": (
                "numerical outward intervals; not a proof-assistant "
                "formalization"
            ),
        },
        "certification": certification,
        "lower_bound": {
            "r_C_lower_m": str(LOWER_TARGET_R),
            "diameter_lower_m": str(lower_diameter),
            "semantics": (
                "J_K(p) is at least this diameter for every p in the "
                "complete central-case C0"
            ),
        },
        "candidate": {
            "point_local_m": list(point),
            "movement_m": math.hypot(*point),
            "inner_solver_status": inner["status"],
            "diameter_upper_m": inner["worst_diameter"]["envelope_upper_m"],
            "high_precision_check": inner["high_precision_check"]["status"],
            "upper_bound_semantics": (
                "adaptive float envelope on H; numerical, not an "
                "interval certificate"
            ),
        },
        "bracket": {
            "global_lower_m": float(lower_diameter),
            "feasible_upper_m": inner["worst_diameter"]["envelope_upper_m"],
            "width_m": float(gap),
        },
        "conclusion": {
            "continuous_global_infimum_bracketed_on_stated_scope": (
                certification["status"] == "closed"
                and gap > 0
                and upper_diameter >= lower_diameter
            ),
            "exact_global_optimum_equality_proved": False,
            "reason": (
                "the continuous lower endpoint is certified to the shown "
                "bracket; the feasible upper endpoint is an adaptive float "
                "envelope, so exact equality is not claimed"
            ),
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = build_report()
    text = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")
    if report["certification"]["status"] != "closed":
        raise SystemExit(2)


if __name__ == "__main__":
    main()
