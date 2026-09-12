"""Deterministic checks for the central continuous-bound certificate."""
from decimal import Decimal
import json
import math
from pathlib import Path
import sys
import unittest


HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import verify_central as verify  # noqa: E402


DEC = Decimal


class IntervalTests(unittest.TestCase):
    def test_basic_arithmetic_contains_exact_endpoints(self):
        left = verify.Interval(DEC("1.25"), DEC("2.75"))
        right = verify.Interval(DEC("-0.5"), DEC("1.5"))
        addition = left + right
        subtraction = left - right
        self.assertLessEqual(addition.lo, DEC("0.75"))
        self.assertGreaterEqual(addition.hi, DEC("4.25"))
        self.assertLessEqual(subtraction.lo, DEC("-0.25"))
        self.assertGreaterEqual(subtraction.hi, DEC("3.25"))

    def test_trig_interval_contains_float_samples(self):
        for center in (-1.0, 0.0, 1.0, 10.0, 40.8):
            angle = verify.Interval(
                DEC(str(center - 0.05)),
                DEC(str(center + 0.05)),
            )
            sin_interval, cos_interval = verify.sin_cos_interval_deg(angle)
            for sample in (center - 0.05, center, center + 0.05):
                radians = math.radians(sample)
                self.assertLessEqual(
                    sin_interval.lo,
                    DEC(str(math.sin(radians))),
                )
                self.assertGreaterEqual(
                    sin_interval.hi,
                    DEC(str(math.sin(radians))),
                )
                self.assertLessEqual(
                    cos_interval.lo,
                    DEC(str(math.cos(radians))),
                )
                self.assertGreaterEqual(
                    cos_interval.hi,
                    DEC(str(math.cos(radians))),
                )


class ReductionTests(unittest.TestCase):
    def test_active_endpoint_belongs_to_physical_K(self):
        certificate = verify.endpoint_membership_certificate()
        self.assertTrue(certificate["inside_D0"])
        self.assertEqual(certificate["first_range_m"], "1500")
        self.assertEqual(certificate["wedge_position"], "lower boundary ray of W1")
        self.assertTrue(certificate["belongs_to_K"])

    def test_beta_domain_has_analytic_margins(self):
        certificate = verify.beta_domain_certificate()
        self.assertGreater(Decimal(certificate["upper_sine_margin"]), 0)
        self.assertGreater(Decimal(certificate["chord_radius_margin_m"]), 0)
        self.assertIn("b_code-b_exact", certificate["lower_reason"])

    def test_diameter_bound_rejects_non_monotone_radius(self):
        projection = (
            verify.LENGTH * verify.COS_TWO_ALPHA.hi
        )
        with self.assertRaises(ValueError):
            verify.diameter_lower_bound(projection)

    def test_radius_increases_with_R_A(self):
        for beta_degree in (10.0, 20.0, 30.0, 39.4, 40.5):
            beta = DEC(str(beta_degree))
            radii = []
            for distance in (500.0, 700.0, 900.0, 1100.0):
                radii.append(
                    verify.radius_from_distance(beta, DEC(str(distance)))
                )
            self.assertEqual(radii, sorted(radii))

    def test_radius_formula_recovers_opposite_vertex(self):
        for beta_degree in (10.0, 20.0, 30.0, 39.4, 40.5):
            beta = DEC(str(beta_degree))
            distance = verify.boundary_distance(beta)
            self.assertIsNotNone(distance)
            radius = verify.radius_from_distance(beta, distance)
            point = verify.boundary_point(beta)
            cosine_alpha, sine_alpha = verify.decimal_unit(verify.ALPHA)
            endpoint = (
                verify.LENGTH * cosine_alpha,
                -verify.LENGTH * sine_alpha,
            )
            opposite = (
                radius * cosine_alpha,
                radius * sine_alpha,
            )
            endpoint_vector = (
                endpoint[0] - point[0],
                endpoint[1] - point[1],
            )
            opposite_vector = (
                opposite[0] - point[0],
                opposite[1] - point[1],
            )
            endpoint_norm = math.hypot(
                float(endpoint_vector[0]),
                float(endpoint_vector[1]),
            )
            opposite_norm = math.hypot(
                float(opposite_vector[0]),
                float(opposite_vector[1]),
            )
            cosine_beta, sine_beta = verify.decimal_unit(beta)
            self.assertLess(
                abs(
                    float(
                        endpoint_vector[0] * sine_beta
                        - endpoint_vector[1] * cosine_beta
                    )
                ),
                1e-8 * endpoint_norm,
            )
            cosine_direction, sine_direction = verify.decimal_unit(
                beta - DEC(2) * verify.ALPHA
            )
            self.assertLess(
                abs(
                    float(
                        opposite_vector[0] * sine_direction
                        - opposite_vector[1] * cosine_direction
                    )
                ),
                1e-8 * opposite_norm,
            )

    def test_declared_beta_interval_covers_sampled_domain(self):
        endpoint = (
            verify.LENGTH * verify.decimal_unit(verify.ALPHA)[0],
            -verify.LENGTH * verify.decimal_unit(verify.ALPHA)[1],
        )
        for index in range(1001):
            phi = math.radians(-89.0 + 88.0 * index / 1000.0)
            point = (
                float(verify.DOMAIN_RADIUS) * math.cos(phi),
                float(verify.DOMAIN_RADIUS) * math.sin(phi),
            )
            a_value, b_value = point
            threshold = 1500.0 * math.tan(math.radians(1.0)) + (
                1500.0 - a_value
            ) * math.tan(math.radians(3.1))
            if b_value >= -threshold:
                continue
            near_margin = (
                abs(b_value) * math.cos(math.radians(1.0))
                - a_value * math.sin(math.radians(1.0))
            )
            if near_margin <= 5.0:
                continue
            beta = math.degrees(
                math.atan2(
                    float(endpoint[1]) - b_value,
                    float(endpoint[0]) - a_value,
                )
            )
            self.assertGreaterEqual(beta, float(verify.LOWER_BETA))
            self.assertLessEqual(beta, float(verify.UPPER_BETA))

    def test_candidate_mirror_has_same_leading_metrics(self):
        point = verify.candidate_point()
        reverse = (point[0], -point[1])
        problem = verify.Problem({"x": 0, "y": 0, "svd_deg": 0})
        for candidate in (point, reverse):
            self.assertTrue(problem.effective_domain(candidate))
        first = verify.evaluate_candidate(
            problem,
            point,
            max_intervals=128,
            abs_tolerance_m=0.1,
            rel_tolerance=0.0,
            high_precision=False,
        )
        second = verify.evaluate_candidate(
            problem,
            reverse,
            max_intervals=128,
            abs_tolerance_m=0.1,
            rel_tolerance=0.0,
            high_precision=False,
        )
        for key in ("worst_diameter", "worst_mec_radius"):
            self.assertAlmostEqual(
                first[key]["sample_lower_m"],
                second[key]["sample_lower_m"],
                places=7,
            )
            self.assertAlmostEqual(
                first[key]["envelope_upper_m"],
                second[key]["envelope_upper_m"],
                places=7,
            )

    def test_box_budget_is_explicit(self):
        result = verify.certify_lower_bound(
            verify.DEC("1e9"),
            max_boxes=4,
            min_width=verify.DEC("1e-9"),
        )
        self.assertEqual(result["status"], "box_budget_reached")
        self.assertGreater(result["pending_boxes"], 0)


class CertificateTests(unittest.TestCase):
    def test_generated_certificate_schema_and_boundary(self):
        path = HERE / "certificate.json"
        report = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(report["schema_version"], 2)
        self.assertEqual(report["case"], "central_synthetic")
        self.assertEqual(report["certification"]["status"], "closed")
        self.assertEqual(report["certification"]["pending_boxes"], 0)
        self.assertGreater(
            report["bracket"]["feasible_upper_m"],
            report["bracket"]["global_lower_m"],
        )
        self.assertLess(report["bracket"]["width_m"], 1e-4)
        self.assertTrue(
            report["conclusion"][
                "continuous_global_infimum_bracketed_on_stated_scope"
            ]
        )

    def test_scope_does_not_overclaim_full_C_safe(self):
        report = json.loads(
            (HERE / "certificate.json").read_text(encoding="utf-8")
        )
        limitations = " ".join(report["scope"]["not_certified"])
        self.assertIn("C_safe", limitations)
        self.assertIn("non-central", limitations)
        self.assertFalse(
            report["conclusion"]["exact_global_optimum_equality_proved"]
        )


if __name__ == "__main__":
    unittest.main()
