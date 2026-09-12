# Round 13: expanded safe transit with immediate replanning

O's first 20 pairs: all-clear, 246.052 s/source vs M 249.686 and E11 270.860, with lower P95/CVaR/max and 12.171 km movement. Promising, not final confirmation.

New random seeds 52001200–52001219, E11/O/P, fixed_field, four workers and 120 second guard. P changes only transit-point generation: use the existing E geometry-based safe polygon test and information score, now combined with O's event replanning. E alone failed previously; this is an explicit interaction test because its new clear opportunities were previously followed by completion of the old destination. No parameter changes to that point generator. Reception requires every feasible vertex <=1000m; information gain is only a heuristic. Do not enable coverage deformation or official runs here.
