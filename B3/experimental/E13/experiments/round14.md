# Round 14: joint choice of localization side

P was all-clear but slower than O (254.340 vs 247.618 s/source) and moved 0.918 km more despite 17.3 fewer measurements. Retain this rejected interaction result.

Q changes O's joint localization point selection. Each source offers the original point and its reflection through the observable MEC center, only if the latter also certifies <=1000m reception for every outer-polygon vertex and is not a repeated observation. Alternate route ordering with predecessor/successor-aware side selection, at most six passes, retaining the best full route. This changes measurement geometry, not just a stronger TSP on the same fixed nodes. All-clear/risk/coverage rules unchanged.

Fresh random seeds 52001300–52001319; E11/O/Q paired; fixed_field, four workers, 120s guard. Evaluate movement and tails, retain all outcomes. No grid-based coverage certificate and no coverage deformation.
