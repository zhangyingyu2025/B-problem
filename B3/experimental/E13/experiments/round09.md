# Round 09: information-triggered routing

E11/J/M on fresh random seeds 52000800–52000819, fixed_field, four workers and 120 seconds per strategy guard. M preserves J geometry, small risky clears, cover positions, safe transit-point generation and ordinary next-node scoring.

Only change: when a transit observation adds a clearable task or immediately clears a source, recompute the joint route at that actual position. If its first action changes, discard the old pending destination and execute the newly planned action. This tests delayed replanning as a source of online regret; it does not predict unknown source locations. An interrupted leg requires observed state progress, and a finite decision guard remains. All outcomes and raw actions retained; no promotion unless movement/time/tails improve with all-clear.
