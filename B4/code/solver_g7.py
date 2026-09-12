"""G7 DEV: finer zero-detour sampling locations along already-paid travel segments."""
from solver_g4 import G4TransitSolver
class G7Fine9(G4TransitSolver): transit_fracs=tuple(i/10 for i in range(1,10))
class G7Fine7(G4TransitSolver): transit_fracs=(.125,.25,.375,.5,.625,.75,.875)
class G7Mid5(G4TransitSolver): transit_fracs=(.1,.3,.5,.7,.9)
class G7Fine9K3(G7Fine9): transit_per_leg=3
class G7Fine9K5(G7Fine9): transit_per_leg=5
