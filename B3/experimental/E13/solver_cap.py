from types import SimpleNamespace
from solver import E13Solver
from e12_coverage_hook import E11
from run_e11_rehearsal import build_solver,ClientEnv
class CapSolver(E13Solver):
 def __init__(self,case,variant='C3'):
  super().__init__(case,'R');self.variant=variant
  if not(variant.startswith('C') and variant[1:].isdigit()):raise ValueError(variant)
  self.transit_cap=int(variant[1:])
 def joint_phase(self,remaining_cover):
  import event_route_cap;return event_route_cap.run(self,remaining_cover,self.transit_cap)
def build_cap(client,variant):
 facade=SimpleNamespace(e9=E11.e9,E11Solver=lambda case:CapSolver(case,variant));return build_solver(facade,ClientEnv(client))
