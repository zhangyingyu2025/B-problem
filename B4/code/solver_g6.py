"""G6 DEV: diminishing-return cap on zero-detour/opportunity measurements."""
from collections import defaultdict
from solver_g4 import G4TransitSolver

class _G6Cap(G4TransitSolver):
    opportunity_cap=6
    def __init__(self,*a,**kw):
        super().__init__(*a,**kw); self._opp_count=defaultdict(int)
    def opportunity_score(self,c,p):
        if self._opp_count[c]>=self.opportunity_cap:return None
        return super().opportunity_score(c,p)
    def observe(self,p,c,reason,site=None,backside=False):
        body=super().observe(p,c,reason,site=site,backside=backside)
        if reason in ('B4_certificate_opportunity','B4_transit_opportunity'):
            self._opp_count[c]+=1
        return body
class G6Cap4(_G6Cap):opportunity_cap=4
class G6Cap5(_G6Cap):opportunity_cap=5
class G6Cap6(_G6Cap):opportunity_cap=6
class G6Cap7(_G6Cap):opportunity_cap=7
class G6Cap8(_G6Cap):opportunity_cap=8
