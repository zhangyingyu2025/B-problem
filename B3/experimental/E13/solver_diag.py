"""Experimental diagnostic-before-rotation solver. Source-blind.
Take a short interior move after origin scan, remeasure a few origin-seen sources,
then choose the hex rotation using the resulting live B1+ tasks.
"""
from types import SimpleNamespace
import math
from solver_cap import CapSolver
from e12_coverage_hook import E11
from run_e11_rehearsal import build_solver, ClientEnv

class DiagSolver(CapSolver):
    def __init__(self, case, variant='Q2003'):
        super().__init__(case,'C3')
        self.variant=variant
        if not (variant.startswith('Q') and len(variant)>=5): raise ValueError(variant)
        body=variant[1:]
        # Q{radius}_{m} also accepted; compact Q2003 => r=200,m=3
        if '_' in body:
            rs,ms=body.split('_',1); self.diag_r=float(rs); self.diag_m=int(ms)
        else:
            self.diag_r=float(body[:-1]); self.diag_m=int(body[-1])
        self.diag_point=None; self.diag_channels=[]

    @staticmethod
    def _adiff(a,b):
        d=(a-b+180)%360-180
        return d

    def _choose_diag(self):
        obs=[]
        for c,t in self.tracks.items():
            if c in self.env.cleared or not t.get('dirs'): continue
            obs.append((c,float(t['dirs'][0]['svd_deg'])))
        if not obs: return None,[]
        # A short baseline perpendicular to as many observed bearings as possible.
        # For each candidate angle select at most M tracks with largest parallax.
        best=None
        for deg in range(0,360,5):
            scored=sorted(((abs(math.sin(math.radians(self._adiff(deg,b)))),c,b) for c,b in obs), reverse=True)
            chosen=scored[:min(self.diag_m,len(scored))]
            # reward parallax; mild penalty for angular concentration among selected bearings
            quality=sum(x[0]**2 for x in chosen)
            # deterministic tie break, slight preference for the current channel if useful
            keep=1 if any(c==self.env.channel for _,c,_ in chosen) else 0
            key=(quality,keep,-deg)
            if best is None or key>best[0]: best=(key,deg,[c for _,c,_ in chosen])
        deg=best[1]; p=(self.diag_r*math.cos(math.radians(deg)),self.diag_r*math.sin(math.radians(deg)))
        # reorder to reduce switches: current channel first, then numeric nearest-like deterministic
        ch=best[2]
        if self.env.channel in ch:
            ch.remove(self.env.channel); ch=[self.env.channel]+sorted(ch)
        else: ch=sorted(ch)
        return p,ch

    def _candidate_nodes(self, outer):
        structural=[('cover',k+1,p) for k,p in enumerate(outer)] + self.eligible_targets()
        nodes=list(structural)
        for t in self.broad_tracks():
            p=self.route_fit_supp(t,structural)
            if p is not None: nodes.append(('supp',t['channel'],p))
        return nodes

    def choose_rotation_after_diag(self):
        best=None
        for deg in range(0,60,self.rotation_step):
            outer=[(self.rho*math.cos(math.radians(deg+60*k)),self.rho*math.sin(math.radians(deg+60*k))) for k in range(6)]
            nodes=self._candidate_nodes(outer)
            route=E11.e5.fast_open_route(self.env.pos,nodes)
            cost=E11.e5.route_cost(self.env.pos,route)
            if best is None or cost<best[0]-1e-9 or (abs(cost-best[0])<=1e-9 and deg<best[1]):
                best=(cost,deg,outer)
        self.rotation=best[1]
        return best[2]

    def run_search(self):
        self.scan_site(0,(0.,0.))
        if self.discovery_complete(): return
        p,chs=self._choose_diag(); self.diag_point=p; self.diag_channels=list(chs)
        if p is not None:
            for c in chs:
                if c in self.env.cleared: continue
                self.measure(p,c,None,'E13_rotation_diagnostic')
                if c in self.tracks: self.maybe_clear_near(self.tracks[c])
        outer=self.choose_rotation_after_diag()
        self.sites=[(0.,0.)]+outer
        self.joint_phase(list(enumerate(outer,start=1)))

def build_diag(client,variant):
    facade=SimpleNamespace(e9=E11.e9,E11Solver=lambda case:DiagSolver(case,variant))
    return build_solver(facade,ClientEnv(client))
