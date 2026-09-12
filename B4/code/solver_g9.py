"""G9 DEV: transit discovery probes only when one more detection could trigger N=16 early stop."""
from collections import defaultdict
import math
from solver_g4 import G4TransitSolver

class _G9LateProbe(G4TransitSolver):
    probes=1
    min_tracks=15
    min_leg=250.
    def __init__(self,*a,**kw):
        super().__init__(*a,**kw);self._probe=defaultdict(int)
        self.stats.update({'late_probe_measurements':0,'late_probe_discoveries':0,'late_probe_no_signal':0})
    def transit_opportunities(self,destination,exclude_channel=None):
        start=self.port.position;length=math.dist(start,destination)
        # First plan normal G4, but we need ensure no backtracking if we add midpoint.
        # If late probing is inactive, use exact parent behavior.
        active=(len(self.tracks)>=self.min_tracks and not self.discovery_complete() and length>=self.min_leg)
        if not active:return super().transit_opportunities(destination,exclude_channel)
        # Reproduce G4 known opportunities with u retained.
        options=[]
        for c,t in self.tracks.items():
            if c in self.cleared or c==exclude_channel or t.near is not None or not t.dirs:continue
            best=None
            for u in self.transit_fracs:
                p=(start[0]+u*(destination[0]-start[0]),start[1]+u*(destination[1]-start[1]))
                score=self.opportunity_score(c,p)
                if score is not None and (best is None or score>best[0]):best=(score,u,p,c,'known')
            if best is not None:options.append(best)
        chosen=sorted(options,reverse=True)[:self.transit_per_leg]
        unknown=[c for c in range(1,21) if c not in self.tracks and c not in self.absent]
        unknown.sort(key=lambda c:(self._probe[c],c))
        u=.5;p=(start[0]+.5*(destination[0]-start[0]),start[1]+.5*(destination[1]-start[1]))
        for c in unknown[:self.probes]:chosen.append((0.,u,p,c,'unknown'))
        chosen.sort(key=lambda x:(x[1],x[4]!='unknown'))
        for _,u,p,c,typ in chosen:
            if typ=='unknown':
                if c in self.tracks or c in self.absent:continue
                self._probe[c]+=1;body=self.observe(p,c,'B4_late_transit_discovery');self.stats['late_probe_measurements']+=1
                if body['measure_result']=='no_signal':self.stats['late_probe_no_signal']+=1
                else:self.stats['late_probe_discoveries']+=1
            else:
                if c in self.cleared or self.tracks[c].near is not None or self._already_sampled(self.tracks[c],p):continue
                body=self.observe(p,c,'B4_transit_opportunity');self.stats['transit_opportunity_measurements']+=1
                k=body['measure_result'];self.stats['transit_opportunity_directions']+=k=='direction';self.stats['transit_opportunity_near']+=k=='near';self.stats['transit_opportunity_no_signal']+=k=='no_signal'
class G9P1(_G9LateProbe):probes=1
class G9P2(_G9LateProbe):probes=2
class G9P3(_G9LateProbe):probes=3
class G9P5(_G9LateProbe):probes=5
