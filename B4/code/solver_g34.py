"""G34 DEV: one certified-distance center probe before fan when the current MEC is small.
If P is contained in B(c,r) with r<1000, every feasible source is within guaranteed
minimum reception range of c. Therefore no_signal at c is a proven backside event,
not a distance ambiguity. A signal gives a short-baseline bearing near the eventual
clear location. SPF-beta and finite fallback remain unchanged.
"""
from solver_g31 import G31F

class _G34CenterProbe(G31F):
    probe_radius=250.
    def __init__(self,*a,**kw):
        super().__init__(*a,**kw);self.center_probed=set();self.stats.update({'center_probes':0,'center_probe_signal':0,'center_probe_no_signal':0,'center_probe_near':0})
    def source_task(self,c):
        task=super().source_task(c)
        if task[0]=='fan' and c not in self.center_probed:
            t=self.tracks[c]
            if t.mec and 20.<t.mec[1]<=self.probe_radius and not self._already_sampled(t,t.mec[0]):
                return ('probe',c,t.mec[0])
        return task
    def execute_source(self,task):
        if task[0]!='probe':return super().execute_source(task)
        _,c,p=task;t=self.tracks[c]
        # Recheck the current outer certificate before using no_signal as backside.
        if not t.mec or t.mec[1]>self.probe_radius+1e-7:
            return super().execute_source(self.source_task(c))
        self.center_probed.add(c);self.stats['center_probes']+=1
        body=self.observe(p,c,'B4_center_probe',backside=True)
        k=body['measure_result']
        if k=='direction':self.stats['center_probe_signal']+=1
        elif k=='near':self.stats['center_probe_near']+=1
        else:self.stats['center_probe_no_signal']+=1

class G34R80(_G34CenterProbe):probe_radius=80.
class G34R120(_G34CenterProbe):probe_radius=120.
class G34R180(_G34CenterProbe):probe_radius=180.
class G34R250(_G34CenterProbe):probe_radius=250.
class G34R350(_G34CenterProbe):probe_radius=350.
class G34R500(_G34CenterProbe):probe_radius=500.
class G34R800(_G34CenterProbe):probe_radius=800.

class G34R40(_G34CenterProbe):probe_radius=40.
class G34R60(_G34CenterProbe):probe_radius=60.
class G34R100(_G34CenterProbe):probe_radius=100.
