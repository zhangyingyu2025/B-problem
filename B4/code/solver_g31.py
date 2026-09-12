"""G31 DEV: certified adaptive SPF-beta based only on current outer geometry.
Small beta approaches the source efficiently when the certified distance lower bound
is informative; larger beta buys stronger bearing geometry when lb is weak.  Every
selected (beta,k) individually satisfies the G30 theorem, so adaptivity changes only
performance, never correctness.
"""
import math
from solver_g30 import _G30FanBase, paired_fan_beta

SAFE={4.:(1.0),5.:(1.0),7.5:1.0,10.:1.0,15.:1.02,20.:1.04,25.:1.08,30.:1.13,45.:1.30}
class _G31Adaptive(_G30FanBase):
    # thresholds are upper ratio bounds paired with beta, final beta otherwise
    schedule=((.3,30.),(.5,15.),(.7,7.5),(2.,4.))
    def _choice(self,t,lb):
        proxy=self._proxy(t)
        est=math.dist(t.last_signal_point,proxy) if proxy is not None else lb
        ratio=min(1.,lb/max(lb,est,1e-9))
        for hi,beta in self.schedule:
            if ratio<hi:return beta,SAFE[beta],ratio
        beta=self.schedule[-1][1];return beta,SAFE[beta],ratio
    def _fan_pair(self,t,lb):
        b,k,_=self._choice(t,lb)
        return paired_fan_beta(t.last_signal_point,t.last_signal_bearing,lb,b,k)
    def execute_source(self,task):
        # G30 execution records class-level beta; harmless, add actual choice event.
        if task[0]=='fan':
            t=self.tracks[task[1]]; from b4_geometry import lower_distance
            lb=max(20.,lower_distance(t.last_signal_point,t.geometry['polygon']))
            b,k,r=self._choice(t,lb)
            self.port.record({'kind':'B4_adaptive_fan_choice','channel':task[1],'beta_deg':b,'factor':k,'lb_proxy_ratio':r})
        return super().execute_source(task)

class G31A(_G31Adaptive): schedule=((.25,30.),(.45,15.),(.70,7.5),(2.,4.))
class G31B(_G31Adaptive): schedule=((.20,30.),(.40,15.),(.65,7.5),(2.,4.))
class G31C(_G31Adaptive): schedule=((.30,20.),(.55,10.),(.75,5.),(2.,4.))
class G31D(_G31Adaptive): schedule=((.25,25.),(.50,15.),(.75,7.5),(2.,4.))
class G31E(_G31Adaptive): schedule=((.15,30.),(.35,20.),(.60,10.),(.80,5.),(2.,4.))
class G31F(_G31Adaptive): schedule=((.35,15.),(.65,7.5),(2.,4.))
