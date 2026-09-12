"""G30 DEV: generalized certified paired fan SPF-beta.

For bearing error |delta|<=alpha, symmetric fan angle beta>alpha and step d=k*lb:
1) both fan points remain no farther from the true source than the signal anchor if
   k <= 2*cos(beta+alpha);
2) the anchor vector lies in the positive cone of the two fan vectors, hence every
   emitting half-plane through the source that contains the anchor contains at least
   one fan point, if k <= cos(alpha)/cos(beta).
Thus the original SPF45 is one member of a certified family.  Ambiguous no_signal and
all G18 fallback logic are unchanged.
"""
import math
from solver_g18 import G18B8
from solver_g0 import InvariantFailure
from b4_geometry import lower_distance

ALPHA_DEG=1.0

def paired_fan_beta(anchor,bearing_deg,lb,beta_deg,factor):
    if not (beta_deg>ALPHA_DEG and 0<lb<=1500): raise ValueError('invalid SPF-beta input')
    limit=min(math.cos(math.radians(ALPHA_DEG))/math.cos(math.radians(beta_deg)),
              2*math.cos(math.radians(beta_deg+ALPHA_DEG)))
    if not factor < limit: raise ValueError(f'unsafe SPF-beta factor {factor} >= {limit}')
    step=factor*lb
    return tuple((anchor[0]+step*math.cos(math.radians(bearing_deg+s*beta_deg)),
                  anchor[1]+step*math.sin(math.radians(bearing_deg+s*beta_deg))) for s in (-1,1))

class _G30FanBase(G18B8):
    beta_deg=10.0; fan_factor=1.0
    # carry forward the better opportunity budget found after G18.
    transit_per_leg=4
    def __init__(self,*a,**kw):
        super().__init__(*a,**kw)
        self.opportunity_per_site=4
        self.max_proxy_distance=1000.
        self.min_line_angle=10.
        self.stats.update({'spf_beta_deg':self.beta_deg,'spf_factor':self.fan_factor})
    def _fan_pair(self,t,lb):
        return paired_fan_beta(t.last_signal_point,t.last_signal_bearing,lb,self.beta_deg,self.fan_factor)
    def source_task(self,c):
        t=self.tracks[c]
        if t.near is not None:return ('near',c,t.near)
        if t.mec and t.mec[1]<=20-1e-6:
            # let inherited full-clear-region logic optimize the guaranteed representative
            return super().source_task(c)
        if t.fan_steps>=self.fan_limit:return ('fallback',c,t.mec[0])
        anchor=t.last_signal_point;lb=lower_distance(anchor,t.geometry['polygon'])
        if lb<=20 and anchor not in t.clear_fail_points:return ('anchor',c,anchor)
        pair=self._fan_pair(t,max(lb,20.))
        return ('fan',c,min(pair,key=lambda p:math.dist(self.port.position,p)))
    def execute_source(self,task):
        kind,c,p=task;t=self.tracks[c]
        if kind!='fan':return super().execute_source(task)
        anchor=t.last_signal_point;lb=max(20.,lower_distance(anchor,t.geometry['polygon']))
        pair=sorted(self._fan_pair(t,lb),key=lambda q:math.dist(self.port.position,q))
        if math.dist(p,pair[0])>1e-6:raise InvariantFailure('stale SPF-beta task')
        t.fan_steps+=1;self.stats['fan_steps']+=1
        self.port.record({'kind':'B4_fan','channel':c,'anchor':anchor,'lower_bound_m':lb,
                          'points':pair,'beta_deg':self.beta_deg,'factor':self.fan_factor})
        first=self.observe(pair[0],c,'B4_fan_first',backside=True)
        if first['measure_result']=='no_signal':
            self.stats['fan_second_required']+=1
            second=self.observe(pair[1],c,'B4_fan_second',backside=True)
            if second['measure_result']=='no_signal':
                self.stats['fan_invariant_failures']+=1
                raise InvariantFailure('both SPF-beta branches returned no_signal')
        else:self.stats['fan_first_success']+=1

class G30B2(_G30FanBase): beta_deg=2.; fan_factor=.995
class G30B3(_G30FanBase): beta_deg=3.; fan_factor=.995
class G30B5(_G30FanBase): beta_deg=5.; fan_factor=1.0
class G30B75(_G30FanBase): beta_deg=7.5; fan_factor=1.0
class G30B10(_G30FanBase): beta_deg=10.; fan_factor=1.0
class G30B15(_G30FanBase): beta_deg=15.; fan_factor=1.02
class G30B20(_G30FanBase): beta_deg=20.; fan_factor=1.04
class G30B25(_G30FanBase): beta_deg=25.; fan_factor=1.08
class G30B30(_G30FanBase): beta_deg=30.; fan_factor=1.13

class G30B45(_G30FanBase): beta_deg=45.; fan_factor=1.30
class G30B4(_G30FanBase): beta_deg=4.; fan_factor=1.0
class G30B45x(_G30FanBase): beta_deg=4.5; fan_factor=1.0
class G30B55(_G30FanBase): beta_deg=5.5; fan_factor=1.0
class G30B6(_G30FanBase): beta_deg=6.; fan_factor=1.0
class G30B65(_G30FanBase): beta_deg=6.5; fan_factor=1.0
class G30B8(_G30FanBase): beta_deg=8.; fan_factor=1.0
class G30B5F98(_G30FanBase): beta_deg=5.; fan_factor=.98
class G30B5F99(_G30FanBase): beta_deg=5.; fan_factor=.99
class G30B5F1001(_G30FanBase): beta_deg=5.; fan_factor=1.001
class G30B5F1002(_G30FanBase): beta_deg=5.; fan_factor=1.002
class G30B6F1003(_G30FanBase): beta_deg=6.; fan_factor=1.003
