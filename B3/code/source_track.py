"""Read-only B1/B2 imports and per-channel observation state."""
from dataclasses import dataclass,field
import math
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'B1/code'))
sys.path.insert(0,str(ROOT/'B2/code'))
from b1_geometry import solve_b1
from b2_solver import solve_b2


@dataclass
class SourceTrack:
    channel:int
    observations:list=field(default_factory=list)
    geometry:dict=field(default_factory=dict)
    cleared:bool=False
    near_position:tuple|None=None
    b2_used:bool=False
    invalid_clear_certificate:bool=False
    diagnostics:list=field(default_factory=list)

    def observe(self,position,response):
        kind=response['measure_result']
        if kind=='near': self.near_position=tuple(position);return
        if kind!='direction': return
        record={'x':position[0],'y':position[1],'svd_deg':response['svd_deg']}
        if record in self.observations: return
        self.observations.append(record)
        self.geometry=solve_b1(self.observations)
        if self.geometry['status'] not in ('bounded','point','segment','unbounded'):
            self.diagnostics.append({'code':'b1_'+self.geometry['status'],'details':self.geometry.get('diagnostics',[])})

    def trusted_vertices(self):
        if self.invalid_clear_certificate or self.geometry.get('status') not in ('bounded','point','segment'): return None
        return self.geometry['vertices']

    def clear_point(self,current):
        if self.cleared: return None
        if self.near_position is not None: return self.near_position
        vertices=self.trusted_vertices()
        if not vertices: return None
        if max(math.dist(current,q) for q in vertices)<=20-1e-6: return tuple(current)
        circle=self.geometry.get('minimum_enclosing_circle')
        if circle and circle['radius_m']<=20-1e-6: return tuple(circle['center'])
        return None

    def supplemental_point(self):
        if not self.observations or self.b2_used: return None
        self.b2_used=True
        result=solve_b2(self.observations[0],options={'grid_step_m':250,'refinement_levels':1,'max_candidates':40,
                                                   'max_intervals':64,'final_intervals':192})
        best=result.get('recommended')
        if best and best['high_precision_check']['status']=='passed': return tuple(best['point_world_m'])
        self.diagnostics.append({'code':'b2_no_verified_recommendation','status':result['status']})
        return None

    def snapshot(self):
        circle=self.geometry.get('minimum_enclosing_circle') or {}
        return {'channel':self.channel,'state':'CLEARED' if self.cleared else 'CLEARABLE' if self.clear_point((0,0)) is not None else 'LOCATING',
                'observation_count':len(self.observations),'b1_status':self.geometry.get('status'),
                'diameter_m':self.geometry.get('diameter_m'),'mec_radius_m':circle.get('radius_m'),
                'b2_used':self.b2_used,'diagnostics':self.diagnostics}
