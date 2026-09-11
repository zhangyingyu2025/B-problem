"""Synthetic deterministic test double, NEVER an official rehearsal result."""
import hashlib
import json
import math
import random


def make_case(seed,count=None,pattern='random'):
    rng=random.Random(seed);count=10+seed%7 if count is None else count
    sources=[]
    channels=rng.sample(range(1,21),count)
    for i,channel in enumerate(channels):
        if pattern=='boundary':
            r=1800;angle=math.radians(30+60*(i%6)+(i//6)*.03)
        elif pattern=='cluster':
            r=rng.uniform(900,1050);angle=rng.uniform(.1,.14)
        elif pattern=='origin' and i==0: r=0;angle=0
        else: r=1800*math.sqrt(rng.random());angle=rng.random()*2*math.pi
        sources.append({'channel':channel,'x':r*math.cos(angle),'y':r*math.sin(angle),
                        'radius':1000 if pattern=='boundary' else rng.uniform(1000,1500)})
    return {'kind':'synthetic_offline','seed':seed,'pattern':pattern,'sources':sources}


class OfflineTransport:
    def __init__(self,case,robot_id='offline-test',error_mode='fixed_field'):
        self._sources={s['channel']:dict(s) for s in case['sources']};self.seed=case.get('seed',0)
        self.robot_id=robot_id;self.error_mode=error_mode;self.cleared=set();self.active=False
        self.position=(0.,0.);self.channel=1;self.vt=0.;self.cache={}
    def __call__(self,path,payload):
        identity=json.dumps([path,payload],sort_keys=True,separators=(',',':'))
        key=payload['request_id']
        if key in self.cache:
            old,response=self.cache[key]
            return response if old==identity else (409,self._body(False))
        if payload.get('robot_id')!=self.robot_id or payload.get('arena_id')!='default': return 200,self._body(False)
        if path=='/enter':
            if self.active: return 200,self._body(False)
            self.active=True
            body=self._body(True);body.update(max_virtual_duration_s=360000,max_real_duration_s=1200,remaining_real_duration_s=1200)
        elif not self.active: return 200,self._body(False)
        elif path=='/exit': self.active=False;body=self._body(True);body['exit_reason']='user_exit'
        elif path in ('/measure','/clear'):
            position=(payload['position']['x'],payload['position']['y']);channel=payload['channel']
            self.vt+=math.dist(position,self.position)/5;self.position=position
            source=self._sources.get(channel);exists=source is not None and channel not in self.cleared
            distance=math.dist(position,(source['x'],source['y'])) if exists else math.inf
            if path=='/measure':
                self.vt+=5+int(channel!=self.channel);self.channel=channel
                body=self._body(True)
                if not exists or distance>source['radius']: body['measure_result']='no_signal'
                elif distance<=5: body['measure_result']='near'
                else:
                    true=math.degrees(math.atan2(source['y']-position[1],source['x']-position[0]))
                    lo=math.ceil((true-1)*100);hi=math.floor((true+1)*100)
                    keytext=f'{self.seed}:{channel}:{position[0]:.9f}:{position[1]:.9f}'
                    field=int(hashlib.sha256(keytext.encode()).hexdigest()[:16],16)
                    if self.error_mode=='endpoints': value=lo if field%2 else hi
                    elif self.error_mode=='zero': value=round(true*100)
                    else: value=lo+field%(hi-lo+1)
                    body.update(measure_result='direction',svd_deg=(value/100)%360)
            else:
                success=exists and distance<=20
                self.vt+=5 if success else 3
                if success: self.cleared.add(channel)
                body=self._body(True);body['clear_result']='success' if success else 'no_target_in_range'
        else: return 404,self._body(False)
        response=(200,body);self.cache[key]=(identity,response);return response
    def _body(self,accepted):
        return {'accepted':accepted,'real_timestamp_ms':0,'virtual_time_s':round(self.vt,6) if accepted else 0}
