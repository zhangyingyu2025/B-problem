"""Serial official HTTP adapter with idempotent transport retries and logs."""
import json
import math
from pathlib import Path
import time
import unicodedata
from urllib.error import HTTPError,URLError
from urllib.parse import urlsplit
from urllib.request import Request,urlopen
import uuid


class ProtocolError(RuntimeError): pass
class BudgetExceeded(ProtocolError): pass


class HTTPTransport:
    def __init__(self,base_url='http://127.0.0.1:2026',timeout=5.0):
        parsed=urlsplit(base_url)
        if parsed.scheme!='http' or parsed.hostname not in ('127.0.0.1','localhost','::1') or parsed.path not in ('','/') or parsed.query or parsed.fragment:
            raise ValueError('official simulator transport must use a local HTTP base URL')
        self.base_url=base_url.rstrip('/');self.timeout=timeout
    def __call__(self,path,payload):
        request=Request(self.base_url+path,data=json.dumps(payload,allow_nan=False,separators=(',',':')).encode('utf-8'),
                        headers={'Content-Type':'application/json'},method='POST')
        try:
            with urlopen(request,timeout=self.timeout) as response:
                return response.status,json.loads(response.read().decode('utf-8'))
        except HTTPError as exc:
            try: body=json.loads(exc.read().decode('utf-8'))
            except (ValueError,UnicodeError): body={'error':'non-JSON HTTP error response'}
            return exc.code,body


class Client:
    def __init__(self,transport,robot_id,log_path=None,retries=2,clock=time.monotonic,session_id=None):
        if not isinstance(robot_id,str) or not 1<=len(robot_id.encode('utf-8'))<=64 or any(unicodedata.category(c) in ('Cc','Cf') for c in robot_id):
            raise ValueError('robot_id must be the valid competition team identifier')
        self.transport=transport;self.robot_id=robot_id;self.retries=retries;self.clock=clock
        self.session_id=session_id or uuid.uuid4().hex;self.serial=0
        self.position=(0.,0.);self.channel=1;self.virtual_time_s=0.;self.active=False
        self.deadline=math.inf;self.max_virtual=360000.;self.started=None
        self.log_path=Path(log_path) if log_path else None;self.events=[]
        self.stats={'movement_m':0.,'measurements':0,'switches':0,'clear_attempts':0,'clear_successes':0,'clear_failures':0,'transport_retries':0}

    def record(self,event):
        event={'wall_elapsed_s':0 if self.started is None else self.clock()-self.started,**event}
        self.events.append(event)
        if self.log_path:
            self.log_path.parent.mkdir(parents=True,exist_ok=True)
            with self.log_path.open('a',encoding='utf-8') as file:
                file.write(json.dumps(event,ensure_ascii=False,allow_nan=False)+'\n')

    def _call(self,path,position=None,channel=None,reason=''):
        if path!='/enter' and not self.active: raise ProtocolError('session is not active')
        if path!='/enter' and self.clock()>=self.deadline-2: raise BudgetExceeded('real-time budget nearly exhausted')
        payload={'arena_id':'default','robot_id':self.robot_id,'request_id':f'{self.session_id}-{self.serial}'}
        self.serial+=1
        if position is not None:
            if len(position)!=2 or any(isinstance(v,bool) or not isinstance(v,(float,int)) or not math.isfinite(v) or abs(v)>2000000 for v in position):
                raise ValueError('invalid position')
            if isinstance(channel,bool) or not isinstance(channel,int) or not 1<=channel<=20: raise ValueError('invalid channel')
            payload.update(position={'x':position[0],'y':position[1]},channel=channel)
            if self.virtual_time_s+math.dist(self.position,position)/5+6>=self.max_virtual:
                raise BudgetExceeded('virtual-time budget nearly exhausted')
        before={'position':self.position,'channel':self.channel,'virtual_time_s':self.virtual_time_s}
        sent_at=self.clock()
        for attempt in range(self.retries+1):
            try:
                status,body=self.transport(path,payload)
                break
            except (URLError,TimeoutError,ConnectionError,OSError) as exc:
                self.record({'kind':'transport_failure','path':path,'payload':payload,'attempt':attempt,'error':str(exc)})
                if attempt==self.retries: raise ProtocolError('transport outcome unconfirmed; abort without a new action') from exc
                self.stats['transport_retries']+=1
        if status!=200 or not isinstance(body,dict) or body.get('accepted') is not True:
            self.record({'kind':'rejected','path':path,'payload':payload,'http_status':status,'response':body,'before':before})
            raise ProtocolError(f'{path} rejected: HTTP {status}; state not advanced locally')
        def number(key):
            value=body.get(key)
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<0:
                raise ProtocolError('invalid response field: '+key)
            return float(value)
        try:
            vt=number('virtual_time_s');number('real_timestamp_ms')
            if vt<self.virtual_time_s-1e-6: raise ProtocolError('virtual time moved backwards')
            if path=='/enter':
                remaining=number('remaining_real_duration_s');maximum=number('max_virtual_duration_s');number('max_real_duration_s')
                if remaining>1200: raise ProtocolError('invalid remaining real duration')
            elif path=='/measure':
                if body.get('measure_result') not in ('direction','near','no_signal'): raise ProtocolError('invalid measure_result')
                if body['measure_result']=='direction' and not 0<=number('svd_deg')<360: raise ProtocolError('invalid bearing')
                if body['measure_result']!='direction' and 'svd_deg' in body: raise ProtocolError('unexpected bearing without direction')
            elif path=='/clear' and body.get('clear_result') not in ('success','no_target_in_range'): raise ProtocolError('invalid clear_result')
            elif path=='/exit' and body.get('exit_reason')!='user_exit': raise ProtocolError('invalid exit_reason')
        except ProtocolError as exc:
            self.record({'kind':'invalid_accepted_response','path':path,'payload':payload,'response':body,'error':str(exc)})
            raise
        if path=='/enter':
            self.active=True;self.started=sent_at;self.deadline=sent_at+remaining;self.max_virtual=maximum
        elif path in ('/measure','/clear'):
            self.stats['movement_m']+=math.dist(self.position,position)
            self.position=tuple(position)
            if path=='/measure':
                self.stats['measurements']+=1;self.stats['switches']+=int(channel!=self.channel);self.channel=channel
            else:
                self.stats['clear_attempts']+=1
                self.stats['clear_successes' if body['clear_result']=='success' else 'clear_failures']+=1
        else: self.active=False
        self.virtual_time_s=vt
        self.record({'kind':'action','path':path,'payload':payload,'response':body,'before':before,
                     'after':{'position':self.position,'channel':self.channel,'virtual_time_s':vt},'decision_reason':reason})
        return body
    def enter(self):
        if self.active: raise ProtocolError('already entered')
        return self._call('/enter')
    def measure(self,position,channel,reason=''): return self._call('/measure',position,channel,reason)
    def clear(self,position,channel,reason=''): return self._call('/clear',position,channel,reason)
    def exit(self): return self._call('/exit')
