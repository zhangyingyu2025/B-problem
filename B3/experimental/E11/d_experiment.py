from pathlib import Path
HERE=Path(__file__).resolve().parent
(HERE/'results').mkdir(exist_ok=True)
import math, random, hashlib, json, statistics, time
from functools import lru_cache

RHO=1130.0
ALPHA=1.0

# --- synthetic environment identical to B3/offline_environment.py ---
def make_case(seed,count=None,pattern='random'):
    rng=random.Random(seed);count=10+seed%7 if count is None else count
    sources=[];channels=rng.sample(range(1,21),count)
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

class Env:
    def __init__(self,case,error_mode='fixed_field'):
        self.sources={s['channel']:dict(s) for s in case['sources']}; self.seed=case['seed']; self.error_mode=error_mode
        self.cleared=set(); self.pos=(0.,0.); self.channel=1; self.vt=0.; self.stats={'movement_m':0.,'measurements':0,'switches':0,'clear_attempts':0,'clear_successes':0,'clear_failures':0}
    def move_cost(self,p):
        d=math.dist(self.pos,p); self.vt+=d/5; self.stats['movement_m']+=d; self.pos=tuple(p)
    def measure(self,p,c):
        self.move_cost(p); self.vt += 5 + (c!=self.channel); self.stats['measurements']+=1; self.stats['switches']+=int(c!=self.channel); self.channel=c
        s=self.sources.get(c); exists=s is not None and c not in self.cleared
        d=math.dist(p,(s['x'],s['y'])) if exists else math.inf
        if not exists or d>s['radius']: return {'measure_result':'no_signal'}
        if d<=5: return {'measure_result':'near'}
        true=math.degrees(math.atan2(s['y']-p[1],s['x']-p[0]))
        lo=math.ceil((true-1)*100); hi=math.floor((true+1)*100)
        keytext=f'{self.seed}:{c}:{p[0]:.9f}:{p[1]:.9f}'; field=int(hashlib.sha256(keytext.encode()).hexdigest()[:16],16)
        if self.error_mode=='endpoints': value=lo if field%2 else hi
        elif self.error_mode=='zero': value=round(true*100)
        else: value=lo+field%(hi-lo+1)
        return {'measure_result':'direction','svd_deg':(value/100)%360}
    def clear(self,p,c):
        self.move_cost(p); self.stats['clear_attempts']+=1
        s=self.sources.get(c); exists=s is not None and c not in self.cleared
        d=math.dist(p,(s['x'],s['y'])) if exists else math.inf; ok=exists and d<=20
        self.vt += 5 if ok else 3
        if ok: self.cleared.add(c); self.stats['clear_successes']+=1
        else: self.stats['clear_failures']+=1
        return ok

# --- geometry: conservative disk polygon + bearing wedge clipping ---
def disk_outer_polygon(radius=1800,n=180):
    rr=radius/math.cos(math.pi/n)
    return [(rr*math.cos(2*math.pi*k/n),rr*math.sin(2*math.pi*k/n)) for k in range(n)]
BASE_POLY=disk_outer_polygon()

def unit(deg):
    a=math.radians(deg); return math.cos(a),math.sin(a)

def wedge_halfplanes(obs):
    x,y,t=obs['x'],obs['y'],obs['svd_deg']; low=unit(t-ALPHA); high=unit(t+ALPHA)
    # a*x+b*y <= c
    out=[]
    for a,b in ((low[1],-low[0]),(-high[1],high[0])): out.append((a,b,a*x+b*y))
    return out

def clip(poly,h,eps=1e-9):
    a,b,c=h; out=[]
    if not poly: return out
    def val(p): return a*p[0]+b*p[1]-c
    prev=poly[-1]; vp=val(prev); inp=vp<=eps
    for cur in poly:
        vc=val(cur); inc=vc<=eps
        if inc!=inp:
            den=vp-vc
            if abs(den)>1e-18:
                t=vp/den; out.append((prev[0]+t*(cur[0]-prev[0]),prev[1]+t*(cur[1]-prev[1])))
        if inc: out.append(cur)
        prev, vp, inp = cur, vc, inc
    return out

def feasible_polygon(observations):
    if not observations: return []
    poly=list(BASE_POLY)
    for o in observations:
        for h in wedge_halfplanes(o):
            poly=clip(poly,h)
            if not poly: return []
    return poly

# Smallest enclosing circle (Nayuki-style deterministic shuffled)
def circle2(a,b):
    c=((a[0]+b[0])/2,(a[1]+b[1])/2); return c,math.dist(a,b)/2

def circle3(a,b,c):
    ax,ay=a; bx,by=b; cx,cy=c
    d=2*(ax*(by-cy)+bx*(cy-ay)+cx*(ay-by))
    if abs(d)<1e-12: return None
    ux=((ax*ax+ay*ay)*(by-cy)+(bx*bx+by*by)*(cy-ay)+(cx*cx+cy*cy)*(ay-by))/d
    uy=((ax*ax+ay*ay)*(cx-bx)+(bx*bx+by*by)*(ax-cx)+(cx*cx+cy*cy)*(bx-ax))/d
    o=(ux,uy); return o,math.dist(o,a)

def contains(circle,p,eps=1e-7): return math.dist(circle[0],p)<=circle[1]+eps

def mec(points):
    pts=list(points)
    if not pts: return None
    # deterministic pseudo-shuffle to avoid bad order
    rng=random.Random(12345); rng.shuffle(pts)
    c=(pts[0],0.0)
    for i,p in enumerate(pts):
        if contains(c,p): continue
        c=(p,0.0)
        for j,q in enumerate(pts[:i]):
            if contains(c,q): continue
            c=circle2(p,q)
            for r in pts[:j]:
                if contains(c,r): continue
                cc=circle3(p,q,r)
                if cc is not None: c=cc
    return c

def track_geometry(track):
    poly=feasible_polygon(track['dirs'])
    track['poly']=poly; track['mec']=mec(poly) if poly else None
    return track['mec']

def clearable(track):
    return track.get('mec') is not None and track['mec'][1] <= 19.999

# exact open TSP through target points from current (n<=16)
def open_tsp_order(start,items):
    # items list (channel, point)
    n=len(items)
    if n<=1: return items
    dist0=[math.dist(start,p) for _,p in items]
    d=[[math.dist(items[i][1],items[j][1]) for j in range(n)] for i in range(n)]
    dp={}
    parent={}
    for i in range(n): dp[(1<<i,i)]=dist0[i]
    for mask in range(1,1<<n):
        for j in range(n):
            key=(mask,j)
            if key not in dp: continue
            v=dp[key]
            rem=((1<<n)-1)^mask
            k=0
            while rem:
                lb=rem & -rem; k=lb.bit_length()-1; nm=mask|lb
                nv=v+d[j][k]
                if nv < dp.get((nm,k),1e100): dp[(nm,k)]=nv; parent[(nm,k)]=j
                rem-=lb
    full=(1<<n)-1; end=min(range(n),key=lambda j:dp[(full,j)])
    seq=[];mask=full;j=end
    while True:
        seq.append(j); prev=parent.get((mask,j))
        if prev is None: break
        mask ^= 1<<j; j=prev
    seq.reverse(); return [items[i] for i in seq]

class DSolver:
    def __init__(self,case,aggressive=True,shared_rounds=2):
        self.env=Env(case,'endpoints' if case['pattern']=='boundary' else 'fixed_field'); self.tracks={}; self.checked={c:set() for c in range(1,21)}
        self.sites=[(0.,0.)]+[(RHO*math.cos(math.radians(60*k)),RHO*math.sin(math.radians(60*k))) for k in range(6)]
        self.aggressive=aggressive; self.shared_rounds=shared_rounds; self.stats={'onroute_attempts':0,'onroute_dirs':0,'supplemental_points':0}
    def discovery_complete(self):
        if len(self.tracks)>=16: return True
        return all(c in self.tracks or len(self.checked[c])>=len(self.sites) for c in range(1,21))
    def handle(self,p,c,res,site_idx=None,reason=''):
        if res['measure_result']=='no_signal':
            if site_idx is not None and c not in self.tracks: self.checked[c].add(site_idx)
            t=self.tracks.get(c)
            if t is not None: t['nosig'].append(tuple(p))
            return
        t=self.tracks.setdefault(c,{'channel':c,'dirs':[],'nosig':[],'near':None,'poly':[],'mec':None})
        if res['measure_result']=='near': t['near']=tuple(p)
        else:
            rec={'x':p[0],'y':p[1],'svd_deg':res['svd_deg']}
            if rec not in t['dirs']: t['dirs'].append(rec); track_geometry(t)
    def measure(self,p,c,site_idx=None,reason=''):
        res=self.env.measure(p,c); self.handle(p,c,res,site_idx,reason); return res
    def maybe_clear_near(self,t):
        if t['channel'] in self.env.cleared: return
        if t.get('near') is not None: self.env.clear(t['near'],t['channel'])
    def run_search(self):
        for idx,site in enumerate(self.sites):
            if self.discovery_complete(): break
            # unknown discovery channels, snake ordering
            unknown=[c for c in range(1,21) if c not in self.tracks and len(self.checked[c])<len(self.sites)]
            unknown=sorted(unknown,reverse=bool(idx%2))
            if self.env.channel in unknown: unknown.remove(self.env.channel); unknown.insert(0,self.env.channel)
            for c in unknown:
                if self.discovery_complete(): break
                self.measure(site,c,idx,'discovery')
                if c in self.tracks: self.maybe_clear_near(self.tracks[c])
            if self.aggressive:
                # measure every discovered uncleared channel at this site unless already direction/no-signal here
                chans=[]
                for c,t in self.tracks.items():
                    if c in self.env.cleared: continue
                    if any(math.dist(site,(o['x'],o['y']))<1e-7 for o in t['dirs']) or any(math.dist(site,q)<1e-7 for q in t['nosig']): continue
                    chans.append(c)
                chans.sort()
                if self.env.channel in chans: chans.remove(self.env.channel); chans.insert(0,self.env.channel)
                for c in chans:
                    self.stats['onroute_attempts']+=1
                    res=self.measure(site,c,None,'onroute')
                    if res['measure_result']=='direction': self.stats['onroute_dirs']+=1
                    self.maybe_clear_near(self.tracks[c])
    def supplemental_candidate(self,t,sign):
        # conservative B2-like local point relative first bearing; try a small fixed bank and choose nearest current
        o=t['dirs'][0]; th=math.radians(o['svd_deg']); e=(math.cos(th),math.sin(th)); n=(-e[1],e[0])
        candidates=[]
        alpha=math.radians(1); margin=math.radians(3.1); tn=math.tan(alpha)
        for a in (550,700,800,900):
            for babs in (350,450,550,650):
                b=sign*babs
                q=a*a+b*b
                # analytic C_cert and effective separation conditions adapted from B2
                if q>1000**2: continue
                if q>2000*(a*math.cos(alpha)-abs(b)*math.sin(alpha)): continue
                threshold=1500*tn+(1500-a)*math.tan(margin)
                if abs(b)<=threshold: continue
                if abs(b)*math.cos(alpha)-a*math.sin(alpha)<=5: continue
                p=(o['x']+a*e[0]+b*n[0],o['y']+a*e[1]+b*n[1])
                candidates.append(p)
        return min(candidates,key=lambda p:math.dist(self.env.pos,p)) if candidates else None
    def shared_supplement(self):
        # Greedy rounds: candidate bank from all unresolved tracks. Score how many unresolved tracks are likely observable
        # using current angular proxy; one move may collect multiple bearings.
        for _ in range(self.shared_rounds):
            unresolved=[t for t in self.tracks.values() if t['channel'] not in self.env.cleared and not clearable(t) and t['dirs']]
            if not unresolved: return
            bank=[]
            for t in unresolved:
                for sign in (-1,1):
                    p=self.supplemental_candidate(t,sign)
                    if p: bank.append(p)
            if not bank: return
            def score(p):
                hits=0; quality=0.0
                for t in unresolved:
                    # proxy from midpoint 750m along first bearing, probable reception + angular separation
                    o=t['dirs'][0]; th=math.radians(o['svd_deg']); proxy=(o['x']+750*math.cos(th),o['y']+750*math.sin(th))
                    if math.dist(p,proxy)<=1450:
                        a1=math.degrees(math.atan2(proxy[1]-p[1],proxy[0]-p[0])); sep=abs((a1-o['svd_deg']+180)%360-180)
                        if sep>=8: hits+=1; quality+=min(sep,90)/90
                detour=math.dist(self.env.pos,p)/5
                return hits*120 + quality*30 - detour
            p=max(bank,key=score); self.stats['supplemental_points']+=1
            chans=sorted(t['channel'] for t in unresolved)
            if self.env.channel in chans: chans.remove(self.env.channel); chans.insert(0,self.env.channel)
            for c in chans:
                self.measure(p,c,None,'shared_supplement')
                self.maybe_clear_near(self.tracks[c])
    def finish(self):
        # shared supplements first
        self.shared_supplement()
        # If still unresolved, do per-source safe supplements one by one, but at each point measure all unresolved tracks.
        guard=0
        while True:
            unresolved=[t for t in self.tracks.values() if t['channel'] not in self.env.cleared and not clearable(t)]
            if not unresolved: break
            guard+=1
            if guard>40: break
            t=min(unresolved,key=lambda t:math.dist(self.env.pos,(t['dirs'][0]['x'],t['dirs'][0]['y'])) if t['dirs'] else 1e9)
            if not t['dirs']: break
            opts=[self.supplemental_candidate(t,s) for s in (-1,1)]; opts=[p for p in opts if p]
            if not opts: break
            p=min(opts,key=lambda p:math.dist(self.env.pos,p)); self.stats['supplemental_points']+=1
            # measure all unresolved at this point, owner first
            chans=[t['channel']]+[u['channel'] for u in unresolved if u['channel']!=t['channel']]
            for c in chans:
                self.measure(p,c,None,'per_source_supplement')
                self.maybe_clear_near(self.tracks[c])
            # avoid repeatedly same owner point if no progress by marking? geometry may still not clear, alternate sign next loop unavailable
            if guard>2*len(self.tracks): break
        # clear all geometrically clearable in exact open TSP order
        items=[]
        for c,t in self.tracks.items():
            if c in self.env.cleared: continue
            if clearable(t): items.append((c,t['mec'][0]))
        for c,p in open_tsp_order(self.env.pos,items): self.env.clear(p,c)
        # finite fallback grid from first direction for any leftovers (safe but expensive)
        for c,t in list(self.tracks.items()):
            if c in self.env.cleared: continue
            if not t['dirs']: continue
            o=t['dirs'][0]; th=math.radians(o['svd_deg']); e=(math.cos(th),math.sin(th)); n=(-e[1],e[0]); h=1500*math.sin(math.radians(1))
            nx=54; ny=2
            xs=[(i+.5)*1500/nx for i in range(nx)]; ys=[-h/2,h/2]
            route=[]
            for j,y in enumerate(ys):
                cols=xs if j%2==0 else list(reversed(xs))
                for x in cols: route.append((o['x']+x*e[0]+y*n[0],o['y']+x*e[1]+y*n[1]))
            if route and math.dist(self.env.pos,route[-1])<math.dist(self.env.pos,route[0]): route=list(reversed(route))
            for p in route:
                if self.env.clear(p,c): break
    def run(self):
        self.run_search(); self.finish()
        return {'seed':self.env.seed,'cleared':len(self.env.cleared),'total':len(self.env.sources),'vt':self.env.vt,'movement':self.env.stats['movement_m'],**self.env.stats,**self.stats}

def cases12():
    cs=[make_case(20260911+i) for i in range(8)]
    cs += [make_case(20261001,pattern='boundary'),make_case(20261002,pattern='boundary'),make_case(20261003,pattern='cluster'),make_case(20261004,pattern='origin')]
    return cs

def experiment(aggressive=True,shared_rounds=2):
    rows=[]
    for i,c in enumerate(cases12()):
        t=time.perf_counter(); r=DSolver(c,aggressive,shared_rounds).run(); r['wall']=time.perf_counter()-t; rows.append(r)
        print(i+1,c['pattern'],r)
    print('ALL CLEAR',all(r['cleared']==r['total'] for r in rows))
    print('mean total',statistics.mean(r['vt'] for r in rows))
    print('mean per source arithmetic',statistics.mean(r['vt']/r['total'] for r in rows))
    print('weighted per source',sum(r['vt'] for r in rows)/sum(r['total'] for r in rows))
    print('mean movement',statistics.mean(r['movement'] for r in rows))
    print('mean measurements',statistics.mean(r['measurements'] for r in rows))
    print('mean switches',statistics.mean(r['switches'] for r in rows))
    return rows

if __name__=='__main__':
    rows=experiment(True,2)
    with open(str(HERE/'results'/'d_experiment_results.json'),'w',encoding='utf-8') as f: json.dump(rows,f,ensure_ascii=False,indent=2)

def angle_diff(a,b): return abs((a-b+180)%360-180)

def candidate_particles(t,step=3.0):
    if not t['dirs']: return []
    first=t['dirs'][0]; th=math.radians(first['svd_deg']); e=(math.cos(th),math.sin(th)); n=(-e[1],e[0]); tan=math.tan(math.radians(ALPHA))
    # grid in first measured wedge bounding strip; cell centers-ish plus axes
    pts=[]
    x=step/2
    while x<=1500:
        half=x*tan
        # ensure at least y=0 plus regular bands
        ys=[0.0]
        k=1
        while k*step<=half+step/2:
            ys += [k*step,-k*step]; k+=1
        for y in ys:
            if abs(y)>half+1e-9: continue
            p=(first['x']+x*e[0]+y*n[0],first['y']+x*e[1]+y*n[1])
            if math.hypot(*p)>1800+2: continue
            sig=[]; ok=True
            for o in t['dirs']:
                d=math.dist(p,(o['x'],o['y']))
                if d<=5 or d>1500+2: ok=False; break
                bearing=math.degrees(math.atan2(p[1]-o['y'],p[0]-o['x']))%360
                if angle_diff(bearing,o['svd_deg'])>ALPHA+0.12: ok=False; break
                sig.append(d)
            if not ok: continue
            lower=max([1000.0]+sig)
            if lower>1500+2: continue
            if t['nosig']:
                upper=min([1500.0]+[math.dist(p,q) for q in t['nosig']])
                if lower>=upper+3.0: continue
            pts.append(p)
        x+=step
    return pts

def particle_bbox(t,step=3.0):
    pts=candidate_particles(t,step)
    if not pts: return None,[]
    xs=[p[0] for p in pts]; ys=[p[1] for p in pts]; c=((min(xs)+max(xs))/2,(min(ys)+max(ys))/2); r=max(math.dist(c,p) for p in pts)
    return (c,r),pts
