"""Physical outer source set and explicitly separated candidate domains."""
import math
from geometry import dot, sub, unit


def finite(value, name):
    if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite real number")
    return float(value)


class Problem:
    def __init__(self, observation, error_deg=1.0, domain_center=(0,0), domain_radius=1800.0,
                 min_range=1000.0, max_range=1500.0, near_range=5.0, angle_margin_deg=0.1):
        if not isinstance(observation,dict) or set(observation)!={"x","y","svd_deg"}:
            raise ValueError("observation requires exactly x, y, svd_deg")
        self.s1=(finite(observation['x'],'x'),finite(observation['y'],'y'))
        self.theta=finite(observation['svd_deg'],'svd_deg')
        if not 0<=self.theta<360: raise ValueError("svd_deg must lie in [0,360)")
        self.alpha=finite(error_deg,'error_deg')
        self.rmin=finite(min_range,'min_range');self.rmax=finite(max_range,'max_range')
        self.near=finite(near_range,'near_range');self.radius=finite(domain_radius,'domain_radius')
        self.margin=finite(angle_margin_deg,'angle_margin_deg')
        if not (0<self.alpha<=5 and 0<=self.margin<5 and 0<self.near<self.rmin<=self.rmax and self.radius>0):
            raise ValueError("require 0<alpha<=5, 0<=margin<5, 0<near<min_range<=max_range, radius>0")
        self.e=unit(self.theta);self.n=(-self.e[1],self.e[0])
        if len(domain_center)!=2: raise ValueError("domain_center must have two coordinates")
        self.domain_center=tuple(finite(v,'domain_center') for v in domain_center)
        self.center=self.to_local(self.domain_center)
        self.eps=1e-10*max(1,self.radius,self.rmax,math.hypot(*self.center))
        self.base_points=self._boundary_points()
        if not self.base_points:
            self.status='inconsistent_observation'
        else:
            maximum=max(math.hypot(*q) for q in self.base_points)
            self.status=('inconsistent_observation' if maximum<self.near-self.eps else
                         'numerically_uncertain' if maximum<=self.near+self.eps else 'ready')

    def to_local(self,p):
        q=sub(p,self.s1)
        return (dot(q,self.e),dot(q,self.n))

    def to_world(self,p):
        return (self.s1[0]+p[0]*self.e[0]+p[1]*self.n[0],
                self.s1[1]+p[0]*self.e[1]+p[1]*self.n[1])

    def in_outer(self,q):
        x,y=q;t=math.tan(math.radians(self.alpha));eps=self.eps
        return (x>=-eps and abs(y)<=t*x+eps and math.hypot(x,y)<=self.rmax+eps
                and math.dist(q,self.center)<=self.radius+eps)

    def in_physical(self,q):
        return self.in_outer(q) and math.hypot(*q)>self.near

    def _boundary_points(self):
        """Endpoints of boundary arcs plus radial extrema, no source grid."""
        points=[(0.0,0.0)]
        circles=[((0.,0.),self.rmax),(self.center,self.radius)]
        for angle in (-self.alpha,self.alpha):
            u=unit(angle)
            points.append((self.rmax*u[0],self.rmax*u[1]))
            c=self.center;v=dot(u,c);disc=v*v-dot(c,c)+self.radius**2
            if disc>=-self.eps*max(self.radius,1):
                root=math.sqrt(max(0,disc))
                for r in (v-root,v+root):
                    if r>=-self.eps: points.append((max(0,r)*u[0],max(0,r)*u[1]))
        d=math.hypot(*self.center)
        if d>self.eps and abs(self.rmax-self.radius)-self.eps<=d<=self.rmax+self.radius+self.eps:
            a=(self.rmax**2-self.radius**2+d*d)/(2*d)
            h2=self.rmax**2-a*a
            if h2>=-self.eps*self.rmax:
                h=math.sqrt(max(0,h2));u=(self.center[0]/d,self.center[1]/d)
                for sign in (-1,1):
                    points.append((a*u[0]-sign*h*u[1],a*u[1]+sign*h*u[0]))
        for c,r in circles:
            angles=[0,90,180,270]
            if math.hypot(*c)>self.eps:
                phi=math.degrees(math.atan2(c[1],c[0]));angles += [phi,phi+180]
            for angle in angles:
                u=unit(angle);points.append((c[0]+r*u[0],c[1]+r*u[1]))
        return [q for q in points if self.in_outer(q)]

    def bearing_interval(self,p):
        """View of convex H=D0 intersect W1 intersect B(s1,rmax).

        p in effective_domain lies strictly outside H. Circle tangencies and
        boundary endpoints attain the angular extrema. H contains K; excluding
        the near disk is deliberately not used to claim a sharper exact K view.
        """
        points=list(self.base_points)
        for c,r in [((0.,0.),self.rmax),(self.center,self.radius)]:
            v=sub(p,c);d2=dot(v,v)
            if d2<r*r: continue
            factor=r*r/d2 if d2 else 0
            side=r*math.sqrt(max(0,d2-r*r))/d2 if d2 else 0
            for sign in (-1,1):
                q=(c[0]+factor*v[0]-sign*side*v[1],c[1]+factor*v[1]+sign*side*v[0])
                if self.in_outer(q): points.append(q)
        if not points: raise ArithmeticError("empty physical outer source set")
        # Upper/lower lobes keep all true directions in one open half circle.
        angles=[math.degrees(math.atan2(q[1]-p[1],q[0]-p[0])) for q in points]
        lo_i=min(range(len(points)),key=lambda i:angles[i]);hi_i=max(range(len(points)),key=lambda i:angles[i])
        lo,hi=angles[lo_i],angles[hi_i]
        if hi-lo>=180: raise ArithmeticError("view interval crosses unsupported half circle")
        return {"true_deg":[lo,hi],"observed_deg":[lo-self.alpha,hi+self.alpha],
                "endpoint_sources_local":[points[lo_i],points[hi_i]],
                "source_set":"H_convex_outer_of_K", "conservative":True}

    def analytic_safe(self,p):
        a,b=p;q=a*a+b*b;c,s=unit(self.alpha)
        slack=min(self.rmin**2-q,2*self.rmin*(a*c-abs(b)*s)-q)
        # Strict inward guard only excludes marginal search points, not an
        # expansion of the physical error or signal range.
        return slack>self.eps*self.rmin

    def effective_domain(self,p):
        """Implemented sufficient subset C0_sub of the theoretical C0."""
        a,b=p;t=math.tan(math.radians(self.alpha))
        threshold=self.rmax*t+(self.rmax-a)*math.tan(math.radians(3*self.alpha+self.margin))
        c,s=unit(self.alpha)
        return (self.analytic_safe(p) and abs(b)>threshold+self.eps
                and abs(b)*c-a*s>self.near+self.eps)

    def witness(self,p,observed,interval):
        true=min(interval['true_deg'][1],max(interval['true_deg'][0],observed))
        u=unit(true);lo=0.;hi=math.inf
        for c,r in [((0.,0.),self.rmax),(self.center,self.radius)]:
            v=sub(p,c);b=dot(v,u);disc=b*b-dot(v,v)+r*r
            if disc < -self.eps*max(r,1): return None
            root=math.sqrt(max(0,disc));lo=max(lo,-b-root);hi=min(hi,-b+root)
        t=math.tan(math.radians(self.alpha))
        for normal in [(-t,1),(-t,-1)]:
            z=dot(normal,u);rhs=-dot(normal,p)
            if abs(z)<1e-14:
                if rhs < -self.eps: return None
            elif z>0: hi=min(hi,rhs/z)
            else: lo=max(lo,rhs/z)
        if lo>hi+self.eps or not math.isfinite(hi): return None
        for r in ((lo+hi)/2,lo,hi):
            q=(p[0]+r*u[0],p[1]+r*u[1])
            if self.in_physical(q):
                return {"target_world_m":self.to_world(q),"true_bearing_local_deg":true,
                        "second_error_deg":observed-true,
                        "compatible_radius_m":max(self.rmin,math.hypot(*q))}
        return None

    def domain_descriptions(self):
        return {
            "physical_source_K":{"definition":"D0 ∩ W1 ∩ {5 < r1 <= 1500}; use configured radii",
                                 "status":self.status},
            "guaranteed_reception_C_safe":{"definition":"intersection over G in K of B(G,max(r_min,r1(G)))",
                                         "representation":"implicit exact mathematical definition; full boundary not computed",
                                         "may_return_near":True},
            "analytic_safe_C_cert":{"definition":"B(s1,r_min) ∩ B(s1+r_min*u_minus,r_min) ∩ B(s1+r_min*u_plus,r_min)",
                                    "relation":"sufficient subset of C_safe; inward numeric guard used for search"},
            "effective_direction_C0":{"definition":"C_cert with dist(p,T)>near and angular separation >3alpha+margin",
                                      "implemented_subset":"C0_sub: abs(b)>L*tan(alpha)+(L-a)*tan(3alpha+margin) and abs(b)*cos(alpha)-a*sin(alpha)>near",
                                      "guarantees":"normal second reading and bounded angular intersection",
                                      "complete_domain":False},
            "preferred_detection_region":{"representation":"finite evaluated candidates; no continuous near-optimal-region certificate",
                                          "metric":"worst diameter over conservative one-dimensional H reading interval"}}
