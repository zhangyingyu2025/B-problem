"""Seven-site discovery certificate and finite 20 m clearance fallback."""
import math


def sites(rho=1130.0,rotation_deg=0.0):
    if not 1122.955833<=rho<=1732.050807: raise ValueError('rho outside guarded seven-site range')
    return [(0.,0.)]+[(rho*math.cos(math.radians(rotation_deg+60*k)),
                       rho*math.sin(math.radians(rotation_deg+60*k))) for k in range(6)]


def certificate(rho=1130.0):
    sites(rho)
    values=[math.sqrt(r*r+rho*rho-2*r*rho*math.cos(math.pi/6)) for r in [1000,1800]]
    return {'rho_m':rho,'annulus_endpoint_distances_m':values,
            'annulus_upper_m':max(values),'center_disk_upper_m':1000,
            'open_route_length_m':6*rho,'open_route_movement_s':6*rho/5,
            'global_minimum_site_count_claim':False}


def clearance_grid(observation,vertices=None,current=(0,0),cell_size=28.0):
    """Grid covers a physical source rectangle, optionally clipped by a B1 bbox.

    It does not alter the B1 angular region. Every cell is covered by its center's
    20 m disk. Finite fallback even without any second direction measurement.
    """
    if not 0<cell_size<20*math.sqrt(2): raise ValueError('cell size must have strict 20 m diagonal margin')
    origin=(observation['x'],observation['y']);theta=math.radians(observation['svd_deg'])
    e=(math.cos(theta),math.sin(theta));n=(-e[1],e[0])
    def local(q):
        u,v=q[0]-origin[0],q[1]-origin[1]
        return (u*e[0]+v*e[1],u*n[0]+v*n[1])
    def world(q): return (origin[0]+q[0]*e[0]+q[1]*n[0],origin[1]+q[0]*e[1]+q[1]*n[1])
    h=1500*math.sin(math.radians(1))
    xmin,xmax,ymin,ymax=0.,1500.,-h,h
    if vertices:
        points=[local(q) for q in vertices]
        bounds=(max(xmin,min(q[0] for q in points)-1e-6),min(xmax,max(q[0] for q in points)+1e-6),
                max(ymin,min(q[1] for q in points)-1e-6),min(ymax,max(q[1] for q in points)+1e-6))
        if bounds[0]<=bounds[1] and bounds[2]<=bounds[3]: xmin,xmax,ymin,ymax=bounds
    nx=max(1,math.ceil((xmax-xmin)/cell_size));ny=max(1,math.ceil((ymax-ymin)/cell_size))
    xs=[xmin+(i+.5)*(xmax-xmin)/nx for i in range(nx)]
    ys=[ymin+(j+.5)*(ymax-ymin)/ny for j in range(ny)]
    routes=[]
    for reverse_x in [False,True]:
        for reverse_y in [False,True]:
            rows=list(reversed(ys)) if reverse_y else ys
            route=[]
            for j,y in enumerate(rows):
                columns=list(reversed(xs)) if (j%2==1)^reverse_x else xs
                route.extend(world((x,y)) for x in columns)
            routes.append(route)
    return min(routes,key=lambda route:math.dist(current,route[0]))
