"""Independent convex geometry for rectangular rules and the sampled footprint.

Separating axes and endpoint-to-edge distances; no production gate imports.
Touching within 1e-10 m is conservatively counted as intersection.
"""
import math

EPS = 1e-10


def point_segment_distance(point, left, right):
    dx=right[0]-left[0];dy=right[1]-left[1]
    length=dx*dx+dy*dy
    t=max(0.,min(1.,((point[0]-left[0])*dx+(point[1]-left[1])*dy)/length)) if length else 0.
    return math.hypot(point[0]-left[0]-t*dx,point[1]-left[1]-t*dy)


def edges(points):
    return list(zip(points,points[1:]+points[:1]))


def convex_intersects(left,right):
    for p,q in edges(left)+edges(right):
        nx=-(q[1]-p[1]);ny=q[0]-p[0]
        norm=math.hypot(nx,ny)
        if not norm:continue
        nx/=norm;ny/=norm
        a=[x*nx+y*ny for x,y in left]
        b=[x*nx+y*ny for x,y in right]
        if max(a)<min(b)-EPS or max(b)<min(a)-EPS:
            return False
    return True


def convex_distance(left,right):
    if convex_intersects(left,right):return 0.
    return min([point_segment_distance(p,a,b) for p in left for a,b in edges(right)]+
               [point_segment_distance(p,a,b) for p in right for a,b in edges(left)])


def rectangle(points):
    points=[tuple(p[:2]) for p in points]
    assert len(points)==4 and len(set(points))==4
    xs=[p[0] for p in points];ys=[p[1] for p in points]
    assert set(points)=={(x,y) for x in (min(xs),max(xs)) for y in (min(ys),max(ys))}
    assert all(a[0]==b[0] or a[1]==b[1] for a,b in edges(points))
    return points


def transform(points,x,y,yaw):
    c=math.cos(yaw);s=math.sin(yaw)
    return [(x+c*px-s*py,y+s*px+c*py) for px,py in points]
