"""Compare the independent audit geometry with GEOS on fixed generated cases."""
import json
import math
from pathlib import Path
import random

import shapely
from shapely.geometry import LineString, Polygon
from geometry_audit import convex_distance,convex_intersects,transform


def main():
    rng=random.Random(20261117)
    rectangle=[(-.4,-.3),(.4,-.3),(.4,.3),(-.4,.3)]
    footprint=[(-.21,-.17),(.11,-.17),(.11,.17),(-.21,.17)]
    errors=[]
    for index in range(1000):
        obstacle=transform(rectangle,rng.uniform(-2,2),rng.uniform(-2,2),0.)
        if index%2:
            shape=transform(footprint,rng.uniform(-2,2),rng.uniform(-2,2),rng.uniform(-math.pi,math.pi))
            reference=Polygon(shape)
        else:
            shape=[(rng.uniform(-2,2),rng.uniform(-2,2)),(rng.uniform(-2,2),rng.uniform(-2,2))]
            reference=LineString(shape)
        target=Polygon(obstacle)
        error=abs(convex_distance(shape,obstacle)-reference.distance(target))
        assert error<1e-9,(index,error)
        assert convex_intersects(shape,obstacle)==reference.intersects(target),index
        errors.append(error)
    result=dict(passed=True,fixed_seed=20261117,cases=1000,shapely_version=shapely.__version__,
                maximum_distance_error_m=max(errors),intersection_disagreements=0)
    output=Path(__file__).with_name('geometry_crosscheck.json')
    with output.open('x') as stream:
        json.dump(result,stream,indent=2);stream.write('\n')
    print(json.dumps(result))


if __name__=='__main__':main()
