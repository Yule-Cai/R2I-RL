"""Bounded offline map preparation from declared source collisions.

Consumes a content-pinned projection_spec.json, including original model paths,
world poses and an explicit operational ROI. Native scans must validate the map.
"""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import shapely
import trimesh


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def pose(text):
    x,y,z,r,p,a=map(float,(text or '0 0 0 0 0 0').split())
    cr,sr=math.cos(r),math.sin(r);cp,sp=math.cos(p),math.sin(p);ca,sa=math.cos(a),math.sin(a)
    rx=np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]]);ry=np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]]);rz=np.array([[ca,-sa,0],[sa,ca,0],[0,0,1]])
    return rz@ry@rx,np.array([x,y,z])


def clip(vertices,height,above):
    result=[]
    for a,b in zip(vertices,np.roll(vertices,-1,axis=0)):
        ai=a[2]>=height if above else a[2]<=height;bi=b[2]>=height if above else b[2]<=height
        if ai:result.append(a)
        if ai!=bi:result.append(a+(b-a)*(height-a[2])/(b[2]-a[2]))
    return np.asarray(result).reshape(-1,3)


def resource(assets,prefix,uri):
    value=uri.strip()
    if value.startswith('model://'):relative='models/'+value[8:]
    elif value.startswith('file://models/'):relative=value[7:]
    else:
        assert not value.startswith(('/', 'file:', 'https:', 'http:'))
        relative='models/'+prefix+'/'+value
    path=assets/relative;assert path.resolve().is_relative_to(assets.resolve()) and path.is_file(),path
    return path


def collect(assets,spec,groups=None):
    pieces=[];cache={};planes=[]
    for item in spec['models']:
        root=ET.parse(assets/item['model_source']).getroot()
        model=(root.find("world/model[@name='"+item['source_world_model']+"']") if item.get('source_world_model') else root.find('model'))
        assert model is not None
        assert all(abs(v)<1e-12 for v in map(float,model.findtext('pose',default='0 0 0 0 0 0').split())) or item.get('source_world_model')
        wr,wt=pose(item['pose'])
        for link in model.findall('link'):
            assert link.find('pose') is None or not link.find('pose').attrib.get('relative_to')
            lr,lt=pose(link.findtext('pose'))
            for collision in link.findall('collision'):
                assert collision.find('pose') is None or not collision.find('pose').attrib.get('relative_to')
                cr,ct=pose(collision.findtext('pose'));geometry=collision.find('geometry');assert len(geometry)==1
                kind=geometry[0].tag;shape=geometry[0]
                if kind=='plane':
                    normal=np.fromstring(shape.findtext('normal'),sep=' ');normal=wr@lr@cr@normal
                    assert abs(abs(normal[2])-1)<1e-8
                    planes.append(dict(model=item['name'],z=float(((ct@lr.T+lt)@wr.T+wt)[2])))
                    continue
                if kind=='mesh':
                    path=resource(assets,item.get('source_uri_prefix',''),shape.findtext('uri'));key=str(path)
                    if key not in cache:
                        mesh=trimesh.load_scene(path,process=False,ignore_broken=False).to_geometry();assert isinstance(mesh,trimesh.Trimesh) and len(mesh.faces)
                        xyz=np.array(mesh.triangles,dtype=float,copy=True)
                        if path.suffix.lower()=='.dae':
                            dae=ET.parse(path).getroot();ns={'c':dae.tag.split('}')[0][1:]};unit=dae.find('c:asset/c:unit',ns);xyz*=float(unit.get('meter','1')) if unit is not None else 1.
                        cache[key]=xyz
                    scale=np.fromstring(shape.findtext('scale',default='1 1 1'),sep=' ');assert scale.shape==(3,) and (scale>0).all();xyz=cache[key]*scale
                elif kind=='box':
                    size=np.fromstring(shape.findtext('size'),sep=' ');assert size.shape==(3,) and (size>0).all();xyz=trimesh.creation.box(extents=size).triangles
                elif kind=='cylinder':
                    radius=float(shape.findtext('radius'));length=float(shape.findtext('length'));assert radius>0 and length>0
                    xyz=trimesh.creation.cylinder(radius=radius/math.cos(math.pi/128),height=length,sections=128).triangles
                else:raise ValueError('Unsupported collision shape: '+kind)
                xyz=(xyz@cr.T+ct)@lr.T+lt;transformed=xyz@wr.T+wt;pieces.append(transformed)
                if groups is not None:groups.append((kind,transformed))
    return np.concatenate(pieces),planes,len(cache)


def filled_mesh_sections(triangles,height=.0400001):
    """Even-odd interior of each connected mesh component; preserve cavities.

    Surface projections plus a lower-slab solid cross section cover the projected
    occupied volume. Separate components are unioned rather than XORed.
    """
    parent=list(range(len(triangles)));owners={}
    def find(i):
        while parent[i]!=i:parent[i]=parent[parent[i]];i=parent[i]
        return i
    for i,tri in enumerate(triangles):
        for v in np.round(tri,7):
            key=tuple(v)
            if key in owners:
                a,b=find(i),find(owners[key])
                if a!=b:parent[a]=b
            else:owners[key]=i
    components={}
    for i in range(len(triangles)):components.setdefault(find(i),[]).append(i)
    result=[]
    for indices in components.values():
        segments=set()
        for tri in triangles[indices]:
            points=[]
            for a,b in zip(tri,np.roll(tri,-1,axis=0)):
                if (a[2]>height)!=(b[2]>height):points.append(tuple(np.round((a+(b-a)*(height-a[2])/(b[2]-a[2]))[:2],7)))
            if len(points)==2 and points[0]!=points[1]:segments.add(tuple(sorted(points)))
        if len(segments)<3:continue
        segments=sorted(segments);lines=shapely.multilinestrings(segments);faces=list(shapely.get_parts(shapely.polygonize(shapely.get_parts(shapely.union_all(lines)))))
        for face in faces:
            point=face.representative_point();x,y=point.x,point.y;crossings=0
            for (ax,ay),(bx,by) in segments:
                if (ay>y)!=(by>y) and ax+(bx-ax)*(y-ay)/(by-ay)>x:crossings+=1
            if crossings%2:result.append(face)
    return result


def project(assets):
    spec=json.loads((assets/'projection_spec.json').read_text());assert all(sha(assets/p)==h for p,h in spec['source_files_sha256'].items())
    groups=[];triangles,planes,meshes=collect(assets,spec,groups);floor=[];obstacles=[]
    for tri in triangles:
        normal=np.cross(tri[1]-tri[0],tri[2]-tri[0]);n=np.linalg.norm(normal)
        if n<1e-12:continue
        if max(abs(tri[:,2]))<=.03 and abs(normal[2])/n>=.98:floor.append(shapely.convex_hull(shapely.multipoints(tri[:,:2])))
        if tri[:,2].max()<.04 or tri[:,2].min()>.35:continue
        polygon=clip(tri,.04,True)
        if len(polygon):polygon=clip(polygon,.35,False)
        if len(polygon):
            points=polygon[:,:2];hull=shapely.convex_hull(shapely.multipoints(points))
            if hull.geom_type=='Polygon' and hull.area<1e-10:
                distances=np.sum((points[:,None,:]-points[None,:,:])**2,axis=2);i,j=np.unravel_index(distances.argmax(),distances.shape);hull=shapely.LineString([points[i],points[j]])
            obstacles.append(hull)
    # Primitive solids occupy their interiors, not just projected side faces.
    # Convex hull of the clipped surface vertices equals the projection of a
    # convex primitive intersected with the declared robot-height slab.
    for kind,group in groups:
        if kind=='mesh':
            obstacles.extend(filled_mesh_sections(group))
            continue
        if kind not in ('box','cylinder'):continue
        points=[]
        for tri in group:
            polygon=clip(tri,.04,True)
            if len(polygon):polygon=clip(polygon,.35,False)
            if len(polygon):points.extend(polygon[:,:2])
        if points:obstacles.append(shapely.convex_hull(shapely.multipoints(points)))
    bounds=spec.get('operational_roi_xy')
    if bounds is None:
        assert spec['roi_policy']=='all finite source collisions plus1m,ground-plane support'
        bounds=[float(triangles[:,:,0].min()-1),float(triangles[:,:,1].min()-1),float(triangles[:,:,0].max()+1),float(triangles[:,:,1].max()+1)]
    roi=shapely.box(*bounds)
    if planes:
        assert all(abs(p['z'])<.03 for p in planes);floor.append(roi)
    assert floor and obstacles
    domain=shapely.intersection(shapely.union_all(floor),roi);blocked=shapely.union_all(obstacles);xmin,ymin,xmax,ymax=bounds;res=.05
    ox=math.floor(xmin/res)*res-res;oy=math.floor(ymin/res)*res-res;width=math.ceil((xmax-ox)/res)+1;height=math.ceil((ymax-oy)/res)+1;assert width*height<=4000000
    g=np.full((height,width),-1,dtype=np.int8)
    for r in range(0,height,40):
        xx,yy=np.meshgrid(ox+(np.arange(width)+.5)*res,oy+(np.arange(r,min(r+40,height))+.5)*res);cells=shapely.box(xx-.025,yy-.025,xx+.025,yy+.025);part=g[r:r+len(yy)]
        part[shapely.covers(domain,cells)]=0;part[shapely.intersects(blocked,cells)]=100
    record=dict(width=width,height=height,resolution=res,origin=[ox,oy,0.],data=g.ravel().astype(int).tolist(),source='offline declared source-collision projection; native sensor validation pending')
    report=dict(projection_spec_sha256=sha(assets/'projection_spec.json'),triangles=len(triangles),unique_meshes=meshes,ground_planes=planes,operational_roi_xy=bounds,collision_bounds_xyz=[triangles.min(axis=(0,1)).tolist(),triangles.max(axis=(0,1)).tolist()],height_slab_m=[.04,.35],floor_tolerance_m=.03,mesh_volume_projection='connected-component even-odd cross section atz.0400001m plus slab surface projection;1e-7m endpoint precision;holes retained',primitive_volume_projection='filled convex solid intersection with height slab',degenerate_mesh_projection='hulls with area<1e-10 replaced by farthest-vertex segment',cylinder_representation='circumscribed128-sided prism; source SDF cylinders unchanged',loader='trimesh node transforms and DAE metre (gz-common7 convention)',packages=dict(trimesh=trimesh.__version__,numpy=np.__version__,shapely=shapely.__version__),cells={str(k):int(np.sum(g==k)) for k in [-1,0,100]},builder_sha256=sha(Path(__file__)))
    return record,report
