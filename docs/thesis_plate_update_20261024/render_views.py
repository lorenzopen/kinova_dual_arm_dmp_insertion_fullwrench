"""Render measured bench poses from a clearer fixed camera; no new dynamics."""
from bench_plate import *

DEST=OUT/'rendered'
DEST.mkdir(exist_ok=True)
results=json.loads((OUT/'results.json').read_text())
for record in results['cases']:
    name=record['case'];p.connect(p.DIRECT)
    body,_=wall(name)
    tool=FP.FloatingPlateTool([-.06,0,0],[0,0,0,1],pads=FP.PadSprings(dt=DT),
                              centre=FP.CentralSpring(dt=DT),mu=.7,dt=DT,quiet=True)
    p.changeVisualShape(tool.id,-1,rgbaColor=[.12,.44,.70,1])
    p.changeVisualShape(tool.plate_id,-1,rgbaColor=[.91,.73,.30,1])
    for i in range(3):p.changeVisualShape(tool.plate_id,i,rgbaColor=[.85,.34,.16,1])
    if name=='faceted':
        # Visual-only face tints make the actual ridge visible, without adding collisions.
        az,ay,ridge,y0=.10,.10,.35,.008
        surf=lambda y,z:az*z+ay*y+ridge*abs(y-y0)-ridge*y0-.00001
        for i,(a,b) in enumerate([(-.075,y0),(y0,.075)]):
            verts=[[surf(y,z),y,z] for y,z in [(a,-.065),(b,-.065),(b,.065),(a,.065)]]
            v=p.createVisualShape(p.GEOM_MESH,vertices=verts,indices=[0,1,2,0,2,3],
                                   rgbaColor=([.57,.61,.65,1] if i==0 else [.72,.75,.78,1]))
            p.createMultiBody(0,-1,v)
    for phase in ['first_load','settled']:
        st=record[phase]
        p.resetBasePositionAndOrientation(tool.id,st['flange_position'],st['flange_quat'])
        p.resetBasePositionAndOrientation(tool.plate_id,st['plate_position'],st['plate_quat'])
        view=p.computeViewMatrix([-.24,-.10,.075],[-.005,0,0],[0,0,1])
        projection=p.computeProjectionMatrixFOV(36,1.30,.005,1.)
        rgba=p.getCameraImage(780,600,view,projection,renderer=p.ER_TINY_RENDERER,
            lightDirection=[-2,-3,5],shadow=1,lightAmbientCoeff=.75,lightDiffuseCoeff=.55)[2]
        Image.fromarray(np.asarray(rgba,dtype=np.uint8).reshape(600,780,4)[:,:,:3]).save(DEST/(name+'_'+phase+'.png'))
    p.disconnect()
print('Six views rendered from recorded flange and plate poses; no forces altered.')
