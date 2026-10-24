"""Derived Bullet URDF; original lab source remains read-only."""
import xml.etree.ElementTree as ET
from pathlib import Path
import sys,json,hashlib,math
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[4]

def build():
    source=ROOT/'urdf/gen3_ati_tool.urdf';tree=ET.parse(source);robot=tree.getroot()
    changes=[]
    for mesh in robot.findall('.//mesh'):
        name=Path(mesh.get('filename')).stem+'.STL';path=OUT/'assets/meshes'/name
        assert path.exists(),name
        mesh.set('filename',str(path.resolve()).replace('\\','/'))
    # Empty semantic frames are massless in the lab URDF; Bullet otherwise
    # assigns a silent 1kg default to links without an inertial tag.
    for link in robot.findall('link'):
        if link.find('inertial') is None:
            inertial=ET.SubElement(link,'inertial')
            ET.SubElement(inertial,'mass',value='0')
            ET.SubElement(inertial,'inertia',ixx='1e-9',iyy='1e-9',izz='1e-9',ixy='0',ixz='0',iyz='0')
            changes.append(link.get('name')+' explicit massless semantic frame, no Bullet1kg default')
    # Same convention already used by production reach_ik. Continuous joints
    # must not inherit the lab export's artificial +/-2.24 caps.
    continuous={1,3,5,7};bounded={2:128.9,4:147.8,6:120.3}
    for joint in robot.findall('joint'):
        name=joint.get('name')
        if name.startswith('joint_') and name[-1].isdigit():
            n=int(name[-1]);limit=joint.find('limit')
            if n in continuous:
                joint.set('type','continuous')
                for key in ['lower','upper']:limit.attrib.pop(key,None)
            else:
                limit.set('lower',str(-math.radians(bounded[n])));limit.set('upper',str(math.radians(bounded[n])))
            changes.append(name+' actual continuous/bounded convention from reach_ik')
    # Keep 150g total tool mass, without double-counting the explicit 40g
    # spring/plate body. The remaining 110g stay rigid on the arm.
    mount=robot.find("link[@name='ati_tool_payload']")
    mount.find('inertial/mass').set('value','.110')
    mount.find('inertial/origin').set('xyz','0 0 '+str((.15*.035-.04*.085)/.11))
    for colour,radius,length,z in [('0.15 0.15 0.17 1',.022,.02,.01),('0.7 0.7 0.73 1',.0085,.05,.055)]:
        v=ET.SubElement(mount,'visual');ET.SubElement(v,'origin',xyz=f'0 0 {z}',rpy='0 0 0')
        ET.SubElement(ET.SubElement(v,'geometry'),'cylinder',radius=str(radius),length=str(length))
        ET.SubElement(ET.SubElement(v,'material',name=''),'color',rgba=colour)
    sensor=ET.SubElement(robot,'link',name='wrist_sensor_link')
    inertial=ET.SubElement(sensor,'inertial');ET.SubElement(inertial,'mass',value='0.000001')
    ET.SubElement(inertial,'inertia',ixx='1e-9',iyy='1e-9',izz='1e-9',ixy='0',ixz='0',iyz='0')
    j=ET.SubElement(robot,'joint',name='wrist_sensor_joint',type='fixed')
    ET.SubElement(j,'origin',xyz='0 0 0',rpy='0 0 0')
    ET.SubElement(j,'parent',link='ati_tool_payload');ET.SubElement(j,'child',link='wrist_sensor_link')
    path=OUT/'assets/gen3_lab_bullet.urdf';tree.write(path,encoding='utf-8',xml_declaration=True)
    manifest=dict(original=str(source),original_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        derived=str(path),derived_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        changes=changes+['Mesh URI resolution to official Kinova STL',
          'Rigid payload .110kg plus explicit plate/pads .040kg; nominal combined CoM preserved',
          'Original rough payload inertia retained for mount (not calibrated full tool inertia)',
          'Added tiny-mass fixed wrist FT joint downstream of rigid mount; external springs react on sensor link'])
    (OUT/'urdf_provenance.json').write_text(json.dumps(manifest,indent=2)+'\n')
    return path

if __name__=='__main__':print(build())
