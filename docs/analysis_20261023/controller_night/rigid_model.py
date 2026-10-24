"""Small read-only URDF rigid-body model for offline damping sensitivity.

No controller/ROS imports. Reflected drive inertia, friction and flexible joints
are NOT in the URDF model. Verify FK/Jacobian before interpreting calculations.
"""
import hashlib
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R


def vector(text,default='0 0 0'):
    return np.fromstring(text if text is not None else default,sep=' ')


def origin(node):
    h=np.eye(4)
    if node is not None:
        h[:3,:3]=R.from_euler('xyz',vector(node.get('rpy'))).as_matrix()
        h[:3,3]=vector(node.get('xyz'))
    return h


class RigidURDF:
    def __init__(self,path):
        self.path=Path(path);self.sha256=hashlib.sha256(self.path.read_bytes()).hexdigest()
        root=ET.parse(path).getroot();self.links={n.get('name'):n for n in root.findall('link')}
        self.joints=[];self.children={}
        for node in root.findall('joint'):
            joint=dict(name=node.get('name'),type=node.get('type'),
                       parent=node.find('parent').get('link'),child=node.find('child').get('link'),
                       origin=origin(node.find('origin')),
                       axis=vector(node.find('axis').get('xyz')) if node.find('axis') is not None else np.array([1.,0.,0.]))
            self.joints.append(joint);self.children.setdefault(joint['parent'],[]).append(joint)
        self.active=[j['name'] for j in self.joints if j['type'] in ['revolute','continuous']]
        assert self.active==['joint_'+str(i) for i in range(1,8)]
        self.index={name:i for i,name in enumerate(self.active)}
        self.root=next(name for name in self.links if name not in {j['child'] for j in self.joints})

    def compute(self,q):
        frames={self.root:np.eye(4)};ancestors={self.root:[]};pivots={};axes={}
        def visit(parent):
            for joint in self.children.get(parent,[]):
                pre=frames[parent]@joint['origin'];h=pre.copy();chain=list(ancestors[parent])
                if joint['name'] in self.index:
                    i=self.index[joint['name']];axis=joint['axis']/np.linalg.norm(joint['axis'])
                    pivots[i]=pre[:3,3];axes[i]=pre[:3,:3]@axis
                    rot=np.eye(4);rot[:3,:3]=R.from_rotvec(axis*q[i]).as_matrix();h=pre@rot;chain.append(i)
                else:assert joint['type']=='fixed'
                frames[joint['child']]=h;ancestors[joint['child']]=chain;visit(joint['child'])
        visit(self.root)
        def jacobian(point,chain):
            jv=np.zeros((3,7));jw=np.zeros((3,7))
            for i in chain:
                jv[:,i]=np.cross(axes[i],point-pivots[i]);jw[:,i]=axes[i]
            return np.vstack([jv,jw])
        mass=np.zeros((7,7));body_mass=0.
        for name,node in self.links.items():
            inertial=node.find('inertial')
            if inertial is None:continue
            mi=float(inertial.find('mass').get('value'));n=inertial.find('inertia')
            ii=np.array([[float(n.get('ixx')),float(n.get('ixy')),float(n.get('ixz'))],
                         [float(n.get('ixy')),float(n.get('iyy')),float(n.get('iyz'))],
                         [float(n.get('ixz')),float(n.get('iyz')),float(n.get('izz'))]])
            hc=frames[name]@origin(inertial.find('origin'));j=jacobian(hc[:3,3],ancestors[name])
            inertia=hc[:3,:3]@ii@hc[:3,:3].T
            mass+=mi*j[:3].T@j[:3]+j[3:].T@inertia@j[3:]
            if ancestors[name]:body_mass+=mi
        ee=frames['end_effector_link'];j=jacobian(ee[:3,3],ancestors['end_effector_link'])
        assert np.linalg.eigvalsh(mass).min()>0
        inverse_cart_mass=j@np.linalg.solve(mass,j.T)
        assert np.linalg.eigvalsh(inverse_cart_mass).min()>1e-10
        cart_mass=np.linalg.inv(inverse_cart_mass)
        return dict(ee=ee,jacobian=j,joint_mass=mass,cart_mass=(cart_mass+cart_mass.T)/2,
                    moving_link_mass_kg=body_mass)

    def verify_jacobian(self,q,eps=1e-6):
        base=self.compute(q);j=base['jacobian'];numeric=np.zeros_like(j)
        for i in range(7):
            qp=q.copy();qm=q.copy();qp[i]+=eps;qm[i]-=eps
            hp=self.compute(qp)['ee'];hm=self.compute(qm)['ee']
            numeric[:3,i]=(hp[:3,3]-hm[:3,3])/(2*eps)
            numeric[3:,i]=R.from_matrix(hp[:3,:3]@hm[:3,:3].T).as_rotvec()/(2*eps)
        err=float(abs(numeric-j).max());assert err<1e-7,err
        return err


def system(cart_mass,normal,contact_stiffness,damping_trans=60,damping_rot=7,
           stiffness_trans=1500,stiffness_rot=20,inertia_scale=1):
    """Lambda xdd + D xd + (K + kc b b')x = K b u; force = kc b'x.

    Coordinates are small flange displacement [translation; base-axis rotation].
    Normal force acts through flange axis; no contact bending stiffness. The
    current tool normal is collinear with its flange-to-pad lever arm.
    """
    b=np.r_[normal,np.zeros(3)];k=np.diag([stiffness_trans]*3+[stiffness_rot]*3)
    translation_d=np.broadcast_to(np.asarray(damping_trans,dtype=float),(3,))
    d=np.diag(np.r_[translation_d,[damping_rot]*3])
    l=cart_mass*inertia_scale;total=k+contact_stiffness*np.outer(b,b)
    amat=np.block([[np.zeros((6,6)),np.eye(6)],[-np.linalg.solve(l,total),-np.linalg.solve(l,d)]])
    bmat=np.r_[np.zeros(6),np.linalg.solve(l,k@b)][:,None]
    cmat=np.r_[contact_stiffness*b,np.zeros(6)][None,:]
    dc=float(contact_stiffness*b@np.linalg.solve(total,k@b))
    return amat,bmat,cmat,dict(dc_gain_N_m=dc,normal_direction=b.tolist(),
                             cartesian_mass=l.tolist(),K=k.tolist(),D=d.tolist(),K_total=total.tolist())


def frequency_response(amat,bmat,cmat,hz):
    matrices=2j*np.pi*hz[:,None,None]*np.eye(len(amat))[None,:,:]-amat[None,:,:]
    return (cmat@np.linalg.solve(matrices,np.broadcast_to(bmat,(len(hz),)+bmat.shape)))[:,0,0]
