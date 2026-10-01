#!/usr/bin/env python3
import sys, numpy as np
def quat_to_R(w,x,y,z):
    n=np.sqrt(w*w+x*x+y*y+z*z); w,x,y,z=w/n,x/n,y/n,z/n
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def R2ypr(R):
    n=R[:,0];o=R[:,1];a=R[:,2]
    y=np.arctan2(n[1],n[0]); p=np.arctan2(-n[2],n[0]*np.cos(y)+n[1]*np.sin(y))
    r=np.arctan2(a[0]*np.sin(y)-a[1]*np.cos(y),-o[0]*np.sin(y)+o[1]*np.cos(y))
    return np.array([y,p,r])
def geo(R):  # geodesic angle of rotation matrix (deg)
    return np.degrees(np.arccos(np.clip((np.trace(R)-1)/2,-1,1)))
def load(path):
    T=[];P=[];R=[]
    for l in open(path):
        if l.startswith('#') or not l.strip(): continue
        v=[float(x) for x in l.split()]
        T.append(v[0]);P.append(v[1:4]);R.append(quat_to_R(v[7],v[4],v[5],v[6]))
    return np.array(T),np.array(P),np.array(R)
gt_t,gt_P,gt_R=load(sys.argv[1])
def nidx(ts,t):
    i=np.searchsorted(ts,t); i=np.clip(i,1,len(ts)-1)
    return np.where(np.abs(ts[i-1]-t)<np.abs(ts[i]-t),i-1,i)
L=float(sys.argv[4]) if len(sys.argv)>4 else 49.0
for label,est in [("IMU-off",sys.argv[2]),("r=0.1",sys.argv[3])]:
    et,eP,eR=load(est)
    gi=nidx(gt_t,et); good=np.abs(gt_t[gi]-et)<0.06
    et,eR,eP,gi=et[good],eR[good],eP[good],gi[good]; gR=gt_R[gi]; gP=gt_P[gi]
    # cumulative distance along estimate
    d=np.concatenate([[0],np.cumsum(np.linalg.norm(np.diff(eP,axis=0),axis=1))])
    rpy=[]; geod=[]
    j0=0
    for i in range(len(et)):
        f=np.searchsorted(d,d[i]+L)
        if f>=len(et): break
        dRe=eR[i].T@eR[f]; dRg=gR[i].T@gR[f]; dE=dRg.T@dRe
        y=R2ypr(dE)*180/np.pi; rpy.append([y[2],y[1],y[0]]); geod.append(geo(dE))
    rpy=np.array(rpy); geod=np.array(geod)
    rm=np.sqrt(np.mean(rpy**2,axis=0))
    print(f"{label:8s} {L:.0f}m-window rel-rot ERR rmse(deg): geodesic={np.sqrt(np.mean(geod**2)):.3f} | roll={rm[0]:.3f} pitch={rm[1]:.3f} yaw={rm[2]:.3f}  n={len(rpy)}")
