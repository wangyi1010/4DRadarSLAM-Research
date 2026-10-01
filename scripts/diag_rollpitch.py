#!/usr/bin/env python3
import sys, numpy as np, rosbag

# extrinsicRPY from config (IMU->radar), row-major 3x3
extRPY = np.array([
 [0.999735807578, -0.0215215701795, -0.0081643477385],
 [-0.02148120581797, -0.9997581134183, 0.00502853428037],
 [-0.00826995351904, -0.0048509797951, -0.99995400578406]])

def quat_to_R(w,x,y,z):
    n=np.sqrt(w*w+x*x+y*y+z*z); w,x,y,z=w/n,x/n,y/n,z/n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])

def R2ypr(R):  # matches code's ros_utils.hpp
    n=R[:,0]; o=R[:,1]; a=R[:,2]
    y=np.arctan2(n[1],n[0])
    p=np.arctan2(-n[2], n[0]*np.cos(y)+n[1]*np.sin(y))
    r=np.arctan2(a[0]*np.sin(y)-a[1]*np.cos(y), -o[0]*np.sin(y)+o[1]*np.cos(y))
    return np.array([y,p,r])  # yaw,pitch,roll

bagpath, gtpath, seq = sys.argv[1], sys.argv[2], sys.argv[3]

# --- IMU: /vectornav/imu, apply extQRPY, reference to initial ---
imu_t=[]; imu_R=[]
for _,msg,_ in rosbag.Bag(bagpath).read_messages(topics=['/vectornav/imu']):
    q=msg.orientation
    Rimu=quat_to_R(q.w,q.x,q.y,q.z)
    Rdes = Rimu @ extRPY   # q_imu * extQRPY  == R_imu * R_extRPY
    imu_t.append(msg.header.stamp.to_sec()); imu_R.append(Rdes)
imu_t=np.array(imu_t); imu_R=np.array(imu_R)
R0_imu = imu_R[0]
# reference each to initial (left-inverse), then ypr
imu_rp=np.array([R2ypr(R0_imu.T @ R)[[2,1]] for R in imu_R])  # [roll,pitch]

# --- GT: timestamp tx ty tz qx qy qz qw ---
gt=[]
for line in open(gtpath):
    if line.startswith('#') or not line.strip(): continue
    v=[float(x) for x in line.split()]
    t,qx,qy,qz,qw = v[0],v[4],v[5],v[6],v[7]
    gt.append((t,quat_to_R(qw,qx,qy,qz)))
gt_t=np.array([g[0] for g in gt]); gt_R=np.array([g[1] for g in gt])
R0_gt=gt_R[0]
gt_rp=np.array([R2ypr(R0_gt.T @ R)[[2,1]] for R in gt_R])

def interp_rp(src_t, src_rp, tgt_t):
    out=np.full((len(tgt_t),2),np.nan)
    for j in range(2):
        out[:,j]=np.interp(tgt_t, src_t, src_rp[:,j], left=np.nan, right=np.nan)
    return out

# radar timestamps = GT timestamps restricted to IMU coverage (both have roll/pitch)
lo=max(imu_t[0], gt_t[0]); hi=min(imu_t[-1], gt_t[-1])
mask=(gt_t>=lo)&(gt_t<=hi)
rt=gt_t[mask]; gt_rp_r=gt_rp[mask]

print(f"=== {seq}: IMU vs GT roll/pitch (rad->deg), n={len(rt)} ===")
print(f"IMU rate ~{1/np.median(np.diff(imu_t)):.0f}Hz, GT rate ~{1/np.median(np.diff(gt_t)):.0f}Hz, overlap {hi-lo:.0f}s")

# timestamp-offset sweep
print("dt_ms |  roll_mean roll_med roll_rmse |  pitch_mean pitch_med pitch_rmse  (deg)")
best=None
for dt_ms in range(-50,51,10):
    imu_rp_i = interp_rp(imu_t+dt_ms/1000.0, imu_rp, rt)
    err = (imu_rp_i - gt_rp_r)*180/np.pi
    ok=~np.isnan(err[:,0])
    e=err[ok]
    rr=np.sqrt(np.mean(e[:,0]**2)); pr=np.sqrt(np.mean(e[:,1]**2))
    print(f"{dt_ms:+5d} | {np.mean(e[:,0]):+8.3f} {np.median(e[:,0]):+7.3f} {rr:7.3f} | {np.mean(e[:,1]):+9.3f} {np.median(e[:,1]):+7.3f} {pr:7.3f}")
    if best is None or (rr+pr)<best[1]: best=(dt_ms, rr+pr)
print(f"best dt = {best[0]:+d} ms (min roll+pitch rmse)")

# at dt=0, error-vs-time slope (is it constant or growing?)
imu_rp0 = interp_rp(imu_t, imu_rp, rt)
err0=(imu_rp0-gt_rp_r)*180/np.pi; ok=~np.isnan(err0[:,0]); e=err0[ok]; tt=rt[ok]-rt[ok][0]
for j,name in [(0,'roll'),(1,'pitch')]:
    A=np.vstack([tt,np.ones_like(tt)]).T
    slope,inter=np.linalg.lstsq(A,e[:,j],rcond=None)[0]
    print(f"{name}: mean={np.mean(e[:,j]):+.3f} std={np.std(e[:,j]):.3f} slope={slope:+.4f} deg/s intercept={inter:+.3f}")
