# Recorded data and coordinates

Every runs/<id>/ contains STARTED.json, COMPLETED.json, trace.npz,
manifold.npz, after_release.png and final.png. Physical stops are in
COMPLETED.json.error; they are not discarded. FAILED.json would represent a
Python exception (none occurred in the 111-case campaign).

The last trace sample in an uninterrupted run is 4.4905 s, although integration
continues to 4.5 s. The record interval is 0.01 s. All vectors use world axes
unless explicitly stated. Indices 0-2 are left pads, 3-5 right pads.

| Array | Meaning |
| --- | --- |
| t, angle, phase | Sample time, prescribed y tilt in degrees, fixture/hold/wield |
| force [time,6,3] | Actual force ON the object, one resultant per pad [N] |
| points [time,6,3] | Actual object contact point, normal-force weighted [m] |
| normals [time,6,3] | Outward object normal, averaged and normalized; zero if absent |
| normal_N [time,6] | Sum of native normal contact loads [N] |
| normal_spread | Maximum manifold-normal deviation within a pad [deg] |
| slip_speed | Maximum tangential relative speed at the pad contacts [m/s] |
| desired_wrist [time,2,6] | Planned wrench about the nominal flange reference |
| measured_wrist [time,2,6] | Contact-reconstructed virtual wrench about actual flange |
| witness [time,6,3] | Auxiliary force allocation, not individual pad commands |
| rho [time,6] | Required nominal signed-distance margin [N] |
| nominal_normals | Normals supplied by prescribed nominal geometry |
| object_pos, object_quat | Actual COM and orientation; used for plant/scoring only |
| object_vel | COM velocity followed by angular velocity |
| tool_pos, tool_quat | Actual two flange poses |
| plate_deflection [time,2,6] | Translation and rotation vector in mount coordinates |
| entries | Number of native contact manifold entries in this sample |

manifold.npz stores one row per native contact entry, columns:
t, pad, normal_N, tangential_N, fx,fy,fz,nx,ny,nz,px,py,pz.
This preserves the information lost by an averaged per-pad normal.

Wrench order is Fx,Fy,Fz,Mx,My,Mz. Moments are Nm; forces are N. The virtual
wrist is reconstructed from object contact forces; it is not a full physical
ATI joint measurement including tool inertia/gravity. For comparisons it is
transported to the same nominal reference pole before calculating errors.

NaN witness entries in Wong profiles indicate lack of a nominal discrete
three-pad force witness. They are retained as evidence of the aggregate vs
discrete modeling gap; no artificial witness is substituted. Native PyBullet
contacts are read in their original A/B order so friction-direction signs are
not altered by filtered body-order queries.
