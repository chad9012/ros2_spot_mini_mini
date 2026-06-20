import numpy as np
import copy
from spot_mini_mini_control.spotmicro.Kinematics.SpotKinematics import SpotModel
from spot_mini_mini_control.spotmicro.GaitGenerator.Bezier import BezierGait

spot = SpotModel()
bzg = BezierGait(dt=0.02)
T_bf0 = spot.WorldToFoot
T_bf = copy.deepcopy(T_bf0)

print("=== Test 1: Default stance IK ===")
angles = spot.IK(np.array([0,0,0]), np.array([0,0,0]), T_bf0)
print("Joint angles:\n", angles)
print("Expected: ~[0.0, 0.785, -1.57] per leg\n")

print("=== Test 2: One step forward ===")
T_bf = bzg.GenerateTrajectory(
    L=0.03, LateralFraction=0, YawRate=0, vel=0.1,
    T_bf_=T_bf0, T_bf_curr=T_bf,
    clearance_height=0.04, penetration_depth=0.005,
    contacts=[1,1,1,1], dt=0.02
)
angles = spot.IK(np.array([0,0,0]), np.array([0,0,0]), T_bf)
print("After one step:\n", angles)
print("Should differ slightly from default stance\n")

print("=== Test 3: Reachability check ===")
from spot_mini_mini_control.spotmicro.Kinematics.LegKinematics import LegIK
leg = LegIK("RIGHT", 0.055, 0.10652, 0.145)
D = leg.get_domain(0.0, 0.0, -0.20)
print(f"Domain for foot at z=-0.20: {D:.3f} (should be between -1 and 1)")