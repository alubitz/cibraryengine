
local P = dofile("Files/Scripts/modelphysics_util.lua")
local Vec3         = ba.createVector
local Sphere       = P.sphere
local Bone         = P.add_single_bone
local SymBones     = P.add_symmetric_bones
local Joint        = P.add_single_joint
local SymJoints    = P.add_symmetric_joints
local TorqueLimits = P.torque_limits

local m = 98.0 / 25.0;		-- divide total mass of 98 evenly across each of 20 bones, 5 of which have double weight
local M = m * 2;
local n = m;
local N = M;

local b = { }				-- bones
local j = { }				-- joints

local Z = N
Bone(b,     "pelvis",   Z, { Sphere( 0.00, 1.08,  0.03, 0.18), Sphere( 0.14, 1.07, -0.07, 0.16), Sphere(-0.14, 1.07, -0.07, 0.16) } )
Bone(b,     "torso 1",  N, { Sphere( 0.09, 1.28,  0.06, 0.15), Sphere(-0.09, 1.28,  0.06, 0.15), Sphere( 0.00, 1.37, -0.04, 0.20), Sphere(0.00, 1.28, -0.01, 0.20) } )
Bone(b,     "torso 2",  N, { Sphere( 0.00, 1.65, -0.20, 0.20), Sphere( 0.14, 1.50,  0.09, 0.16), Sphere(-0.14, 1.50,  0.09, 0.16) } )
Bone(b,     "head",     n, { Sphere( 0.00, 1.80,  0.00, 0.15) } )

SymBones(b, "shoulder", N, { Sphere( 0.29, 1.52,  0.00, 0.16), Sphere( 0.36, 1.66, -0.05, 0.11), Sphere( 0.42, 1.67, -0.31, 0.14), Sphere(0.30, 1.60, -0.37, 0.14) } )
SymBones(b, "arm 1",    n, { Sphere( 0.30, 1.51, -0.02, 0.14), Sphere( 0.53, 1.40, -0.05, 0.11) } )
SymBones(b, "arm 2",    n, { Sphere( 0.53, 1.40, -0.05, 0.11), Sphere( 0.77, 1.27,  0.02, 0.10) } )
SymBones(b, "hand",     n, { Sphere( 0.92, 1.20,  0.01, 0.08), Sphere( 0.90, 1.21,  0.10, 0.08), Sphere( 1.08, 1.08,  0.09, 0.07) } )
local z = m
local zz = m
SymBones(b, "leg 1",   zz, { Sphere( 0.15, 1.04,  0.03, 0.15), Sphere( 0.19, 0.63,  0.05, 0.13), Sphere( 0.15, 1.04, -0.04, 0.13) } )
SymBones(b, "leg 2",   zz, { Sphere( 0.19, 0.63,  0.05, 0.13), Sphere( 0.23, 0.28,  0.04, 0.11), Sphere( 0.23, 0.30, -0.10, 0.11) } )
SymBones(b, "heel",     z, { Sphere( 0.23, 0.09, -0.12, 0.09), Sphere( 0.23, 0.10, -0.03, 0.10) } )
SymBones(b, "toe",      z, { Sphere( 0.24, 0.09,  0.13, 0.09), Sphere( 0.27, 0.07,  0.26, 0.07), Sphere( 0.21, 0.07,  0.26, 0.07) } )

local hand_axes = { Vec3(11.6, 22.0, 0.0), Vec3(0, 0, -1) }
local knee_axes = { Vec3(28.2, 2.5,  0.0) }

local TL = { spine = 1200, neck = 150, wrist = 200, elbow = 350, shoulder_b = 600, shoulder_a = 700, hip = 1400, knee = 1000, ankle = 600, heel_toe = 600 }

Joint(j, b,     "pelvis",   "torso 1",  Vec3(0.00, 1.34, -0.10), nil, nil,       Vec3(-0.55, -0.5,  -0.45), Vec3(0.5,   0.5,  0.45), TorqueLimits(TL.spine))
Joint(j, b,     "torso 1",  "torso 2",  Vec3(0.00, 1.57,  0.00), nil, nil,       Vec3(-0.55, -0.5,  -0.45), Vec3(0.5,   0.5,  0.45), TorqueLimits(TL.spine))
Joint(j, b,     "torso 2",  "head",     Vec3(0.00, 1.73,  0.00), nil, nil,       Vec3(-0.4,  -1.2,  -0.26), Vec3(0.3,   1.2,  0.26), TorqueLimits(TL.neck))
SymJoints(j, b, "torso 2",  "shoulder", Vec3(0.22, 1.65, -0.16), nil, nil,       Vec3(-0.25, -0.25, -0.25), Vec3(0.25,  0.25, 0.25), TorqueLimits(TL.shoulder_a))
SymJoints(j, b, "shoulder", "arm 1",    Vec3(0.30, 1.51, -0.04), nil, nil,       Vec3(-1.8,  -1.8,  -1.0 ), Vec3(1.8,   1.0,  0.8 ), TorqueLimits(TL.shoulder_b))
SymJoints(j, b, "arm 1",    "arm 2",    Vec3(0.53, 1.40, -0.04), nil, hand_axes, Vec3(-2.0,  -0.5,  -1.0 ), Vec3(0.5,   0.5,  1.0 ), TorqueLimits(TL.elbow))
SymJoints(j, b, "arm 2",    "hand",     Vec3(0.84, 1.25,  0.02), nil, hand_axes, Vec3(-1.0,  -1.5,  -1.8 ), Vec3(1.0,   1.5,  1.8 ), TorqueLimits(TL.wrist))
SymJoints(j, b, "pelvis",   "leg 1",    Vec3(0.15, 1.04, -0.02), nil, nil,       Vec3(-1.0,  -0.25, -0.2 ), Vec3(1.0,   0.25, 1.0 ), TorqueLimits(TL.hip))
SymJoints(j, b, "leg 1",    "leg 2",    Vec3(0.19, 0.63,  0.05), nil, knee_axes, Vec3(-0.02,  0.0,   0.0 ), Vec3(1.745, 0.0,  0.0 ), TorqueLimits(TL.knee, 0, 0))
SymJoints(j, b, "leg 2",    "heel",     Vec3(0.23, 0.16, -0.06), nil, nil,       Vec3(-0.3,  -0.3,  -0.3 ), Vec3(0.3,   0.3,  0.3 ), TorqueLimits(TL.ankle))
SymJoints(j, b, "heel",     "toe",      Vec3(0.24, 0.10,  0.08), nil, nil,       Vec3(-0.3,  -0.05, -0.1 ), Vec3(0.3,   0.05, 0.1 ), TorqueLimits(TL.heel_toe))

ba.saveModelPhysics(b, j, "soldier")
