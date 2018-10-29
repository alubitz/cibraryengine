
local P = dofile("Files/Scripts/modelphysics_util.lua")
local Vec3         = ba.createVector
local Sphere       = P.sphere
local Bone         = P.add_single_bone
local SymBones     = P.add_symmetric_bones
local Joint        = P.add_single_joint
local SymJoints    = P.add_symmetric_joints
local TorqueLimits = P.torque_limits


local b = { }
local j = { }

local carapace_spheres =
{
	Sphere( 0.0,   1.07,  0.56,  0.10 ),
	Sphere( 0.0,   1.18, -0.34,  0.15 ),
	Sphere( 0.0,   1.04, -0.63,  0.15 ),
	Sphere( 0.38,  1.00, -0.55,  0.12 ),
	Sphere(-0.38,  1.00, -0.55,  0.12 ),
	Sphere( 0.59,  1.02, -0.09,  0.15 ),
	Sphere(-0.59,  1.02, -0.09,  0.15 )
}

Bone(b,     "carapace", 54.08, carapace_spheres)
Bone(b,     "head",     4.22,  { Sphere( 0.0,   1.008,  0.529,  0.20 ), Sphere( 0.0,    0.850,  0.388,  0.22 ),  Sphere( 0.0,  0.9, -0.1,  0.15 ) } )
Bone(b,     "tail",     2.15,  { Sphere( 0.0,   0.942, -0.723,  0.25 ), Sphere( 0.0,    0.45,  -0.43,   0.02 ) } )

SymBones(b, "leg a 1",  0.73,  { Sphere( 0.379, 0.838,  0.322,  0.06 ), Sphere( 0.436,  0.544,  0.535,  0.16 ) } )
SymBones(b, "leg a 2",  0.89,  { Sphere( 0.436, 0.544,  0.535,  0.16 ), Sphere( 0.472,  0.694,  0.942,  0.09 ) } )
SymBones(b, "leg a 3",  0.39,  { Sphere( 0.472, 0.694,  0.942,  0.09 ), Sphere( 0.288,  0.01,   1.293,  0.01 ) } )

SymBones(b, "leg b 1",  1.33,  { Sphere( 0.701, 0.906,  0.062,  0.08 ), Sphere( 1.051,  0.700,  0.162,  0.13 ) } )
SymBones(b, "leg b 2",  1.70,  { Sphere( 1.049, 0.741,  0.166,  0.10 ), Sphere( 1.610,  1.034,  0.339,  0.14 ) } )
SymBones(b, "leg b 3",  1.08,  { Sphere( 1.660, 1.058,  0.348,  0.10 ), Sphere( 2.068,  0.01,   0.470,  0.01 ) } )

SymBones(b, "leg c 1",  1.06,  { Sphere( 0.429, 0.869, -0.474,  0.08 ), Sphere( 0.540,  0.665, -0.678,  0.13 ) } )
SymBones(b, "leg c 2",  1.21,  { Sphere( 0.540, 0.665, -0.678,  0.13 ), Sphere( 0.714,  0.781, -1.091,  0.09 ) } )
SymBones(b, "leg c 3",  0.72,  { Sphere( 0.714, 0.781, -1.091,  0.09 ), Sphere( 0.798,  0.01,  -1.366,  0.01 ) } )
-- rotation limit for things that aren't supposed to rotate much
local T = 0.0
local a_hinge_axes = { Vec3( 0.2068, -0.0443, -0.0094 ) }
local b_hinge_axes = { Vec3( 0.0336,  0.0,    -0.1125 ) }local c_hinge_axes = { Vec3(-0.2552,  0.0232, -0.1112 ) }

local TL = { neck = 200, tail = 150, leg_a = 0.9, leg_b = 1.0, leg_c = 0.9, joint_1 = 1400, joint_2 = 1100, joint_3 = 900 }

Joint(j, b,     "carapace", "head",    Vec3( 0.0,   1.11,   0.38  ), nil, nil,          Vec3(-0.1, -0.1, -0.1), Vec3(0.1, 0.1, 0.1), TorqueLimits(TL.neck))
Joint(j, b,     "carapace", "tail",    Vec3( 0.0,   1.04,  -0.83  ), nil, nil,          Vec3(-0.3, -0.3, -0.3), Vec3(0.3, 0.3, 0.3), TorqueLimits(TL.tail))

SymJoints(j, b, "carapace", "leg a 1", Vec3( 0.365, 1.015,  0.328 ), nil, nil,          nil,                nil,                     TorqueLimits(TL.leg_a * TL.joint_1))
SymJoints(j, b, "leg a 1",  "leg a 2", Vec3( 0.437, 0.574,  0.539 ), nil, a_hinge_axes, Vec3(-1.4, -T, -T), Vec3(1.0, T, T),         TorqueLimits(TL.leg_a * TL.joint_2, 0, 0))
SymJoints(j, b, "leg a 2",  "leg a 3", Vec3( 0.470, 0.694,  0.993 ), nil, a_hinge_axes, Vec3(-1.0, -T, -T), Vec3(1.3, T, T),         TorqueLimits(TL.leg_a * TL.joint_3, 0, 0))

SymJoints(j, b, "carapace", "leg b 1", Vec3( 0.701, 0.906,  0.062 ), nil, nil,          nil,                nil,                     TorqueLimits(TL.leg_b * TL.joint_1))
SymJoints(j, b, "leg b 1",  "leg b 2", Vec3( 1.043, 0.742,  0.164 ), nil, b_hinge_axes, Vec3(-1.4, -T, -T), Vec3(1.0, T, T),         TorqueLimits(TL.leg_b * TL.joint_2, 0, 0))
SymJoints(j, b, "leg b 2",  "leg b 3", Vec3( 1.631, 1.050,  0.339 ), nil, b_hinge_axes, Vec3(-1.0, -T, -T), Vec3(1.3, T, T),         TorqueLimits(TL.leg_b * TL.joint_3, 0, 0))

SymJoints(j, b, "carapace", "leg c 1", Vec3( 0.452, 0.891, -0.421 ), nil, nil,          nil,                nil,                     TorqueLimits(TL.leg_c * TL.joint_1))
SymJoints(j, b, "leg c 1",  "leg c 2", Vec3( 0.544, 0.676, -0.693 ), nil, c_hinge_axes, Vec3(-1.4, -T, -T), Vec3(1.0, T, T),         TorqueLimits(TL.leg_c * TL.joint_2, 0, 0))
SymJoints(j, b, "leg c 2",  "leg c 3", Vec3( 0.707, 0.786, -1.076 ), nil, c_hinge_axes, Vec3(-1.0, -T, -T), Vec3(1.3, T, T),         TorqueLimits(TL.leg_c * TL.joint_3, 0, 0))

ba.saveModelPhysics(b, j, "crab_bug")
