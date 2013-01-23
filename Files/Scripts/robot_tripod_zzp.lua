
local P = dofile("Files/Scripts/modelphysics_util.lua")

local bones = { }
P.add_single_bone(bones,	"root",				20.0, { P.sphere(	0.0,	1.081,	0.0,	0.625) } )
P.add_single_bone(bones,	"robot arm a 1",	1.33, { P.sphere(	0.701,	0.906,	0.062,	0.08),	P.sphere(	1.051,	0.700,	0.162,	0.13) } )P.add_single_bone(bones,	"robot arm a 2",	1.70, { P.sphere(	1.049,	0.741,	0.166,	0.10),	P.sphere(	1.610,	1.034,	0.339,	0.14) } )
P.add_single_bone(bones,	"robot arm a 3",	1.08, { P.sphere(	1.660,	1.058,	0.348,	0.10),	P.sphere(	2.068,	0.01,	0.470,	0.01) } )
P.add_single_bone(bones,	"robot arm b 1",	1.33, { P.sphere(	-0.297,	0.906,	-0.638,	0.08),	P.sphere(	-0.385,	0.7,	-0.991,	0.13) } )
P.add_single_bone(bones,	"robot arm b 2",	1.70, { P.sphere(	-0.381,	0.741,	-0.991,	0.10),	P.sphere(	-0.511,	1.034,	-1.564,	0.14) } )
P.add_single_bone(bones,	"robot arm b 3",	1.08, { P.sphere(	-0.529,	1.058,	-1.612,	0.10),	P.sphere(	-0.627,	0.01,	-2.026,	0.01) } )

P.add_single_bone(bones,	"robot arm c 1",	1.33, { P.sphere(	-0.404,	0.906,	0.576,	0.08),	P.sphere(	-0.666,	0.7,	0.829,	0.13) } )
P.add_single_bone(bones,	"robot arm c 2",	1.70, { P.sphere(	-0.668,	0.741,	0.825,	0.10),	P.sphere(	-1.099,	1.034,	1.225,	0.14) } )
P.add_single_bone(bones,	"robot arm c 3",	1.08, { P.sphere(	-1.131,	1.058,	1.264,	0.10),	P.sphere(	-1.441,	0.01,	1.556,	0.01) } )


local joints = { }

local hinge_axes_a = { ba.createVector(	-0.0336,	0.0,	0.1125	) }
local hinge_axes_b = { ba.createVector(	0.1142,		0.0,	-0.0272	) }
local hinge_axes_c = { ba.createVector(	-0.0806,	0.0,	-0.0853	) }

-- rotation limit for things that aren't supposed to rotate much
local T = 0.0

P.add_single_joint(joints, bones,	"robot arm a 1",	"root",				ba.createVector(0.701,	0.906,	0.062))
P.add_single_joint(joints, bones,	"robot arm a 2",	"robot arm a 1",	ba.createVector(1.043,	0.742,	0.164), nil, hinge_axes_a,	ba.createVector(-1.4, -T, -T),	ba.createVector(1.0, T, T))
P.add_single_joint(joints, bones,	"robot arm a 3",	"robot arm a 2",	ba.createVector(1.631,	1.050,	0.339), nil, hinge_axes_a,	ba.createVector(-1.0, -T, -T),	ba.createVector(1.3, T, T))

P.add_single_joint(joints, bones,	"robot arm b 1",	"root",				ba.createVector(-0.297,	0.906,	-0.638))
P.add_single_joint(joints, bones,	"robot arm b 2",	"robot arm b 1",	ba.createVector(-0.379,	0.742,	-0.985), nil, hinge_axes_b,	ba.createVector(-1.4, -T, -T),	ba.createVector(1.0, T, T))
P.add_single_joint(joints, bones,	"robot arm b 3",	"robot arm b 2",	ba.createVector(-0.522,	1.050,	-1.582), nil, hinge_axes_b,	ba.createVector(-1.0, -T, -T),	ba.createVector(1.3, T, T))

P.add_single_joint(joints, bones,	"robot arm c 1",	"root",				ba.createVector(-0.404,	0.906,	0.576))
P.add_single_joint(joints, bones,	"robot arm c 2",	"robot arm c 1",	ba.createVector(-0.664,	0.742,	0.821), nil, hinge_axes_c,	ba.createVector(-1.4, -T, -T),	ba.createVector(1.0, T, T))
P.add_single_joint(joints, bones,	"robot arm c 3",	"robot arm c 2",	ba.createVector(-1.109,	1.050,	1.243), nil, hinge_axes_c,	ba.createVector(-1.0, -T, -T),	ba.createVector(1.3, T, T))

ba.saveModelPhysics(bones, joints, "robot_tripod")
