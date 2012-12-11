
local P = dofile("Files/Scripts/modelphysics_util.lua")

local bones = { }P.add_single_bone(bones,	"robot arm 1",	1.33, { P.sphere(	0.701,	0.906,	0.062,	0.08),	P.sphere(	1.051,	0.700,	0.162,	0.13) } )P.add_single_bone(bones,	"robot arm 2",	1.70, { P.sphere(	1.049,	0.741,	0.166,	0.10),	P.sphere(	1.610,	1.034,	0.339,	0.14) } )
P.add_single_bone(bones,	"robot arm 3",	1.08, { P.sphere(	1.660,	1.058,	0.348,	0.10),	P.sphere(	2.068,	0.01,	0.470,	0.01) } )
local joints = { }

P.add_single_joint(joints, bones,	"robot arm 1",	"robot arm 2",	ba.createVector(1.043,	0.742,	0.164))
P.add_single_joint(joints, bones,	"robot arm 2",	"robot arm 3",	ba.createVector(1.631,	1.050,	0.339))

ba.saveModelPhysics(bones, joints, "robot_arm")
