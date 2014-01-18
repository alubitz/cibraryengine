
local P = dofile("Files/Scripts/modelphysics_util.lua")

local bones = { }

local mass = 98.0 / 18.0;

P.add_single_bone(bones,		"pelvis",	mass, { P.sphere(0.0,  1.08,  0.03, 0.18),	P.sphere( 0.14, 1.07, -0.07, 0.16),	P.sphere(-0.14, 1.07, -0.07, 0.16) } )
P.add_single_bone(bones,		"torso 1",	mass, { P.sphere(0.09, 1.28,  0.06, 0.15),	P.sphere(-0.09, 1.28,  0.06, 0.15),	P.sphere( 0.0,  1.37, -0.04, 0.20),	P.sphere(0.0,  1.28, -0.01, 0.20) } )
P.add_single_bone(bones,		"torso 2",	mass, { P.sphere(0.0,  1.65, -0.2,  0.20),	P.sphere( 0.14, 1.50,  0.09, 0.16),	P.sphere(-0.14, 1.50,  0.09, 0.16) } )
P.add_single_bone(bones,		"head",		mass, { P.sphere(0.0,  1.8,   0.0,  0.15) } )

P.add_symmetric_bones(bones,	"shoulder",	mass, { P.sphere(0.29, 1.52,  0.0,  0.16),	P.sphere( 0.36, 1.66, -0.05, 0.11),	P.sphere( 0.42, 1.67, -0.31, 0.14),	P.sphere(0.30, 1.60, -0.37, 0.14) } )
P.add_symmetric_bones(bones,	"arm 1",	mass, { P.sphere(0.30, 1.51, -0.02, 0.14),	P.sphere( 0.53, 1.4,  -0.05, 0.11) } )
P.add_symmetric_bones(bones,	"arm 2",	mass, { P.sphere(0.53, 1.4,  -0.05, 0.11),	P.sphere( 0.77, 1.27,  0.02, 0.10) } )
P.add_symmetric_bones(bones,	"hand",		mass, { P.sphere(0.92, 1.2,   0.01, 0.08),	P.sphere( 0.9,  1.21,  0.1,  0.08),	P.sphere( 1.08, 1.08,  0.09, 0.07) } )
P.add_symmetric_bones(bones,	"leg 1",	mass, { P.sphere(0.15, 1.04,  0.03, 0.15),	P.sphere( 0.19, 0.63,  0.05, 0.13),	P.sphere( 0.15, 1.04, -0.04, 0.13) } )
P.add_symmetric_bones(bones,	"leg 2",	mass, { P.sphere(0.19, 0.63,  0.05, 0.13),	P.sphere( 0.23, 0.28,  0.04, 0.11),	P.sphere( 0.23, 0.30, -0.10, 0.11) } )
P.add_symmetric_bones(bones,	"foot",		mass, { P.sphere(0.23, 0.11, -0.10, 0.11),	P.sphere( 0.27, 0.07,  0.26, 0.07),	P.sphere( 0.21, 0.07,  0.26, 0.07) } )

local joints = { }

local hand_axes = {	ba.createVector(11.6,	22.0,	0.0), ba.createVector(0, 0, -1) }

P.add_single_joint(joints, bones,		"pelvis",	"torso 1",	ba.createVector(0.0,  1.34, -0.1),	nil, nil,									ba.createVector(-0.5,  -0.5,  -0.25),	ba.createVector(0.25,  0.5,  0.25))
P.add_single_joint(joints, bones,		"torso 1",	"torso 2",	ba.createVector(0.0,  1.57,  0.0),	nil, nil,									ba.createVector(-0.5,  -0.5,  -0.25),	ba.createVector(0.25,  0.5,  0.25))
P.add_single_joint(joints, bones,		"torso 2",	"head",		ba.createVector(0.0,  1.73,  0.0),	nil, nil,									ba.createVector(-0.4,  -1.2,  -0.26),	ba.createVector(0.3,   1.2,  0.26))
P.add_symmetric_joints(joints, bones,	"torso 2",	"shoulder",	ba.createVector(0.22, 1.65, -0.16),	nil, nil,									ba.createVector(-0.25, -0.25, -0.25),	ba.createVector(0.25,  0.25, 0.25))
P.add_symmetric_joints(joints, bones,	"shoulder",	"arm 1",	ba.createVector(0.30, 1.51, -0.04),	nil, nil,									ba.createVector(-1.8,  -1.8,  -1.0 ),	ba.createVector(1.8,   1.0,  0.8 ))
P.add_symmetric_joints(joints, bones,	"arm 1",	"arm 2",	ba.createVector(0.53, 1.4,  -0.04),	nil, hand_axes,								ba.createVector(-2.0,  -0.5,  -1.0 ),	ba.createVector(0.5,   0.5,  1.0 ))
P.add_symmetric_joints(joints, bones,	"arm 2",	"hand",		ba.createVector(0.84, 1.25,  0.02),	nil, hand_axes,								ba.createVector(-1.0,  -1.5,  -1.8 ),	ba.createVector(1.0,   1.5,  1.8 ))
P.add_symmetric_joints(joints, bones,	"pelvis",	"leg 1",	ba.createVector(0.15, 1.04, -0.02),	nil, nil,									ba.createVector(-1.0,  -0.25, -0.2 ),	ba.createVector(1.0,   0.25, 1.0 ))
P.add_symmetric_joints(joints, bones,	"leg 1",	"leg 2",	ba.createVector(0.19, 0.63,  0.05),	nil, { ba.createVector(28.2, 2.5, 0.0) },	ba.createVector(-0.02, -0.02, -0.02),	ba.createVector(1.745, 0.02, 0.02))
P.add_symmetric_joints(joints, bones,	"leg 2",	"foot",		ba.createVector(0.23, 0.16, -0.06),	nil, { ba.createVector(1.0,  0.0, 0.0) },	ba.createVector(-1.0,  -0.3,  -0.3 ),	ba.createVector(1.0,   0.3,  0.3 ))

ba.saveModelPhysics(bones, joints, "soldier")
