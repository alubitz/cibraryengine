
local P = dofile("Files/Scripts/modelphysics_util.lua")

local bones = { }

local mass = 98.0 / 18.0;

P.add_single_bone(bones,	"pelvis",	mass, { P.sphere(0.0,		1.07,	-0.12,	0.13),	P.sphere(0.0,		1.05,	0.07,	0.15) } )
P.add_single_bone(bones,	"torso 1",	mass, { P.sphere(0.0,		1.3,	0.0,	0.15),	P.sphere(0.0,		1.32,	-0.07,	0.15),	P.sphere(0.0,		1.28,	-0.06,	0.15) } )
P.add_single_bone(bones,	"torso 2",	mass, { P.sphere(0.0,		1.65,	-0.2,	0.2),	P.sphere(0.0,		1.5,	0.05,	0.2) } )
P.add_single_bone(bones,	"head",		mass, { P.sphere(0.0,		1.8,	0.0,	0.15) } )

P.add_symetric_bones(bones,	"shoulder",	mass, { P.sphere(0.32,	1.69,	0.0,	0.14),		P.sphere(0.34,	1.69,	-0.28,	0.12),		P.sphere(0.3,		1.5,	-0.22,	0.1),	P.sphere(0.28,	1.6,	0.15,	0.1) } )
P.add_symetric_bones(bones,	"arm 1",	mass, { P.sphere(0.28,	1.54,	-0.05,	0.13),		P.sphere(0.53,	1.4,	-0.06,	0.11) } )
P.add_symetric_bones(bones,	"arm 2",	mass, { P.sphere(0.53,	1.4,	-0.06,	0.11),		P.sphere(0.82,	1.25,	0.01,	0.08) } )
P.add_symetric_bones(bones,	"hand",		mass, { P.sphere(0.92,	1.2,	0.01,	0.08),		P.sphere(0.9,	1.21,	0.1,	0.08),		P.sphere(1.08,	1.08,	0.09,	0.07) } )
P.add_symetric_bones(bones,	"leg 1",	mass, { P.sphere(0.15,	1.04,	0.0,	0.15),		P.sphere(0.19,	0.65,	0.01,	0.12),		P.sphere(0.15,	1.04,	-0.09,	0.15) } )
P.add_symetric_bones(bones,	"leg 2",	mass, { P.sphere(0.19,	0.65,	0.01,	0.13),		P.sphere(0.27,	0.15,	-0.11,	0.09),		P.sphere(0.23,	0.4,	-0.11,	0.13) } )
P.add_symetric_bones(bones,	"foot",		mass, { P.sphere(0.27,	0.11,	-0.11,	0.11),		P.sphere(0.22,	0.06,	0.25,	0.06),		P.sphere(0.32,	0.06,	0.25,	0.06) } )

local joints = { }

local hand_axes = {	ba.createVector(11.6,	22.0,	0.0), ba.createVector(0, 0, -1) }

P.add_single_joint(joints, bones,		"pelvis",	"torso 1",	ba.createVector(0.0,	1.34,	-0.1),	nil, nil,										ba.createVector(-0.5,	-0.5,	-0.5),	ba.createVector(0.25,	0.5,	0.5))
P.add_single_joint(joints, bones,		"torso 1",	"torso 2",	ba.createVector(0.0,	1.57,	0.0),	nil, nil,										ba.createVector(-0.5,	-0.5,	-0.5),	ba.createVector(0.25,	0.5,	0.5))
P.add_single_joint(joints, bones,		"torso 2",	"head",		ba.createVector(0.0,	1.73,	0.0),	nil, nil,										ba.createVector(-0.26,	-1.2,	-0.26),	ba.createVector(0.26,	1.2,	0.26))

P.add_symmetric_joints(joints, bones,	"torso 2",	"shoulder",	ba.createVector(0.27,	1.69,	0.0),	nil, nil,										ba.createVector(-0.1,	-0.1,	-0.1),	ba.createVector(0.1,	0.1,	0.1))
P.add_symmetric_joints(joints, bones,	"shoulder",	"arm 1",	ba.createVector(0.28,	1.52,	-0.05))
P.add_symmetric_joints(joints, bones,	"arm 1",	"arm 2",	ba.createVector(0.53,	1.4,	-0.06),	nil, {	ba.createVector(11.6,	22.0,	0.0) },	ba.createVector(-2.618,	-0.02,	-0.02),	ba.createVector(0.02,	0.02,	0.02))
P.add_symmetric_joints(joints, bones,	"arm 2",	"hand",		ba.createVector(0.82,	1.25,	0.01),	nil, hand_axes,									ba.createVector(-0.44,	-0.44,	-0.1),	ba.createVector(0.175,	1.57,	0.1))

P.add_symmetric_joints(joints, bones,	"pelvis",	"leg 1",	ba.createVector(0.15,	1.04,	-0.02),	nil, nil,										ba.createVector(-1.0,	-0.25,	-0.2),	ba.createVector(1.0,	0.25,	1.0))
P.add_symmetric_joints(joints, bones,	"leg 1",	"leg 2",	ba.createVector(0.19,	0.65,	0.01),	nil, {	ba.createVector(28.2,	2.5,	0.0) },	ba.createVector(-0.02,	-0.02,	-0.02),	ba.createVector(2.618,	0.02,	0.02))
P.add_symmetric_joints(joints, bones,	"leg 2",	"foot",		ba.createVector(0.27,	0.14,	-0.11),	nil, {	ba.createVector(1.0,	0.0,	0.0) },	ba.createVector(-1.0,	-0.3,	-0.3),	ba.createVector(1.0,	0.3,	0.3))

ba.saveModelPhysics(bones, joints, "soldier")
