
local P = dofile("Files/Scripts/modelphysics_util.lua")

local bones = { }

local carapace_spheres =
{
	P.sphere(	0.0,	1.07,	0.56,	0.10	),
	P.sphere(	0.0,	1.18,	-0.34,	0.15	),
	P.sphere(	0.0,	1.04,	-0.63,	0.15	),
	P.sphere(	0.38,	1.00,	-0.55,	0.12	),
	P.sphere(	-0.38,	1.00,	-0.55,	0.12	),
	P.sphere(	0.59,	1.02,	-0.09,	0.15	),
	P.sphere(	-0.59,	1.02,	-0.09,	0.15	)
}

P.add_single_bone(bones,		"carapace",	54.08,	carapace_spheres)
P.add_single_bone(bones,		"head",		4.22,	{ P.sphere(	0.0,	1.008,	0.529,	0.20),	P.sphere(	0.0,	0.850,	0.388,	0.22),	P.sphere(	0.0,	0.9,	-0.1,	0.15) } )
P.add_single_bone(bones,		"tail",		2.15,	{ P.sphere(	0.0,	0.942,	-0.723,	0.25),	P.sphere(	0.0,	0.45,	-0.43,	0.02) } )

P.add_symmetric_bones(bones,	"leg a 1",	0.73,	{ P.sphere(	0.379,	0.838,	0.322,	0.06),	P.sphere(	0.436,	0.544,	0.535,	0.16) } )
P.add_symmetric_bones(bones,	"leg a 2",	0.89,	{ P.sphere(	0.436,	0.544,	0.535,	0.16),	P.sphere(	0.472,	0.694,	0.942,	0.09) } )
P.add_symmetric_bones(bones,	"leg a 3",	0.39,	{ P.sphere(	0.472,	0.694,	0.942,	0.09),	P.sphere(	0.288,	0.01,	1.293,	0.01) } )

P.add_symmetric_bones(bones,	"leg b 1",	1.33,	{ P.sphere(	0.701,	0.906,	0.062,	0.08),	P.sphere(	1.051,	0.700,	0.162,	0.13) } )
P.add_symmetric_bones(bones,	"leg b 2",	1.70,	{ P.sphere(	1.049,	0.741,	0.166,	0.10),	P.sphere(	1.610,	1.034,	0.339,	0.14) } )
P.add_symmetric_bones(bones,	"leg b 3",	1.08,	{ P.sphere(	1.660,	1.058,	0.348,	0.10),	P.sphere(	2.068,	0.01,	0.470,	0.01) } )

P.add_symmetric_bones(bones,	"leg c 1",	1.06,	{ P.sphere(	0.429,	0.869,	-0.474,	0.08),	P.sphere(	0.540,	0.665,	-0.678,	0.13) } )
P.add_symmetric_bones(bones,	"leg c 2",	1.21,	{ P.sphere(	0.540,	0.665,	-0.678,	0.13),	P.sphere(	0.714,	0.781,	-1.091,	0.09) } )
P.add_symmetric_bones(bones,	"leg c 3",	0.72,	{ P.sphere(	0.714,	0.781,	-1.091,	0.09),	P.sphere(	0.798,	0.01,	-1.366,	0.01) } )


local joints = { }

P.add_single_joint(joints, bones,		"head",		"carapace",	ba.createVector(0.0,	1.11,	0.38), nil, nil,			ba.createVector(-0.1, -0.1, -0.1),	ba.createVector(0.1, 0.1, 0.1))
P.add_single_joint(joints, bones,		"tail",		"carapace",	ba.createVector(0.0,	1.04,	-0.83), nil, nil,			ba.createVector(-0.3, -0.3, -0.3),	ba.createVector(0.3, 0.3, 0.3))
-- rotation limit for things that aren't supposed to rotate much
local T = 0.0

local a_hinge_axes = { ba.createVector(0.2068, -0.0443, -0.0094) }
P.add_symmetric_joints(joints, bones,	"leg a 1",	"carapace",	ba.createVector(0.365,	1.015,	0.328))
P.add_symmetric_joints(joints, bones,	"leg a 2",	"leg a 1",	ba.createVector(0.437,	0.574,	0.539), nil, a_hinge_axes,	ba.createVector(-1.4, -T, -T),		ba.createVector(1.0, T, T))
P.add_symmetric_joints(joints, bones,	"leg a 3",	"leg a 2",	ba.createVector(0.470,	0.694,	0.993), nil, a_hinge_axes,	ba.createVector(-1.0, -T, -T),		ba.createVector(1.3, T, T))

local b_hinge_axes = { ba.createVector(0.0336, 0.0, -0.1125) }
P.add_symmetric_joints(joints, bones,	"leg b 1",	"carapace",	ba.createVector(0.701,	0.906,	0.062))
P.add_symmetric_joints(joints, bones,	"leg b 2",	"leg b 1",	ba.createVector(1.043,	0.742,	0.164), nil, b_hinge_axes,	ba.createVector(-1.4, -T, -T),		ba.createVector(1.0, T, T))
P.add_symmetric_joints(joints, bones,	"leg b 3",	"leg b 2",	ba.createVector(1.631,	1.050,	0.339), nil, b_hinge_axes,	ba.createVector(-1.0, -T, -T),		ba.createVector(1.3, T, T))

local c_hinge_axes = { ba.createVector(0.2552, -0.0232, 0.1112) }
P.add_symmetric_joints(joints, bones,	"leg c 1",	"carapace",	ba.createVector(0.452,	0.891,	-0.421))
P.add_symmetric_joints(joints, bones,	"leg c 2",	"leg c 1",	ba.createVector(0.544,	0.676,	-0.693), nil, c_hinge_axes,	ba.createVector(-1.4, -T, -T),		ba.createVector(1.0, T, T))
P.add_symmetric_joints(joints, bones,	"leg c 3",	"leg c 2",	ba.createVector(0.707,	0.786,	-1.076), nil, c_hinge_axes,	ba.createVector(-1.0, -T, -T),		ba.createVector(1.3, T, T))

ba.saveModelPhysics(bones, joints, "crab_bug")
