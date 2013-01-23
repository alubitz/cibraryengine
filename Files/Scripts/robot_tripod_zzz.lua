
local U = dofile("Files/Scripts/ubermodel_util.lua")

local models = { { model = "robot tripod", material = "crab_bug" } }

local bones = { }

U.add_single_bone(bones,	"root",				nil,				ba.createVector(	0.0,	1.081,	0.0		) )

U.add_single_bone(bones,	"robot arm a 1",	"root",				ba.createVector(	0.635,	0.980,	0.022	) )U.add_single_bone(bones,	"robot arm a 2",	"robot arm a 1",	ba.createVector(	1.043,	0.742,	0.164	) )U.add_single_bone(bones,	"robot arm a 3",	"robot arm a 2",	ba.createVector(	1.631,	1.050,	0.339	) )U.add_single_bone(bones,	"end effector a",	"robot arm a 3",	ba.createVector(	2.068,	0.0,	0.470	) )

U.add_single_bone(bones,	"robot arm b 1",	"root",				ba.createVector(	-0.298,	0.980,	-0.561	) )
U.add_single_bone(bones,	"robot arm b 2",	"robot arm b 1",	ba.createVector(	-0.379,	0.742,	-0.985	) )
U.add_single_bone(bones,	"robot arm b 3",	"robot arm b 2",	ba.createVector(	-0.522,	1.05,	-1.582	) )
U.add_single_bone(bones,	"end effector b",	"robot arm b 3",	ba.createVector(	-0.627,	0.0,	-2.026	) )

U.add_single_bone(bones,	"robot arm c 1",	"root",				ba.createVector(	-0.337,	0.980,	0.539	) )
U.add_single_bone(bones,	"robot arm c 2",	"robot arm c 1",	ba.createVector(	-0.664,	0.742,	0.821	) )
U.add_single_bone(bones,	"robot arm c 3",	"robot arm c 2",	ba.createVector(	-1.109,	1.05,	1.243	) )
U.add_single_bone(bones,	"end effector c",	"robot arm c 3",	ba.createVector(	-1.441,	0.0,	1.556	) )


ba.saveUberModel(models, bones, "robot_tripod")