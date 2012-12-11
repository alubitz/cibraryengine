
local U = dofile("Files/Scripts/ubermodel_util.lua")

local models = { { model = "robot arm", material = "crab_bug" } }

local bones = { }

U.add_single_bone(bones,	"robot arm 1",	nil,			ba.createVector(	0.635,	0.980,	0.022	) )U.add_single_bone(bones,	"robot arm 2",	"robot arm 1",	ba.createVector(	1.043,	0.742,	0.164	) )U.add_single_bone(bones,	"robot arm 3",	"robot arm 2",	ba.createVector(	1.631,	1.050,	0.339	) )U.add_single_bone(bones,	"end effector",	"robot arm 3",	ba.createVector(	2.068,	0.0,	0.470	) )ba.saveUberModel(models, bones, "robot_arm")