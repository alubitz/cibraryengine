
local U = dofile("Files/Scripts/ubermodel_util.lua")

local models = { { model = "robot arm", material = "crab_bug" } }

local bones = { }

U.add_single_bone(bones,	"robot arm 1",	nil,			ba.createVector(	0.635,	0.980,	0.022	) )