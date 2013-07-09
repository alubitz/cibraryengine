local U = dofile("Files/Scripts/ubermodel_util.lua")

local models = { { model = "nuarmor", material = "soldier" } }

local bones = { }

U.add_single_bone(bones,		"pelvis",	nil,		ba.createVector(	0.0,	1.12,	-0.09	) )U.add_single_bone(bones,		"torso 1",	"pelvis",	ba.createVector(	0.0,	1.34,	-0.2	) )
U.add_single_bone(bones,		"torso 2",	"torso 1",	ba.createVector(	0.0,	1.57,	-0.2	) )
U.add_single_bone(bones,		"head",		"torso 2",	ba.createVector(	0.0,	1.73,	0.0		) )

U.add_symmetric_bones(bones,	"shoulder",	"torso 2",	ba.createVector(	0.27,	1.69,	0.0		) )U.add_symmetric_bones(bones,	"arm 1",	"shoulder",	ba.createVector(	0.28,	1.52,	-0.05	) )
U.add_symmetric_bones(bones,	"arm 2",	"arm 1",	ba.createVector(	0.53,	1.4,	-0.06	) )U.add_symmetric_bones(bones,	"hand",		"arm 2",	ba.createVector(	0.82,	1.25,	0.01	) )U.add_symmetric_bones(bones,	"leg 1",	"pelvis",	ba.createVector(	0.15,	1.04,	-0.02	) )U.add_symmetric_bones(bones,	"leg 2",	"leg 1",	ba.createVector(	0.19,	0.64,	0.01	) )
U.add_symmetric_bones(bones,	"foot",		"leg 2",	ba.createVector(	0.27,	0.14,	-0.11	) )U.add_single_bone(bones,		"eye",		"head",		ba.createVector(	0.0,	1.80,	0.18	) )
U.add_single_bone(bones,		"l grip",	"l hand",	ba.createVector(	1.015, 1.110, -0.430	), ba.createVector(	-0.0703434,	-0.0146932,	2.50207		) )U.add_single_bone(bones,		"r grip",	"r hand",	ba.createVector(	-1.029,	0.990,	0.179	), ba.createVector(	1.27667,	-0.336123,	-0.64284	) )
ba.saveUberModel(models, bones, "soldier")--[[l grip:	pos = 1.015, 1.110, -0.430	x axis = -1.603, 1.192, -0.094	y axis = -1.191, -1.606, -0.054	z axis = -0.108, 0.013, 1.997	pyr = 0.0703434, 0.0146932, -2.50207r grip:	pos = -1.029, 0.990, 0.179	x axis = 1.562, -1.228, -0.228	y axis = 0.514, 0.298, 1.910	z axis = -1.139, -1.550, 0.549	pyr = -1.27667, 0.336123, 0.64284gun barrel pos = 0, 0.08, 0.84]]----[[models = { { model = "gun3", material = "gun" } }bones = { }U.add_single_bone(bones, "main", nil, ba.createVector(0,0,0))ba.saveUberModel(models, bones, "gun3")]]--