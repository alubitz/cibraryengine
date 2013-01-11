
local U = dofile("Files/Scripts/ubermodel_util.lua")

local models = { { model = "nu_crab", material = "nu_crab" } }

local bones = { }

U.add_single_bone(bones,	"carapace",		nil,			ba.createVector(	0.0,	1.05,	-0.026	) )
U.add_single_bone(bones,	"head",			"carapace",		ba.createVector(	0.0,	1.11,	0.38	) )U.add_single_bone(bones,	"tail",			"carapace",		ba.createVector(	0.0,	1.04,	-0.83	) )
U.add_single_bone(bones,	"eye",			"head",			ba.createVector(	0.0,	1.04,	0.66	) )

U.add_symetric_bones(bones,	"leg a 1",		"carapace",		ba.createVector(	0.365,	1.015,	0.328	) )
U.add_symetric_bones(bones,	"leg a 2",		"leg a 1",		ba.createVector(	0.437,	0.574,	0.539	) )
U.add_symetric_bones(bones,	"leg a 3",		"leg a 2",		ba.createVector(	0.470,	0.694,	0.993	) )
U.add_symetric_bones(bones,	"leg a point",	"leg a 3",		ba.createVector(	0.288,	0.0,	1.293	) )U.add_symetric_bones(bones,	"leg b 1",		"carapace",		ba.createVector(	0.635,	0.980,	0.022	) )
U.add_symetric_bones(bones,	"leg b 2",		"leg b 1",		ba.createVector(	1.043,	0.742,	0.164	) )
U.add_symetric_bones(bones,	"leg b 3",		"leg b 2",		ba.createVector(	1.631,	1.050,	0.339	) )
U.add_symetric_bones(bones,	"leg b point",	"leg b 3",		ba.createVector(	2.068,	0.0,	0.470	) )

U.add_symetric_bones(bones,	"leg c 1",		"carapace",		ba.createVector(	0.452,	0.891,	-0.421	) )
U.add_symetric_bones(bones,	"leg c 2",		"leg c 1",		ba.createVector(	0.544,	0.676,	-0.693	) )
U.add_symetric_bones(bones,	"leg c 3",		"leg c 2",		ba.createVector(	0.707,	0.786,	-1.076	) )
U.add_symetric_bones(bones,	"leg c point",	"leg c 3",		ba.createVector(	0.798,	0.0,	-1.366	) )

ba.saveUberModel(models, bones, "nu_crab")
