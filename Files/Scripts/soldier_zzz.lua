

local models = { { model = "nuarmor", material = "soldier" } }

local bones = { }

U.add_single_bone(bones,	"pelvis",	nil,		ba.createVector(	0.0,	1.12,	-0.09	) )
U.add_single_bone(bones,	"torso 2",	"torso 1",	ba.createVector(	0.0,	1.57,	-0.2	) )
U.add_single_bone(bones,	"head",		"torso 2",	ba.createVector(	0.0,	1.73,	0.0		) )

U.add_symetric_bones(bones,	"shoulder",	"torso 2",	ba.createVector(	0.27,	1.69,	0.0		) )
U.add_symetric_bones(bones,	"arm 2",	"arm 1",	ba.createVector(	0.53,	1.4,	-0.06	) )
U.add_symetric_bones(bones,	"foot",		"leg 2",	ba.createVector(	0.27,	0.14,	-0.11	) )
U.add_symetric_bones(bones,	"grip",		"hand",		ba.createVector(	0.9,	1.15,	0.04	), ba.createVector(1.47884, 0.625244, 0.625244) )

ba.saveUberModel(models, bones, "soldier")