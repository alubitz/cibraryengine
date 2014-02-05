local U = dofile("Files/Scripts/ubermodel_util.lua")

local models = { { model = "flea", material = "flea" } }

local bones = { }

U.add_single_bone(bones,     "carapace", nil,        ba.createVector(  0.00,  0.00,  0.00 ) )

U.add_symmetric_bones(bones, "leg a 1",  "carapace", ba.createVector(  2.60,  7.25,  3.41 ) )
U.add_symmetric_bones(bones, "leg a 2",  "leg a 1",  ba.createVector(  3.85,  5.59,  5.32 ) )
U.add_symmetric_bones(bones, "leg a 3",  "leg a 2",  ba.createVector(  5.09,  6.77,  8.38 ) )
U.add_symmetric_bones(bones, "leg b 1",  "carapace", ba.createVector(  3.45,  7.26,  1.09 ) )
U.add_symmetric_bones(bones, "leg b 2",  "leg b 1",  ba.createVector(  6.69,  5.76,  2.33 ) )
U.add_symmetric_bones(bones, "leg b 3",  "leg b 2",  ba.createVector( 10.62,  8.63,  4.71 ) )

U.add_symmetric_bones(bones, "leg c 1",  "carapace", ba.createVector(  3.39,  7.23, -1.09 ) )
U.add_symmetric_bones(bones, "leg c 2",  "leg c 1",  ba.createVector(  5.12,  5.71, -3.53 ) )
U.add_symmetric_bones(bones, "leg c 3",  "leg c 2",  ba.createVector(  5.88, 10.23, -7.98 ) )

ba.saveUberModel(models, bones, "flea")
