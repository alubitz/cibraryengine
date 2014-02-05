
local P = dofile("Files/Scripts/modelphysics_util.lua")

local bones = { }

local carapace_spheres =
{
	P.sphere(0.0, 6.15,  5.68, 2.2),
	P.sphere(0.0, 8.42,  2.29, 4.0),
	P.sphere(0.0, 8.50, -2.51, 4.5),
	P.sphere(0.0, 9.08, -8.17, 3.2)
}

P.add_single_bone(bones,     "carapace", 200, carapace_spheres)

P.add_symmetric_bones(bones, "leg a 1",  100, { P.sphere(  2.60,  7.25,  3.41, 1.2 ), P.sphere(  3.85,  5.59,   5.32, 0.5 ) } )
P.add_symmetric_bones(bones, "leg a 2",  100, { P.sphere(  3.85,  5.59,  5.32, 0.5 ), P.sphere(  5.09,  6.77,   8.38, 0.5 ) } )
P.add_symmetric_bones(bones, "leg a 3",  100, { P.sphere(  5.09,  6.77,  8.38, 0.5 ), P.sphere(  6.68,  0.20,  12.27, 0.2 ) } )

P.add_symmetric_bones(bones, "leg b 1",  100, { P.sphere(  3.45,  7.26,  1.09, 1.2 ), P.sphere(  6.69,  5.76,   2.33, 0.5 ) } )
P.add_symmetric_bones(bones, "leg b 2",  100, { P.sphere(  6.69,  5.76,  2.33, 0.5 ), P.sphere( 10.62,  8.63,   4.71, 0.5 ) } )
P.add_symmetric_bones(bones, "leg b 3",  100, { P.sphere( 10.62,  8.63,  4.71, 0.5 ), P.sphere( 16.38,  0.20,   8.56, 0.2 ) } )

P.add_symmetric_bones(bones, "leg c 1",  100, { P.sphere(  3.39,  7.23, -1.09, 1.2 ), P.sphere(  5.12,  5.71,  -3.53, 0.5 ) } )
P.add_symmetric_bones(bones, "leg c 2",  100, { P.sphere(  5.12,  5.71, -3.53, 0.5 ), P.sphere(  5.88, 10.23,  -7.98, 0.5 ) } )
P.add_symmetric_bones(bones, "leg c 3",  100, { P.sphere(  5.88, 10.23, -7.98, 0.5 ), P.sphere(  7.05,  0.20, -15.31, 0.2 ) } )

local joints = { }

P.add_symmetric_joints(joints, "leg a 1", "carapace", ba.createVector(  2.60,  7.25,  3.41 ))
P.add_symmetric_joints(joints, "leg a 2", "leg a 1",  ba.createVector(  3.85,  5.59,  5.32 ))
P.add_symmetric_joints(joints, "leg a 3", "leg a 2",  ba.createVector(  5.09,  6.77,  8.38 ))

P.add_symmetric_joints(joints, "leg b 1", "carapace", ba.createVector(  3.45,  7.26,  1.09 ))
P.add_symmetric_joints(joints, "leg b 2", "leg b 1",  ba.createVector(  6.69,  5.76,  2.33 ))
P.add_symmetric_joints(joints, "leg b 3", "leg b 2",  ba.createVector( 10.62,  8.63,  4.71 ))

P.add_symmetric_joints(joints, "leg c 1", "carapace", ba.createVector(  3.39,  7.23, -1.09 ))
P.add_symmetric_joints(joints, "leg c 2", "leg c 1",  ba.createVector(  5.12,  5.71, -3.53 ))
P.add_symmetric_joints(joints, "leg c 3", "leg c 2",  ba.createVector(  5.88, 10.23, -7.98 ))

ba.saveModelPhysics(bones, joints, "flea")