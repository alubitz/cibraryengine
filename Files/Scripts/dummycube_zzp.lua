local P = dofile("Files/Scripts/modelphysics_util.lua")

local bones = { }
local joints = { }

local spheres =
{
	P.sphere(-0.4, -0.4, -0.4, 0.1),
	P.sphere(-0.4, -0.4,  0.4, 0.1),
	P.sphere(-0.4,  0.4, -0.4, 0.1),
	P.sphere(-0.4,  0.4,  0.4, 0.1),
	P.sphere( 0.4, -0.4, -0.4, 0.1),
	P.sphere( 0.4, -0.4,  0.4, 0.1),
	P.sphere( 0.4,  0.4, -0.4, 0.1),
	P.sphere( 0.4,  0.4,  0.4, 0.1)
}

P.add_single_bone(bones,	"main", 5.0, spheres)

ba.saveModelPhysics(bones, joints, "dummycube")
