local P = dofile("Files/Scripts/modelphysics_util.lua")

local bones = { }
local joints = { }

--[[

	l grip (gun)        =  0.000,  0.000,  0.468
	l grip (hand)       =  0.990,  1.113,  0.037

	r grip (gun)        =  0.000, -0.063, -0.152
	r grip (hand)       = -0.959,  1.098,  0.077

	gun barrel pos      =  0.000,  0.087,  0.842
	grenade barrel pos  =  0.000,  0.034,  0.840

]]--

local spheres =
{
	P.sphere(0.00,  0.06,  0.80, 0.05),
	P.sphere(0.00,  0.11, -0.56, 0.05),
	P.sphere(0.00, -0.17, -0.20, 0.04),
	P.sphere(0.00, -0.20,  0.12, 0.03),
	P.sphere(0.00,  0.12,  0.18, 0.06)
}

P.add_single_bone(bones,	"main", 18.0, spheres)

ba.saveModelPhysics(bones, joints, "gun")
