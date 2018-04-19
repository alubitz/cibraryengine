
local P = dofile("Files/Scripts/modelphysics_util.lua")
local Vec3      = ba.createVector
local Sphere    = P.sphere
local Bone      = P.add_single_bone
local SymBones  = P.add_symmetric_bones
local Joint     = P.add_single_joint
local SymJoints = P.add_symmetric_joints


local b = { }
local j = { }

Bone(b,     "carapace", 64.0, { Sphere( 0.0, 1.0, 0.0, 0.2) } )

local leg_names = { "a", "b", "c" }
local hip_distance = 0.15
local toe_distance = 1.00

for i = 1, 3 do

	local bname = "leg " .. leg_names[i] .. " 1"

	local theta = (i - 1) * math.pi * 2.0 / 3.0
	local rx = math.sin(theta)
	local rz = math.cos(theta)
	local rm = ba.createMat3(rx, 0, rz, 0, 1, 0, -rz, 0, rx)

	local axes = { rm * Vec3(0, 0, 1), Vec3(0, 1, 0) }
	local spheres = {
		{ center = rm * Vec3(hip_distance, 0.8,  0), radius = 0.05 },
		{ center = rm * Vec3(toe_distance, 0.01, 0), radius = 0.01 }
	}

	Bone(b, bname, 12.0, spheres)
	Joint(j, b, "carapace", bname, rm * Vec3(hip_distance, 0.8, 0), nil, axes, Vec3(-3, 0, 0), Vec3(3, 0, 0) )

end

ba.saveModelPhysics(b, j, "tripod")
