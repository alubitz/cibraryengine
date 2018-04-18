
local P = dofile("Files/Scripts/modelphysics_util.lua")
local Vec3      = ba.createVector
local Sphere    = P.sphere
local Bone      = P.add_single_bone
local SymBones  = P.add_symmetric_bones
local Joint     = P.add_single_joint
local SymJoints = P.add_symmetric_joints


local b = { }
local j = { }

Bone(b,     "carapace", 60.0, { Sphere( 0.0, 1.0, 0.0, 0.2) } )

local leg_names = { "a", "b", "c" }
for i = 1, 3 do
	local theta = (i - 1) * math.pi * 2.0 / 3.0
	local rx = math.sin(theta)
	local rz = math.cos(theta)
	local bname = "leg " .. leg_names[i] .. " 1"
	Bone(b, bname, 10.0, { Sphere( rx * 0.15, 0.8, rz * 0.15, 0.05), Sphere(rx * 0.5, 0.01, rz * 0.5, 0.01) } )
	Joint(j, b, "carapace", bname, Vec3(rx * 0.15, 0.8, rz * 0.15))
end

ba.saveModelPhysics(b, j, "tripod")
