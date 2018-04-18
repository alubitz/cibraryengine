
local U = dofile("Files/Scripts/ubermodel_util.lua")

local models = { { model = "nu_crab", material = "nu_crab" } }

local bones = { }

U.add_single_bone(bones, "carapace", nil, ba.createVector(0.0, 1.0, 0.0))

local leg_names = { "a", "b", "c" }
for i = 1, 3 do
	local theta = (i - 1) * math.pi * 2.0 / 3.0
	local rx = math.sin(theta)
	local rz = math.cos(theta)
	local bname = "leg " .. leg_names[i] .. " 1"
	U.add_single_bone(bones, bname, "carapace", ba.createVector( rx * 0.15, 0.8, rz * 0.15))
end

U.add_single_bone(bones, "eye", "carapace", ba.createVector(0, 1.0, 0.2))

ba.saveUberModel(models, bones, "tripod")
