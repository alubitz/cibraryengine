
--dofile("Files/Scripts/nu_crab_zzp.lua")
--dofile("Files/Scripts/soldier_zzp.lua")
--dofile("Files/Scripts/soldier_zzz.lua")
--dofile("Files/Scripts/gun_zzp.lua")

--dofile("Files/Scripts/tripod_zzp.lua")
--dofile("Files/Scripts/tripod_zzz.lua")
dofile("Files/Scripts/pendulum_zzu.lua")

-- tell the game what models (zzz & zzp) we need loaded
ba.loadModel("soldier")
ba.loadModel("gun")
ba.loadModel("crab_bug")
--ba.loadModel("flea")
--ba.loadModel("tripod")
ba.loadModel("pendulum")


--[[
local U = dofile("Files/Scripts/ubermodel_util.lua")
models = { { model = "bramble_node", material = "bramble" } }
bones = { }

U.add_single_bone(bones, "main", nil, ba.createVector(0,0,0))

ba.saveUberModel(models, bones, "bramble_node")
]]--
