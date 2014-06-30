
-- tell the game what models (zzz & zzp) we need loaded
ba.loadModel("soldier")
ba.loadModel("gun")
--ba.loadModel("crab_bug")
--ba.loadModel("flea")


--[[
local U = dofile("Files/Scripts/ubermodel_util.lua")
models = { { model = "bramble_node", material = "bramble" } }
bones = { }

U.add_single_bone(bones, "main", nil, ba.createVector(0,0,0))

ba.saveUberModel(models, bones, "bramble_node")
]]--
