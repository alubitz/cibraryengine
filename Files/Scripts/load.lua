
-- tell the game what we need loaded
crab_bug = ba.loadModel("crab_bug")
soldier = ba.loadModel("soldier")
flea = ba.loadModel("flea")

--[[
local A = math.pi * 2.0 / 3.0
local C = math.cos(A)
local S = math.sin(A)

local function rot120(x, y, z)
	ba.println("(" .. 	x				.. ",\t" .. y .. ",\t" .. z					.. ") -->")
	ba.println("\t(" .. x * C - z * S	.. ",\t" .. y .. ",\t" .. x * S + z * C		.. ")")
	ba.println("\t(" .. x * C + z * S	.. ",\t" .. y .. ",\t" .. -x * S + z * C	.. ")")
end

rot120(0.635,	0.980,	0.022	)
rot120(	1.043,	0.742,	0.164	)
rot120(	1.631,	1.050,	0.339	)
rot120(	2.068,	0.0,	0.470	)
]]--

dofile("Files/Scripts/robot_tripod_zzp.lua")
