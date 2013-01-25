
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
]]--

--dofile("Files/Scripts/robot_tripod_zzp.lua")
