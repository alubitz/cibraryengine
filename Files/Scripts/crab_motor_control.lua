
-- for documentation, see the top of soldier_motor_control.lua

-- i'll probably move these functions below to a separate file, since they're potentially useful in any scripted motor control file



-- dot product of two vec3s ... equal to the product of the lengths times the cosine of the angle between them
local function dot(a, b)
	return a.x * b.x + a.y * b.y + a.z * b.z
end

-- cross product of two vec3s ... a vector perpendicular to both inputs, whose magnitude is the product of the lengths times the sine of the angle between them
local function cross(a, b)
	return ba.createVector(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x)
end

-- find the joint between the specified two bones (bones must be provided in the correct order)
local function findJoint(a, b)
	for k,v in pairs(hv.joints) do
		if v.a == a and v.b == b then
			return v
		end
	end
	return nil
end

-- like the above, but will find two joints, one on the left side of the body and the other on the right, if one or both of the joined bones has the corresponding prefix
local function findSymmetricJoints(a, b)
	local left = nil
	local right = nil
	local la = "l " .. a
	local lb = "l " .. b
	local ra = "r " .. a
	local rb = "r " .. b
	for k,v in pairs(hv.joints) do
		if (v.a == la and v.b == b) or (v.a == a and v.b == lb) or (v.a == la and v.b == lb) then
			left = v
		elseif (v.a == ra and v.b == b) or (v.a == a and v.b == rb) or (v.a == ra and v.b == rb) then
			right = v
		end
	end
	return left, right
end



-- put all the joints into named variables
local neck = findJoint( "carapace", "head" )
local tail = findJoint( "carapace", "tail" )

local lja1, rja1 = findSymmetricJoints( "carapace", "leg a 1" )
local lja2, rja2 = findSymmetricJoints( "leg a 1",  "leg a 2" )
local lja3, rja3 = findSymmetricJoints( "leg a 2",  "leg a 3" )

local ljb1, rjb1 = findSymmetricJoints( "carapace", "leg b 1" )
local ljb2, rjb2 = findSymmetricJoints( "leg b 1",  "leg b 2" )
local ljb3, rjb3 = findSymmetricJoints( "leg b 2",  "leg b 3" )

local ljc1, rjc1 = findSymmetricJoints( "carapace", "leg c 1" )
local ljc2, rjc2 = findSymmetricJoints( "leg c 1",  "leg c 2" )
local ljc3, rjc3 = findSymmetricJoints( "leg c 2",  "leg c 3" )


local inv_timestep = 60.0				-- in the c++ code inv_timestep is actually derived from timestep, but w/e
local timestep = 1.0 / inv_timestep


-- [flail wildly]
lja1.setOrientedTorque(ba.createVector(500, 0, 0))
rja1.setOrientedTorque(ba.createVector(500, 0, 0))
ljb1.setOrientedTorque(ba.createVector(500, 0, 0))
rjb1.setOrientedTorque(ba.createVector(500, 0, 0))
ljc1.setOrientedTorque(ba.createVector(500, 0, 0))
rjc1.setOrientedTorque(ba.createVector(500, 0, 0))
