-- for documentation, see the top of soldier_motor_control.lua
--Functions used in motor control scripts
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