--[[
	hv.bones elements have...
		read-only:
			pos							world-coords position of the bone's CoM
			vel							velocity of the bone
			ori							the bone's rotation matrix
			rot							the bone's angular velocity
			mass						mass
			moi							the moment of inertia matrix
			impulse_linear				measured net linear impulse (delta of mass * vel) on this bone
			impulse_angular				measured net angular impulse (delta of MoI * angular vel) on this bone
			applied_torque				net torque applied by joints; updates when you use any of the joint methods
		read-write:
			desired_torque				use this to control the values satisfyA and satisfyB try to reach

	hv.joints elements have...
		read-only:
			pos							world-coords position of the joint
			axes						the axes of the joint's rotation (not all are necessarily usable) - specifies the coordinate system of min/max extents/torques and rvec
			min_extents, max_extents	rotation limits of the joint in any direction
			min_torque, max_torque		limits on how much torque the joint motor can apply in any direction
			rvec						current orientation of the joint (orientation of bone b wrt bone a, in the coordinate system specified by axes relative to bone_a)
			local_torque				torque to be applied by the joint in its local coordinate system (same coords as _extents, _torque, and rvec)
			world_torque				world-coords torque to be applied by the joint
			impulse_linear				measured value from the previous timestep, of how much linear impulse was applied to A from B
			impulse_angular				measured value from the previous timestep, of how much angular impulse was applied to A from B
			a							name of the first adjoined bone
			b							name of the second adjoined bone
		methods:
			setWorldTorque(Vec3)		attempt to set the local torque to a value that will produce the given world torque, or as close as possible
			setOrientedTorque(Vec3)		set the local torque to the given value, clamped to the min and max torque range
			satisfyA					like setWorldTorque, but the value used is bone A's desired_torque minus applied_torque
			satisfyB					like setWorldTorque, but the value used is bone B's desired_torque minus applied_torque

	hv.nozzles elements have...
		read-only:
			bone						the name of the bone this jetpack nozzle is attached to
			pos							world-coords position of the nozzle
			center						world-coords center vector of the nozzle
			cone_cossq					square of the cosine of the angle of the cone around "center" within which a force must lie to be valid
			max_force					max magnitude of a force this nozzle can apply
			force						world-coords force to be applied
			torque						whole-dood torque that will result from applying the specified force
			fft							matrix that maps nozzle forces to dood torques
		methods:
			setForce(Vec3)				attempt to set the world force to the given value, or as close as possible within the cone

	hv.controls contains...
		yaw_mat							matrix representing the dood's desired rotation around the y axis (vertical)
		pitch							angle above or below horizontal the dood should be aiming, in radians
		walkvel							desired horizontal velocity (to be achieved by walking)
		jpaccel							desired jetpack acceleration vector (only relevant when jetpacking)
		jump							whether the jump key (spacebar) is pressed

	contact points:
		hv.oldcp contains contact points from the previous timestep, whose net linear and angular impulses are now known
		hv.newcp contains contact points for the current timestep, which has yet to be resolved
	hv.oldcp and hv.newcp elements have...
		a								the name of the first of the two objects in contact (either a bone name, "gun", or "<external>")
		b								the name of the second of the two objects in contact (either a bone name, "gun", or "<external>")
		pos								the position of this contact point
		normal							the normal vector of this contact point (points from A to B)
		restitution						restitution coefficient for this contact point (like "bounciness", or resistance to inter-penetration)
		friction						coefficient of friction between these two objects
		bounce_threshold				a threshold above which these objects may bounce instead of sticking to each other
		impulse_linear					measured value from the previous timestep, of how much linear impulse was applied to A from B
		impulse_angular					measured value from the previous timestep, of how much angular impulse was applied to A from B

	hv.age is how many ticks old the dood is



	type vec3 has...
		x, y, z							read-write
		length							read only; the square root of the sum of the squares of x, y, and z

	create a vec3 with ba.createVector() or ba.createVector(x, y, z)

	supported operations on vec3 A, vec3 B, and scalar k:
		A == B							test whether two vec3s are equal
		-A
		A + B
		A - B
		A * k
		A / k



	type mat3 has indices [0] through [8], in reading order
	also it has the following members...
		inverse							computes the inverse of this 3x3 matrix -- it computes it every time, so don't spam it
		transpose						returns a copy of this matrix, but with the rows and columns swapped - not as expensive, but still computes it every time
										the inverse of a rotation matrix is the same as its transpose, so doing the transpose is cheaper (if it's a rotation matrix)

	create a mat3 with ba.createMat3() or ba.createMat3(m11, m12, m13, m21, m22, m23, m31, m32, m33)

	supported operations on mat3 A, mat3 B, vec3 C, and scalar k:
		A == B							test whether two mat3s are equal
		-A
		A + B
		A - B
		A * B							returns a mat3
		A * C							returns a vec3 !!! use this to rotate/transform stuff
		A * k							returns a mat3
		A / k							returns a mat3

	a "rotation vector" or "rvec" is a vec3 whose axis indicates an axis of rotation, and whose magnitude (length) indicates the angle of rotation (in radians)
	use ba.mat3ToRvec(...) and ba.rvecToMat3(...) to convert back and forth between rvec and rotation matrix

]]--

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



-- lower body control
local pelvis = hv.bones["pelvis"]
local pelvisMissing = pelvis.desired_torque - pelvis.applied_torque
findJoint("pelvis", "l leg 1").setWorldTorque(pelvisMissing * 0.5)
findJoint("pelvis", "r leg 1").satisfyA()

--findJoint("l leg 1", "l leg 2").satisfyA()
--findJoint("l leg 2", "l heel").satisfyA()
--findJoint("r leg 1", "r leg 2").satisfyA()
--findJoint("r leg 2", "r heel").satisfyA()



-- jetpack control
-- tries to achieve the requested amount of thrust by splitting it evenly across all nozzles
-- then if that didn't match the requested amount, splits the remainder ... tries up to 3 times
local desiredThrust = hv.controls.jpaccel * 98.0			-- 98 kg Dood
local actualThrust = ba.createVector()
for i = 1, 3 do
	local missing = desiredThrust - actualThrust
	if missing.length > 0 then
		for k,v in pairs(hv.nozzles) do
			v.setForce(v.force + missing / #hv.nozzles)		-- currently there are 10 nozzles
		end
	end
	actualThrust = ba.createVector()
	for k,v in pairs(hv.nozzles) do
		actualThrust = actualThrust + v.force
	end
end

local netTorque = ba.createVector()
for k,v in pairs(hv.nozzles) do
	netTorque = netTorque + v.torque
end


-- print some debug output ... this is actually surprisingly slow
if true then
	ba.println("")
	ba.println("age = " .. hv.age)
	ba.println("hv.joints:")
	for k,v in pairs(hv.joints) do
		ba.println("\tjoint between '" .. v.a .. "' and '" .. v.b .. "': local torque = " .. v.local_torque .. "; measured impulse L = " .. v.impulse_linear .. ", A = " .. v.impulse_angular)
	end

	ba.println("hv.bones:")
	for k,v in pairs(hv.bones) do
		ba.println("\t" .. k .. ": missing torque = " .. (v.desired_torque - v.applied_torque))
	end

	ba.println("hv.nozzles: desired force = " .. desiredThrust .. "; got = " .. actualThrust .. "; net torque = " .. netTorque)
	for k,v in pairs(hv.nozzles) do
		ba.println("\tnozzle " .. k .. " (on '" .. v.bone .. "'): force = " .. v.force .. "; torque = " .. v.torque)
	end

	ba.println("hv.oldcp:")
	for k,v in pairs(hv.oldcp) do
		ba.println("\tcp between '" .. v.a .. "' and '" .. v.b .. "': pos = " .. v.pos .. "; normal = " .. v.normal .. "; measured impulse L = " .. v.impulse_linear .. "; A = " .. v.impulse_angular)
	end

	ba.println("hv.newcp:")
	for k,v in pairs(hv.newcp) do
		ba.println("\tcp between '" .. v.a .. "' and '" .. v.b .. "': pos = " .. v.pos .. "; normal = " .. v.normal)
	end
end
