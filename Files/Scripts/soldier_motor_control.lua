--[[
	hv.bones elements have...
		read-only:
			pos							world-coords position of the bone's CoM
			vel							velocity of the bone
			ori							the bone's rotation matrix
			rot							the bone's angular velocity
			mass						mass
			moi							the moment of inertia matrix
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

ba.println("hv.bones:")
for k,v in pairs(hv.bones) do
	ba.println("\t" .. k .. "; missing torque = " .. (v.desired_torque - v.applied_torque))
end

ba.println("hv.joints:")
for k,v in pairs(hv.joints) do
	if v.a == "pelvis" and v.b ~= "torso 1" then
		v.satisfyA()
		ba.println("\tjoint between " .. v.a .. " and " .. v.b .. ": local torque = " .. v.local_torque)
	end
end

ba.println("pelvis missing torque = " .. (hv.bones.pelvis.desired_torque - hv.bones.pelvis.applied_torque))

ba.println("hv.nozzles:")
for k,v in pairs(hv.nozzles) do
	local desired = hv.controls.jpaccel * (98.0 / 10.0)		-- 98 kg / 10 nozzles
	v.setForce(desired)--ba.createVector(0, 150, 0))
	ba.println("\tnozzle " .. k .. " desired force = " .. desired .. "; got = " .. v.force)
	ba.println("\t" .. (v.ftt * v.force) .. " - " .. v.torque .. " = " .. (v.ftt * v.force - v.torque))
end
