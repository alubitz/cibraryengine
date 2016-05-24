--[[
	hv.bones elements have...
		read-only:
			pos							world-coords position of the bone's CoM
			vel							velocity of the bone
			orix, oriy, oriz			the axes of the bone's rotation matrix
			rot							the bone's angular velocity
			mass						mass
			moix, moiy, moiz			the axes of the moment of inertia matrix
			applied_torque				net torque applied by joints; updates when you use any of the joint methods
		read-write:
			desired_torque				use this to control the values satisfyA and satisfyB try to reach

	hv.joints elements have...
		read-only:
			pos							world-coords position of the joint
			axisx, axisy, axisz			the axes of the joint's rotation (not all are necessarily usable)
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
			fftx, ffty, fftz			axes of the matrix that maps nozzle forces to dood torques
		methods:
			setForce(Vec3)				attempt to set the world force to the given value, or as close as possible within the cone
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

for k,v in pairs(hv.nozzles) do
	v.setForce(ba.createVector(0, 150, 0))
end
