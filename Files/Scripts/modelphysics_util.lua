---------------------------------------------------------------------------
-------- Utility functions for creating a ModelPhysics file (.zzp) --------
---------------------------------------------------------------------------

local P = { }

P.sphere = function(x, y, z, r) return { center = ba.createVector(x, y, z), radius = r } end

P.add_single_bone = function(add_to, name, mass, shape) 
	local result = { name = name, mass = mass, shape = shape }

	add_to[#add_to + 1] = result
	return result
end

P.add_symmetric_bones = function(add_to, name, mass, left_shape)

	local right_shape = { }
	for i, cur_sphere in ipairs(left_shape) do
		right_shape[i] = P.sphere(-cur_sphere.center.x, cur_sphere.center.y, cur_sphere.center.z, cur_sphere.radius)
	end

	local left_result = P.add_single_bone(add_to, "l " .. name, mass, left_shape)
	local right_result = P.add_single_bone(add_to, "r " .. name, mass, right_shape)

	return left_result, right_result
end

P.add_single_joint = function(add_to, bones_list, parent, child, pos, name, axes, min_extents, max_extents, min_torque, max_torque)
	local joint = { name = name, pos = pos, axes = axes, min_extents = min_extents, max_extents = max_extents, min_torque = min_torque, max_torque = max_torque }
	for i, bone in ipairs(bones_list) do
		if bone.name == parent then
			joint.bone_b = i
		elseif bone.name == child then
			joint.bone_a = i
		end
	end

	add_to[#add_to + 1] = joint
	return joint
end

P.add_symmetric_joints = function(add_to, bones_list, parent, child, left_pos, name, left_axes, min_extents, max_extents, min_torque, max_torque)

	local left_joint = { pos = left_pos, min_extents = min_extents, max_extents = max_extents, min_torque = min_torque, max_torque = max_torque }
	local right_joint = { pos = ba.createVector(-left_pos.x, left_pos.y, left_pos.z), min_extents = min_extents, max_extents = max_extents, min_torque = min_torque, max_torque = max_torque }

	if left_axes ~= nil then
		left_joint.axes = left_axes
		right_joint.axes = { }
		for i, axis in ipairs(left_axes) do
			right_joint.axes[i] = ba.createVector(axis.x, -axis.y, -axis.z)
		end
	else
		right_joint.axes = { ba.createVector(1, 0, 0), ba.createVector(0, -1, 0), ba.createVector(0, 0, -1) }
	end

	if name ~= nil then
		left_joint.name = "l " .. name
		right_joint.name = "r " .. name
	end

	local l_parent = "l " .. parent
	local r_parent = "r " .. parent
	local l_child = "l " .. child
	local r_child = "r " .. child

	for i, bone in ipairs(bones_list) do
		local bone_name = bone.name

		if bone_name == parent then
			left_joint.bone_b = i
			right_joint.bone_b = i
		elseif bone_name == child then
			left_joint.bone_a = i
			right_joint.bone_a = i
		elseif bone_name == l_parent then
			left_joint.bone_b = i
		elseif bone_name == r_parent then
			right_joint.bone_b = i
		elseif bone_name == l_child then
			left_joint.bone_a = i
		elseif bone_name == r_child then
			right_joint.bone_a = i
		end
	end

	add_to[#add_to + 1] = left_joint
	add_to[#add_to + 1] = right_joint

	return left_joint, right_joint
end

P.torque_limits = function(x, y, z)
	if y == nil then
		y = x
	end
	if z == nil then
		z = x
	end
	return ba.createVector(-x, -y, -z), ba.createVector(x, y, z)
end

return P
