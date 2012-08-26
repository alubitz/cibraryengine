
local function sphere(x, y, z, r) return { center = ba.createVector(x, y, z), radius = r } end

local function add_single_bone(add_to, name, mass, shape) 
	local result = { name = name, mass = mass, shape = shape }
	add_to[#add_to + 1] = result
	return result
end

local function add_symetric_bones(add_to, name, mass, left_shape)

	local right_shape = { }
	for i, cur_sphere in ipairs(left_shape) do
		right_shape[i] = sphere(-cur_sphere.center.x, cur_sphere.center.y, cur_sphere.center.z, cur_sphere.radius)
	end

	local left_result = add_single_bone(add_to, "l " .. name, mass, left_shape)
	local right_result = add_single_bone(add_to, "r " .. name, mass, right_shape)
	return left_result, right_result
end

local function add_single_joint(add_to, bones_list, parent, child, pos, name, axes, min_extents, max_extents)
	local joint = { name = name, pos = pos, axes = axes, min_extents = min_extents, max_extents = max_extents }
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

local function add_symmetric_joints(add_to, bones_list, parent, child, left_pos, name, left_axes, min_extents, max_extents)

	local left_joint = { pos = left_pos, min_extents = min_extents, max_extents = max_extents }
	local right_joint = { pos = ba.createVector(-left_pos.x, left_pos.y, left_pos.z), min_extents = min_extents, max_extents = max_extents }

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

local bones = { }

local mass = 98.0 / 18.0;

add_single_bone(bones,		"pelvis",	mass, { sphere(0.0,		1.07,	-0.12,	0.13),	sphere(0.0,		1.05,	0.07,	0.15) } )
add_single_bone(bones,		"torso 1",	mass, { sphere(0.0,		1.3,	0.0,	0.15),	sphere(0.0,		1.32,	-0.07,	0.15),	sphere(0.0,		1.28,	-0.06,	0.15) } )
add_single_bone(bones,		"torso 2",	mass, { sphere(0.0,		1.65,	-0.2,	0.2),	sphere(0.0,		1.5,	0.05,	0.2) } )
add_single_bone(bones,		"head",		mass, { sphere(0.0,		1.8,	0.0,	0.15) } )

add_symetric_bones(bones,	"shoulder",	mass, { sphere(0.32,	1.69,	0.0,	0.14),	sphere(0.34,	1.69,	-0.28,	0.12),	sphere(0.3,		1.5,	-0.22,	0.1),	sphere(0.28,	1.6,	0.15,	0.1) } )
add_symetric_bones(bones,	"arm 1",	mass, { sphere(0.28,	1.54,	-0.05,	0.13),	sphere(0.53,	1.4,	-0.06,	0.11) } )
add_symetric_bones(bones,	"arm 2",	mass, { sphere(0.53,	1.4,	-0.06,	0.11),	sphere(0.82,	1.25,	0.01,	0.08) } )
add_symetric_bones(bones,	"hand",		mass, { sphere(0.92,	1.2,	0.01,	0.08),	sphere(0.9,		1.21,	0.1,	0.08),	sphere(1.08,	1.08,	0.09,	0.07) } )
add_symetric_bones(bones,	"leg 1",	mass, { sphere(0.15,	1.04,	0.0,	0.15),	sphere(0.19,	0.65,	0.01,	0.12),	sphere(0.15,	1.04,	-0.09,	0.15) } )
add_symetric_bones(bones,	"leg 2",	mass, { sphere(0.19,	0.65,	0.01,	0.13),	sphere(0.27,	0.15,	-0.11,	0.09),	sphere(0.23,	0.4,	-0.11,	0.13) } )
add_symetric_bones(bones,	"foot",		mass, { sphere(0.27,	0.11,	-0.11,	0.11),	sphere(0.22,	0.06,	0.25,	0.06),	sphere(0.32,	0.06,	0.25,	0.06) } )

local joints = { }

local hand_axes = {	ba.createVector(11.6,	22.0,	0.0), ba.createVector(0, 0, -1) }

add_single_joint(joints, bones,		"pelvis",	"torso 1",	ba.createVector(0.0,	1.34,	-0.1),	nil, nil,										ba.createVector(-0.5,	-0.5,	-0.5),	ba.createVector(0.25,	0.5,	0.5))
add_single_joint(joints, bones,		"torso 1",	"torso 2",	ba.createVector(0.0,	1.57,	0.0),	nil, nil,										ba.createVector(-0.5,	-0.5,	-0.5),	ba.createVector(0.25,	0.5,	0.5))
add_single_joint(joints, bones,		"torso 2",	"head",		ba.createVector(0.0,	1.73,	0.0),	nil, nil,										ba.createVector(-0.26,	-1.2,	-0.26),	ba.createVector(0.26,	1.2,	0.26))

add_symmetric_joints(joints, bones,	"torso 2",	"shoulder",	ba.createVector(0.27,	1.69,	0.0),	nil, nil,										ba.createVector(-0.1,	-0.1,	-0.1),	ba.createVector(0.1,	0.1,	0.1))
add_symmetric_joints(joints, bones,	"shoulder",	"arm 1",	ba.createVector(0.28,	1.52,	-0.05))
add_symmetric_joints(joints, bones,	"arm 1",	"arm 2",	ba.createVector(0.53,	1.4,	-0.06),	nil, {	ba.createVector(11.6,	22.0,	0.0) },	ba.createVector(-2.618,	-0.02,	-0.02),	ba.createVector(0.02,	0.02,	0.02))
add_symmetric_joints(joints, bones,	"arm 2",	"hand",		ba.createVector(0.82,	1.25,	0.01),	nil, hand_axes,									ba.createVector(-0.44,	-0.44,	-0.1),	ba.createVector(0.175,	1.57,	0.1))

add_symmetric_joints(joints, bones,	"pelvis",	"leg 1",	ba.createVector(0.15,	1.04,	-0.02),	nil, nil,										ba.createVector(-1.0,	-0.25,	-0.2),	ba.createVector(1.0,	0.25,	1.0))
add_symmetric_joints(joints, bones,	"leg 1",	"leg 2",	ba.createVector(0.19,	0.65,	0.01),	nil, {	ba.createVector(28.2,	2.5,	0.0) },	ba.createVector(-0.02,	-0.02,	-0.02),	ba.createVector(2.618,	0.02,	0.02))
add_symmetric_joints(joints, bones,	"leg 2",	"foot",		ba.createVector(0.27,	0.14,	-0.11),	nil, {	ba.createVector(1.0,	0.0,	0.0) },	ba.createVector(-1.0,	-0.3,	-0.3),	ba.createVector(1.0,	0.3,	0.3))

ba.saveModelPhysics(bones, joints, "soldier")
