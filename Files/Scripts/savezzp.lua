
local function sphere(x, y, z, r) return { center = ba.createVector(x, y, z), radius = r } end

local function add_single_bone(add_to, name, mass, shape) 
	add_to[#add_to + 1] = { name = name, mass = mass, shape = shape }
end

local function add_symetric_bones(add_to, name, mass, left_shape)

	local right_shape = { }
	for i in ipairs(left_shape) do
		local cur_sphere = left_shape[i]
		right_shape[i] = sphere(-cur_sphere.center.x, cur_sphere.center.y, cur_sphere.center.z, cur_sphere.radius)
	end

	add_single_bone(add_to, "l " .. name, mass, left_shape)
	add_single_bone(add_to, "r " .. name, mass, right_shape)
end

local function add_single_joint(add_to, bones_list, parent, child, pos, name)

	local joint = { name = name, pos = pos }
	for i in ipairs(bones_list) do
		if bones_list[i].name == parent then
			joint.bone_b = i
		elseif bones_list[i].name == child then
			joint.bone_a = i
		end
	end

	add_to[#add_to + 1] = joint
end

local function add_symmetric_joints(add_to, bones_list, parent, child, left_pos, name)

	local left_joint = { pos = left_pos }
	local right_joint = { pos = ba.createVector(-left_pos.x, left_pos.y, left_pos.z) }

	if name ~= nil then
		left_joint.name = "l " .. name
		right_joint.name = "r " .. name
	end

	local l_parent = "l " .. parent
	local r_parent = "r " .. parent
	local l_child = "l " .. child
	local r_child = "r " .. child
	for i in ipairs(bones_list) do
		local bone_name = bones_list[i].name
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
end

local bones = { }

local mass = 98.0 / 18.0;

add_single_bone(bones,		"pelvis",	mass, { sphere(0.0,	1.12,	-0.09,	0.15) } )
add_single_bone(bones,		"torso 1",	mass, { sphere(0.0,	1.34,	-0.2,	0.15) } )
add_single_bone(bones,		"torso 2",	mass, { sphere(0.0,	1.57,	-0.2,	0.15) } )
add_single_bone(bones,		"head",		mass, { sphere(0.0,	1.73,	0.0,	0.15) } )

add_symetric_bones(bones,	"shoulder",	mass, { sphere(0.27,	1.69,	0.0,	0.15) } )
add_symetric_bones(bones,	"arm 1",	mass, { sphere(0.28,	1.52,	-0.05,	0.15) } )
add_symetric_bones(bones,	"arm 2",	mass, { sphere(0.53,	1.4,	-0.06,	0.15) } )
add_symetric_bones(bones,	"hand",		mass, { sphere(0.82,	1.25,	0.01,	0.15) } )
add_symetric_bones(bones,	"leg 1",	mass, { sphere(0.15,	1.04,	-0.02,	0.15) } )
add_symetric_bones(bones,	"leg 2",	mass, { sphere(0.19,	0.65,	0.01,	0.15) } )
add_symetric_bones(bones,	"foot",		mass, { sphere(0.27,	0.14,	-0.11,	0.15) } )

local joints = { }

add_single_joint(joints, bones,		"pelvis",	"torso 1",	ba.createVector(0.0,	1.34,	-0.2))
add_single_joint(joints, bones,		"torso 1",	"torso 2",	ba.createVector(0.0,	1.57,	-0.2))
add_single_joint(joints, bones,		"torso 2",	"head",		ba.createVector(0.0,	1.73,	0.0))

add_symmetric_joints(joints, bones,	"torso 2",	"shoulder",	ba.createVector(0.27,	1.69,	0.0))
add_symmetric_joints(joints, bones,	"shoulder",	"arm 1",	ba.createVector(0.28,	1.52,	-0.05))
add_symmetric_joints(joints, bones,	"arm 1",	"arm 2",	ba.createVector(0.53,	1.4,	-0.06))
add_symmetric_joints(joints, bones,	"arm 2",	"hand",		ba.createVector(0.82,	1.25,	0.01))

add_symmetric_joints(joints, bones,	"pelvis",	"leg 1",	ba.createVector(0.15,	1.04,	-0.02))
add_symmetric_joints(joints, bones,	"leg 1",	"leg 2",	ba.createVector(0.19,	0.65,	0.01))
add_symmetric_joints(joints, bones,	"leg 2",	"foot",		ba.createVector(0.27,	0.14,	-0.11))


ba.saveModelPhysics(bones, joints, "soldier")
