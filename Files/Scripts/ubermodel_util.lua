-------------------------------------------------------------------------
-------- Utility functions for creating an UberModel file (.zzz) --------
-------------------------------------------------------------------------

local U = { }

U.add_single_bone = function(add_to, name, parent_name, pos, ori)
	local result = { name = name, parent = parent_name, pos = pos, ori = ori }
	add_to[#add_to + 1] = result
	return result
end

U.add_symmetric_bones = function(add_to, name, parent_name, left_pos, left_ori)
	local left_result = { name = "l " .. name, pos = left_pos, ori = left_ori }
	local right_result = { name = "r " .. name, pos = ba.createVector(-left_pos.x, left_pos.y, left_pos.z) }

	if left_ori then
		right_result.ori = ba.createVector(left_ori.x, -left_ori.y, -left_ori.z)
	end
	if parent_name ~= nil then		local l_parent = "l " .. parent_name
		local r_parent = "r " .. parent_name

		for i, bone in ipairs(add_to) do
			local bone_name = bone.name			if bone_name == parent_name then
				left_result.parent = parent_name
				right_result.parent = parent_name
			elseif bone_name == l_parent then
				left_result.parent = l_parent
			elseif bone_name == r_parent then
				right_result.parent = r_parent
			end
		end
	end

	add_to[#add_to + 1] = left_result
	add_to[#add_to + 1] = right_result

	return left_result, right_result
end

return U
