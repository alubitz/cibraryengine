-----------------------------------------------------------------------------------------------------------------------------------
-------- Hierarchical system for goal-driven agent behavior, based on Mat Buckland's book "Programming Game AI by Example" --------
-----------------------------------------------------------------------------------------------------------------------------------

GoalStatus = {}
GoalStatus.INACTIVE = 1
GoalStatus.ACTIVE = 2
GoalStatus.COMPLETED = 3
GoalStatus.FAILED = 4

function create_goal(owner, activate, process, terminate, handle_message, composite)

	local goal = {}

	goal.owner = owner
	goal.status = GoalStatus.INACTIVE

	goal.activate = function()
		activate(goal)
		goal.status = GoalStatus.ACTIVE
	end

	goal.terminate = function() terminate(goal) end
	goal.isComplete = function() return goal.status == GoalStatus.COMPLETED end
	goal.hasFailed = function() return goal.status == GoalStatus.FAILED end
	goal.process = function() process(goal) return goal.status end

	if composite then
		goal.subgoals = {}

		goal.process_subgoals = function()
			while #goal.subgoals > 0 and (goal.subgoals[1].isComplete() or goal.subgoals[1].hasFailed()) do
				goal.subgoals[1].terminate()
				table.remove(goal.subgoals, 1)
			end

			if #goal.subgoals > 0 then
				local status_of_goals = goal.subgoals[1].process()
				if status_of_goals == GoalStatus.COMPLETED and #goal.subgoals > 0 then
					return GoalStatus.ACTIVE
				else
					return status_of_goals
				end
			else
				return GoalStatus.COMPLETED
			end
		end

		goal.remove_all_subgoals = function()
			for i = 1, #goal.subgoals do goal.subgoals[i].terminate() end
			goal.subgoals = {}
		end

		goal.handle_message = function(message)
			if not(#goal.subgoals >= 1 and goal.subgoals[1].handle_message(message)) then
				return handle_message(goal, message)			-- only handle this ourselves if the first subgoal didn't handle it
			else
				return true
			end
		end

		goal.add_subgoal = function(subgoal) table.insert(goal.subgoals, 1, subgoal) end
	else
		goal.handle_message = function(message) return handle_message(goal, message) end
	end

	return goal

end

function goal_traverse_edge(owner, edge, final)

	local function terminate(goal) end
	local function handle_message(goal, message) end

	local function activate(goal)
		if final then
			get_steering_behavior(owner).moveTo(edge.pos)
		else
			get_steering_behavior(owner).moveThrough(edge.pos)
		end
	end

	local function process(goal)

		if goal.status == GoalStatus.INACTIVE then goal.activate() end

		local tol
		if final then tol = 3 else tol = 3 end

		if (edge.pos - goal.owner.position).length < tol then
			goal.status = GoalStatus.COMPLETED
			get_steering_behavior(goal.owner).cancel()
		end
	end

	local result = create_goal(owner, activate, process, terminate, handle_message, false)
	result.edge = edge
	return result
end

function goal_follow_path(owner, path)

	local function terminate(goal) end
	local function handle_message(goal, message) end

	local function activate(goal)
		-- maybe somebody gave us an object containing a path, instead of a path?
		if goal.path.path then goal.path = goal.path.path end

		local edge = goal.path[1]
		if edge then
			goal.add_subgoal(goal_traverse_edge(owner, edge, #goal.path > 1))
			table.remove(goal.path, 1)
		end
	end

	local function process(goal)
		if goal.status == GoalStatus.INACTIVE then goal.activate() end

		goal.status = goal.process_subgoals()

		-- if the subgoal completed but there are more nodes in the path, keep this goal active!
		if goal.status == GoalStatus.COMPLETED then
			if #goal.path > 0 then
				goal.activate()
			else
				get_steering_behavior(goal.owner).cancel()
			end
		end
	end

	local result = create_goal(owner, activate, process, terminate, handle_message, true)
	result.path = path

	return result
end

function goal_find_path(owner, source_nav, dest_nav, result_out)

	local function terminate(goal) goal.status = GoalStatus.COMPLETED end
	local function handle_message(goal, message) end

	local function activate(goal)
		goal.path_search = gs.newPathSearch(source_nav, dest_nav)
	end

	local function process(goal)
		if goal.status == GoalStatus.INACTIVE then goal.activate(goal) end
		goal.path_search.think(5)

		if goal.path_search.finished then
			result_out.path = goal.path_search.solution
			goal.status = GoalStatus.COMPLETED
		end
	end

	return create_goal(owner, activate, process, terminate, handle_message, false)
end

function goal_move_to_position(owner, position, distance)

	local function terminate(goal) goal.status = GoalStatus.COMPLETED end
	local function handle_message(goal, message) end

	local path_result = {}

	local function activate(goal)
		goal.add_subgoal(goal_follow_path(owner, path_result))
		goal.add_subgoal(goal_find_path(owner, gs.getNearestNav(owner.position), gs.getNearestNav(position), path_result))
	end

	local function process(goal)

		if goal.status == GoalStatus.INACTIVE then goal.activate() end

		if (goal.owner.position - goal.position).length < goal.distance then
			goal.remove_all_subgoals()
		end

		goal.status = goal.process_subgoals()

	end

	local result = create_goal(owner, activate, process, terminate, handle_message, true)
	result.position = position
	result.distance = distance
	return result
end

-- move to target when not in range; attack when in range
function goal_move_attack(owner, target)

	local function terminate(goal) goal.status = GoalStatus.COMPLETED end
	local function handle_message(goal, message) end

	local function activate(goal)
		if not goal.in_range() then
			goal.add_move_subgoal()
		end
	end

	local function process(goal)
		-- if target is dead, this goal is done
		if not target.is_valid then
			goal.remove_all_subgoals()
			goal.status = GoalStatus.COMPLETED
			return
		end

		local sub_status = goal.process_subgoals()
		if sub_status == GoalStatus.ACTIVE then
			-- we are already moving toward the target
			if goal.in_range() then
				-- we are now close enough to the target
				goal.remove_all_subgoals()
			else
				local dist = (goal.move_to - gs.getNearestNav(target.position).pos).length
				if not goal.move_to or dist > 7 then
					-- target has moved away from where we were heading
					goal.remove_all_subgoals()
					goal.add_move_subgoal()
				end
			end
		elseif sub_status == GoalStatus.COMPLETED then
			-- see if target has gotten out of range
			if not goal.in_range() then
				goal.add_move_subgoal()
			else
				get_steering_behavior(goal.owner).attackMove(target.position)
			end
		end
	end

	local result = create_goal(owner, activate, process, terminate, handle_message, true)
	result.target = target

	result.in_range = function()
		if not target.is_valid then
			return false
		else
			local dx = result.target.position - result.owner.position
			return dx.length < 10 --50
		end
	end

	result.add_move_subgoal = function()
		result.move_to = result.target.position
		result.add_subgoal(goal_move_to_position(result.owner, result.move_to, 10))
	end

	return result
end

----------------------------------------------------------------------------------------------------
-------- Steering behaviors; handles low-level control in order to complete certain actions --------
----------------------------------------------------------------------------------------------------

if steering_behaviors ~= nil then
	for k in pairs(steering_behaviors) do
		steering_behaviors[k] = nil
	end
else
	steering_behaviors = {}
end

function do_steering_behavior(dood, control_state)
	get_steering_behavior(dood).update(dood, control_state)
end

function get_steering_behavior(dood)
	if not steering_behaviors[dood.id] then
		steering_behaviors[dood.id] = create_steering_behavior(dood)
	end
	return steering_behaviors[dood.id]
end

function create_steering_behavior(owner)
	local sb = {}

	sb.owner = owner

	sb.cancel = function()
		sb.update = function(dood, control_state)
			control_state.primary_fire = false
			control_state.forward = 0
			control_state.leap = false
		end
	end

	local function makeMoveFunction(pos, attack)
		return function(dood, control_state)
			control_state.primary_fire = false
			control_state.forward = 0
			control_state.leap = false

			local yaw = dood.yaw
			local forward = ba.createVector(-math.sin(yaw), 0, math.cos(yaw))
			local rightward = ba.createVector(-forward.z, 0, forward.x)

			local t_pos = pos

			local dx = t_pos - dood.position
			local dist = dx.length

			local rdot = dot(rightward, dx)
			local fdot = dot(forward, dx)

			control_state.yaw = math.atan2(rdot, fdot)
			control_state.pitch = math.asin(-dx.y / dist ) - dood.pitch

			if fdot > 0 then
				if dist < 15 and dist > 10 and fdot > dist * 0.95 and dx.y < 2 and dx.y > -3 then
					control_state.leap = true
				else
					control_state.forward = 1
					if attack and dist < 50 then control_state.primary_fire = true end
				end

			end
		end
	end

	sb.moveTo = function(pos)
		sb.update = makeMoveFunction(pos)
	end

	sb.moveThrough = function(pos)
		sb.update = makeMoveFunction(pos)
	end

	sb.attackMove = function(pos)
		sb.update = makeMoveFunction(pos, true)
	end

	sb.cancel()
	return sb
end
