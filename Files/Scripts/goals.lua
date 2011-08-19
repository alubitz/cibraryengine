
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
				if status_of_goals == GoalStatus.COMPLETED and #goal.subgoals > 1 then
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

		if (edge.pos - goal.owner.getPosition()).length < tol then
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


----------------------------------------------------------------------------------------------------
-------- Steering behaviors; handles low-level control in order to complete certain actions --------
----------------------------------------------------------------------------------------------------

steering_behaviors = {}

function do_steering_behavior(dood, control_state)
	get_steering_behavior(dood).update(dood, control_state)
end

function get_steering_behavior(dood)
	if not steering_behaviors[dood] then
		steering_behaviors[dood] = create_steering_behavior(dood)
	end
	return steering_behaviors[dood]
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

	local function makeMoveFunction(pos)
		return function(dood, control_state)
			control_state.primary_fire = false
			control_state.forward = 0
			control_state.leap = false

			local yaw = dood.getYaw()
			local forward = ba.createVector(-math.sin(yaw), 0, math.cos(yaw))
			local rightward = ba.createVector(-forward.z, 0, forward.x)

			local t_pos = pos

			local dx = t_pos - dood.getPosition()
			local dist = dx.length

			local rdot = dot(rightward, dx)
			local fdot = dot(forward, dx)

			control_state.yaw = math.atan2(rdot, fdot)
			control_state.pitch = math.asin(-dx.y / dist ) - dood.getPitch()

			if fdot > 0 then
				if dist < 15 and dist > 10 and fdot > dist * 0.95 and dx.y < 2 and dx.y > -3 then
					control_state.leap = true
				else
					control_state.forward = 1
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

	sb.cancel()
	return sb
end
