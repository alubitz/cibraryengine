if not disable_ai then

	local dood = hv.pawn

	-- sometimes this will get called after the bug is dead... return here to suppress error messages
	if not dood.is_valid then return end

	local props = dood_properties[dood.id]
	if not props then return end

	-- select a target
	local my_pos = dood.position

	local dood_list = gs.getDoodsList()

	local target = nil
	for i, ent in ipairs(dood_list) do
		if ent.is_player and ent ~= dood then
			if gs.checkLineOfSight(my_pos + ba.createVector(0, 0.5, 0), ent.position + ba.createVector(0, 1, 0)) then
				target = ent
			end
		end
	end

	-- reset certain parts of the control state...
	local control_state = hv.control_state
	control_state.primary_fire = false
	control_state.forward = 0
	control_state.leap = false

	-- engage target, if applicable
	if target then
		if not props.goal then
			props.goal = goal_move_attack(dood, target)
			props.goal.activate()
		end
	elseif props.goal then
		props.goal.terminate()
		props.goal = nil
	end

	if props.goal and props.goal.status == GoalStatus.ACTIVE then
		props.goal.process()
	else
		get_steering_behavior(dood).cancel()
	end

	do_steering_behavior(dood, control_state)
end
