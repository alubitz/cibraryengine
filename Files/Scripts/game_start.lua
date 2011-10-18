-- function to be called when the player dies
function player_death(dood)
	local retry_text = "Press Esc to return to the main menu"
	local you_died_pre = "You died on wave "
	local you_died_post = "!"
	gs.showChapterText("GAME OVER!", retry_text .. "\n\n" .. you_died_pre .. level .. you_died_post, -1.0)
end

-- figure out which dood is the player, and remember that
player = gs.spawnPlayer(ba.createVector(8.3, 149.2 + 1, 69.5))
player.ai_callback = player_ai
player.death_callback = player_death

dood_properties = {}

poll_mouse_motion()

level = 0
kills = 0
kills_this_level = 0

bot_spawn_timer = 0

bots_spawned = 0

disable_enemies = false

god_toggle = false
nav_edit_toggle = false
debug_draw_toggle = false

game_over = false

gs.setGodMode(god_mode)
gs.setNavEditMode(nav_edit_mode)
gs.setDebugDrawMode(debug_draw_mode)

dofile("Files/Scripts/goals.lua")

-- the crab bugs' ai
function crab_bug_ai(dood)
	local props = dood_properties[dood]

	-- sometimes this will get called after the bug is dead... return here to suppress error messages
	if not props then return end

	-- select a target
	local my_pos = dood.position

	local dood_list = gs.getDoodsList()

	local target = nil
	for i, ent in ipairs(dood_list) do
		if gs.checkLineOfSight(my_pos + ba.createVector(0, 0.5, 0), ent.position + ba.createVector(0, 1, 0)) then
			if ent.is_player and ent ~= dood then
				target = ent
			end
		end
	end

	-- reset certain parts of the control state...
	local control_state = dood.control_state
	control_state.primary_fire = false
	control_state.forward = 0
	control_state.leap = false

	-- engage target, if applicable
	if target then
		if not props.goal then
			props.goal = goal_move_attack(dood, target)
			props.goal.activate()
		end
	end

	if props.goal and props.goal.status == GoalStatus.ACTIVE then
		props.goal.process()
	else
		get_steering_behavior(dood).cancel()
	end

	do_steering_behavior(dood, control_state)
end

-- function called every time a bug dies
function crab_bug_death(dood)
	if not game_over then
		kills_this_level = kills_this_level + 1
		kills = kills + 1

		dood_properties[dood] = nil
		steering_behaviors[dood] = nil
	end
end