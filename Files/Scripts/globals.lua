start_time = os.time()
ba.println("script system started at t = " .. start_time)
math.randomseed(start_time)

dood_properties = {}

-- function to spawn a bot 1 meter above the terrain at the specified x and z coordinates
function spawnBotAtPosition(gs, x, z)
	return gs.spawnBot(ba.createVector(x, gs.getTerrainHeight(x, z) + 1, z))
end

-- function that returns the player
function getPlayer(gs)
	local doods = gs.getDoodsList()
	local i
	for i = 1, #doods do
		local dood = doods[i]
		if dood.isPlayer() then
			return dood
		end
	end
	return nil
end

key_states = {}
ba.keyStateCallback = function(key, state)
	key_states[key] = state
end

mb_states = {}
ba.mouseButtonStateCallback = function(button, state)
	mb_states[button] = state
end

mouse_state = {}
mouse_state.dx = 0
mouse_state.dy = 0
ba.mouseMovementCallback = function(x, y, dx, dy)
	mouse_state.x = x
	mouse_state.y = y
	mouse_state.dx = mouse_state.dx + dx
	mouse_state.dy = mouse_state.dy + dy
end

function poll_mouse_motion()
	local x = mouse_state.dx
	local y = mouse_state.dy
	mouse_state.dx = 0
	mouse_state.dy = 0
	return x, y
end



function force_bool(v) if v then return v else return false end end
function force_float(v) if v then return v else return 0 end end
function bool_to_float(b) b = force_bool(b) if b then return 1 else return 0 end end

god_toggle = false
nav_edit_toggle = false
debug_draw_toggle = false
god_mode = false
nav_edit_mode = false
debug_draw_mode = false

function player_ai(dood)
	local function get_key_float(i) local k = key_states[string.byte(i, 1)] if k then return 1 else return 0 end end
	local function get_key_bool(i) local k = key_states[string.byte(i)] if k then return true else return false end end

	local control_state = dood.getControlState()

	control_state.forward = get_key_float("W") - get_key_float("S")
	control_state.sidestep = get_key_float("D") - get_key_float("A")

	control_state.jump = get_key_bool(" ")
	control_state.primary_fire = force_bool(mb_states[0])
	control_state.reload = get_key_bool("R")
	local x, y = poll_mouse_motion()

	if force_bool(key_states[112]) then
		if not god_toggle then
			god_toggle = true
			god_mode = not god_mode
			gs.setGodMode(god_mode)
			if god_mode then gs.showChapterText("God mode on") else gs.showChapterText("God mode off") end
		end
	else god_toggle = false end

	if force_bool(key_states[113]) then
		if not nav_edit_toggle then
			nav_edit_toggle = true
			nav_edit_mode = not nav_edit_mode
			gs.setNavEditMode(nav_edit_mode)
			if nav_edit_mode then gs.showChapterText("Nav editor on") else gs.showChapterText("Nav editor off") end
		end
	else nav_edit_toggle = false end

	if force_bool(key_states[114]) then
		if not debug_draw_toggle then
			debug_draw_toggle = true
			debug_draw_mode = not debug_draw_mode
			gs.setDebugDrawMode(debug_draw_mode)
		end
	else debug_draw_toggle = false end

	control_state.yaw = force_float(control_state.yaw) + x * 0.005
	control_state.pitch = force_float(control_state.pitch) + y * 0.005
end

function dot(a, b)
	return a.x * b.x + a.y * b.y + a.z * b.z
end
