start_time = os.time()
ba.println("script system started at t = " .. start_time)
math.randomseed(start_time)

--dofile("Files/Scripts/foothelper_zzp.lua")

dood_properties = {}

-- function to spawn a bot some number of meters above the terrain at the specified x and z coordinates
function spawnBotAtPosition(gs, x, z, artillery)
	local alt_above = 0		--2
	if artillery then
		return gs.spawnArtilleryBug(ba.createVector(x, gs.getTerrainHeight(x, z) + alt_above, z))
	else
		return gs.spawnBot(ba.createVector(x, gs.getTerrainHeight(x, z) + alt_above, z))
	end
end

-- function that returns the player
function getPlayer(gs)
	local doods = gs.getDoodsList()
	local i
	for i = 1, #doods do
		local dood = doods[i]
		if dood.is_player then
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


god_toggle          = false
nav_edit_toggle     = false
debug_draw_toggle   = falsethird_person_toggle = false
god_mode          = false
nav_edit_mode     = false
debug_draw_mode   = false
third_person_mode = false

function dot(a, b)
	return a.x * b.x + a.y * b.y + a.z * b.z
end
