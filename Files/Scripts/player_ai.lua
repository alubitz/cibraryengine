local function force_bool(v)                      if v then return v else return false end end
local function force_float(v)                     if v then return v else return 0     end end
local function bool_to_float(b) b = force_bool(b) if b then return 1 else return 0     end end

local function get_key_float(i) local k = key_states[string.byte(i, 1)] if k then return 1    else return 0     end end
local function get_key_bool(i)  local k = key_states[string.byte(i)]    if k then return true else return false end end

local control_state = hv.control_state

control_state.forward      = get_key_float("W") - get_key_float("S")
control_state.elevate      = get_key_float(" ") - get_key_float("C")			-- only used by ghost camera
control_state.sidestep     = get_key_float("D") - get_key_float("A")

control_state.jump         = get_key_bool(" ")
control_state.reload       = get_key_bool("R")control_state.primary_fire = force_bool(mb_states[0])
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
		if nav_edit_mode then gs.showChapterText("Nav graph on") else gs.showChapterText("Nav graph off") end
	end
else nav_edit_toggle = false end

if force_bool(key_states[114]) then
	if not debug_draw_toggle then
		debug_draw_toggle = true
		debug_draw_mode = not debug_draw_mode
		gs.setDebugDrawMode(debug_draw_mode)
	end
else debug_draw_toggle = false end

if force_bool(key_states[116]) then		-- 13 for enter, 116 for f5
	if not third_person_toggle then
		third_person_toggle = true
		third_person_mode = not third_person_mode
		control_state.third_person = third_person_mode
	end
else third_person_toggle = false end

if not player_got_game_start then
	control_state.third_person = third_person_mode
	player_got_game_start = true
end


control_state.yaw   = force_float(control_state.yaw)   + x * 0.005
control_state.pitch = force_float(control_state.pitch) + y * 0.005
