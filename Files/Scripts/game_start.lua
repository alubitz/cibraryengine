-- function to be called when the player dies
function player_death(dood)
	local retry_text = "Press Esc to return to the main menu"
	local you_died_pre = "You died on wave "
	local you_died_post = "!"
	gs.showChapterText("GAME OVER!", retry_text .. "\n\n" .. you_died_pre .. level .. you_died_post, -1.0)
end

-- figure out which dood is the player, and remember that

local player_pos
local flat_world = true         -- true for spawning on flat grid world; false for terrain world
if flat_world then
	player_pos = ba.createVector(0, 0, 0)
else
	player_pos = ba.createVector(8.3, 149.2, 69.5)
end

local only_one_dood = false      -- true for having only one player soldier/bug/etc. in the game.
if only_one_dood then
	player = gs.spawnPlayer(player_pos)
else
	for x = -2, 2 do
		for z = -2, 2 do
			player = gs.spawnPlayer(ba.createVector(x * 15, 0, z * 15))
		end
	end
end

player.death_callback = player_death

poll_mouse_motion()

level = 0
kills = 0
kills_this_level = 0

bot_spawn_timer = 0

bots_spawned = 0

disable_enemies = true
disable_waves   = false
disable_ai      = false


-- whether the KEY is pressed! the actual enabled/disabled state is *_mode
god_toggle          = false
nav_edit_toggle     = false
debug_draw_toggle   = false
third_person_toggle = false

game_over = false
player_got_game_start = false

gs.setGodMode(god_mode)
gs.setNavEditMode(nav_edit_mode)
gs.setDebugDrawMode(debug_draw_mode)

--spawnBotAtPosition(gs, 0, 10)
num_rubbish = 0

if num_rubbish > 0 then
	for i = 1, num_rubbish do
		local x = math.random() * 10.0 - 5.0
		local z = math.random() * 10.0 + 30.0
		local y = gs.getTerrainHeight(x, z) + 200.0 * i / num_rubbish + 5.0

		gs.spawnRubbish(ba.createVector(x, y, z))
	end
end

-- function called every time a bug dies
function crab_bug_death(dood)
	if not game_over then
		kills_this_level = kills_this_level + 1
		kills = kills + 1
	end
	dood_properties[dood.id].dood = nil
	dood_properties[dood.id] = nil

	if steering_behaviors[dood.id] ~= nil then
		steering_behaviors[dood.id].owner = nil
		steering_behaviors[dood.id] = nil
	end
end

function levelStartMessage()
	gs.showChapterText("Wave " .. level)
end

-- a specific manner of spawning bots
function spawn_one(gs, player_pos, artillery)
	local min_dist = 20.0
	local max_dist = 120.0

	local min_dist_sq = min_dist * min_dist;
	local max_dist_sq = max_dist * max_dist;

	local ok = false
	local x, z

	while not ok do

		x = math.random() * 196.0 - 98.0
		z = math.random() * 196.0 - 98.0

		local dx = x - player_pos.x
		local dz = z - player_pos.z

		local dsq = dx * dx + dz * dz
		if dsq < max_dist_sq and dsq > min_dist_sq then
			ok = true
		end
	end

	local bot = spawnBotAtPosition(gs, x, z, artillery)
	bots_spawned = bots_spawned + 1

	bot.death_callback = crab_bug_death

	local props = {}
	props.dood = bot
	props.nav = gs.getNearestNav(bot.position)

	dood_properties[bot.id] = props
end

function begin_level(gs, player_pos, level)
	if not disable_enemies then
		local bugs_this_level = 30 --1 + 2 * level + math.floor(math.random() * 3.0)
		local num_artillery   = 0
		for i = 1, bugs_this_level do
			spawn_one(gs, player_pos, i <= num_artillery)
		end

		levelStartMessage()
	end
end
