-- function to be called when the player dies
function player_death(dood)
	local retry_text = "Press Esc to return to the main menu"
	local you_died_pre = "You died on wave "
	local you_died_post = "!"
	gs.showChapterText("GAME OVER!", retry_text .. "\n\n" .. you_died_pre .. level .. you_died_post, -1.0)
end

-- figure out which dood is the player, and remember that
--player = gs.spawnPlayer(ba.createVector(0, 1, 0))
player = gs.spawnPlayer(ba.createVector(8.3, 149.2 + 1, 69.5))
player.death_callback = player_death

dood_properties = {}

poll_mouse_motion()

level = 0
kills = 0
kills_this_level = 0

bot_spawn_timer = 0

bots_spawned = 0

disable_enemies = true
disable_waves = false
disable_ai = false

num_boxes = 2100

-- whether the KEY is pressed! the actual enabled/disabled state is *_mode
god_toggle = false
nav_edit_toggle = false
debug_draw_toggle = false

game_over = false

gs.setGodMode(god_mode)
gs.setNavEditMode(nav_edit_mode)
gs.setDebugDrawMode(debug_draw_mode)

if num_boxes > 0 then
	for i = 0, num_boxes do
		local x = math.random() * 160.0 - 80.0
		local z = math.random() * 160.0 - 80.0
		local y = gs.getTerrainHeight(x, z) + 20.0

		gs.spawnRubbish(ba.createVector(x, y, z))
	end
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

function levelStartMessage()
	gs.showChapterText("Wave " .. level)
end

-- a specific manner of spawning bots
function spawn_one(gs, player_pos, artillery)
	local theta, radius = math.random() * math.pi * 2.0, math.random() * 70 + 50
	local x = radius * math.cos(theta) + player_pos.x
	local z = radius * math.sin(theta) + player_pos.z

	if x > 98 then x = 98 end
	if x < -98 then x = -98 end
	if z > 98 then z = 98 end
	if z < -98 then z = -98 end

	local bot = spawnBotAtPosition(gs, x, z, artillery)
	bots_spawned = bots_spawned + 1

	bot.death_callback = crab_bug_death

	local props = {}
	props.dood = bot
	props.nav = gs.getNearestNav(bot.position)

	dood_properties[bot] = props
end

function begin_level(gs, player_pos, level)
	if not disable_enemies then
		local bugs_this_level = 20 --1 + 2 * level + math.floor(math.random() * 3.0)
		local num_artillery = 0 --6
		for i = 1, bugs_this_level do
			spawn_one(gs, player_pos, i <= num_artillery)
		end

		levelStartMessage()
	end
end