function levelStartMessage()
	gs.showChapterText("Wave " .. level)
end

-- a specific manner of spawning bots
function spawn_one(gs, player_pos)
	local theta, radius = math.random() * math.pi * 2.0, math.random() * 70 + 50
	local x = radius * math.cos(theta) + player_pos.x
	local z = radius * math.sin(theta) + player_pos.z

	if x > 98 then x = 98 end
	if x < -98 then x = -98 end
	if z > 98 then z = 98 end
	if z < -98 then z = -98 end

	local bot = spawnBotAtPosition(gs, x, z)
	bots_spawned = bots_spawned + 1

	bot.ai_callback = crab_bug_ai
	bot.death_callback = crab_bug_death

	local props = {}
	props.dood = bot
	props.nav = gs.getNearestNav(bot.position)

	dood_properties[bot] = props
end

function begin_level(gs, player_pos, level)
	local bugs_this_level = 1 + 2 * level + math.floor(math.random() * 3.0)
	for i = 1, bugs_this_level do
		spawn_one(gs, player_pos)
	end
	levelStartMessage()
end

player_pos = player.position
if level == 0 then
	level = 1
	begin_level(gs, player_pos, level)
end

num_bots = gs.getNumberOfBugs()

bot_spawn_timer = bot_spawn_timer + gs.getElapsedTime()
if bot_spawn_timer > 2 then
	bot_spawn_timer = 0
end

if num_bots == 0 then
	level = level + 1
	kills_this_level = 0

	begin_level(gs, player_pos, level)
end