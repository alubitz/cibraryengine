
-- don't try to update player position if the player is dead
if player.is_valid then
	player_pos = player.position
elseif not player_pos then
	player_pos = ba.createVector()
end

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
