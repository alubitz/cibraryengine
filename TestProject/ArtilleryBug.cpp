#include "StdAfx.h"
#include "ArtilleryBug.h"

namespace Test
{
	/*
	 * ArtilleryBug methods
	 */
	ArtilleryBug::ArtilleryBug(GameState* game_state, UberModel* model, Vec3 pos, Team& team) :
		Dood(game_state, model, pos, team)
	{
	}
}
