#pragma once
#include "StdAfx.h"

#include "Dood.h"

namespace Test
{
	class ArtilleryBug : public Dood
	{
		public:

			ArtilleryBug(GameState* game_state, UberModel* model, Vec3 pos, Team& team);
	};
}
