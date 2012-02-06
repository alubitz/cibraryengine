#pragma once
#include "StdAfx.h"

#include "Dood.h"

namespace Test
{
	struct BoneEntry;

	class ArtilleryBug : public Dood
	{
		public:

			ArtilleryBug(GameState* game_state, UberModel* model, Vec3 pos, Team& team);

			static void GetBoneEntries(vector<BoneEntry>& bone_entries);			// just for convenience in the conversion process
	};
}
