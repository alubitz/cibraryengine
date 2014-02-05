#include "StdAfx.h"
#include "ArtilleryBug.h"

namespace Test
{
	/*
	 * ArtilleryBug methods
	 */
	ArtilleryBug::ArtilleryBug(GameState* game_state, UberModel* model, ModelPhysics* mphys, const Vec3& pos, Team& team) :
		Dood(game_state, model, mphys, pos, team),
		walk_pose(new WalkPose())
	{
		yaw_rate   *= 0.1f;
		pitch_rate *= 0.1f;

		hp *= 3.0f;

		posey->active_poses.push_back(walk_pose);
		character->mat_tex_precision = 1024.0f;

		vis_bs_radius = 25.0f;
		ragdoll_timer = 50.0f;
	}

	void ArtilleryBug::PreUpdatePoses(const TimingInfo& time)
	{
		walk_pose->pos = pos;
		walk_pose->yaw = yaw;
	}

	void ArtilleryBug::RegisterFeet()
	{
		feet.push_back(new FootState(Bone::string_table["l leg a 3"]));
		feet.push_back(new FootState(Bone::string_table["r leg a 3"]));
		feet.push_back(new FootState(Bone::string_table["l leg b 3"]));
		feet.push_back(new FootState(Bone::string_table["r leg b 3"]));
		feet.push_back(new FootState(Bone::string_table["l leg c 3"]));
		feet.push_back(new FootState(Bone::string_table["r leg c 3"]));
	}
}
