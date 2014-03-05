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
		feet.push_back(new FootState(Bone::string_table["l leg a 3"], Vec3(  6.68f,  0.0f,  12.27f)));
		feet.push_back(new FootState(Bone::string_table["r leg a 3"], Vec3( -6.68f,  0.0f,  12.27f)));
		feet.push_back(new FootState(Bone::string_table["l leg b 3"], Vec3( 16.38f,  0.0f,   8.56f)));
		feet.push_back(new FootState(Bone::string_table["r leg b 3"], Vec3(-16.38f,  0.0f,   8.56f)));
		feet.push_back(new FootState(Bone::string_table["l leg c 3"], Vec3(  7.05f,  0.0f, -15.31f)));
		feet.push_back(new FootState(Bone::string_table["r leg c 3"], Vec3( -7.05f,  0.0f, -15.31f)));
	}

	void ArtilleryBug::MaybeSinkCheatyVelocity(float timestep, Vec3& cheaty_vel, Vec3& cheaty_rot, float net_mass, const Mat3& net_moi)
	{
		cheaty_rot = Vec3();
		Dood::MaybeSinkCheatyVelocity(timestep, cheaty_vel, cheaty_rot, net_mass, net_moi);
	}
}
