#pragma once

#include "StdAfx.h"

#include "Dood.h"

namespace Test
{
	class PoseAimingGun;
	class WalkPose;

	class Soldier : public Dood
	{
		protected:

			void DoJumpControls(TimingInfo time, Vec3 forward, Vec3 rightward);

			void PreUpdatePoses(TimingInfo time);
			void PostUpdatePoses(TimingInfo time);

			void DoCheatyPose(float timestep, const Vec3& net_vel);
			void MaybeSinkCheatyVelocity(float timestep, Vec3& cheaty_vel, Vec3& cheaty_rot, float net_mass, const Mat3& net_moi);

		public:

			Bone* gun_hand_bone;

			PoseAimingGun* p_ag;
			WalkPose* walk_pose;

			float jet_fuel;

			SoundBuffer* jet_start_sound;
			SoundBuffer* jet_loop_sound;
			SoundSource* jet_loop;

			Soldier(GameState* game_state, UberModel* model, ModelPhysics* mphys, Vec3 pos, Team& team);

			void RegisterFeet();

			void Update(TimingInfo time);

			void Die(Damage cause);

			void Spawned();
			void DeSpawned();
	};
}
