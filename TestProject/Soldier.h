#pragma once

#include "StdAfx.h"

#include "Dood.h"

namespace Test
{
	class Soldier : public Dood
	{
		protected:

			void InnerDispose();

			void DoJumpControls(TimingInfo time, Vec3 forward, Vec3 rightward);
			void DoWeaponControls(TimingInfo time);
			void PreUpdatePoses(TimingInfo time);
			void PostUpdatePoses(TimingInfo time);

		public:

			Bone* gun_hand_bone;

			PoseAimingGun* p_ag;

			float jump_fuel;

			SoundBuffer* jet_start_sound;
			SoundBuffer* jet_loop_sound;
			SoundSource* jet_loop;

			Soldier(GameState* game_state, UberModel* model, ModelPhysics* mphys, Vec3 pos, Team& team);

			void Update(TimingInfo time);
	};
}
