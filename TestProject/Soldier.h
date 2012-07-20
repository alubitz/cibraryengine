#pragma once

#include "StdAfx.h"

#include "Dood.h"

namespace Test
{
	struct BoneEntry;

	class PoseAimingGun;
	class PoseStanding;

	class Soldier : public Dood
	{
		protected:

			void DoJumpControls(TimingInfo time, Vec3 forward, Vec3 rightward);
			void DoWeaponControls(TimingInfo time);
			void PreUpdatePoses(TimingInfo time);
			void PostUpdatePoses(TimingInfo time);

		public:

			Bone* gun_hand_bone;

			PoseAimingGun* p_ag;
			PoseStanding* p_s;

			float jump_fuel;

			SoundBuffer* jet_start_sound;
			SoundBuffer* jet_loop_sound;
			SoundSource* jet_loop;

			Soldier(GameState* game_state, UberModel* model, ModelPhysics* mphys, Vec3 pos, Team& team);

			static void GetBoneEntries(vector<BoneEntry>& bone_entries);			// just for convenience in the conversion process
	};
}
