#pragma once

#include "StdAfx.h"

#include "Dood.h"

#include "../CibraryEngine/IKChain.h"
#include "../CibraryEngine/IKAnimation.h"
#include "../CibraryEngine/IKWalkPose.h"
#include "../CibraryEngine/StepPose.h"

namespace Test
{
	struct BoneEntry;

	class PoseAimingGun;

	class Soldier : public Dood
	{
		protected:

			void DoJumpControls(TimingInfo time, Vec3 forward, Vec3 rightward);
			void DoMovementControls(TimingInfo time, Vec3 forward, Vec3 rightward);
			void DoWeaponControls(TimingInfo time);
			void PreUpdatePoses(TimingInfo time);
			void PostUpdatePoses(TimingInfo time);

			void InnerDispose();

		public:

			Bone* gun_hand_bone;

			PoseAimingGun* p_ag;
			IKWalkPose* ik_pose;

			float jump_fuel;

			SoundBuffer* jet_start_sound;
			SoundBuffer* jet_loop_sound;
			SoundSource* jet_loop;

			Soldier(GameState* game_state, UberModel* model, ModelPhysics* mphys, Vec3 pos, Team& team);

			void Spawned();

			void Die(Damage cause);



			static void GetBoneEntries(vector<BoneEntry>& bone_entries);			// just for convenience in the conversion process
	};
}
