#pragma once

#include "StdAfx.h"

#include "Dood.h"

namespace Test
{
	class PoseAimingGun;
	class WalkPose;

	class Soldier : public Dood
	{
		private:

			struct Imp;
			Imp* imp;

			friend struct Imp;

		protected:

			void InnerDispose();

			void DoJumpControls(const TimingInfo& time, const Vec3& forward, const Vec3& rightward);

			void PreUpdatePoses(const TimingInfo& time);

			void DoInitialPose();

		public:

			Bone* gun_hand_bone;

			PoseAimingGun* p_ag;
			WalkPose* walk_pose;

			float jet_fuel;

			vector<RigidBody*> jet_bones;			// bones to which the jetpack applies force

			SoundBuffer* jet_start_sound;
			SoundBuffer* jet_loop_sound;
			SoundSource* jet_loop;

			Soldier(GameState* game_state, UberModel* model, ModelPhysics* mphys, const Vec3& pos, Team& team);

			void PhysicsToCharacter();
			void RegisterFeet();
			void Update(const TimingInfo& time);

			void Vis(SceneRenderer* renderer);

			void Die(const Damage& cause);

			void Spawned();

			void DeSpawned();

			void PreparePAG(const TimingInfo& time, const Quaternion& t2ori);

			virtual void PreCPHFT(float timestep);
			virtual void PostCPHFT(float timestep);



			bool IsExperimentDone() const;			// for if we're trying to do some rapid-update experimentation

			static void SaveExperimentData();
			static void LoadExperimentData();
	};
}
