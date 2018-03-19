#pragma once

#include "StdAfx.h"

#include "Dood.h"

namespace Test
{
	class GAExperiment;

	class CrabBug : public Dood
	{
		private:

			struct Imp;
			Imp* imp;

			friend struct Imp;

			class CrabHeading: public Pose
			{
				public:

					float yaw;

					CrabHeading() : Pose(), yaw() { }

					void UpdatePose(const TimingInfo& time) { SetBonePose(Bone::string_table["carapace"], Vec3(0, -yaw, 0), Vec3()); }
			};
			CrabHeading* crab_heading;

		protected:

			void InnerDispose();

			void DoJumpControls(const TimingInfo& time, const Vec3& forward, const Vec3& rightward);

			void PreUpdatePoses(const TimingInfo& time);

			void MaybeSinkCheatyVelocity(float timestep, Vec3& cheaty_vel, Vec3& cheaty_rot, float net_mass, const Mat3& net_moi);

			virtual void InitBoneHelpers();
			virtual void InitJointHelpers();

		public:

			CrabBug(GameState* game_state, GAExperiment* experiment, UberModel* model, ModelPhysics* mphys, const Vec3& pos, Team& team);

			void Update(const TimingInfo& time);

			void RegisterFeet();
	};
}
