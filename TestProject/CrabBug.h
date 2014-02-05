#pragma once

#include "StdAfx.h"

#include "Dood.h"

namespace Test
{
	class CrabBug : public Dood
	{
		private:

			class CrabHeading: public Pose
			{
				public:

					float yaw;

					CrabHeading() : Pose(), yaw() { }

					void UpdatePose(const TimingInfo& time) { /*SetBonePose(Bone::string_table["carapace"], Vec3(0, -yaw, 0), Vec3());*/ }
			};
			CrabHeading* crab_heading;

		protected:

			void DoJumpControls(const TimingInfo& time, const Vec3& forward, const Vec3& rightward);

			void PreUpdatePoses(const TimingInfo& time);

		public:

			CrabBug(GameState* game_state, UberModel* model, ModelPhysics* mphys, const Vec3& pos, Team& team);

			void Update(const TimingInfo& time);

			void RegisterFeet();
	};
}
