#pragma once

#include "StdAfx.h"

#include "Dood.h"

namespace Test
{
	class CrabBug : public Dood
	{
		private:

			class WalkPose : public Pose
			{
				public:

					float yaw;
					Vec3 pos;

					WalkPose() : Pose(), yaw(), pos() { }

					void UpdatePose(TimingInfo time) { SetBonePose(Bone::string_table["carapace"], Vec3(0, -yaw, 0), Vec3(), 1.0f); }
			} walk_pose;

		protected:

			void DoJumpControls(TimingInfo time, Vec3 forward, Vec3 rightward);

			void PreUpdatePoses(TimingInfo time);

		public:

			CrabBug(GameState* game_state, UberModel* model, Vec3 pos, Team& team);
	};
}
