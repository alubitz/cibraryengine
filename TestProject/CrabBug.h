#pragma once

#include "StdAfx.h"

#include "Dood.h"

namespace Test
{
	struct BoneEntry;

	class CrabBug : public Dood
	{
		private:

			class CrabHeading: public Pose
			{
				public:

					float yaw;

					CrabHeading() : Pose(), yaw() { }

					void UpdatePose(TimingInfo time) { SetBonePose(Bone::string_table["carapace"], Vec3(0, -yaw, 0), Vec3()); }
			};
			CrabHeading* crab_heading;

		protected:

			void DoJumpControls(TimingInfo time, Vec3 forward, Vec3 rightward);

			void PreUpdatePoses(TimingInfo time);

		public:

			CrabBug(GameState* game_state, UberModel* model, ModelPhysics* mphys, Vec3 pos, Team& team);

			void Update(TimingInfo time);

			static void GetBoneEntries(vector<BoneEntry>& bone_entries);			// just for convenience in the conversion process
	};
}
