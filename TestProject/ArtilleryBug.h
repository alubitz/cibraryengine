#pragma once
#include "StdAfx.h"

#include "Dood.h"

namespace Test
{
	struct BoneEntry;

	class ArtilleryBug : public Dood
	{
		private:

			class WalkPose : public Pose
			{
				public:

					float yaw;
					Vec3 pos;

					WalkPose() : Pose(), yaw(), pos() { }

					void UpdatePose(const TimingInfo& time) { /*SetBonePose(Bone::string_table["carapace"], Vec3(0, -yaw, 0), Vec3());*/ }
			};
			WalkPose* walk_pose;

		protected:

			void PreUpdatePoses(const TimingInfo& time);

		public:

			ArtilleryBug(GameState* game_state, UberModel* model, ModelPhysics* mphys, const Vec3& pos, Team& team);

			static void GetBoneEntries(vector<BoneEntry>& bone_entries);			// just for convenience in the conversion process

			void RegisterFeet();
	};
}
