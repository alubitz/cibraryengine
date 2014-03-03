#pragma once
#include "StdAfx.h"

#include "Dood.h"

namespace Test
{
	class ArtilleryBug : public Dood
	{
		private:

			class WalkPose : public Pose
			{
				public:

					float yaw;
					Vec3 pos;

					WalkPose() : Pose(), yaw(), pos() { }

					void UpdatePose(const TimingInfo& time) { SetBonePose(Bone::string_table["carapace"], Vec3(0, -yaw, 0), Vec3()); }
			};
			WalkPose* walk_pose;

		protected:

			void PreUpdatePoses(const TimingInfo& time);

			void MaybeSinkCheatyVelocity(float timestep, Vec3& cheaty_vel, Vec3& cheaty_rot, float net_mass, const Mat3& net_moi);

		public:

			ArtilleryBug(GameState* game_state, UberModel* model, ModelPhysics* mphys, const Vec3& pos, Team& team);

			void RegisterFeet();
	};
}
