#pragma once

#include "StdAfx.h"

#include "WeaponIntrinsic.h"

namespace Test
{
	class ArtilleryWeapon : public WeaponIntrinsic
	{
		public:

			UberModel* proj_model;
			ModelPhysics* proj_mphys;

			float attack_ready_time;
			float attack_interval;

			Bone* mount_bone;
			Vec3 mount_pos;

			ArtilleryWeapon(TestGame* game_state, Dood* owner, Bone* mount_bone, const Vec3& mount_pos);

			void OwnerUpdate(TimingInfo time);

			void LaunchRocketBug(float now);
	};
}
