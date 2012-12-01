#include "StdAfx.h"
#include "ArtilleryWeapon.h"

#include "RocketBug.h"
#include "Dood.h"

#include "TestGame.h"

namespace Test
{
	/*
	 * ArtilleryWeapon methods
	 */
	ArtilleryWeapon::ArtilleryWeapon(TestGame* game_state, Dood* owner, Bone* mount_bone, const Vec3& mount_pos) :
		WeaponIntrinsic(game_state, owner),
		proj_model(NULL),
		proj_mphys(NULL),
		attack_ready_time(-1.0f),
		attack_interval(0.32154631461f),
		mount_bone(mount_bone),
		mount_pos(mount_pos)
	{
		proj_model = game_state->ubermodel_cache->Load("dummycube");
		proj_mphys = game_state->mphys_cache->Load("dummycube");
	}

	void ArtilleryWeapon::OwnerUpdate(TimingInfo time)
	{
		if(IsFiring(1) && time.total >= attack_ready_time)
			LaunchRocketBug(time.total);
	}

	void ArtilleryWeapon::LaunchRocketBug(float now)
	{
		Vec3 direction = Mat3::FromScaledAxis(0, -owner->yaw, 0) * Mat3::FromScaledAxis(owner->pitch, 0, 0) * Vec3(0, 0, 1);

		game_state->Spawn(new RocketBug(
			game_state,
			owner,
			proj_model,
			proj_mphys,
			owner->pos + mount_bone->GetTransformationMatrix().TransformVec3_1(mount_pos),
			owner->vel,// + direction * 50.0f,
			Quaternion::FromRotationMatrix(Util::FindOrientationZEdge(direction))));

		attack_ready_time = now + attack_interval;
	}
}
