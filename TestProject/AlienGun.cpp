#include "AlienGun.h"

#include "AlienShot.h"

#include "TestGame.h"
#include "DSNMaterial.h"

namespace Test
{
	using namespace CibraryEngine;

	AlienGun::AlienGun(TestGame* game_state, Dood* owner, UberModel* gun_model, VTNModel* mflash_model, VTNModel* shot_model, GlowyModelMaterial* mflash_material, GlowyModelMaterial* shot_material, SoundBuffer* fire_sound, SoundBuffer* chamber_click_sound, SoundBuffer* reload_sound) :
		Gun(game_state, owner, gun_model, mflash_model, mflash_material, fire_sound, chamber_click_sound, reload_sound),
		shot_model(shot_model),
		shot_material(shot_material)
	{
		clip = clip_size = 1000;
		refire_interval = 0.8;
	}

	Shot* AlienGun::CreateShot(Vec3 origin, Vec3 weapon_vel, Vec3 direction)
	{
		Vec3 vel = /*weapon_vel + */Vec3::Normalize(direction, 200);
		Quaternion ori = Quaternion::FromRotationMatrix(Util::FindOrientationZEdge(direction));
		return new AlienShot(game_state, shot_model, shot_material, origin, vel, ori, owner);
	}
};
