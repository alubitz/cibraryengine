#include "StdAfx.h"
#include "DefaultWeapon.h"

#include "Shot.h"

#include "TestGame.h"

#include "RocketBug.h"

namespace Test
{
	using namespace CibraryEngine;

	DefaultWeapon::DefaultWeapon(TestGame* game_state, Dood* owner, UberModel* gun_model, VertexBuffer* mflash_model, VertexBuffer* shot_model, GlowyModelMaterial* mflash_material, BillboardMaterial* shot_material, SoundBuffer* fire_sound, SoundBuffer* chamber_click_sound, SoundBuffer* reload_sound) :
		Gun(game_state, owner, gun_model, mflash_model, mflash_material, fire_sound, chamber_click_sound, reload_sound),
		shot_model(shot_model),
		shot_material(shot_material)
	{
		clip = clip_size = 150;
	}

	Shot* DefaultWeapon::CreateShot(Vec3 origin, Vec3 weapon_vel, Vec3 direction)
	{
#if 0
		game_state->Spawn(new RocketBug(
			game_state,
			owner,
			((TestGame*)game_state)->ubermodel_cache->Load("dummycube"),
			((TestGame*)game_state)->mphys_cache->Load("dummycube"),
			origin,
			weapon_vel + direction * 50.0f,
			Quaternion::FromRotationMatrix(Util::FindOrientationZEdge(direction))));
#endif

		Vec3 vel = Vec3::Normalize(direction, 300) + weapon_vel;
		return new Shot(game_state, shot_model, shot_material, origin, vel, owner);
	}

	bool DefaultWeapon::GetAmmoCount(int& result) { result = clip; return true; }
};
