#pragma once

#include "StdAfx.h"

#include "Gun.h"

namespace Test
{
	using namespace CibraryEngine;

	class DefaultWeapon : public Gun
	{
		public:

			VertexBuffer* shot_model;
			BillboardMaterial* shot_material;

			DefaultWeapon(TestGame* game_state, Dood* owner, UberModel* gun_model, VertexBuffer* mflash_model, VertexBuffer* shot_model, GlowyModelMaterial* mflash_material, BillboardMaterial* shot_material, SoundBuffer* fire_sound, SoundBuffer* chamber_click_sound, SoundBuffer* reload_sound);
			Shot* CreateShot(Vec3 origin, Vec3 weapon_vel, Vec3 direction);

			bool GetAmmoCount(int& result);
	};
}