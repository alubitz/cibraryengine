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

			DefaultWeapon(TestGame* game_state, Dood* owner, UberModel* gun_model, VertexBuffer* mflash_model, GlowyModelMaterial* mflash_material, ModelPhysics* mphys, VertexBuffer* shot_model, BillboardMaterial* shot_material, SoundBuffer* fire_sound, SoundBuffer* chamber_click_sound, SoundBuffer* reload_sound);
			Shot* CreateShot(const Vec3& origin, const Vec3& weapon_vel, const Vec3& direction);

			bool GetAmmoCount(int& result);

			Mat4 GetInitialXform();
	};
}