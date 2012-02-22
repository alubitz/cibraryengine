#pragma once

#include "WeaponEquip.h"

namespace Test
{
	class Gun : public WeaponEquip
	{
		public:

			float reload_time;
			float refire_interval;
			float chamber_click_delay;
			int clip_size;
			int clip;
			bool reloading;

			float fire_wait, reload_wait;
			float mflash_size;

			UberModel* gun_model;
			VertexBuffer* mflash_model;
			vector<Material*> gun_materials;
			GlowyModelMaterial* mflash_material;

			SoundBuffer* fire_sound;
			SoundBuffer* chamber_click_sound;
			SoundBuffer* reload_sound;

			Gun(GameState* game_state, Dood* owner, UberModel* gun_model, VertexBuffer* mflash_model, GlowyModelMaterial* mflash_material, SoundBuffer* fire_sound, SoundBuffer* chamber_click_sound, SoundBuffer* reload_sound);

			virtual void OwnerUpdate(TimingInfo time);

			virtual void Vis(SceneRenderer* renderer);

			virtual Shot* CreateShot(Vec3 origin, Vec3 weapon_vel, Vec3 direction) = 0;
			virtual void Fire(float total_inaccuracy, float now);
			virtual bool NeedsReloading();
			virtual void BeginReload();
			virtual void FinishReload();
			virtual bool GetAmmoFraction(float& result);
	};
}
