#pragma once

#include "WeaponEquip.h"

namespace Test
{
	class Gun : public WeaponEquip
	{
		protected:

			virtual void InnerDispose();

			virtual void OwnerUpdate(const TimingInfo& time);			// called by the owner's Update
			virtual void UnownedUpdate(const TimingInfo& time);			// called by our Update if there is no owner
			virtual void SharedUpdate(const TimingInfo& time);			// called by the default implementations of OwnerUpdate and UnownedUpdate

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

			ModelPhysics* model_phys;

			RigidBody* rigid_body;
			FixedJointConstraint *l_grip, *r_grip;
			PhysicsWorld* physics;

			Vec3 barrel_pos;

			SoundBuffer* fire_sound;
			SoundBuffer* chamber_click_sound;
			SoundBuffer* reload_sound;

			Gun(GameState* game_state, Dood* owner, UberModel* gun_model, VertexBuffer* mflash_model, GlowyModelMaterial* mflash_material, ModelPhysics* mphys, SoundBuffer* fire_sound, SoundBuffer* chamber_click_sound, SoundBuffer* reload_sound);

			void Update(const TimingInfo& time);

			virtual void Vis(SceneRenderer* renderer);

			virtual Shot* CreateShot(const Vec3& origin, const Vec3& weapon_vel, const Vec3& direction) = 0;
			virtual void Fire(float total_inaccuracy, float now);
			virtual bool NeedsReloading();
			virtual void BeginReload();
			virtual void FinishReload();
			virtual bool GetAmmoFraction(float& result);

			virtual Mat4 GetInitialXform();

			virtual void Spawned();
			virtual void DeSpawned();

			virtual void Equip(Dood* new_owner);
			virtual void UnEquip(Dood* old_owner);
	};
}
