#include "StdAfx.h"
#include "Gun.h"

#include "Dood.h"
#include "Shot.h"
#include "TestGame.h"

namespace Test
{
	/*
	 * Gun methods
	 */
	Gun::Gun(GameState* game_state, Dood* owner, UberModel* gun_model, VertexBuffer* mflash_model, GlowyModelMaterial* mflash_material, SoundBuffer* fire_sound, SoundBuffer* chamber_click_sound, SoundBuffer* reload_sound) :
		WeaponEquip(game_state, owner),
		reload_time(1.7f),
		refire_interval(0.089f),
		chamber_click_delay(0.5f),
		clip_size(80),
		clip(),
		reloading(false),
		fire_wait(),
		reload_wait(),
		mflash_size(),
		gun_model(gun_model),
		mflash_model(mflash_model),
		gun_materials(),
		mflash_material(mflash_material),
		model_phys(NULL),
		rigid_body(NULL),
		physics(NULL),
		barrel_pos(0, 0.08f, 0.84f),				// TODO: load this from a file... either the ubermodel or the mphys
		fire_sound(fire_sound),
		chamber_click_sound(chamber_click_sound),
		reload_sound(reload_sound)
	{
		FinishReload();

		if(gun_model != NULL)
		{
			Cache<Material>* mat_cache = game_state->content->GetCache<Material>();
			for(vector<string>::iterator iter = gun_model->materials.begin(); iter != gun_model->materials.end(); ++iter)
				gun_materials.push_back(mat_cache->Load(*iter));
		}
	}

	void Gun::InnerDispose()
	{
		if(rigid_body) { rigid_body->Dispose(); delete rigid_body; rigid_body = NULL; }

		// TODO: dispose and delete physics constraints if they are still around

		WeaponEquip::InnerDispose();
	}

	void Gun::Update(TimingInfo time)
	{
		if(owner == NULL)
			UnownedUpdate(time);

		WeaponEquip::Update(time);
	}

	void Gun::OwnerUpdate(TimingInfo time)
	{
		float timestep = time.elapsed;

		fire_wait -= timestep;

		float total_inaccuracy = 0;
		for(list<Inaccuracy>::iterator iter = inaccuracy.begin(); iter != inaccuracy.end(); )
		{
			if(iter->time >= time.total)
			{
				total_inaccuracy += iter->amount;
				++iter;
			}
			else
				iter = inaccuracy.erase(iter);
		}

		SharedUpdate(time);

		if(reloading)
		{
			reload_wait -= timestep;
			if(reload_wait <= 0)
				FinishReload();
		}

		if(IsFiring(1))
			if(fire_wait <= 0 && !reloading)
			{
				if(clip > 0)
					Fire(total_inaccuracy, time.total);
				else
				{
					PlayWeaponSound(chamber_click_sound, 1.0f, false);

					fire_wait = chamber_click_delay;
					Dood::AmmoFailureEvent evt(owner, this);
					owner->OnAmmoFailure(&evt);
				}
			}
	}

	void Gun::UnownedUpdate(TimingInfo time)
	{
		SharedUpdate(time);

		for(list<Inaccuracy>::iterator iter = inaccuracy.begin(); iter != inaccuracy.end(); )
		{
			if(time.total > iter->time)
				iter = inaccuracy.erase(iter);
			else
				++iter;
		}
	}

	void Gun::SharedUpdate(TimingInfo time)
	{
		mflash_size *= expf(-16.0f * time.elapsed);
		mflash_size -= 0.05f * time.elapsed;
	}

	void Gun::Fire(float total_inaccuracy, float now)
	{
		owner->PoseCharacter();						// to make sure we have a gun_xform... it will conveniently change our gun_xform for us

		Mat4 shot_mat = gun_xform * Mat4::Translation(barrel_pos);

		Vec3 origin = shot_mat.TransformVec3_1(0, 0, 0);
		
		//Vec3 direction = shot_mat.TransformVec3_0(0, 0, 1);
		Vec3 direction = Mat3::FromScaledAxis(0, -owner->yaw, 0) * Mat3::FromScaledAxis(owner->pitch, 0, 0) * Vec3(0, 0, 1);

		if(Shot* shot = CreateShot(origin, vel, (direction + Random3D::RandomNormalizedVector(total_inaccuracy + 0.001f))))
			game_state->Spawn(shot);

		inaccuracy.push_back(Inaccuracy(0.015f, now + 0.1f));

		PlayWeaponSound(fire_sound, 1.0f, false);

		mflash_size = 1.0f;

		fire_wait = refire_interval;
		clip--;

		FireOnce(1);
	}

	bool Gun::NeedsReloading() { return clip == 0 && !reloading; }

	void Gun::BeginReload()
	{
		if(clip < clip_size && reload_wait <= 0)
		{
			reload_wait = reload_time;
			reloading = true;

			PlayWeaponSound(reload_sound, 1.0f, false);
		}
	}

	void Gun::FinishReload()
	{
		clip = clip_size;
		fire_wait = refire_interval;
		reloading = false;
	}

	bool Gun::GetAmmoFraction(float& result)
	{
		if(clip_size == 0)
			return false;

		result = (float)clip / clip_size;
		return true;
	}

	void Gun::Vis(SceneRenderer* renderer)
	{
		Sphere bs = Sphere(pos, 3);
		if(renderer->camera->CheckSphereVisibility(bs))
		{
			if(gun_model != NULL)
				gun_model->Vis(renderer, 0, gun_xform, NULL, &gun_materials);

			if(mflash_model != NULL && mflash_size > 0)
			{
				Mat4 mflash_xform = gun_xform * Mat4::Translation(barrel_pos) * Mat4::FromQuaternion(Quaternion::FromPYR(0, 0, -float(M_PI) * 0.5f)) * Mat4::UniformScale(mflash_size);
				renderer->objects.push_back(RenderNode(mflash_material, new GlowyModelMaterialNodeData(mflash_model, mflash_xform), Vec3::Dot(renderer->camera->GetForward(), bs.center)));
			}
		}
	}

	void Gun::Spawned()
	{
		WeaponEquip::Spawned();

		physics = game_state->physics_world;
		
		if(model_phys != NULL && model_phys->bones.size() > 0)
		{
			Mat4 xform;				// TODO: initialize this from somewhere

			Vec3 pos = xform.TransformVec3_1(0, 0, 0);
			Vec3 a = xform.TransformVec3_0(1, 0, 0);
			Vec3 b = xform.TransformVec3_0(0, 1, 0);
			Vec3 c = xform.TransformVec3_0(0, 0, 1);

			ModelPhysics::BonePhysics& bone = model_phys->bones[0];

			rigid_body = new RigidBody(this, bone.collision_shape, bone.mass_info, pos, Quaternion::FromRotationMatrix(Mat3(a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z)));
			physics->AddCollisionObject(rigid_body);
		}
	}

	void Gun::DeSpawned()
	{
		WeaponEquip::DeSpawned();

		if(rigid_body != NULL)
			physics->RemoveCollisionObject(rigid_body);

		// TODO: break physics constraints if they are still around
	}

	void Gun::Equip(Dood* new_owner)
	{
		WeaponEquip::Equip(new_owner);

		// TODO: create physics constraints, etc.
	}

	void Gun::UnEquip(Dood* old_owner)
	{
		// TODO: break physics constraints, etc.

		WeaponEquip::UnEquip(old_owner);
	}
}
