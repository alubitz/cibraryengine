#include "StdAfx.h"
#include "Gun.h"

#include "Dood.h"
#include "Shot.h"
#include "TestGame.h"
#include "DSNMaterial.h"

namespace Test
{
	/*
	 * Gun methods
	 */
	Gun::Gun(GameState* game_state, Dood* owner, UberModel* gun_model, VTNModel* mflash_model, GlowyModelMaterial* mflash_material, SoundBuffer* fire_sound, SoundBuffer* chamber_click_sound, SoundBuffer* reload_sound) :
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
		fire_sound(fire_sound),
		chamber_click_sound(chamber_click_sound),
		reload_sound(reload_sound)
	{
		FinishReload();

		if(gun_model != NULL)
			for(unsigned int i = 0; i < gun_model->materials.size(); i++)
			{
				string material_name = gun_model->materials[i];
				DSNMaterial* mat = (DSNMaterial*)game_state->content->Load<Material>(material_name);
				gun_materials.push_back(mat);
			}
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
				iter++;
			}
			else
				iter = inaccuracy.erase(iter);
		}

		mflash_size *= exp(-16.0f * timestep);
		mflash_size -= 0.05f * timestep;

		if (reloading)
		{
			reload_wait -= timestep;
			if (reload_wait <= 0)
				FinishReload();
		}

		if (IsFiring(1))
			if (fire_wait <= 0 && !reloading)
			{
				if (clip > 0)
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

	void Gun::Fire(float total_inaccuracy, float now)
	{
		owner->PoseCharacter();						// to make sure we have a gun_xform... it will conveniently change our gun_xform for us

		Mat4 shot_mat = gun_xform * Mat4::Translation(0, 0.03f, 0.79f);

		Vec3 origin = shot_mat.TransformVec3(0, 0, 0, 1);
		Vec3 direction = Mat3::FromScaledAxis(0, -owner->yaw, 0) * Mat3::FromScaledAxis(owner->pitch, 0, 0) * Vec3(0, 0, 1);

		Shot* shot = CreateShot(origin, vel, (direction + Random3D::RandomNormalizedVector(total_inaccuracy + 0.001f)));
		if(shot != NULL)
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
		if (clip < clip_size && reload_wait <= 0)
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
		VisCleanup();				// just in case

		Sphere bs = Sphere(pos, 3);
		if (renderer->camera->CheckSphereVisibility(bs))
		{
			if(gun_model != NULL)
				((TestGame*)game_state)->VisUberModel(renderer, gun_model, 0, gun_xform, NULL, &gun_materials);

			if(mflash_model != NULL && mflash_size > 0)
			{
				Mat4 mflash_xform = gun_xform * Mat4::Translation(0, 0.03f, 0.79f) * Mat4::FromQuaternion(Quaternion::FromPYR(0, 0, -M_PI * 0.5f)) * Mat4::UniformScale(mflash_size);
				renderer->objects.push_back(RenderNode(mflash_material, new GlowyModelMaterialNodeData(mflash_model->GetVBO(), mflash_xform), Vec3::Dot(renderer->camera->GetPosition(), bs.center)));
			}
		}
	}

	void Gun::VisCleanup() { }
}
