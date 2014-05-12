#include "StdAfx.h"
#include "Gun.h"

#include "Dood.h"
#include "Shot.h"
#include "TestGame.h"

#define USE_GUN_XFORM_AS_SHOT_XFORM 1

#define ENABLE_LEFT_GRIP            1
#define ENABLE_RIGHT_GRIP           1

namespace Test
{
	/*
	 * Gun methods
	 */
	Gun::Gun(GameState* game_state, Dood* owner, UberModel* gun_model, VertexBuffer* mflash_model, GlowyModelMaterial* mflash_material, ModelPhysics* mphys, SoundBuffer* fire_sound, SoundBuffer* chamber_click_sound, SoundBuffer* reload_sound) :
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
		model_phys(mphys),
		rigid_body(NULL),
		l_grip(NULL),
		r_grip(NULL),
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
		if(rigid_body)	{ rigid_body->Dispose();	delete rigid_body;	rigid_body = NULL; }

		if(l_grip)		{ l_grip->Dispose();		delete l_grip;		l_grip = NULL; }
		if(r_grip)		{ r_grip->Dispose();		delete r_grip;		r_grip = NULL; }

		WeaponEquip::InnerDispose();
	}

	void Gun::Update(const TimingInfo& time)
	{
		if(owner == NULL)
			UnownedUpdate(time);

		WeaponEquip::Update(time);
	}

	void Gun::OwnerUpdate(const TimingInfo& time)
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

	void Gun::UnownedUpdate(const TimingInfo& time)
	{
		SharedUpdate(time);

		for(list<Inaccuracy>::iterator iter = inaccuracy.begin(); iter != inaccuracy.end(); )
		{
			if(time.total > iter->time)
				iter = inaccuracy.erase(iter);
			else
				++iter;
		}

		if(rigid_body != NULL)
		{
			gun_xform = rigid_body->GetTransformationMatrix();
			sound_pos = rigid_body->GetPosition();
			sound_vel = rigid_body->GetLinearVelocity();
		}
	}

	void Gun::SharedUpdate(const TimingInfo& time)
	{
		mflash_size *= expf(-16.0f * time.elapsed);
		mflash_size -= 0.05f * time.elapsed;
	}

	void Gun::Fire(float total_inaccuracy, float now)
	{
		Mat4 shot_mat = gun_xform * Mat4::Translation(barrel_pos);
		Vec3 origin = shot_mat.TransformVec3_1(0, 0, 0);

#if USE_GUN_XFORM_AS_SHOT_XFORM
		Vec3 direction = shot_mat.TransformVec3_0(0, 0, 1);
#else
		Vec3 direction = Mat3::FromRVec(0, -owner->yaw, 0) * Mat3::FromRVec(owner->pitch, 0, 0) * Vec3(0, 0, 1);
#endif

		if(Shot* shot = CreateShot(origin, vel, (direction + Random3D::RandomNormalizedVector(total_inaccuracy + 0.001f))))
			game_state->Spawn(shot);

		inaccuracy.push_back(Inaccuracy(0.015f, now + 0.1f));

		PlayWeaponSound(fire_sound, 1.0f, false);

		mflash_size = 1.0f;

		fire_wait = refire_interval;
		--clip;

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
				Mat4 mflash_xform = gun_xform * Mat4::Translation(barrel_pos) * Mat4::FromQuaternion(Quaternion::FromRVec(0, 0, -float(M_PI) * 0.5f)) * Mat4::UniformScale(mflash_size);
				renderer->objects.push_back(RenderNode(mflash_material, new GlowyModelMaterialNodeData(mflash_model, mflash_xform), Vec3::Dot(renderer->camera->GetForward(), bs.center)));
			}
		}
	}

	Mat4 Gun::GetInitialXform() { return owner->root_rigid_body->GetTransformationMatrix(); }

	void Gun::Spawned()
	{
		WeaponEquip::Spawned();

		physics = game_state->physics_world;

		if(model_phys != NULL && model_phys->bones.size() > 0)
		{
			Mat4 xform = GetInitialXform();

			Vec3 pos = xform.TransformVec3_1(0, 0, 0);
			Vec3 a = xform.TransformVec3_0(1, 0, 0);
			Vec3 b = xform.TransformVec3_0(0, 1, 0);
			Vec3 c = xform.TransformVec3_0(0, 0, 1);

			ModelPhysics::BonePhysics& bone = model_phys->bones[0];

			rigid_body = new RigidBody(this, bone.collision_shape, bone.mass_info, pos, Quaternion::FromRotationMatrix(Mat3(a.x, b.x, c.x, a.y, b.y, c.y, a.z, b.z, c.z)));
			physics->AddCollisionObject(rigid_body);
		}
	}

	void Gun::DeSpawned()
	{
		WeaponEquip::DeSpawned();

		if(owner != NULL)
			UnEquip(owner);

		if(rigid_body != NULL)
			physics->RemoveCollisionObject(rigid_body);
	}

	void Gun::Equip(Dood* new_owner)
	{
		WeaponEquip::Equip(new_owner);

		if(rigid_body != NULL)
		{
			if(l_grip == NULL)
			{
				RigidBody* gripper_rb = NULL;

				unsigned int id = Bone::string_table["l hand"];
				for(unsigned int i = 0; i < new_owner->character->skeleton->bones.size(); ++i)
					if(new_owner->character->skeleton->bones[i]->name == id)
					{
						gripper_rb = new_owner->bone_to_rbody[i];
						break;
					}
#if ENABLE_LEFT_GRIP
				if(gripper_rb != NULL)
					l_grip = new FixedJointConstraint(rigid_body, gripper_rb, Vec3( 0.000f,  0.000f,  0.468f), Vec3( 0.990f,  1.113f,  0.037f), Quaternion::FromRVec(-Vec3(0.0703434f, 0.0146932f, -2.50207f)));
#endif
			}
			else
				l_grip->obj_a = rigid_body;

			if(r_grip == NULL)
			{
				RigidBody* gripper_rb = NULL;

				unsigned int id = Bone::string_table["r hand"];
				for(unsigned int i = 0; i < new_owner->character->skeleton->bones.size(); ++i)
					if(new_owner->character->skeleton->bones[i]->name == id)
					{
						gripper_rb = new_owner->bone_to_rbody[i];
						break;
					}
#if ENABLE_RIGHT_GRIP
				if(gripper_rb != NULL)
					r_grip = new FixedJointConstraint(rigid_body, gripper_rb, Vec3( 0.000f, -0.063f, -0.152f), Vec3(-0.959f,  1.098f,  0.077f), Quaternion::FromRVec(-Vec3(-1.27667f, 0.336123f, 0.64284f)));
#endif
			}
			else
				r_grip->obj_a = rigid_body;

			if(l_grip != NULL)
				physics->AddConstraint(l_grip);
			if(r_grip != NULL)
				physics->AddConstraint(r_grip);

			rigid_body->SetCollisionEnabled(new_owner->collision_group, false);			// TODO: maybe disable this?

			new_owner->velocity_change_bodies.insert(rigid_body);
		}
	}

	void Gun::UnEquip(Dood* old_owner)
	{
		if(rigid_body != NULL)
		{
			// break physics constraints (and mark them in a way that indicates they are broken)
			if(l_grip != NULL && l_grip->obj_a != NULL)
			{
				physics->RemoveConstraint(l_grip);
				l_grip->obj_a = NULL;
			}
			if(r_grip != NULL && r_grip->obj_a != NULL)
			{
				physics->RemoveConstraint(r_grip);
				r_grip->obj_a = NULL;
			}

			rigid_body->SetCollisionEnabled(old_owner->collision_group, true);

			set<RigidBody*>::iterator found = old_owner->velocity_change_bodies.find(rigid_body);
			if(found != old_owner->velocity_change_bodies.end())
				old_owner->velocity_change_bodies.erase(found);
		}

		WeaponEquip::UnEquip(old_owner);
	}
}
