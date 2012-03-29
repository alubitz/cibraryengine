#include "StdAfx.h"
#include "Dood.h"
#include "TestGame.h"

#include "Soldier.h"

#include "PoseAimingGun.h"
#include "WeaponEquip.h"
#include "WeaponIntrinsic.h"
#include "Shot.h"
#include "Damage.h"

#include "Particle.h"
#include "Corpse.h"

namespace Test
{
	bool enable_ragdolls = false;

	bool MaybeDoScriptedUpdate(Dood* dood);
	bool MaybeDoScriptedDeath(Dood* dood);

	/*
	 * Dood constants
	 */
	float ground_traction = 20.0f, air_traction = 0.1f;
	float top_speed_forward = 7.0f;							// running speed of a person can be around 5.8333[...] m/s
	float top_speed_sideways = 5.0f;

	float movement_damp = 0.2f;

	float air_spin_fix = 100;

	float yaw_rate = 10.0f, pitch_rate = 10.0f;




	/*
	 * Dood methods
	 */
	Dood::Dood(GameState* gs, UberModel* model, Vec3 pos, Team& team) :
		Pawn(gs),
		character_pose_time(-1),
		team(team),
		materials(),
		pos(pos),
		vel(),
		yaw(0),
		pitch(0),
		angular_vel(),
		jump_start_timer(0),
		hp(1.0f),
		eye_bone(NULL),
		model(model),
		character(NULL),
		rigid_body(NULL),
		physics(NULL),
		standing(0),
		equipped_weapon(NULL),
		intrinsic_weapon(NULL),
		collision_callback(this),
		OnAmmoFailure(),
		OnDamageTaken(),
		OnJumpFailure(),
		OnDeath()
	{
		// creating character
		character = new SkinnedCharacter(model->CreateSkeleton());

		eye_bone = character->skeleton->GetNamedBone("eye");

		// too bad this isn't saved somewhere?
		MassInfo mass_info = MassInfo();
		for(int i = -3; i <= 3; ++i)
			for(int j = 0; j <= 20; ++j)
				for(int k = -3; k <= 3; ++k)
					mass_info += MassInfo(Vec3(i * 0.1f, j * 0.1f, k * 0.1f), 0.09523809523809523809523809523808f);
		mass = mass_info.mass;
		inverse_moi = Mat3::Invert(Mat3(mass_info.moi));

		// look up all the materials the model uses in advance
		Cache<Material>* mat_cache = ((TestGame*)gs)->mat_cache;
		for(unsigned int i = 0; i < model->materials.size(); ++i)
		{
			string material_name = model->materials[i];
			DSNMaterial* mat = (DSNMaterial*)mat_cache->Load(material_name);
			materials.push_back(mat);
		}
	}

	void Dood::InnerDispose()
	{
		if(rigid_body != NULL)
		{
			rigid_body->Dispose();
			delete rigid_body;
		}

		Pawn::InnerDispose();
	}

	// overridden by subclasses
	void Dood::DoJumpControls(TimingInfo time, Vec3 forward, Vec3 rightward) { }

	void Dood::DoMovementControls(TimingInfo time, Vec3 forward, Vec3 rightward)
	{
		float timestep = time.elapsed;
		float traction = standing * ground_traction + (1 - standing) * air_traction;

		Vec3 desired_vel = forward * (top_speed_forward * max(-1.0f, min(1.0f, control_state->GetFloatControl("forward")))) + rightward * (top_speed_sideways * max(-1.0f, min(1.0f, control_state->GetFloatControl("sidestep"))));

		if (timestep > 0 && desired_vel.ComputeMagnitudeSquared() > 0)
		{
			desired_vel = (vel - desired_vel) * exp(-traction * timestep * standing) + desired_vel;

			Vec3 force = (desired_vel - vel) * mass / timestep;
			rigid_body->ApplyCentralForce(force);
		}
	}

	void Dood::DoWeaponControls(TimingInfo time)
	{
		if(control_state->GetBoolControl("reload"))
		{
			if(dynamic_cast<WeaponEquip*>(equipped_weapon) != NULL)
				((WeaponEquip*)equipped_weapon)->BeginReload();

			control_state->SetBoolControl("reload", false);
		}

		if(character != NULL)
		{
			PoseCharacter(time);

			if(equipped_weapon != NULL)
			{
				equipped_weapon->SetFiring(1, true, control_state->GetBoolControl("primary_fire"));
				equipped_weapon->OwnerUpdate(time);
			}
			if(intrinsic_weapon != NULL)
			{
				intrinsic_weapon->SetFiring(1, true, control_state->GetBoolControl("primary_fire"));
				intrinsic_weapon->OwnerUpdate(time);
			}

			character->UpdatePoses(time);
		}
	}

	void Dood::Update(TimingInfo time)
	{
		Pawn::Update(time);

		// figure out if you're standing on the ground or not

		pos = GetPosition();

		float timestep = time.elapsed;

		Vec3 forward = Vec3(-sin(yaw), 0, cos(yaw));
		Vec3 rightward = Vec3(-forward.z, 0, forward.x);

		Vec3 vel = rigid_body->GetLinearVelocity();												// velocity prior to forces being applied

		Vec3 delta_v = vel - this->vel;
		// collision damage!
		float falling_damage_base = abs(delta_v.ComputeMagnitude()) - 10.0f;
		if (falling_damage_base > 0)
			TakeDamage(Damage(this, falling_damage_base * 0.068f), Vec3());						// zero-vector indicates damage came from self

		/*
		TestGame* test_game = (TestGame*)game_state;
		if(this == test_game->player_pawn)
		{
			float speed = vel.ComputeMagnitude();
			int i = (int)floor(speed);
			int j = (int)(10.0f * (speed - i));

			stringstream ss;
			ss << "Speed = " << i << "." << j << " m/s";
			test_game->debug_text = ss.str();
		}
		*/

		this->vel = vel;

		if (timestep > 0)
		{
			Vec3 post_damp_vel = vel * exp(-timestep * movement_damp);
			Vec3 damp_force = (post_damp_vel - vel) * mass / timestep;

			rigid_body->ApplyCentralForce(damp_force);
		}

		DoMovementControls(time, forward, rightward);
		DoJumpControls(time, forward, rightward);

		DoPitchAndYawControls(time);

		// uncontrolled spinning!
		angular_vel *= exp(-(standing) * 200 * timestep);

		float angular_speed = angular_vel.ComputeMagnitude();
		if (angular_speed > 0)
		{
			float new_speed = max(0.0f, angular_speed - air_spin_fix * timestep);
			angular_vel = angular_vel * new_speed / angular_speed;

			yaw += angular_vel.y * timestep;
		}

		MaybeDoScriptedUpdate(this);

		DoWeaponControls(time);

		standing = 0;				// reset this after everything that needs to use it has used it
	}

	void Dood::DoPitchAndYawControls(TimingInfo time)
	{
		float timestep = time.elapsed;

		float desired_yaw = max(-1.0f, min(1.0f, control_state->GetFloatControl("yaw")));
		float desired_pitch = max(-1.0f, min(1.0f, control_state->GetFloatControl("pitch")));

		if (abs(desired_yaw) <= timestep * yaw_rate)
		{
			yaw += desired_yaw;
			control_state->SetFloatControl("yaw", 0.0f);
		}
		else if (desired_yaw < 0)
		{
			yaw -= timestep * yaw_rate;
			control_state->SetFloatControl("yaw", desired_yaw + timestep * yaw_rate);
		}
		else
		{
			yaw += timestep * yaw_rate;
			control_state->SetFloatControl("yaw", desired_yaw - timestep * yaw_rate);
		}

		if (abs(desired_pitch) <= timestep * pitch_rate)
		{
			pitch += desired_pitch;
			control_state->SetFloatControl("pitch", 0.0f);
		}
		else if (desired_pitch < 0)
		{
			pitch -= timestep * pitch_rate;
			control_state->SetFloatControl("pitch", desired_pitch + timestep * pitch_rate);
		}
		else
		{
			pitch += timestep * pitch_rate;
			control_state->SetFloatControl("pitch", desired_pitch - timestep * pitch_rate);
		}

		pitch = min((float)M_PI * 0.5f, max(-(float)M_PI * 0.5f, pitch));
	}

	Vec3 Dood::GetPosition() { return rigid_body->GetPosition(); }
	void Dood::SetPosition(Vec3 pos) { rigid_body->SetPosition(pos); }

	void Dood::Vis(SceneRenderer* renderer)
	{
		if(character != NULL)		// character will be null right when it becomes dead
		{
			Sphere bs = Sphere(pos, 2.5);
			if(renderer->camera->CheckSphereVisibility(bs))
			{
				double dist = (renderer->camera->GetPosition() - pos).ComputeMagnitude();
				int use_lod = dist < 45.0f ? 0 : 1;

				((TestGame*)game_state)->VisUberModel(renderer, model, use_lod, Mat4::Translation(pos), character, &materials);
			}
		}
	}

	Mat4 Dood::GetViewMatrix()
	{
		Mat4 flip = Mat4::FromQuaternion(Quaternion::FromPYR(0, float(M_PI), 0));

		if(eye_bone == NULL)
		{	
			Mat4 pitch_mat	= Mat4::FromQuaternion(Quaternion::FromPYR(	-pitch,	0,		0 ));
			Mat4 yaw_mat	= Mat4::FromQuaternion(Quaternion::FromPYR(	0,		yaw,	0 ));
			Mat4 loc		= Mat4::Translation(-pos);

			return flip * pitch_mat * yaw_mat * loc;
		}
		else
		{
			Mat4 eye_xform = eye_bone->GetTransformationMatrix();

			Vec3 pos_vec	= eye_xform.TransformVec3(eye_bone->rest_pos,	1);
			Vec3 left		= eye_xform.TransformVec3(1, 0, 0,				0);
			Vec3 up			= eye_xform.TransformVec3(0, 1, 0,				0);
			Vec3 forward	= eye_xform.TransformVec3(0, 0, 1,				0);

			float rm_values[] = { left.x, left.y, left.z, up.x, up.y, up.z, forward.x, forward.y, forward.z };
			return flip * Mat4::FromMat3(Mat3(rm_values)) * Mat4::Translation(-(pos + pos_vec));
		}
	}

	SoundSource* Dood::PlayDoodSound(SoundBuffer* buffer, float vol, bool looping)
	{
		if(this == ((TestGame*)game_state)->player_pawn)
			return game_state->sound_system->PlayEffect(buffer, vol, looping);
		else
			return game_state->sound_system->PlayEffect(buffer, pos, vel, vol, looping);
	}

	// overridden by subclasses
	void Dood::PreUpdatePoses(TimingInfo time) { }
	void Dood::PostUpdatePoses(TimingInfo time) { }

	void Dood::PoseCharacter() { PoseCharacter(TimingInfo(game_state->total_game_time - character_pose_time, game_state->total_game_time)); }
	void Dood::PoseCharacter(TimingInfo time)
	{
		if(character == NULL)			// the moment a Dood dies this will fail
			return;

		if (time.total > character_pose_time)
		{
			PreUpdatePoses(time);
			character->UpdatePoses(time);
			PostUpdatePoses(time);

			character_pose_time = time.total;
		}
	}

	void Dood::Spawned()
	{
		physics = game_state->physics_world;

		/*
		btVector3 spheres[] = { btVector3(0, 0.5f, 0), btVector3(0, 1.5f, 0) };
		float radii[] = { 0.5f, 0.5f };
		btConvexShape* shape = new btMultiSphereShape(spheres, radii, 2);
		*/
		CollisionShape* shape = new SphereShape(0.8f);

		MassInfo mass_info = MassInfo(Vec3(0, 1, 0), mass);			// point mass; has zero MoI, which Bullet treats like infinite MoI

		RigidBody* rigid_body = new RigidBody(shape, mass_info, pos);
		rigid_body->SetUserEntity(this);
		rigid_body->SetCollisionCallback(&collision_callback);

		physics->AddRigidBody(rigid_body);
		this->rigid_body = rigid_body;
	}

	void Dood::DeSpawned()
	{
		physics->RemoveRigidBody(rigid_body);

		if(equipped_weapon != NULL)
			equipped_weapon->is_valid = false;
		if(intrinsic_weapon != NULL)
			intrinsic_weapon->is_valid = false;
	}

	bool Dood::GetShot(Shot* shot, Vec3 poi, Vec3 momentum)
	{
		Vec3 from_dir = Vec3::Normalize(shot->origin - pos);
		TakeDamage(shot->GetDamage(), from_dir);

		// let's transfer us some momentum!

		// character controllers are special cases where angular momentum doesn't get applied the same way...
		rigid_body->ApplyCentralForce(momentum);

		Vec3 radius_vec = poi - pos;						// now for the angular part...
		double radius = radius_vec.ComputeMagnitude();
		if (radius > 0)
		{
			Vec3 angular_impulse = Vec3::Cross(momentum, radius_vec);
			Vec3 angular_delta_v = inverse_moi * angular_impulse;

			angular_vel += angular_delta_v;
		}

		Splatter(shot, poi, momentum);
		return true;
	}

	void Dood::TakeDamage(Damage damage, Vec3 from_dir)
	{
		DamageTakenEvent evt(this, from_dir, damage);
		OnDamageTaken(&evt);

		if(!evt.cancel)
		{
			float old_hp = hp;
			hp -= damage.amount;

			if(hp <= 0 && old_hp > 0)
				Die(damage);
		}
	}

	void Dood::Splatter(Shot* shot, Vec3 poi, Vec3 momentum)
	{
		if(blood_material != NULL)
			for (int i = 0; i < 8; ++i)
			{
				Particle* p = new Particle(game_state, poi, Random3D::RandomNormalizedVector(Random3D::Rand(5)) + momentum * Random3D::Rand(), NULL, blood_material, Random3D::Rand(0.05f, 0.15f), 0.25f);
				p->gravity = 9.8f;
				p->damp = 0.05f;

				game_state->Spawn(p);
			}
	}

	void Dood::Die(Damage cause)
	{
		DeathEvent evt(this, cause);
		OnDeath(&evt);

		MaybeDoScriptedDeath(this);

		if(enable_ragdolls)
		{
			Corpse* corpse = new Corpse(game_state, this, 50.0f);
			game_state->Spawn(corpse);
		}

		is_valid = false;
	}

	bool Dood::GetAmmoFraction(float& result)
	{
		if(equipped_weapon != NULL && equipped_weapon->GetAmmoFraction(result))
			return true;
		if(intrinsic_weapon != NULL && intrinsic_weapon->GetAmmoFraction(result))
			return true;
		return false;
	}

	bool Dood::GetAmmoCount(int& result)
	{
		if(equipped_weapon != NULL && equipped_weapon->GetAmmoCount(result))
			return true;
		if(intrinsic_weapon != NULL && intrinsic_weapon->GetAmmoCount(result))
			return true;
		return false;
	}




	/*
	 * Dood::ContactCallback methods
	 */
	
	Dood::ContactCallback::ContactCallback(Dood* dood) : dood(dood) { }
	bool Dood::ContactCallback::OnCollision(const ContactPoint& collision)
	{
		ContactPoint::Part self = collision.a.obj->GetUserEntity() == dood ? collision.a : collision.b;
		ContactPoint::Part other = collision.a.obj->GetUserEntity() == dood ? collision.b : collision.a;

		Vec3 normal = other.norm;
		if(normal.y > 0.1)
		{
			// player only counts as standing when they aren't moving away from the surface		
			float dot = Vec3::Dot(dood->rigid_body->GetLinearVelocity() - other.obj->GetLinearVelocity(), normal);
			if(dot <= 0.1)
				dood->standing = 1;
		}

		return true;
	}




	/*
	 * Dood scripting stuff
	 */
	int dood_index(lua_State* L)
	{
		Dood** dood_ptr = (Dood**)lua_touserdata(L, 1);

		if(lua_isstring(L, 2))
		{
			string key = lua_tostring(L, 2);

			lua_settop(L, 0);

			if(key == "is_valid")						{ lua_pushboolean(L,  *dood_ptr != NULL); return 1; }
			else
			{
				Dood* dood = *dood_ptr;
				if(dood != NULL)
				{
					if		(key == "position")			{ PushLuaVector(L, dood->pos); return 1; }
					else if	(key == "is_player")		{ lua_pushboolean(L, dood == ((TestGame*)dood->game_state)->player_pawn); return 1; }
					else if	(key == "health")			{ lua_pushnumber(L, dood->hp); return 1; }			
					else if	(key == "yaw")				{ lua_pushnumber(L, dood->yaw); return 1; }
					else if	(key == "pitch")			{ lua_pushnumber(L, dood->pitch); return 1; }
					else
						Debug("unrecognized key for Dood:index: " + key + "\n");
				}
				else
					Debug("Attempting to access the properties of an invalid dood handle\n");
			}
		}
		return 0;
	}

	int dood_newindex(lua_State* L)
	{
		Dood** dood_ptr = (Dood**)lua_touserdata(L, 1);
		Dood* dood = *dood_ptr;
		if(dood == NULL)
		{
			Debug("Attempting to modify the properties of an invalid dood handle\n");
			return 0;
		}
		else
		{
			if(lua_isstring(L, 2))
			{
				string key = lua_tostring(L, 2);
				if (key == "update_callback")
				{
					if(lua_isfunction(L, 3))
					{
						lua_getmetatable(L, 1);								// push; top = 4
						lua_getfield(L, 4, "update_callback");				// push the update_callback table; top = 5
						lua_pushvalue(L, 1);								// push the dood; top = 6
						lua_pushvalue(L, 3);								// push the function; top = 7
						lua_settable(L, 5);									// pop x2; top = 5

						lua_settop(L, 0);
						return 0;
					}
				}
				else if (key == "death_callback")
				{
					if(lua_isfunction(L, 3))
					{
						lua_getmetatable(L, 1);								// push; top = 4
						lua_getfield(L, 4, "death_callback");				// push the death_callback table; top = 5
						lua_pushvalue(L, 1);								// push the dood; top = 6
						lua_pushvalue(L, 3);								// push the function; top = 7
						lua_settable(L, 5);									// pop x2; top = 5

						lua_settop(L, 0);
						return 0;
					}
				}
				else
					Debug("unrecognized key for Dood:newindex: " + key + "\n");
			}
		}

		return 0;
	}

	int dood_eq(lua_State* L)
	{
		void* a = lua_touserdata(L, 1);
		void* b = lua_touserdata(L, 2);

		stringstream ss;
		ss << "a = " << a << "; b = " << b << endl;
		Debug(ss.str());

		if(a != NULL && b != NULL && a == b)
			return true;
		return false;
	}

	void PushDoodHandle(lua_State* L, Dood* dood)
	{
		lua_pushlightuserdata(L, dood->GetScriptingHandle());

		lua_getmetatable(L, -1);
		if(lua_istable(L, -1))
		{
			lua_pop(L, 1);
			return;
		}

		// I tried putting this inside the if statement, but that didn't work
		lua_getglobal(L, "DoodMeta");
		if(lua_isnil(L, -1))
		{
			lua_pop(L, 1);
			// must create metatable for globals
			lua_newtable(L);									// push; top = 2

			lua_pushcclosure(L, dood_index, 0);					// push; top = 3
			lua_setfield(L, -2, "__index");						// pop; top = 2

			lua_pushcclosure(L, dood_newindex, 0);
			lua_setfield(L, -2, "__newindex");

			lua_pushcclosure(L, dood_eq, 0);
			lua_setfield(L, -2, "__eq");

			lua_newtable(L);
			lua_setfield(L, -2, "update_callback");
			
			lua_newtable(L);
			lua_setfield(L, -2, "death_callback");

			lua_setglobal(L, "DoodMeta");
			lua_getglobal(L, "DoodMeta");
		}
		lua_setmetatable(L, -2);								// set field of 1; pop; top = 1		
	}

	bool MaybeDoScriptedUpdate(Dood* dood)
	{
		lua_State* L = ScriptSystem::GetGlobalState().GetLuaState();

		lua_settop(L, 0);
		PushDoodHandle(L, dood);

		lua_getmetatable(L, 1);						// push; top = 2
		if(!lua_isnil(L, 2))
		{
			lua_getfield(L, 2, "update_callback");	// pop+push; top = 3
			if(lua_istable(L, 3))
			{
				lua_pushvalue(L, 1);				// push dood; top = 4
				lua_gettable(L, 3);					// pop+push; top = 4

				if(lua_isfunction(L, 4))
				{
					lua_pushvalue(L, 1);			// push dood; top = 5
					ScriptSystem::GetGlobalState().DoFunction(1, 0);

					lua_settop(L, 0);
					return true;
				}

				lua_settop(L, 0);
				return false;
			}
		}

		lua_settop(L, 0);
		return false;
	}




	/*
	 * If this Dood has a death callback associated with it via Lua-scripting, call that callback
	 */
	bool MaybeDoScriptedDeath(Dood* dood)
	{
		lua_State* L = ScriptSystem::GetGlobalState().GetLuaState();

		lua_settop(L, 0);
		PushDoodHandle(L, dood);

		lua_getmetatable(L, 1);					// push; top = 2
		if(!lua_isnil(L, 2))
		{
			lua_getfield(L, 2, "death_callback");	// pop+push; top = 3
			if(lua_istable(L, 3))
			{
				lua_pushvalue(L, 1);			// push dood; top = 4
				lua_gettable(L, 3);				// pop+push; top = 4

				if(lua_isfunction(L, 4))
				{
					lua_pushvalue(L, 1);		// push dood; top = 5
					ScriptSystem::GetGlobalState().DoFunction(1, 0);		// pop

					lua_settop(L, 0);
					return true;
				}

				lua_settop(L, 0);
				return false;
			}
		}

		lua_settop(L, 0);
		return false;
	}
}
