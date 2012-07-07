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

#include "DoodOrientationConstraint.h"

namespace Test
{
	/*
	 * Dood constants
	 */
	float ground_traction = 20.0f, air_traction = 0.1f;
	float top_speed_forward = 7.0f;							// running speed of a person can be around 5.8333[...] m/s
	float top_speed_sideways = 5.0f;

	float yaw_rate = 10.0f, pitch_rate = 10.0f;




	/*
	 * Dood::BoneShootable methods
	 */
	bool Dood::BoneShootable::GetShot(Shot* shot, Vec3 poi, Vec3 momentum)
	{
		Vec3 from_dir = Vec3::Normalize(shot->origin - dood->pos);
		dood->TakeDamage(shot->GetDamage(), from_dir);

		dood->Splatter(shot, poi, momentum);

		Mat4 xform;
		{
			xform = rbi->GetTransformationMatrix();
			float ori_values[] = {xform[0], xform[1], xform[2], xform[4], xform[5], xform[6], xform[8], xform[9], xform[10]};
			Quaternion rigid_body_ori = Quaternion::FromRotationMatrix(Mat3(ori_values).Transpose());
			Vec3 pos = xform.TransformVec3_1(0, 0, 0);
			xform = Mat4::FromPositionAndOrientation(pos, rigid_body_ori.ToMat3().Transpose());
		}

		Vec3 pos = xform.TransformVec3_1(0, 0, 0);
		Vec3 x_axis = xform.TransformVec3_0(1, 0, 0);
		Vec3 y_axis = xform.TransformVec3_0(0, 1, 0);
		Vec3 z_axis = xform.TransformVec3_0(0, 0, 1);

		Vec3 local_poi;
		local_poi = poi - pos;
		local_poi = Vec3(Vec3::Dot(local_poi, x_axis), Vec3::Dot(local_poi, y_axis), Vec3::Dot(local_poi, z_axis));
		local_poi = local_poi.x * x_axis + local_poi.y * y_axis + local_poi.z * z_axis;

		rbi->ApplyImpulse(momentum, local_poi);
		return true;
	}




	/*
	 * Dood methods
	 */
	bool MaybeDoScriptedUpdate(Dood* dood);
	bool MaybeDoScriptedDeath(Dood* dood);

	Dood::Dood(GameState* gs, UberModel* model, ModelPhysics* mphys, Vec3 pos, Team& team) :
		Pawn(gs),
		character_pose_time(-1),
		team(team),
		materials(),
		pos(pos),
		vel(),
		yaw(0),
		pitch(0),
		jump_start_timer(0),
		hp(1.0f),
		eye_bone(NULL),
		model(model),
		character(NULL),
		posey(NULL),
		root_rigid_body(NULL),
		rigid_bodies(),
		shootables(),
		bone_indices(),
		constraints(),
		orientation_constraint(NULL),
		physics(NULL),
		mphys(mphys),
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
		posey = new PosedCharacter(new Skeleton(character->skeleton));

		eye_bone = character->skeleton->GetNamedBone("eye");

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
		DeSpawned();

		if(posey)
		{
			for(list<Pose*>::iterator iter = posey->active_poses.begin(); iter != posey->active_poses.end(); ++iter)
				delete *iter;
			posey->active_poses.clear();

			posey->Dispose();
			delete posey;
			posey = NULL;
		}

		if(character)
		{
			character->Dispose();
			delete character;
			character = NULL;
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
			if(timestep < 0.01f)			// bit of hackery here to make stuff work properly with extremely high framerates
				timestep = 0.01f;

			desired_vel = (vel - desired_vel) * exp(-traction * timestep * standing) + desired_vel;
			Vec3 dv = desired_vel - vel;
			for(vector<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
				(*iter)->ApplyCentralImpulse(dv * (*iter)->GetMassInfo().mass);
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

			posey->UpdatePoses(time);
		}
	}

	void Dood::Update(TimingInfo time)
	{
		Pawn::Update(time);

		pos = GetPosition();

		float timestep = time.elapsed;

		Vec3 forward = Vec3(-sin(yaw), 0, cos(yaw));
		Vec3 rightward = Vec3(-forward.z, 0, forward.x);

		// prevent bones from falling asleep (not yet possible, but whatever)
		for(vector<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
			(*iter)->Activate();

		Vec3 vel = root_rigid_body->GetLinearVelocity();										// velocity prior to forces being applied

		Vec3 delta_v = vel - this->vel;

		// TODO: make this work again sometime
		// collision damage!
		//float falling_damage_base = abs(delta_v.ComputeMagnitude()) - 10.0f;
		//if (falling_damage_base > 0)
		//	TakeDamage(Damage(this, falling_damage_base * 0.068f), Vec3());						// zero-vector indicates damage came from self

		orientation_constraint->desired_ori = Quaternion::FromPYR(0, -yaw, 0);

		this->vel = vel;

		DoMovementControls(time, forward, rightward);
		DoJumpControls(time, forward, rightward);
		DoPitchAndYawControls(time);

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

	Vec3 Dood::GetPosition() { return root_rigid_body->GetPosition(); }
	void Dood::SetPosition(Vec3 pos) { root_rigid_body->SetPosition(pos); }

	void Dood::Vis(SceneRenderer* renderer)
	{
		if(character != NULL)		// character will be null right when it becomes dead
		{
			Sphere bs = Sphere(pos, 2.5);
			if(renderer->camera->CheckSphereVisibility(bs))
			{
				double dist = (renderer->camera->GetPosition() - pos).ComputeMagnitude();
				int use_lod = dist < 45.0f ? 0 : 1;

				SkinnedCharacter::RenderInfo render_info = character->GetRenderInfo();
				((TestGame*)game_state)->VisUberModel(renderer, model, use_lod, Mat4::Translation(pos), &render_info, &materials);
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

			Vec3 pos_vec	= eye_xform.TransformVec3_1(eye_bone->rest_pos);
			Vec3 left		= eye_xform.TransformVec3_0(1, 0, 0);
			Vec3 up			= eye_xform.TransformVec3_0(0, 1, 0);
			Vec3 backward	= eye_xform.TransformVec3_0(0, 0, 1);

			Mat3 rm(left.x, left.y, left.z, up.x, up.y, up.z, backward.x, backward.y, backward.z);
			return flip * Mat4::FromMat3(rm) * Mat4::Translation(-(pos + pos_vec));
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
		if(!is_valid)
			return;
		float now = time.total;
		if (now > character_pose_time)
		{
			PreUpdatePoses(time);

			origin = rigid_bodies[0]->GetPosition();

			for(unsigned int i = 0; i < rigid_bodies.size(); ++i)
			{
				RigidBody* body = rigid_bodies[i];
				unsigned int bone_index = bone_indices[i];
				Bone* bone = character->skeleton->bones[bone_index];

				Quaternion rigid_body_ori = body->GetOrientation();
				Vec3 rigid_body_pos = body->GetPosition();

				bone->ori = Quaternion::Reverse(rigid_body_ori);

				// model origin = rigid body pos - model rot * rest pos
				Vec3 offset = Mat4::FromQuaternion(bone->ori).TransformVec3_1(0, 0, 0);
				bone->pos = rigid_body_pos - offset - origin;			//subtract origin to account for that whole-model transform in Corpse::Imp::Vis
			}

			character->render_info.Invalidate();
			character->skeleton->InvalidateCachedBoneXforms();
			posey->UpdatePoses(TimingInfo(character_pose_time >= 0 ? now - character_pose_time : 0, now));

			for(vector<PhysicsConstraint*>::iterator iter = constraints.begin(); iter != constraints.end(); ++iter)
			{
				JointConstraint* constraint = (JointConstraint*)(*iter);
				
				for(unsigned int i = 0; i < rigid_bodies.size(); ++i)
				{
					RigidBody* ibody = rigid_bodies[i];
					if(ibody == constraint->obj_a)
					{
						for(unsigned int j = i + 1; j < rigid_bodies.size(); ++j)
						{
							RigidBody* jbody = rigid_bodies[j];
							if(jbody == constraint->obj_b)
							{
								Bone* i_bone = posey->skeleton->bones[bone_indices[i]];
								Bone* j_bone = posey->skeleton->bones[bone_indices[j]];

								constraint->SetDesiredOrientation(j_bone->ori);

								i = j = rigid_bodies.size();			// skip to end of these two for loops
							}
						}
					}
				}
			}

			PostUpdatePoses(time);

			character_pose_time = now;
		}
	}

	void Dood::Spawned()
	{
		if(mphys == NULL)
		{
			Debug("Dood's mphys is NULL; this Dood will be removed!\n");

			is_valid = false;
			return;
		}

		whole_xform = Mat4::Translation(pos);

		physics = game_state->physics_world;

		unsigned int count = mphys->bones.size();
		ModelPhysics::BonePhysics** bone_physes = new ModelPhysics::BonePhysics* [count];

		// given string id, get index of rigid body
		map<unsigned int, unsigned int> name_indices;

		// create rigid bodies
		for(unsigned int i = 0; i < count; ++i)
		{
			if(ModelPhysics::BonePhysics* phys = &mphys->bones[i])
			{
				bone_physes[i] = phys;

				CollisionShape* shape = phys->collision_shape;
				if(shape != NULL)
				{
					RigidBody* rigid_body = new RigidBody(shape, phys->mass_info, pos);

					rigid_body->SetDamp(0.05f);
					rigid_body->SetFriction(1.0f);

					physics->AddRigidBody(rigid_body);
					rigid_bodies.push_back(rigid_body);

					BoneShootable* shootable = new BoneShootable(game_state, this, rigid_body, blood_material);
					rigid_body->SetUserEntity(shootable);
					rigid_body->SetCollisionCallback(&collision_callback);


					unsigned int bone_name = Bone::string_table[phys->bone_name];
					name_indices[bone_name] = bone_indices.size();

					shootables.push_back(shootable);
					bone_indices.push_back(i);

					if(bone_name == Bone::string_table["carapace"] || bone_name == Bone::string_table["pelvis"])
						root_rigid_body = rigid_body;
				}
			}
		}

		// create constraints between bones
		for(vector<ModelPhysics::JointPhysics>::iterator iter = mphys->joints.begin(); iter != mphys->joints.end(); ++iter)
		{
			ModelPhysics::JointPhysics& phys = *iter;
				
			if(phys.bone_b != 0)						// don't deal with special attachment points
			{
				const string& bone_a_name = mphys->bones[phys.bone_a - 1].bone_name;
				const string& bone_b_name = mphys->bones[phys.bone_b - 1].bone_name;

				RigidBody* bone_a = rigid_bodies[name_indices[Bone::string_table[bone_a_name]]];
				RigidBody* bone_b = rigid_bodies[name_indices[Bone::string_table[bone_b_name]]];

				JointConstraint* c = new JointConstraint(bone_b, bone_a, phys.pos, phys.axes, phys.max_extents, phys.angular_damp);

				constraints.push_back(c);

				physics->AddConstraint(c);
			}
		}

		// emancipate bones
		for(unsigned int i = 0; i < count; ++i)
		{
			Bone* bone = character->skeleton->bones[i];

			if(name_indices.find(bone->name) != name_indices.end())		// only emancipate bones which have rigid bodies!
				bone->parent = NULL;									// orientation and position are no longer relative to a parent!
		}

		if(rigid_bodies.size() == 0)
		{
			Debug("Dood has no rigid bodies; this Dood will be removed!\n");
			is_valid = false;
		}

		delete[] bone_physes;

		orientation_constraint = new DoodOrientationConstraint(this);
		physics->AddConstraint(orientation_constraint);
	}

	void Dood::DeSpawned()
	{
		for(unsigned int i = 0; i < shootables.size(); ++i)
			delete shootables[i];
		shootables.clear();

		// clear constraints
		for(unsigned int i = 0; i < constraints.size(); ++i)
		{
			PhysicsConstraint* c = constraints[i];
			physics->RemoveConstraint(c);

			c->Dispose();
			delete c;
		}
		constraints.clear();

		if(orientation_constraint != NULL)
		{
			physics->RemoveConstraint(orientation_constraint);
			delete orientation_constraint;
			orientation_constraint = NULL;
		}

		// clear rigid bodies
		for(unsigned int i = 0; i < rigid_bodies.size(); ++i)
		{
			RigidBody* body = rigid_bodies[i];
			if(physics != NULL)
				physics->RemoveRigidBody(body);

			body->DisposePreservingCollisionShape();
			delete body;
		}
		rigid_bodies.clear();

		if(equipped_weapon != NULL)
			equipped_weapon->is_valid = false;
		if(intrinsic_weapon != NULL)
			intrinsic_weapon->is_valid = false;
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

		// TODO: revise this; we don't have a Corpse class anymore; a ragdoll is just a dead Dood

		if(this != ((TestGame*)game_state)->player_pawn)
			controller->is_valid = false;
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
		for(vector<RigidBody*>::iterator iter = dood->rigid_bodies.begin(); iter != dood->rigid_bodies.end(); ++iter)
			if(collision.obj_a == *iter || collision.obj_b == *iter)
			{
				ContactPoint::Part self = collision.obj_a == *iter ? collision.a : collision.b;
				ContactPoint::Part other = collision.obj_a == *iter ? collision.b : collision.a;

				for(vector<RigidBody*>::iterator jter = dood->rigid_bodies.begin(); jter != dood->rigid_bodies.end(); ++jter)
					if(iter != jter && collision.obj_a == *jter || collision.obj_b == *jter)
						return true;			// collisions between bones of the same dood don't count as "standing"

				Vec3 normal = other.norm;
				if(normal.y > 0.1)
					dood->standing = 1;

				return true;
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
					if		(key == "id")				{ lua_pushnumber(L, dood->GetID()); return 1; }
					else if	(key == "position")			{ PushLuaVector(L, dood->pos); return 1; }
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
						lua_pushnumber(L, dood->GetID());					// push the dood's id; top = 6
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
						lua_pushnumber(L, dood->GetID());					// push the dood's id; top = 6
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

	int dood_gc(lua_State* L)
	{
		Dood** dood_ptr = (Dood**)lua_touserdata(L, 1);
		if(dood_ptr != NULL)
		{
			if(Dood* dood = *dood_ptr)
				dood->TossScriptingHandle();
		}

		lua_settop(L, 0);
		return 0;
	}

	void PushDoodHandle(lua_State* L, Dood* dood)
	{
		dood->PushScriptingHandle(L);

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

			lua_pushcclosure(L, dood_gc, 0);
			lua_setfield(L, -2, "__gc");

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
				lua_pushnumber(L, dood->GetID());	// push dood's id; top = 4
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
				lua_pushnumber(L, dood->GetID());	// push dood's id; top = 4
				lua_gettable(L, 3);					// pop+push; top = 4

				if(lua_isfunction(L, 4))
				{
					lua_pushvalue(L, 1);									// push dood; top = 5
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
