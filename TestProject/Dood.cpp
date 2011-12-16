#include "StdAfx.h"
#include "Dood.h"
#include "TestGame.h"

#include "PoseAimingGun.h"
#include "WeaponEquip.h"
#include "WeaponIntrinsic.h"
#include "Shot.h"
#include "Damage.h"

#include "Particle.h"
#include "Corpse.h"

namespace Test
{
	bool enable_ragdolls = true;

	bool MaybeDoScriptedAI(Dood* dood);
	bool MaybeDoScriptedUpdate(Dood* dood);
	bool MaybeDoScriptedDeath(Dood* dood);

	/*
	 * Dood constants
	 */
	float ground_traction = 20.0f, air_traction = 0.1f;
	float top_speed_forward = 7.0f;							// running speed of a person can be around 5.8333[...] m/s
	float top_speed_sideways = 5.0f;
	float jump_speed = 4.0f;
	float jump_pack_accel = 15.0f;

	float bug_leap_duration = 0.5f;
	float bug_leap_speed = 8.0f;

	float jump_to_fly_delay = 0.3f;

	float gravity = 9.8f;
	float movement_damp = 0.2f;

	float air_spin_fix = 100;

	float yaw_rate = 10.0f, pitch_rate = 10.0f;

	float jump_fuel_spend_rate = 0.5f, jump_fuel_refill_rate = 0.4f;
	float flying_accel = 8.0f;


	// some lua-related dood functions
	bool GetBoolControl(Dood* dood, string control_name);
	float GetFloatControl(Dood* dood, string control_name);
	void SetBoolControl(Dood* dood, string control_name, bool value);
	void SetFloatControl(Dood* dood, string control_name, float value);

	void GenerateHardCodedWalkAnimation(IKPose* ik_pose)
	{
		KeyframeAnimation* ka = ik_pose->keyframe_animation;

		{
			Keyframe kf(0.5f);
			kf.next = 1;

			kf.values["l leg a 1"] = BoneInfluence(Vec3(	1,	0.3f,	0	), Vec3(), 1);
			kf.values["l leg b 1"] = BoneInfluence(Vec3(	0,	-1,		0	), Vec3(), 1);
			kf.values["l leg c 1"] = BoneInfluence(Vec3(	1,	0.5f,	0	), Vec3(), 1);

			kf.values["r leg a 1"] = BoneInfluence(Vec3(	0,	0.3f,	0	), Vec3(), 1);
			kf.values["r leg b 1"] = BoneInfluence(Vec3(	1,	-1,		0	), Vec3(), 1);
			kf.values["r leg c 1"] = BoneInfluence(Vec3(	0,	0.5f,	0	), Vec3(), 1);

			ka->frames.push_back(kf);
		}
		{
			Keyframe kf(0.5f);
			kf.next = 2;
			
			kf.values["l leg a 1"] = BoneInfluence(Vec3(	0,	0.3f,	0	), Vec3(), 1);
			kf.values["l leg b 1"] = BoneInfluence(Vec3(	1,	-1,		0	), Vec3(), 1);
			kf.values["l leg c 1"] = BoneInfluence(Vec3(	0,	0.5f,	0	), Vec3(), 1);

			kf.values["r leg a 1"] = BoneInfluence(Vec3(	1,	0.3f,	0	), Vec3(), 1);
			kf.values["r leg b 1"] = BoneInfluence(Vec3(	0,	-1,		0	), Vec3(), 1);
			kf.values["r leg c 1"] = BoneInfluence(Vec3(	1,	0.5f,	0	), Vec3(), 1);

			ka->frames.push_back(kf);
		}
		{
			Keyframe kf(0.5f);
			kf.next = 3;

			kf.values["l leg a 1"] = BoneInfluence(Vec3(	0,	-0.3f,	0	), Vec3(), 1);
			kf.values["l leg b 1"] = BoneInfluence(Vec3(	1,	1,		0	), Vec3(), 1);
			kf.values["l leg c 1"] = BoneInfluence(Vec3(	0,	-0.5f,	0	), Vec3(), 1);

			kf.values["r leg a 1"] = BoneInfluence(Vec3(	1,	-0.3f,	0	), Vec3(), 1);
			kf.values["r leg b 1"] = BoneInfluence(Vec3(	0,	1,		0	), Vec3(), 1);
			kf.values["r leg c 1"] = BoneInfluence(Vec3(	1,	-0.5f,	0	), Vec3(), 1);
			
			ka->frames.push_back(kf);
		}
		{
			Keyframe kf(0.5f);
			kf.next = 0;

			kf.values["l leg a 1"] = BoneInfluence(Vec3(	1,	-0.3f,	0	), Vec3(), 1);
			kf.values["l leg b 1"] = BoneInfluence(Vec3(	0,	1,		0	), Vec3(), 1);
			kf.values["l leg c 1"] = BoneInfluence(Vec3(	1,	-0.5f,	0	), Vec3(), 1);

			kf.values["r leg a 1"] = BoneInfluence(Vec3(	0,	-0.3f,	0	), Vec3(), 1);
			kf.values["r leg b 1"] = BoneInfluence(Vec3(	1,	1,		0	), Vec3(), 1);
			kf.values["r leg c 1"] = BoneInfluence(Vec3(	0,	-0.5f,	0	), Vec3(), 1);
			
			ka->frames.push_back(kf);
		}
	}

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
		hp(1.0),
		eye_bone(NULL),
		gun_hand_bone(NULL),
		model(model),
		character(NULL),
		p_adp(NULL),
		rigid_body(NULL),
		physics(NULL),
		standing(0),
		jump_start_timer(0),
		jump_fuel(1.0),
		equipped_weapon(NULL),
		intrinsic_weapon(NULL),
		OnAmmoFailure(),
		OnDamageTaken(),
		OnJumpFailure(),
		OnDeath(),
		contact_callback(NULL)
	{
		control_state = new ControlState();

		contact_callback = new MyContactResultCallback(this);

		// creating character
		character = new SkinnedCharacter(model->CreateSkeleton());
		
		// character animation stuff
		ik_pose = new IKPose(game_state, character->skeleton, pos, pitch, yaw);

		GenerateHardCodedWalkAnimation(ik_pose);
		ik_pose->AddEndEffector("l leg a 3", Vec3(	0.27f,	0,	1.29f	), true);
		ik_pose->AddEndEffector("r leg a 3", Vec3(	-0.27f,	0,	1.29f	), false);
		ik_pose->AddEndEffector("l leg b 3", Vec3(	1.98f,	0,	0.44f	), false);
		ik_pose->AddEndEffector("r leg b 3", Vec3(	-1.98f,	0,	0.44f	), true);
		ik_pose->AddEndEffector("l leg c 3", Vec3(	0.80f,	0,	-1.36f	), true);
		ik_pose->AddEndEffector("r leg c 3", Vec3(	-0.80f,	0,	-1.36f	), false);

		character->active_poses.push_back(ik_pose);

		p_adp = new PoseAimingGun();
		character->active_poses.push_back(p_adp);

		// figure out which bones are the eye and gun-holding bones
		for(vector<Bone*>::iterator iter = character->skeleton->bones.begin(); iter != character->skeleton->bones.end(); iter++)
		{
			if((*iter)->name == "eye")
				eye_bone = *iter;
			else if((*iter)->name == "r grip")
				gun_hand_bone = *iter;
		}

		// too bad this isn't saved somewhere?
		MassInfo mass_info = MassInfo();
		for(int i = -3; i <= 3; i++)
			for(int j = 0; j <= 20; j++)
				for(int k = -3; k <= 3; k++)
					mass_info += MassInfo(Vec3(i * 0.1f, j * 0.1f, k * 0.1f), 0.09523809523809523809523809523808f);
		mass = mass_info.mass;
		inverse_moi = Mat3::Invert(Mat3(mass_info.moi));

		// look up all the materials the model uses in advance
		Cache<Material>* mat_cache = ((TestGame*)gs)->mat_cache;
		for(unsigned int i = 0; i < model->materials.size(); i++)
		{
			string material_name = model->materials[i];
			DSNMaterial* mat = (DSNMaterial*)mat_cache->Load(material_name);
			materials.push_back(mat);
		}
	}

	void Dood::InnerDispose()
	{
		delete control_state;
		control_state = NULL;

		VisCleanup();

		rigid_body->Dispose();
		delete rigid_body;

		delete contact_callback;

		delete p_adp;
		p_adp = NULL;

		delete ik_pose;
		ik_pose = NULL;

		Pawn::InnerDispose();
	}

	void Dood::Update(TimingInfo time)
	{
		pos = ik_pose->pos;
		yaw = ik_pose->yaw;
		pitch = ik_pose->pitch;

		Pawn::Update(time);

		MaybeDoScriptedAI(this);

		rigid_body->body->activate();
		rigid_body->body->clearForces();

		standing = 0;
		physics->dynamics_world->contactTest(rigid_body->body, *contact_callback);

		pos = GetPosition();

		float timestep = time.elapsed;

		Vec3 forward = Vec3(-sin(yaw), 0, cos(yaw));
		Vec3 rightward = Vec3(-forward.z, 0, forward.x);

		btVector3 bt_vel = rigid_body->body->getLinearVelocity();
		Vec3 vel = Vec3((float)bt_vel.getX(), (float)bt_vel.getY(), (float)bt_vel.getZ());		// velocity prior to forces being applied

		Vec3 delta_v = vel - this->vel;
		// collision damage!
		float falling_damage_base = abs(delta_v.ComputeMagnitude()) - 10.0f;
		if (falling_damage_base > 0)
			TakeDamage(Damage(this, falling_damage_base * 0.068f), Vec3());						// zero-vector indicates damage came from self

		this->vel = vel;

		if (timestep > 0)
		{
			Vec3 post_damp_vel = vel * exp(-timestep * movement_damp);
			Vec3 damp_force = (post_damp_vel - vel) * mass / timestep;

			rigid_body->body->applyCentralForce(btVector3(damp_force.x, damp_force.y, damp_force.z));
		}

		float traction = standing * ground_traction + (1 - standing) * air_traction;

		bool can_recharge = true;
		Vec3 desired_vel = forward * (top_speed_forward * max(-1.0f, min(1.0f, GetFloatControl(this, "forward")))) + rightward * (top_speed_sideways * max(-1.0f, min(1.0f, GetFloatControl(this, "sidestep"))));

		if (timestep > 0 && desired_vel.ComputeMagnitudeSquared() > 0)
		{
			desired_vel = (vel - desired_vel) * exp(-traction * timestep * standing) + desired_vel;

			Vec3 force = (desired_vel - vel) * mass / timestep;
			rigid_body->body->applyCentralForce(btVector3(force.x, force.y, force.z));
		}

		bool jumped = false;
		if (standing > 0)
		{
			if (GetBoolControl(this, "jump"))
			{
				//jump off the ground
				rigid_body->body->applyCentralImpulse(btVector3(0, jump_speed * mass, 0));
				jump_start_timer = time.total + jump_to_fly_delay;

				jumped = true;
			}
			else if(GetBoolControl(this, "leap") && time.total > jump_start_timer)
			{
				// crab bug leaps forward
				float leap_angle = 0.4f;
				Vec3 leap_vector = (forward * (cosf(leap_angle)) + Vec3(0, sinf(leap_angle), 0)) * (mass * bug_leap_speed);
				rigid_body->body->applyCentralImpulse(btVector3(leap_vector.x, leap_vector.y, leap_vector.z));

				jump_start_timer = time.total + bug_leap_duration;

				jumped = true;
			}
		}
		else if (GetBoolControl(this, "jump"))
		{
			can_recharge = false;

			if (jump_fuel > 0)
			{
				// jetpacking
				if (time.total > jump_start_timer)
				{
					jump_fuel -= timestep * (jump_fuel_spend_rate);

					Vec3 jump_accel_vec = Vec3(0, jump_pack_accel, 0);
					Vec3 lateral_accel = forward * max(-1.0f, min(1.0f, GetFloatControl(this, "forward"))) + rightward * max(-1.0f, min(1.0f, GetFloatControl(this, "sidestep")));
					jump_accel_vec += lateral_accel * (flying_accel);

					Vec3 jump_force = jump_accel_vec * mass;

					rigid_body->body->applyCentralForce(btVector3(jump_force.x, jump_force.y, jump_force.z));
				}
			}
			else
			{
				// out of fuel! flash hud gauge if it's relevant
				JumpFailureEvent evt = JumpFailureEvent(this);
				OnJumpFailure(&evt);
			}
		}

		if (can_recharge)
			jump_fuel = min(jump_fuel + jump_fuel_refill_rate * timestep, 1.0f);

		DoPitchAndYawControls(timestep);

		// uncontrolled spinning!
		angular_vel *= exp(-(standing) * 200 * timestep);

		float angular_speed = angular_vel.ComputeMagnitude();
		if (angular_speed > 0)
		{
			float new_speed = max(0.0f, angular_speed - air_spin_fix * timestep);
			angular_vel = angular_vel * new_speed / angular_speed;

			yaw += angular_vel.y * timestep;
		}

		p_adp->pos = pos;
		p_adp->yaw = yaw;
		p_adp->pitch = pitch;

		if(GetBoolControl(this, "reload"))
		{
			if(dynamic_cast<WeaponEquip*>(equipped_weapon) != NULL)
				((WeaponEquip*)equipped_weapon)->BeginReload();

			SetBoolControl(this, "reload", false);
		}

		MaybeDoScriptedUpdate(this);

		if(character != NULL)
		{
			PoseCharacter(time);

			if(equipped_weapon != NULL)
			{
				equipped_weapon->SetFiring(1, true, GetBoolControl(this, "primary_fire"));
				equipped_weapon->OwnerUpdate(time);
			}
			if(intrinsic_weapon != NULL)
			{
				intrinsic_weapon->SetFiring(1, true, GetBoolControl(this, "primary_fire"));
				intrinsic_weapon->OwnerUpdate(time);
			}

			character->UpdatePoses(time);
		}

		ik_pose->SetDesiredState(game_state->ik_solver, pos, pitch, yaw);
	}

	void Dood::DoPitchAndYawControls(float timestep)
	{
		float desired_yaw = max(-1.0f, min(1.0f, GetFloatControl(this, "yaw")));
		float desired_pitch = max(-1.0f, min(1.0f, GetFloatControl(this, "pitch")));

		if (abs(desired_yaw) <= timestep * yaw_rate)
		{
			yaw += desired_yaw;
			SetFloatControl(this, "yaw", 0.0f);
		}
		else if (desired_yaw < 0)
		{
			yaw -= timestep * yaw_rate;
			SetFloatControl(this, "yaw", desired_yaw + timestep * yaw_rate);
		}
		else
		{
			yaw += timestep * yaw_rate;
			SetFloatControl(this, "yaw", desired_yaw - timestep * yaw_rate);
		}

		if (abs(desired_pitch) <= timestep * pitch_rate)
		{
			pitch += desired_pitch;
			SetFloatControl(this, "pitch", 0.0f);
		}
		else if (desired_pitch < 0)
		{
			pitch -= timestep * pitch_rate;
			SetFloatControl(this, "pitch", desired_pitch + timestep * pitch_rate);
		}
		else
		{
			pitch += timestep * pitch_rate;
			SetFloatControl(this, "pitch", desired_pitch - timestep * pitch_rate);
		}

		pitch = min((float)M_PI * 0.5f, max(-(float)M_PI * 0.5f, pitch));
	}

	Vec3 Dood::GetPosition()
	{
		if(rigid_body->body == NULL)
			return pos;
		else
			return rigid_body->GetPosition();
	}
	void Dood::SetPosition(Vec3 pos)
	{
		if(rigid_body->body == NULL)
			this->pos = pos;
		else
			rigid_body->SetPosition(pos);
	}

	void Dood::Vis(SceneRenderer* renderer)
	{
		VisCleanup();				// just in case

		if(character != NULL)		// character will be null right when it becomes dead
		{
			Sphere bs = Sphere(pos, 2.5);
			if(renderer->camera->CheckSphereVisibility(bs))
				((TestGame*)game_state)->VisUberModel(renderer, model, 0, Mat4::Translation(pos), character, &materials);
		}
	}

	void Dood::VisCleanup() { }

	Mat4 Dood::GetViewMatrix()
	{
		Mat4 flip = Mat4::FromQuaternion(Quaternion::FromPYR(0, M_PI, 0));

		if(eye_bone == NULL)
		{	
			Mat4 pitch_mat = Mat4::FromQuaternion(Quaternion::FromPYR(-pitch, 0, 0));
			Mat4 yaw_mat = Mat4::FromQuaternion(Quaternion::FromPYR(0, yaw, 0));
			Mat4 loc = Mat4::Translation(-pos);

			return flip * pitch_mat * yaw_mat * loc;
		}
		else
		{
			Mat4 eye_xform = eye_bone->GetTransformationMatrix();
			Vec3 pos_vec = eye_xform.TransformVec3(eye_bone->rest_pos, 1.0);
			Vec3 left = eye_xform.TransformVec3(Vec3(1, 0, 0), 0.0);
			Vec3 up = eye_xform.TransformVec3(Vec3(0, 1, 0), 0.0);
			Vec3 forward = eye_xform.TransformVec3(Vec3(0, 0, 1), 0.0);

			float rm_values[] = { left.x, left.y, left.z, up.x, up.y, up.z, forward.x, forward.y, forward.z };
			return flip * Mat4::FromMat3(Mat3(rm_values)) * Mat4::Translation(-(pos + pos_vec));
		}
	}

	void Dood::PoseCharacter() { PoseCharacter(TimingInfo(game_state->total_game_time - character_pose_time, game_state->total_game_time)); }
	void Dood::PoseCharacter(TimingInfo time)
	{
		if(character == NULL)			// the moment a Dood dies this will fail
			return;

		if (time.total > character_pose_time)
		{
			p_adp->pos = pos;
			p_adp->yaw = yaw;
			p_adp->pitch = pitch;

			character->UpdatePoses(time);

			if(equipped_weapon != NULL && gun_hand_bone != NULL)
			{
				equipped_weapon->gun_xform = Mat4::Translation(pos) * gun_hand_bone->GetTransformationMatrix() * Mat4::Translation(gun_hand_bone->rest_pos) /* * Mat4::Translation(-0.3, 1.3, 0.08) * Mat4::FromQuaternion(Quaternion::FromPYR(0, 0, -0.75) * Quaternion::FromPYR(1.5, 0.0, 0.0) * Quaternion::FromPYR(0, 0.1, 0) * Quaternion::FromPYR(0.1, 0, 0)) * Mat4::Translation(0, 0.05, 0.35) */;
				equipped_weapon->sound_pos = equipped_weapon->pos = equipped_weapon->gun_xform.TransformVec3(0, 0, 0, 1);
				equipped_weapon->sound_vel = equipped_weapon->vel = vel;
			}

			character_pose_time = time.total;
		}
	}

	void Dood::Spawned()
	{
		physics = game_state->physics_world;

		btVector3 spheres[] = { btVector3(0, 0.5f, 0), btVector3(0, 1.5f, 0) };
		float radii[] = { 0.5f, 0.5f };
		btConvexShape* shape = new btMultiSphereShape(spheres, radii, 2);

		MassInfo mass_info = MassInfo(Vec3(0, 1, 0), mass);			// point mass; has zero MoI, which Bullet treats like infinite MoI

		RigidBodyInfo* rigid_body = new RigidBodyInfo(shape, mass_info, pos);
		rigid_body->body->setUserPointer(this);

		rigid_body->body->setCollisionFlags(rigid_body->body->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);

		physics->AddRigidBody(rigid_body);
		this->rigid_body = rigid_body;
	}

	void Dood::DeSpawned()
	{
		physics->RemoveRigidBody(rigid_body);

		if(equipped_weapon != NULL)
			equipped_weapon->is_valid = NULL;
		if(intrinsic_weapon != NULL)
			intrinsic_weapon->is_valid = NULL;
	}

	bool Dood::GetShot(Shot* shot, Vec3 poi, Vec3 momentum)
	{
		Vec3 from_dir = Vec3::Normalize(shot->origin - pos);
		TakeDamage(shot->GetDamage(), from_dir);

		// let's transfer us some momentum!

		// character controllers are special cases where angular momentum doesn't get applied the same way...
		rigid_body->body->applyCentralForce(btVector3(momentum.x, momentum.y, momentum.z));

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
		BillboardMaterial* b_mat = ((TestGame*)game_state)->blood_billboard;

		for (int i = 0; i < 8; i++)
		{
			Particle* p = new Particle(game_state, poi, Random3D::RandomNormalizedVector(Random3D::Rand(5)) + momentum * Random3D::Rand(), NULL, b_mat, Random3D::Rand(0.05f, 0.15f), 0.25f);
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
			Corpse* corpse = new Corpse(game_state, this);
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
	 * Dood::MyContactResultCallback methods
	 */
	Dood::MyContactResultCallback::MyContactResultCallback(Dood* dood) : btCollisionWorld::ContactResultCallback(), dood(dood) { }
	btScalar Dood::MyContactResultCallback::addSingleResult(btManifoldPoint& cp, const btCollisionObject* colObj0, int partId0, int index0, const btCollisionObject* colObj1, int partId1, int index1)
	{
		Vec3 normal = Vec3(cp.m_normalWorldOnB.getX(), cp.m_normalWorldOnB.getY(), cp.m_normalWorldOnB.getZ());

		// player can only stand on an upward-facing surface
		if(normal.y > 0.1)
		{
			btRigidBody* self = dood->rigid_body->body;
			// player only counts as standing when they aren't moving away from the surface

			btVector3 bt_vel_0 = self->getLinearVelocity();
			Vec3 vel_0 = Vec3(bt_vel_0.getX(), bt_vel_0.getY(), bt_vel_0.getZ());

			Vec3 vel_1;

			if(colObj1->getInternalType() == btCollisionObject::CO_RIGID_BODY)
			{
				btRigidBody* other = (btRigidBody*)colObj1;
				btVector3 bt_vel_1 = other->getLinearVelocity();
				vel_1 = Vec3(bt_vel_1.getX(), bt_vel_1.getY(), bt_vel_1.getZ());
			}
			else
				vel_1 = Vec3();

			float dot = Vec3::Dot(vel_0 - vel_1, normal);
			if(dot <= 0.1)
				dood->standing = 1;
		}

		return 0;
	}




	/*
	 * Dood scripting stuff
	 */
	void GetDoodControlState(lua_State* L, Dood* dood)
	{
		PushDoodHandle(L, dood);

		lua_getmetatable(L, -1);							// push; top = 2

		// control_state is a field of the dood metatable; control_state[dood] = the control state for a particular dood
		lua_getfield(L, -1, "control_state");				// push; top = 3

		if(lua_istable(L, -1))
		{
			lua_pushvalue(L, -3);
			lua_gettable(L, -2);								// pop, push; top = 4

			// is there a control state yet? if not, create one
			if(lua_isnil(L, -1))
			{
				lua_pop(L, 1);									// pop; top = 3

				lua_pushvalue(L, -3);
				lua_newtable(L);								// push; top = 5

					// begin contents of control state table
					lua_pushnumber(L, 0);
					lua_setfield(L, -2, "forward");

					lua_pushnumber(L, 0);
					lua_setfield(L, -2, "sidestep");

					lua_pushnumber(L, 0);
					lua_setfield(L, -2, "yaw");

					lua_pushnumber(L, 0);
					lua_setfield(L, -2, "pitch");

					lua_pushboolean(L, false);
					lua_setfield(L, -2, "jump");

					lua_pushboolean(L, false);
					lua_setfield(L, -2, "primary_fire");

					lua_pushboolean(L, false);
					lua_setfield(L, -2, "reload");
					// end contents of control state table

				lua_settable(L, -3);			// pop x2; top = 3

				lua_pushvalue(L, -3);
				lua_gettable(L, -2);
			}

			lua_replace(L, -4);
			lua_settop(L, -3);
		}
		else
		{
			lua_settop(L, -3);
			lua_pushnil(L);
		}
	}

	int dood_index(lua_State* L)
	{
		Dood** dood_ptr = (Dood**)lua_touserdata(L, 1);

		if(lua_isstring(L, 2))
		{
			string key = lua_tostring(L, 2);

			lua_settop(L, 0);

			if		(key == "is_valid")					{ lua_pushboolean(L,  *dood_ptr != NULL); return 1; }
			else
			{
				Dood* dood = *dood_ptr;
				if(dood != NULL)
				{
					if		(key == "position")			{ PushLuaVector(L, dood->pos); return 1; }
					else if	(key == "is_player")			{ lua_pushboolean(L, dood == ((TestGame*)dood->game_state)->player_pawn); return 1; }
					else if	(key == "health")			{ lua_pushnumber(L, dood->hp); return 1; }			
					else if	(key == "control_state")		{ GetDoodControlState(L, dood); return 1; }
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
				if (key == "ai_callback")
				{
					if(lua_isfunction(L, 3))
					{
						lua_getmetatable(L, 1);								// push; top = 4
						lua_getfield(L, 4, "ai_callback");					// push the ai_callback table; top = 5
						lua_pushvalue(L, 1);								// push the dood; top = 6
						lua_pushvalue(L, 3);								// push the function; top = 7
						lua_settable(L, 5);									// pop x2; top = 5

						lua_settop(L, 0);
						return 0;
					}
				}
				else if (key == "update_callback")
				{
					if(lua_isfunction(L, 3))
					{
						lua_getmetatable(L, 1);								// push; top = 4
						lua_getfield(L, 4, "update_callback");				// push the ai_callback table; top = 5
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
						lua_getfield(L, 4, "death_callback");				// push the ai_callback table; top = 5
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
			lua_setfield(L, -2, "ai_callback");

			lua_newtable(L);
			lua_setfield(L, -2, "update_callback");
			
			lua_newtable(L);
			lua_setfield(L, -2, "death_callback");

			lua_newtable(L);
			lua_setfield(L, -2, "control_state");

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

	// return value = success or failure; success pushes 1 onto stack; failure doesn't affect stack
	bool PushControl(lua_State* L, Dood* dood, string control_name)
	{
		GetDoodControlState(L, dood);						// top = 1

		if(lua_istable(L, -1))
		{
			lua_getfield(L, -1, control_name.c_str());		// top = 2
			if(lua_isnil(L, -1))
			{
				lua_settop(L, -2);							// top = 0
				return false;
			}
			else
			{
				lua_replace(L, -2);							// top = 1
				return true;
			}
		}
		else
			return false;
	}

	bool GetBoolControl(Dood* dood, string control_name)
	{
		lua_State* L = ScriptSystem::GetGlobalState().GetLuaState();
		lua_settop(L, 0);

		if(PushControl(L, dood, control_name))
			if(lua_isboolean(L, 1))
				return lua_toboolean(L, 1) != 0;

		return false;
	}

	float GetFloatControl(Dood* dood, string control_name)
	{
		lua_State* L = ScriptSystem::GetGlobalState().GetLuaState();
		lua_settop(L, 0);

		if(PushControl(L, dood, control_name))
			if(lua_isnumber(L, 1))
				return (float)lua_tonumber(L, 1);

		return 0.0f;
	}

	void SetBoolControl(Dood* dood, string control_name, bool value)
	{
		lua_State* L = ScriptSystem::GetGlobalState().GetLuaState();
		lua_settop(L, 0);

		GetDoodControlState(L, dood);

		if(lua_istable(L, -1))
		{
			lua_pushboolean(L, value);
			lua_setfield(L, -2, control_name.c_str());
		}
	}

	void SetFloatControl(Dood* dood, string control_name, float value)
	{
		lua_State* L = ScriptSystem::GetGlobalState().GetLuaState();
		lua_settop(L, 0);

		GetDoodControlState(L, dood);

		if(lua_istable(L, -1))
		{
			lua_pushnumber(L, value);
			lua_setfield(L, -2, control_name.c_str());
		}
	}




	/*
	 * If this Dood has an AI callback associated with it via Lua-scripting, call that callback
	 */
	bool MaybeDoScriptedAI(Dood* dood)
	{
		lua_State* L = ScriptSystem::GetGlobalState().GetLuaState();

		lua_settop(L, 0);
		PushDoodHandle(L, dood);

		lua_getmetatable(L, 1);					// push; top = 2
		if(!lua_isnil(L, 2))
		{
			lua_getfield(L, 2, "ai_callback");	// pop+push; top = 3
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
