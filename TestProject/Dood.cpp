#include "StdAfx.h"
#include "Dood.h"
#include "TestGame.h"

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

		body->ApplyWorldImpulse(momentum, poi);

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
		rbody_to_posey(),
		bone_to_rbody(),
		constraints(),
		foot_bones(),
		physics(NULL),
		mphys(mphys),
		equipped_weapon(NULL),
		intrinsic_weapon(NULL),
		OnAmmoFailure(),
		OnDamageTaken(),
		OnJumpFailure(),
		OnDeath()
	{
		standing_callback.dood = this;

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
		float standing = standing_callback.standing;

		float timestep = time.elapsed;
		float traction = standing * ground_traction + (1 - standing) * air_traction;

		Vec3 desired_vel = forward * (top_speed_forward * max(-1.0f, min(1.0f, control_state->GetFloatControl("forward")))) + rightward * (top_speed_sideways * max(-1.0f, min(1.0f, control_state->GetFloatControl("sidestep"))));

		if(timestep > 0 && desired_vel.ComputeMagnitudeSquared() > 0)
		{
			if(timestep < 0.01f)			// bit of hackery here to make stuff work properly with extremely high framerates
				timestep = 0.01f;

			desired_vel = (vel - desired_vel) * exp(-traction * timestep * standing) + desired_vel;
			Vec3 dv = desired_vel - vel;

			standing_callback.ApplyVelocityChange(dv);
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

	RigidBody* Dood::RigidBodyForNamedBone(const string& name)
	{
		unsigned int id = Bone::string_table[name];

		for(unsigned int i = 0; i < character->skeleton->bones.size(); ++i)
			if(character->skeleton->bones[i]->name == id)
				return bone_to_rbody[i];

		return NULL;
	}

	void Dood::Update(TimingInfo time)
	{
		Pawn::Update(time);

		pos = GetPosition();

		float timestep = time.elapsed;

		Vec3 forward = Vec3(-sinf(yaw), 0, cosf(yaw));
		Vec3 rightward = Vec3(-forward.z, 0, forward.x);

		// prevent bones from falling asleep (not yet possible, but whatever)
		for(vector<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
			(*iter)->Activate();

		Vec3 vel = root_rigid_body->GetLinearVelocity();										// velocity prior to forces being applied

		// TODO: make this work again sometime
		// collision damage!
		//Vec3 delta_v = vel - this->vel;
		//float falling_damage_base = abs(delta_v.ComputeMagnitude()) - 10.0f;
		//if (falling_damage_base > 0)
		//	TakeDamage(Damage(this, falling_damage_base * 0.068f), Vec3());						// zero-vector indicates damage came from self

		this->vel = vel;



		DoMovementControls(time, forward, rightward);
		DoJumpControls(time, forward, rightward);
		DoPitchAndYawControls(time);

		MaybeDoScriptedUpdate(this);

		DoWeaponControls(time);

		standing_callback.Reset();
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

				SkinnedCharacterRenderInfo render_info = character->GetRenderInfo();
				model->Vis(renderer, use_lod, Mat4::Translation(pos), &render_info, &materials);
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
			float third_person_distance = 5.0f;
			
			Mat4 eye_xform = eye_bone->GetTransformationMatrix();
#if 1
			Mat4 ori_derp = Mat4::FromQuaternion(Quaternion::FromPYR(0, -yaw, 0) * Quaternion::FromPYR(pitch, 0, 0));
#else
			Mat4& ori_derp = eye_xform;
#endif

			Vec3 pos_vec	= eye_xform.TransformVec3_1(eye_bone->rest_pos);
			Vec3 left		= ori_derp.TransformVec3_0(1, 0, 0);
			Vec3 up			= ori_derp.TransformVec3_0(0, 1, 0);
			Vec3 backward	= ori_derp.TransformVec3_0(0, 0, 1);

			Mat3 rm(left.x, left.y, left.z, up.x, up.y, up.z, backward.x, backward.y, backward.z);
			return flip * Mat4::FromMat3(rm) * Mat4::Translation(-(pos + pos_vec - backward * third_person_distance));
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

	void Dood::UpdateIKChain(IKChain* chain)
	{
		for(vector<IKChain::ChainNode>::iterator iter = chain->bones.begin(); iter != chain->bones.end(); ++iter)
		{
			IKChain::ChainNode& node = *iter;

			// get the bones from the character's skeleton which correspond to these bones from posey's skeleton
			Bone* parent = character->skeleton->GetNamedBone(node.from->name);
			Bone* child = character->skeleton->GetNamedBone(node.to->name);

			if(node.from == node.child)
				swap(parent, child);

			Mat4 relative = Mat4::Invert(child->GetTransformationMatrix()) * parent->GetTransformationMatrix();
			Vec3 junk;
			relative.Decompose(junk, node.ori);
		}
	}

	void Dood::PoseCharacter() { PoseCharacter(TimingInfo(game_state->total_game_time - character_pose_time, game_state->total_game_time)); }
	void Dood::PoseCharacter(TimingInfo time)
	{
		if(!is_valid)
			return;

		float now = time.total;
		if (now > character_pose_time)
		{
			float timestep = character_pose_time >= 0 ? now - character_pose_time : 0;

			origin = rigid_bodies[0]->GetPosition();

			for(unsigned int i = 0; i < bone_to_rbody.size(); ++i)
			{
				Bone* bone = character->skeleton->bones[i];
				if(RigidBody* body = bone_to_rbody[i])
				{
					Quaternion rigid_body_ori = body->GetOrientation();
					Vec3 rigid_body_pos = body->GetPosition();

					bone->ori = Quaternion::Reverse(rigid_body_ori);

					// model origin = rigid body pos - model rot * rest pos
					Vec3 offset = Mat4::FromQuaternion(bone->ori).TransformVec3_1(0, 0, 0);
					bone->pos = rigid_body_pos - offset - origin;			//subtract origin to account for that whole-model transform in Dood::Vis
				}
			}

			character->render_info.Invalidate();
			character->skeleton->InvalidateCachedBoneXforms();

			PreUpdatePoses(time);
			posey->UpdatePoses(TimingInfo(timestep, now));

			Vec3 net_vel;
			Vec3 com, rest_com;					// rest_com = where the com would be if the dood were in this pose at the origin
			float net_mass = 0.0f;
			for(unsigned int i = 0; i < rigid_bodies.size(); ++i)
			{
				RigidBody* body = rigid_bodies[i];

				float mass = body->GetMass();
				net_vel += body->GetLinearVelocity() * mass;
				com += body->GetCenterOfMass() * mass;
				rest_com += rbody_to_posey[i]->GetTransformationMatrix().TransformVec3_1(body->GetMassInfo().com) * mass;
				net_mass += mass;
			}
			net_vel /= net_mass;
			com /= net_mass;
			rest_com /= net_mass;

			// make bones conform to pose
			if(hp > 0)
				for(unsigned int i = 0; i < rigid_bodies.size(); ++i)
				{
					RigidBody* body = rigid_bodies[i];
					MassInfo mass_info = body->GetMassInfo();
					float mass = mass_info.mass;

					Bone* bone = rbody_to_posey[i];

					Mat4 bone_xform = bone->GetTransformationMatrix();
					Vec3 dummy;
					Quaternion bone_ori;
					bone_xform.Decompose(dummy, bone_ori);

					Vec3 bone_pos = bone_xform.TransformVec3_1(mass_info.com) + com - rest_com;

					float time_coeff = 10.0f;							// TODO: figure out a correct formula for this

					Vec3 nu_v = net_vel + (bone_pos - body->GetCenterOfMass()) * time_coeff;
					Vec3 dv = nu_v - body->GetLinearVelocity();
					body->ApplyCentralImpulse(dv * mass);

					Vec3 nu_av = (Quaternion::Reverse(body->GetOrientation()) * bone_ori).ToPYR() * time_coeff;
					Vec3 d_av = nu_av - body->GetAngularVelocity();
					body->ApplyAngularImpulse(Mat3(body->GetTransformedMassInfo().moi) * d_av);
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

		// given string id, get index of rigid body
		map<unsigned int, unsigned int> name_indices;			

		// create rigid bodies and shootables
		for(unsigned int i = 0; i < count; ++i)
		{
			ModelPhysics::BonePhysics& phys = mphys->bones[i];

			CollisionShape* shape = phys.collision_shape;
			if(shape != NULL)
			{
				RigidBody* rigid_body = new RigidBody(shape, phys.mass_info, pos);

				rigid_body->SetDamp(0.05f);

				unsigned int bone_name = Bone::string_table[phys.bone_name];
				name_indices[bone_name] = rigid_bodies.size();

				for(vector<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
					rigid_body->SetCollisionEnabled(*iter, false);		// disables collisions both ways

				physics->AddRigidBody(rigid_body);
				rigid_bodies.push_back(rigid_body);

				BoneShootable* shootable = new BoneShootable(game_state, this, rigid_body, blood_material);
				rigid_body->SetUserEntity(shootable);

				shootables.push_back(shootable);

				rbody_to_posey.push_back(posey->skeleton->GetNamedBone(bone_name));

				// give feet a collision callback
				for(map<unsigned int, RigidBody*>::iterator iter = foot_bones.begin(); iter != foot_bones.end(); ++iter)
					if(bone_name == iter->first)
					{
						iter->second = rigid_body;
						rigid_body->SetCollisionCallback(&standing_callback);

						break;
					}

				if(bone_name == Bone::string_table["carapace"] || bone_name == Bone::string_table["pelvis"])
					root_rigid_body = rigid_body;
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

				JointConstraint* c = new JointConstraint(bone_b, bone_a, phys.pos, phys.axes, phys.min_extents, phys.max_extents, phys.angular_damp);
				//c->enable_motor = true;

				constraints.push_back(c);

				physics->AddConstraint(c);
			}
		}

		// populate bone-to-rigid-body table, and emancipate bones which have rigid bodies
		for(vector<Bone*>::iterator iter = character->skeleton->bones.begin(); iter != character->skeleton->bones.end(); ++iter)
		{
			Bone* bone = *iter;

			map<unsigned int, unsigned int>::iterator found = name_indices.find(bone->name);
			if(found != name_indices.end())
			{
				bone->parent = NULL;									// orientation and position are no longer relative to a parent!
				bone_to_rbody.push_back(rigid_bodies[found->second]);
			}
			else
				bone_to_rbody.push_back(NULL);
		}

		if(rigid_bodies.size() == 0)
		{
			Debug("Dood has no rigid bodies; this Dood will be removed!\n");
			is_valid = false;
		}
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

		for(vector<PhysicsConstraint*>::iterator iter = constraints.begin(); iter != constraints.end(); ++iter)
			if(JointConstraint* jc = dynamic_cast<JointConstraint*>(*iter))
				jc->enable_motor = false;

		for(vector<RigidBody*>::iterator iter = rigid_bodies.begin(); iter != rigid_bodies.end(); ++iter)
			for(vector<RigidBody*>::iterator jter = iter; jter != rigid_bodies.end(); ++jter)
				if(iter != jter)
					(*iter)->SetCollisionEnabled(*jter, true);

		if(this != ((TestGame*)game_state)->player_pawn)
			controller->is_valid = false;
		//is_valid = false;
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
	 * Dood::StandingCallback methods
	 */
	bool Dood::StandingCallback::OnCollision(const ContactPoint& collision)
	{
		for(vector<RigidBody*>::iterator iter = dood->rigid_bodies.begin(); iter != dood->rigid_bodies.end(); ++iter)
			if(collision.obj_a == *iter || collision.obj_b == *iter)
			{
				ContactPoint::Part self = collision.obj_a == *iter ? collision.a : collision.b;
				ContactPoint::Part other = collision.obj_a == *iter ? collision.b : collision.a;

				for(vector<RigidBody*>::iterator jter = dood->rigid_bodies.begin(); jter != dood->rigid_bodies.end(); ++jter)
					if(iter != jter && (collision.obj_a == *jter || collision.obj_b == *jter))
						return true;			// collisions between bones of the same dood don't count as "standing"

				Vec3 normal = other.norm;
				if(normal.y > 0.1f)
				{
					standing_on.push_back(collision.obj_a == *iter ? collision.obj_b : collision.obj_a);
					standing = 1.0f;
				}

				return true;
			}

		return true;
	}

	void Dood::StandingCallback::Reset() { standing_on.clear(); standing = 0; }

	void Dood::StandingCallback::ApplyVelocityChange(const Vec3& dv)
	{
		for(vector<RigidBody*>::iterator iter = dood->rigid_bodies.begin(); iter != dood->rigid_bodies.end(); ++iter)
			(*iter)->ApplyCentralImpulse(dv * (*iter)->GetMassInfo().mass);

		// TODO: apply opposite impulse to what we're standing on
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
