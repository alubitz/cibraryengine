#include "StdAfx.h"
#include "Corpse.h"
#include "Shot.h"
#include "TestGame.h"
#include "Particle.h"

namespace Test
{
	float active_lifetime = 5.0f;
	bool allow_become_permanent = false;
	
	
	

	// Each bone is a separate shootable object
	struct CorpseBoneShootable : Entity, Shootable
	{
		Corpse* corpse;
		RigidBodyInfo* rbi;

		CorpseBoneShootable(GameState* gs, Corpse* corpse, RigidBodyInfo* rbi) : Entity(gs), corpse(corpse), rbi(rbi) { }

		bool GetShot(Shot* shot, Vec3 poi, Vec3 momentum)
		{
			BillboardMaterial* b_mat = ((TestGame*)corpse->game_state)->blood_billboard;

			for (int i = 0; i < 8; ++i)
			{
				Particle* p = new Particle(corpse->game_state, poi, Random3D::RandomNormalizedVector(Random3D::Rand(5)) + momentum * Random3D::Rand(), NULL, b_mat, Random3D::Rand(0.05f, 0.15f), 0.25f);
				p->gravity = 9.8f;
				p->damp = 0.05f;

				corpse->game_state->Spawn(p);
			}

			Mat4 xform;
			{
				xform = rbi->GetTransformationMatrix();
				float ori_values[] = {xform[0], xform[1], xform[2], xform[4], xform[5], xform[6], xform[8], xform[9], xform[10]};
				Quaternion rigid_body_ori = Quaternion::FromRotationMatrix(Mat3(ori_values).Transpose());
				Vec3 pos = xform.TransformVec3(0, 0, 0, 1);
				xform = Mat4::FromPositionAndOrientation(pos, rigid_body_ori.ToMat3().Transpose());
			}

			Vec3 pos = xform.TransformVec3(0, 0, 0, 1);
			Vec3 x_axis = xform.TransformVec3(1, 0, 0, 0);
			Vec3 y_axis = xform.TransformVec3(0, 1, 0, 0);
			Vec3 z_axis = xform.TransformVec3(0, 0, 1, 0);

			Vec3 local_poi;
			local_poi = poi - pos;
			local_poi = Vec3(Vec3::Dot(local_poi, x_axis), Vec3::Dot(local_poi, y_axis), Vec3::Dot(local_poi, z_axis));
			local_poi = local_poi.x * x_axis + local_poi.y * y_axis + local_poi.z * z_axis;

			rbi->body->activate();
			rbi->body->applyImpulse(btVector3(momentum.x, momentum.y, momentum.z), btVector3(local_poi.x, local_poi.y, local_poi.z));
			return true;
		}
	};




	/*
	 * Corpse implementation; private variables and methods
	 */
	struct Corpse::Imp
	{
		Corpse* corpse;

		vector<Material*> materials;
		SkinnedCharacter* character;
		UberModel* model;

		Mat4 whole_xform;
		Vec3 origin;

		Vec3 initial_vel;

		float character_pose_time;

		float fizzle_time;
		bool immortal;

		PhysicsWorld* physics;
		vector<RigidBodyInfo*> rigid_bodies;
		vector<CorpseBoneShootable*> shootables;
		vector<Vec3> bone_offsets;
		vector<btTypedConstraint*> constraints;

		// constructor with big long initializer list
		Imp(Corpse* corpse, GameState* gs, Dood* dood) : 
			corpse(corpse),
			character(dood->character),
			model(dood->model),
			whole_xform(Mat4::Translation(dood->pos + Vec3(0, 0.2f, 0))),
			origin(dood->pos),
			initial_vel(dood->vel),
			character_pose_time(-1),
			fizzle_time(gs->total_game_time + active_lifetime),
			immortal(false),
			physics(NULL),
			rigid_bodies(),
			bone_offsets(),
			constraints()
		{
			character->active_poses.clear();
			dood->character = NULL;

			for(unsigned int i = 0; i < model->materials.size(); ++i)
			{
				string material_name = model->materials[i];
				DSNMaterial* mat = (DSNMaterial*)((TestGame*)gs)->mat_cache->Load(material_name);
				materials.push_back(mat);
			}
		}

		void Dispose()
		{
			DeSpawned();

			for(unsigned int i = 0; i < shootables.size(); ++i)
				delete shootables[i];
			shootables.clear();

			character->Dispose();
			delete character;
			character = NULL;
		}

		// position of the drawn bones
		void PoseCharacter() { float now = corpse->game_state->total_game_time; PoseCharacter(TimingInfo(now - character_pose_time, now)); }
		void PoseCharacter(TimingInfo time)
		{
			if(immortal || !corpse->is_valid)
				return;
			float now = time.total;
			if (now > character_pose_time)
			{
				origin = rigid_bodies[0]->GetTransformationMatrix().TransformVec3(0, 0, 0, 1);

				for(unsigned int i = 0; i < rigid_bodies.size(); ++i)
				{
					RigidBodyInfo* body = rigid_bodies[i];
					Bone* bone = character->skeleton->bones[i];

					Mat4 mat = body->GetTransformationMatrix();

					float ori_values[] = {mat[0], mat[1], mat[2], mat[4], mat[5], mat[6], mat[8], mat[9], mat[10]};
					Quaternion rigid_body_ori = Quaternion::FromRotationMatrix(Mat3(ori_values).Transpose());

					Vec3 rigid_body_pos = mat.TransformVec3(0, 0, 0, 1);

					bone->ori = rigid_body_ori;

					// model origin = rigid body pos - model rot * rest pos
					Vec3 offset = Mat4::FromQuaternion(rigid_body_ori).TransformVec3(bone_offsets[i], 1);
					bone->pos = rigid_body_pos - offset - origin;			//subtract origin to account for that whole-model transform in Corpse::Imp::Vis
				}

				character->UpdatePoses(TimingInfo(character_pose_time >= 0 ? now - character_pose_time : 0, now));

				character_pose_time = now;
			}
		}

		void Spawned()
		{
			physics = corpse->game_state->physics_world;

			// get bone pos/ori info
			vector<Mat4> mats = vector<Mat4>();
			unsigned int count = character->skeleton->bones.size();
			for(unsigned int i = 0; i < count; ++i)	
			{
				Bone* bone = character->skeleton->bones[i];

				bone_offsets.push_back(bone->rest_pos);
				mats.push_back(whole_xform * bone->GetTransformationMatrix());
			}

			UberModel::BonePhysics** bone_physes = new UberModel::BonePhysics* [count];

			float total_mass = 0;

			// create rigid bodies
			for(unsigned int i = 0; i < count; ++i)
			{
				Bone* bone = character->skeleton->bones[i];

				Mat4 mat = mats[i];
				float ori_values[] = {mat[0], mat[1], mat[2], mat[4], mat[5], mat[6], mat[8], mat[9], mat[10]};
				bone->ori = Quaternion::FromRotationMatrix(Mat3(ori_values));

				Vec3 bone_pos = mat.TransformVec3(bone_offsets[i], 1);

				UberModel::BonePhysics* phys = NULL;
				for(unsigned int j = 0; j < model->bone_physics.size(); ++j)
					if(Bone::string_table[model->bone_physics[j].bone_name] == bone->name)
						phys = &model->bone_physics[j];

				bone_physes[i] = phys;

				if(phys != NULL)
				{
					btCollisionShape* shape = phys->shape;
					if(shape != NULL)
					{
						btVector3 local_inertia;
						shape->calculateLocalInertia(phys->mass, local_inertia);

						MassInfo mass_info;
						mass_info.mass = phys->mass;
						total_mass += mass_info.mass;

						mass_info.moi[0] = local_inertia.getX();
						mass_info.moi[4] = local_inertia.getY();
						mass_info.moi[8] = local_inertia.getZ();

						RigidBodyInfo* rigid_body = new RigidBodyInfo(shape, mass_info, bone_pos, bone->ori);
						rigid_body->body->setLinearVelocity(btVector3(initial_vel.x, initial_vel.y, initial_vel.z));
						
						CorpseBoneShootable* shootable = new CorpseBoneShootable(corpse->game_state, corpse, rigid_body);
						shootables.push_back(shootable);
						rigid_body->body->setUserPointer(shootable);

						// these constants taken from the ragdoll demo
						rigid_body->body->setDamping(0.05f, 0.85f);
						rigid_body->body->setDeactivationTime(0.8f);
						rigid_body->body->setSleepingThresholds(1.6f, 2.5f);

						rigid_body->body->setFriction(1.0f);
						rigid_body->body->setRestitution(0.01f);

						physics->AddRigidBody(rigid_body);
						rigid_bodies.push_back(rigid_body);
					}
				}
			}

			// create constraints between bones
			for(unsigned int i = 0; i < count; ++i)
			{
				UberModel::BonePhysics* phys = bone_physes[i];
				if(phys != NULL)
				{
					Bone* bone = character->skeleton->bones[i];
					Bone* parent = bone->parent;

					if(parent != NULL)
					{
						// find index of parent (bone's index is the same as rigid body info's index)
						for(unsigned int j = 0; j < count; ++j)
						{
							if(character->skeleton->bones[j] == parent)
							{
								RigidBodyInfo* my_body = rigid_bodies[i];
								RigidBodyInfo* parent_body = rigid_bodies[j];

								const btTransform a_frame = btTransform(btQuaternion::getIdentity(), btVector3(0, 0, 0));
								const btTransform b_frame = btTransform(btQuaternion(phys->ori.x, phys->ori.y, phys->ori.z, phys->ori.w), btVector3(phys->pos.x, phys->pos.y, phys->pos.z));

								btConeTwistConstraint* c = new btConeTwistConstraint(*my_body->body, *parent_body->body, a_frame, b_frame);
								c->setLimit(phys->span.x, phys->span.y, phys->span.z);
								c->setDamping(0.1f);			// default is 0.01
								constraints.push_back(c);

								physics->dynamics_world->addConstraint(c, true);			// true = prevent them from colliding normally

								break;
							}
						}
					}
				}
			}

			// emancipate bones
			for(unsigned int i = 0; i < count; ++i)
			{
				Bone* bone = character->skeleton->bones[i];
				bone->parent = NULL;									// orientation and position are no longer relative to a parent!
			}

			if(rigid_bodies.size() == 0)
				corpse->is_valid = false;

			delete[] bone_physes;
		}

		void DeSpawned()
		{
			// clear constraints
			for(unsigned int i = 0; i < constraints.size(); ++i)
			{
				btTypedConstraint* c = constraints[i];
				physics->dynamics_world->removeConstraint(c);

				delete c;
			}
			constraints.clear();

			// clear rigid bodies
			for(unsigned int i = 0; i < rigid_bodies.size(); ++i)
			{
				RigidBodyInfo* body = rigid_bodies[i];
				if(physics != NULL)
					physics->RemoveRigidBody(body);

				body->DisposePreservingCollisionShape();
				delete body;
			}
			rigid_bodies.clear();
		}

		void Update(TimingInfo time)
		{
			if(immortal)
				return;
			else if(time.total > fizzle_time)
				corpse->is_valid = false;
			else if(allow_become_permanent)
			{
				unsigned int i;
				unsigned int n = rigid_bodies.size();
				for(i = 0; i < n; ++i)
				{
					RigidBodyInfo* r = rigid_bodies[i];
					if(r->body->getActivationState() != ISLAND_SLEEPING)
						break;
				}
				if(i == n)
				{
					for(i = 0; i < n; ++i)
						rigid_bodies[i]->body->setCollisionFlags(btCollisionObject::CF_STATIC_OBJECT | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
					// clear constraints
					for(i = 0; i < constraints.size(); ++i)
					{
						btTypedConstraint* c = constraints[i];
						physics->dynamics_world->removeConstraint(c);

						delete c;
					}
					constraints.clear();

					immortal = true;
				}
			}
		}

		void Vis(SceneRenderer* renderer)
		{
			PoseCharacter();

			Vec3 pos = origin;
			Sphere bs = Sphere(pos, 2.5);

			if(renderer->camera->CheckSphereVisibility(bs))
				((TestGame*)corpse->game_state)->VisUberModel(renderer, model, 0, Mat4::Translation(pos), character, &materials);
		}
	};




	/*
	 * Corpse methods
	 */
	Corpse::Corpse(GameState* gs, Dood* dood) : Entity(gs), imp(new Imp(this, gs, dood)) { }
	void Corpse::InnerDispose() { imp->Dispose(); delete imp; imp = NULL; Entity::InnerDispose(); }
	void Corpse::Spawned() { imp->Spawned(); }
	void Corpse::DeSpawned() { imp->DeSpawned(); }
	void Corpse::Update(TimingInfo time) { imp->Update(time); }
	void Corpse::Vis(SceneRenderer* renderer) { imp->Vis(renderer); }
	Vec3 Corpse::GetPosition() { return imp->origin; }
}
