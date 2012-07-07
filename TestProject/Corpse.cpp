#include "StdAfx.h"
#include "Corpse.h"
#include "Shot.h"
#include "TestGame.h"
#include "Particle.h"

namespace Test
{
	// Each bone is a separate shootable object (and also an Entity for technical reasons, but it's never spawned)
	struct CorpseBoneShootable : Entity, Shootable
	{
		Corpse* corpse;
		RigidBody* rbi;

		BillboardMaterial* blood_material;

		CorpseBoneShootable(GameState* gs, Corpse* corpse, RigidBody* rbi, BillboardMaterial* blood_material) : Entity(gs), corpse(corpse), rbi(rbi), blood_material(blood_material) { }
		~CorpseBoneShootable() { }

		bool GetShot(Shot* shot, Vec3 poi, Vec3 momentum)
		{
			if(blood_material != NULL)
				for (int i = 0; i < 8; ++i)
				{
					Particle* p = new Particle(corpse->game_state, poi, Random3D::RandomNormalizedVector(Random3D::Rand(5)) + momentum * Random3D::Rand(), NULL, blood_material, Random3D::Rand(0.05f, 0.15f), 0.25f);
					p->gravity = 9.8f;
					p->damp = 0.05f;

					corpse->game_state->Spawn(p);
				}

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
	};




	/*
	 * Corpse implementation; private variables and methods
	 */
	struct Corpse::Imp
	{
		Corpse* corpse;

		vector<Material*> materials;
		SkinnedCharacter* draw_phys_character;
		PosedCharacter* pose_character;
		UberModel* model;
		ModelPhysics* mphys;

		BillboardMaterial* blood_material;

		Mat4 whole_xform;
		Vec3 origin;

		Vec3 initial_vel;

		float character_pose_time;

		float fizzle_time;

		PhysicsWorld* physics;
		vector<RigidBody*> rigid_bodies;
		vector<CorpseBoneShootable*> shootables;
		vector<Vec3> bone_offsets;
		vector<unsigned int> bone_indices;
		vector<PhysicsConstraint*> constraints;

		// constructor with big long initializer list
		Imp(Corpse* corpse, GameState* gs, Dood* dood, float ttl) : 
			corpse(corpse),
			draw_phys_character(dood->character),
			pose_character(dood->posey),
			model(dood->model),
			mphys(dood->mphys),
			blood_material(dood->blood_material),
			whole_xform(Mat4::Translation(dood->pos)),
			origin(dood->pos),
			initial_vel(dood->vel),
			character_pose_time(-1),
			fizzle_time(gs->total_game_time + ttl),
			physics(NULL),
			rigid_bodies(),
			shootables(),
			bone_offsets(),
			bone_indices(),
			constraints()
		{
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

			for(list<Pose*>::iterator iter = pose_character->active_poses.begin(); iter != pose_character->active_poses.end(); ++iter)
				delete *iter;
			pose_character->active_poses.clear();

			pose_character->Dispose();
			delete pose_character;
			pose_character = NULL;

			draw_phys_character->Dispose();
			delete draw_phys_character;
			draw_phys_character = NULL;
		}

		// position of the drawn bones
		void PoseCharacter() { float now = corpse->game_state->total_game_time; PoseCharacter(TimingInfo(now - character_pose_time, now)); }
		void PoseCharacter(TimingInfo time)
		{
			if(!corpse->is_valid)
				return;
			float now = time.total;
			if (now > character_pose_time)
			{
				origin = rigid_bodies[0]->GetPosition();

				for(unsigned int i = 0; i < rigid_bodies.size(); ++i)
				{
					RigidBody* body = rigid_bodies[i];
					unsigned int bone_index = bone_indices[i];
					Bone* bone = draw_phys_character->skeleton->bones[bone_index];

					Quaternion rigid_body_ori = body->GetOrientation();
					Vec3 rigid_body_pos = body->GetPosition();

					bone->ori = Quaternion::Reverse(rigid_body_ori);

					// model origin = rigid body pos - model rot * rest pos
					Vec3 offset = Mat4::FromQuaternion(bone->ori).TransformVec3_1(bone_offsets[bone_index]);
					bone->pos = rigid_body_pos - offset - origin;			//subtract origin to account for that whole-model transform in Corpse::Imp::Vis
				}

				draw_phys_character->render_info.Invalidate();
				pose_character->UpdatePoses(TimingInfo(character_pose_time >= 0 ? now - character_pose_time : 0, now));

				character_pose_time = now;
			}
		}

		void Spawned()
		{
			if(mphys == NULL)
			{
				corpse->is_valid = false;
				return;
			}

			physics = corpse->game_state->physics_world;

			// get bone pos/ori info
			vector<Mat4> mats = vector<Mat4>();
			unsigned int count = draw_phys_character->skeleton->bones.size();
			for(unsigned int i = 0; i < count; ++i)
			{
				Bone* bone = draw_phys_character->skeleton->bones[i];

				bone_offsets.push_back(Vec3());		//bone->rest_pos);
				mats.push_back(whole_xform * bone->GetTransformationMatrix());
			}

			ModelPhysics::BonePhysics** bone_physes = new ModelPhysics::BonePhysics* [count];

			// given string id, get index of rigid body
			map<unsigned int, unsigned int> name_indices;

			// create rigid bodies
			for(unsigned int i = 0; i < count; ++i)
			{
				Bone* bone = draw_phys_character->skeleton->bones[i];

				Mat4 mat = mats[i];
				float ori_values[] = {mat[0], mat[1], mat[2], mat[4], mat[5], mat[6], mat[8], mat[9], mat[10]};
				bone->ori = Quaternion::Reverse(Quaternion::FromRotationMatrix(Mat3(ori_values)));

				Vec3 bone_pos = mat.TransformVec3_1(bone_offsets[i]);

				ModelPhysics::BonePhysics* phys = NULL;
				for(unsigned int j = 0; j < mphys->bones.size(); ++j)
					if(Bone::string_table[mphys->bones[j].bone_name] == bone->name)
					{
						phys = &mphys->bones[j];
						break;
					}

				bone_physes[i] = phys;

				if(phys != NULL)
				{
					CollisionShape* shape = phys->collision_shape;
					if(shape != NULL)
					{
						RigidBody* rigid_body = new RigidBody(shape, phys->mass_info, bone_pos, bone->ori);

						rigid_body->SetLinearVelocity(initial_vel);

						rigid_body->SetDamp(0.05f);
						rigid_body->SetFriction(1.0f);

						physics->AddRigidBody(rigid_body);
						rigid_bodies.push_back(rigid_body);

						CorpseBoneShootable* shootable = new CorpseBoneShootable(corpse->game_state, corpse, rigid_body, blood_material);
						rigid_body->SetUserEntity(shootable);

						name_indices[bone->name] = bone_indices.size();

						shootables.push_back(shootable);
						bone_indices.push_back(i);
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
					c->SetDesiredOrientation(Quaternion::Invert(bone_a->GetOrientation()) * bone_b->GetOrientation());

					constraints.push_back(c);

					physics->AddConstraint(c);
				}
			}

			// emancipate bones
			for(unsigned int i = 0; i < count; ++i)
			{
				Bone* bone = draw_phys_character->skeleton->bones[i];
				bone->parent = NULL;									// orientation and position are no longer relative to a parent!
			}

			if(rigid_bodies.size() == 0)
				corpse->is_valid = false;

			delete[] bone_physes;
		}

		void DeSpawned()
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
		}

		void Update(TimingInfo time)
		{
			if(time.total > fizzle_time)
				corpse->is_valid = false;
		}

		void Vis(SceneRenderer* renderer)
		{
			PoseCharacter();

			Vec3 pos = origin;
			Sphere bs = Sphere(pos, 2.5);

			//if(renderer->camera->CheckSphereVisibility(bs))
			{
				SkinnedCharacter::RenderInfo render_info = draw_phys_character->GetRenderInfo();
				((TestGame*)corpse->game_state)->VisUberModel(renderer, model, 0, Mat4::Translation(pos), &render_info, &materials);
			}
		}
	};




	/*
	 * Corpse methods
	 */
	Corpse::Corpse(GameState* gs, Dood* dood, float ttl) : Entity(gs), imp(new Imp(this, gs, dood, ttl)) { }
	void Corpse::InnerDispose() { imp->Dispose(); delete imp; imp = NULL; Entity::InnerDispose(); }
	void Corpse::Spawned() { imp->Spawned(); }
	void Corpse::DeSpawned() { imp->DeSpawned(); }
	void Corpse::Update(TimingInfo time) { imp->Update(time); }
	void Corpse::Vis(SceneRenderer* renderer) { imp->Vis(renderer); }
	Vec3 Corpse::GetPosition() { return imp->origin; }
}
