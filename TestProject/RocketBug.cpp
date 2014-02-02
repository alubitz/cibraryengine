#include "StdAfx.h"
#include "RocketBug.h"

#include "TestGame.h"

namespace Test
{
	/*
	 * RocketBug methods
	 */
	RocketBug::RocketBug(GameState* game_state, Dood* firer, UberModel* model, ModelPhysics* mphys, const Vec3& pos, const Vec3& vel, const Quaternion& ori) :
		Entity(game_state),
		materials(),
		model(model),
		mphys(mphys),
		physics(NULL),
		rigid_body(NULL),
		firer(firer),
		pos(pos),
		vel(vel),
		ori(ori),
		arm_time(game_state->total_game_time),
		detonation_time(game_state->total_game_time + 30.0f),
		impact_callback()
	{
		// look up all the materials the model uses in advance
		Cache<Material>* mat_cache = ((TestGame*)game_state)->mat_cache;
		for(vector<string>::iterator iter = model->materials.begin(); iter != model->materials.end(); ++iter)
			materials.push_back(mat_cache->Load(*iter));

		impact_callback.bug = this;
	}

	void RocketBug::InnerDispose()
	{
		DeSpawned();

		if(rigid_body) { rigid_body->Dispose(); delete rigid_body; rigid_body = NULL; }

		Entity::InnerDispose();
	}

	void RocketBug::Update(const TimingInfo& time)
	{
		pos = rigid_body->GetPosition();
		vel = rigid_body->GetLinearVelocity();
		ori = rigid_body->GetOrientation();
		
		if(time.total > detonation_time)
			Explode();
		else
		{
			Mat3 rm = ori.ToMat3();
			Vec3 fwd = rm * Vec3(0, 0, 1);

			float mass = rigid_body->GetMass();

			rigid_body->ApplyWorldForce(fwd * (mass * 50.0f), pos + fwd * 0.5f);				// thrust
			rigid_body->ApplyWorldForce(vel * (-mass * 0.01f), pos - fwd * 0.5f);				// drag
		}
	}

	void RocketBug::Spawned()
	{
		physics = game_state->physics_world;

		ModelPhysics::BonePhysics& bphys = mphys->bones[0];

		rigid_body = new RigidBody(this, bphys.collision_shape, bphys.mass_info, pos, ori);
		rigid_body->SetLinearVelocity(vel);

		rigid_body->SetCollisionCallback(&impact_callback);

		physics->AddCollisionObject(rigid_body);
	}

	void RocketBug::DeSpawned() { if(rigid_body) { physics->RemoveCollisionObject(rigid_body); } }

	void RocketBug::Vis(SceneRenderer* renderer)
	{
		if(renderer->camera->CheckSphereVisibility(pos, 1.0f))
			model->Vis(renderer, 0, rigid_body->GetTransformationMatrix(), NULL, &materials);
	}

	void RocketBug::Explode()
	{
		// TODO: apply damage to nearby doods; also push stuff with physics!

		is_valid = false;
	}




	/*
	 * RocketBug::ImpactCallback method
	 */
	void RocketBug::ImpactCallback::OnCollision(const ContactPoint& collision)
	{
		if(bug->game_state->total_game_time >= bug->arm_time && bug->is_valid)
			bug->Explode();
	}
}
