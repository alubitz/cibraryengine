#include "StdAfx.h"

#include "Rubbish.h"
#include "DSNMaterial.h"
#include "TestGame.h"
#include "Particle.h"

namespace Test
{
	Rubbish::Rubbish(GameState* gs, UberModel* model, Vec3 pos, Quaternion ori) :
		Entity(gs),
		model(model),
		materials(),
		xform(Mat4::FromPositionAndOrientation(pos, ori)),
		bs(model->GetBoundingSphere()),
		rigid_body(NULL),
		physics(NULL)
	{
		Cache<Material>* mat_cache = gs->content->GetCache<Material>();
		for(unsigned int i = 0; i < model->materials.size(); i++)
		{
			string material_name = model->materials[i];
			DSNMaterial* mat = (DSNMaterial*)mat_cache->Load(material_name);
			materials.push_back(mat);
		}
	}

	void Rubbish::InnerDispose()
	{
		VisCleanup();

		Entity::InnerDispose();

		if(rigid_body != NULL)
		{
			rigid_body->DisposePreservingCollisionShape();
			delete rigid_body;
		}
	}

	void Rubbish::Vis(SceneRenderer* renderer)
	{
		VisCleanup();				// just in case

		if(renderer->camera->CheckSphereVisibility(Sphere(xform.TransformVec3(bs.center, 1.0), bs.radius)))
			((TestGame*)game_state)->VisUberModel(renderer, model, 0, xform, NULL, &materials);
	}

	void Rubbish::VisCleanup() { }

	void Rubbish::Spawned()
	{
		physics = game_state->physics_world;
		if(model->bone_physics.size() > 0)
		{
			DEBUG();
			btCollisionShape* shape = model->bone_physics[0].shape;

			if(shape != NULL)
				DEBUG();

			btVector3 local_inertia;
			
			MassInfo mass_info(Vec3(), 20);
			mass_info.moi[0] = mass_info.moi[4] = mass_info.moi[8] = 10;

			Vec3 pos = xform.TransformVec3(0, 0, 0, 1);
			Vec3 a = xform.TransformVec3(1, 0, 0, 0);
			Vec3 b = xform.TransformVec3(0, 1, 0, 0);
			Vec3 c = xform.TransformVec3(0, 0, 1, 0);
			float values[] = { a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z };

			rigid_body = new RigidBodyInfo(shape, mass_info, pos, Quaternion::FromRotationMatrix(Mat3(values)));
			rigid_body->body->setUserPointer(this);

			rigid_body->body->setDamping(0.05f, 0.85f);
			rigid_body->body->setDeactivationTime(0.8f);
			rigid_body->body->setSleepingThresholds(1.6f, 2.5f);

			rigid_body->body->setFriction(1.0f);
			rigid_body->body->setRestitution(0.01f);

			physics->AddRigidBody(rigid_body);
			this->rigid_body = rigid_body;
		}
	}

	void Rubbish::DeSpawned()
	{
		if(model->bone_physics.size() > 0)
			physics->RemoveRigidBody(rigid_body);
	}

	bool Rubbish::GetShot(Shot* shot, Vec3 poi, Vec3 momentum)
	{
		ParticleMaterial* dirt_particle = ((TestGame*)game_state)->dirt_particle;
		for (int i = 0; i < 6; i++)
		{
			Particle* p = new Particle(game_state, poi, Random3D::RandomNormalizedVector(5), dirt_particle, 0.05f, 1);
			p->gravity = 9.8f;
			p->damp = 2.0f;
			p->angle = -M_PI * 0.5f;

			game_state->Spawn(p);
		}

		return true;
	}

	void Rubbish::Update(TimingInfo time)
	{
		xform = rigid_body->GetTransformationMatrix();
	}

}
