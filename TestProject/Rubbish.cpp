#include "StdAfx.h"

#include "Rubbish.h"
#include "DSNMaterial.h"
#include "TestGame.h"
#include "Particle.h"

namespace Test
{
	Rubbish::Rubbish(GameState* gs, UberModel* model, Vec3 pos, Quaternion ori, ParticleMaterial* dirt_particle) :
		Entity(gs),
		model(model),
		materials(),
		dirt_particle(dirt_particle),
		xform(Mat4::FromPositionAndOrientation(pos, ori)),
		bs(model->GetBoundingSphere()),
		rigid_body(NULL),
		physics(NULL)
	{
		Cache<Material>* mat_cache = gs->content->GetCache<Material>();
		for(unsigned int i = 0; i < model->materials.size(); ++i)
		{
			string material_name = model->materials[i];
			DSNMaterial* mat = (DSNMaterial*)mat_cache->Load(material_name);
			materials.push_back(mat);
		}
	}

	void Rubbish::InnerDispose()
	{
		Entity::InnerDispose();

		if(rigid_body != NULL)
		{
			//rigid_body->DisposePreservingCollisionShape();
			rigid_body->Dispose();
			delete rigid_body;
			rigid_body = NULL;
		}
	}

	void Rubbish::Vis(SceneRenderer* renderer)
	{
		if(renderer->camera->CheckSphereVisibility(Sphere(xform.TransformVec3(bs.center, 1.0), bs.radius)))
			((TestGame*)game_state)->VisUberModel(renderer, model, 0, xform, NULL, &materials);
	}

	void Rubbish::Spawned()
	{
		physics = game_state->physics_world;
		
		//if(model->bone_physics.size() > 0)
		//if(false)
		{
			//btCollisionShape* shape = model->bone_physics[0].shape;

			CollisionShape* shape = NULL;
			MassInfo mass_info;

#if 0
			shape = new SphereShape(0.6f);
			mass_info = shape->ComputeMassInfo() * 50.0f;
#else		
			{
				static const float b = 0.4f, r = 0.1f, s = b + r;

	#if 1
				Vec3 centers[] = { Vec3(-b, -b, -b), Vec3(-b, -b, b), Vec3(-b, b, -b), Vec3(-b, b, b), Vec3(b, -b, -b), Vec3(b, -b, b), Vec3(b, b, -b), Vec3(b, b, b) };
				float radii[] = { r, r, r, r, r, r, r, r };
				shape = new MultiSphereShape(centers, radii, 8);

				// constructing MassInfo for a cube
				mass_info.mass = 50;
				mass_info.com = Vec3();
				mass_info.moi[0] = mass_info.moi[4] = mass_info.moi[8] = mass_info.mass * (4.0f * s * s) / 1.0f;			// formula says divide by six, but some reason that doesn't work right
	#else
				Vec3 centers[] = { Vec3(0, 0.5f, 0), Vec3(0, 1.5f, 0) };
				float radii[] = { 0.5f, 0.5f };
				shape = new MultiSphereShape(centers, radii, 2);
	#endif
			}
#endif

			Vec3 pos = xform.TransformVec3(0, 0, 0, 1);
			Vec3 a = xform.TransformVec3(1, 0, 0, 0);
			Vec3 b = xform.TransformVec3(0, 1, 0, 0);
			Vec3 c = xform.TransformVec3(0, 0, 1, 0);
			float values[] = { a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z };

			rigid_body = new RigidBody(shape, mass_info, pos, Quaternion::FromRotationMatrix(Mat3(values)));
			rigid_body->SetUserEntity(this);

			physics->AddRigidBody(rigid_body);
			this->rigid_body = rigid_body;
		}
	}

	void Rubbish::DeSpawned()
	{
		if(rigid_body != NULL)
			physics->RemoveRigidBody(rigid_body);
	}

	bool Rubbish::GetShot(Shot* shot, Vec3 poi, Vec3 momentum)
	{
		for (int i = 0; i < 6; ++i)
		{
			Particle* p = new Particle(game_state, poi, Random3D::RandomNormalizedVector(5), dirt_particle, NULL, 0.05f, 1);
			p->gravity = 9.8f;
			p->damp = 2.0f;
			p->angle = -float(M_PI) * 0.5f;

			game_state->Spawn(p);
		}

		Vec3 pos = xform.TransformVec3(0, 0, 0, 1);
		rigid_body->ApplyImpulse(momentum, poi - pos);

		return true;
	}

	void Rubbish::Update(TimingInfo time) { xform = rigid_body->GetTransformationMatrix(); }
}
