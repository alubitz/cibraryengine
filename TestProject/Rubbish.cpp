#include "StdAfx.h"

#include "Rubbish.h"
#include "DSNMaterial.h"
#include "TestGame.h"

#include "Spark.h"

namespace Test
{
	Rubbish::Rubbish(GameState* gs, UberModel* model, ModelPhysics* model_phys, Vec3 pos, Quaternion ori, ParticleMaterial* dirt_particle) :
		Entity(gs),
		model(model),
		materials(),
		dirt_particle(dirt_particle),
		xform(Mat4::FromPositionAndOrientation(pos, ori)),
		bs(model->GetBoundingSphere()),
		model_phys(model_phys),
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
			rigid_body->DisposePreservingCollisionShape();
			delete rigid_body;
			rigid_body = NULL;
		}
	}

	void Rubbish::Vis(SceneRenderer* renderer)
	{
		if(renderer->camera->CheckSphereVisibility(Sphere(xform.TransformVec3_1(bs.center), bs.radius)))
			((TestGame*)game_state)->VisUberModel(renderer, model, 0, xform, NULL, &materials);
	}

	void Rubbish::Spawned()
	{
		physics = game_state->physics_world;
		
		if(model_phys->bones.size() > 0)
		{
			Vec3 pos = xform.TransformVec3_1(0, 0, 0);
			Vec3 a = xform.TransformVec3_0(1, 0, 0);
			Vec3 b = xform.TransformVec3_0(0, 1, 0);
			Vec3 c = xform.TransformVec3_0(0, 0, 1);

			ModelPhysics::BonePhysics& bone = model_phys->bones[0];

			rigid_body = new RigidBody(bone.collision_shape, bone.mass_info, pos, Quaternion::FromRotationMatrix(Mat3(a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z)));
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
#if 1
	#if 1
		BillboardMaterial* trail_mat = (BillboardMaterial*)((TestGame*)game_state)->mat_cache->Load("spark");
	#else
		BillboardMaterial* trail_mat = NULL;
	#endif

		for (int i = 0; i < 6; ++i)
			game_state->Spawn(new Spark(game_state, poi, trail_mat));
#endif

		Vec3 pos = xform.TransformVec3_1(0, 0, 0);
		rigid_body->ApplyImpulse(momentum, poi - pos);
		return true;

	}

	void Rubbish::Update(TimingInfo time) { xform = rigid_body->GetTransformationMatrix(); }
}
