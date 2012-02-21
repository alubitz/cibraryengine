#include "StdAfx.h"
#include "StaticLevelGeometry.h"

#include "DSNMaterial.h"
#include "TestGame.h"
#include "Particle.h"
#include "Rubbish.h"

namespace Test
{
	/*
	 * StaticLevelGeometry methods
	 */
	StaticLevelGeometry::StaticLevelGeometry(GameState* gs, UberModel* model, Vec3 pos, Quaternion ori) :
		Entity(gs),
		model(model),
		materials(),
		pos(pos),
		ori(ori),
		rigid_body(NULL),
		physics(NULL)
	{

		bs = model->GetBoundingSphere();
		bs.center += pos;

		Cache<Material>* mat_cache = gs->content->GetCache<Material>();
		for(unsigned int i = 0; i < model->materials.size(); ++i)
		{
			string material_name = model->materials[i];
			DSNMaterial* mat = (DSNMaterial*)mat_cache->Load(material_name);
			materials.push_back(mat);
		}

		dirt_particle = (ParticleMaterial*)mat_cache->Load("dirt_impact");
	}

	void StaticLevelGeometry::InnerDispose()
	{
		VisCleanup();

		Entity::InnerDispose();

		if(rigid_body != NULL)
		{
			//rigid_body->DisposePreservingCollisionShape();
			rigid_body->Dispose();
			delete rigid_body;
		}
	}

	void StaticLevelGeometry::Vis(SceneRenderer* renderer)
	{
		VisCleanup();				// just in case

		if(renderer->camera->CheckSphereVisibility(bs))
			((TestGame*)game_state)->VisUberModel(renderer, model, 0, Mat4::FromPositionAndOrientation(pos, ori), NULL, &materials);
	}
	void StaticLevelGeometry::VisCleanup() { }

	void StaticLevelGeometry::Spawned() 
	{
		physics = game_state->physics_world;
		if(model->bone_physics.size() > 0)
		{
			CollisionShape* shape = new TriangleMeshShape();//model->bone_physics[0].shape;

			RigidBody* rigid_body = new RigidBody(shape, MassInfo(), pos, ori);

			physics->AddRigidBody(rigid_body);
			this->rigid_body = rigid_body;
		}
	}

	void StaticLevelGeometry::DeSpawned()
	{
		if(model->bone_physics.size() > 0)
			physics->RemoveRigidBody(rigid_body);
	}

	bool StaticLevelGeometry::GetShot(Shot* shot, Vec3 poi, Vec3 momentum)
	{
		for (int i = 0; i < 6; ++i)
		{
			Particle* p = new Particle(game_state, poi, Random3D::RandomNormalizedVector(5), dirt_particle, NULL, 0.05f, 1);
			p->gravity = 9.8f;
			p->damp = 2.0f;
			p->angle = -float(M_PI) * 0.5f;

			game_state->Spawn(p);
		}

//		game_state->Spawn(new Rubbish(game_state, ((TestGame*)game_state)->ubermodel_cache->Load("dummycube"), poi + Vec3(0, 0.5f, 0), Quaternion::FromPYR(0, Random3D::Rand(2.0 * M_PI), 0), dirt_particle));

		return true;
	}




	/*
	 * StaticLevelGeometry table setter stuff
	 */
	struct StaticGeometryParams : public NamedItemDictionaryTableParser
	{
		string model_name;
		Vec3 pos;
		Quaternion ori;

		Vec3Setter pos_setter;
		QuaternionSetter ori_setter;

		GameState* game;

		StaticGeometryParams(string model_name, istream* stream, GameState* game) :
			NamedItemDictionaryTableParser(stream),
			model_name(model_name),
			pos(),
			ori(Quaternion::Identity()),
			pos_setter(&pos, stream),
			ori_setter(&ori, stream),
			game(game)
		{
			field_setters["pos"] = &pos_setter;
			field_setters["ori"] = &ori_setter;
		}

		void End()
		{
			ContentMan* content = game->content;

			StaticLevelGeometry* geom = new StaticLevelGeometry(game, content->GetCache<UberModel>()->Load(model_name), pos, ori);
			game->Spawn(geom);
		}
	};

	StaticGeometrySetter::StaticGeometrySetter(istream* stream, GameState* game) : stream(stream), game(game) { }
	TableParseable* StaticGeometrySetter::Set(string val) { return new StaticGeometryParams(val.substr(1, val.length() - 2), stream, game); }
}
