#include "StdAfx.h"
#include "StaticLevelGeometry.h"

#include "TestGame.h"
#include "Particle.h"
#include "Spark.h"
#include "Rubbish.h"

namespace Test
{
	/*
	 * StaticLevelGeometry methods
	 */
	StaticLevelGeometry::StaticLevelGeometry(GameState* gs, UberModel* model, CollisionShape* collision_shape, Vec3 pos, Quaternion ori) :
		Entity(gs),
		model(model),
		materials(),
		pos(pos),
		ori(ori),
		collision_shape(collision_shape),
		rigid_body(NULL),
		physics(NULL)
	{
		bs = model->GetBoundingSphere();
		bs.center += pos;

		Cache<Material>* mat_cache = gs->content->GetCache<Material>();
		for(vector<string>::iterator iter = model->materials.begin(); iter != model->materials.end(); ++iter)
			materials.push_back(mat_cache->Load(*iter));

		dirt_particle = (ParticleMaterial*)mat_cache->Load("dirt_impact");
		dust_particle = (ParticleMaterial*)mat_cache->Load("dust_poof");
	}

	void StaticLevelGeometry::InnerDispose()
	{
		Entity::InnerDispose();

		if(rigid_body) { rigid_body->Dispose(); delete rigid_body; rigid_body = NULL; }
	}

	void StaticLevelGeometry::Vis(SceneRenderer* renderer)
	{
		if(renderer->camera->CheckSphereVisibility(bs))
			model->Vis(renderer, 0, Mat4::FromPositionAndOrientation(pos, ori), NULL, &materials);
	}

	void StaticLevelGeometry::Spawned() 
	{
		physics = game_state->physics_world;
		if(collision_shape != NULL)
		{
			RigidBody* rigid_body = new RigidBody(this, collision_shape, MassInfo(), pos, ori);

			physics->AddCollisionObject(rigid_body);
			this->rigid_body = rigid_body;
		}
	}

	void StaticLevelGeometry::DeSpawned() { if(rigid_body != NULL) { physics->RemoveCollisionObject(rigid_body); } }

	bool StaticLevelGeometry::GetShot(Shot* shot, const Vec3& poi, const Vec3& vel, float mass)
	{
		for(int i = 0; i < 8; ++i)
		{
			Particle* p = new Particle(game_state, poi, Random3D::RandomNormalizedVector(5), dirt_particle, NULL, 0.05f, 1.5f);
			p->gravity = 9.8f;
			p->damp = 0.2f;
			p->angle = -float(M_PI) * 0.5f;

			game_state->Spawn(p);
		}

		for(int i = 0; i < 8; ++i)
		{
			Particle* p = new Particle(game_state, poi, Random3D::RandomNormalizedVector(0.25f), dust_particle, NULL, 0.25f, 1.5f);
			p->gravity = 9.8f / 32.0f;
			p->damp = 1.0f;
			p->angle = -float(M_PI) * 0.5f;

			game_state->Spawn(p);
		}

		//game_state->Spawn(new Rubbish(game_state, ((TestGame*)game_state)->ubermodel_cache->Load("dummycube"), ((TestGame*)game_state)->mphys_cache->Load("dummycube"), poi + Vec3(0, 0.5f, 0), Quaternion::FromPYR(0, Random3D::Rand(2.0f * M_PI), 0), dirt_particle));

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

			UberModel* uber = content->GetCache<UberModel>()->Load(model_name);
			uber->LoadCachedMaterials(content->GetCache<Material>());

			CollisionShape* shape = NULL;

			ModelPhysics* phys = content->GetCache<ModelPhysics>()->Load(model_name);
			if(phys != NULL && !phys->bones.empty())
				shape = phys->bones[0].collision_shape;

			StaticLevelGeometry* geom = new StaticLevelGeometry(game, uber, shape, pos, ori);
			game->Spawn(geom);
		}
	};

	StaticGeometrySetter::StaticGeometrySetter(istream* stream, GameState* game) : stream(stream), game(game) { }
	TableParseable* StaticGeometrySetter::Set(string val) { return new StaticGeometryParams(val.substr(1, val.length() - 2), stream, game); }
}
