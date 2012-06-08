#pragma once

#include "StdAfx.h"

#include "Shootable.h"

namespace Test
{
	class StaticLevelGeometry : public Entity, public Shootable, public VisionBlocker
	{
		protected:

			void InnerDispose();

		public:

			UberModel* model;
			vector<Material*> materials;
			ParticleMaterial* dirt_particle;
			ParticleMaterial* dust_particle;

			Vec3 pos;
			Quaternion ori;

			Sphere bs;

			CollisionShape* collision_shape;
			RigidBody* rigid_body;
			PhysicsWorld* physics;

			StaticLevelGeometry(GameState* gs, UberModel* model, CollisionShape* collision_shape, Vec3 pos, Quaternion ori);

			void Vis(SceneRenderer* renderer);

			void Spawned();
			void DeSpawned();

			bool GetShot(Shot* shot, Vec3 poi, Vec3 momentum);
	};

	struct StaticGeometrySetter : public NamedItemDictionaryTableParser::FieldSetter
	{
		istream* stream;
		GameState* game;

		StaticGeometrySetter(istream* stream, GameState* game);
		TableParseable* Set(string val);
	};
}
