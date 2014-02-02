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

			StaticLevelGeometry(GameState* gs, UberModel* model, CollisionShape* collision_shape, const Vec3& pos, const Quaternion& ori);

			void Vis(SceneRenderer* renderer);

			void Spawned();
			void DeSpawned();

			bool GetShot(Shot* shot, const Vec3& poi, const Vec3& vel, float mass);
	};

	struct StaticGeometrySetter : public NamedItemDictionaryTableParser::FieldSetter
	{
		istream* stream;
		GameState* game;

		StaticGeometrySetter(istream* stream, GameState* game);
		TableParseable* Set(const string& val);
	};
}
