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

			Vec3 pos;
			Quaternion ori;

			Sphere bs;

			RigidBodyInfo* rigid_body;
			PhysicsWorld* physics;

			StaticLevelGeometry(GameState* gs, UberModel* model, Vec3 pos, Quaternion ori);

			void Vis(SceneRenderer* renderer);
			void VisCleanup();

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
