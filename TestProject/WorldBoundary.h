#pragma once

#include "../CibraryEngine/CibraryEngine.h"

#include "Shootable.h"

namespace Test
{
	using namespace CibraryEngine;

	class WorldBoundary : public Entity, public Shootable
	{
		public:

			Plane plane;

			RigidBodyInfo* rigid_body;

			WorldBoundary(GameState* gs, Plane plane);

			bool GetShot(Shot* shot, Vec3 poi, Vec3 momentum);

			void Spawned();
			void DeSpawned();
	};

	struct WorldBoundarySetter : public NamedItemDictionaryTableParser::FieldSetter
	{
		istream* stream;
		GameState* game;

		WorldBoundarySetter(istream* stream, GameState* game);
		TableParseable* Set(string val);
	};
}
