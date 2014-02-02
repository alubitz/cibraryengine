#pragma once

#include "StdAfx.h"

#include "Shootable.h"

namespace Test
{
	using namespace CibraryEngine;

	class WorldBoundary : public Entity, public Shootable
	{
		public:

			Plane plane;

			RigidBody* rigid_body;

			WorldBoundary(GameState* gs, const Plane& plane);

			bool GetShot(Shot* shot, const Vec3& poi, const Vec3& vel, float mass);

			void Spawned();
			void DeSpawned();
	};

	struct WorldBoundarySetter : public NamedItemDictionaryTableParser::FieldSetter
	{
		istream* stream;
		GameState* game;

		WorldBoundarySetter(istream* stream, GameState* game);
		TableParseable* Set(const string& val);
	};
}
