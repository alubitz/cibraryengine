#include "StdAfx.h"
#include "WorldBoundary.h"

namespace Test
{
	/*
	 * WorldBoundary methods
	 */
	WorldBoundary::WorldBoundary(GameState* gs, const Plane& plane) : Entity(gs), plane(plane), rigid_body(NULL) { }

	bool WorldBoundary::GetShot(Shot* shot, const Vec3& poi, const Vec3& vel, float mass) { return true; }

	void WorldBoundary::Spawned()
	{
		CollisionShape* shape = new InfinitePlaneShape(plane);

		rigid_body = new RigidBody(this, shape, MassInfo(), Vec3(plane.normal * plane.offset));
		game_state->physics_world->AddCollisionObject(rigid_body);
	}

	void WorldBoundary::DeSpawned()
	{
		if(rigid_body)
		{
			game_state->physics_world->RemoveCollisionObject(rigid_body);

			rigid_body->Dispose();
			delete rigid_body;
			rigid_body = NULL;
		}
	}




	/*
	 * TableParse stuff
	 */
	struct WorldBoundaryParams : public NamedItemDictionaryTableParser
	{
		GameState* game;

		Vec3 pos;
		Vec3 norm;

		Vec3Setter pos_setter;
		Vec3Setter norm_setter;

		WorldBoundaryParams(istream* stream, GameState* game) :
			NamedItemDictionaryTableParser(stream),
			game(game),
			pos(),
			norm(),
			pos_setter(&pos, stream),
			norm_setter(&norm, stream)
		{
			field_setters["pos"] = &pos_setter;
			field_setters["normal"] = &norm_setter;
		}

		void End()
		{
			if(norm.ComputeMagnitude() == 0)
				Debug("Failed to create world boundary; normal vector was unspecified or degenerate\n");
			else
				game->Spawn(new WorldBoundary(game, Plane::FromPositionNormal(pos, norm)));
		}
	};

	WorldBoundarySetter::WorldBoundarySetter(istream* stream, GameState* game) : stream(stream), game(game) { }
	TableParseable* WorldBoundarySetter::Set(const string& val) { return new WorldBoundaryParams(stream, game); }
}
