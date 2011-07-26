#include "WorldBoundary.h"

namespace Test
{
	/*
	 * WorldBoundary methods
	 */
	WorldBoundary::WorldBoundary(GameState* gs, Plane plane) : Entity(gs), plane(plane), rigid_body(NULL) { }

	bool WorldBoundary::GetShot(Shot* shot, Vec3 poi, Vec3 momentum) { return true; }

	void WorldBoundary::Spawned()
	{
		btStaticPlaneShape* shape = new btStaticPlaneShape(btVector3(plane.normal.x, plane.normal.y, plane.normal.z), 0);

		RigidBodyInfo* rigid_body = new RigidBodyInfo(shape, MassInfo(), Vec3(plane.normal * plane.offset));
		rigid_body->body->setUserPointer(this);

		rigid_body->body->setCollisionFlags(rigid_body->body->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
		rigid_body->body->setFriction(0);

		game_state->physics_world->AddRigidBody(rigid_body);
		this->rigid_body = rigid_body;
	}
	void WorldBoundary::DeSpawned() { }




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
	TableParseable* WorldBoundarySetter::Set(string val) { return new WorldBoundaryParams(stream, game); }
}
