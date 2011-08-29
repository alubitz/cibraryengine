#pragma once

#include "StdAfx.h"
#include "Disposable.h"
#include "TimingInfo.h"

namespace CibraryEngine
{
	using namespace std;

	class PhysicsWorld;
	class SoundSystem;
	class IKSolver;

	class Entity;
	struct ContentMan;

	class EntityList;
	class EntityQualifier;

	class ScriptingState;

	/** Class storing the state of the game's simulation */
	class GameState : public Disposable
	{
		private:

			bool spawn_directly;

		protected:

			list<Entity*> entities;
			list<Entity*> spawning;

			virtual void InnerDispose();

		public:

			ContentMan* content;

			/** The PhysicsWorld the game is using */
			PhysicsWorld* physics_world;
			/** The SoundSystem the game uses */
			SoundSystem* sound_system;

			IKSolver* ik_solver;

			/** The total amount of game time that has passed */
			float total_game_time;

			float elapsed_game_time;

			/** Initializes a GameState */
			GameState();
			/** Is this actually needed? */
			virtual ~GameState();

			/** Steps the simulation */
			virtual void Update(TimingInfo time);
			/** Draws the GameState, given the dimensions of the viewing area */
			virtual void Draw(int width, int height);

			/** Spawns an Entity, and returns it (for convenience); depending on when it is called, the entity may not be added to the entities list until the next simulation step */
			virtual Entity* Spawn(Entity* entity);				// returns the spawned entity (for convenience)

			/** Gets a list of all entities meeting the specified condition*/
			EntityList GetQualifyingEntities(EntityQualifier& cond);

			virtual void SetupScripting(ScriptingState& state);
	};
}
