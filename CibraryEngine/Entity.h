#pragma once

#include "StdAfx.h"
#include "TimingInfo.h"

#include "Disposable.h"

namespace CibraryEngine
{
	using namespace std;

	class GameState;
	class SceneRenderer;
	class Component;

	/** Class representing an entity within the simulation */
	class Entity : public Disposable
	{
		private:

			unsigned long int id;

			Entity** scripting_handle;

		protected:

			virtual void InnerDispose();

		public:

			/** The game state which this entity belongs to */
			GameState* game_state;

	
			vector<Component*> components;

			/** Whether this entity is valid; if not, it may be removed from the GameState */
			bool is_valid;




			/** Initializes an Entity with the specified GameState */
			Entity(GameState* gs);
			/** Entity destructor... does this need to exist? */
			virtual ~Entity();

			/** Updates the entity given how much time has elapsed */
			virtual void Update(TimingInfo time);

			/** Lets this entity tell the renderer how to draw it */
			virtual void Vis(SceneRenderer* renderer);
			/** Allows the entity to clean up anything it created in a call to Vis */
			virtual void VisCleanup();

			/** Called when the Entity is spawned into the GameState */
			virtual void Spawned();
			/** Called when the Entity is removed from the GameState */
			virtual void DeSpawned();



			unsigned long int GetID();

			virtual Entity** GetScriptingHandle();
	};
}
