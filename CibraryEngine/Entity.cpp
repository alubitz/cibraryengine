#include "StdAfx.h"
#include "Entity.h"

#include "TimingInfo.h"

#include "Component.h"

namespace CibraryEngine
{

	map<unsigned int, Entity*> all_entities = map<unsigned int, Entity*>();
	unsigned long int next_id = 1;


	/*
	 * Entity methods
	 */
	Entity::Entity(GameState* gs) : id(next_id++), scripting_handle(NULL), game_state(gs), is_valid(true) { all_entities[id] = this; }
	Entity::~Entity() { Dispose(); }

	void Entity::Update(TimingInfo time)
	{
		for(vector<Component*>::iterator iter = components.begin(); iter != components.end(); ++iter)
			(*iter)->Update(time);
	}

	void Entity::Vis(SceneRenderer* renderer)
	{
		for(vector<Component*>::iterator iter = components.begin(); iter != components.end(); ++iter)
			(*iter)->Vis(renderer);
	}

	void Entity::InnerDispose()			{ all_entities[id] = NULL; if(scripting_handle) { *scripting_handle = NULL; } }

	void Entity::Spawned()				{ }
	void Entity::DeSpawned()			{ }

	unsigned long int Entity::GetID()	{ return id; }

	void Entity::PushScriptingHandle(lua_State* L)
	{
		if(scripting_handle == NULL)
		{
			scripting_handle = (Entity**)lua_newuserdata(L, sizeof(Entity*));
			*scripting_handle = this;
		}
		else
			lua_pushlightuserdata(L, scripting_handle);
	}

	void Entity::TossScriptingHandle()	{ scripting_handle = NULL; }
}