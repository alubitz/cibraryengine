#include "StdAfx.h"
#include "GameState.h"
#include "Physics.h"

#include "IKSolver.h"

#include "Entity.h"
#include "EntityList.h"
#include "SoundSystem.h"

#include "Scripting.h"

namespace CibraryEngine
{
	GameState::GameState() : spawn_directly(true), entities(), spawning(), content(NULL), sound_system(NULL) { physics_world = new PhysicsWorld(); ik_solver = new IKSolver(physics_world); }

	GameState::~GameState() { }

	void GameState::InnerDispose()
	{
		spawn_directly = false;			// don't let anybody spawn anything now

		// dispose of already-spawned entities
		for(list<Entity*>::iterator iter = entities.begin(); iter != entities.end(); iter++)
		{
			(*iter)->DeSpawned();
			(*iter)->Dispose();
			delete *iter;
		}
		entities.clear();

		// dispose of entities that never entered the main list, as well
		for(list<Entity*>::iterator iter = spawning.begin(); iter != spawning.end(); iter++)
		{
			(*iter)->Dispose();
			delete *iter;
		}
		spawning.clear();

		physics_world->Dispose();
		delete physics_world;
		physics_world = NULL;

		if(ik_solver != NULL)
		{
			ik_solver->Dispose();
			delete ik_solver;
			ik_solver = NULL;
		}
	}

	void GameState::Update(TimingInfo time)
	{
		total_game_time = time.total;
		elapsed_game_time = time.elapsed;

		spawn_directly = false;

		list<Entity*>::iterator iter = entities.begin();
		while(iter != entities.end())
		{
			Entity* e = *iter;
			e->Update(time);
			if(e->is_valid)
				iter++;
			else
			{
				e->DeSpawned();
				e->Dispose();
				iter = entities.erase(iter);
			}
		}

		physics_world->Update(time);

		spawn_directly = true;

		for(list<Entity*>::iterator jter = spawning.begin(); jter != spawning.end(); jter++)
		{
			(*jter)->Spawned();
			entities.push_back(*jter);
		}
		spawning.clear();

		if(sound_system != NULL)
			sound_system->Update(time);
	}

	void GameState::Draw(int width, int height) { }

	Entity* GameState::Spawn(Entity* e)
	{
		if(spawn_directly)
		{
			entities.push_back(e);
			e->Spawned();
		}
		else
			spawning.push_back(e);

		return e;
	}

	EntityList GameState::GetQualifyingEntities(EntityQualifier& cond)
	{
		vector<Entity*> ent_vector;
		for(list<Entity*>::iterator iter = entities.begin(); iter != entities.end(); iter++)
		{
			Entity *ent = *iter;
			if(cond.Accept(ent))
				ent_vector.push_back(ent);
		}

		return EntityList(ent_vector);
	}

	int gs_getElapsedTime(lua_State* L);
	int gs_getTotalTime(lua_State* L);
	void GameState::SetupScripting(ScriptingState& state)
	{
		lua_State* L = state.GetLuaState();

		lua_pushlightuserdata(L, (void*)this);
		lua_pushcclosure(L, gs_getElapsedTime, 1);
		lua_setfield(L, 1, "getElapsedTime");

		lua_pushlightuserdata(L, (void*)this);
		lua_pushcclosure(L, gs_getTotalTime, 1);
		lua_setfield(L, 1, "getTotalTime");
	}




	/*
	 * Implementation of those two functions used in GameState::SetupScripting
	 */
	int gs_getElapsedTime(lua_State* L)
	{
		GameState* gs = (GameState*)lua_touserdata(L, lua_upvalueindex(1));

		lua_settop(L, 0);
		lua_pushnumber(L, gs->elapsed_game_time);

		return 1;
	}

	int gs_getTotalTime(lua_State* L)
	{
		GameState* gs = (GameState*)lua_touserdata(L, lua_upvalueindex(1));

		lua_settop(L, 0);
		lua_pushnumber(L, gs->total_game_time);

		return 1;
	}

}