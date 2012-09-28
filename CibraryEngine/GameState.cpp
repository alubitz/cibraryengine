#include "StdAfx.h"
#include "GameState.h"
#include "Physics.h"

#include "Entity.h"
#include "EntityList.h"
#include "SoundSystem.h"

#include "Scripting.h"

#include "DebugLog.h"
#include "ProfilingTimer.h"

#define PROFILE_GAMESTATE_UPDATE 0

namespace CibraryEngine
{

#if PROFILE_GAMESTATE_UPDATE
	static float timer_total = 0.0f;
	static float timer_update = 0.0f;
	static float timer_spawn = 0.0f;
	static float timer_physics = 0.0f;
	static unsigned int counter_game_update = 0;
#endif



	/*
	 * GameState methods
	 */
	GameState::GameState() : spawn_directly(true), entities(), spawning(), content(NULL), network_role(NR_SinglePlayer), sound_system(NULL)
	{
		physics_world = new PhysicsWorld(); 

		total_game_time = elapsed_game_time = 0.0f;
	}

	GameState::~GameState() { }

	void GameState::InnerDispose()
	{
		spawn_directly = false;			// don't let anybody spawn anything now

		// dispose of already-spawned entities
		for(list<Entity*>::iterator iter = entities.begin(); iter != entities.end(); ++iter)
		{
			(*iter)->DeSpawned();
			(*iter)->Dispose();
			delete *iter;
		}
		entities.clear();

		// dispose of entities that never entered the main list, as well
		for(list<Entity*>::iterator iter = spawning.begin(); iter != spawning.end(); ++iter)
		{
			(*iter)->Dispose();
			delete *iter;
		}
		spawning.clear();

		physics_world->Dispose();
		delete physics_world;
		physics_world = NULL;

#if PROFILE_GAMESTATE_UPDATE
		Debug(((stringstream&)(stringstream() << "total for " << counter_game_update << " calls to GameState::Update = " << timer_total << endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "update =\t\t\t\t"		<< timer_update <<	endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "spawn =\t\t\t\t\t"	<< timer_spawn <<	endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "physics =\t\t\t\t"	<< timer_physics <<	endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "total of above =\t\t"	<< timer_update + timer_spawn + timer_physics << endl)).str());
#endif
	}

	void GameState::Update(TimingInfo time)
	{
#if PROFILE_GAMESTATE_UPDATE
		ProfilingTimer timer, timer2;
		timer2.Start();
#endif

		total_game_time = time.total;
		elapsed_game_time = time.elapsed;

#if PROFILE_GAMESTATE_UPDATE
		timer.Start();
#endif

		spawn_directly = false;

		list<Entity*>::iterator iter = entities.begin();
		while(iter != entities.end())
		{
			Entity* e = *iter;
			e->Update(time);
			if(e->is_valid)
				++iter;
			else
			{
				e->DeSpawned();
				e->Dispose();
				iter = entities.erase(iter);
			}
		}

#if PROFILE_GAMESTATE_UPDATE
		timer_update += timer.GetAndRestart();
#endif

		for(list<Entity*>::iterator jter = spawning.begin(); jter != spawning.end(); ++jter)
		{
			(*jter)->Spawned();
			entities.push_back(*jter);
		}
		spawning.clear();

#if PROFILE_GAMESTATE_UPDATE
		timer_spawn += timer.GetAndRestart();
#endif

		physics_world->Update(time);

#if PROFILE_GAMESTATE_UPDATE
		timer_physics += timer.Stop();
#endif

		spawn_directly = true;

		if(sound_system != NULL)
			sound_system->Update(time);

#if PROFILE_GAMESTATE_UPDATE
		timer_total += timer2.Stop();
		++counter_game_update;
#endif
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
		for(list<Entity*>::iterator iter = entities.begin(); iter != entities.end(); ++iter)
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