#include "StdAfx.h"
#include "Scripting.h"

#include "Serialize.h"
#include "DebugLog.h"
#include "Vector.h"
#include "Content.h"
#include "GameState.h"

namespace CibraryEngine
{
	using namespace std;

	/*
	 * ScriptingState private implementation
	 */
	struct ScriptingState::Imp
	{
		lua_State* state;

		Imp() : state(lua_open()) { LoadDefaultLibs(); }
		Imp(lua_State* L) : state(L) { if(L != NULL) { LoadDefaultLibs(); } }

		void LoadDefaultLibs()
		{
			luaopen_base(state);
			luaopen_string(state);
			luaopen_table(state);
			luaopen_math(state);
			luaopen_os(state);
//			luaopen_io(state);
		}

		void Dispose() { lua_close(state); state = NULL; }

		int DoFunction(int args, int results)
		{
			if(int error = lua_pcall(state, args, results, 0))
			{
				stringstream msg;
				msg << "Script error: " << lua_tostring(state, -1) << endl;
				Debug(msg.str());

				return error;
			}
			return 0;
		}

		int DoString(string str) { return luaL_loadstring(state, str.c_str()) || DoFunction(0, LUA_MULTRET); }

		int DoFile(string filename)
		{
			string code;
			int load_error = GetFileString(filename, &code);
			if(load_error == 0)
			{
				int result = DoString(code);
				if(result != 0)
				{
					stringstream ss;
					ss << "Script error: " << lua_tostring(state, -1) << endl;
					Debug(ss.str());

					return result + 1;
				}
				else
					return 0;
			}
			else
			{
				Debug("Couldn't load script file (" + filename + ")\n");
				return 1;
			}
		}
	};




	/*
	 * ScriptingState methods
	 */
	ScriptingState::ScriptingState() : imp(new Imp()) { }
	ScriptingState::ScriptingState(lua_State* L) : imp(new Imp(L)) { }

	void ScriptingState::InnerDispose() { imp->Dispose(); delete imp; imp = NULL; }

	lua_State* ScriptingState::GetLuaState() { return imp->state; }

	int ScriptingState::DoFunction(int args, int results) { return imp->DoFunction(args, results); }
	int ScriptingState::DoString(string str) { return imp->DoString(str); }
	int ScriptingState::DoFile(string filename) { return imp->DoFile(filename); }

	bool ScriptingState::IsValid() { return imp != NULL && imp->state != NULL; }




	/*
	 * ScriptSystem methods (and some stuff related to that)
	 */
	ScriptingState global_state(NULL);
	GameState* game_state(NULL);
	ContentReqList* content_req_list(NULL);

	void SetupDefaultFunctions(ScriptingState& state);
	void DoGlobalsScript(ScriptingState& state);

	void ScriptSystem::Init()
	{
		if(global_state.IsValid())
		{
			Debug("You're calling ScriptSystem::Init more than once, is this deliberate?\n");
			global_state.Dispose();
		}

		global_state = ScriptingState();

		SetupDefaultFunctions(global_state);
		DoGlobalsScript(global_state);
	}
	void ScriptSystem::Shutdown() { global_state.Dispose(); }

	bool ScriptSystem::IsInit() { return global_state.GetLuaState() != NULL; }

	ScriptingState ScriptSystem::GetGlobalState() { return global_state; }

	void ScriptSystem::SetGS(GameState* gs)
	{
		game_state = gs;
		if(gs != NULL)
		{
			lua_State* L = global_state.GetLuaState();
			lua_settop(L, 0);
			lua_newtable(L);

			gs->SetupScripting(global_state);
			assert(lua_gettop(L) == 1 && "GameState::SetupScripting (or some override of that) pushed or popped too many items from the lua stack!");

			lua_setglobal(L, "gs");
		}
	}

	void ScriptSystem::SetContentReqList(ContentReqList* req) { content_req_list = req; }

	void ScriptSystem::DoKeyStateCallback(int key, bool state)
	{
		if(!ScriptSystem::IsInit())
			return;

		lua_State* L = global_state.GetLuaState();
		lua_settop(L, 0);
		lua_getglobal(L, "ba");						// push; top = 1
		lua_getfield(L, 1, "keyStateCallback");		// push; top = 2
		if(!lua_isnil(L, 2))
		{
			lua_pushnumber(L, key);						// push; top = 3
			lua_pushboolean(L, state);					// push; top = 4
			lua_call(L, 2, 0);							// pop x3; top = 1
		}
		lua_settop(L, 0);
	}

	void ScriptSystem::DoMouseButtonStateCallback(int button, bool state)
	{
		if(!ScriptSystem::IsInit())
			return;

		lua_State* L = global_state.GetLuaState();
		lua_settop(L, 0);
		lua_getglobal(L, "ba");								// push; top = 1
		lua_getfield(L, 1, "mouseButtonStateCallback");		// push; top = 2
		if(!lua_isnil(L, 2))
		{
			lua_pushnumber(L, button);						// push; top = 3
			lua_pushboolean(L, state);						// push; top = 4
			lua_call(L, 2, 0);								// pop x3; top = 1
		}
		lua_settop(L, 0);
	}

	void ScriptSystem::DoMouseMovementCallback(int x, int y, int dx, int dy)
	{
		if(!ScriptSystem::IsInit())
			return;

		lua_State* L = global_state.GetLuaState();
		lua_settop(L, 0);
		lua_getglobal(L, "ba");								// push; top = 1
		lua_getfield(L, 1, "mouseMovementCallback");		// push; top = 2
		if(!lua_isnil(L, 2))
		{
			lua_pushnumber(L, x);							// push; top = 3
			lua_pushnumber(L, y);							// push; top = 4
			lua_pushnumber(L, dx);							// push; top = 5
			lua_pushnumber(L, dy);							// push; top = 6
			lua_call(L, 4, 0);								// pop x5; top = 1
		}
		lua_settop(L, 0);
	}




	/*
	 * Implementation of those functions used by ScriptSystem
	 */
	int ba_println(lua_State* L);
	int ba_loadModel(lua_State* L);

	void SetupDefaultFunctions(ScriptingState& state)
	{
		lua_State* L = state.GetLuaState();
		lua_settop(L, 0);

		// begin namespace "ba" (base)
		lua_newtable(L);							// push; top = 1

		lua_pushcclosure(L, ba_println, 0);			// push; top = 2
		lua_setfield(L, 1, "println");				// set field of 1; pop; top = 1
		lua_pushcclosure(L, ba_createVector, 0);	// push; top = 2
		lua_setfield(L, 1, "createVector");			// set field of 1; pop; top = 1
		lua_pushcclosure(L, ba_loadModel, 0);
		lua_setfield(L, 1, "loadModel");

		lua_setglobal(L, "ba");						// pop; top = 0
	}

	void DoGlobalsScript(ScriptingState& state) { state.DoFile("Files/Scripts/globals.lua"); }




	/*
	 * And now for the implementation of those lua functions...
	 */
	int ba_println(lua_State* L)
	{
		int n = lua_gettop(L);

		if(n == 1)
		{
			lua_getglobal(L, "tostring");
			lua_pushvalue(L, 1);
			lua_call(L, 1, 1);

			string str = lua_tostring(L, 2);
			Debug("[Lua] " + str + "\n");
		}

		lua_settop(L, 0);

		return 0;
	}

	int ba_generic_concat(lua_State* L)
	{
		// top = 2
		lua_getglobal(L, "tostring");
		lua_pushvalue(L, 1);
		lua_call(L, 1, 1);					// top = 3

		lua_getglobal(L, "tostring");
		lua_pushvalue(L, 2);
		lua_call(L, 1, 1);					// top = 4

		lua_concat(L, 2);

		lua_replace(L, 1);
		lua_settop(L, 1);

		return 1;
	}

	int ba_loadModel(lua_State* L)
	{
		int n = lua_gettop(L);

		if(content_req_list == NULL)
		{
			Debug("ba.loadModel is not valid in this context; returning nil\n");
			return 0;
		}

		if(n == 1 && lua_isstring(L, 1))
		{
			string model_name = lua_tostring(L, 1);
			ContentHandle<UberModel> handle  = content_req_list->LoadModel(model_name);

			lua_settop(L, 0);
			ContentHandle<UberModel>* ptr = (ContentHandle<UberModel>*)lua_newuserdata(L, sizeof(ContentHandle<UberModel>));
			*ptr = handle;

			return 1;
		}

		Debug("ba.loadModel takes exactly one argument, the name of the model to load; returning nil\n");
		return 0;
	}
}
