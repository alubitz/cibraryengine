#include "StdAfx.h"
#include "Control.h"

#include "GameState.h"
#include "DebugLog.h"

#include "Scripting.h"
#include "Random3D.h"

namespace CibraryEngine
{
	using boost::unordered_map;

	/*
	 * ControlState methods
	 */
	ControlState::ControlState() : control_values() { }
	ControlState::~ControlState() { }

	bool ControlState::GetBoolControl(string control_name) { return GetFloatControl(control_name) != 0.0f; }

	float ControlState::GetFloatControl(string control_name)
	{ 
		unordered_map<string, float>::iterator found = control_values.find(control_name);
		return found != control_values.end() ? found->second : 0.0f;
	}

	void ControlState::SetBoolControl(string control_name, bool value)
	{
		// only change the float value if the bool value has changed
		if(value != GetBoolControl(control_name))
			control_values[control_name] = value ? 1.0f : 0.0f;
	}

	void ControlState::SetFloatControl(string control_name, float value) { control_values[control_name] = value; }




	/*
	 Pawn methods
	 */
	Pawn::Pawn(GameState* gs) : Entity(gs), controller(NULL), control_state(new ControlState()) { }
	Pawn::~Pawn() 
	{
		if(control_state != NULL)
		{
			delete control_state;
			control_state = NULL;
		}
	}




	/*
	 * Controller methods
	 */
	Controller::Controller(GameState* gs) : Entity(gs), pawn(NULL), ctrl_update_interval(0.1f) { next_ctrl_update = last_ctrl_update = gs->total_game_time; }
	Controller::~Controller() { }

	Pawn* Controller::GetControlledPawn() { return pawn; }

	void Controller::SetControlledPawn(Pawn* pawn_)
	{
		if(pawn != pawn_)
			pawn = pawn_;
	}

	void Controller::Possess(Pawn* pawn_)
	{
		Exorcise();
		pawn = pawn_;
		pawn->controller = this;
	}

	void Controller::Exorcise()
	{
		if (pawn != NULL)
			pawn->controller = NULL;
		pawn = NULL;
	}

	ControlState* Controller::GetControlState() { return pawn != NULL ? pawn->control_state : NULL; }

	void Controller::Update(TimingInfo time)
	{
		float now = time.total;

		if(now > next_ctrl_update)			// using > in case time.elapsed is 0 somehow
		{
			UpdateController(TimingInfo(now - last_ctrl_update, now));

			last_ctrl_update = now;
			next_ctrl_update = now + ctrl_update_interval * Random3D::Rand(0.5f, 1.5f);
		}
	}




	/*
	 * ScriptedController methods
	 */
	ScriptedController::ScriptedController(GameState* gs, string script) : Controller(gs), script(script) { }

	int cs_newindex(lua_State* L);
	int cs_index(lua_State* L);

	void ScriptedController::UpdateController(TimingInfo time)
	{
		string filename = ((stringstream&)(stringstream() << "Files/Scripts/" << script << ".lua")).str();

		ScriptingState script = ScriptSystem::GetGlobalState();
		lua_State* L = script.GetLuaState();

		// hook variables (hv)
		lua_newtable(L);

		// hv.control_state
		ControlState* cs = GetControlState();
		lua_newtable(L);

		lua_newtable(L);
		lua_pushlightuserdata(L, cs);
		lua_pushcclosure(L, cs_newindex, 1);
		lua_setfield(L, -2, "__newindex");
		lua_pushlightuserdata(L, cs);
		lua_pushcclosure(L, cs_index, 1);
		lua_setfield(L, -2, "__index");
		lua_setmetatable(L, -2);

		lua_setfield(L, -2, "control_state");

		// hv.time
		lua_newtable(L);
		lua_pushnumber(L, time.total);
		lua_setfield(L, -2, "total");
		lua_pushnumber(L, time.elapsed);
		lua_setfield(L, -2, "elapsed");
		lua_setfield(L, -2, "time");

		// hv.pawn
		GetControlledPawn()->PushScriptingHandle(L);
		lua_setfield(L, -2, "pawn");

		lua_setglobal(L, "hv");

		script.DoFile(filename);

		lua_pushnil(L);
		lua_setglobal(L, "hv");
	}

	int cs_newindex(lua_State* L)
	{
		ControlState* control_state = (ControlState*)lua_touserdata(L, lua_upvalueindex(1));

		if(lua_isstring(L, 2))
		{
			string key = lua_tostring(L, 2);

			if(lua_isnumber(L, 3))
				control_state->SetFloatControl(key, (float)lua_tonumber(L, 3));
			else if(lua_isboolean(L, 3))
				control_state->SetBoolControl(key, (bool)lua_toboolean(L, 3));
		}

		return 0;
	}

	int cs_index(lua_State* L)
	{
		ControlState* control_state = (ControlState*)lua_touserdata(L, lua_upvalueindex(1));

		if(lua_isstring(L, 2))
		{
			string key = lua_tostring(L, 2);

			lua_settop(L, 0);
			lua_pushnumber(L, control_state->GetFloatControl(key));

			return 1;
		}

		return 0;
	}
}
