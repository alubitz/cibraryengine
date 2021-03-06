#pragma once

#include "StdAfx.h"
#include "Disposable.h"

struct lua_State;
namespace CibraryEngine
{
	using namespace std;

	class GameState;
	class ContentReqList;

	class ScriptingState : public Disposable
	{
		private:

			struct Imp;
			Imp* imp;

		protected:

			void InnerDispose();

		public:

			ScriptingState();
			ScriptingState(lua_State* L);

			lua_State* GetLuaState();

			int DoFunction(int args, int results);
			int DoString(string& str);
			int DoFile(string filename);

			bool IsValid();
	};

	struct ScriptSystem
	{
		static void Init();
		static void Shutdown();

		static bool IsInit();			// init and not shut down

		static ScriptingState GetGlobalState();
		static void SetGS(GameState* gs);
		static void SetContentReqList(ContentReqList* req);

		static void DoKeyStateCallback(int key, bool state);
		static void DoMouseButtonStateCallback(int button, bool state);
		static void DoMouseMovementCallback(int x, int y, int dx, int dy);
	};

	int ba_generic_concat(lua_State* L);
}
