#pragma once

#include "StdAfx.h"

namespace Test
{
	using namespace std;

	// not implementing Disposable due to stupid stuff that happened with inheritance and lua_newuserdata
	class PathSearch
	{
		private:
			struct Imp;
			Imp* imp;
			
		public:

			PathSearch(unsigned int graph, unsigned int source, unsigned int target);

			void Dispose();

			void Think(int steps);
			void Solve();

			int GetGraph();
			int GetSource();
			int GetTarget();

			bool IsFinished();

			list<unsigned int> GetSolution();
	};

	void PushPathSearchHandle(lua_State* L, unsigned int graph, unsigned int source, unsigned int target);
}
