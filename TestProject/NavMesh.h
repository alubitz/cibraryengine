#pragma once

#include "../CibraryEngine/CibraryEngine.h"

namespace Test
{
	using namespace CibraryEngine;

	struct NavEdge;
	struct NavGraph;

	struct NavNode
	{
		NavGraph* graph;
		Vec3 pos;
		vector<NavEdge> neighbors;

		NavNode(NavGraph* graph, Vec3 pos);
	};

	struct NavEdge
	{
		NavNode* node;
		float cost;
	};

	class NavGraph : public Disposable
	{
		protected:

			void InnerDispose();

			list<NavNode*> nodes;

		public:

			GameState* game_state;

			NavGraph(GameState* game_state);

			NavNode* CreateNode(Vec3 pos);
			void RemoveNode(NavNode* node);

			// this function is expensive, so don't call it too often
			vector<NavNode*> GetVisibleNodes(Vec3 pos);

			// screw visibility, just find the nearest node
			NavNode* GetNearestNode(Vec3 pos);
	};




	int PushNavNodeHandle(lua_State* L, NavNode* node);
}
