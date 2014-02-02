#pragma once

#include "StdAfx.h"

#include "Disposable.h"

namespace CibraryEngine
{
	using namespace std;

	class GameState;

	struct Vec3;

	class NavGraph : public Disposable
	{
		private:

			NavGraph();

		public:

			static const unsigned int PD_IN             = 0x0001;
			static const unsigned int PD_OUT            = 0x0002;
			static const unsigned int PD_EITHER         = PD_IN | PD_OUT;
			typedef unsigned int PathDirection;

			static const unsigned int ERR_INVALID_GRAPH = 0x0001;
			static const unsigned int ERR_INVALID_NODE  = 0x0002;
			static const unsigned int ERR_INVALID_EDGE  = 0x0003;
			static const unsigned int ERR_INVALID_ENUM  = 0x0004;
			typedef unsigned int ErrorCode;

			static ErrorCode GetError();
			static string GetErrorString(ErrorCode error);

			// navgraph creation/deletion
			static unsigned int NewNavGraph(GameState* game_state);
			static void DeleteNavGraph(unsigned int id);

			// graph modifier functions
			static unsigned int NewNode(unsigned int graph, const Vec3& pos);
			static void DeleteNode(unsigned int graph, unsigned int node);

			// graph getter functions
			static vector<unsigned int> GetAllNodes(unsigned int graph);
			static vector<unsigned int> GetVisibleNodes(unsigned int graph, const Vec3& pos);			// this function is expensive, so don't call it too often
			static unsigned int GetNearestNode(unsigned int graph, const Vec3& pos);					// screw visibility, just find the nearest node

			static Vec3 GetNodePosition(unsigned int graph, unsigned int node);
			static vector<unsigned int> GetNodeEdges(unsigned int graph, unsigned int node, PathDirection dir);
			static bool NodeHasEdge(unsigned int graph, unsigned int node, unsigned int other, PathDirection dir);

			static float GetEdgeCost(unsigned int graph, unsigned int from, unsigned int to);

			static void NodeSetPosition(unsigned int graph, unsigned int node, const Vec3& pos);

			static void NewEdge(unsigned int graph, unsigned int node, unsigned int other, float cost);
			static void DeleteEdge(unsigned int graph, unsigned int node, unsigned int other);
	
	};


	struct LuaNavNode
	{
		unsigned int graph;
		unsigned int node;
	};

	void SaveNavGraph(unsigned int graph, const string& filename);
	unsigned int LoadNavGraph(GameState* game_state, const string& filename);

	int PushNavNodeHandle(lua_State* L, unsigned int graph, unsigned int node);
}
