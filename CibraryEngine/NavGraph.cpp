#include "StdAfx.h"

#include "NavGraph.h"

#include "Serialize.h"
#include "Vector.h"
#include "GameState.h"
#include "Entity.h"
#include "Physics.h"
#include "VisionBlocker.h"

namespace CibraryEngine
{
	using boost::unordered_map;

	/*
	 * NavNode and NavEdge structs
	 */
	struct NavEdge
	{
		float cost;

		NavEdge(float cost);
	};

	struct NavNode
	{
		Vec3 pos;
		map<unsigned int, NavEdge*> edges;

		NavNode(Vec3 pos);
		~NavNode();

		NavEdge* GetEdgeByID(unsigned int id);
		void DeleteEdge(unsigned int id);
		void NewEdge(unsigned int id, float cost);
	};




	/*
	 * NavNode methods
	 */
	NavNode::NavNode(Vec3 pos) :
			pos(pos),
			edges()
		{
		}

	NavNode::~NavNode()
	{ 
		for(map<unsigned int, NavEdge*>::iterator iter = edges.begin(); iter != edges.end(); ++iter)
			delete iter->second;
		edges.clear();
	};

	NavEdge* NavNode::GetEdgeByID(unsigned int id)
	{
		map<unsigned int, NavEdge*>::iterator found = edges.find(id);
		if(found != edges.end())
			return found->second;
		else
			return NULL;
	}

	void NavNode::DeleteEdge(unsigned int id)
	{
		map<unsigned int, NavEdge*>::iterator found = edges.find(id);
		if(found != edges.end())
			edges.erase(found);
	}

	void NavNode::NewEdge(unsigned int id, float cost)
	{
		if(NavEdge* old_edge = GetEdgeByID(id))
			delete old_edge;
		edges[id] = new NavEdge(cost);
	}




	/*
	 * NavEdge methods
	 */
	NavEdge::NavEdge(float cost) :
		cost(cost)
	{
	}




	/*
	 * NavGraph methods
	 */
	NavGraph::ErrorCode navgraph_error = 0;
	NavGraph::ErrorCode NavGraph::GetError()
	{
		NavGraph::ErrorCode result = navgraph_error;

		navgraph_error = 0;
		return result;
	}
	string NavGraph::GetErrorString(ErrorCode error)
	{
		switch(error)
		{
		case 0:
			return "everything's peachy";
		case ERR_INVALID_GRAPH:
			return "invalid graph";
		case ERR_INVALID_NODE:
			return "invalid node";
		case ERR_INVALID_EDGE:
			return "invalid edge";
		case ERR_INVALID_ENUM:
			return "invalid enum";
		default:
			return "unknown error";
		}
	}

	struct NavGraphObject : public Disposable
	{
		protected:						// non-public only because Disposable says so

			void InnerDispose()
			{
				for(unordered_map<unsigned int, NavNode*>::iterator iter = nodes.begin(); iter != nodes.end(); ++iter)
					delete iter->second;
				nodes.clear();

				Disposable::InnerDispose();
			}

		public:

			unsigned int next_node_id;

			unordered_map<unsigned int, NavNode*> nodes;

			NavNode* GetNodeByID(unsigned int id)
			{
				unordered_map<unsigned int, NavNode*>::iterator found = nodes.find(id);
				if(found == nodes.end())
					return NULL;
				else
					return found->second;
			}
			NavEdge* GetEdgeByID(unsigned int a, unsigned int b)
			{
				if(NavNode* node_a = GetNodeByID(a))
					return node_a->GetEdgeByID(b);
				else
					return NULL;
			}
			
			GameState* game_state;
			
			NavGraphObject(GameState* game_state) :
				nodes(),
				next_node_id(1),
				game_state(game_state)
			{
			}

			unsigned int NewNode(Vec3 pos)
			{
				nodes[next_node_id] = new NavNode(pos);
				return next_node_id++;
			}

			void DeleteNode(unsigned int id)
			{
				unordered_map<unsigned int, NavNode*>::iterator found = nodes.find(id);
				if(found != nodes.end())
				{
					// this destructor will delete any edges from this node...
					delete found->second;
					nodes.erase(found);

					// ...but we also need to delete edges from other nodes to this node
					for(unordered_map<unsigned int, NavNode*>::iterator iter = nodes.begin(); iter != nodes.end(); ++iter)
						iter->second->DeleteEdge(id);
				}
			}

			vector<unsigned int> GetAllNodes()
			{
				vector<unsigned int> results;
				for(unordered_map<unsigned int, NavNode*>::iterator iter = nodes.begin(); iter != nodes.end(); ++iter)
					results.push_back(iter->first);
				return results;
			}

			vector<unsigned int> GetVisibleNodes(Vec3 pos)
			{
				vector<unsigned int> results;
				for(unordered_map<unsigned int, NavNode*>::iterator iter = nodes.begin(); iter != nodes.end(); ++iter)
				{
					if(VisionBlocker::CheckLineOfSight(game_state->physics_world, pos, iter->second->pos))
						results.push_back(iter->first);
				}

				return results;
			}

			unsigned int GetNearestNode(Vec3 pos)
			{
				unsigned int result = 0;
				float closest = 0;

				for(unordered_map<unsigned int, NavNode*>::iterator iter = nodes.begin(); iter != nodes.end(); ++iter)
				{
					Vec3 node_pos = iter->second->pos;
					float dist_sq = (pos - node_pos).ComputeMagnitudeSquared();
					if(result == 0 || dist_sq < closest)
					{
						closest = dist_sq;
						result = iter->first;
					}
				}

				return result;
			}

			vector<unsigned int> GetEdges(unsigned int node_id, NavGraph::PathDirection dir)
			{
				vector<unsigned int> results;

				if(dir > NavGraph::PD_EITHER)
				{
					navgraph_error = NavGraph::ERR_INVALID_ENUM;
					return results;
				}

				NavNode* node = GetNodeByID(node_id);
				if(!node)
				{
					navgraph_error = NavGraph::ERR_INVALID_NODE;
					return results;
				}

				if(dir & NavGraph::PD_OUT)
					for(map<unsigned int, NavEdge*>::iterator iter = node->edges.begin(); iter != node->edges.end(); ++iter)
						results.push_back(iter->first);

				if(dir & NavGraph::PD_IN)
					for(unordered_map<unsigned int, NavNode*>::iterator iter = nodes.begin(); iter != nodes.end(); ++iter)
						if(iter->second->GetEdgeByID(node_id))
							results.push_back(iter->first);

				return results;
			}

			bool HasEdge(unsigned int a, unsigned int b, NavGraph::PathDirection dir)
			{
				if(dir > NavGraph::PD_EITHER)
				{
					navgraph_error = NavGraph::ERR_INVALID_ENUM;
					return false;
				}
				else if(!GetNodeByID(a) || !GetNodeByID(b))
				{
					navgraph_error = NavGraph::ERR_INVALID_NODE;
					return false;
				}

				if((dir & NavGraph::PD_OUT) && GetEdgeByID(a, b) != NULL)
					return true;
				if((dir & NavGraph::PD_IN) && GetEdgeByID(b, a) != NULL)
					return true;

				return false;
			}

			float GetEdgeCost(unsigned int from, unsigned int to)
			{
				
				if(!GetNodeByID(from) || !GetNodeByID(to))
				{
					navgraph_error = NavGraph::ERR_INVALID_NODE;
					return 0.0f;
				}
				else if(NavEdge* edge = GetEdgeByID(from, to))
					return edge->cost;
				else
				{
					navgraph_error = NavGraph::ERR_INVALID_EDGE;
					return 0.0f;
				}
			}

			void SetPosition(unsigned int node, Vec3 pos)
			{
				if(NavNode* n = GetNodeByID(node))
					n->pos = pos;
				else
					navgraph_error = NavGraph::ERR_INVALID_NODE;
			}

			void NewEdge(unsigned int from, unsigned int to, float cost)
			{
				if(NavNode* from_node = GetNodeByID(from))
					if(NavNode* to_node = GetNodeByID(to))
					{
						from_node->NewEdge(to, cost);
						return;
					}

				// if we get here one of the above conditions wasn't met
				navgraph_error = NavGraph::ERR_INVALID_NODE;
				return;
			}

			void DeleteEdge(unsigned int from, unsigned int to)
			{
				if(NavNode* from_node = GetNodeByID(from))
					if(NavNode* to_node = GetNodeByID(to))
					{
						from_node->DeleteEdge(to);
						return;
					}
				
				// if we get here one of the above conditions wasn't met
				navgraph_error = NavGraph::ERR_INVALID_NODE;
				return;
			}
	};


	unordered_map<unsigned int, NavGraphObject*> all_nav_graphs;
	unsigned int next_nav_graph = 1;

	NavGraphObject* GetGraphByID(unsigned int id)
	{
		unordered_map<unsigned int, NavGraphObject*>::iterator obj = all_nav_graphs.find(id);
		if(obj == all_nav_graphs.end())
			return NULL;
		else
			return obj->second;
	}




	unsigned int NavGraph::NewNavGraph(GameState* game_state)
	{
		NavGraphObject* g = new NavGraphObject(game_state);
		all_nav_graphs[next_nav_graph] = g;

		return next_nav_graph++;
	}
	
	void NavGraph::DeleteNavGraph(unsigned int id)
	{
		NavGraphObject* obj = GetGraphByID(id);
		if(obj)
		{
			obj->Dispose();
			delete obj;

			all_nav_graphs[id] = NULL;
		}
	}

	unsigned int NavGraph::NewNode(unsigned int graph, Vec3 pos)
	{
		NavGraphObject* obj = GetGraphByID(graph);
		if(obj)
			return obj->NewNode(pos);
		else
		{
			navgraph_error = NavGraph::ERR_INVALID_GRAPH;
			return 0;
		}
	}

	void NavGraph::DeleteNode(unsigned int graph, unsigned int node)
	{
		NavGraphObject* obj = GetGraphByID(graph);
		if(obj)
			obj->DeleteNode(node);
		else
			navgraph_error = NavGraph::ERR_INVALID_GRAPH;		
	}

	vector<unsigned int> NavGraph::GetAllNodes(unsigned int graph)
	{
		NavGraphObject* obj = GetGraphByID(graph);
		if(obj)
			return obj->GetAllNodes();
		else
		{
			navgraph_error = NavGraph::ERR_INVALID_GRAPH;
			return vector<unsigned int>();
		}
	}

	vector<unsigned int> NavGraph::GetVisibleNodes(unsigned int graph, Vec3 pos)
	{
		NavGraphObject* obj = GetGraphByID(graph);
		if(obj)
			return obj->GetVisibleNodes(pos);
		else
		{
			navgraph_error = NavGraph::ERR_INVALID_GRAPH;
			return vector<unsigned int>();
		}
	}

	unsigned int NavGraph::GetNearestNode(unsigned int graph, Vec3 pos)
	{
		NavGraphObject* obj = GetGraphByID(graph);
		if(obj)
			return obj->GetNearestNode(pos);
		else
		{
			navgraph_error = NavGraph::ERR_INVALID_GRAPH;
			return 0;
		}
	}

	Vec3 NavGraph::GetNodePosition(unsigned int graph, unsigned int node)
	{
		NavGraphObject* obj = GetGraphByID(graph);
		if(obj)
			return obj->nodes[node]->pos;
		else
		{
			navgraph_error = NavGraph::ERR_INVALID_GRAPH;
			return Vec3();
		}
	}

	vector<unsigned int> NavGraph::GetNodeEdges(unsigned int graph, unsigned int node, NavGraph::PathDirection dir)
	{
		NavGraphObject* obj = GetGraphByID(graph);
		if(obj)
			return obj->GetEdges(node, dir);
		else
		{
			navgraph_error = NavGraph::ERR_INVALID_GRAPH;
			return vector<unsigned int>();
		}
	}

	bool NavGraph::NodeHasEdge(unsigned int graph, unsigned int node, unsigned int other, NavGraph::PathDirection dir)
	{
		NavGraphObject* obj = GetGraphByID(graph);
		if(obj)
			return obj->HasEdge(node, other, dir);
		else
		{
			navgraph_error = NavGraph::ERR_INVALID_GRAPH;
			return false;
		}
	}

	float NavGraph::GetEdgeCost(unsigned int graph, unsigned int from, unsigned int to)
	{
		NavGraphObject* obj = GetGraphByID(graph);
		if(obj)
			return obj->GetEdgeCost(from, to);
		else
		{
			navgraph_error = NavGraph::ERR_INVALID_GRAPH;
			return false;
		}
	}
	
	void NavGraph::NodeSetPosition(unsigned int graph, unsigned int node, Vec3 pos)
	{
		NavGraphObject* obj = GetGraphByID(graph);
		if(obj)
			obj->SetPosition(node, pos);
		else
			navgraph_error = NavGraph::ERR_INVALID_GRAPH;
	}

	void NavGraph::NewEdge(unsigned int graph, unsigned int node, unsigned int other, float cost)
	{
		NavGraphObject* obj = GetGraphByID(graph);
		if(obj)
			obj->NewEdge(node, other, cost);
		else
			navgraph_error = NavGraph::ERR_INVALID_GRAPH;
	}

	void NavGraph::DeleteEdge(unsigned int graph, unsigned int node, unsigned int other)
	{
		NavGraphObject* obj = GetGraphByID(graph);
		if(obj)
			obj->DeleteEdge(node, other);
		else
			navgraph_error = NavGraph::ERR_INVALID_GRAPH;
	}

	




	/*
	 * NavGraph scripting stuff
	 */
	int navnode_index(lua_State* L)
	{
		LuaNavNode* node = (LuaNavNode*)lua_touserdata(L, 1);

		if(lua_isstring(L, 2))
		{
			string key = lua_tostring(L, 2);

			lua_settop(L, 0);

			if (key == "pos")
			{
				PushLuaVector(L, NavGraph::GetNodePosition(node->graph, node->node));
				return 1;
			}
			else if	(key == "neighbors")
			{
				lua_newtable(L);

				vector<unsigned int> neighbors = NavGraph::GetNodeEdges(node->graph, node->node, NavGraph::PD_OUT);
				for(vector<unsigned int>::iterator iter = neighbors.begin(); iter != neighbors.end(); ++iter)
				{
					PushNavNodeHandle(L, node->graph, *iter);
					lua_pushnumber(L, NavGraph::GetEdgeCost(node->graph, node->node, *iter));
					lua_settable(L, 1);
					}

				return 1;
			}
		}

		return 0;
	}

	int navnode_eq(lua_State* L)
	{
		LuaNavNode* a = (LuaNavNode*)lua_touserdata(L, 1);
		LuaNavNode* b = (LuaNavNode*)lua_touserdata(L, 2);

		bool result = a->graph == b->graph && a->node == b->node;
		lua_settop(L, 0);
		lua_pushboolean(L, result);

		return 1;
	}




	/*
	 * Make some of this NavGraph stuff available to scripting
	 */
	int PushNavNodeHandle(lua_State* L, unsigned int graph, unsigned int node)
	{	
		LuaNavNode* my_node = (LuaNavNode*)lua_newuserdata(L, sizeof(LuaNavNode));
		my_node->graph = graph;
		my_node->node = node;

		lua_getglobal(L, "NavNodeMeta");
		if(lua_isnil(L, -1))
		{
			lua_pop(L, 1);
			// must create nav node metatable
			lua_newtable(L);

			lua_pushcclosure(L, navnode_index, 0);
			lua_setfield(L, -2, "__index");

			lua_pushcclosure(L, navnode_eq, 0);
			lua_setfield(L, -2, "__eq");

			lua_setglobal(L, "NavNodeMeta");
			lua_getglobal(L, "NavNodeMeta");
		}
		lua_setmetatable(L, -2);

		return 1;
	}




	/*
	 * NavGraph I/O functions
	 */
	void SaveNavGraph(unsigned int graph, string filename)
	{
		ofstream file(filename.c_str(), ios::out | ios::binary);
		if(!file)
			return;			// failed

		vector<unsigned int> nodes = NavGraph::GetAllNodes(graph);
		WriteUInt32(nodes.size(), file);

		for(unsigned int i = 0; i < nodes.size(); ++i)
		{
			unsigned int node = nodes[i];
			Vec3 pos = NavGraph::GetNodePosition(graph, node);
			
			WriteUInt32(node, file);
			WriteVec3(pos, file);
		}
		for(unsigned int i = 0; i < nodes.size(); ++i)
		{
			unsigned int node = nodes[i];
			WriteUInt32(node, file);

			vector<unsigned int> edges = NavGraph::GetNodeEdges(graph, node, NavGraph::PD_OUT);

			WriteUInt32(edges.size(), file);

			for(unsigned int j = 0; j < edges.size(); ++j)
			{
				unsigned int edge = edges[j];
				float cost = NavGraph::GetEdgeCost(graph, node, edge);

				WriteUInt32(edge, file);
				WriteSingle(cost, file);
			}
		}
	}

	unsigned int LoadNavGraph(GameState* game_state, string filename)
	{
		ifstream file(filename.c_str(), ios::in | ios::binary);
		if(!file)
			return 0;

		unsigned int graph = NavGraph::NewNavGraph(game_state);

		unsigned int n_nodes = ReadUInt32(file);
		unordered_map<unsigned int, unsigned int> nodes;
		for(unsigned int i = 0; i < n_nodes; ++i)
		{
			unsigned int node = ReadUInt32(file);
			Vec3 pos = ReadVec3(file);
			nodes[node] = NavGraph::NewNode(graph, pos);
		}
		for(unsigned int i = 0; i < n_nodes; ++i)
		{
			unsigned int node = ReadUInt32(file);
			unsigned int n_edges = ReadUInt32(file);

			for(unsigned int j = 0; j < n_edges; ++j)
			{
				unsigned int edge = ReadUInt32(file);
				float cost = ReadSingle(file);

				NavGraph::NewEdge(graph, nodes[node], nodes[edge], cost);
			}
		}

		return graph;
	}
}
