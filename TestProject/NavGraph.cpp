#include "StdAfx.h"
#include "NavGraph.h"

#include "StaticLevelGeometry.h"

namespace Test
{
	struct NavNode
	{
		Vec3 pos;

		NavNode(Vec3 pos) : pos(pos) { }
	};

	struct NavEdge
	{
		float cost;

		NavEdge(float cost) : cost(cost) { }
	};

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
				for(map<unsigned int, NavNode*>::iterator iter = nodes.begin(); iter != nodes.end(); iter++)
					delete iter->second;
				nodes.clear();
				
				for(unsigned int y = 0; y < edges.size(); y++)
				{
					for(unsigned int x = 0; x < edges[y].size(); x++)
						if(edges[y][x] != NULL)
							delete edges[y][x];
					edges[y].clear();
				}
				edges.clear();

				Disposable::InnerDispose();
			}

		public:

			unsigned int next_node_id;

			map<unsigned int, NavNode*> nodes;
			vector<vector<NavEdge*> > edges;

			NavNode* GetNodeByID(unsigned int id)
			{
				map<unsigned int, NavNode*>::iterator found = nodes.find(id);
				if(found == nodes.end())
					return NULL;
				else
					return found->second;
			}
			NavEdge* GetEdgeByID(unsigned int a, unsigned int b)
			{
				if(a > edges.size())
					return NULL;
				else if(b > edges[a].size())
					return NULL;
				else
					return edges[a][b];
			}
			
			GameState* game_state;
			
			NavGraphObject(GameState* game_state) :
				nodes(),
				edges(),
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
				NavNode* node = nodes[id];
				delete node;
				nodes[id] = NULL;
				
				// delete everything this node connects to
				if(edges.size() > id)
				{
					vector<NavEdge*>& row = edges[id];
					for(unsigned int x = 0; x < edges[id].size(); x++)
					{
						delete row[x];
						row[x] = NULL;
					}
				}
				// as well as anything that has edges connecting to this node
				for(unsigned int y = 0; y < edges.size(); y++)
				{
					vector<NavEdge*>& row = edges[y];
					if(row.size() > id)
					{
						delete row[id];
						row[id] = NULL;
					}
				}
			}

			vector<unsigned int> GetVisibleNodes(Vec3 pos)
			{
				vector<unsigned int> results;
				for(map<unsigned int, NavNode*>::iterator iter = nodes.begin(); iter != nodes.end(); iter++)
				{
					// define a callback for when a ray intersects an object
					struct : btCollisionWorld::RayResultCallback
					{
						float result;

						btScalar addSingleResult(btCollisionWorld::LocalRayResult& rayResult, bool normalInWorldSpace)
						{
							void* void_pointer = rayResult.m_collisionObject->getUserPointer();
							if(void_pointer != NULL)
							{
								StaticLevelGeometry* slg = dynamic_cast<StaticLevelGeometry*>((Entity*)void_pointer);
								if(slg != NULL)
								{
									float frac = rayResult.m_hitFraction;
									if(frac > result)
										result = frac;
								}
							}
							return 1;
						}
					} ray_callback;

					ray_callback.result = 0;

					// run that function for anything on this ray...
					Vec3 node_pos = iter->second->pos;
					game_state->physics_world->dynamics_world->rayTest(btVector3(node_pos.x, node_pos.y, node_pos.z), btVector3(pos.x, pos.y, pos.z), ray_callback);

					// if it's still zero, that means nothing is between those two points
					if(ray_callback.result == 0)
						results.push_back(iter->first);
				}

				return results;
			}

			unsigned int GetNearestNode(Vec3 pos)
			{
				unsigned int result = 0;
				float closest = 0;

				for(map<unsigned int, NavNode*>::iterator iter = nodes.begin(); iter != nodes.end(); iter++)
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

			vector<unsigned int> GetEdges(unsigned int node, NavGraph::PathDirection dir)
			{
				vector<unsigned int> results;

				if(dir > NavGraph::PD_EITHER)
				{
					navgraph_error = NavGraph::ERR_INVALID_ENUM;
					return results;
				}
				else if(nodes.size() < node || nodes[node] == NULL)
				{
					navgraph_error = NavGraph::ERR_INVALID_NODE;
					return results;
				}

				if(dir & NavGraph::PD_OUT)
					if(edges.size() > node)
					{
						vector<NavEdge*>& row = edges[node];
						for(unsigned int x = 0; x < row.size(); x++)
							if(row[x] != NULL)
								results.push_back(x);
					}

				if(dir & NavGraph::PD_IN)
					for(unsigned int y = 0; y < edges.size(); y++)
					{
						vector<NavEdge*>& row = edges[y];
						if(row.size() > node && row[node] != NULL)
							results.push_back(y);
					}

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
				if(!GetNodeByID(from) || !GetNodeByID(to))
				{
					navgraph_error = NavGraph::ERR_INVALID_NODE;
					return;
				}

				while(edges.size() <= from) { edges.push_back(vector<NavEdge*>()); }
				
				vector<NavEdge*>& row = edges[from];
				while(row.size() <= to)
					row.push_back(NULL);

				row[to] = new NavEdge(cost);
			}

			void DeleteEdge(unsigned int from, unsigned int to)
			{
				if(!GetNodeByID(from) || !GetNodeByID(to))
				{
					navgraph_error = NavGraph::ERR_INVALID_NODE;
					return;
				}

				if(edges.size() > from)
					if(edges[from].size() > to)
					{
						delete edges[from][to];
						edges[from][to] = NULL;
					}
			}
	};


	map<unsigned int, NavGraphObject*> all_nav_graphs;
	unsigned int next_nav_graph = 1;

	NavGraphObject* GetGraphByID(unsigned int id)
	{
		map<unsigned int, NavGraphObject*>::iterator obj = all_nav_graphs.find(id);
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
	struct LuaNavNode
	{
		unsigned int graph;
		unsigned int node;
	};

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
				for(vector<unsigned int>::iterator iter = neighbors.begin(); iter != neighbors.end(); iter++)
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

			lua_setglobal(L, "NavNodeMeta");
			lua_getglobal(L, "NavNodeMeta");
		}
		lua_setmetatable(L, -2);

		return 1;
	}
}
