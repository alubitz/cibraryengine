#include "NavMesh.h"

#include "StaticLevelGeometry.h"

namespace Test
{
	/*
	 * NavNode methods
	 */
	NavNode::NavNode(NavGraph* graph, Vec3 pos) : graph(graph), pos(pos), neighbors() { }




	/*
	 * NavGraph methods
	 */
	NavGraph::NavGraph(GameState* game_state) :
		nodes(),
		game_state(game_state)
	{
	}

	void NavGraph::InnerDispose()
	{
		for(list<NavNode*>::iterator iter = nodes.begin(); iter != nodes.end(); iter++)
			delete *iter;
		nodes.clear();
	}

	NavNode* NavGraph::CreateNode(Vec3 pos)
	{
		NavNode* node = new NavNode(this, pos);
		nodes.push_back(node);
		return node;
	}

	void NavGraph::RemoveNode(NavNode* node)
	{
		// first check that the node we're removing actually belongs to us...
		if(node->graph == this)
		{
			for(list<NavNode*>::iterator iter = nodes.begin(); iter != nodes.end(); iter++)
				if(*iter == node)
				{
					nodes.erase(iter);
					delete node;
					return;
				}
		}
	}

	vector<NavNode*> NavGraph::GetVisibleNodes(Vec3 pos)
	{
		vector<NavNode*> results;

		for(list<NavNode*>::iterator iter = nodes.begin(); iter != nodes.end(); iter++)
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
			Vec3 node_pos = (*iter)->pos;
			game_state->physics_world->dynamics_world->rayTest(btVector3(node_pos.x, node_pos.y, node_pos.z), btVector3(pos.x, pos.y, pos.z), ray_callback);

			// if it's still zero, that means nothing is between those two points
			if(ray_callback.result == 0)
				results.push_back(*iter);
		}

		return results;
	}

	NavNode* NavGraph::GetNearestNode(Vec3 pos)
	{
		NavNode* result = NULL;
		float closest = 0;

		for(list<NavNode*>::iterator iter = nodes.begin(); iter != nodes.end(); iter++)
		{
			Vec3 node_pos = (*iter)->pos;
			float dist_sq = (pos - node_pos).ComputeMagnitudeSquared();
			if(result == NULL || dist_sq < closest)
			{
				closest = dist_sq;
				result = *iter;
			}
		}

		return result;
	}




	/*
	 * NavMesh scripting stuff
	 */
	int navnode_index(lua_State* L)
	{
		NavNode* node = *((NavNode**)lua_touserdata(L, 1));

		if(lua_isstring(L, 2))
		{
			string key = lua_tostring(L, 2);

			lua_settop(L, 0);

			if		(key == "pos") { PushLuaVector(L, node->pos); return 1; }
			else if	(key == "neighbors")
			{
				lua_newtable(L);

				for(vector<NavEdge>::iterator iter = node->neighbors.begin(); iter != node->neighbors.end(); iter++)
				{
					PushNavNodeHandle(L, iter->node);
					lua_pushnumber(L, iter->cost);
					lua_settable(L, 1);
				}

				return 1;
			}
		}

		return 0;
	}

	int PushNavNodeHandle(lua_State* L, NavNode* node)
	{
		if(node == NULL)
		{
			lua_pushnil(L);
			return 1;
		}
		else
		{
			NavNode** my_node = (NavNode**)lua_newuserdata(L, sizeof(NavNode*));
			*my_node = node;

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
}
