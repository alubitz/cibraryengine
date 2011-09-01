#include "StdAfx.h"

#include "Pathfinding.h"
#include "NavGraph.h"
#include "Vector.h"
#include "DebugLog.h"

namespace CibraryEngine
{
	struct Edge
	{
		unsigned int to;
		float cost;

		Edge() : to(0), cost(0) { }
		Edge(unsigned int to, float cost) : to(to), cost(cost) { }
	};
	struct PriorityQueue
	{
		list<unsigned int> queue;
		vector<float>* priorities;

		PriorityQueue(vector<float>* priorities) : queue(), priorities(priorities) { }

		unsigned int Pop()
		{
			if(queue.empty()) 
				return 0;
			else
			{
				unsigned int result = queue.front();
				queue.pop_front();
				return result;
			}
		}

		void Insert(unsigned int node)
		{
			float priority = (*priorities)[node];
			list<unsigned int>::iterator iter;
			for(iter = queue.begin(); iter != queue.end(); iter++)
				if((*priorities)[*iter] > priority)
				{
					queue.insert(iter, node);
					break;
				}
			if(iter == queue.end())
				queue.push_back(node);
		}

		bool Empty() { return queue.empty(); }

		void ChangePriority(unsigned int node)
		{
			for(list<unsigned int>::iterator iter = queue.begin(); iter != queue.end(); iter++)
				if(*iter == node)
				{
					queue.erase(iter);
					break;
				}
			Insert(node);
		}
	};




	/*
	 * PathSearch private implementation struct
	 */
	struct PathSearch::Imp
	{
		unsigned int graph;
		unsigned int source;
		unsigned int target;

		bool finished;

		list<unsigned int> solution;

		vector<unsigned int> nodes;
		vector<Edge*> shortest_path_tree;
		vector<Edge*> search_frontier;
		map<unsigned int, unsigned int> node_to_index;
		vector<float> f_costs;
		vector<float> g_costs;
		PriorityQueue pq;

		Imp(unsigned int graph, unsigned int source, unsigned int target) :
			graph(graph),
			source(source),
			target(target),
			finished(false),
			solution(),
			nodes(NavGraph::GetAllNodes(graph)),
			shortest_path_tree(nodes.size(), NULL),
			search_frontier(nodes.size(), NULL),
			node_to_index(),
			f_costs(nodes.size(), 0.0f),
			g_costs(nodes.size(), 0.0f),
			pq(&f_costs)
		{

			for(unsigned int i = 0; i < nodes.size(); i++)
				node_to_index[nodes[i]] = i;

			pq.Insert(node_to_index[source]);
		}

		void Think(int steps)
		{
			// if the search has already finished, there's no need to do any more iterations
			if(finished)
				return;

			int iteration = 0;

			// if steps < 0, this will loop until the search is finished
			while(!pq.Empty() && iteration != steps)
			{
				unsigned int closest = pq.Pop();

				shortest_path_tree[closest] = search_frontier[closest];
				if(nodes[closest] == target)
				{
					// hooray, found a path; skip to end of loop
					finished = true;
					break;
				}

				Vec3 pos_a = NavGraph::GetNodePosition(graph, nodes[closest]);

				vector<unsigned int> edges = NavGraph::GetNodeEdges(graph, nodes[closest], NavGraph::PD_OUT);
				for(vector<unsigned int>::iterator iter = edges.begin(); iter != edges.end(); iter++)
				{
					Vec3 pos_b = NavGraph::GetNodePosition(graph, *iter);
					float edge_cost = NavGraph::GetEdgeCost(graph, nodes[closest], *iter);
					float g_cost = g_costs[closest] + edge_cost;

					unsigned int index = node_to_index[*iter];

					// if this is the first time we are examining a node, we must store the cost we computed
					// if we've already examined it, we only store the new value if it's cheaper than what we stored before
					if(search_frontier[index] == NULL || g_cost < g_costs[index] && shortest_path_tree[index] == NULL)
					{
						float h_cost = (pos_a - pos_b).ComputeMagnitude();				// our heuristic
						f_costs[index] = g_cost + h_cost;
						g_costs[index] = g_cost;
						if(search_frontier[index] == NULL)
							pq.Insert(index);
						else
						{
							pq.ChangePriority(index);
							delete search_frontier[index];
						}
						search_frontier[index] = new Edge(nodes[closest], edge_cost);
					}
				}

				iteration++;
			}

			if(pq.Empty())
				finished = true;

			if(finished)
			{
				// collect result path into a list
				list<unsigned int> results;
				unsigned int cur = target;
				while(cur != source)
				{
					int index = node_to_index[cur];
					Edge* edge = shortest_path_tree[index];
					if(edge != NULL)
					{
						results.push_back(cur);
						cur = edge->to;
					}
					else
					{
						results.clear();
						break;
					}
				}
				results.reverse();

				// delete new'd stuffs
				for(vector<Edge*>::iterator iter = search_frontier.begin(); iter != search_frontier.end(); iter++)
					delete *iter;

				// store the path we computed
				solution = results;
			}
		}
	};




	/*
	 * PathSearch methods
	 */
	PathSearch::PathSearch(unsigned int graph, unsigned int source, unsigned int target) :
		imp(new Imp(graph, source, target))
	{
	}

	void PathSearch::Dispose()
	{
		delete imp;
		imp = NULL;
	}

	void PathSearch::Think(int steps) { imp->Think(steps); }

	void PathSearch::Solve() { imp->Think(-1); }

	int PathSearch::GetGraph() { return imp->graph; }
	int PathSearch::GetSource() { return imp->source; }
	int PathSearch::GetTarget() { return imp->target; }

	bool PathSearch::IsFinished() { return imp->finished; }

	list<unsigned int> PathSearch::GetSolution()
	{
		list<unsigned int> result;
		for(list<unsigned int>::iterator iter = imp->solution.begin(); iter != imp->solution.end(); iter++)
			result.push_back(*iter);
		return result;
	}




	/*
	 * Scripting stuff
	 */
	int pathsearch_gc(lua_State* L);
	int pathsearch_index(lua_State* L);

	void PushPathSearchHandle(lua_State* L, unsigned int graph, unsigned int source, unsigned int target)
	{
		int n = lua_gettop(L);

		PathSearch* ptr = (PathSearch*)lua_newuserdata(L, sizeof(PathSearch));//new PathSearch(graph, source, target);
		*ptr = PathSearch(graph, source, target);

		lua_getglobal(L, "PathSearchMeta");
		if(lua_isnil(L, n + 2))
		{
			lua_pop(L, 1);
			// must create metatable for globals
			lua_newtable(L);	
			
			lua_pushcclosure(L, pathsearch_gc, 0);
			lua_setfield(L, n + 2, "__gc");

			lua_pushcclosure(L, pathsearch_index, 0);
			lua_setfield(L, n + 2, "__index");
			
			lua_setglobal(L, "PathSearchMeta");
			lua_getglobal(L, "PathSearchMeta");
		}
		lua_setmetatable(L, n + 1);							// set field of 1; pop; top = 1
	}

	int pathsearch_gc(lua_State* L)
	{
		PathSearch* path_search = (PathSearch*)lua_touserdata(L, 1);
		path_search->Dispose();

		lua_settop(L, 0);
		return 0;
	}

	int pathsearch_think(lua_State* L)
	{
		PathSearch* path_search = (PathSearch*)lua_touserdata(L, lua_upvalueindex(1));
		
		int n = lua_gettop(L);
		if(n == 1)
			if(lua_isnumber(L, 1))
			{
				path_search->Think((int)lua_tonumber(L, 1));

				lua_settop(L, 0);
				return 0;
			}

		Debug("pathsearch.think takes exactly one parameter, an int\n");
		return 0;
	}

	int pathsearch_index(lua_State* L)
	{
		PathSearch* path_search = (PathSearch*)lua_touserdata(L, 1);

		if(lua_isstring(L, 2))
		{
			string key = lua_tostring(L, 2);

			lua_settop(L, 0);

			if		(key == "finished") { lua_pushboolean(L, path_search->IsFinished()); return 1; }
			else if	(key == "source") { PushNavNodeHandle(L, path_search->GetGraph(), path_search->GetSource()); return 1; }
			else if	(key == "target") { PushNavNodeHandle(L, path_search->GetGraph(), path_search->GetTarget()); return 1; }
			else if (key == "think") { lua_pushlightuserdata(L, path_search); lua_pushcclosure(L, pathsearch_think, 1); return 1; }
			else if	(key == "solution")
			{
				lua_settop(L, 0);

				if(path_search->IsFinished())
				{
					list<unsigned int> solution = path_search->GetSolution();
					int i = 1;

					lua_newtable(L);
					for(list<unsigned int>::iterator iter = solution.begin(); iter != solution.end(); iter++)
					{
						lua_pushnumber(L, i++);
						PushNavNodeHandle(L, path_search->GetGraph(), *iter);
						lua_settable(L, 1);
					}
					return 1;
				}
				
				return 0;
			}
		}

		return 0;
	}
}
