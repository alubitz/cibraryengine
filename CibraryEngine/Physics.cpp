#include "StdAfx.h"
#include "Physics.h"

#include "RigidBody.h"
#include "RayCollider.h"
#include "CollisionGroup.h"

#include "PhysicsRegion.h"
#include "GridRegionManager.h"

#include "RayShape.h"
#include "SphereShape.h"
#include "TriangleMeshShape.h"
#include "InfinitePlaneShape.h"
#include "MultiSphereShape.h"

#include "ConstraintGraph.h"

#include "Matrix.h"

#include "DebugLog.h"
#include "Serialize.h"

#include "RenderNode.h"
#include "SceneRenderer.h"

#include "Util.h"

#include "DebugDrawMaterial.h"

#include "ProfilingTimer.h"

#define MAX_SEQUENTIAL_SOLVER_ITERATIONS 5

#define PHYSICS_TICK_FREQUENCY 60
#define MAX_FIXED_STEPS_PER_UPDATE 1

#define PROFILE_DOFIXEDSTEP 1
#define PROFILE_SOLVE_CGRAPH 0							// not so much skewing from this, though

namespace CibraryEngine
{
	using namespace boost::asio;

	using boost::unordered_set;
	using boost::unordered_map;

	// forward declare a few functions
	static void DoSphereSphere(RigidBody* ibody, RigidBody* jbody, float radius, const Vec3& pos, const Vec3& vel, float timestep, ConstraintGraph& hits);
	static void DoSphereMesh(RigidBody* ibody, RigidBody* jbody, float radius, const Vec3& pos, const Vec3& vel, float timestep, ConstraintGraph& hits);
	static void DoSpherePlane(RigidBody* ibody, RigidBody* jbody, float radius, const Vec3& pos, const Vec3& vel, float timestep, ConstraintGraph& hits);
	static void DoSphereMultisphere(RigidBody* ibody, RigidBody* jbody, float radius, const Vec3& pos, const Vec3& vel, float timestep, ConstraintGraph& hits);

	



	/*
	 * PhysicsWorld orphan callback struct (private)
	 */
	struct PhysicsWorld::MyOrphanCallback : public ObjectOrphanedCallback
	{
		bool shutdown;

		MyOrphanCallback() : shutdown(false) { }

		void OnObjectOrphaned(CollisionObject* object)
		{
			//if(!shutdown)
			//	Debug("An object has been orphaned!\n");
		}
	};




	/*
	 * Subgraph struct used within PhysicsWorld::SolveConstraintGraph
	 */
	struct Subgraph
	{
		static vector<Subgraph*> subgraph_recycle_bin;
		static vector<unordered_map<RigidBody*, ConstraintGraph::Node*>*> nodemaps_recycle_bin;

		unordered_map<RigidBody*, ConstraintGraph::Node*>* nodes;
		vector<PhysicsConstraint*> constraints;

		Subgraph() : constraints()
		{
			if(nodemaps_recycle_bin.empty())
				nodes = new unordered_map<RigidBody*, ConstraintGraph::Node*>();
			else
			{
				unordered_map<RigidBody*, ConstraintGraph::Node*>* result = *nodemaps_recycle_bin.rbegin();
				nodemaps_recycle_bin.pop_back();

				nodes = new (result) unordered_map<RigidBody*, ConstraintGraph::Node*>();
			}
		}
		~Subgraph() { if(nodes) { nodes->~unordered_map(); nodemaps_recycle_bin.push_back(nodes); nodes = NULL; } }

		bool ContainsNode(ConstraintGraph::Node* node) { return nodes->find(node->body) != nodes->end(); }

		static Subgraph* New()
		{
			if(subgraph_recycle_bin.empty())
				return new Subgraph();
			else
			{
				Subgraph* result = *subgraph_recycle_bin.rbegin();
				subgraph_recycle_bin.pop_back();

				return new (result) Subgraph();
			}
		}
		static void Delete(Subgraph* s) { s->~Subgraph(); subgraph_recycle_bin.push_back(s); }

		static void EmptyRecycleBins()
		{
			for(vector<Subgraph*>::iterator iter = subgraph_recycle_bin.begin(), bin_end = subgraph_recycle_bin.end(); iter != bin_end; ++iter)
				delete *iter;
			subgraph_recycle_bin.clear();

			for(vector<unordered_map<RigidBody*, ConstraintGraph::Node*>*>::iterator iter = nodemaps_recycle_bin.begin(), bin_end = nodemaps_recycle_bin.end(); iter != bin_end; ++iter)
				delete *iter;
			nodemaps_recycle_bin.clear();
		}
	};
	vector<Subgraph*> Subgraph::subgraph_recycle_bin = vector<Subgraph*>();
	vector<unordered_map<RigidBody*, ConstraintGraph::Node*>*> Subgraph::nodemaps_recycle_bin = vector<unordered_map<RigidBody*, ConstraintGraph::Node*>*>();




#if PROFILE_DOFIXEDSTEP
	static float timer_update_vel = 0.0f;
	static float timer_ray_update = 0.0f;
	static float timer_collide = 0.0f;
	static float timer_constraints = 0.0f;
	static float timer_cgraph = 0.0f;
	static float timer_update_pos = 0.0f;
	static float timer_total = 0.0f;
	static unsigned int counter_dofixedstep = 0;
#endif

#if PROFILE_SOLVE_CGRAPH
	static float timer_cgraph_whole = 0.0f;
	static float timer_cgraph_setup = 0.0f;
	static float timer_cgraph_shutdown = 0.0f;
	static float timer_make_subgraphs = 0.0f;
	static float timer_do_constraints = 0.0f;
	static unsigned int counter_solve_cgraph = 0;
#endif




	/*
	 * PhysicsWorld methods
	 */
	PhysicsWorld::PhysicsWorld() :
		all_objects(),
		rays(),
		dynamic_objects(),
		all_regions(),
		region_man(NULL),
		gravity(0, -9.8f, 0),
		internal_timer(),
		timer_interval(1.0f / PHYSICS_TICK_FREQUENCY),
		orphan_callback(new MyOrphanCallback())
	{
		region_man = new GridRegionManager(&all_regions, orphan_callback);
	}
	void PhysicsWorld::InnerDispose()
	{
		// suppress "object orphaned" messages
		orphan_callback->shutdown = true;

		// dispose physics regions
		for(unordered_set<PhysicsRegion*>::iterator iter = all_regions.begin(), regions_end = all_regions.end(); iter != regions_end; ++iter)
		{
			(*iter)->Dispose();
			delete *iter;
		}
		all_regions.clear();

		// dispose collision objects
		for(unordered_set<CollisionObject*>::iterator iter = all_objects.begin(), objects_end = all_objects.end(); iter != objects_end; ++iter)
		{
			(*iter)->Dispose();
			delete *iter;
		}
		all_objects.clear();

		rays.clear();
		dynamic_objects.clear();

		delete region_man;
		region_man = NULL;

		delete orphan_callback;
		orphan_callback = NULL;

		Subgraph::EmptyRecycleBins();
		ConstraintGraph::EmptyRecycleBins();

#if PROFILE_DOFIXEDSTEP
		Debug(((stringstream&)(stringstream() << "total for " << counter_dofixedstep << " calls to PhysicsWorld::DoFixedStep = " << timer_total << endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "update_vel =\t\t\t"		<< timer_update_vel						<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "ray_update =\t\t\t"		<< timer_ray_update						<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "collide =\t\t\t\t"		<< timer_collide						<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "constraints =\t\t\t"		<< timer_constraints					<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "cgraph =\t\t\t\t"			<< timer_cgraph							<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "update_pos =\t\t\t"		<< timer_update_pos						<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "total of above =\t\t"		<< timer_update_vel + timer_ray_update + timer_collide + timer_constraints + timer_cgraph + timer_update_pos << endl)).str());
#endif

#if PROFILE_SOLVE_CGRAPH
		Debug(((stringstream&)(stringstream() << "total for " << counter_solve_cgraph << " calls to SolveConstraintGraph = " << timer_cgraph_whole << endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "cgraph_setup =\t\t\t"		<< timer_cgraph_setup					<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "make_subgraphs =\t\t"		<< timer_make_subgraphs					<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "do_constraints =\t\t"		<< timer_do_constraints					<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "cgraph_shutdown =\t\t"	<< timer_cgraph_shutdown				<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "total of above =\t\t"		<< timer_cgraph_setup + timer_make_subgraphs + timer_do_constraints + timer_cgraph_shutdown << endl)).str());
#endif
	}



	float PhysicsWorld::GetUseMass(RigidBody* ibody, RigidBody* jbody, const Vec3& position, const Vec3& direction, float& B)
	{
		float A;
		B = 0;

		if(jbody->can_move)
		{
			A = ibody->inv_mass + jbody->inv_mass;
			B = Vec3::Dot(ibody->vel, direction) - Vec3::Dot(jbody->vel, direction);
		}
		else
		{
			A = ibody->inv_mass;
			B = Vec3::Dot(ibody->vel, direction);
		}

		if(ibody->can_rotate)
		{
			Vec3 nr1 = Vec3::Cross(direction, position - ibody->cached_com);
			A += Vec3::Dot(ibody->inv_moi * nr1, nr1);
			B += Vec3::Dot(ibody->rot, nr1);
		}

		if(jbody->can_rotate)
		{
			Vec3 nr2 = Vec3::Cross(direction, position - jbody->cached_com);
			A += Vec3::Dot(jbody->inv_moi * nr2, nr2);
			B -= Vec3::Dot(jbody->rot, nr2);
		}

		return 1.0f / A;
	}

	float PhysicsWorld::GetUseMass(RigidBody* ibody, RigidBody* jbody, const Vec3& position, const Vec3& direction)
	{
		float A = jbody->can_move ? ibody->inv_mass + jbody->inv_mass : ibody->inv_mass;

		if(ibody->can_rotate)
		{
			Vec3 nr1 = Vec3::Cross(direction, position - ibody->cached_com);
			A += Vec3::Dot(ibody->inv_moi * nr1, nr1);
		}

		if(jbody->can_rotate)
		{
			Vec3 nr2 = Vec3::Cross(direction, position - jbody->cached_com);
			A += Vec3::Dot(jbody->inv_moi * nr2, nr2);
		}

		return 1.0f / A;
	}

	float PhysicsWorld::GetUseMass(RayCollider* collider, RigidBody* body, const Vec3& position, const Vec3& direction, float& B)
	{
		float A;
		B = 0;

		if(body->can_move)
		{
			A = collider->inv_mass + body->inv_mass;
			B = Vec3::Dot(collider->vel, direction) - Vec3::Dot(body->vel, direction);
		}
		else
		{
			A = collider->inv_mass;
			B = Vec3::Dot(collider->vel, direction);
		}

		if(body->can_rotate)
		{
			Vec3 nr2 = Vec3::Cross(direction, position - body->cached_com);
			A += Vec3::Dot(body->inv_moi * nr2, nr2);
			B -= Vec3::Dot(body->rot, nr2);
		}

		return 1.0f / A;
	}

	float PhysicsWorld::GetUseMass(RayCollider* collider, RigidBody* body, const Vec3& position, const Vec3& direction)
	{
		float A;

		if(body->can_move)
			A = collider->inv_mass + body->inv_mass;
		else
			A = collider->inv_mass;

		if(body->can_rotate)
		{
			Vec3 nr2 = Vec3::Cross(direction, position - body->cached_com);
			A += Vec3::Dot(body->inv_moi * nr2, nr2);
		}

		return 1.0f / A;
	}

	void PhysicsWorld::SolveConstraintGraph(ConstraintGraph& graph)
	{
#if PROFILE_SOLVE_CGRAPH
		ProfilingTimer timer, timer2;
		timer2.Start();
		timer.Start();
#endif

		unsigned int graph_nodes = graph.nodes.size();

		// break the graph into separate subgraphs
		vector<Subgraph*> subgraphs;
		subgraphs.reserve(graph_nodes);

		unordered_map<RigidBody*, Subgraph*> body_subgraphs;
		body_subgraphs.rehash((int)ceil(graph_nodes / body_subgraphs.max_load_factor()));

		vector<ConstraintGraph::Node*> fringe(graph_nodes);

#if PROFILE_SOLVE_CGRAPH
		timer_cgraph_setup += timer.GetAndRestart();
#endif

		for(unordered_map<RigidBody*, ConstraintGraph::Node*>::iterator iter = graph.nodes.begin(), nodes_end = graph.nodes.end(); iter != nodes_end; ++iter)
		{
			if(body_subgraphs.find(iter->first) == body_subgraphs.end())
			{
				Subgraph* subgraph = Subgraph::New();
				subgraphs.push_back(subgraph);

				fringe.clear();
				fringe.push_back(iter->second);

				while(!fringe.empty())
				{
					ConstraintGraph::Node* node = *fringe.rbegin();
					fringe.pop_back();

					if(!subgraph->ContainsNode(node))
					{
						subgraph->nodes->operator[](node->body) = node;
						body_subgraphs[node->body] = subgraph;

						const vector<ConstraintGraph::Edge>& edges = *node->edges;
						for(vector<ConstraintGraph::Edge>::const_iterator jter = edges.begin(), edges_end = edges.end(); jter != edges_end; ++jter)
						{
							ConstraintGraph::Node* other = jter->other_node;
							if(other == NULL || !subgraph->ContainsNode(other))
							{
								subgraph->constraints.push_back(jter->constraint);

								if(other)
									fringe.push_back(other);
							}
						}
					}
				}
			}
		}

#if PROFILE_SOLVE_CGRAPH
		timer_make_subgraphs += timer.GetAndRestart();
#endif

		vector<PhysicsConstraint*> active;
		unordered_set<PhysicsConstraint*> nu_active;
		vector<RigidBody*> wakeup_list;

		// now go through each subgraph and do as many iterations as are necessary
		for(vector<Subgraph*>::iterator iter = subgraphs.begin(), subgraphs_end = subgraphs.end(); iter != subgraphs_end; ++iter)
		{
			Subgraph& subgraph = **iter;

			active.assign(subgraph.constraints.begin(), subgraph.constraints.end());

			for(int i = 0; i < MAX_SEQUENTIAL_SOLVER_ITERATIONS && !active.empty(); ++i)
			{
				nu_active.clear();
				for(vector<PhysicsConstraint*>::iterator jter = active.begin(), active_end = active.end(); jter != active_end; ++jter)
				{
					PhysicsConstraint& constraint = **jter;

					wakeup_list.clear();
					constraint.DoConstraintAction(wakeup_list);

					// constraint says we should wake up these rigid bodies
					for(vector<RigidBody*>::iterator kter = wakeup_list.begin(), wakeup_end = wakeup_list.end(); kter != wakeup_end; ++kter)
					{
						ConstraintGraph::Node* node = subgraph.nodes->operator[](*kter);

						for(vector<ConstraintGraph::Edge>::iterator kter = node->edges->begin(), edges_end = node->edges->end(); kter != edges_end; ++kter)
							if(kter->constraint != *jter)
								nu_active.insert(kter->constraint);
					}
				}

				active.assign(nu_active.begin(), nu_active.end());
			}
		}

#if PROFILE_SOLVE_CGRAPH
		timer_do_constraints += timer.GetAndRestart();
#endif

		// clean up subgraphs
		for(vector<Subgraph*>::iterator iter = subgraphs.begin(), subgraphs_end = subgraphs.end(); iter != subgraphs_end; ++iter)
			Subgraph::Delete(*iter);

#if PROFILE_SOLVE_CGRAPH
		timer_cgraph_shutdown += timer.Stop();
		timer_cgraph_whole += timer2.Stop();

		++counter_solve_cgraph;
#endif
	}



	void PhysicsWorld::DoFixedStep()
	{
		float timestep = timer_interval;

#if PROFILE_DOFIXEDSTEP
		ProfilingTimer timer, timer2;
		timer2.Start();
		timer.Start();
#endif

		// set forces to what was applied by gravity / user forces; also the objects may deactivate now
		for(unordered_set<CollisionObject*>::iterator iter = dynamic_objects.begin(), objects_end = dynamic_objects.end(); iter != objects_end; ++iter)
			(*iter)->UpdateVel(timestep);
		for(unordered_set<RayCollider*>::iterator iter = rays.begin(), rays_end = rays.end(); iter != rays_end; ++iter)
			(*iter)->UpdateVel(timestep);

#if PROFILE_DOFIXEDSTEP
		timer_update_vel += timer.GetAndRestart();
#endif

		// handle all the collisions involving rays
		struct MyRayCallback : public RayCallback
		{
			PhysicsWorld* world;
			MyRayCallback(PhysicsWorld* world) : world(world) { }

			bool OnCollision(RayResult& rr)			// return value controls whether to continue iterating through ray collisions
			{
				RayCallback* callback = rr.collider->GetRayCallback();

				if(callback && !callback->OnCollision(rr))
					return true;
				else
				{
					rr.collider->DoCollisionResponse(rr);
					return true;
				}
			}
		} ray_callback(this);

		for(unordered_set<RayCollider*>::iterator iter = rays.begin(), rays_end = rays.end(); iter != rays_end; ++iter)
		{
			RayCollider* collider = (RayCollider*)*iter;
			RayTestPrivate(collider->pos, collider->pos + collider->vel, ray_callback, timestep, collider);
		}

#if PROFILE_DOFIXEDSTEP
		timer_ray_update += timer.Stop();
#endif

		// populate a constraint graph with all the collisions that are going on
		ConstraintGraph constraint_graph;

#if PROFILE_DOFIXEDSTEP
		timer.Start();
#endif

		vector<ContactPoint> contact_points;
		for(unordered_set<CollisionObject*>::iterator iter = dynamic_objects.begin(), objects_end = dynamic_objects.end(); iter != objects_end; ++iter)
		{
			(*iter)->InitiateCollisions(timestep, contact_points);

			for(vector<ContactPoint>::iterator jter = contact_points.begin(), cp_end = contact_points.end(); jter != cp_end; ++jter)
				constraint_graph.AddContactPoint(*jter);
			contact_points.clear();
		}

#if PROFILE_DOFIXEDSTEP
		timer_collide += timer.GetAndRestart();
#endif

		for(vector<ContactPoint*>::iterator iter = constraint_graph.contact_points.begin(), cp_end = constraint_graph.contact_points.end(); iter != cp_end; ++iter)
		{
			(*iter)->DoUpdateAction(timestep);
			constraint_graph.AddConstraint(*iter);
		}

		for(unordered_set<PhysicsConstraint*>::iterator iter = all_constraints.begin(), constraints_end = all_constraints.end(); iter != constraints_end; ++iter)
		{
			(*iter)->DoUpdateAction(timestep);
			constraint_graph.AddConstraint(*iter);
		}

#if PROFILE_DOFIXEDSTEP
		timer_constraints += timer.GetAndRestart();
#endif

		SolveConstraintGraph(constraint_graph);

#if PROFILE_DOFIXEDSTEP
		timer_cgraph += timer.GetAndRestart();
#endif

		// update positions
		for(unordered_set<CollisionObject*>::iterator iter = dynamic_objects.begin(), objects_end = dynamic_objects.end(); iter != objects_end; ++iter)
			(*iter)->UpdatePos(timestep, region_man);
		for(unordered_set<RayCollider*>::iterator iter = rays.begin(), rays_end = rays.end(); iter != rays_end; ++iter)
			(*iter)->UpdatePos(timestep, region_man);

#if PROFILE_DOFIXEDSTEP
		timer_update_pos += timer.Stop();
		timer_total += timer2.Stop();

		++counter_dofixedstep;
#endif
	}



	void PhysicsWorld::AddCollisionObject(CollisionObject* obj)
	{
		all_objects.insert(obj);
		switch(obj->GetType())
		{
			case COT_RigidBody:
			{
				RigidBody* r = (RigidBody*)obj;
				if(r->can_move)
					dynamic_objects.insert(r);

				r->gravity = gravity;

				break;
			}

			case COT_RayCollider:
			{
				rays.insert((RayCollider*)obj);
				break;
			}

			case COT_CollisionGroup:
			{
				CollisionGroup* cgroup = (CollisionGroup*)obj;
				dynamic_objects.insert(cgroup);

				cgroup->SetGravity(gravity);

				break;
			}
		}

		region_man->OnObjectAdded(obj, obj->regions);
	}

	void PhysicsWorld::RemoveCollisionObject(CollisionObject* obj)
	{
		all_objects.erase(obj);

		CollisionObjectType type = obj->GetType();
		switch(type)
		{
			case COT_RigidBody:
			{
				if(obj->can_move)
					dynamic_objects.erase(obj);
				break;
			}

			case COT_RayCollider:
			{
				rays.erase((RayCollider*)obj);
				break;
			}
		}

		region_man->OnObjectRemoved(obj, obj->regions);
		obj->regions.Clear();

		const set<CollisionObject*>& disabled_collisions = obj->disabled_collisions;
		for(set<CollisionObject*>::iterator iter = disabled_collisions.begin(), disabled_end = disabled_collisions.end(); iter != disabled_end; ++iter)
			(*iter)->disabled_collisions.erase(obj);
		obj->disabled_collisions.clear();

		if(type == COT_RigidBody)
		{
			RigidBody* r = (RigidBody*)obj;
			for(set<PhysicsConstraint*>::iterator iter = r->constraints.begin(), constraints_end = r->constraints.end(); iter != constraints_end; ++iter)
			{
				PhysicsConstraint* c = *iter;
				if(c->obj_a == r && c->obj_b)
					c->obj_b->constraints.erase(c);
				else if(c->obj_b == r && c->obj_a)
					c->obj_a->constraints.erase(c);

				all_constraints.erase(c);

				// TODO: figure out where the constraint will be deleted?
			}
		}
	}

	void PhysicsWorld::AddConstraint(PhysicsConstraint* c)
	{
		if(c->obj_a)
			c->obj_a->constraints.insert(c);
		if(c->obj_b)
			c->obj_b->constraints.insert(c);
		all_constraints.insert(c);
	}

	void PhysicsWorld::RemoveConstraint(PhysicsConstraint* c)
	{
		if(c->obj_a)
			c->obj_a->constraints.erase(c);
		if(c->obj_b)
			c->obj_b->constraints.erase(c);

		all_constraints.erase(c);
	}

	void PhysicsWorld::Update(TimingInfo time)
	{
		internal_timer += time.elapsed;
		for(int i = 0; internal_timer >= timer_interval && i < MAX_FIXED_STEPS_PER_UPDATE; ++i)
		{
			DoFixedStep();
			internal_timer -= timer_interval;
		}

		for(unordered_set<CollisionObject*>::iterator iter = dynamic_objects.begin(), objects_end = dynamic_objects.end(); iter != objects_end; ++iter)
			(*iter)->ResetForces();
		for(unordered_set<RayCollider*>::iterator iter = rays.begin(), rays_end = rays.end(); iter != rays_end; ++iter)
			(*iter)->ResetForces();
	}

	void PhysicsWorld::DebugDrawWorld(SceneRenderer* renderer)
	{
		for(unordered_set<CollisionObject*>::iterator iter = all_objects.begin(), objects_end = all_objects.end(); iter != objects_end; ++iter)
			(*iter)->DebugDraw(renderer);

		renderer->Render();
		renderer->Cleanup();
	}

	Vec3 PhysicsWorld::GetGravity() { return gravity; }
	void PhysicsWorld::SetGravity(const Vec3& gravity_)
	{
		gravity = gravity_;

		for(unordered_set<CollisionObject*>::iterator iter = dynamic_objects.begin(); iter != dynamic_objects.end(); ++iter)
			((RigidBody*)*iter)->gravity = gravity_;
		for(unordered_set<RayCollider*>::iterator iter = rays.begin(); iter != rays.end(); ++iter)
			((RayCollider*)*iter)->gravity = gravity_;
	}

	void PhysicsWorld::RayTestPrivate(const Vec3& from, const Vec3& to, RayCallback& callback, float max_time, RayCollider* collider)
	{
		// find out what objects are relevant
		set<PhysicsRegion*> regions;
		Vec3 endpoint = from + (to - from) * max_time;

		region_man->GetRegionsOnRay(from, endpoint, regions);

		AABB ray_aabb(from);
		ray_aabb.Expand(endpoint);

		static RelevantObjectsQuery relevant_objects;

		for(set<PhysicsRegion*>::iterator iter = regions.begin(); iter != regions.end(); ++iter)
			(*iter)->GetRelevantObjects(ray_aabb, relevant_objects);

		// now do the actual collision testing
		Ray ray(from, to - from);

		list<RayResult> hits;
		for(unsigned int i = 0; i < RelevantObjectsQuery::hash_size; ++i)
		{
			vector<CollisionObject*>& bucket = relevant_objects.buckets[i];
			for(vector<CollisionObject*>::iterator iter = bucket.begin(), bucket_end = bucket.end(); iter != bucket_end; ++iter)
			{
				CollisionObject* cobj = *iter;
				switch(cobj->GetType())
				{
					case COT_RigidBody:			RayCollider::CollideRigidBody(		(RigidBody*)*iter,		ray, max_time, hits, collider); break;
					case COT_CollisionGroup:	RayCollider::CollideCollisionGroup(	(CollisionGroup*)*iter,	ray, max_time, hits, collider); break;
				}
			}
		}

		// run the collision callback on whatever we found
		if(!hits.empty())
		{
			hits.sort();

			for(list<RayResult>::iterator jter = hits.begin(), hits_end = hits.end(); jter != hits_end; ++jter)
				if(callback.OnCollision(*jter))
					break;
		}

		relevant_objects.Clear();
	}
	void PhysicsWorld::RayTest(const Vec3& from, const Vec3& to, RayCallback& callback) { RayTestPrivate(from, to, callback); }




	/*
	 * ContactPoint methods
	 */
	void ContactPoint::BuildCache()
	{
		if(!cache_valid)
		{
			use_pos = (a.pos + b.pos) * 0.5f;
			normal = Vec3::Normalize(a.norm - b.norm);

			if(obj_a)
			{
				bounciness = obj_a->bounciness * obj_b->bounciness;	
				sfric_coeff = obj_a->friction * obj_b->friction;
				moi_n = Mat3::Invert(obj_a->inv_moi + obj_b->inv_moi) * normal;
			}
			else			// for ray-tests
			{
				bounciness = obj_b->bounciness;	
				sfric_coeff = obj_b->friction;
				moi_n = Mat3::Invert(obj_b->inv_moi) * normal;
			}

			kfric_coeff = sfric_coeff;			// would be * 0.9f but in practice the sim treats everything as kinetic anyway

			cache_valid = true;
		}
	}

	bool ContactPoint::DoCollisionResponse() const
	{
		assert(cache_valid);

		RigidBody* ibody = obj_a;
		RigidBody* jbody = obj_b;

		bool j_can_move = jbody->can_move;

		Vec3 dv = obj_b->GetLocalVelocity(use_pos) - obj_a->GetLocalVelocity(use_pos);
		float nvdot = Vec3::Dot(normal, dv);
		if(nvdot < 0.0f)
		{
			float B;
			float use_mass = PhysicsWorld::GetUseMass(ibody, jbody, use_pos, normal, B);
			float impulse_mag = -(1.0f + bounciness) * B * use_mass;

			if(impulse_mag < 0)
			{
				Vec3 impulse = normal * impulse_mag;

				// normal force
				if(impulse.ComputeMagnitudeSquared() != 0)
				{
					ibody->ApplyWorldImpulse(impulse, use_pos);
					if(j_can_move)
						jbody->ApplyWorldImpulse(-impulse, use_pos);

					// applying this impulse means we need to recompute dv and nvdot!
					dv = obj_b->GetLocalVelocity(use_pos) - obj_a->GetLocalVelocity(use_pos);
					nvdot = Vec3::Dot(normal, dv);
				}

				Vec3 t_dv = dv - normal * nvdot;
				float t_dv_magsq = t_dv.ComputeMagnitudeSquared();

				// linear friction
				if(t_dv_magsq > 0)								// object is moving; apply kinetic friction
				{
					float t_dv_mag = sqrtf(t_dv_magsq), inv_tdmag = 1.0f / t_dv_mag;
					Vec3 u_tdv = t_dv * inv_tdmag;

					use_mass = PhysicsWorld::GetUseMass(ibody, jbody, use_pos, u_tdv);

					Vec3 fric_impulse = t_dv * min(use_mass, fabs(impulse_mag * kfric_coeff * inv_tdmag));

					ibody->ApplyWorldImpulse(fric_impulse, use_pos);
					if(j_can_move)
						jbody->ApplyWorldImpulse(-fric_impulse, use_pos);
				}
				else											// object isn't moving; apply static friction
				{
					Vec3 df = jbody->applied_force - ibody->applied_force;
					float nfdot = Vec3::Dot(normal, df);

					Vec3 t_df = df - normal * nfdot;
					float t_df_mag = t_df.ComputeMagnitude();

					float fric_i_mag = min(impulse_mag * sfric_coeff, t_df_mag);
					if(fric_i_mag > 0)
					{
						Vec3 fric_impulse = t_df * (-fric_i_mag / t_df_mag);

						ibody->ApplyWorldImpulse(fric_impulse, use_pos);
						if(j_can_move)
							jbody->ApplyWorldImpulse(-fric_impulse, use_pos);
					}
				}

				// TODO: make angular friction less wrong
#if 0
				// angular friction (wip; currently completely undoes angular velocity around the normal vector)
				float angular_dv = Vec3::Dot(normal, obj_b->rot - obj_a->rot);
				if(fabs(angular_dv) > 0)
				{
					Vec3 angular_impulse = moi_n * angular_dv;

					ibody->ApplyAngularImpulse(angular_impulse);
					if(j_can_move && jbody->can_rotate)
						jbody->ApplyAngularImpulse(-angular_impulse);
				}
#endif

				if(j_can_move)
					jbody->active = true;

				return true;
			}
		}

		return false;
	}

	void ContactPoint::DoConstraintAction(vector<RigidBody*>& wakeup_list)
	{
		BuildCache();

		if(DoCollisionResponse())
		{
			// because we applied an impulse, we should wake up edges for the rigid bodies involved
			wakeup_list.push_back(obj_a);
			if(obj_b->MergesSubgraphs())
				wakeup_list.push_back(obj_b);

			if(obj_a->collision_callback)
				obj_a->collision_callback->OnCollision(*this);
			if(obj_b->collision_callback)
				obj_b->collision_callback->OnCollision(*this);
		}
	}

	void ContactPoint::DoUpdateAction(float timestep)
	{
		// magical anti-penetration displacement! directly modifies position, instead of working with velocity
		Vec3 dx = b.pos - a.pos;

		static const float undo_penetration_coeff = 8.0f;

		if(dx.x || dx.y || dx.z)
		{
			static float saved_timestep = timestep - 1;				// make sure first initialization triggers the if... could instead init move_frac in two places?
			static float move_frac;

			if(timestep != saved_timestep)
			{
				saved_timestep = timestep;
				move_frac = 1.0f - exp(-undo_penetration_coeff * timestep);
			}

			if(!obj_b->can_move)
			{
				obj_a->pos -= dx * move_frac;
				obj_a->xform_valid = false;
			}
			else
			{
				float total = obj_a->inv_mass + obj_b->inv_mass, inv_total = move_frac / total;

				obj_a->pos -= dx * (inv_total * obj_a->inv_mass);
				obj_b->pos += dx * (inv_total * obj_b->inv_mass);

				obj_a->xform_valid = false;
				obj_b->xform_valid = false;
			}
		}
	}




	/*
	 * SphereShape collision functions
	 */
	static void DoSphereSphere(RigidBody* ibody, RigidBody* jbody, float radius, const Vec3& pos, const Vec3& vel, float timestep, ConstraintGraph& hits)
	{
		float sr = radius + ((SphereShape*)jbody->GetCollisionShape())->radius;

		Vec3 other_pos = jbody->GetPosition();
		Vec3 other_vel = jbody->GetLinearVelocity();

		Ray ray(pos - other_pos, vel - other_vel);

		float first = 0, second = 0;
		if(ray.origin.ComputeMagnitudeSquared() < sr * sr || Util::RaySphereIntersect(ray, Sphere(Vec3(), sr), first, second))
			if(first >= 0 && first <= timestep)
			{
				ContactPoint p;

				p.obj_a = ibody;
				p.obj_b = jbody;
				p.a.pos = pos + first * vel;
				p.b.pos = other_pos + first * other_vel;
				p.b.norm = Vec3::Normalize(p.a.pos - other_pos, 1.0f);
				p.a.norm = -p.b.norm;

				hits.AddContactPoint(p);
			}
	}

	static void DoSphereMesh(RigidBody* ibody, RigidBody* jbody, float radius, const Vec3& pos, const Vec3& vel, float timestep, ConstraintGraph& hits)
	{
		TriangleMeshShape* shape = (TriangleMeshShape*)jbody->GetCollisionShape();

		Ray ray(pos, vel);

		vector<unsigned int> relevant_triangles;
		shape->GetRelevantTriangles(AABB(pos, radius), relevant_triangles);

		for(vector<unsigned int>::iterator kter = relevant_triangles.begin(), triangles_end = relevant_triangles.end(); kter != triangles_end; ++kter)
		{
			const TriangleMeshShape::TriCache& tri = shape->GetTriangleData(*kter);

			float dist = tri.DistanceToPoint(pos);
			if(dist < radius)
			{
				ContactPoint p;

				p.obj_a = ibody;
				p.obj_b = jbody;
				p.a.pos = pos - tri.plane.normal * radius;
				p.b.pos = p.a.pos;
				p.a.norm = -tri.plane.normal;
				p.b.norm = tri.plane.normal;

				hits.AddContactPoint(p);
			}
		}
	}

	static void DoSpherePlane(RigidBody* ibody, RigidBody* jbody, float radius, const Vec3& pos, const Vec3& vel, float timestep, ConstraintGraph& hits)
	{
		InfinitePlaneShape* shape = (InfinitePlaneShape*)jbody->GetCollisionShape();

		Vec3 plane_norm = shape->plane.normal;
		float plane_offset = shape->plane.offset;

		float center_offset = Vec3::Dot(plane_norm, pos) - plane_offset;
		float vel_dot = Vec3::Dot(plane_norm, vel);

		// if the sphere is moving away from the plane, don't bounce!
		if(vel_dot <= 0.0f)
		{
			float dist = fabs(center_offset) - radius;
			float t = center_offset - radius < 0.0f ? 0.0f : dist / fabs(vel_dot);
			if(t >= 0 && t <= timestep)
			{
				ContactPoint p;

				p.obj_a = ibody;
				p.obj_b = jbody;
				p.a.pos = pos - plane_norm * radius;
				p.b.pos = p.a.pos - plane_norm * (Vec3::Dot(p.a.pos, plane_norm) - plane_offset);
				p.b.norm = plane_norm;
				p.a.norm = -plane_norm;

				hits.AddContactPoint(p);
			}
		}
	}

	static void DoSphereMultisphere(RigidBody* ibody, RigidBody* jbody, float radius, const Vec3& pos, const Vec3& vel, float timestep, ConstraintGraph& hits)
	{
		MultiSphereShape* shape = (MultiSphereShape*)jbody->GetCollisionShape();

		Mat4 inv_xform = Mat4::Invert(jbody->GetTransformationMatrix());
		Vec3 nu_pos = inv_xform.TransformVec3_1(pos);

		ContactPoint cp;
		if(shape->CollideSphere(Sphere(nu_pos, radius), cp, ibody, jbody))
			hits.AddContactPoint(cp);
	}
}
