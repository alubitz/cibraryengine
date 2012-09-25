#include "StdAfx.h"
#include "Physics.h"

#include "RigidBody.h"
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

#define PROFILE_DOFIXEDSTEP 0
#define PROFILE_COLLIDE_MSPHERE 0						// warning: if both are enabled, the results for DoFixedStep will be skewed

namespace CibraryEngine
{
	using boost::unordered_set;
	using boost::unordered_map;

	// forward declare a few functions
	static void DoRaySphere(RigidBody* ibody, RigidBody* jbody, const Ray& ray, float max_time, list<RayResult>& hits);
	static void DoRayMesh(RigidBody* ibody, RigidBody* jbody, const Ray& ray, float max_time, list<RayResult>& hits);
	static void DoRayPlane(RigidBody* ibody, RigidBody* jbody, const Ray& ray, float max_time, list<RayResult>& hits);
	static void DoRayMultisphere(RigidBody* ibody, RigidBody* jbody, const Ray& ray, float max_time, list<RayResult>& hits);

	static void DoSphereSphere(RigidBody* ibody, RigidBody* jbody, float radius, const Vec3& pos, const Vec3& vel, float timestep, ConstraintGraph& hits);
	static void DoSphereMesh(RigidBody* ibody, RigidBody* jbody, float radius, const Vec3& pos, const Vec3& vel, float timestep, ConstraintGraph& hits);
	static void DoSpherePlane(RigidBody* ibody, RigidBody* jbody, float radius, const Vec3& pos, const Vec3& vel, float timestep, ConstraintGraph& hits);
	static void DoSphereMultisphere(RigidBody* ibody, RigidBody* jbody, float radius, const Vec3& pos, const Vec3& vel, float timestep, ConstraintGraph& hits);

	static void DoMultisphereMesh(RigidBody* ibody, RigidBody* jbody, MultiSphereShape* ishape, const Mat4& xform, ConstraintGraph& hits);
	static void DoMultispherePlane(RigidBody* ibody, RigidBody* jbody, MultiSphereShape* ishape, const Mat4& xform, ConstraintGraph& hits);
	static void DoMultisphereMultisphere(RigidBody* ibody, RigidBody* jbody, MultiSphereShapeInstanceCache* ishape, MultiSphereShapeInstanceCache* jshape, ConstraintGraph& hits);




	/*
	 * PhysicsWorld orphan callback struct (private)
	 */
	struct PhysicsWorld::MyOrphanCallback : public ObjectOrphanedCallback
	{
		bool shutdown;

		MyOrphanCallback() : shutdown(false) { }

		void OnObjectOrphaned(RigidBody* object)
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
	static float timer_sphere_collide = 0.0f;
	static float timer_msphere_collide = 0.0f;
	static float timer_constraints = 0.0f;
	static float timer_cgraph = 0.0f;
	static float timer_update_pos = 0.0f;
	static float timer_total = 0.0f;
	static unsigned int counter_dofixedstep = 0;
#endif

#if PROFILE_COLLIDE_MSPHERE
	static float timer_init_msphere = 0.0f;
	static float timer_init_msphere_init = 0.0f;
	static float timer_get_relevant = 0.0f;
	static float timer_remove_constrained = 0.0f;
	static float timer_msphere_msphere = 0.0f;
	static float timer_msphere_mesh = 0.0f;
	static float timer_msphere_plane = 0.0f;
	static float timer_conclude = 0.0f;
#endif




	/*
	 * PhysicsWorld methods
	 */
	PhysicsWorld::PhysicsWorld() :
		all_objects(),
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

		// dispose rigid bodies
		for(unsigned int i = 0; i < ST_ShapeTypeMax; ++i)
		{
			for(unordered_set<RigidBody*>::iterator iter = all_objects[i].begin(), objects_end = all_objects[i].end(); iter != objects_end; ++iter)
			{
				(*iter)->Dispose();
				delete *iter;
			}

			all_objects[i].clear();
			dynamic_objects[i].clear();
		}

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
		Debug(((stringstream&)(stringstream() << '\t' << "sphere_collide =\t\t"		<< timer_sphere_collide					<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "msphere_collide =\t\t"	<< timer_msphere_collide				<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "constraints =\t\t\t"		<< timer_constraints					<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "cgraph =\t\t\t\t"			<< timer_cgraph							<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "update_pos =\t\t\t"		<< timer_update_pos						<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "total of above =\t\t"		<< timer_update_vel + timer_ray_update + timer_sphere_collide + timer_msphere_collide + timer_constraints + timer_cgraph + timer_update_pos << endl)).str());
#endif

#if PROFILE_COLLIDE_MSPHERE
		Debug(((stringstream&)(stringstream() << "total within InitiateCollisionsForMultisphere = " << timer_init_msphere	<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "init_msphere_init =\t\t"	<< timer_init_msphere_init				<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "get_relevant =\t\t\t"		<< timer_get_relevant					<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "remove_constrained =\t"	<< timer_remove_constrained				<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "msphere_msphere =\t\t"	<< timer_msphere_msphere				<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "msphere_mesh =\t\t\t"		<< timer_msphere_mesh					<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "msphere_plane =\t\t\t"	<< timer_msphere_plane					<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "conclude =\t\t\t\t"		<< timer_conclude						<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "total of above =\t\t"		<< timer_init_msphere_init + timer_get_relevant + timer_remove_constrained + timer_msphere_msphere + timer_msphere_mesh + timer_msphere_plane + timer_conclude << endl)).str());
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

	void PhysicsWorld::SolveConstraintGraph(ConstraintGraph& graph)
	{
		unsigned int graph_nodes = graph.nodes.size();

		// break the graph into separate subgraphs
		vector<Subgraph*> subgraphs;
		subgraphs.reserve(graph_nodes);

		unordered_map<RigidBody*, Subgraph*> body_subgraphs;
		body_subgraphs.rehash((int)ceil(graph_nodes / body_subgraphs.max_load_factor()));

		vector<ConstraintGraph::Node*> fringe(graph_nodes);

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

								if(other != NULL)
									fringe.push_back(other);
							}
						}
					}
				}
			}
		}

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

		// clean up subgraphs
		for(vector<Subgraph*>::iterator iter = subgraphs.begin(), subgraphs_end = subgraphs.end(); iter != subgraphs_end; ++iter)
			Subgraph::Delete(*iter);
	}

	void PhysicsWorld::InitiateCollisionsForSphere(RigidBody* body, float timestep, ConstraintGraph& constraint_graph)
	{
		SphereShape* shape = (SphereShape*)body->GetCollisionShape();

		float radius = shape->radius;
		Vec3 pos = body->GetPosition();
		Vec3 vel = body->GetLinearVelocity();

		// find out what might be colliding with us
		AABB aabb(pos, radius);
		aabb.Expand(AABB(pos + vel, radius));

		static RelevantObjectsQuery relevant_objects;

		for(unsigned int i = 0; i < RegionSet::hash_size; ++i)
		{
			vector<PhysicsRegion*>& bucket = body->regions.buckets[i];
			for(vector<PhysicsRegion*>::iterator iter = bucket.begin(), bucket_end = bucket.end(); iter != bucket_end; ++iter)
				(*iter)->GetRelevantObjects(aabb, relevant_objects);
		}

		body->RemoveConstrainedBodies(relevant_objects);

		// do collision detection with those objects
		for(vector<RigidBody*>::iterator iter = relevant_objects.objects.begin(), relevant_end = relevant_objects.objects.end(); iter != relevant_end; ++iter)
			if(RigidBody* other = *iter)
			{
				switch(other->GetShapeType())
				{
					case ST_Sphere:
						if(other < body)
							DoSphereSphere(body, other, radius, pos, vel, timestep, constraint_graph);
						break;

					case ST_TriangleMesh:
						DoSphereMesh(body, other, radius, pos, vel, timestep, constraint_graph);
						break;

					case ST_InfinitePlane:
						DoSpherePlane(body, other, radius, pos, vel, timestep, constraint_graph);
						break;

					case ST_MultiSphere:
						DoSphereMultisphere(body, other, radius, pos, vel, timestep, constraint_graph);
						break;
				}
			}

		relevant_objects.ConcludeQuery();
	}

	void PhysicsWorld::InitiateCollisionsForMultisphere(RigidBody* body, float timestep, ConstraintGraph& constraint_graph)
	{
#if PROFILE_COLLIDE_MSPHERE
		ProfilingTimer timer, timer2;
		timer.Start();
		timer2.Start();
#endif
		MultiSphereShape* shape = (MultiSphereShape*)body->GetCollisionShape();

		MultiSphereShapeInstanceCache* cache = (MultiSphereShapeInstanceCache*)body->shape_cache;
		if(!cache)
			body->shape_cache = cache = new MultiSphereShapeInstanceCache();
		cache->UpdateAsNeeded(body);

		Mat4 xform = body->GetTransformationMatrix();
		Mat4 inv_xform = body->GetInvTransform();
		AABB xformed_aabb = shape->GetTransformedAABB(xform);

#if PROFILE_COLLIDE_MSPHERE
		timer_init_msphere_init += timer.GetAndRestart();
#endif

		// find out what might be colliding with us
		static RelevantObjectsQuery relevant_objects;

		for(unsigned int i = 0; i < RegionSet::hash_size; ++i)
		{
			vector<PhysicsRegion*>& bucket = body->regions.buckets[i];
			for(vector<PhysicsRegion*>::iterator iter = bucket.begin(), bucket_end = bucket.end(); iter != bucket_end; ++iter)
				(*iter)->GetRelevantObjects(xformed_aabb, relevant_objects);
		}

#if PROFILE_COLLIDE_MSPHERE
		timer_get_relevant += timer.GetAndRestart();
#endif

		body->RemoveConstrainedBodies(relevant_objects);

#if PROFILE_COLLIDE_MSPHERE
		timer_remove_constrained += timer.GetAndRestart();

		for(vector<RigidBody*>::iterator iter = relevant_objects.objects.begin(), relevant_end = relevant_objects.objects.end(); iter != relevant_end; ++iter)
			if(RigidBody* other = *iter)
				if(other->GetShapeType() == ST_TriangleMesh)
					DoMultisphereMesh(body, other, shape, xform, constraint_graph);

		timer_msphere_mesh += timer.GetAndRestart();

		for(vector<RigidBody*>::iterator iter = relevant_objects.objects.begin(), relevant_end = relevant_objects.objects.end(); iter != relevant_end; ++iter)
			if(RigidBody* other = *iter)
				if(other->GetShapeType() == ST_InfinitePlane)
					DoMultispherePlane(body, other, shape, xform, constraint_graph);

		timer_msphere_plane += timer.GetAndRestart();

		for(vector<RigidBody*>::iterator iter = relevant_objects.objects.begin(), relevant_end = relevant_objects.objects.end(); iter != relevant_end; ++iter)
			if(RigidBody* other = *iter)
				if(other->GetShapeType() == ST_MultiSphere)
					if(other < body)
					{
						MultiSphereShapeInstanceCache* other_cache = (MultiSphereShapeInstanceCache*)other->shape_cache;
						if(!other_cache)
							other->shape_cache = other_cache = new MultiSphereShapeInstanceCache();
						other_cache->UpdateAsNeeded(other);

						DoMultisphereMultisphere(body, other, cache, other_cache, constraint_graph);
					}

		timer_msphere_msphere += timer.GetAndRestart();
#else
		// do collision detection with those objects
		for(vector<RigidBody*>::iterator iter = relevant_objects.objects.begin(), relevant_end = relevant_objects.objects.end(); iter != relevant_end; ++iter)
			if(RigidBody* other = *iter)
			{
				switch(other->GetShapeType())
				{
					case ST_TriangleMesh:
						DoMultisphereMesh(body, other, shape, xform, constraint_graph);
						break;

					case ST_InfinitePlane:
						DoMultispherePlane(body, other, shape, xform, constraint_graph);
						break;

					case ST_MultiSphere:
						if(other < body)
						{
							MultiSphereShapeInstanceCache* other_cache = (MultiSphereShapeInstanceCache*)other->shape_cache;
							if(!other_cache)
								other->shape_cache = other_cache = new MultiSphereShapeInstanceCache();
							other_cache->UpdateAsNeeded(other);

							DoMultisphereMultisphere(body, other, cache, other_cache, constraint_graph);
						}
						break;
				}
			}
#endif

		relevant_objects.ConcludeQuery();

#if PROFILE_COLLIDE_MSPHERE
		timer_conclude += timer.Stop();
		timer_init_msphere += timer2.Stop();
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

		// set forces to what was applied by gravity / user forces; also the objects may deactivate or move between physics regions now
		for(unsigned int i = 1; i < ST_ShapeTypeMax; ++i)
			if(CollisionShape::CanShapeTypeMove((ShapeType)i))
				for(unordered_set<RigidBody*>::iterator iter = dynamic_objects[i].begin(), objects_end = dynamic_objects[i].end(); iter != objects_end; ++iter)
					(*iter)->UpdateVel(timestep);

#if PROFILE_DOFIXEDSTEP
		timer_update_vel += timer.GetAndRestart();
#endif

		// handle all the collisions involving rays
		struct RayCallback : public CollisionCallback
		{
			PhysicsWorld* world;
			RayCallback(PhysicsWorld* world) : world(world) { }

			bool OnCollision(const ContactPoint& cp)			// return value controls whether to continue iterating through ray collisions
			{
				CollisionCallback* callback = cp.obj_a->GetCollisionCallback();

				if(callback && !callback->OnCollision(cp))
					return true;
				else
				{
					cp.DoCollisionResponse();

					return true;
				}
			}
		} ray_callback(this);

		for(unordered_set<RigidBody*>::iterator iter = dynamic_objects[ST_Ray].begin(), rays_end = dynamic_objects[ST_Ray].end(); iter != rays_end; ++iter)
		{
			RigidBody* body = *iter;
			RayTestPrivate(body->pos, body->pos + body->vel, ray_callback, timestep, body);
		}

#if PROFILE_DOFIXEDSTEP
		timer_ray_update += timer.Stop();
#endif

		// populate a constraint graph with all the collisions that are going on
		ConstraintGraph constraint_graph;

#if PROFILE_DOFIXEDSTEP
		timer.Start();
#endif

		for(unordered_set<RigidBody*>::iterator iter = dynamic_objects[ST_Sphere].begin(), spheres_end = dynamic_objects[ST_Sphere].end(); iter != spheres_end; ++iter)
			InitiateCollisionsForSphere(*iter, timestep, constraint_graph);

#if PROFILE_DOFIXEDSTEP
		timer_sphere_collide += timer.GetAndRestart();
#endif

		for(unordered_set<RigidBody*>::iterator iter = dynamic_objects[ST_MultiSphere].begin(), mspheres_end = dynamic_objects[ST_MultiSphere].end(); iter != mspheres_end; ++iter)
			if(MultiSphereShapeInstanceCache* mssic = (MultiSphereShapeInstanceCache*)(*iter)->shape_cache)
				mssic->valid = false;

		for(unordered_set<RigidBody*>::iterator iter = dynamic_objects[ST_MultiSphere].begin(), mspheres_end = dynamic_objects[ST_MultiSphere].end(); iter != mspheres_end; ++iter)
			InitiateCollisionsForMultisphere(*iter, timestep, constraint_graph);

#if PROFILE_DOFIXEDSTEP
		timer_msphere_collide += timer.GetAndRestart();
#endif

		for(vector<ContactPoint*>::iterator iter = constraint_graph.contact_points.begin(), cp_end = constraint_graph.contact_points.end(); iter != cp_end; ++iter)
			(*iter)->DoUpdateAction(timestep);

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
		for(unsigned int i = 1; i < ST_ShapeTypeMax; ++i)
			if(CollisionShape::CanShapeTypeMove((ShapeType)i))
				for(unordered_set<RigidBody*>::iterator iter = dynamic_objects[i].begin(), objects_end = dynamic_objects[i].end(); iter != objects_end; ++iter)
					(*iter)->UpdatePos(timestep, region_man);

#if PROFILE_DOFIXEDSTEP
		timer_update_pos += timer.Stop();
		timer_total += timer2.Stop();

		++counter_dofixedstep;
#endif
	}



	void PhysicsWorld::AddRigidBody(RigidBody* r)
	{
		ShapeType type = r->GetShapeType();

		all_objects[type].insert(r);
		if(r->can_move)
			dynamic_objects[type].insert(r);

		r->gravity = gravity;

		region_man->OnObjectAdded(r, r->regions);
	}

	void PhysicsWorld::RemoveRigidBody(RigidBody* r)
	{
		ShapeType type = r->GetShapeType();
		all_objects[type].erase(r);
		if(r->can_move)
			dynamic_objects[type].erase(r);

		region_man->OnObjectRemoved(r, r->regions);

		const RegionSet& regions = r->regions;
		for(unsigned int i = 0; i < RegionSet::hash_size; ++i)
		{
			vector<PhysicsRegion*>& bucket = r->regions.buckets[i];
			for(vector<PhysicsRegion*>::const_iterator iter = bucket.begin(), bucket_end = bucket.end(); iter != bucket_end; ++iter)
				(*iter)->RemoveRigidBody(r);
		}
		r->regions.Clear();

		const set<RigidBody*>& disabled_collisions = r->disabled_collisions;
		for(set<RigidBody*>::iterator iter = disabled_collisions.begin(), disabled_end = disabled_collisions.end(); iter != disabled_end; ++iter)
			(*iter)->disabled_collisions.erase(r);
		r->disabled_collisions.clear();

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

	void PhysicsWorld::AddConstraint(PhysicsConstraint* c)
	{
		if(c->obj_a != NULL)
			c->obj_a->constraints.insert(c);
		if(c->obj_b != NULL)
			c->obj_b->constraints.insert(c);
		all_constraints.insert(c);
	}

	void PhysicsWorld::RemoveConstraint(PhysicsConstraint* c)
	{
		if(c->obj_a != NULL)
			c->obj_a->constraints.erase(c);
		if(c->obj_b != NULL)
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

		for(unsigned int i = 1; i < ST_ShapeTypeMax; ++i)
			if(CollisionShape::CanShapeTypeMove((ShapeType)i))
				for(unordered_set<RigidBody*>::iterator iter = all_objects[i].begin(), objects_end = all_objects[i].end(); iter != objects_end; ++iter)
					(*iter)->ResetForces();
	}

	void PhysicsWorld::DebugDrawWorld(SceneRenderer* renderer)
	{
		for(unsigned int i = 0; i < ST_ShapeTypeMax; ++i)
			for(unordered_set<RigidBody*>::iterator iter = all_objects[i].begin(), objects_end = all_objects[i].end(); iter != objects_end; ++iter)
				(*iter)->DebugDraw(renderer);

		renderer->Render();
		renderer->Cleanup();
	}

	Vec3 PhysicsWorld::GetGravity() { return gravity; }
	void PhysicsWorld::SetGravity(const Vec3& gravity_)
	{
		gravity = gravity_;

		for(unsigned int i = 1; i < ST_ShapeTypeMax; ++i)
			if(CollisionShape::CanShapeTypeMove((ShapeType)i))
				for(unordered_set<RigidBody*>::iterator iter = all_objects[i].begin(); iter != all_objects[i].end(); ++iter)
					(*iter)->gravity = gravity_;
	}

	void PhysicsWorld::RayTestPrivate(const Vec3& from, const Vec3& to, CollisionCallback& callback, float max_time, RigidBody* ibody)
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
		for(vector<RigidBody*>::iterator iter = relevant_objects.objects.begin(), relevant_end = relevant_objects.objects.end(); iter != relevant_end; ++iter)
			if(RigidBody* other = *iter)
			{
				switch(other->GetShapeType())
				{
					case ST_Sphere:
						DoRaySphere(ibody, other, ray, max_time, hits);
						break;

					case ST_TriangleMesh:
						DoRayMesh(ibody, other, ray, max_time, hits);
						break;

					case ST_InfinitePlane:
						DoRayPlane(ibody, other, ray, max_time, hits);
						break;

					case ST_MultiSphere:
						DoRayMultisphere(ibody, other, ray, max_time, hits);
						break;
				}
			}

		// run the collision callback on whatever we found
		if(!hits.empty())
		{
			hits.sort();

			for(list<RayResult>::iterator jter = hits.begin(), hits_end = hits.end(); jter != hits_end; ++jter)
			{
				jter->p.BuildCache();
				if(callback.OnCollision(jter->p))
					break;
			}
		}

		relevant_objects.ConcludeQuery();
	}
	void PhysicsWorld::RayTest(const Vec3& from, const Vec3& to, CollisionCallback& callback) { RayTestPrivate(from, to, callback); }




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
			else
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
			float bounciness = ibody->bounciness * jbody->bounciness;
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

		if(float magsq = dx.ComputeMagnitude())
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
				float total = 1.0f / obj_a->mass_info.mass + 1.0f / obj_b->mass_info.mass, inv_total = move_frac / total;

				obj_a->pos -= dx * (inv_total / obj_a->mass_info.mass);
				obj_b->pos += dx * (inv_total / obj_b->mass_info.mass);

				obj_a->xform_valid = false;
				obj_b->xform_valid = false;
			}
		}
	}




	/*
	 * Ray intersect functions
	 */
	static void DoRaySphere(RigidBody* ibody, RigidBody* jbody, const Ray& ray, float max_time, list<RayResult>& hits)
	{
		float first, second;

		if(Util::RaySphereIntersect(ray, Sphere(jbody->GetPosition(), ((SphereShape*)jbody->GetCollisionShape())->radius), first, second))
			if(first >= 0 && first < max_time)
			{
				ContactPoint p;

				p.obj_a = ibody;
				p.obj_b = jbody;
				p.a.pos = ray.origin + first * ray.direction;
				p.b.norm = Vec3::Normalize(p.a.pos - jbody->GetPosition(), 1.0f);

				hits.push_back(RayResult(first, p));
			}
	}

	static void DoRayMesh(RigidBody* ibody, RigidBody* jbody, const Ray& ray, float max_time, list<RayResult>& hits)
	{
		TriangleMeshShape* mesh = (TriangleMeshShape*)jbody->GetCollisionShape();

		Mat4 inv_mat = jbody->GetInvTransform();
		Ray scaled_ray(inv_mat.TransformVec3_1(ray.origin), inv_mat.TransformVec3_0(ray.direction * max_time));

		vector<Intersection> mesh_hits = mesh->RayTest(scaled_ray);
		for(vector<Intersection>::iterator kter = mesh_hits.begin(), hits_end = mesh_hits.end(); kter != hits_end; ++kter)
		{
			float t = kter->time * max_time;
			if(t >= 0 && t < max_time)
			{
				ContactPoint p;

				p.obj_a = ibody;
				p.obj_b = jbody;
				p.a.pos = ray.origin + t * ray.direction;
				p.b.norm = kter->normal;

				hits.push_back(RayResult(t, p));
			}
		}
	}

	static void DoRayPlane(RigidBody* ibody, RigidBody* jbody, const Ray& ray, float max_time, list<RayResult>& hits)
	{
		InfinitePlaneShape* plane = (InfinitePlaneShape*)jbody->GetCollisionShape();

		float t = Util::RayPlaneIntersect(ray, plane->plane);
		if(t >= 0 && t < max_time)
		{
			ContactPoint p;

			p.obj_a = ibody;
			p.obj_b = jbody;
			p.a.pos = ray.origin + t * ray.direction;
			p.b.norm = plane->plane.normal;

			hits.push_back(RayResult(t, p));
		}
	}

	static void DoRayMultisphere(RigidBody* ibody, RigidBody* jbody, const Ray& ray, float max_time, list<RayResult>& hits)
	{
		MultiSphereShape* shape = (MultiSphereShape*)jbody->GetCollisionShape();

		Mat4 inv_mat = jbody->GetInvTransform();

		Ray nu_ray(inv_mat.TransformVec3_1(ray.origin), inv_mat.TransformVec3_0(ray.direction * max_time));

		ContactPoint p;
		float t;
		if(shape->CollideRay(nu_ray, p, t, ibody, jbody))
		{
			p.a.pos = jbody->GetTransformationMatrix().TransformVec3_1(p.b.pos);
			hits.push_back(RayResult(t * max_time, p));
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




	/*
	 * MultiSphereShape collision functions
	 */
	static void DoMultisphereMesh(RigidBody* ibody, RigidBody* jbody, MultiSphereShape* ishape, const Mat4& xform, ConstraintGraph& hits)
	{
		TriangleMeshShape* jshape = (TriangleMeshShape*)jbody->GetCollisionShape();
		Mat4 j_xform = jbody->GetTransformationMatrix();
		Mat4 jinv = jbody->GetInvTransform();

		Mat4 inv_net_xform = jinv * xform;
		AABB xformed_aabb = ishape->GetTransformedAABB(inv_net_xform);						// the AABB of the multisphere in the coordinate system of the mesh

		vector<unsigned int> relevant_triangles;
		jshape->GetRelevantTriangles(xformed_aabb, relevant_triangles);
		if(relevant_triangles.empty())
			return;

		vector<Sphere> my_spheres;															// CollideMesh function will modify this if it's empty, otherwise use existing values

		ContactPoint p;
		for(vector<unsigned int>::iterator kter = relevant_triangles.begin(), triangles_end = relevant_triangles.end(); kter != triangles_end; ++kter)
		{
			const TriangleMeshShape::TriCache& tri = jshape->GetTriangleData(*kter);

			if(ishape->CollideMesh(inv_net_xform, my_spheres, tri, p, ibody, jbody))
			{
				p.a.pos = j_xform.TransformVec3_1(p.a.pos);
				p.b.pos = j_xform.TransformVec3_1(p.b.pos);
				p.b.norm = j_xform.TransformVec3_0(p.b.norm);
				p.a.norm = -p.b.norm;

				hits.AddContactPoint(p);
			}
		}
	}

	static void DoMultispherePlane(RigidBody* ibody, RigidBody* jbody, MultiSphereShape* ishape, const Mat4& xform, ConstraintGraph& hits)
	{
		InfinitePlaneShape* jshape = (InfinitePlaneShape*)jbody->GetCollisionShape();

		vector<ContactPoint> results;

		if(ishape->CollidePlane(xform, jshape->plane, results, ibody, jbody))
			for(vector<ContactPoint>::iterator iter = results.begin(), results_end = results.end(); iter != results_end; ++iter)
				hits.AddContactPoint(*iter);
	}

	static void DoMultisphereMultisphere(RigidBody* ibody, RigidBody* jbody, MultiSphereShapeInstanceCache* ishape, MultiSphereShapeInstanceCache* jshape, ConstraintGraph& hits)
	{
		static ContactPoint p;

		struct MaxExtentGetter
		{
			float operator()(const Vec3& direction, const vector<Sphere>& spheres)
			{
				vector<Sphere>::const_iterator iter = spheres.begin(), spheres_end = spheres.end();

				float maximum = Vec3::Dot(direction, iter->center) + iter->radius;
				++iter;

				while(iter != spheres_end)
				{
					maximum = max(maximum, Vec3::Dot(direction, iter->center) + iter->radius);
					++iter;
				}

				return maximum;
			}
		} GetMaximumExtent;

		struct MinExtentGetter
		{
			float operator()(const Vec3& direction, const vector<Sphere>& spheres)
			{
				vector<Sphere>::const_iterator iter = spheres.begin(), spheres_end = spheres.end();

				float minimum = Vec3::Dot(direction, iter->center) - iter->radius;
				++iter;

				while(iter != spheres_end)
				{
					minimum = min(minimum, Vec3::Dot(direction, iter->center) - iter->radius);
					++iter;
				}

				return minimum;
			}
		} GetMinimumExtent;

		AABB overlap;
		if(AABB::Intersect(ishape->aabb, jshape->aabb, overlap))
		{
			vector<Sphere>& my_spheres = ishape->spheres;
			vector<Sphere>& other_spheres = jshape->spheres;

			// try to find a separating axis
			Vec3 direction;
			float score = -1;
			float search_scale = 0.6f;

			char best_test;
			Vec3 test_dir[8];

			static const float x_offsets[] = {	-1,	-1,	-1, -1,	1,	1,	1,	1 };
			static const float y_offsets[] = {	-1,	-1,	1,	1,	-1,	-1,	1,	1 };
			static const float z_offsets[] = {	-1,	1,	-1,	1,	-1,	1,	-1,	1 };

			for(char i = 0; i < 5; ++i)
			{
				float best_score;

				for(char j = 0; j < 8; ++j)
				{
					Vec3& dir = test_dir[j] = Vec3::Normalize(Vec3(
						direction.x + x_offsets[j] * search_scale,
						direction.y + y_offsets[j] * search_scale,
						direction.z + z_offsets[j] * search_scale));

					float test_score = GetMaximumExtent(dir, my_spheres) - GetMinimumExtent(dir, other_spheres);

					if(test_score < 0)							// found a separating plane? go home early
						return;
					else
					{
						if(j == 0 || test_score < best_score)
						{
							best_test = j;
							best_score = test_score;
						}
					}
				}

				if(i != 0 && best_score >= score)
					search_scale *= 0.5f;
				else
				{
					direction = test_dir[best_test];
					score = best_score;
				}
			}

			p = ContactPoint();
			p.obj_a = ibody;
			p.obj_b = jbody;

			p.a.norm = direction;
			p.b.norm = -direction;

			Vec3 pos = overlap.GetCenterPoint();					// TODO: do this better
			Vec3 offset = direction * (score * 0.5f);
			p.a.pos = pos - offset;
			p.b.pos = pos + offset;

			hits.AddContactPoint(p);
		}
	}
}
