#include "StdAfx.h"
#include "Physics.h"

#include "RigidBody.h"
#include "RayCollider.h"
#include "CollisionGroup.h"

#include "ContactPoint.h"
#include "ContactRegion.h"
#include "ContactDataCollector.h"

#include "PhysicsRegion.h"
#include "GridRegionManager.h"

#include "TriangleMeshShape.h"
#include "InfinitePlaneShape.h"
#include "MultiSphereShape.h"

#include "CPUConstraintGraphSolver.h"

#include "Matrix.h"

#include "DebugLog.h"
#include "Serialize.h"
#include "Util.h"

#include "RenderNode.h"
#include "SceneRenderer.h"
#include "DebugDrawMaterial.h"

#include "TaskThread.h"

#include "ProfilingTimer.h"

#define MAX_SEQUENTIAL_SOLVER_ITERATIONS 15

#define PHYSICS_TICK_FREQUENCY 60
#define MAX_FIXED_STEPS_PER_UPDATE 1

#define PROFILE_DOFIXEDSTEP 1

#define NUM_COLLISION_THREADS 1//8

namespace CibraryEngine
{
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




#if PROFILE_DOFIXEDSTEP
	static float timer_step_callback		= 0.0f;
	static float timer_update_vel			= 0.0f;
	static float timer_ray_update			= 0.0f;
	static float timer_collide				= 0.0f;
	static float timer_constraints			= 0.0f;
	static float timer_cgraph				= 0.0f;
	static float timer_postcg               = 0.0f;
	static float timer_update_pos			= 0.0f;
	static float timer_total				= 0.0f;

	static unsigned int counter_dofixedstep	= 0;
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
		all_constraints(),
		gravity(0, -9.8f, 0),
		internal_timer(),
		timer_interval(1.0f / PHYSICS_TICK_FREQUENCY),
		task_threads(),
		cp_collectors(),
		cgraph_solver(new CPUConstraintGraphSolver(&task_threads)),
		orphan_callback(new MyOrphanCallback()),
		step_callback(NULL)
	{
		region_man = new GridRegionManager(&all_regions, orphan_callback);

		for(unsigned int i = 0; i < NUM_COLLISION_THREADS; ++i)
		{
			task_threads.push_back(new TaskThread());
			cp_collectors.push_back(ContactDataCollector::NewCollector());
		}
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

		for(vector<ContactDataCollector*>::iterator iter = cp_collectors.begin(); iter != cp_collectors.end(); ++iter)
			ContactDataCollector::DeleteCollector(*iter);
		cp_collectors.clear();

		for(vector<TaskThread*>::iterator iter = task_threads.begin(); iter != task_threads.end(); ++iter)
		{
			(*iter)->Shutdown();
			delete *iter;
		}
		task_threads.clear();

		delete cgraph_solver;
		cgraph_solver = NULL;

#if PROFILE_DOFIXEDSTEP
		Debug(((stringstream&)(stringstream() << "total for " << counter_dofixedstep << " calls to PhysicsWorld::DoFixedStep = " << timer_total << endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "step_callback =\t\t\t"	<< timer_step_callback					<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "update_vel =\t\t\t"		<< timer_update_vel						<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "ray_update =\t\t\t"		<< timer_ray_update						<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "collide =\t\t\t\t"		<< timer_collide						<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "constraints =\t\t\t"		<< timer_constraints					<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "cgraph =\t\t\t\t"			<< timer_cgraph							<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "postcg =\t\t\t\t"			<< timer_postcg							<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "update_pos =\t\t\t"		<< timer_update_pos						<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "total of above =\t\t"		<< timer_step_callback + timer_update_vel + timer_ray_update + timer_collide + timer_constraints + timer_cgraph + timer_postcg + timer_update_pos << endl)).str());
#endif
	}

	void PhysicsWorld::InitConstraintGraphSolver(ContentMan* content) { cgraph_solver->Init(content); }



	float PhysicsWorld::GetUseMass(RayCollider* collider, RigidBody* body, const Vec3& position, const Vec3& direction, float& B)
	{
		float A;

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



	void PhysicsWorld::DoFixedStep()
	{
#if PROFILE_DOFIXEDSTEP
		ProfilingTimer timer, timer2;
		timer2.Start();
		timer.Start();
#endif
		float timestep = timer_interval;

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
				if(RayCallback* callback = rr.collider->GetRayCallback())
					return callback->OnCollision(rr);
				else
				{
					rr.collider->DoCollisionResponse(rr);
					return true;
				}
			}
		} ray_callback(this);

		for(unordered_set<RayCollider*>::iterator iter = rays.begin(), rays_end = rays.end(); iter != rays_end; ++iter)
		{
			RayCollider* collider = *iter;
			RayTestPrivate(collider->pos, collider->pos + collider->vel, ray_callback, timestep, collider);
		}

#if PROFILE_DOFIXEDSTEP
		timer_ray_update += timer.GetAndRestart();
#endif

		// detect all the collisions between dynamic objects, collecting contact points for those collisions
		struct CollisionInitiator : public ThreadTask
		{
			float timestep;
			vector<CollisionObject*>* objects;
			ContactDataCollector* collect;
			unsigned int from, to;

			void SetTaskParams(ContactDataCollector* collect_, float timestep_, vector<CollisionObject*>* objects_, unsigned int from_, unsigned int to_) { collect = collect_; timestep = timestep_; objects = objects_; from = from_; to = to_; }

			void DoTask()
			{
				CollisionObject **data = objects->data(), **start_ptr = data + from, **finish_ptr = data + to;
				for(CollisionObject** obj_ptr = start_ptr; obj_ptr != finish_ptr; ++obj_ptr)
					(*obj_ptr)->InitiateCollisions(timestep, collect);
			}
		} collision_initiators[NUM_COLLISION_THREADS];

		vector<CollisionObject*> dynamic_objects_vector(dynamic_objects.begin(), dynamic_objects.end());

		vector<ContactPoint*> temp_contact_points;
		unsigned int num_objects = dynamic_objects_vector.size();
		unsigned int use_collider_threads = max(1u, min((unsigned)NUM_COLLISION_THREADS, num_objects / 40));

		for(unsigned int i = 0; i < use_collider_threads; ++i)
		{
			collision_initiators[i].SetTaskParams(cp_collectors[i], timestep, &dynamic_objects_vector, i * num_objects / use_collider_threads, (i + 1) * num_objects / use_collider_threads);
			task_threads[i]->StartTask(&collision_initiators[i]);
		}

		for(unsigned int i = 0; i < use_collider_threads; ++i)
		{
			task_threads[i]->WaitForCompletion();
			for(vector<ContactRegion*>::iterator iter = collision_initiators[i].collect->results.begin(); iter != collision_initiators[i].collect->results.end(); ++iter)
				temp_contact_points.insert(temp_contact_points.end(), (*iter)->points.begin(), (*iter)->points.end());
		}

		for(unsigned int i = 0; i < temp_contact_points.size(); ++i)
		{
			const ContactPoint& cp = *temp_contact_points[i];
			if(cp.obj_a->contact_callback != NULL)
				cp.obj_a->contact_callback->OnContact(cp);
			if(cp.obj_b->contact_callback != NULL)
				cp.obj_b->contact_callback->OnContact(cp);
		}

#if PROFILE_DOFIXEDSTEP
		timer_collide += timer.GetAndRestart();
#endif

		if(step_callback != NULL) { step_callback->OnPhysicsStep(this, timestep); }

#if PROFILE_DOFIXEDSTEP
		timer_step_callback += timer.GetAndRestart();
#endif


		// do once-per-tick update actions for all the constraints in the physics world
		vector<PhysicsConstraint*> use_constraints;
		for(unordered_set<PhysicsConstraint*>::iterator iter = all_constraints.begin(), constraints_end = all_constraints.end(); iter != constraints_end; ++iter)
			use_constraints.push_back(*iter);
		for(vector<ContactPoint*>::iterator iter = temp_contact_points.begin(), cp_end = temp_contact_points.end(); iter != cp_end; ++iter)
			use_constraints.push_back(*iter);

		struct ConstraintUpdater : public ThreadTask
		{
			float timestep;
			vector<PhysicsConstraint*>* constraints;
			unsigned int from, to;

			void SetTaskParams(float timestep_, vector<PhysicsConstraint*>* constraints_, unsigned int from_, unsigned int to_) { timestep = timestep_; constraints = constraints_; from = from_; to = to_; }

			void DoTask()
			{
				PhysicsConstraint **data = constraints->data(), **start_ptr = data + from, **finish_ptr = data + to;
				for(PhysicsConstraint** obj_ptr = start_ptr; obj_ptr != finish_ptr; ++obj_ptr)
					(*obj_ptr)->DoUpdateAction(timestep);
			}
		} constraint_updaters[NUM_COLLISION_THREADS];

		unsigned int num_constraints = use_constraints.size();
		unsigned int use_constraint_threads = NUM_COLLISION_THREADS;

		for(unsigned int i = 0; i < use_constraint_threads; ++i)
		{
			constraint_updaters[i].SetTaskParams(timestep, &use_constraints, i * num_constraints / use_constraint_threads, (i + 1) * num_constraints / use_constraint_threads);
			task_threads[i]->StartTask(&constraint_updaters[i]);
		}
		for(unsigned int i = 0; i < use_constraint_threads; ++i)
			task_threads[i]->WaitForCompletion();

#if PROFILE_DOFIXEDSTEP
		timer_constraints += timer.GetAndRestart();
#endif

		// evaluate the constraints we collected
		cgraph_solver->Solve(timestep, MAX_SEQUENTIAL_SOLVER_ITERATIONS, use_constraints);

#if PROFILE_DOFIXEDSTEP
		timer_cgraph += timer.GetAndRestart();
#endif

		for(unsigned int i = 0; i < use_collider_threads; ++i)
		{
			collision_initiators[i].collect->RunPostResolutionCallbacks();
			collision_initiators[i].collect->ClearResults();
		}

#if PROFILE_DOFIXEDSTEP
		timer_postcg += timer.GetAndRestart();
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
				RigidBody* rigid_body = (RigidBody*)obj;
				if(rigid_body->can_move)
					dynamic_objects.insert(rigid_body);

				rigid_body->gravity = gravity;

				break;
			}

			case COT_RayCollider:
			{
				RayCollider* ray_collider = (RayCollider*)obj;
				rays.insert(ray_collider);

				ray_collider->gravity = gravity;

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

			case COT_CollisionGroup:
			{
				dynamic_objects.erase(obj);
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
				{
					c->obj_b->constraints.erase(c);
					c->obj_a = NULL;
				}
				else if(c->obj_b == r && c->obj_a)
				{
					c->obj_a->constraints.erase(c);
					c->obj_b = NULL;
				}

				c->OnObjectRemoved(r);

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
		if(c->obj_a != NULL)
			c->obj_a->constraints.erase(c);
		if(c->obj_b != NULL)
			c->obj_b->constraints.erase(c);

		if(all_constraints.find(c) != all_constraints.end())		// TODO: fix this better?
			all_constraints.erase(c);
	}

	void PhysicsWorld::Update(const TimingInfo& time)
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
		Vec3 endpoint = from + (to - from) * max_time;

		static RegionSet regions;
		assert(regions.count == 0);

		region_man->GetRegionsOnRay(from, endpoint, regions);

		AABB ray_aabb(from);
		ray_aabb.Expand(endpoint);

		static RelevantObjectsQuery relevant_objects;
		assert(relevant_objects.count == 0);
		
		for(unsigned int i = 0; i < RegionSet::hash_size; ++i)
		{
			vector<PhysicsRegion*>& bucket = regions.buckets[i];
			for(vector<PhysicsRegion*>::iterator iter = bucket.begin(); iter != bucket.end(); ++iter)
				(*iter)->GetRelevantObjects(ray_aabb, relevant_objects);
		}

		if(relevant_objects.count != 0)
		{
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
						case COT_RigidBody:      { RayCollider::CollideRigidBody     ((RigidBody*)*iter,      ray, max_time, hits, collider); break; }
						case COT_CollisionGroup: { RayCollider::CollideCollisionGroup((CollisionGroup*)*iter, ray, max_time, hits, collider); break; }
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

		regions.Clear();
	}
	void PhysicsWorld::RayTest(const Vec3& from, const Vec3& to, RayCallback& callback) { RayTestPrivate(from, to, callback); }
}
