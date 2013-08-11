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

#include "RayShape.h"
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

#define MAX_SEQUENTIAL_SOLVER_ITERATIONS 5

#define PHYSICS_TICK_FREQUENCY 60
#define MAX_FIXED_STEPS_PER_UPDATE 1

#define PROFILE_DOFIXEDSTEP 1

#define NUM_COLLISION_THREADS 4

namespace CibraryEngine
{
	using boost::unordered_set;
	using boost::unordered_map;




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
	static float timer_step_callback = 0.0f;
	static float timer_update_vel = 0.0f;
	static float timer_ray_update = 0.0f;
	static float timer_collide = 0.0f;
	static float timer_constraints = 0.0f;
	static float timer_cgraph = 0.0f;
	static float timer_update_pos = 0.0f;
	static float timer_total = 0.0f;

	static unsigned int counter_dofixedstep = 0;
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
		cgraph_solver(new CPUConstraintGraphSolver()),
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
		Debug(((stringstream&)(stringstream() << '\t' << "update_pos =\t\t\t"		<< timer_update_pos						<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "total of above =\t\t"		<< timer_step_callback + timer_update_vel + timer_ray_update + timer_collide + timer_constraints + timer_cgraph + timer_update_pos << endl)).str());
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

		if(step_callback != NULL) { step_callback->OnPhysicsStep(this, timestep); }

#if PROFILE_DOFIXEDSTEP
		timer_step_callback += timer.GetAndRestart();
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
			RayCollider* collider = *iter;
			RayTestPrivate(collider->pos, collider->pos + collider->vel, ray_callback, timestep, collider);
		}

#if PROFILE_DOFIXEDSTEP
		timer_ray_update += timer.Stop();
		timer.Start();
#endif

		struct CollisionInitiator : public ThreadTask
		{
			float timestep;
			vector<CollisionObject*>* objects;
			ContactDataCollector* collect;
			unsigned int from, to;

			CollisionInitiator() { }
			CollisionInitiator(ContactDataCollector* collect, float timestep, vector<CollisionObject*>* objects, unsigned int from, unsigned int to) : timestep(timestep), objects(objects), collect(collect), from(from), to(to) { }

			void DoTask()
			{
				CollisionObject **data = objects->data(), **start_ptr = data + from, **finish_ptr = data + to;
				for(CollisionObject** obj_ptr = start_ptr; obj_ptr != finish_ptr; ++obj_ptr)
					(*obj_ptr)->InitiateCollisions(timestep, collect);
			}
		};
		CollisionInitiator collision_initiators[NUM_COLLISION_THREADS];

		vector<CollisionObject*> dynamic_objects_vector(dynamic_objects.begin(), dynamic_objects.end());

		unsigned int num_objects = dynamic_objects_vector.size();
		unsigned int use_threads = max(1u, min((unsigned)NUM_COLLISION_THREADS, num_objects / 40));

		for(unsigned int i = 0; i < use_threads; ++i)
		{
			collision_initiators[i] = CollisionInitiator(cp_collectors[i], timestep, &dynamic_objects_vector, i * num_objects / use_threads, (i + 1) * num_objects / use_threads);
			task_threads[i]->StartTask(&collision_initiators[i]);
		}

		vector<ContactPoint*> temp_contact_points;
		for(unsigned int i = 0; i < use_threads; ++i)
		{
			task_threads[i]->WaitForCompletion();
			for(vector<ContactRegion*>::iterator iter = collision_initiators[i].collect->results.begin(); iter != collision_initiators[i].collect->results.end(); ++iter)
				temp_contact_points.insert(temp_contact_points.end(), (*iter)->points.begin(), (*iter)->points.end());
		}

#if PROFILE_DOFIXEDSTEP
		timer_collide += timer.GetAndRestart();
#endif

		vector<PhysicsConstraint*> use_constraints;
		for(unordered_set<PhysicsConstraint*>::iterator iter = all_constraints.begin(), constraints_end = all_constraints.end(); iter != constraints_end; ++iter)
		{
			(*iter)->DoUpdateAction(timestep);
			use_constraints.push_back(*iter);
		}

		for(vector<ContactPoint*>::iterator iter = temp_contact_points.begin(), cp_end = temp_contact_points.end(); iter != cp_end; ++iter)
		{
			(*iter)->DoUpdateAction(timestep);
			use_constraints.push_back(*iter);
		}

#if PROFILE_DOFIXEDSTEP
		timer_constraints += timer.GetAndRestart();
#endif

		cgraph_solver->Solve(timestep, MAX_SEQUENTIAL_SOLVER_ITERATIONS, use_constraints);

		for(unsigned int i = 0; i < use_threads; ++i)
			collision_initiators[i].collect->ClearResults();

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

	PhysicsStepCallback* PhysicsWorld::GetStepCallback()				{ return step_callback; }
	void PhysicsWorld::SetStepCallback(PhysicsStepCallback* callback)	{ step_callback = callback; }
}
