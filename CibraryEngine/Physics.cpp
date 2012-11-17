#include "StdAfx.h"
#include "Physics.h"

#include "RigidBody.h"
#include "RayCollider.h"
#include "CollisionGroup.h"

#include "PhysicsRegion.h"
#include "GridRegionManager.h"

#include "RayShape.h"
#include "TriangleMeshShape.h"
#include "InfinitePlaneShape.h"
#include "MultiSphereShape.h"

#include "JointConstraint.h"

#include "ConstraintGraphSolver.h"

#include "Matrix.h"

#include "DebugLog.h"
#include "Serialize.h"
#include "Util.h"

#include "RenderNode.h"
#include "SceneRenderer.h"
#include "DebugDrawMaterial.h"

#include "VertexBuffer.h"

#include "TaskThread.h"
#include "HardwareAcceleratedComputation.h"

#include "ProfilingTimer.h"

#define MAX_SEQUENTIAL_SOLVER_ITERATIONS 5

#define PHYSICS_TICK_FREQUENCY 60
#define MAX_FIXED_STEPS_PER_UPDATE 1

#define PROFILE_DOFIXEDSTEP 1

#define NUM_THREADS 8

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
		cgraph_solver(new ConstraintGraphSolver()),
		orphan_callback(new MyOrphanCallback())
	{
		region_man = new GridRegionManager(&all_regions, orphan_callback);

		for(unsigned int i = 0; i < NUM_THREADS; ++i)
			task_threads.push_back(new TaskThread());
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

		ContactPoint::EmptyRecycleBins();

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
		Debug(((stringstream&)(stringstream() << '\t' << "update_vel =\t\t\t"		<< timer_update_vel						<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "ray_update =\t\t\t"		<< timer_ray_update						<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "collide =\t\t\t\t"		<< timer_collide						<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "constraints =\t\t\t"		<< timer_constraints					<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "cgraph =\t\t\t\t"			<< timer_cgraph							<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "update_pos =\t\t\t"		<< timer_update_pos						<< endl)).str());
		Debug(((stringstream&)(stringstream() << '\t' << "total of above =\t\t"		<< timer_update_vel + timer_ray_update + timer_collide + timer_constraints + timer_cgraph + timer_update_pos << endl)).str());
#endif
	}

	void PhysicsWorld::InitConstraintGraphSolver(ContentMan* content) { cgraph_solver->Init(content); }



	float PhysicsWorld::GetUseMass(RigidBody* ibody, RigidBody* jbody, const Vec3& position, const Vec3& direction, float& B)
	{
		float A;

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
			vector<ContactPoint*> contact_points;
			unsigned int from, to;

			CollisionInitiator() { }
			CollisionInitiator(float timestep, vector<CollisionObject*>* objects, unsigned int from, unsigned int to) : timestep(timestep), objects(objects), from(from), to(to) { }

			void DoTask()
			{
				CollisionObject **data = objects->data(), **start_ptr = data + from, **finish_ptr = data + to;
				for(CollisionObject** obj_ptr = start_ptr; obj_ptr != finish_ptr; ++obj_ptr)
					(*obj_ptr)->InitiateCollisions(timestep, contact_points);
			}
		};
		CollisionInitiator collision_initiators[NUM_THREADS];

		vector<CollisionObject*> dynamic_objects_vector;
		for(unordered_set<CollisionObject*>::iterator iter = dynamic_objects.begin(), objects_end = dynamic_objects.end(); iter != objects_end; ++iter)
			dynamic_objects_vector.push_back(*iter);
		unsigned int num_objects = dynamic_objects_vector.size();
		unsigned int use_threads = max(1u, min((unsigned)NUM_THREADS, num_objects / 40));

		for(unsigned int i = 0; i < use_threads; ++i)
		{
			collision_initiators[i] = CollisionInitiator(timestep, &dynamic_objects_vector, i * num_objects / use_threads, (i + 1) * num_objects / use_threads);
			task_threads[i]->StartTask(&collision_initiators[i]);
		}

		vector<ContactPoint*> temp_contact_points;
		for(unsigned int i = 0; i < use_threads; ++i)
		{
			task_threads[i]->WaitForCompletion();
			for(vector<ContactPoint*>::iterator iter = collision_initiators[i].contact_points.begin(); iter != collision_initiators[i].contact_points.end(); ++iter)
				temp_contact_points.push_back(*iter);
		}

#if PROFILE_DOFIXEDSTEP
		timer_collide += timer.GetAndRestart();
#endif

		vector<PhysicsConstraint*> use_constraints;
		for(vector<ContactPoint*>::iterator iter = temp_contact_points.begin(), cp_end = temp_contact_points.end(); iter != cp_end; ++iter)
		{
			(*iter)->DoUpdateAction(timestep);
			use_constraints.push_back(*iter);
		}

		for(unordered_set<PhysicsConstraint*>::iterator iter = all_constraints.begin(), constraints_end = all_constraints.end(); iter != constraints_end; ++iter)
		{
			(*iter)->DoUpdateAction(timestep);
			use_constraints.push_back(*iter);
		}

#if PROFILE_DOFIXEDSTEP
		timer_constraints += timer.GetAndRestart();
#endif

		//SolveConstraintGraph(use_constraints);
		cgraph_solver->Solve(timestep, MAX_SEQUENTIAL_SOLVER_ITERATIONS, use_constraints);

		for(vector<ContactPoint*>::iterator iter = temp_contact_points.begin(); iter != temp_contact_points.end(); ++iter)
			ContactPoint::Delete(*iter);

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

			bounce_coeff = -(1.0f + obj_a->bounciness * obj_b->bounciness);
			kfric_coeff = sfric_coeff = obj_a->friction * obj_b->friction;			// kfric would be * 0.9f but in practice the sim treats everything as kinetic anyway
			//moi_n = Mat3::Invert(obj_a->inv_moi + obj_b->inv_moi) * normal;			// this isn't actually used right now, because angular friction is disabled

			use_mass = PhysicsWorld::GetUseMass(obj_a, obj_b, use_pos, normal);
			r1 = use_pos - obj_a->cached_com;
			r2 = use_pos - obj_b->cached_com;
			nr1 = Vec3::Cross(normal, r1);
			nr2 = Vec3::Cross(normal, r2);

			cache_valid = true;
		}
	}

	Vec3 ContactPoint::GetRelativeLocalVelocity() const { return obj_b->vel - obj_a->vel + Vec3::Cross(r2, obj_b->rot) - Vec3::Cross(r1, obj_a->rot); }

	float ContactPoint::GetInwardVelocity() const
	{
		// based on the computations in GetUseMass
		float B = obj_b->can_move ? Vec3::Dot(obj_a->vel - obj_b->vel, normal) : Vec3::Dot(obj_a->vel, normal);
		if(obj_a->can_rotate)
			B += Vec3::Dot(obj_a->rot, nr1);
		if(obj_b->can_rotate)
			B -= Vec3::Dot(obj_b->rot, nr2);
		return B;
	}

	void ContactPoint::ApplyImpulse(const Vec3& impulse) const
	{
		if(obj_a->active)
		{
			obj_a->vel += impulse * obj_a->inv_mass;
			if(obj_a->can_rotate)
				obj_a->rot += obj_a->inv_moi * Vec3::Cross(impulse, r1);
		}

		if(obj_b->active && obj_b->can_move)
		{
			obj_b->vel -= impulse * obj_b->inv_mass;
			if(obj_b->can_rotate)
				obj_b->rot -= obj_b->inv_moi * Vec3::Cross(impulse, r2);
		}
	}

	bool ContactPoint::DoCollisionResponse() const
	{
		assert(cache_valid);

		RigidBody* ibody = obj_a;
		RigidBody* jbody = obj_b;

		bool j_can_move = jbody->can_move;

		Vec3 dv = GetRelativeLocalVelocity();
		float nvdot = Vec3::Dot(normal, dv);
		if(nvdot < 0.0f)
		{
			float impulse_mag = bounce_coeff * GetInwardVelocity() * use_mass;
			if(impulse_mag < 0)
			{
				Vec3 impulse = normal * impulse_mag;

				// normal force
				if(impulse.ComputeMagnitudeSquared() != 0)
				{
					ApplyImpulse(impulse);

					// applying this impulse means we need to recompute dv and nvdot!
					dv = GetRelativeLocalVelocity();
					nvdot = Vec3::Dot(normal, dv);
				}

				Vec3 t_dv = dv - normal * nvdot;
				float t_dv_magsq = t_dv.ComputeMagnitudeSquared();

				// linear friction
				if(t_dv_magsq > 0)								// object is moving; apply kinetic friction
				{
					float t_dv_mag = sqrtf(t_dv_magsq), inv_tdmag = 1.0f / t_dv_mag;
					Vec3 u_tdv = t_dv * inv_tdmag;

					float use_mass2 = PhysicsWorld::GetUseMass(ibody, jbody, use_pos, u_tdv);
					ApplyImpulse(t_dv * min(use_mass2, fabs(impulse_mag * kfric_coeff * inv_tdmag)));
				}
				else											// object isn't moving; apply static friction
				{
					Vec3 df = jbody->applied_force - ibody->applied_force;
					float nfdot = Vec3::Dot(normal, df);

					Vec3 t_df = df - normal * nfdot;
					float t_df_mag = t_df.ComputeMagnitude();

					float fric_i_mag = min(impulse_mag * sfric_coeff, t_df_mag);
					if(fric_i_mag > 0)
						ApplyImpulse(t_df * (-fric_i_mag / t_df_mag));
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

				return true;
			}
		}

		return false;
	}

	void ContactPoint::DoConstraintAction()
	{
		BuildCache();

		if(DoCollisionResponse())
		{
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

	void ContactPoint::WriteDataToBuffer(float* ptr)
	{
		BuildCache();

		ptr[0] =	1.0f;
		ptr[1] =	use_mass;
		ptr[2] =	bounce_coeff;
		ptr[3] =	sfric_coeff;

		memcpy(ptr + 4,		&normal,	3 * sizeof(float));			// ptr[4] through ptr[6]
		memcpy(ptr + 8,		&r1,		3 * sizeof(float));			// ptr[8] through ptr[10]
		memcpy(ptr + 12,	&r2,		3 * sizeof(float));			// ptr[12] through ptr[14]
		memcpy(ptr + 16,	&nr1,		3 * sizeof(float));			// ptr[16] through ptr[18]
		memcpy(ptr + 20,	&use_pos,	3 * sizeof(float));			// ptr[20] through ptr[22]

		ptr[7] =	nr2.x;
		ptr[11] =	nr2.y;
		ptr[15] =	nr2.z;

		// ptr[19] is unused
		// ptr[23] is unused
	}

	/*
	 * ContactPoint recycle bin stuffs
	 */
	static vector<ContactPoint*> cp_recycle_bin = vector<ContactPoint*>();

	ContactPoint* ContactPoint::New()
	{
		/*
		if(cp_recycle_bin.empty())
			return new ContactPoint();
		else
		{
			ContactPoint* result = *cp_recycle_bin.rbegin();
			cp_recycle_bin.pop_back();

			return new (result) ContactPoint();
		}
		*/
		return new ContactPoint();
	}

	ContactPoint* ContactPoint::New(const ContactPoint& cp)
	{
		/*
		if(cp_recycle_bin.empty())
			return new ContactPoint(cp);
		else
		{
			ContactPoint* result = *cp_recycle_bin.rbegin();
			cp_recycle_bin.pop_back();

			return new (result) ContactPoint(cp);
		}
		*/
		return new ContactPoint(cp);
	}

	void ContactPoint::Delete(ContactPoint* cp) { /*cp->~ContactPoint(); cp_recycle_bin.push_back(cp);*/ delete cp; }

	void ContactPoint::EmptyRecycleBins()
	{
		for(vector<ContactPoint*>::iterator iter = cp_recycle_bin.begin(); iter != cp_recycle_bin.end(); ++iter)
			delete *iter;
		cp_recycle_bin.clear();
	}
}
