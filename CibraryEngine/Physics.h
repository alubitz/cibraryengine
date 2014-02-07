#pragma once

#include "StdAfx.h"
#include "Disposable.h"

#include "CollisionShape.h"

#include "Vector.h"
#include "Quaternion.h"
#include "TimingInfo.h"
#include "Events.h"

#include "MassInfo.h"

#include "SmartHashSet.h"

namespace CibraryEngine
{
	using namespace std;

	using boost::unordered_map;
	using boost::unordered_set;

	struct Mat4;

	class CollisionObject;
	class RayCollider;

	class PhysicsConstraint;
	struct ContactPoint;
	class ContactDataCollector;

	class RayCallback;
	class CollisionCallback;
	class PhysicsStepCallback;

	class PhysicsRegion;
	class PhysicsRegionManager;

	class ConstraintGraphSolver;

	typedef SmartHashSet<PhysicsRegion, 7> RegionSet;
	typedef SmartHashSet<CollisionObject, 17> RelevantObjectsQuery;

	class SceneRenderer;

	struct TaskThread;
	struct HardwareAcceleratedComputation;

	struct ContentMan;

	/** Class for a physical simulation */
	class PhysicsWorld : public Disposable
	{
		private:

			unordered_set<CollisionObject*> all_objects;
			unordered_set<RayCollider*> rays;
			unordered_set<CollisionObject*> dynamic_objects;

			unordered_set<PhysicsRegion*> all_regions;
			PhysicsRegionManager* region_man;

			unordered_set<PhysicsConstraint*> all_constraints;

			Vec3 gravity;

			float internal_timer, timer_interval;

			vector<TaskThread*> task_threads;
			vector<ContactDataCollector*> cp_collectors;
			ConstraintGraphSolver* cgraph_solver;

			void DoFixedStep();

			void RayTestPrivate(const Vec3& from, const Vec3& to, RayCallback& callback, float max_time = 1.0f, RayCollider* collider = NULL);

			struct MyOrphanCallback;
			MyOrphanCallback* orphan_callback;

			PhysicsStepCallback* step_callback;

			// no copying!
			void operator=(PhysicsWorld& other) { }
			PhysicsWorld(PhysicsWorld& other)   { }

		protected:

			void InnerDispose();

		public:

			/** Initializes a PhysicsWorld */
			PhysicsWorld();

			void InitConstraintGraphSolver(ContentMan* content);

			/** Adds a collision object to the simulation */
			void AddCollisionObject(CollisionObject* obj);
			/**
			 * Removes a collision object from the simulation.
			 * This will also remove any constraints with other objects, and it's up to you to know when to delete them!
			 */
			void RemoveCollisionObject(CollisionObject* obj);

			void AddConstraint(PhysicsConstraint* c);
			void RemoveConstraint(PhysicsConstraint* c);

			/** Steps the simulation */
			void Update(const TimingInfo& time);

			void DebugDrawWorld(SceneRenderer* renderer);

			Vec3 GetGravity() const                             { return gravity; }
			void SetGravity(const Vec3& gravity);

			void RayTest(const Vec3& from, const Vec3& to, RayCallback& callback);

			PhysicsStepCallback* GetStepCallback() const        { return step_callback; }
			void SetStepCallback(PhysicsStepCallback* callback) { step_callback = callback; }

			/** Hard to explain... return value is like mass, and B is like inward velocity */
			static float GetUseMass(RayCollider* collider, RigidBody* body, const Vec3& position, const Vec3& direction, float& B);
			static float GetUseMass(RayCollider* collider, RigidBody* body, const Vec3& position, const Vec3& direction);
	};

	class RigidBody;

	/** Things for the constraint solver to solve */
	class PhysicsConstraint : public Disposable
	{
		public:

			PhysicsConstraint() : obj_a(NULL), obj_b(NULL) { }
			PhysicsConstraint(RigidBody* obj_a, RigidBody* obj_b) : obj_a(obj_a), obj_b(obj_b) { }

			virtual ~PhysicsConstraint() { }

			virtual bool DoConstraintAction() = 0;
			virtual void DoUpdateAction(float timestep)		{ }

			virtual void OnObjectRemoved(RigidBody* object)	{ }		// one of our objects has been removed from the world, so this constraint must be removed as well

			RigidBody* obj_a;
			RigidBody* obj_b;
	};

	class PhysicsRegionManager
	{
	protected:

		unordered_set<PhysicsRegion*>* all_regions;

	public:

		PhysicsRegionManager(unordered_set<PhysicsRegion*>* all_regions) : all_regions(all_regions) { }		
		virtual ~PhysicsRegionManager() { };

		// region should call TakeOwnership
		virtual void OnObjectAdded(CollisionObject* object, RegionSet& object_regions) = 0;

		virtual void OnObjectUpdate(CollisionObject* object, RegionSet& object_regions, float timestep) = 0;

		// region should NOT call Disown
		virtual void OnObjectRemoved(CollisionObject* object, RegionSet& object_regions) = 0;

		/** Get the PhysicsRegion containing the specified point, if one exists */
		virtual PhysicsRegion* GetRegion(const Vec3& point) = 0;

		virtual void GetRegionsOnRay(const Vec3& from, const Vec3& to, RegionSet& results) = 0;
	};

	class CollisionCallback
	{
		public:

			/** A collision has occurred! Return whether or not to do the normal collision response behavior */
			virtual void OnCollision(const ContactPoint& collision) = 0;
	};

	class PhysicsStepCallback
	{
		public:
			
			/** The PhysicsWorld is calling DoFixedStep */
			virtual void OnPhysicsStep(PhysicsWorld* physics, float timestep) = 0;
	};
}
