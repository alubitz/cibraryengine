#pragma once

#include "StdAfx.h"
#include "Disposable.h"

#include "CollisionShape.h"

#include "Vector.h"
#include "Quaternion.h"
#include "TimingInfo.h"
#include "Events.h"

#include "MassInfo.h"

namespace CibraryEngine
{
	using namespace std;

	using boost::unordered_map;
	using boost::unordered_set;

	struct Mat4;

	class RigidBody;
	class PhysicsConstraint;
	class CollisionCallback;
	struct ConstraintGraph;

	class PhysicsRegion;
	class PhysicsRegionManager;

	class SceneRenderer;

	/** Class for a physical simulation */
	class PhysicsWorld : public Disposable
	{
		private:

			unordered_set<RigidBody*> all_objects[ST_ShapeTypeMax];
			unordered_set<RigidBody*> dynamic_objects[ST_ShapeTypeMax];

			unordered_set<PhysicsRegion*> all_regions;

			unordered_set<PhysicsConstraint*> all_constraints;

			PhysicsRegionManager* region_man;

			Vec3 gravity;

			float internal_timer, timer_interval;

			void SolveConstraintGraph(ConstraintGraph& graph);

			void InitiateCollisionsForSphere(RigidBody* body, float timestep, ConstraintGraph& contraint_graph);
			void InitiateCollisionsForMultisphere(RigidBody* body, float timestep, ConstraintGraph& constrain_graph);

			void DoFixedStep();

			void RayTestPrivate(const Vec3& from, const Vec3& to, CollisionCallback& callback, float max_time = 1.0f, RigidBody* ibody = NULL);

			struct MyOrphanCallback;
			MyOrphanCallback* orphan_callback;

			// no copying!
			void operator=(PhysicsWorld& other) { }
			PhysicsWorld(PhysicsWorld& other) { }

		protected:

			void InnerDispose();

		public:

			/** Initializes a PhysicsWorld */
			PhysicsWorld();

			/** Adds a rigid body to the simulation */
			void AddRigidBody(RigidBody* r);
			/**
			 * Removes a rigid body from the simulation.
			 * This will also remove any constraints with other objects, and it's up to you to know when to delete them!
			 */
			void RemoveRigidBody(RigidBody* r);

			void AddConstraint(PhysicsConstraint* c);
			void RemoveConstraint(PhysicsConstraint* c);

			/** Steps the simulation */
			void Update(TimingInfo time);

			void DebugDrawWorld(SceneRenderer* renderer);

			Vec3 GetGravity();
			void SetGravity(const Vec3& gravity);

			void RayTest(const Vec3& from, const Vec3& to, CollisionCallback& callback);

			/** Hard to explain... return value is like mass, and B is like inward velocity */
			static float GetUseMass(RigidBody* ibody, RigidBody* jbody, const Vec3& position, const Vec3& direction, float& B);

			static float GetUseMass(RigidBody* ibody, RigidBody* jbody, const Vec3& position, const Vec3& direction);
	};

	class RigidBody;

	/** Things for the constraint solver to solve */
	class PhysicsConstraint : public Disposable
	{
		public:

			PhysicsConstraint() : obj_a(NULL), obj_b(NULL) { }
			PhysicsConstraint(RigidBody* obj_a, RigidBody* obj_b) : obj_a(obj_a), obj_b(obj_b) { }

			virtual void DoConstraintAction(vector<RigidBody*>& wakeup) = 0;
			virtual void DoUpdateAction(float timestep) { }

			RigidBody* obj_a;
			RigidBody* obj_b;
	};

	/** A point of contact between two physics objects */
	struct ContactPoint : public PhysicsConstraint
	{
		struct Part
		{
			Vec3 pos, norm;					// both are world coords

			Part() : pos(), norm() { }
		} a, b;

		bool cache_valid;
		// cached values (must be computed if cache_valid is false)
		Vec3 use_pos;
		Vec3 normal;
		float bounciness, sfric_coeff, kfric_coeff;
		Vec3 moi_n;

		ContactPoint() : cache_valid(false) { }

		void BuildCache();

		void DoConstraintAction(vector<RigidBody*>& wakeup);
		void DoUpdateAction(float timestep);

		bool DoCollisionResponse() const;
	};

	class PhysicsRegionManager
	{
	protected:

		unordered_set<PhysicsRegion*>* all_regions;

	public:

		PhysicsRegionManager(unordered_set<PhysicsRegion*>* all_regions) : all_regions(all_regions) { }		
		virtual ~PhysicsRegionManager() { };

		// region should call TakeOwnership
		virtual void OnObjectAdded(RigidBody* object, set<PhysicsRegion*>& object_regions) = 0;

		virtual void OnObjectUpdate(RigidBody* object, set<PhysicsRegion*>& object_regions, float timestep) = 0;

		// region should NOT call Disown
		virtual void OnObjectRemoved(RigidBody* object, set<PhysicsRegion*>& object_regions) = 0;

		/** Get the PhysicsRegion containing the specified point, if one exists */
		virtual PhysicsRegion* GetRegion(const Vec3& point) = 0;

		virtual void GetRegionsOnRay(const Vec3& from, const Vec3& to, set<PhysicsRegion*>& results) = 0;
	};

	struct RayResult
	{
		float t;
		ContactPoint p;

		RayResult(float t, const ContactPoint& p) : t(t), p(p) { }
		bool operator <(const RayResult& h) { return t < h.t; }
	};

	class CollisionCallback
	{
		public:

			/** A collision has occurred! Return whether or not to do the normal collision response behavior */
			virtual bool OnCollision(const ContactPoint& collision) = 0;
	};
}
