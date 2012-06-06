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

	struct ContactPoint;
	class RigidBody;
	class CollisionCallback;
	struct CollisionGraph;

	class PhysicsRegion;
	class PhysicsRegionManager;

	class SceneRenderer;

	class Entity;

	/** Class for a physical simulation */
	class PhysicsWorld : public Disposable
	{
		private:

			unordered_set<RigidBody*> all_objects[ST_ShapeTypeMax];
			unordered_set<RigidBody*> dynamic_objects[ST_ShapeTypeMax];

			unordered_set<PhysicsRegion*> all_regions;

			PhysicsRegionManager* region_man;

			Vec3 gravity;

			float internal_timer, timer_interval;

			void GetUseMass(const Vec3& direction, const ContactPoint& cp, float& A, float& B);

			// returns whether or not a collision response was actually needed (i.e. whether an impulse was applied to prevent interpenetration)
			bool DoCollisionResponse(const ContactPoint& cp);

			void SolveCollisionGraph(CollisionGraph& graph);

			void InitiateCollisionsForSphere(RigidBody* body, float timestep, CollisionGraph& collision_graph);
			void InitiateCollisionsForMultiSphere(RigidBody* body, float timestep, CollisionGraph& collision_graph);

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
			/** Removes a rigid body from the simulation */
			void RemoveRigidBody(RigidBody* r);

			/** Steps the simulation */
			void Update(TimingInfo time);

			void DebugDrawWorld(SceneRenderer* renderer);

			Vec3 GetGravity();
			void SetGravity(const Vec3& gravity);

			void RayTest(const Vec3& from, const Vec3& to, CollisionCallback& callback);
	};

	class RigidBody;

	/** A point of contact between two physics objects */
	struct ContactPoint
	{
		struct Part
		{
			RigidBody* obj;
			Vec3 pos, norm;					// both are world coords

			Part() : obj(NULL), pos(), norm() { }
		} a, b;
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
