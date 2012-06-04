#pragma once

#include "StdAfx.h"

#include "Disposable.h"
#include "CollisionShape.h"

namespace CibraryEngine
{
	using namespace std;

	using boost::unordered_map;
	using boost::unordered_set;

	class RigidBody;
	struct NearPairs;
	struct CollisionGraph;
	class CollisionCallback;
	class SceneRenderer;

	struct AABB;

	class PhysicsRegion : public Disposable
	{
		protected:

			/** All objects owned by this region */
			unordered_set<RigidBody*> rigid_bodies;

			/** Objects owned by this region, categorized by shape type */
			unordered_set<RigidBody*> shape_bodies[ST_ShapeTypeMax];

			/** Store all static objects which stick into this region (regardless of whether they are owned by this region) */
			unordered_set<RigidBody*> static_bodies[ST_ShapeTypeMax];

			/** Stores all objects which stick into this region from an outside region, as well as static bodies */
			unordered_set<RigidBody*> overlapping_bodies[ST_ShapeTypeMax];

			virtual void InnerDispose();


			/** Override this! Default implementation gives an empty results vector */
			virtual void GetRelevantNeighbors(const AABB& query, vector<PhysicsRegion*>& results);

			/** Override this! Default implementation returns all objects owned by this region */
			virtual void GetRelevantObjects(const AABB& query, vector<RigidBody*>& results);

		public:

			PhysicsRegion();



			// create a new rigid body within this region
			void AddRigidBody(RigidBody* body);

			// remove a rigid body from the simulation (true if the body was present)
			bool RemoveRigidBody(RigidBody* body);

			// take ownership of a rigid body from another region
			void TakeOwnership(RigidBody* body);

			// relinquish ownership of a rigid body (for when another region takes ownership)
			void Disown(RigidBody* body);


			void DebugDrawRegion(SceneRenderer* renderer);


			void RayTest(const Vec3& from, const Vec3& to, CollisionCallback& callback, float max_time = 1.0f, RigidBody* ibody = NULL);

			void UpdateVel(float timestep);
			void UpdatePos(float timestep);

			void ResetForces();
			void SetGravity(const Vec3& gravity);


			// conveniently doesn't need to know about any of that collision graph business
			void DoRayUpdates(float timestep, CollisionCallback& callback);

			void ResetOverlappingObjects();

			void AddNearPairs(float timestep, NearPairs& results);
	};
}
