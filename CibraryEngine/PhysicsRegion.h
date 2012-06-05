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

			unordered_set<RigidBody*> all_objects[ST_ShapeTypeMax];

			unordered_set<RigidBody*> active_objects[ST_ShapeTypeMax];
			unordered_set<RigidBody*> inactive_objects[ST_ShapeTypeMax];
			unordered_set<RigidBody*> static_objects[ST_ShapeTypeMax];

			virtual void InnerDispose();

		public:

			PhysicsRegion();



			// add a rigid body to this region
			void AddRigidBody(RigidBody* body);

			// remove a rigid body from this region
			void RemoveRigidBody(RigidBody* body);

			// add a rigid body to this region, and add this region to its regions list
			void TakeOwnership(RigidBody* body);

			// remove a rigid body from this region, and remove this region from its regions list
			void Disown(RigidBody* body);


			void DebugDrawRegion(SceneRenderer* renderer);


			void RayTest(const Vec3& from, const Vec3& to, CollisionCallback& callback, float max_time = 1.0f, RigidBody* ibody = NULL);

			/** Override this! Default implementation returns all objects owned by this region */
			virtual void GetRelevantObjects(const AABB& query, unordered_set<RigidBody*>* results);
	};
}
