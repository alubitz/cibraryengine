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
	struct ConstraintGraph;
	class CollisionCallback;
	class SceneRenderer;

	struct AABB;

	class ObjectOrphanedCallback;

	class PhysicsRegion : public Disposable
	{
		protected:

			ObjectOrphanedCallback* orphan_callback;

			unordered_set<RigidBody*> all_objects[ST_ShapeTypeMax];

			unordered_set<RigidBody*> active_objects[ST_ShapeTypeMax];
			unordered_set<RigidBody*> inactive_objects[ST_ShapeTypeMax];
			unordered_set<RigidBody*> static_objects[ST_ShapeTypeMax];

			virtual void InnerDispose();

		public:

			PhysicsRegion();
			PhysicsRegion(ObjectOrphanedCallback* orphan_callback);



			// add a rigid body to this region
			void AddRigidBody(RigidBody* body);

			// remove a rigid body from this region
			void RemoveRigidBody(RigidBody* body);

			// add a rigid body to this region, and add this region to its regions list
			void TakeOwnership(RigidBody* body);

			// remove a rigid body from this region, and remove this region from its regions list
			void Disown(RigidBody* body);


			/** Override this! Default implementation returns all objects owned by this region */
			virtual void GetRelevantObjects(const AABB& query, unordered_set<RigidBody*>* results);
	};

	class ObjectOrphanedCallback
	{
		public:

			/** An object is no longer contained by any PhysicsRegion! */
			virtual void OnObjectOrphaned(RigidBody* object) = 0;
	};
}
