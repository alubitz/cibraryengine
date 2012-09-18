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
	struct RelevantObjectsQuery;

	struct AABB;

	class ObjectOrphanedCallback;

	class PhysicsRegion : public Disposable
	{
		protected:

			struct ObjectSet
			{
				static const unsigned int hash_size = 5;

				vector<RigidBody*> buckets[hash_size];
				unsigned int count;

				ObjectSet();

				void Insert(RigidBody* obj);
				void Erase(RigidBody* obj);
			};

			ObjectOrphanedCallback* orphan_callback;

			ObjectSet all_objects[ST_ShapeTypeMax];

			ObjectSet active_objects[ST_ShapeTypeMax];
			ObjectSet inactive_objects[ST_ShapeTypeMax];
			ObjectSet static_objects[ST_ShapeTypeMax];

			virtual void InnerDispose();

		public:

			PhysicsRegion(ObjectOrphanedCallback* orphan_callback = NULL);



			// add a rigid body to this region
			void AddRigidBody(RigidBody* body);

			// remove a rigid body from this region
			void RemoveRigidBody(RigidBody* body);

			// add a rigid body to this region, and add this region to its regions list
			void TakeOwnership(RigidBody* body);

			// remove a rigid body from this region, and remove this region from its regions list
			void Disown(RigidBody* body);


			virtual void GetRelevantObjects(ShapeType type, const AABB& aabb, RelevantObjectsQuery& results);
			virtual void GetRelevantObjects(const AABB& aabb, RelevantObjectsQuery& results);

			unsigned int NumObjects() const;
			unsigned int NumRays() const;
			unsigned int NumDynamicObjects() const;			// NOTE: doesn't include rays!
			unsigned int NumStaticObjects() const;
	};

	class ObjectOrphanedCallback
	{
		public:

			/** An object is no longer contained by any PhysicsRegion! */
			virtual void OnObjectOrphaned(RigidBody* object) = 0;
	};
}
