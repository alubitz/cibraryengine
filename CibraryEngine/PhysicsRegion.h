#pragma once

#include "StdAfx.h"

#include "Disposable.h"
#include "CollisionShape.h"

namespace CibraryEngine
{
	using namespace std;

	using boost::unordered_map;
	using boost::unordered_set;

	class CollisionObject;
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

				vector<CollisionObject*> buckets[hash_size];
				unsigned int count;

				ObjectSet();

				void Insert(CollisionObject* obj);
				void Erase(CollisionObject* obj);
			};

			ObjectOrphanedCallback* orphan_callback;

			ObjectSet all_objects;

			ObjectSet active_objects;
			ObjectSet inactive_objects;
			ObjectSet static_objects;

			ObjectSet rays;

			virtual void InnerDispose();

		public:

			PhysicsRegion(ObjectOrphanedCallback* orphan_callback = NULL);



			// add a collision object to this region
			void AddCollisionObject(CollisionObject* obj);

			// remove a collision object from this region
			void RemoveCollisionObject(CollisionObject* obj);

			// add a collision object to this region, and add this region to its regions list
			void TakeOwnership(CollisionObject* obj);

			// remove a collision object from this region, and remove this region from its regions list
			void Disown(CollisionObject* obj);

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
			virtual void OnObjectOrphaned(CollisionObject* object) = 0;
	};
}
