#pragma once

#include "StdAfx.h"

#include "Disposable.h"
#include "CollisionShape.h"

#include "SmartHashSet.h"

namespace CibraryEngine
{
	using namespace std;

	using boost::unordered_map;
	using boost::unordered_set;

	class CollisionObject;
	class CollisionCallback;
	class SceneRenderer;

	typedef SmartHashSet<CollisionObject, 17> RelevantObjectsQuery;

	struct AABB;

	class ObjectOrphanedCallback;

	class PhysicsRegion : public Disposable
	{
		public:
			typedef SmartHashSet<CollisionObject, 5> ObjectSet;
		protected:

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
