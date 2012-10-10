#include "StdAfx.h"
#include "PhysicsRegion.h"

#include "Physics.h"
#include "RigidBody.h"
#include "ConstraintGraph.h"

#include "CollisionShape.h"
#include "RayShape.h"
#include "SphereShape.h"
#include "TriangleMeshShape.h"
#include "InfinitePlaneShape.h"
#include "MultiSphereShape.h"

#include "RenderNode.h"
#include "SceneRenderer.h"

#include "Util.h"

#include "DebugLog.h"

namespace CibraryEngine
{
	/*
	 * PhysicsRegion methods
	 */
	PhysicsRegion::PhysicsRegion(ObjectOrphanedCallback* orphan_callback) :
		Disposable(),
		orphan_callback(orphan_callback),
		all_objects(),
		active_objects(),
		inactive_objects(),
		static_objects()
	{
	}

	void PhysicsRegion::InnerDispose()
	{
		for(unsigned int j = 0; j < ObjectSet::hash_size; ++j)
		{
			vector<CollisionObject*>& bucket = all_objects.buckets[j];
			for(vector<CollisionObject*>::iterator iter = bucket.begin(), bucket_end = bucket.end(); iter != bucket_end; ++iter)
			{
				CollisionObject* cobj = *iter;
				
				cobj->regions.Erase(this);
				if(!cobj->regions.count && orphan_callback)
					orphan_callback->OnObjectOrphaned(cobj);
			}
			bucket.clear();
		}
	}

	void PhysicsRegion::AddCollisionObject(CollisionObject* obj)
	{
		all_objects.Insert(obj);

		switch(obj->GetType())
		{
			case COT_RigidBody:
			{
				RigidBody* body = (RigidBody*)obj;
				if(!body->can_move)
					static_objects.Insert(body);
				else if(body->active)
					active_objects.Insert(body);
				else
					inactive_objects.Insert(body);

				break;
			}

			case COT_RayCollider:
			{
				rays.Insert(obj);
				break;
			}
		}
	}

	void PhysicsRegion::RemoveCollisionObject(CollisionObject* obj)
	{
		all_objects.Erase(obj);

		switch(obj->GetType())
		{
			case COT_RigidBody:
			{
				RigidBody* body = (RigidBody*)obj;
				if(!body->can_move)
					static_objects.Erase(body);
				else if(body->active)
					active_objects.Erase(body);
				else
					inactive_objects.Erase(body);

				break;
			}

			case COT_RayCollider:
			{
				rays.Erase(obj);
				break;
			}
		}		
	}

	void PhysicsRegion::TakeOwnership(CollisionObject* obj)
	{
		AddCollisionObject(obj);
		obj->regions.Insert(this);
	}

	void PhysicsRegion::Disown(CollisionObject* obj)
	{
		RemoveCollisionObject(obj);

		obj->regions.Erase(this);
		if(!obj->regions.count && orphan_callback)
			orphan_callback->OnObjectOrphaned(obj);
	}
	
	void PhysicsRegion::GetRelevantObjects(const AABB& aabb, RelevantObjectsQuery& results)
	{
		for(unsigned int i = 0; i < ObjectSet::hash_size; ++i)
		{
			vector<CollisionObject*>& bucket = active_objects.buckets[i];
			for(vector<CollisionObject*>::iterator iter = bucket.begin(), bucket_end = bucket.end(); iter != bucket_end; ++iter)
			{
				RigidBody* object = (RigidBody*)*iter;
				if(AABB::IntersectTest(aabb, object->GetCachedAABB()))
					results.Insert(object);
			}
		}

		for(unsigned int i = 0; i < ObjectSet::hash_size; ++i)
		{
			vector<CollisionObject*>& bucket = inactive_objects.buckets[i];
			for(vector<CollisionObject*>::iterator iter = bucket.begin(), bucket_end = bucket.end(); iter != bucket_end; ++iter)
			{
				RigidBody* object = (RigidBody*)*iter;
				if(AABB::IntersectTest(aabb, object->GetCachedAABB()))
					results.Insert(object);
			}
		}

		for(unsigned int i = 0; i < ObjectSet::hash_size; ++i)
		{
			vector<CollisionObject*>& bucket = static_objects.buckets[i];
			for(vector<CollisionObject*>::iterator iter = bucket.begin(), bucket_end = bucket.end(); iter != bucket_end; ++iter)
			{
				RigidBody* object = (RigidBody*)*iter;
				if(object->GetShapeType() == ST_InfinitePlane || AABB::IntersectTest(aabb, object->GetCachedAABB()))			// infinte planes may only be static
					results.Insert(object);
			}
		}
	}

	unsigned int PhysicsRegion::NumObjects() const { return all_objects.count; }
	unsigned int PhysicsRegion::NumRays() const { return rays.count; }
	unsigned int PhysicsRegion::NumDynamicObjects() const { return inactive_objects.count + active_objects.count; }
	unsigned int PhysicsRegion::NumStaticObjects() const { return static_objects.count; }




	/*
	 * PhysicsRegion::ObjectSet methods
	 */
	PhysicsRegion::ObjectSet::ObjectSet() : buckets(), count(0) { }

	void PhysicsRegion::ObjectSet::Insert(CollisionObject* obj)
	{
		unsigned int hash = ((unsigned int)obj / sizeof(RigidBody)) % hash_size;

		vector<CollisionObject*>& bucket = buckets[hash];
		for(vector<CollisionObject*>::iterator iter = bucket.begin(), bucket_end = bucket.end(); iter != bucket_end; ++iter)
			if(*iter == obj)
				return;

		bucket.push_back(obj);
		++count;
	}

	void PhysicsRegion::ObjectSet::Erase(CollisionObject* obj)
	{
		if(count)
		{
			unsigned int hash = ((unsigned int)obj / sizeof(RigidBody)) % hash_size;

			vector<CollisionObject*>& bucket = buckets[hash];
			for(unsigned int i = 0, bucket_size = bucket.size(); i < bucket_size; ++i)
				if(bucket[i] == obj)
				{
					bucket[i] = bucket[bucket_size - 1];			// replace this element with the last one in the array
					bucket.pop_back();

					assert(count);
					--count;

					return;
				}
		}
	}
}
