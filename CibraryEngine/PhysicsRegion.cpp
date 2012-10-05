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
		for(unsigned int i = 0; i < ST_ShapeTypeMax; ++i)
		{
			ObjectSet& objects = all_objects[i];

			for(unsigned int j = 0; j < ObjectSet::hash_size; ++j)
			{
				vector<RigidBody*>& bucket = objects.buckets[j];
				for(vector<RigidBody*>::iterator iter = bucket.begin(), bucket_end = bucket.end(); iter != bucket_end; ++iter)
				{
					RigidBody* body = *iter;
				
					body->regions.Erase(this);
					if(!body->regions.count && orphan_callback)
						orphan_callback->OnObjectOrphaned(body);
				}
				bucket.clear();
			}
		}
	}

	void PhysicsRegion::AddRigidBody(RigidBody* body)
	{
		ShapeType type = body->GetShapeType();

		all_objects[type].Insert(body);

		if(!body->can_move)
			static_objects[type].Insert(body);
		else if(body->active)
			active_objects[type].Insert(body);
		else
			inactive_objects[type].Insert(body);
	}

	void PhysicsRegion::RemoveRigidBody(RigidBody* body)
	{
		ShapeType type = body->GetShapeType();

		all_objects[type].Erase(body);

		if(!body->can_move)
			static_objects[type].Erase(body);
		else if(body->active)
			active_objects[type].Erase(body);
		else
			inactive_objects[type].Erase(body);		
	}

	void PhysicsRegion::TakeOwnership(RigidBody* body)
	{
		AddRigidBody(body);
		body->regions.Insert(this);
	}

	void PhysicsRegion::Disown(RigidBody* body)
	{
		RemoveRigidBody(body);

		body->regions.Erase(this);
		if(!body->regions.count && orphan_callback)
			orphan_callback->OnObjectOrphaned(body);
	}



	void PhysicsRegion::GetRelevantObjects(ShapeType type, const AABB& aabb, RelevantObjectsQuery& results)
	{
		ObjectSet& objects = all_objects[type];
		if(type == ST_InfinitePlane)
		{
			for(unsigned int i = 0; i < ObjectSet::hash_size; ++i)
			{
				vector<RigidBody*>& bucket = objects.buckets[i];
				for(vector<RigidBody*>::iterator iter = bucket.begin(), bucket_end = bucket.end(); iter != bucket_end; ++iter)
					results.Insert(*iter);
			}
		}
		else
			for(unsigned int i = 0; i < ObjectSet::hash_size; ++i)
			{
				vector<RigidBody*>& bucket = objects.buckets[i];
				for(vector<RigidBody*>::iterator iter = bucket.begin(), bucket_end = bucket.end(); iter != bucket_end; ++iter)
				{
					RigidBody* object = *iter;
					if(AABB::IntersectTest(aabb, object->GetCachedAABB()))
						results.Insert(object);
				}
			}
	}
	
	void PhysicsRegion::GetRelevantObjects(const AABB& aabb, RelevantObjectsQuery& results)
	{
		for(unsigned int i = ST_Sphere; i < ST_ShapeTypeMax; ++i)						// skip past ST_Ray
			GetRelevantObjects((ShapeType)i, aabb, results);
	}

	unsigned int PhysicsRegion::NumObjects() const
	{
		unsigned int tot = 0;
		for(int i = ST_Ray; i < ST_ShapeTypeMax; ++i)
			tot += all_objects[i].count;
		return tot;
	}
	unsigned int PhysicsRegion::NumRays() const { return all_objects[ST_Ray].count; }
	unsigned int PhysicsRegion::NumDynamicObjects() const { return inactive_objects[ST_Sphere].count + active_objects[ST_Sphere].count + inactive_objects[ST_MultiSphere].count + active_objects[ST_MultiSphere].count; }
	unsigned int PhysicsRegion::NumStaticObjects() const
	{
		unsigned int tot = 0;
		for(int i = ST_Sphere; i < ST_ShapeTypeMax; ++i)
			tot += static_objects[i].count;

		return tot;
	}




	/*
	 * PhysicsRegion::ObjectSet methods
	 */
	PhysicsRegion::ObjectSet::ObjectSet() : buckets(), count(0) { }

	void PhysicsRegion::ObjectSet::Insert(RigidBody* obj)
	{
		unsigned int hash = ((unsigned int)obj / sizeof(RigidBody)) % hash_size;

		vector<RigidBody*>& bucket = buckets[hash];
		for(vector<RigidBody*>::iterator iter = bucket.begin(), bucket_end = bucket.end(); iter != bucket_end; ++iter)
			if(*iter == obj)
				return;

		bucket.push_back(obj);
		++count;
	}

	void PhysicsRegion::ObjectSet::Erase(RigidBody* obj)
	{
		if(count)
		{
			unsigned int hash = ((unsigned int)obj / sizeof(RigidBody)) % hash_size;

			vector<RigidBody*>& bucket = buckets[hash];
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
