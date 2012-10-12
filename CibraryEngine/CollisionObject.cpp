#include "StdAfx.h"
#include "CollisionObject.h"

#include "PhysicsRegion.h"

namespace CibraryEngine
{
	/*
	 * CollisionObject methods
	 */
	CollisionObject::CollisionObject(Entity* user_entity, CollisionObjectType type) : Disposable(), type(type), can_move(false), regions(), disabled_collisions(), user_entity(user_entity) { }

	void CollisionObject::SetCollisionEnabled(CollisionObject* other, bool enabled)
	{
		if(enabled)
		{
			disabled_collisions.erase(other);
			other->disabled_collisions.erase(this);
		}
		else
		{
			disabled_collisions.insert(other);
			other->disabled_collisions.insert(this);
		}
	}

	Entity* CollisionObject::GetUserEntity() { return user_entity; }
	void CollisionObject::SetUserEntity(Entity* entity) { user_entity = entity; }

	void CollisionObject::RemoveDisabledCollisions(RelevantObjectsQuery& eligible_bodies)
	{
		for(set<CollisionObject*>::const_iterator iter = disabled_collisions.begin(), disabled_end = disabled_collisions.end(); iter != disabled_end; ++iter)
			eligible_bodies.Erase(*iter);
	}



	
	/*
	 * RegionSet methods
	 */
	RegionSet::RegionSet() : buckets(), count(0) { }

	void RegionSet::Insert(PhysicsRegion* region)
	{
		unsigned int hash = ((unsigned int)region / sizeof(PhysicsRegion)) % hash_size;

		vector<PhysicsRegion*>& bucket = buckets[hash];
		for(vector<PhysicsRegion*>::iterator iter = bucket.begin(), bucket_end = bucket.end(); iter != bucket_end; ++iter)
			if(*iter == region)
				return;

		bucket.push_back(region);
		++count;
	}

	void RegionSet::Erase(PhysicsRegion* region)
	{
		if(count)
		{
			unsigned int hash = ((unsigned int)region / sizeof(PhysicsRegion)) % hash_size;

			vector<PhysicsRegion*>& bucket = buckets[hash];
			for(unsigned int i = 0, bucket_size = bucket.size(); i < bucket_size; ++i)
				if(bucket[i] == region)
				{
					bucket[i] = bucket[bucket_size - 1];			// replace this element with the last one in the array
					bucket.pop_back();

					assert(count);
					--count;

					return;
				}
		}
	}

	void RegionSet::Clear()
	{
		for(unsigned int i = 0; i < hash_size; ++i)
			buckets[i].clear();
	}




	/*
	 * RelevantObjectsQuery methods
	 */
	RelevantObjectsQuery::RelevantObjectsQuery() : buckets(), count(0) { }
	RelevantObjectsQuery::~RelevantObjectsQuery() { }

	void RelevantObjectsQuery::Insert(CollisionObject* object)
	{
		unsigned int hash = ((unsigned int)object / sizeof(CollisionObject)) % hash_size;

		vector<CollisionObject*>& bucket = buckets[hash];
		for(vector<CollisionObject*>::iterator iter = bucket.begin(), bucket_end = bucket.end(); iter != bucket_end; ++iter)
			if(*iter == object)
				return;

		bucket.push_back(object);
		++count;
	}
	
	void RelevantObjectsQuery::Erase(CollisionObject* object)
	{
		if(count)
		{
			unsigned int hash = ((unsigned int)object / sizeof(CollisionObject)) % hash_size;

			vector<CollisionObject*>& bucket = buckets[hash];
			for(unsigned int i = 0, bucket_size = bucket.size(); i < bucket_size; ++i)
				if(bucket[i] == object)
				{
					bucket[i] = bucket[bucket_size - 1];			// replace this element with the last one in the array
					bucket.pop_back();

					assert(count);
					--count;

					return;
				}
		}
	}

	void RelevantObjectsQuery::Clear()
	{
		for(unsigned int i = 0; i < hash_size; ++i)
			buckets[i].clear();

		count = 0;
	}
}
