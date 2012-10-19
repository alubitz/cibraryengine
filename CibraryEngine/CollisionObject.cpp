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
}
