#include "StdAfx.h"
#include "CollisionGroup.h"

#include "AABB.h"

#include "Physics.h"
#include "PhysicsRegion.h"

#include "RigidBody.h"

namespace CibraryEngine
{
	// TODO: find a way to access protected members without having to cast a RigidBody* as a CollisionGroup*

	/*
	 * CollisionGroup::DummyRegionManager methods
	 */
	CollisionGroup::DummyRegionManager::DummyRegionManager() : PhysicsRegionManager(NULL) { }




	/*
	 * CollisionGroup methods
	 */
	CollisionGroup::CollisionGroup(Entity* entity) :
		CollisionObject(entity, COT_CollisionGroup),
		dummy_region_man(new DummyRegionManager()),
		children(),
		collide_within(false),
		gravity()
	{
	}

	void CollisionGroup::InnerDispose()
	{
		for(boost::unordered_set<RigidBody*>::iterator iter = children.begin(); iter != children.end(); ++iter)
		{
			RigidBody* child = *iter;

			child->Dispose();		// CAUTION: this will destroy the collision shapes! consider removing children first
			delete child;
		}
		children.clear();

		if(dummy_region_man)	{ delete dummy_region_man; dummy_region_man = NULL; }
	}

	void CollisionGroup::DebugDraw(SceneRenderer* renderer)
	{
		for(boost::unordered_set<RigidBody*>::iterator iter = children.begin(); iter != children.end(); ++iter)
			(*iter)->DebugDraw(renderer);
	}

	AABB CollisionGroup::GetAABB(float timestep)
	{
		if(children.empty())
			return AABB();
		else
		{
			boost::unordered_set<RigidBody*>::iterator iter = children.begin(), children_end = children.end();

			AABB result = (*iter)->GetCachedAABB();
			while(++iter != children_end)
				result.Expand((*iter)->GetCachedAABB());

			return result;
		}
	}

	void CollisionGroup::AddChild(RigidBody* child)		{ assert(((CollisionGroup*)child)->can_move); child->SetGravity(gravity); children.insert(child); }
	void CollisionGroup::RemoveChild(RigidBody* child)	{ children.erase(child); }

	void CollisionGroup::UpdateVel(float timestep)
	{
		for(boost::unordered_set<RigidBody*>::iterator iter = children.begin(); iter != children.end(); ++iter)
			(*iter)->UpdateVel(timestep);
	}

	void CollisionGroup::UpdatePos(float timestep, PhysicsRegionManager* region_man)
	{
		for(boost::unordered_set<RigidBody*>::iterator iter = children.begin(); iter != children.end(); ++iter)
			(*iter)->UpdatePos(timestep, dummy_region_man);		// NOTE: we are passing a different PhysicsRegionManager from the one we were passed

		region_man->OnObjectUpdate(this, regions, timestep);
	}

	void CollisionGroup::InitiateCollisions(float timestep, vector<ContactPoint>& contact_points)
	{
		// do collisions between contained objects, if enabled
		// TODO: don't do disabled collisions!
		if(collide_within)
		{
			for(boost::unordered_set<RigidBody*>::iterator iter = children.begin(), children_end = children.end(); iter != children_end; ++iter)
			{
				RigidBody* ibody = *iter;
				AABB iaabb = ibody->GetCachedAABB();

				for(boost::unordered_set<RigidBody*>::iterator jter = iter; jter != children_end; ++jter)
					if(jter != iter)
					{
						RigidBody* jbody = *jter;
						AABB jaabb = jbody->GetCachedAABB();

						if(AABB::IntersectTest(iaabb, jaabb) && ((CollisionGroup*)ibody)->disabled_collisions.find(jbody) == ((CollisionGroup*)ibody)->disabled_collisions.end())		// ughhhhh
							ibody->CollideRigidBody(jbody, contact_points);
					}
			}
		}

		// now find out what external objects we might be colliding with
		AABB my_aabb = GetAABB(timestep);

		RelevantObjectsQuery relevant_objects;
		for(unsigned int i = 0; i < RegionSet::hash_size; ++i)
		{
			vector<PhysicsRegion*>& bucket = regions.buckets[i];
			for(vector<PhysicsRegion*>::iterator iter = bucket.begin(), bucket_end = bucket.end(); iter != bucket_end; ++iter)
				(*iter)->GetRelevantObjects(my_aabb, relevant_objects);
		}
		RemoveDisabledCollisions(relevant_objects);

		for(unsigned int i = 0; i < RelevantObjectsQuery::hash_size; ++i)
		{
			vector<CollisionObject*>& bucket = relevant_objects.buckets[i];
			for(vector<CollisionObject*>::iterator iter = bucket.begin(), bucket_end = bucket.end(); iter != bucket_end; ++iter)
				switch((*iter)->GetType())
				{
					case COT_RigidBody:
					{
						RigidBody* body = (RigidBody*)*iter;
						if(*iter < this || !CollisionShape::CanShapeTypeMove(body->GetShapeType()))
							CollideRigidBody(body, contact_points);

						break;
					}

					case COT_CollisionGroup:
					{
						CollisionGroup* other = (CollisionGroup*)*iter;
						if(other < this)
							CollideCollisionGroup(other, contact_points);

						break;
					}
				}
		}
	}

	void CollisionGroup::CollideRigidBody(RigidBody* body, vector<ContactPoint>& contact_points)
	{
		AABB body_aabb = body->GetCachedAABB();
		bool aabb_exempt = body->GetShapeType() == ST_InfinitePlane;

		for(boost::unordered_set<RigidBody*>::iterator iter = children.begin(), children_end = children.end(); iter != children.end(); ++iter)
		{
			RigidBody* child = *iter;
			if(aabb_exempt || AABB::IntersectTest(body_aabb, child->GetCachedAABB()))
				child->CollideRigidBody(body, contact_points);
		}
	}

	void CollisionGroup::CollideCollisionGroup(CollisionGroup* other, vector<ContactPoint>& contact_points)
	{
		for(boost::unordered_set<RigidBody*>::iterator iter = children.begin(); iter != children.end(); ++iter)
		{
			RigidBody* ibody = *iter;
			AABB iaabb = ibody->GetCachedAABB();
			for(boost::unordered_set<RigidBody*>::iterator jter = other->children.begin(); jter != other->children.end(); ++jter)
			{
				RigidBody* jbody = *jter;
				if(AABB::IntersectTest(iaabb, jbody->GetCachedAABB()))
					ibody->CollideRigidBody(jbody, contact_points);
			}
		}
	}

	void CollisionGroup::SetGravity(const Vec3& gravity_)
	{
		gravity = gravity_;

		for(boost::unordered_set<RigidBody*>::iterator iter = children.begin(); iter != children.end(); ++iter)
			(*iter)->SetGravity(gravity_);
	}

	void CollisionGroup::ResetForces()
	{
		for(boost::unordered_set<RigidBody*>::iterator iter = children.begin(); iter != children.end(); ++iter)
			(*iter)->ResetForces();
	}
}
