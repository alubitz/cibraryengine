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
		for(unsigned int i = 0; i < ST_ShapeTypeMax; ++i)
		{
			all_objects[i] = unordered_set<RigidBody*>();
			active_objects[i] = unordered_set<RigidBody*>();
			inactive_objects[i] = unordered_set<RigidBody*>();
			static_objects[i] = unordered_set<RigidBody*>();
		}
	}

	void PhysicsRegion::InnerDispose()
	{
		for(unsigned int i = 0; i < ST_ShapeTypeMax; ++i)
		{
			for(unordered_set<RigidBody*>::iterator iter = all_objects[i].begin(); iter != all_objects[i].end(); ++iter)
			{
				RigidBody* body = *iter;
				
				body->regions.erase(this);
				if(body->regions.empty() && orphan_callback)
					orphan_callback->OnObjectOrphaned(body);
			}

			all_objects[i].clear();
			active_objects[i].clear();
			inactive_objects[i].clear();
			static_objects[i].clear();
		}
	}

	void PhysicsRegion::AddRigidBody(RigidBody* body)
	{
		ShapeType type = body->GetShapeType();

		all_objects[type].insert(body);

		if(!body->can_move)
			static_objects[type].insert(body);
		else if(body->active)
			active_objects[type].insert(body);
		else
			inactive_objects[type].insert(body);
	}

	void PhysicsRegion::RemoveRigidBody(RigidBody* body)
	{
		ShapeType type = body->GetShapeType();

		all_objects[type].erase(body);

		if(!body->can_move)
			static_objects[type].erase(body);
		else if(body->active)
			active_objects[type].erase(body);
		else
			inactive_objects[type].erase(body);		
	}

	void PhysicsRegion::TakeOwnership(RigidBody* body)
	{
		AddRigidBody(body);
		body->regions.insert(this);
	}

	void PhysicsRegion::Disown(RigidBody* body)
	{
		RemoveRigidBody(body);

		body->regions.erase(this);
		if(body->regions.empty() && orphan_callback)
			orphan_callback->OnObjectOrphaned(body);
	}



	void PhysicsRegion::GetRelevantObjects(const AABB& aabb, unordered_set<RigidBody*>* results)
	{
		for(unsigned int i = ST_Sphere; i < ST_ShapeTypeMax; ++i)						// skip past ST_Ray
		{
			for(unordered_set<RigidBody*>::iterator iter = all_objects[i].begin(); iter != all_objects[i].end(); ++iter)
			{
				RigidBody* object = *iter;
				
				if(i == ST_InfinitePlane || AABB::IntersectTest(aabb, object->GetCachedAABB()))
					results[i].insert(object);
			}
		}
	}

	unsigned int PhysicsRegion::NumObjects() const
	{
		unsigned int tot = 0;
		for(int i = ST_Ray; i < ST_ShapeTypeMax; ++i)
			tot += all_objects[i].size();
		return tot;
	}
	unsigned int PhysicsRegion::NumRays() const { return all_objects[ST_Ray].size(); }
	unsigned int PhysicsRegion::NumDynamicObjects() const { return inactive_objects[ST_Sphere].size() + active_objects[ST_Sphere].size() + inactive_objects[ST_MultiSphere].size() + active_objects[ST_MultiSphere].size(); }
	unsigned int PhysicsRegion::NumStaticObjects() const
	{
		unsigned int tot = 0;
		for(int i = ST_Sphere; i < ST_ShapeTypeMax; ++i)
			tot += static_objects[i].size();

		return tot;
	}
}
